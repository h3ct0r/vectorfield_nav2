// Copyright 2026 Héctor Azpúrua
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
/// @brief In-process emulation of Nav2's controller_server for the plugin's layer-2 tests
/// (plan §9 "Test layers", 2).
///
/// Implemented in test/support/server_harness.cpp (M2 wave 2, harness). Together with that file,
/// this is the **only test code with distro conditionals** (plan §2): it reproduces, per API
/// generation, the order in which the server calls the plugin, the goal checker and (Gen C) the
/// path handler, and what the server does with each exception. Test files use this API only and
/// stay distro-free.
///
/// **What it loads** (as the server does): our plugin through pluginlib
/// (`nav2_core::Controller`, "vectorfield_nav2_controller::VectorFieldController"), optionally as
/// the primary of `nav2_rotation_shim_controller::RotationShimController`; the real goal checker
/// through pluginlib (`nav2_controller::SimpleGoalChecker`, `PositionGoalChecker`,
/// `StoppedGoalChecker`; `AxisGoalChecker` on Lyrical and later); on Gen C the real
/// `nav2_controller::FeasiblePathHandler` through pluginlib (`nav2_core::PathHandler`).
///
/// **World.** One server node ("controller_server", a compat::LifecycleNode) and one
/// `nav2_costmap_2d::Costmap2DROS` ("local_costmap"): configured, **not activated**, fixed origin,
/// not rolling, no layers, so its master grid only holds what the test writes (setLethalBox()).
/// **The only TF buffer is `costmap->getTfBuffer()`**: it is passed as `tf` to configure() and
/// initialize(), and the harness injects map→odom and odom→base_link with
/// `setTransform(t, "harness", true)` (static), rewriting odom→base_link every cycle (a separate
/// buffer would make Gen C's FeasiblePathHandler throw; nav2_api_matrix.md §3.3). The robot is
/// `vectorfield_core::sim::integrate` with the HarnessOptions plant, stepped once per cycle with
/// the published command. Time is ROS time driven by the harness (use_sim_time, starting at
/// 1000 s), advanced by 1/controller_frequency per cycle, so every run is deterministic.
///
/// **Per-distro server emulation** (nav2_api_matrix.md §3.6, §4.1–§4.3):
///
/// | Step | Gen A (Humble) | Gen B (Jazzy) | Gen C (Lyrical, Rolling) |
/// |---|---|---|---|
/// | lifecycle | goal checker initialised in configure | as Gen A | goal checker and path handler initialised in activate, after the controller's activate() |
/// | goal start / replan (`setPlannerPath`) | empty → abort; `setPlan(path)`; `end_pose_ = path.back()`; `checker->reset()` | empty → abort INVALID_PATH; `setPlan(path)`; `checker->reset()` | empty → abort INVALID_PATH; `newPathReceived(path)`; `handler->setPlan(path)`; `checker->reset()` |
/// | cancel requested | publish zero, CANCELED at once; the plugin is not told | poll `cancel()`; on true: CANCELED, zero, `reset()`; else the cycle continues | as Gen B |
/// | pending replan | `updateGlobalPath()` | as Gen A | as Gen A |
/// | window and goal | the plugin's (LegacyPathHandler) | as Gen A | `transformedPlanAndGoal()` at robot-pose stamp t1: `findPlanSegment` + `transformLocalPlan` → window; `transformed_end_pose_` = window end |
/// | goal check | **after** compute: `end_pose_` transformed into the costmap frame (failure ignored) | **before** compute: `end_pose_` transformed (failure → abort TF_ERROR) | **before** compute, against `transformed_end_pose_` and the window |
/// | compute | `computeVelocityCommands(pose, velocity, checker)` | as Gen A | second `getRobotPose()` (t3), `global_goal = handler->getTransformedGoal(t3)` (setGoalSkew() changes map→odom between t1 and t3), then the 5-argument compute |
/// | exception | any `PlannerException`: patience; other: abort | `NoValidControl`: patience; `InvalidPath`: abort INVALID_PATH; `ControllerTFError`: abort TF_ERROR; other: abort UNKNOWN | as Gen B |
/// | patience | zero command while `now − last_valid < failure_tolerance` (−1: forever), then abort "patience exceeded"; no `reset()` | as Gen A, abort PATIENCE_EXCEEDED | as Gen B |
/// | published command | as returned | as returned | dropped (error) when NaN/Inf |
/// | goal exit | zero; no `reset()` | zero and `reset()` (publishZeroVelocity) | `reset()` and zero (onGoalExit) |
///
/// Measured velocity: the plant's body twist, thresholded per axis at 1e-4 like the server.

#ifndef VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__SERVER_HARNESS_HPP_
#define VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__SERVER_HARNESS_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_nav2_controller/vectorfield_controller.hpp"
#include "vectorfield_ros/compat.hpp"

namespace vectorfield_nav2_controller
{
/// @brief Test support of the controller package (not installed).
namespace test
{

/// @brief Configuration of one harness.
struct HarnessOptions
{
  /// The plugin name: the parameter prefix and the topic namespace; may be dotted.
  std::string plugin_name{"FollowPath"};
  /// Drive `nav2_rotation_shim_controller::RotationShimController` named `plugin_name`, with our
  /// plugin as its primary (Humble/Jazzy: `<name>.primary_controller`; Lyrical and later:
  /// `<name>.primary_controller.plugin`, so our keys move under `<name>.primary_controller.`).
  bool rotation_shim{false};
  /// Parameters set on the server node before configure (full names, prefix included).
  std::vector<rclcpp::Parameter> parameters;

  /// Server `controller_frequency` (Hz); the cycle period of the harness.
  double controller_frequency{20.0};
  /// Server `failure_tolerance` (s); −1 = infinite patience.
  double failure_tolerance{0.0};

  /// Goal-checker plugin type, e.g. "nav2_controller::SimpleGoalChecker".
  std::string goal_checker{"nav2_controller::SimpleGoalChecker"};
  /// Goal-checker `xy_goal_tolerance` (m).
  double xy_goal_tolerance{0.25};
  /// Goal-checker `yaw_goal_tolerance` (rad).
  double yaw_goal_tolerance{0.25};
  /// Extra goal-checker parameters (names relative to the checker, e.g. "stateful").
  std::vector<rclcpp::Parameter> goal_checker_parameters;
  /// Extra FeasiblePathHandler parameters (Gen C; names relative to the handler).
  std::vector<rclcpp::Parameter> path_handler_parameters;

  /// Costmap side (m); square, centred on `costmap_center`. Rounded to whole metres: Nav2
  /// declares `width` and `height` as integers.
  double costmap_size{3.0};
  /// Costmap resolution (m).
  double costmap_resolution{0.05};
  /// Costmap centre (odom frame).
  vectorfield_core::Vec2 costmap_center{vectorfield_core::Vec2::Zero()};
  /// Robot-frame footprint polygon; empty uses `robot_radius`.
  std::vector<geometry_msgs::msg::Point> footprint;
  /// Costmap `robot_radius` (m) when `footprint` is empty.
  double robot_radius{0.22};

  /// Plan frame (raw plans).
  std::string map_frame{"map"};
  /// Costmap global frame (window, poses).
  std::string odom_frame{"odom"};
  /// Robot base frame.
  std::string base_frame{"base_link"};
  /// Initial map→odom transform (pose of odom in map).
  vectorfield_core::Pose2 map_to_odom;
  /// Initial robot pose in the odom frame.
  vectorfield_core::Pose2 initial_pose;
  /// The simulated plant (sim::integrate); its type should match `motion_model`.
  vectorfield_core::Kinematics plant;
};

/// @brief State of the current FollowPath goal.
enum class GoalStatus : std::uint8_t
{
  /// No goal yet, or the last one ended and no new one started.
  kNone = 0,
  /// Executing.
  kRunning = 1,
  /// The goal checker succeeded.
  kSucceeded = 2,
  /// A cancel completed.
  kCanceled = 3,
  /// The server aborted (see GoalResult::reason).
  kAborted = 4,
};

/// @brief How a goal ended.
struct GoalResult
{
  /// Final status.
  GoalStatus status{GoalStatus::kNone};
  /// For kAborted, the server's reason, distro-independent: "INVALID_PATH", "TF_ERROR",
  /// "PATIENCE_EXCEEDED", "NO_VALID_CONTROL", "UNKNOWN" (Humble reports its single
  /// PlannerException path as the matching name).
  std::string reason;
  /// The exception message that caused the abort, if any.
  std::string message;
};

/// @brief One emulated control cycle.
struct CycleRecord
{
  /// Harness time at the start of the cycle (s).
  double t{0.0};
  /// Robot pose (odom frame) at the start of the cycle.
  vectorfield_core::Pose2 pose_odom;
  /// Robot pose (map frame) at the start of the cycle.
  vectorfield_core::Pose2 pose_map;
  /// The plugin's compute was called this cycle.
  bool computed{false};
  /// What compute threw: "NoValidControl", "InvalidPath", "ControllerTFError", the type name of
  /// another exception, or "" for none.
  std::string exception;
  /// Its message.
  std::string message;
  /// The command the server published this cycle (zero during patience and on goal exit).
  geometry_msgs::msg::Twist published;
  /// The server's isGoalReached() result this cycle.
  bool goal_reached{false};
  /// cancel() was called this cycle (Gen B/C), and its result.
  bool cancel_called{false};
  /// Result of cancel() when called.
  bool cancel_result{false};
  /// Our plugin's lastCycle() after the cycle (default in rotation-shim mode).
  CycleDiagnostics diagnostics;
};

/// @brief The emulated controller_server (see the file comment for the per-distro behaviour).
///
/// Single-threaded: every method runs in the test's thread. Requires `rclcpp::init()`.
class ServerHarness
{
public:
  /// @brief Creates the nodes, the costmap (configured), the TF frames and the plant; nothing is
  /// loaded yet.
  explicit ServerHarness(HarnessOptions options);
  /// @brief Cleans up whatever is still configured.
  ~ServerHarness();

  ServerHarness(const ServerHarness &) = delete;
  ServerHarness & operator=(const ServerHarness &) = delete;

  /// @name Lifecycle (server order per distro)
  /// @{
  /// @brief Declares the server parameters, loads and configures the controller (and, Gen A/B,
  /// initialises the goal checker). Exceptions from configure() propagate.
  void configure();
  /// @brief Activates the controller (and, Gen C, then initialises the goal checker and the path
  /// handler).
  void activate();
  /// @brief Deactivates the controller.
  void deactivate();
  /// @brief Cleans up and unloads the controller.
  void cleanup();
  /// @}

  /// @name Goals
  /// @{
  /// @brief Starts a FollowPath goal with `plan` (plan frame = map_frame unless the header says
  /// otherwise): setPlannerPath in the distro's order. An exception aborts the goal.
  void followPath(const nav_msgs::msg::Path & plan);
  /// @brief Queues a preemption with a new plan, applied by the next cycle's updateGlobalPath().
  void replan(const nav_msgs::msg::Path & plan);
  /// @brief Requests a cancel, handled by the next cycle (Gen A at once; Gen B/C polls cancel()).
  void requestCancel();
  /// @brief The current goal's status.
  GoalStatus status() const noexcept;
  /// @brief How the last goal ended (status kRunning while it runs).
  const GoalResult & result() const noexcept;
  /// @}

  /// @name Cycles
  /// @{
  /// @brief One control cycle in the distro's order, then one plant step with the published
  /// command; time advances by one period. Does nothing but advance time when no goal runs.
  /// @return The cycle's record (also appended to history()).
  const CycleRecord & spinOnce();
  /// @brief Cycles until the goal ends or `timeout` (s) of harness time passes.
  /// @param timeout Maximum harness time (s).
  /// @param on_cycle Called after every cycle (e.g. to inject obstacles or replans).
  /// @return The final status (kRunning on timeout).
  GoalStatus runUntilDone(
    double timeout, const std::function<void(const CycleRecord &)> & on_cycle = nullptr);
  /// @brief Every cycle since construction.
  const std::vector<CycleRecord> & history() const noexcept;
  /// @brief Harness time (s).
  double now() const noexcept;
  /// @brief Processes pending callbacks of the harness's subscriptions (visualization tests).
  void spinSome();
  /// @}

  /// @name World
  /// @{
  /// @brief Steps map→odom (pose of odom in map); used by the next lookups.
  void setMapToOdom(const vectorfield_core::Pose2 & map_to_odom);
  /// @brief Gen C: a map→odom change applied after the window (t1) and before the goal (t3) of
  /// every following cycle, then undone; ignored on Gen A/B (one transform per cycle there).
  void setGoalSkew(const vectorfield_core::Pose2 & delta);
  /// @brief Moves the robot (odom frame) and zeroes its velocity.
  void setRobotPose(const vectorfield_core::Pose2 & pose_odom);
  /// @brief Marks the cells of an axis-aligned box (odom frame) LETHAL in the master costmap.
  void setLethalBox(
    const vectorfield_core::Vec2 & min_corner, const vectorfield_core::Vec2 & max_corner);
  /// @brief Marks the cells of a disc (odom frame) LETHAL.
  void setLethalDisc(const vectorfield_core::Vec2 & center, double radius);
  /// @brief Clears every cell to FREE_SPACE.
  void clearObstacles();
  /// @brief Changes the costmap footprint mid-run (`setRobotFootprintPolygon`, whose argument
  /// type differs per distro).
  void setFootprint(const std::vector<geometry_msgs::msg::Point> & polygon);
  /// @brief Calls `setSpeedLimit` on the controller as the server's subscription would.
  void setSpeedLimit(double speed_limit, bool percentage);
  /// @brief `set_parameters_atomically` on the server node (dynamic-parameter tests).
  rcl_interfaces::msg::SetParametersResult setParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  /// @}

  /// @name Access
  /// @{
  /// @brief Our plugin; null in rotation-shim mode (the shim owns it) or before configure().
  VectorFieldController * controller() noexcept;
  /// @brief The controller the server drives (the shim in rotation-shim mode).
  nav2_core::Controller * activeController() noexcept;
  /// @brief The server node.
  vectorfield_ros::compat::NodeSharedPtr node() const;
  /// @brief The local costmap.
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap() const;
  /// @brief The only TF buffer (`costmap()->getTfBuffer()`).
  std::shared_ptr<tf2_ros::Buffer> tf() const;
  /// @brief The goal checker (after it is initialised).
  nav2_core::GoalChecker * goalChecker() noexcept;
  /// @brief The options.
  const HarnessOptions & options() const noexcept;
  /// @}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace test
}  // namespace vectorfield_nav2_controller

#endif  // VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__SERVER_HARNESS_HPP_
