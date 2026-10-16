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
/// @brief `vectorfield_nav2_controller::VectorFieldController`, the `nav2_core::Controller`
/// plugin (plan §5; nav2_api_matrix.md §2.5).
///
/// Implemented in src/vectorfield_controller.cpp (M2 wave 2, plugin), which also holds
/// `PLUGINLIB_EXPORT_CLASS(vectorfield_nav2_controller::VectorFieldController,
/// nav2_core::Controller)`. The plugin type string, the parameter names and the topic names are
/// stable surfaces (plan §9).

#ifndef VECTORFIELD_NAV2_CONTROLLER__VECTORFIELD_CONTROLLER_HPP_
#define VECTORFIELD_NAV2_CONTROLLER__VECTORFIELD_CONTROLLER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_nav2_controller/plan_processing.hpp"
#include "vectorfield_ros/compat.hpp"

namespace vectorfield_nav2_controller
{

/// @brief Snapshot of the last control cycle, for tests and diagnostics (lastCycle()).
struct CycleDiagnostics
{
  /// The cycle's Tracker output. `predicted` views the plugin's scratch and stays valid until
  /// the next compute.
  vectorfield_core::TrackerOutput output;
  /// The state committed after the cycle.
  vectorfield_core::TrackerState state;
  /// Goal identity passed to the Tracker.
  std::uint32_t goal_seq{0};
  /// The command returned by the last compute (zero after a throw).
  geometry_msgs::msg::TwistStamped command;
  /// A cancel is in progress (cancel() was called and reset() was not).
  bool cancelling{false};
  /// The last parameter or footprint rebuild failed; every cycle throws NoValidControl until a
  /// rebuild succeeds.
  bool config_invalid{false};
  /// The last compute threw (the server published zero).
  bool threw{false};
  /// How the window was conditioned.
  WindowKind window_kind{WindowKind::kPolyline};
  /// Index of the virtual tail vertex, or −1.
  int virtual_vertex{-1};
  /// Vertices passed to `PolylineCurve::assign`.
  std::size_t window_vertices{0};
  /// Unit direction of the last segment passed to `PolylineCurve::assign` (costmap frame): the
  /// curve's end tangent, \f$R\,\hat T_{\mathrm{end}}\f$ once the tail is conditioned.
  vectorfield_core::Vec2 window_end_tangent{vectorfield_core::Vec2::UnitX()};
  /// The speed-limit scale in force.
  double speed_limit_scale{1.0};
  /// FieldVisualizer::builderCalls() (layer-2 test: 0 without subscribers).
  std::size_t visualizer_builder_calls{0};
};

/// @brief The vector-field path follower as a Nav2 controller plugin (plan §5).
///
/// **Distro blocks.** This class is the only place in the plugin with distro conditionals:
/// `#if VF_HAS_PATH_HANDLER` (Gen C: newPathReceived and the 5-argument compute) / `#else`
/// (Gen A/B: setPlan and the 3-argument compute, the window built by LegacyPathHandler), and
/// `VF_CANCEL_RESET_OVERRIDE` on cancel() and reset(), which are declared on every distro. Both
/// compute overloads delegate to computeCommandImpl(), and both plan entries to
/// processRawPlan(); those are distro-free.
///
/// **Threads.** configure, activate, deactivate and cleanup run in lifecycle transitions; setPlan
/// / newPathReceived, computeVelocityCommands, cancel and reset in the controller_server's
/// control thread (never concurrently with a transition); setSpeedLimit in the speed_limit
/// subscription's thread; parameter validation in the parameter-service thread. Shared state
/// (speed-limit scale, node-derived configuration for the cross-validator) is guarded by a
/// mutex.
///
/// **Per cycle** (computeCommandImpl(), plan §5.2): empty window → `InvalidPath`; costmap
/// snapshot (copied under the costmap mutex) → OccupancyGrid → EDT, rebuilt only on change;
/// pending parameter or footprint change → transactional rebuild; window conditioning
/// (buildWindow(), `PolylineCurve::assign` with `WindowScan` over \f$[-(d + 0.3), d + 0.5]\f$);
/// goal tolerances and `compat::goal_xy_reached()`; a colliding current footprint →
/// `NoValidControl`; `Tracker::step`; **commit** the state and the command; publish the
/// visualization; then map the status (noValidControlReason()). While cancelling, nothing
/// throws `NoValidControl`.
class VectorFieldController : public nav2_core::Controller
{
public:
  /// @brief An unconfigured plugin (pluginlib constructs it; nothing ROS-side is created).
  VectorFieldController();
  /// @brief Destroys the plugin (cleanup() is the server's job).
  ~VectorFieldController() override;

  VectorFieldController(const VectorFieldController &) = delete;
  VectorFieldController & operator=(const VectorFieldController &) = delete;

  /// @brief Declares the parameters, validates them against the costmap and the footprint, and
  /// builds the Tracker (plan §4, §5.4).
  ///
  /// Declares `vectorfield_common_params` under the prefix `name` (and, on Gen A/B,
  /// `vectorfield_legacy_path_params`, with the LegacyPathHandler); reads the server's
  /// `controller_frequency`; runs `vectorfield_ros::validateFromNode()` with the costmap geometry
  /// (window limit: `prune_distance` on Gen A/B) and `getRobotFootprint()`; builds the Tracker;
  /// registers the cross-validator after the listeners; creates the FieldVisualizer under
  /// `topics::pluginNamespace(name)`; warns once about the rotation shim with contour (plan
  /// §5.3); logs the API generation.
  /// @param parent The controller_server node.
  /// @param name The plugin name (the parameter prefix; may contain '.').
  /// @param tf The server's TF buffer (`costmap_ros->getTfBuffer()`).
  /// @param costmap_ros The local costmap (configured, so its size is known).
  /// @throws vectorfield_core::ConfigError on any validation error (messages list every rule).
  /// @throws rclcpp::exceptions::InvalidParameterValueException from a gpl validator.
  void configure(
    const vectorfield_ros::compat::NodeWeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /// @brief Releases the listeners, the cross-validator, the visualizer and the Tracker.
  void cleanup() override;

  /// @brief Activates the visualization publishers.
  void activate() override;

  /// @brief Clears the markers (DELETEALL) and deactivates the publishers.
  void deactivate() override;

#if VF_HAS_PATH_HANDLER  // Gen C (Lyrical, Rolling)
  /// @brief A new raw global plan (goal start and every replan): processRawPlan(). Cheap; keeps
  /// the tracking state.
  /// @throws vectorfield_ros::compat::InvalidPath for an empty plan or a cusp.
  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;

  /// @brief One control cycle on the server's window: computeCommandImpl(pose, velocity,
  /// goal_checker, transformed_global_plan, global_goal).
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;
#else  // Gen A/B (Humble to Kilted): LegacyPathHandler builds the same window
  /// @brief A new raw global plan (goal start and every replan): processRawPlan(), then the
  /// LegacyPathHandler's setPlan() and the re-arm of the emulated XY latch. On Humble (no
  /// reset() from the server) a full reset first when resetDue() (plan §5.1).
  /// @throws vectorfield_ros::compat::InvalidPath for an empty plan or a cusp.
  void setPlan(const nav_msgs::msg::Path & path) override;

  /// @brief One control cycle: the LegacyPathHandler's window and goal, then
  /// computeCommandImpl(pose, velocity, goal_checker, window, goal).
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
#endif

  /// @brief Scales \f$v_r\f$ (and \f$\omega_{\max}\f$) through `speed_limit_scale`
  /// (speedLimitScale(); `NO_SPEED_LIMIT` restores 1). Thread-safe.
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /// @brief Starts or continues a cancel (Jazzy and later: polled by the server every cycle).
  ///
  /// Sets the cancelling flag, which compute passes as `TrackerInput::cancel` (the core's STOPPED
  /// phase decelerates with `cancel_deceleration`). Returns true once the last returned command
  /// has \f$\lvert v_x\rvert, \lvert v_y\rvert\f$ < 0.01 m/s and \f$\lvert\omega\rvert\f$ <
  /// 0.01 rad/s. Declared on Humble too (never called there).
  bool cancel() VF_CANCEL_RESET_OVERRIDE;

  /// @brief Clears the cancelling flag, the TrackerState, the plan-intake state and the XY latch,
  /// and the markers (DELETEALL). Called by the server on every goal exit (Jazzy and later) and by
  /// the Humble reset emulation.
  void reset() VF_CANCEL_RESET_OVERRIDE;

  /// @brief The last cycle, for tests and diagnostics. Control thread only.
  CycleDiagnostics lastCycle() const;

  /// @brief The configuration the Tracker runs with (auto values resolved), or the defaults
  /// before configure(). Control thread only.
  vectorfield_core::CoreConfig appliedConfig() const;

private:
  /// @brief The distro-free cycle (see the class comment).
  /// @param pose Robot pose (costmap global frame), as passed by the server.
  /// @param velocity Measured twist, as passed by the server.
  /// @param goal_checker The server's goal checker (Gen C: forwarded to isGoalXYReached).
  /// @param window The window (costmap global frame): Gen C the server's
  /// `transformed_global_plan`, Gen A/B LegacyPathHandler's.
  /// @param goal The goal (costmap global frame): Gen C `global_goal`, Gen A/B LegacyPathHandler's.
  /// @throws vectorfield_ros::compat::InvalidPath, ControllerTFError, NoValidControl.
  geometry_msgs::msg::TwistStamped computeCommandImpl(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & window,
    const geometry_msgs::msg::PoseStamped & goal);

  /// @brief The distro-free raw-plan intake: intakePlan() on the plan's poses with
  /// \f$\ell_c\f$ = chordLength(costmap resolution); empty or cusp → InvalidPath; sparse and
  /// unsmoothed plans warn once per goal.
  void processRawPlan(const nav_msgs::msg::Path & raw_plan);

  /// Private state (node handles, listeners, cross-validator, Tracker, TrackerState, scratch,
  /// curve, grid and EDT, plan-intake state, XY latch, cancelling flag, speed limit and its
  /// mutex, visualizer), defined in src/vectorfield_controller.cpp.
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vectorfield_nav2_controller

#endif  // VECTORFIELD_NAV2_CONTROLLER__VECTORFIELD_CONTROLLER_HPP_
