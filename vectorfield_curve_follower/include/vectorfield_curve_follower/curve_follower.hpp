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
/// @brief `vectorfield_curve_follower::CurveFollower`: the lifecycle component with the
/// `follow_curve` action server.
///
/// Implemented in src/curve_follower.cpp, which also registers the component
/// (`RCLCPP_COMPONENTS_REGISTER_NODE`). The node name defaults to `curve_follower`. The action,
/// topic and parameter names are stable surfaces.
///
/// **Interfaces** (all relative names, so the node runs in a namespace):
/// - action server `follow_curve` (`vectorfield_msgs/action/FollowCurve`);
/// - publishes `cmd_vel_nav` (TwistPublisher: Twist, or TwistStamped with
///   `enable_stamped_cmd_vel` on Jazzy and later), only while a goal executes; `~/curve`
///   (transient local); the visualization topics under `~/` (FieldVisualizer); `/diagnostics`;
/// - subscribes to `odom` (`odom_topic`), `speed_limit`, `local_costmap/costmap_raw`,
///   `local_costmap/published_footprint`, `<action>/_action/status` of every
///   `exclusive_actions` entry and, for FRAME_MOVING goals with `frame_odom_topic`, that topic;
/// - TF: `global_frame` ← `robot_base_frame`, `global_frame` ← curve frame, costmap frame ←
///   `global_frame`, `robot_base_frame` ← footprint frame, and `moving_frame_reference` ← curve
///   frame (TF differencing).
///
/// **Execution model.** An active follower never rejects a goal (only an inactive one does;
/// clients report that as GOAL_REJECTED). vectorfield_ros::SingleGoalActionServer runs the goals
/// one at a time: the execute callback
/// validates first (prepareGoal(), the interlock liveness and an active Nav2 action) and
/// terminates an invalid goal before publishing anything; then it runs one control cycle every
/// 1/`controller_frequency` on the node's clock (simulated time included), until success, cancel,
/// abort or preemption (`accept_pending_goal()`, the same validation; the TrackerState is kept
/// only for an identical spec and identical overrides). Subscriptions only swap immutable snapshots under a mutex; all
/// conversions, TF lookups, parameter updates and Tracker rebuilds run in the control thread.
///
/// **Testability.** Every decision is taken by the pure helpers (prepareGoal(), GoalSupervisor,
/// LapCounter, FrameTwistEstimator); the loop waits on the node's clock (with `use_sim_time` it
/// polls `/clock`, so tests drive it by publishing time) and wakes at once on deactivation.

#ifndef VECTORFIELD_CURVE_FOLLOWER__CURVE_FOLLOWER_HPP_
#define VECTORFIELD_CURVE_FOLLOWER__CURVE_FOLLOWER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_ros/compat.hpp"

namespace vectorfield_curve_follower
{

/// @brief Summary of the last control cycle, for tests and diagnostics (lastCycle()).
struct CycleSnapshot
{
  /// Cycles run since activation.
  std::uint64_t cycles{0};
  /// A goal is executing.
  bool executing{false};
  /// The Tracker ran in the last cycle.
  bool tracker_ran{false};
  /// The supervisor's verdict of the last cycle.
  SupervisorDecision decision;
  /// Tracker status bits of the last cycle that ran the Tracker.
  std::uint32_t status{vectorfield_core::Status::kNone};
  /// Phase of the last cycle that ran the Tracker.
  vectorfield_core::Phase phase{vectorfield_core::Phase::kFollow};
  /// The last command published (body twist at base_link).
  vectorfield_core::Twist2 command;
  /// Commands published during the current (or last) goal.
  std::size_t commands_published{0};
  /// Laps completed in the current (or last) goal.
  std::uint32_t laps_completed{0};
  /// Progress in laps of the current (or last) goal.
  double progress{0.0};
  /// The last rebuild failed (the goal aborts UNKNOWN).
  bool config_invalid{false};
  /// Result code of the last finished goal.
  std::uint16_t last_result{result_code::kNone};
  /// Reason of the last finished goal ("" on success).
  std::string last_reason;
  /// Serial number of the goal of the last cycle or refusal: goals taken since construction,
  /// preempting ones included (1 for the first). Tests synchronize goal transitions on it.
  std::uint64_t goal{0};
  /// Commands published since construction, all goals (counted when published).
  std::uint64_t commands_sent{0};
};

/// @brief The curve follower (see the file comment).
class CurveFollower : public vectorfield_ros::compat::LifecycleNode
{
public:
  /// @brief The action type.
  using FollowCurve = vectorfield_msgs::action::FollowCurve;
  /// @brief Lifecycle callback return type.
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Creates the node (`curve_follower`) and its diagnostics; parameters are declared at
  /// configure.
  explicit CurveFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /// @brief Stops a running goal and releases everything.
  ~CurveFollower() override;

  CurveFollower(const CurveFollower &) = delete;
  CurveFollower & operator=(const CurveFollower &) = delete;
  CurveFollower(CurveFollower &&) = delete;
  CurveFollower & operator=(CurveFollower &&) = delete;

  /// @brief The last cycle's summary (thread-safe).
  CycleSnapshot lastCycle() const;
  /// @brief A goal waits to preempt the running one; the next cycle takes it. For tests, which
  /// call it while the node is active: the action server accepts a goal after answering the
  /// client, so the answer alone does not say whether the next cycle runs the new goal.
  bool preemptPending() const;

protected:
  /// @brief Declares the parameters (both listeners), validates them, creates the TF buffer, the
  /// publishers, the subscriptions, the cross-validator and the action server.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /// @brief Activates the publishers and the action server, logs the interlock's status topics,
  /// creates the bond.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /// @brief Stops a running goal (UNKNOWN, zero command), deactivates the action server and the
  /// publishers, destroys the bond.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /// @brief Releases everything on_configure() created.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /// @brief Stops a running goal and releases everything.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// The action server's execute callback.
  void execute();

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vectorfield_curve_follower

#endif  // VECTORFIELD_CURVE_FOLLOWER__CURVE_FOLLOWER_HPP_
