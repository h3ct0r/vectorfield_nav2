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
/// @brief `vectorfield_follower_3d::Follower3D`: the experimental 3D curve follower, a lifecycle
/// component with the `follow_curve_3d` action server (docs/design/follower_3d.md).
///
/// Implemented in src/follower_3d.cpp, which also registers the component. The node name
/// defaults to `follower_3d`.
///
/// **Interfaces** (relative names, so the node runs in a namespace):
/// - action server `follow_curve_3d` (`vectorfield_msgs/action/FollowCurve3`);
/// - publishes `cmd_vel` (`cmd_vel_topic`, `geometry_msgs/TwistStamped` in `global_frame` or,
///   with `cmd_frame: body`, in the heading frame `robot_base_frame`), only while a goal
///   executes; `~/curve` (Curve3, transient local), `~/field_state` (FieldState3, every cycle)
///   and `~/markers` (MarkerArray);
/// - subscribes to `odom` (`odom_topic`), the optional `voxel_cloud_topic` and, for FRAME_MOVING
///   goals with `frame_odom_topic`, that topic;
/// - TF: `global_frame` ← odometry frame (when they differ), `global_frame` ← curve frame,
///   `global_frame` ← cloud frame and `moving_frame_reference` ← curve frame.
///
/// **Execution model and safety contract** (as the planar CurveFollower): an active follower
/// never rejects a goal; execute validates it first (prepareGoal3()) and terminates an invalid
/// goal before publishing anything. One control cycle runs every 1/`controller_frequency` on the
/// node's clock until success, cancel, abort or preemption. The GoalSupervisor decides each
/// cycle; a goal ends with a deceleration ramp (or the tracker's cancel deceleration), exactly
/// one zero, then silence.

#ifndef VECTORFIELD_FOLLOWER_3D__FOLLOWER_3D_HPP_
#define VECTORFIELD_FOLLOWER_3D__FOLLOWER_3D_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"
#include "vectorfield_msgs/action/follow_curve3.hpp"
#include "vectorfield_ros/compat.hpp"

namespace vectorfield_follower_3d
{

/// @brief Summary of the last control cycle, for tests and logs (lastCycle()).
struct CycleSnapshot3
{
  /// Cycles run since activation.
  std::uint64_t cycles{0};
  /// A goal is executing.
  bool executing{false};
  /// The Tracker3 ran in the last cycle.
  bool tracker_ran{false};
  /// The supervisor's verdict of the last cycle.
  vectorfield_curve_follower::SupervisorDecision decision;
  /// Tracker status bits of the last cycle that ran the tracker.
  std::uint32_t status{vectorfield_core::Status::kNone};
  /// Phase of the last cycle that ran the tracker.
  vectorfield_core::Phase phase{vectorfield_core::Phase::kFollow};
  /// Distance to the curve of the last cycle that ran the tracker (m).
  double distance{0.0};
  /// The last command published (world frame).
  vectorfield_core::d3::Twist3 command;
  /// Commands published during the current (or last) goal.
  std::size_t commands_published{0};
  /// Laps completed in the current (or last) goal.
  std::uint32_t laps_completed{0};
  /// The last rebuild failed (the goal aborts UNKNOWN).
  bool config_invalid{false};
  /// Result code of the last finished goal.
  std::uint16_t last_result{vectorfield_curve_follower::result_code::kNone};
  /// Reason of the last finished goal ("" on success).
  std::string last_reason;
  /// Serial number of the goal of the last cycle or refusal (1 for the first).
  std::uint64_t goal{0};
  /// Commands published since construction, all goals (counted when published).
  std::uint64_t commands_sent{0};
};

/// @brief The 3D follower (see the file comment).
class Follower3D : public vectorfield_ros::compat::LifecycleNode
{
public:
  /// @brief The action type.
  using FollowCurve3 = vectorfield_msgs::action::FollowCurve3;
  /// @brief Lifecycle callback return type.
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Creates the node (`follower_3d`); parameters are declared at configure.
  explicit Follower3D(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /// @brief Stops a running goal and releases everything.
  ~Follower3D() override;

  Follower3D(const Follower3D &) = delete;
  Follower3D & operator=(const Follower3D &) = delete;
  Follower3D(Follower3D &&) = delete;
  Follower3D & operator=(Follower3D &&) = delete;

  /// @brief The last cycle's summary (thread-safe).
  CycleSnapshot3 lastCycle() const;

  /// @brief A goal waits to preempt the running one; the next cycle takes it (for tests).
  bool preemptPending() const;

protected:
  /// @brief Declares and validates the parameters; creates the TF buffer, the publishers, the
  /// subscriptions and the action server. An invalid parameter set fails the transition.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /// @brief Activates the publishers and the action server, creates the bond.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /// @brief Stops a running goal (UNKNOWN, one zero), deactivates the server and the publishers.
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

}  // namespace vectorfield_follower_3d

#endif  // VECTORFIELD_FOLLOWER_3D__FOLLOWER_3D_HPP_
