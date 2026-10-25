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
/// @brief Goal validation of a FollowCurve3 goal (docs/design/follower_3d.md). Pure: no node; the
/// caller passes the time and the configuration.
///
/// As in the planar follower, an active server never rejects a goal: execute calls
/// prepareGoal3() first and terminates an invalid goal before publishing anything. The checks, in
/// order:
/// 1. the goal fields: an unknown `frame_motion`, a negative `max_duration` or a non-finite
///    `desired_speed` → INVALID_GOAL;
/// 2. the curve: toCurveSpec3() (type, pose) and `makeCurve3` (the single validation point) →
///    INVALID_CURVE, or UNSUPPORTED for an open time-varying curve; `laps > 0` on an open curve
///    → INVALID_CURVE;
/// 3. the frame sources: FRAME_MOVING without `frame_odom_topic` and without
///    `moving_frame_reference` → UNSUPPORTED;
/// 4. the goal-scoped configuration through `validateTracker3Config` → INVALID_GOAL.

#ifndef VECTORFIELD_FOLLOWER_3D__GOAL_SETUP_HPP_
#define VECTORFIELD_FOLLOWER_3D__GOAL_SETUP_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/d3/curve_spec.hpp"
#include "vectorfield_core/d3/tracker.hpp"
#include "vectorfield_curve_follower/goal_setup.hpp"
#include "vectorfield_msgs/action/follow_curve3.hpp"
#include "vectorfield_msgs/msg/curve3.hpp"

namespace vectorfield_follower_3d
{

/// @brief Why a goal was refused (result code and message), as in the planar follower.
using GoalRejection = vectorfield_curve_follower::GoalRejection;

/// @brief The CurveSpec3 of a Curve3 message.
///
/// A BUILTIN's non-identity `pose` becomes its `cx cy cz yaw pitch roll` parameters (ZYX Euler
/// angles of the normalised quaternion); a pose together with any of those parameters, a
/// quaternion whose norm is not 1 (to 1e-3), a non-identity pose on another type, an unknown
/// `type` or a non-finite `sample_spacing` is an error.
/// @param msg The curve.
/// @param t0 Time origin \f$t_0\f$ (s).
/// @param error Receives the reason on failure (may be null).
/// @return The spec, or nullopt (INVALID_CURVE).
std::optional<vectorfield_core::d3::CurveSpec3> toCurveSpec3(
  const vectorfield_msgs::msg::Curve3 & msg, double t0, std::string * error = nullptr);

/// @brief The Curve3 message of a spec (the node's `~/curve`; built-in poses stay parameters).
vectorfield_msgs::msg::Curve3 toCurve3Msg(const vectorfield_core::d3::CurveSpec3 & spec);

/// @brief The configuration of one goal: the node's with the goal's overrides
/// (`desired_speed` when > 0, `direction` = −1 for `reverse_direction`).
vectorfield_core::d3::Tracker3Config goalScopedConfig3(
  const vectorfield_core::d3::Tracker3Config & base, double desired_speed, bool reverse_direction);

/// @brief What prepareGoal3() needs besides the goal.
struct GoalSetupContext3
{
  /// The node's configuration, before the goal's overrides.
  vectorfield_core::d3::Tracker3Config config;
  /// Curve-intake options.
  vectorfield_core::CurveOptions curve_options;
  /// `global_frame`: the curve frame when the curve's `header.frame_id` is empty.
  std::string global_frame{"map"};
  /// `moving_frame_reference`.
  std::string moving_frame_reference;
  /// Goal acceptance time (s, node clock): \f$t_0\f$ when the curve's stamp is zero.
  double now{0.0};
};

/// @brief A validated goal.
struct PreparedGoal3
{
  /// The curve's specification (`t0` resolved, the pose folded into the parameters).
  vectorfield_core::d3::CurveSpec3 spec;
  /// The curve (shared, immutable).
  std::shared_ptr<const vectorfield_core::d3::Curve3> curve;
  /// Curve frame (resolved: never empty).
  std::string frame_id;
  /// FRAME_MOVING.
  bool moving{false};
  /// MOVING: the frame's Odometry topic; empty: TF differencing.
  std::string frame_odom_topic;
  /// The goal's `desired_speed` (<= 0: the parameter).
  double desired_speed{0.0};
  /// The goal's `reverse_direction`.
  bool reverse_direction{false};
  /// Requested laps (closed curves; 0 = until cancelled).
  std::uint32_t laps{0};
  /// `max_duration` (s); 0 = unlimited.
  double max_duration{0.0};
  /// Spacing of the drawn curve (m): the goal's `sample_spacing`, or the node default.
  double sample_spacing{0.05};
  /// The goal-scoped configuration (validated).
  vectorfield_core::d3::Tracker3Config config;
};

/// @brief Validates a goal and builds its curve and configuration (see the file comment).
/// @return nullopt on success; the rejection otherwise (`out` then unspecified).
std::optional<GoalRejection> prepareGoal3(
  const vectorfield_msgs::action::FollowCurve3::Goal & goal, const GoalSetupContext3 & context,
  PreparedGoal3 * out);

/// @brief Whether a preempting goal keeps the Tracker3State of the current one: the same spec
/// (`t0` ignored for curves that do not vary in time), frame, frame motion and overrides.
bool keepsTrackerState3(const PreparedGoal3 & current, const PreparedGoal3 & next);

}  // namespace vectorfield_follower_3d

#endif  // VECTORFIELD_FOLLOWER_3D__GOAL_SETUP_HPP_
