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
/// @brief Goal validation and the goal-scoped configuration of a FollowCurve goal (plan §6
/// "Execution model"). Pure: no node; the caller passes the time and the configuration.
///
/// An active action server never rejects a goal: execute calls prepareGoal() first and, on a
/// rejection, terminates the goal before publishing anything. The checks, in order:
/// 1. the goal fields: an unknown `frame_motion`, a negative `max_duration` or a non-finite
///    `desired_speed` → INVALID_GOAL;
/// 2. the curve: `toCurveSpec` + `makeCurve` (the single validation point) → INVALID_CURVE, or
///    UNSUPPORTED for an open time-varying curve; `laps > 0` on an open curve → INVALID_CURVE;
/// 3. the frame sources: FRAME_MOVING without `frame_odom_topic` and without
///    `moving_frame_reference` → UNSUPPORTED (the Odometry child-frame check needs the first
///    message and is the node's);
/// 4. the goal-scoped configuration (goalScopedConfig()) through `validateConfig` without costmap
///    rules → INVALID_GOAL.
///
/// The node adds the checks that need the graph or its inputs: the interlock liveness
/// (UNSUPPORTED) and an active Nav2 action (NAV2_ACTIVE).

#ifndef VECTORFIELD_CURVE_FOLLOWER__GOAL_SETUP_HPP_
#define VECTORFIELD_CURVE_FOLLOWER__GOAL_SETUP_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"

namespace vectorfield_curve_follower
{

/// @brief The configuration of one goal: a copy of the node's configuration with the goal's
/// overrides (plan §6 "Goal-scoped configuration"): `desired_speed` when the goal's is > 0,
/// `direction` = −1 for `reverse_direction`, and ROTATE_TO_HEADING disabled (the follower has no
/// goal heading).
vectorfield_core::CoreConfig goalScopedConfig(
  const vectorfield_core::CoreConfig & base, double desired_speed, bool reverse_direction);

/// @brief What prepareGoal() needs besides the goal.
struct GoalSetupContext
{
  /// The node's configuration (tables + node-derived fields), before the goal's overrides.
  vectorfield_core::CoreConfig config;
  /// Curve-intake options (`kFollowerCoreParams`).
  vectorfield_core::CurveOptions curve_options;
  /// `global_frame`: the curve frame when the curve's `header.frame_id` is empty.
  std::string global_frame{"map"};
  /// `moving_frame_reference`.
  std::string moving_frame_reference;
  /// Goal acceptance time (s, node clock): \f$t_0\f$ when the curve's stamp is zero.
  double now{0.0};
};

/// @brief A validated goal.
struct PreparedGoal
{
  /// The curve's specification (`t0` resolved).
  vectorfield_core::CurveSpec spec;
  /// The curve (shared, immutable).
  std::shared_ptr<const vectorfield_core::Curve> curve;
  /// Curve frame \f$\mathcal F\f$ (resolved: never empty).
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
  /// The goal-scoped configuration (validated without costmap rules).
  vectorfield_core::CoreConfig config;
};

/// @brief Why a goal was refused (FollowCurve result code and message).
struct GoalRejection
{
  /// FollowCurve result code (result_code in goal_supervisor.hpp).
  std::uint16_t code{0};
  /// Human-readable reason (FollowCurve result `error_msg`).
  std::string message;
};

/// @brief Validates a goal and builds its curve and configuration (see the file comment).
/// @param goal The goal.
/// @param context Configuration, frames and time.
/// @param out Receives the prepared goal on success.
/// @return nullopt on success; the rejection otherwise (`out` then unspecified).
std::optional<GoalRejection> prepareGoal(
  const vectorfield_msgs::action::FollowCurve::Goal & goal, const GoalSetupContext & context,
  PreparedGoal * out);

/// @brief Whether a preempting goal keeps the TrackerState of the current one (plan §6): the
/// same CurveSpec (`t0` ignored for curves that do not vary in time), the same frame and frame
/// motion, and the same `desired_speed` and `reverse_direction` overrides.
bool keepsTrackerState(const PreparedGoal & current, const PreparedGoal & next);

}  // namespace vectorfield_curve_follower

#endif  // VECTORFIELD_CURVE_FOLLOWER__GOAL_SETUP_HPP_
