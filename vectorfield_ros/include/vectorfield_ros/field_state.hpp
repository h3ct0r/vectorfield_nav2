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
/// @brief `vectorfield_msgs/FieldState` from one Tracker cycle, and the compile-time proof that
/// the message constants equal the core's Phase and Status values.
///
/// The static_asserts below are the contract: FieldState.msg is frozen, and the core
/// promises never to renumber Phase or Status (status.hpp). toFieldState() is implemented in
/// src/field_state.cpp.

#ifndef VECTORFIELD_ROS__FIELD_STATE_HPP_
#define VECTORFIELD_ROS__FIELD_STATE_HPP_

#include <cstdint>

#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_msgs/msg/field_state.hpp"

namespace vectorfield_ros
{

/// @brief Fills every field of a FieldState except `header` (the caller sets the control frame
/// and the cycle stamp).
///
/// Mapping (math_spec Appendix A):
/// phase, status ← output.phase, output.status; control_point ← field.x_eval;
/// closest_point ← field.x_star; tangent ← field.T_d; distance ← field.D;
/// arc_length ← field.s_star; distance_to_end ← field.sigma (NaN for closed curves);
/// g, h, eta ← field.G, field.H, field.eta; feedforward ← field.Phi_T; field ← field.F;
/// obstacle_distance ← field.contour.D_o (+Inf without an obstacle); blend ← field.contour.theta;
/// contour_direction ← field.contour.sigma_c; cmd_raw ← output.cmd_target; cmd ← output.cmd;
/// limiter_scale ← output.limiter_alpha; free_arc_length ← output.s_free (+Inf when
/// collision-free); compute_time_ms ← the argument. Points and vectors have z = 0.
///
/// Allocation-free (FieldState has no dynamic fields besides the header's frame_id, which is not
/// touched).
/// @param output The cycle's output.
/// @param compute_time_ms Wall time of the cycle's compute (ms).
/// @param msg Output.
void toFieldState(
  const vectorfield_core::TrackerOutput & output, double compute_time_ms,
  vectorfield_msgs::msg::FieldState * msg) noexcept;

/// @cond VF_INTERNAL
namespace detail
{
using FieldStateMsg = vectorfield_msgs::msg::FieldState;
using vectorfield_core::Phase;
using vectorfield_core::Status;
}  // namespace detail
/// @endcond

static_assert(
  detail::FieldStateMsg::PHASE_ROTATE_TO_FIELD ==
    vectorfield_core::toUnderlying(detail::Phase::kRotateToField),
  "FieldState.PHASE_ROTATE_TO_FIELD != Phase::kRotateToField");
static_assert(
  detail::FieldStateMsg::PHASE_FOLLOW == vectorfield_core::toUnderlying(detail::Phase::kFollow),
  "FieldState.PHASE_FOLLOW != Phase::kFollow");
static_assert(
  detail::FieldStateMsg::PHASE_FINAL_APPROACH ==
    vectorfield_core::toUnderlying(detail::Phase::kFinalApproach),
  "FieldState.PHASE_FINAL_APPROACH != Phase::kFinalApproach");
static_assert(
  detail::FieldStateMsg::PHASE_ROTATE_TO_HEADING ==
    vectorfield_core::toUnderlying(detail::Phase::kRotateToHeading),
  "FieldState.PHASE_ROTATE_TO_HEADING != Phase::kRotateToHeading");
static_assert(
  detail::FieldStateMsg::PHASE_STOPPED == vectorfield_core::toUnderlying(detail::Phase::kStopped),
  "FieldState.PHASE_STOPPED != Phase::kStopped");

static_assert(
  detail::FieldStateMsg::STATUS_NON_FINITE == detail::Status::kNonFinite,
  "FieldState.STATUS_NON_FINITE != Status::kNonFinite");
static_assert(
  detail::FieldStateMsg::STATUS_ASSUMPTION3_CLAMPED == detail::Status::kAssumption3Clamped,
  "FieldState.STATUS_ASSUMPTION3_CLAMPED != Status::kAssumption3Clamped");
static_assert(
  detail::FieldStateMsg::STATUS_AMBIGUOUS_CLOSEST == detail::Status::kAmbiguousClosest,
  "FieldState.STATUS_AMBIGUOUS_CLOSEST != Status::kAmbiguousClosest");
static_assert(
  detail::FieldStateMsg::STATUS_HAIRPIN == detail::Status::kHairpin,
  "FieldState.STATUS_HAIRPIN != Status::kHairpin");
static_assert(
  detail::FieldStateMsg::STATUS_CURVATURE_SATURATED == detail::Status::kCurvatureSaturated,
  "FieldState.STATUS_CURVATURE_SATURATED != Status::kCurvatureSaturated");
static_assert(
  detail::FieldStateMsg::STATUS_CONTOUR_WATCHDOG == detail::Status::kContourWatchdog,
  "FieldState.STATUS_CONTOUR_WATCHDOG != Status::kContourWatchdog");
static_assert(
  detail::FieldStateMsg::STATUS_HEADING_INFEASIBLE == detail::Status::kHeadingInfeasible,
  "FieldState.STATUS_HEADING_INFEASIBLE != Status::kHeadingInfeasible");
static_assert(
  detail::FieldStateMsg::STATUS_COLLISION_AHEAD == detail::Status::kCollisionAhead,
  "FieldState.STATUS_COLLISION_AHEAD != Status::kCollisionAhead");
static_assert(
  detail::FieldStateMsg::STATUS_BLOCKED == detail::Status::kBlocked,
  "FieldState.STATUS_BLOCKED != Status::kBlocked");
static_assert(
  (detail::FieldStateMsg::STATUS_NON_FINITE | detail::FieldStateMsg::STATUS_ASSUMPTION3_CLAMPED |
   detail::FieldStateMsg::STATUS_AMBIGUOUS_CLOSEST | detail::FieldStateMsg::STATUS_HAIRPIN |
   detail::FieldStateMsg::STATUS_CURVATURE_SATURATED |
   detail::FieldStateMsg::STATUS_CONTOUR_WATCHDOG |
   detail::FieldStateMsg::STATUS_HEADING_INFEASIBLE |
   detail::FieldStateMsg::STATUS_COLLISION_AHEAD | detail::FieldStateMsg::STATUS_BLOCKED) ==
    detail::Status::kAll,
  "FieldState.STATUS_* must cover exactly Status::kAll");

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__FIELD_STATE_HPP_
