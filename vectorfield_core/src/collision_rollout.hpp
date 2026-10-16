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
/// @brief Internal (non-installed) rollout kernel shared by CollisionPredictor and the Tracker.
///
/// CollisionPredictor copies its plant and checker at construction, and copying a polygon
/// footprint allocates. The Tracker therefore calls this kernel with references to the
/// kinematics and checker it owns, which keeps Tracker::step allocation-free (oracle O55).

#ifndef VECTORFIELD_CORE__SRC__COLLISION_ROLLOUT_HPP_
#define VECTORFIELD_CORE__SRC__COLLISION_ROLLOUT_HPP_

#include <vector>

#include "vectorfield_core/collision_predictor.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{
namespace detail
{

/// @brief Extra simulated time a cancel rollout may spend in the STOPPED ramp beyond the
/// horizon before it gives up (s); a safety bound only, the ramp ends far earlier.
inline constexpr double kCancelRampMaxExtraTime = 10.0;

/// @brief Speed of base_link (m/s) under a constant body command once the plant has made it
/// feasible: \f$\lvert v\rvert\f$ for unicycles, \f$\lVert(v_x, v_y)\rVert\f$ for omni and
/// \f$\sqrt{v^2 + (x_r\omega)^2}\f$ for the bicycle (base_link's lateral velocity is
/// \f$-x_r\omega\f$, math_spec §7.5). Constant over a ZOH period, so the base arc length of a
/// period is this speed times its duration.
double baseSpeed(const Kinematics & plant, const Twist2 & command) noexcept;

/// @brief The rollout of math_spec §10.2 against `map` (see CollisionPredictor::rollout()).
///
/// Per control period: call the policy at the period-start pose and time; stop on STOPPED before
/// integrating it (unless `options.through_cancel`, then once the command is below
/// kStopLinearSpeed and kStopAngularSpeed); otherwise integrate the command exactly with
/// sim::integrate (no slip) at sub-steps no longer than one cell of base travel and
/// \f$2\arcsin(\mathrm{res}/(2r_{\mathrm{circ}}))\f$ of yaw, checking each sub-step pose, and
/// stop after a settled step. The start pose is checked after the first policy call, whatever
/// its phase. `collided_in_rotation` is set only when the first
/// collision happens before the rollout has left the rotation phase it started in (the kBlocked
/// rotation rule, math_spec §10.4).
RolloutResult runRollout(
  const Kinematics & plant, const FootprintChecker & checker, const Pose2 & start,
  const ObstacleMap & map, const RolloutOptions & options, RolloutPolicy policy,
  std::vector<Pose2> * poses) noexcept;

}  // namespace detail
}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__SRC__COLLISION_ROLLOUT_HPP_
