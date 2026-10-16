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

#include "vectorfield_core/collision_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "collision_rollout.hpp"

namespace vectorfield_core
{

namespace detail
{

namespace
{

/// Upper bound on the sub-steps of one period: a guard against absurd commands only (at the
/// limits a period needs one or two sub-steps).
constexpr double kMaxSubSteps = 4096.0;

/// Slack on the horizon comparison, so that T_h / period periods are simulated despite rounding.
constexpr double kHorizonSlack = 1e-9;

/// @brief Result of checking one pose against the grid.
enum class PoseCheck : std::uint8_t
{
  kFree = 0,
  kCollides = 1,
  kOffGrid = 2,
};

/// @brief Off-grid test first (the rollout ends there), then the footprint test.
PoseCheck checkPose(
  const FootprintChecker & checker, const ObstacleMap & map, const Pose2 & pose_eval) noexcept
{
  const Pose2 pose_grid = map.T_grid_eval.apply(pose_eval);
  if (checker.offGrid(pose_grid, *map.grid)) {
    return PoseCheck::kOffGrid;
  }
  return checker.collides(pose_grid, *map.grid, *map.edt) ? PoseCheck::kCollides : PoseCheck::kFree;
}

/// @brief The cancel ramp is over: the command is below the stop thresholds (math_spec §9.2).
bool belowStopThresholds(const Twist2 & cmd) noexcept
{
  return std::abs(cmd.vx) < kStopLinearSpeed && std::abs(cmd.vy) < kStopLinearSpeed &&
         std::abs(cmd.wz) < kStopAngularSpeed;
}

}  // namespace

double baseSpeed(const Kinematics & plant, const Twist2 & command) noexcept
{
  const Twist2 u = sim::feasibleTwist(plant, command);
  switch (plant.type) {
    case MotionModel::kOmnidirectional:
      return std::hypot(u.vx, u.vy);
    case MotionModel::kAckermann:
      return std::hypot(u.vx, plant.reference_point_offset * u.wz);
    case MotionModel::kDifferential:
    case MotionModel::kSkidSteer:
      break;
  }
  return std::abs(u.vx);
}

RolloutResult runRollout(
  const Kinematics & plant, const FootprintChecker & checker, const Pose2 & start,
  const ObstacleMap & map, const RolloutOptions & options, RolloutPolicy policy,
  std::vector<Pose2> * poses) noexcept
{
  RolloutResult result;
  poses->clear();
  if (
    map.grid == nullptr || map.edt == nullptr || !policy || map.grid->width() <= 0 ||
    map.grid->height() <= 0) {
    return result;  // nothing to check against
  }
  const double period = options.period;
  const bool period_ok = period > 0.0 && std::isfinite(period);
  const double resolution = map.grid->resolution();
  const double r_circ = checker.footprint().rCirc();
  // A point at r_circ moves by the chord 2 r_circ sin(dpsi/2) <= res (math_spec §10.2).
  const double max_yaw_step =
    2.0 * r_circ > resolution ? 2.0 * std::asin(resolution / (2.0 * r_circ)) : kInf;

  Pose2 pose{start.p, wrapAngle(start.yaw)};
  double arc = 0.0;            // base arc length at `pose`
  double last_free_arc = 0.0;  // base arc length at the last collision-free checked pose
  bool rotation_only = true;   // every step so far was in a rotation phase
  Phase previous_phase = Phase::kFollow;

  for (std::size_t k = 0;; ++k) {
    const double elapsed = static_cast<double>(k) * period;
    if (k > 0 && elapsed >= options.horizon - kHorizonSlack) {
      // While a cancel is in progress the STOPPED ramp is followed to its end (math_spec §10.2).
      const bool ramp = options.through_cancel && previous_phase == Phase::kStopped;
      if (!ramp || elapsed >= options.horizon + kCancelRampMaxExtraTime) {
        result.reached_horizon = true;
        break;
      }
    }

    const RolloutStep step = policy(pose, options.t0 + elapsed);

    if (k == 0) {
      // The start pose is checked in every phase: kBlocked's current-pose test reads it.
      const PoseCheck check = checkPose(checker, map, pose);
      if (check == PoseCheck::kOffGrid) {
        result.off_grid = true;
        result.s_free = 0.0;
        break;
      }
      poses->push_back(pose);
      if (check == PoseCheck::kCollides) {
        result.collided = true;
        result.first_collision = 0;
        result.collided_in_rotation = isRotationPhase(step.phase);
        result.s_free = 0.0;
        break;
      }
    }

    if (step.phase == Phase::kStopped) {
      if (!options.through_cancel || belowStopThresholds(step.cmd)) {
        result.stopped = true;
        break;
      }
    }
    // A settled step is still integrated (its deceleration is checked); the rollout ends after it.
    const bool last_step = step.settled && step.phase != Phase::kStopped;
    if (!period_ok) {
      result.reached_horizon = true;
      break;
    }

    rotation_only = rotation_only && isRotationPhase(step.phase);
    const Twist2 feasible = sim::feasibleTwist(plant, step.cmd);
    const double speed = baseSpeed(plant, step.cmd);
    if (!isFinite(feasible) || !std::isfinite(speed)) {
      break;  // the limiter's guard makes this unreachable; never integrate garbage
    }
    const double travel = speed * period;
    const double turn = std::abs(feasible.wz) * period;
    double sub_steps = std::max(1.0, std::ceil(travel / resolution));
    sub_steps = std::max(sub_steps, std::ceil(turn / max_yaw_step));
    sub_steps = std::min(sub_steps, kMaxSubSteps);
    const auto count = static_cast<std::size_t>(sub_steps);

    bool done = false;
    Pose2 sub = pose;
    for (std::size_t j = 1; j <= count; ++j) {
      // Each sub-step pose is integrated from the period start: exact, no accumulation.
      const double tau = period * static_cast<double>(j) / sub_steps;
      sub = sim::integrate(plant, pose, step.cmd, tau);
      sub.yaw = wrapAngle(sub.yaw);
      const PoseCheck check = checkPose(checker, map, sub);
      if (check == PoseCheck::kOffGrid) {
        result.off_grid = true;
        result.s_free = last_free_arc;
        done = true;
        break;
      }
      poses->push_back(sub);
      if (check == PoseCheck::kCollides) {
        result.collided = true;
        result.first_collision = static_cast<int>(poses->size()) - 1;
        result.collided_in_rotation = rotation_only;
        result.s_free = last_free_arc;
        done = true;
        break;
      }
      last_free_arc = arc + speed * tau;
    }
    if (done) {
      break;
    }
    pose = sub;
    arc += travel;
    previous_phase = step.phase;
    ++result.steps;
    if (last_step) {
      result.settled = true;
      break;
    }
  }
  return result;
}

}  // namespace detail

CollisionPredictor::CollisionPredictor(const Kinematics & plant, const FootprintChecker & checker)
: plant_(plant), checker_(checker)
{
}

RolloutResult CollisionPredictor::rollout(
  const Pose2 & start, const ObstacleMap & map, const RolloutOptions & options,
  RolloutPolicy policy, std::vector<Pose2> * poses) const noexcept
{
  return detail::runRollout(plant_, checker_, start, map, options, policy, poses);
}

const Kinematics & CollisionPredictor::plant() const noexcept { return plant_; }

const FootprintChecker & CollisionPredictor::checker() const noexcept { return checker_; }

double brakingCap(double s_free, double stop_margin, double a_dec) noexcept
{
  if (s_free == kInf) {
    return kInf;
  }
  if (std::isnan(s_free) || !(a_dec > 0.0)) {
    return 0.0;  // no usable distance or deceleration: stop
  }
  const double room = std::max(0.0, s_free - stop_margin);
  return std::sqrt(2.0 * a_dec * room);
}

}  // namespace vectorfield_core
