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

#include "vectorfield_curve_follower/goal_supervisor.hpp"

#include <algorithm>
#include <cmath>

#include "vectorfield_core/status.hpp"

namespace vectorfield_curve_follower
{

namespace vf = vectorfield_core;

namespace
{

SupervisorDecision abortWith(std::uint16_t code, bool decelerate, const char * reason) noexcept
{
  SupervisorDecision decision;
  decision.action = SupervisorAction::kAbort;
  decision.code = code;
  decision.decelerate = decelerate;
  decision.reason = reason;
  return decision;
}

SupervisorDecision hold(SupervisorAction action, const char * reason) noexcept
{
  SupervisorDecision decision;
  decision.action = action;
  decision.reason = reason;
  return decision;
}

}  // namespace

GoalSupervisor::GoalSupervisor(const SupervisorConfig & config, double start_time) noexcept
{
  reset(config, start_time);
}

void GoalSupervisor::reset(const SupervisorConfig & config, double start_time) noexcept
{
  config_ = config;
  start_time_ = start_time;
  last_time_ = start_time;
  has_last_time_ = false;
  stale_since_ = -1.0;
  tf_error_since_ = -1.0;
  blocked_since_ = -1.0;
  a3_time_ = 0.0;
  has_anchor_ = false;
  anchor_ = vf::Vec2::Zero();
  anchor_time_ = start_time;
}

double GoalSupervisor::progressRadius(double nominal_speed) const noexcept
{
  const double speed = std::isfinite(nominal_speed) ? std::max(0.0, nominal_speed) : 0.0;
  return std::min(config_.progress_radius, 0.5 * speed * config_.progress_timeout);
}

SupervisorDecision GoalSupervisor::update(const SupervisorInput & input) noexcept
{
  const double now = input.now;
  const double dt = has_last_time_ ? std::max(0.0, now - last_time_) : 0.0;
  last_time_ = now;
  has_last_time_ = true;
  const std::uint32_t status = input.tracker_ran ? input.status : vf::Status::kNone;

  // 1. Deactivation, then cancel.
  if (input.deactivating) {
    return abortWith(result_code::kUnknown, false, "the follower is deactivating");
  }
  if (input.cancel_requested) {
    return hold(SupervisorAction::kCancel, "cancel requested");
  }
  // 2. Immediate aborts and the goal's time limit.
  if (input.nav2_active) {
    return abortWith(result_code::kNav2Active, false, "a Nav2 action became active");
  }
  if (input.config_invalid) {
    return abortWith(result_code::kUnknown, false, "the configuration was rejected");
  }
  if (vf::Status::has(status, vf::Status::kNonFinite)) {
    return abortWith(result_code::kUnknown, false, "the command was not finite");
  }
  if (config_.max_duration > 0.0 && now - start_time_ >= config_.max_duration) {
    return abortWith(result_code::kTimeout, true, "max_duration reached");
  }

  // 3. Holds: the Tracker did not run. They re-anchor the progress watchdog.
  if (!input.curve_frame_ok) {
    has_anchor_ = false;
    if (tf_error_since_ < 0.0) {
      tf_error_since_ = now;
    }
    if (now - tf_error_since_ >= config_.stale_abort_timeout) {
      return abortWith(result_code::kTfError, false, "the curve frame is not transformable");
    }
    return hold(SupervisorAction::kZero, "the curve frame is not transformable");
  }
  tf_error_since_ = -1.0;
  if (input.inputs_stale || input.frame_stale) {
    has_anchor_ = false;
    if (stale_since_ < 0.0) {
      stale_since_ = now;
    }
    if (now - stale_since_ >= config_.stale_abort_timeout) {
      return abortWith(
        result_code::kStaleInput, true,
        input.frame_stale ? "the moving frame's input is stale" : "an input is stale");
    }
    return hold(
      SupervisorAction::kDecelerate,
      input.frame_stale ? "the moving frame's input is stale" : "an input is stale");
  }
  stale_since_ = -1.0;
  if (!input.tracker_ran) {
    return SupervisorDecision{};
  }

  // 4. Tracker stops.
  if (vf::Status::has(status, vf::Status::kHeadingInfeasible)) {
    return abortWith(
      result_code::kUnsupported, true,
      "the curve's heading is infeasible for the motion model (kHeadingInfeasible)");
  }
  if (vf::Status::has(status, vf::Status::kContourWatchdog)) {
    return abortWith(result_code::kBlocked, true, "the obstacle contour watchdog expired");
  }

  // 5. kBlocked and the progress watchdog.
  const bool blocked = vf::Status::has(status, vf::Status::kBlocked);
  if (blocked) {
    if (blocked_since_ < 0.0) {
      blocked_since_ = now;
    }
    if (now - blocked_since_ >= config_.blocked_timeout) {
      return abortWith(result_code::kBlocked, false, "blocked for blocked_timeout");
    }
  } else {
    blocked_since_ = -1.0;
  }
  const double radius = progressRadius(input.nominal_speed);
  if (!has_anchor_ || (input.position - anchor_).norm() > radius) {
    has_anchor_ = true;
    anchor_ = input.position;
    anchor_time_ = now;
  } else if (now - anchor_time_ >= config_.progress_timeout) {
    return abortWith(result_code::kBlocked, false, "no progress within progress_timeout");
  }

  // 6. The Assumption-3 timer, paused while blocked.
  if (!blocked) {
    if (vf::Status::has(status, vf::Status::kAssumption3Clamped)) {
      a3_time_ += dt;
      if (a3_time_ > config_.assumption3_abort_time) {
        return abortWith(
          result_code::kAssumption3Violated, true,
          "the curve moves too fast for the reference speed (Assumption 3)");
      }
    } else {
      a3_time_ = 0.0;
    }
  }
  return SupervisorDecision{};
}

bool belowStopThresholds(const vf::Twist2 & twist) noexcept
{
  return std::hypot(twist.vx, twist.vy) < kStopLinearThreshold &&
         std::abs(twist.wz) < kStopAngularThreshold;
}

vf::Twist2 rampDown(
  const vf::Twist2 & twist, double dt, double linear_decel, double angular_decel) noexcept
{
  if (
    !vf::isFinite(twist) || !(dt > 0.0) || !std::isfinite(dt) || !(linear_decel > 0.0) ||
    !(angular_decel > 0.0)) {
    return vf::Twist2{};
  }
  vf::Twist2 out;
  const double speed = std::hypot(twist.vx, twist.vy);
  if (speed > 0.0) {
    const double scale = std::max(0.0, speed - linear_decel * dt) / speed;
    out.vx = twist.vx * scale;
    out.vy = twist.vy * scale;
  }
  const double rate = std::max(0.0, std::abs(twist.wz) - angular_decel * dt);
  out.wz = std::copysign(rate, twist.wz);
  return belowStopThresholds(out) ? vf::Twist2{} : out;
}

}  // namespace vectorfield_curve_follower
