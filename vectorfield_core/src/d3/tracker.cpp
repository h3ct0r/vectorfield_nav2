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

// Tracker3 (math_spec §14.5) and its configuration check.

#include "vectorfield_core/d3/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "vectorfield_core/cbf.hpp"
#include "vectorfield_core/errors.hpp"

namespace vectorfield_core::d3
{

namespace
{

void require(bool condition, const std::string & field, double value, const char * rule)
{
  if (!condition) {
    throw ConfigError(
      "Tracker3Config." + field + " = " + std::to_string(value) + ": must be " + rule);
  }
}

bool positive(double value) noexcept { return std::isfinite(value) && value > 0.0; }

}  // namespace

void validateTracker3Config(const Tracker3Config & c)
{
  require(positive(c.desired_speed), "desired_speed", c.desired_speed, "finite and > 0");
  require(positive(c.convergence_gain), "convergence_gain", c.convergence_gain, "finite and > 0");
  require(c.direction == 1 || c.direction == -1, "direction", c.direction, "+1 or -1");
  require(
    std::isfinite(c.assumption3_margin) && c.assumption3_margin > 0.0 && c.assumption3_margin < 1.0,
    "assumption3_margin", c.assumption3_margin, "in (0, 1)");
  require(positive(c.goal_tolerance), "goal_tolerance", c.goal_tolerance, "finite and > 0");
  require(
    positive(c.terminal_braking_distance), "terminal_braking_distance", c.terminal_braking_distance,
    "finite and > 0");
  require(
    std::isfinite(c.min_approach_speed) && c.min_approach_speed >= 0.0 &&
      c.min_approach_speed <= c.desired_speed,
    "min_approach_speed", c.min_approach_speed, "in [0, desired_speed]");
  require(
    std::isfinite(c.final_approach_distance), "final_approach_distance", c.final_approach_distance,
    "finite (negative = auto)");
  require(
    positive(c.control_frequency), "control_frequency", c.control_frequency, "finite and > 0");
  const Limits3 & l = c.limits;
  require(positive(l.max_speed), "limits.max_speed", l.max_speed, "finite and > 0");
  for (int i = 0; i < 3; ++i) {
    require(
      l.max_axis_speed[i] > 0.0 && !std::isnan(l.max_axis_speed[i]),
      std::string("limits.max_axis_speed[") + std::to_string(i) + "]", l.max_axis_speed[i],
      "> 0 (inf allowed)");
  }
  require(positive(l.max_accel), "limits.max_accel", l.max_accel, "finite and > 0");
  require(positive(l.max_decel), "limits.max_decel", l.max_decel, "finite and > 0");
  require(positive(l.cancel_decel), "limits.cancel_decel", l.cancel_decel, "finite and > 0");
  require(positive(l.max_yaw_rate), "limits.max_yaw_rate", l.max_yaw_rate, "finite and > 0");
  require(positive(l.max_yaw_accel), "limits.max_yaw_accel", l.max_yaw_accel, "finite and > 0");
  require(
    std::isfinite(c.yaw.gain) && c.yaw.gain >= 0.0, "yaw.gain", c.yaw.gain, "finite and >= 0");
  require(std::isfinite(c.yaw.fixed_yaw), "yaw.fixed_yaw", c.yaw.fixed_yaw, "finite");
  const Obstacles3Config & o = c.obstacles;
  require(
    o.obstacle_mode == ObstacleMode::kNone || o.obstacle_mode == ObstacleMode::kCbf,
    "obstacles.obstacle_mode", static_cast<double>(o.obstacle_mode),
    "none (0) or cbf (2): contour is planar only");
  require(
    std::isfinite(o.body_radius) && o.body_radius >= 0.0, "obstacles.body_radius", o.body_radius,
    "finite and >= 0");
  require(
    std::isfinite(o.cbf_margin) && o.cbf_margin >= 0.0, "obstacles.cbf_margin", o.cbf_margin,
    "finite and >= 0");
  require(positive(o.cbf_gain), "obstacles.cbf_gain", o.cbf_gain, "finite and > 0");
  require(
    o.cbf_gain < c.control_frequency, "obstacles.cbf_gain", o.cbf_gain,
    "< control_frequency (cbf_gain * dt < 1, math_spec 14.8)");
  require(
    o.cbf_sites >= 1 && o.cbf_sites <= kCbf3MaxSites, "obstacles.cbf_sites", o.cbf_sites,
    "in [1, 4]");
}

namespace
{

Tracker3Config validated(const Tracker3Config & config)
{
  validateTracker3Config(config);
  return config;
}

/// @brief The sites of the CBF filter at the body x and their barriers (math_spec §14.8).
void sampleObstacles(
  const Obstacles3Config & config, const ObstacleSet3 & obstacles, const Vec3 & x,
  Cbf3Sample * cbf) noexcept
{
  *cbf = Cbf3Sample{};
  cbf->on = true;
  cbf->delta = config.body_radius + config.cbf_margin;
  cbf->count = obstacles.sites(x, config.cbf_sites, cbf->sites);
  for (int j = 0; j < cbf->count; ++j) {
    cbf->h = std::min(cbf->h, cbf->sites[j].D - cbf->delta);
  }
}

/// @brief \f$F^\star\f$ of a target F under the sampled sites, with
/// \f$h_j = D_j - \delta_{\mathrm{safe}}\f$.
Vec3 filterCbf(const Obstacles3Config & config, const Vec3 & F, Cbf3Sample * cbf) noexcept
{
  Cbf3Constraint constraints[kCbf3MaxSites];
  for (int j = 0; j < cbf->count; ++j) {
    constraints[j].n = cbf->sites[j].n;
    constraints[j].h = cbf->sites[j].D - cbf->delta;
  }
  const Cbf3Solution solution = solveCbf3(F, constraints, cbf->count, config.cbf_gain);
  cbf->F_in = F;
  cbf->F = solution.F;
  cbf->active = solution.active;
  cbf->relaxed = solution.relaxed;
  cbf->binds = solution.binds;
  const double norm = F.norm();
  cbf->speed_ratio = (solution.active && norm > 0.0) ? solution.F.norm() / norm : 1.0;
  return solution.F;
}

}  // namespace

Tracker3::Tracker3(const Tracker3Config & config)
: config_(validated(config)),
  field_(
    GainFunction(config.gain_function, config.convergence_gain), config.desired_speed,
    config.direction, config.assumption3_margin),
  law_(config.limits, config.yaw),
  profile_(
    config.desired_speed, config.min_approach_speed, config.terminal_braking_distance,
    0.5 * config.goal_tolerance),
  final_approach_distance_(
    isAuto(config.final_approach_distance)
      ? std::max(config.terminal_braking_distance, config.goal_tolerance)
      : config.final_approach_distance)
{
}

Tracker3Output Tracker3::step(
  const Tracker3Input & input, const Tracker3State & in, Tracker3State * out) const noexcept
{
  Tracker3Output output;
  Tracker3State state = in;

  // 1. Goal identity.
  if (input.goal_seq != state.goal_seq) {
    state.goal_seq = input.goal_seq;
    state.xy_latched = false;
    state.holding = false;
    state.blocked = false;
    state.phase = Phase::kFollow;
  }
  // Limiter re-seed (initial state, or a gap longer than two periods).
  if (!state.limiter.valid || input.dt > 2.0 / config_.control_frequency) {
    state.limiter.u_prev = isFinite(input.measured) ? input.measured : Twist3{};
    state.limiter.valid = true;
  }

  // 2. Cancel: STOPPED, latched until goal_seq changes.
  if (input.cancel) {
    state.phase = Phase::kStopped;
  }

  // 3. Closest point and field. The search runs in the curve frame F with the hint kept there;
  // x*, T, n and the velocities are mapped to the evaluation frame (math_spec §14.7). The
  // identity static frame skips every mapping, so it is bit-identical to a tracker without frames.
  const Curve3 * curve = input.curve;
  const CurveFrame3 & frame = input.frame;
  const bool identity = !frame.moving && frame.T_eval_curve.matrix() == Eigen::Matrix4d::Identity();
  const bool inputs_ok = curve != nullptr && isFinite(input.pose) && std::isfinite(input.t) &&
                         (identity || isFinite(frame));
  bool field_ok = false;
  const double tol = config_.goal_tolerance;
  // Goal hold in a moving frame: the chord that keeps the body fixed in F over the next
  // period, (T exp(dt xi) p_F - p)/dt (no drift under ZOH for a constant twist).
  Vec3 hold_velocity = Vec3::Zero();
  if (inputs_ok) {
    const Mat3 R = frame.T_eval_curve.linear();
    const Vec3 c = frame.T_eval_curve.translation();
    const Vec3 x_curve = identity ? input.pose.p : Vec3(R.transpose() * (input.pose.p - c));
    EvalContext3 context;
    context.t = input.t;
    context.direction = config_.direction;
    context.last_tangent = state.last_tangent;
    const ClosestPoint3 cp_curve = curve->closestPoint(x_curve, context, state.hint);
    ClosestPoint3 cp = cp_curve;
    if (!identity) {
      cp.x_star = R * cp_curve.x_star + c;
      cp.T = R * cp_curve.T;
      cp.n = R * cp_curve.n;
    }
    output.x_star_curve = cp_curve.x_star;
    output.frame_velocity = frameVelocity(frame, x_curve);
    if (frame.moving) {
      hold_velocity = output.frame_velocity;
      if (input.dt > 0.0 && std::isfinite(input.dt)) {
        hold_velocity = (extrapolate(frame, input.dt) * x_curve - input.pose.p) / input.dt;
      }
    }
    if (curve->isClosed()) {
      // v_c = R (v_F + w_F x r_F + dr_F/dt) at the foot.
      Vec3 v_c = curve->velocity(cp_curve, input.t);
      if (!identity) {
        if (frame.moving) {
          v_c += frame.twist.v + frame.twist.w.cross(cp_curve.x_star);
        }
        v_c = R * v_c;
      }
      field_.evaluate(input.pose.p, cp, v_c, &output.field);
    } else {
      const double s_goal = config_.direction > 0 ? curve->endStation(input.t) : 0.0;
      const double sigma = static_cast<double>(config_.direction) * (s_goal - cp.s_star);
      if (frame.moving) {
        field_.evaluateTerminal(
          input.pose.p, cp, sigma, profile_, output.frame_velocity, &output.field);
      } else {
        field_.evaluateTerminal(input.pose.p, cp, sigma, profile_, &output.field);
      }
      output.has_goal = true;
      const Vec3 goal_curve = curve->pointAt(s_goal, input.t);
      output.goal = identity ? goal_curve : Vec3(R * goal_curve + c);
    }
    field_ok = output.field.valid;
    if (cp_curve.valid) {
      state.hint = makeHint(cp_curve);
      state.last_tangent = cp_curve.T;
    }
  } else {
    output.field.status |= Status::kNonFinite;
  }
  output.status |= output.field.status;

  // 4. Goal predicates (open curves).
  if (output.has_goal && field_ok) {
    const double distance = (input.pose.p - output.goal).norm();
    if (distance <= tol) {
      state.xy_latched = true;
    }
    output.settled = state.xy_latched && distance <= 0.5 * tol;
    state.holding = output.settled || (state.holding && state.xy_latched);
    if (state.phase == Phase::kFollow && output.field.sigma < final_approach_distance_) {
      state.phase = Phase::kFinalApproach;
    }
  }

  // 5. Target. The goal hold of an open curve in a moving frame rides with the frame: the
  // target is the chord velocity of the body's point of F (inside the kinematic box), not zero.
  const bool stopped = state.phase == Phase::kStopped;
  const bool transport_hold = state.holding && frame.moving && field_ok && !stopped;
  bool zero_target = stopped || (state.holding && !transport_hold) || !field_ok;
  // The CBF filter (math_spec §14.8) acts at the body on every nonzero translational target: the
  // law's command after its kinematic box, and the transported goal hold. A filtered target is
  // boxed again by a uniform scale, which keeps every constraint with h_j >= 0.
  const ObstacleSet3 * obstacles = input.obstacles;
  const bool cbf_on = config_.obstacles.obstacle_mode == ObstacleMode::kCbf &&
                      obstacles != nullptr && !obstacles->empty() && inputs_ok;
  if (cbf_on) {
    sampleObstacles(config_.obstacles, *obstacles, input.pose.p, &output.cbf);
  }
  bool blocked = false;
  if (transport_hold) {
    const Vec3 v =
      cbf_on ? filterCbf(config_.obstacles, hold_velocity, &output.cbf) : hold_velocity;
    output.box_scale = uniformBoxScale(v, config_.limits);
    output.cmd_target.v = output.box_scale * v;
  } else if (!zero_target) {
    LawOutput3 law = law_.command(
      output.field, input.speed_limit_scale, config_.desired_speed, config_.assumption3_margin,
      input.pose.yaw, state.last_psi_d);
    output.status |= law.status;
    if (cbf_on) {
      const Vec3 filtered = filterCbf(config_.obstacles, law.cmd.v, &output.cbf);
      if (output.cbf.active) {
        law = law_.commandUniform(filtered, output.field.T_d, input.pose.yaw, state.last_psi_d);
      }
      // The hold (as math_spec §6.7, §10.4): the filter leaves under 5 % of the field speed
      // (released at 10 %); FOLLOW or FINAL_APPROACH, never settled or holding (here).
      output.cbf.hold = output.cbf.active &&
                        output.cbf.speed_ratio < (state.blocked ? kCbfReleaseRatio : kCbfHoldRatio);
      blocked = output.cbf.hold;
    }
    if (blocked) {
      zero_target = true;  // kBlocked: the target becomes 0 and the limiter brakes
      output.status |= Status::kBlocked;
    } else {
      output.cmd_target = law.cmd;
      output.box_scale = law.c;
      output.psi_d = law.psi_d;
      state.last_psi_d = law.psi_d;
    }
  }
  state.blocked = blocked;

  // 6. Limiter.
  const CommandLimiter3 limiter;
  output.cmd = limiter.limit(
    output.cmd_target, state.limiter, &state.limiter, input.dt, config_.limits,
    stopped && input.cancel, zero_target, &output.status, &output.limiter_alpha);
  output.cancel_complete = stopped && output.cmd.v.norm() < kStopLinearSpeed &&
                           std::abs(output.cmd.wz) < kStopAngularSpeed;
  output.phase = state.phase;
  if (out != nullptr) {
    *out = state;
  }
  return output;
}

}  // namespace vectorfield_core::d3
