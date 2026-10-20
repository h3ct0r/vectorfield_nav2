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

// Speed-shaping terms, the kinematic box and the CommandLimiter (math_spec §8.2–§8.4).

#include "vectorfield_core/limiter.hpp"

#include <algorithm>
#include <cmath>

namespace vectorfield_core
{

namespace
{

constexpr Interval kEmptyInterval{kInf, -kInf};
constexpr Interval kAllReals{-kInf, kInf};

/// @brief \f$\min(c, L/\lvert x\rvert)\f$, dropping the term when its denominator is zero
/// (math_spec §8.3: "terms with a zero denominator are dropped"). A NaN limit gives NaN, so a bad
/// limit reaches the NaN guard instead of being ignored.
double scaleTerm(double c, double limit, double magnitude) noexcept
{
  if (magnitude > 0.0) {
    const double ratio = limit / magnitude;
    return std::isnan(ratio) ? kNaN : std::min(c, ratio);
  }
  return c;
}

/// @brief Acceleration bound of a linear axis (math_spec §8.4, Appendix C item 11):
/// \f$a_{\mathrm{dec}}\f$ when \f$u_{\mathrm{prev},i}\Delta_i < 0\f$, else \f$a_{\mathrm{acc}}\f$;
/// during a cancel \f$a_{\mathrm{can}}\f$, or \f$\max(a_{\mathrm{can}}, a_{\mathrm{dec}})\f$ once a
/// rollout found a collision.
double linearBound(
  double previous, double delta, const Kinematics & kinematics, bool stopped_cancel,
  bool cancel_hard) noexcept
{
  if (stopped_cancel) {
    return cancel_hard ? std::max(kinematics.cancel_deceleration, kinematics.max_linear_decel)
                       : kinematics.cancel_deceleration;
  }
  return previous * delta < 0.0 ? kinematics.max_linear_decel : kinematics.max_linear_accel;
}

/// @brief \f$\min(\alpha, A\Delta t/\lvert\Delta\rvert)\f$ over an axis with \f$\Delta \ne 0\f$.
double axisAlpha(double alpha, double bound, double dt, double delta) noexcept
{
  const double magnitude = std::abs(delta);
  if (magnitude > 0.0) {
    return std::min(alpha, std::max(0.0, bound) * dt / magnitude);
  }
  return alpha;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Speed shaping (math_spec §8.2).

double SpeedShaping::gamma() const noexcept
{
  if (
    std::isnan(gamma_lim) || std::isnan(gamma_cl) || std::isnan(gamma_ap) ||
    std::isnan(gamma_col)) {
    return kNaN;
  }
  return std::min({1.0, gamma_lim, gamma_cl, gamma_ap, gamma_col});
}

ShapingTerm SpeedShaping::binding() const noexcept
{
  const double g = gamma();
  if (!(g < 1.0)) {
    return ShapingTerm::kNone;
  }
  // Ties resolve in the order limit, approach, clearance, collision: the collision cap binds
  // only when it is strictly the smallest term.
  if (gamma_lim == g) {
    return ShapingTerm::kSpeedLimit;
  }
  if (gamma_ap == g) {
    return ShapingTerm::kApproach;
  }
  if (gamma_cl == g) {
    return ShapingTerm::kClearance;
  }
  if (gamma_col == g) {
    return ShapingTerm::kCollision;
  }
  return ShapingTerm::kNone;
}

bool SpeedShaping::capBinds() const noexcept
{
  return gamma_col < std::min({1.0, gamma_lim, gamma_cl, gamma_ap});
}

bool SpeedShaping::collisionAhead() const noexcept
{
  const ShapingTerm term = binding();
  return term == ShapingTerm::kCollision || term == ShapingTerm::kClearance;
}

double clearanceScale(
  double edt_at_base, double w_lat, double slowdown_distance, double slowdown_gain) noexcept
{
  if (!(slowdown_distance > 0.0)) {
    return 1.0;
  }
  // A NaN distance gives zero clearance: the conservative reading.
  const double clearance = std::max(0.0, edt_at_base - w_lat);
  const double scale = slowdown_gain * clearance / slowdown_distance;
  return std::isnan(scale) ? 1.0 : std::min(1.0, scale);
}

double approachScale(double sigma_end, double v_min, double a_dec, double v_r) noexcept
{
  if (std::isnan(sigma_end)) {
    return 1.0;
  }
  const double speed = std::sqrt(v_min * v_min + 2.0 * a_dec * std::max(0.0, sigma_end));
  const double scale = speed / v_r;
  return std::isnan(scale) ? 1.0 : std::min(1.0, scale);
}

double collisionScale(double v_cap, double v_r) noexcept
{
  if (std::isnan(v_cap)) {
    // A failed prediction must not lift the cap: NaN reaches the command and the NaN guard.
    return kNaN;
  }
  return std::min(1.0, std::max(0.0, v_cap) / v_r);
}

// -----------------------------------------------------------------------------------------------
// Kinematic box (math_spec §8.3).

Interval boxAxisInterval(double a, double b, double limit) noexcept
{
  if (std::isnan(a) || std::isnan(b) || std::isnan(limit) || limit < 0.0) {
    return kEmptyInterval;
  }
  if (b != 0.0) {
    const double e1 = (limit - a) / b;
    const double e2 = (-limit - a) / b;
    return Interval{std::min(e1, e2), std::max(e1, e2)};
  }
  return std::abs(a) <= limit ? kAllReals : kEmptyInterval;
}

Interval boxNormInterval(const Vec2 & a, const Vec2 & b, double limit) noexcept
{
  if (!isFinite(a) || !isFinite(b) || std::isnan(limit) || limit < 0.0) {
    return kEmptyInterval;
  }
  // ||a + eta b||^2 <= L^2  <=>  B eta^2 + 2 A eta + C <= 0.
  const double B = b.squaredNorm();
  const double A = a.dot(b);
  const double C = a.squaredNorm() - limit * limit;
  if (B == 0.0) {
    return C <= 0.0 ? kAllReals : kEmptyInterval;
  }
  const double discriminant = A * A - B * C;
  if (discriminant < 0.0) {
    return kEmptyInterval;
  }
  // The roots (-A -+ sqrt(disc))/B of math_spec §8.3, with the root that would cancel taken from
  // the product of the roots, C/B.
  const double root = std::sqrt(discriminant);
  if (A >= 0.0) {
    const double q = A + root;
    return Interval{-q / B, q > 0.0 ? -C / q : 0.0};
  }
  const double q = root - A;
  return Interval{C / q, q / B};
}

BoxResult uniformBox(const Twist2 & cmd, const Kinematics & kinematics, double omega_bar) noexcept
{
  BoxResult result;
  double c = 1.0;
  if (kinematics.isHolonomic()) {
    // Translation only; omega is clamped on its own (math_spec §8.3).
    c = scaleTerm(c, kinematics.max_linear_vel, std::hypot(cmd.vx, cmd.vy));
    c = scaleTerm(c, kinematics.max_lateral_vel, std::abs(cmd.vy));
    result.cmd = Twist2{c * cmd.vx, c * cmd.vy, clamp(cmd.wz, -omega_bar, omega_bar)};
  } else {
    c = scaleTerm(c, kinematics.max_linear_vel, std::abs(cmd.vx));
    c = scaleTerm(c, omega_bar, std::abs(cmd.wz));
    result.cmd = Twist2{c * cmd.vx, 0.0, c * cmd.wz};
  }
  result.c = c;
  return result;
}

BoxResult etaBox(
  const Vec2 & a, const Vec2 & b, double eta1, double holonomic_omega, const Twist2 & fallback_cmd,
  const Kinematics & kinematics, double omega_bar) noexcept
{
  // A negative eta is never commanded: start from [0, eta1] (math_spec §8.3, oracle O44).
  Interval set{0.0, eta1};
  if (std::isnan(eta1)) {
    set = kEmptyInterval;
  }
  if (kinematics.isHolonomic()) {
    set = set.intersect(boxAxisInterval(a.x(), b.x(), kinematics.max_linear_vel));
    set = set.intersect(boxAxisInterval(a.y(), b.y(), kinematics.max_lateral_vel));
    set = set.intersect(boxNormInterval(a, b, kinematics.max_linear_vel));
  } else {
    set = set.intersect(boxAxisInterval(a.x(), b.x(), kinematics.max_linear_vel));
    set = set.intersect(boxAxisInterval(a.y(), b.y(), omega_bar));
  }

  if (set.empty()) {
    // M Phi_T alone violates the box: uniform c on the step-3 command, and the flag.
    BoxResult result = uniformBox(fallback_cmd, kinematics, omega_bar);
    result.fallback = true;
    result.status |= Status::kAssumption3Clamped;
    return result;
  }

  BoxResult result;
  const double eta = set.hi;
  const Vec2 u = a + eta * b;
  if (kinematics.isHolonomic()) {
    result.cmd = Twist2{u.x(), u.y(), clamp(holonomic_omega, -omega_bar, omega_bar)};
  } else {
    result.cmd = Twist2{u.x(), 0.0, u.y()};
  }
  result.c = 1.0;
  result.eta = eta;
  result.eta_box = true;
  return result;
}

// -----------------------------------------------------------------------------------------------
// CommandLimiter (math_spec §8.4).

Twist2 CommandLimiter::limit(
  const Twist2 & target, const LimiterState & in, LimiterState * out, double dt,
  const Kinematics & kinematics, bool stopped_cancel, std::uint32_t * status, double * alpha,
  bool brake_translation) const noexcept
{
  // `out` may alias `in`.
  const LimiterState previous = in;
  std::uint32_t bits = Status::kNone;

  // Unicycle and Ackermann models have no lateral axis: a lateral u_prev (e.g. re-seeded from a
  // slipping odometry twist) must neither be commanded nor slow the other axes.
  const bool holonomic = kinematics.isHolonomic();
  Twist2 goal = target;
  Twist2 from = previous.u_prev;
  if (!holonomic) {
    goal.vy = 0.0;
    from.vy = 0.0;
  }

  Twist2 u = from;
  double used_alpha = 0.0;
  if (!isFinite(goal) || !isFinite(from) || std::isnan(dt) || std::isinf(dt)) {
    u = Twist2{kNaN, kNaN, kNaN};
  } else {
    const double dx = goal.vx - from.vx;
    const double dy = goal.vy - from.vy;
    const double dw = goal.wz - from.wz;
    if (dx == 0.0 && dy == 0.0 && dw == 0.0) {
      used_alpha = 1.0;
    } else if (dt > 0.0) {
      // 1. Acceleration: the largest alpha in [0, 1] meeting every axis bound; all axes move
      //    along the straight line from u_prev to the target. When the phase brakes the
      //    translation (rotation phases, STOPPED), the translational axes get their own alpha,
      //    so omega's alpha_max ramp does not slow the stop [Design, 0.1.1].
      double a_lin = 1.0;
      a_lin = axisAlpha(
        a_lin, linearBound(from.vx, dx, kinematics, stopped_cancel, previous.cancel_hard), dt, dx);
      a_lin = axisAlpha(
        a_lin, linearBound(from.vy, dy, kinematics, stopped_cancel, previous.cancel_hard), dt, dy);
      const double a_ang = axisAlpha(1.0, kinematics.max_angular_accel, dt, dw);
      const double a = std::min(a_lin, a_ang);
      const double a_trans = brake_translation ? a_lin : a;
      const double a_rot = brake_translation ? a_ang : a;
      used_alpha = a;
      u = Twist2{from.vx + a_trans * dx, from.vy + a_trans * dy, from.wz + a_rot * dw};
    }
  }

  // 2. Ackermann curvature clamp.
  if (kinematics.isAckermann() && kinematics.min_turning_radius > 0.0 && isFinite(u)) {
    const double bound = std::abs(u.vx) / kinematics.min_turning_radius;
    if (std::abs(u.wz) > bound) {
      u.wz = sgn(u.wz) * bound;
      bits |= Status::kCurvatureSaturated;
    }
  }

  // 3. Non-finite guard.
  if (!isFinite(u)) {
    u = Twist2{};
    used_alpha = 0.0;
    bits |= Status::kNonFinite;
  }

  if (out != nullptr) {
    out->u_prev = u;
    out->cancel_hard = previous.cancel_hard;
    out->valid = true;
  }
  if (status != nullptr) {
    *status |= bits;
  }
  if (alpha != nullptr) {
    *alpha = used_alpha;
  }
  return u;
}

}  // namespace vectorfield_core
