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

// The path field (math_spec §4–§5) and speed shaping (§8.2).

#include "vectorfield_core/field.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace vectorfield_core
{

namespace
{

/// Relative slack of the Assumption-3 comparisons in DistanceField::shape(). clampFeedforward()
/// scales an over-long \f$\Phi_T\f$ to norm \f$(1 - \varepsilon_v)v_r\f$, which holds only to a
/// few ulp. Without the slack, whether a clamped feedforward counts as "at the margin" (math_spec
/// §8.2: "≥", the safety limit wins) would depend on the last bit of its norm.
constexpr double kMarginRelTol = 1e-12;

bool isZero(const Vec2 & v) noexcept { return v.x() == 0.0 && v.y() == 0.0; }

/// @brief The orthonormal frame \f$(T, n)\f$ the field uses at a closest point.
///
/// \f$T\f$ is normalised and \f$n\f$ is re-orthogonalised against it and normalised (as
/// math_spec §2.5 does for parametric curves). The curves guarantee \f$n^\top T = 0\f$ only to
/// rounding; this makes \f$\lVert\Phi_S\rVert = 1\f$, the \f$\eta\f$ formula and
/// \f$\dot D = -\eta G\f$ hold to a few ulp whatever the curve. \f$n\f$ stays zero on the curve.
/// @return False if T is not a usable direction or a vector is non-finite.
bool orthonormalFrame(const ClosestPoint & cp, Vec2 * T, Vec2 * n) noexcept
{
  if (!isFinite(cp.T) || !isFinite(cp.n)) {
    return false;
  }
  const double t_norm = cp.T.norm();
  if (!(t_norm > 0.0)) {
    return false;
  }
  *T = cp.T / t_norm;
  if (isZero(cp.n)) {
    *n = Vec2::Zero();
    return true;
  }
  const Vec2 normal = cp.n - cp.n.dot(*T) * (*T);
  const double n_norm = normal.norm();
  *n = n_norm > 0.0 ? Vec2(normal / n_norm) : Vec2::Zero();
  return true;
}

/// @brief True iff every quantity that evaluate(), evaluateTerminal() and shape() write is
/// finite (\f$\varphi\f$ enters through \f$\Phi_S\f$).
bool fieldIsFinite(const FieldSample & s) noexcept
{
  return isFinite(s.x_eval) && isFinite(s.x_star) && isFinite(s.T) && isFinite(s.n) &&
         std::isfinite(s.D) && std::isfinite(s.s_star) && std::isfinite(s.G) &&
         std::isfinite(s.H) && std::isfinite(s.eta) && std::isfinite(s.gamma) && isFinite(s.v_c) &&
         isFinite(s.Phi_S) && isFinite(s.Phi_T) && isFinite(s.Phi) && isFinite(s.Phi_gamma) &&
         isFinite(s.F) && isFinite(s.conv) && isFinite(s.circ);
}

/// @brief Marks a sample unusable: valid = false and every field vector zero, so that nothing
/// downstream acts on garbage (a law on F = 0 commands zero). `non_finite` adds kNonFinite.
void invalidate(FieldSample * s, bool non_finite) noexcept
{
  s->valid = false;
  s->eta = 0.0;
  s->Phi_S = Vec2::Zero();
  s->Phi_T = Vec2::Zero();
  s->Phi = Vec2::Zero();
  s->Phi_gamma = Vec2::Zero();
  s->F = Vec2::Zero();
  s->conv = Vec2::Zero();
  s->circ = Vec2::Zero();
  if (non_finite) {
    s->status |= Status::kNonFinite;
  }
}

/// @brief Writes the unshaped field (\f$\gamma = 1\f$) from `Phi_S`, `Phi_T`, `G`, `n` and the
/// scale: \f$\Phi = \eta\Phi_S + \Phi_T\f$, \f$\Phi_\gamma = F = \Phi\f$, the convergence arrow
/// \f$-\eta G n\f$ and the circulation arrow \f$\eta(\Phi_S + G n)\f$ (\f$= \eta H T_d\f$, terminal
/// \f$\eta H\varphi T_d\f$, without needing \f$\varphi\f$).
void setUnshaped(FieldSample * s, double eta, bool uniform) noexcept
{
  s->eta = eta;
  s->Phi = eta * s->Phi_S + s->Phi_T;
  s->gamma = 1.0;
  s->Phi_gamma = s->Phi;
  s->F = s->Phi;
  s->conv = -eta * s->G * s->n;
  s->circ = eta * (s->Phi_S + s->G * s->n);
  s->uniform_shaping = uniform;
}

/// @brief Copies the closest point into the sample and checks it.
/// @return 0 = usable, 1 = invalid closest point, 2 = non-finite input.
int fillClosestPoint(const Vec2 & x_eval, const ClosestPoint & cp, FieldSample * s) noexcept
{
  s->status |= cp.status_bits;
  s->x_eval = x_eval;
  s->x_star = cp.x_star;
  s->D = cp.D;
  s->s_star = cp.s_star;
  if (!cp.valid) {
    return 1;
  }
  Vec2 T;
  Vec2 n;
  if (
    !orthonormalFrame(cp, &T, &n) || !isFinite(x_eval) || !isFinite(cp.x_star) ||
    !std::isfinite(cp.D) || cp.D < 0.0 || !std::isfinite(cp.s_star)) {
    return 2;
  }
  s->T = T;
  s->n = n;
  return 0;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Terminal profile (math_spec §5.1).

TerminalProfile::TerminalProfile(
  double v_r, double v_min, double sigma_b, double sigma_stop) noexcept
: v_r_(v_r), v_min_(v_min), sigma_b_(sigma_b), sigma_stop_(sigma_stop)
{
}

double TerminalProfile::operator()(double sigma) const noexcept
{
  if (std::isnan(sigma)) {
    return kNaN;
  }
  const double magnitude = std::abs(sigma);
  if (magnitude == 0.0) {
    return 0.0;
  }
  const double ramp = magnitude / sigma_b_;
  const double floor_ratio = v_min_ / v_r_;
  const double floor =
    sigma_stop_ > 0.0 ? floor_ratio * std::min(1.0, magnitude / sigma_stop_) : floor_ratio;
  return sgn(sigma) * std::min(1.0, std::max(ramp, floor));
}

double TerminalProfile::lipschitz() const noexcept
{
  const double ramp = 1.0 / sigma_b_;
  if (!(v_min_ > 0.0)) {
    return ramp;
  }
  if (!(sigma_stop_ > 0.0)) {
    // The floor term jumps from 0 to v_min/v_r at sigma = 0.
    return kInf;
  }
  return std::max(ramp, v_min_ / (v_r_ * sigma_stop_));
}

// -----------------------------------------------------------------------------------------------
// Feedforward and eta (math_spec §4.2–§4.3).

Vec2 feedforward(const Vec2 & T, const Vec2 & v_c) noexcept { return v_c - T.dot(v_c) * T; }

ClampedFeedforward clampFeedforward(
  const Vec2 & Phi_T, double v_r, double assumption3_margin) noexcept
{
  ClampedFeedforward result;
  const double limit = std::max(0.0, (1.0 - assumption3_margin) * v_r);
  const double norm = Phi_T.norm();
  if (norm > limit) {
    result.Phi_T = Phi_T * (limit / norm);
    result.clamped = true;
  } else {
    result.Phi_T = Phi_T;
  }
  return result;
}

double solveEta(const Vec2 & Phi_S, const Vec2 & Phi_T, double speed) noexcept
{
  const double b = Phi_S.dot(Phi_T);
  const double c = speed * speed - Phi_T.squaredNorm();
  if (!(c > 0.0)) {
    return kNaN;
  }
  const double root = std::sqrt(b * b + c);
  // eta = -b + root; for b > 0 the equal c / (b + root) has no cancellation (math_spec §4.3).
  return b > 0.0 ? c / (b + root) : root - b;
}

// -----------------------------------------------------------------------------------------------
// DistanceField.

DistanceField::DistanceField(
  const GainFunction & gain, double v_r, int direction, double assumption3_margin) noexcept
: gain_(gain), v_r_(v_r), direction_(direction < 0 ? -1 : 1), eps_v_(assumption3_margin)
{
}

void DistanceField::evaluate(
  const Vec2 & x_eval, const ClosestPoint & cp, const Vec2 & v_c,
  FieldSample * sample) const noexcept
{
  if (sample == nullptr) {
    return;
  }
  FieldSample & s = *sample;
  s.v_c = v_c;
  const int check = fillClosestPoint(x_eval, cp, &s);
  if (check != 0 || !isFinite(v_c)) {
    invalidate(&s, check != 1);
    return;
  }
  s.T_d = static_cast<double>(direction_) * s.T;
  s.G = gain_.G(s.D);
  s.H = gain_.H(s.D);
  s.Phi_S = -s.G * s.n + s.H * s.T_d;

  const ClampedFeedforward clamped = clampFeedforward(feedforward(s.T, v_c), v_r_, eps_v_);
  s.Phi_T = clamped.Phi_T;
  if (clamped.clamped) {
    s.status |= Status::kAssumption3Clamped;
  }
  const bool time_varying = !isZero(s.Phi_T);
  const double eta = time_varying ? solveEta(s.Phi_S, s.Phi_T, v_r_) : v_r_;
  // The unshaped sample is what shape(1, 1, false) would produce: the eta branch iff the
  // feedforward is below the margin.
  const bool uniform =
    !time_varying || !(s.Phi_T.norm() < (1.0 - eps_v_) * v_r_ * (1.0 - kMarginRelTol));
  setUnshaped(&s, eta, uniform);
  s.valid = fieldIsFinite(s);
  if (!s.valid) {
    invalidate(&s, true);
  }
}

void DistanceField::evaluateTerminal(
  const Vec2 & x_eval, const ClosestPoint & cp, double sigma, const TerminalProfile & profile,
  FieldSample * sample) const noexcept
{
  if (sample == nullptr) {
    return;
  }
  FieldSample & s = *sample;
  s.v_c = Vec2::Zero();
  s.terminal = true;
  const int check = fillClosestPoint(x_eval, cp, &s);
  const double phi = profile(sigma);
  if (check != 0 || std::isnan(phi)) {
    invalidate(&s, check != 1);
    return;
  }
  s.T_d = static_cast<double>(direction_) * s.T;
  s.G = gain_.G(s.D);
  s.H = gain_.H(s.D);
  s.phi = phi;
  s.Phi_S = -s.G * s.n + (s.H * s.phi) * s.T_d;
  s.Phi_T = Vec2::Zero();
  setUnshaped(&s, v_r_, true);
  s.valid = fieldIsFinite(s);
  if (!s.valid) {
    invalidate(&s, true);
  }
}

void DistanceField::shape(
  double gamma, double gamma_lim, bool force_uniform, FieldSample * sample) const noexcept
{
  if (sample == nullptr) {
    return;
  }
  FieldSample & s = *sample;
  if (std::isnan(gamma)) {
    s.gamma = gamma;
    invalidate(&s, true);
    return;
  }
  const double g = std::max(0.0, gamma);
  if (!s.valid) {
    s.gamma = g;
    return;
  }

  // The nominal feedforward and scale, recomputed from (T, v_c) exactly as evaluate() did: the
  // sample's Phi_T and eta hold the *shaped* values after a first shape(), and recomputing makes
  // shape() idempotent on one evaluated sample (e.g. the Tracker's re-run with the cap). The
  // terminal field has v_c = 0, hence Phi_T = 0.
  const Vec2 Phi_T = clampFeedforward(feedforward(s.T, s.v_c), v_r_, eps_v_).Phi_T;
  const bool time_varying = !isZero(Phi_T);
  const double p_T = Phi_T.norm();
  const double margin = 1.0 - eps_v_;

  // kAssumption3Clamped iff ||Phi_T|| >= (1 - eps_v) v_r min(1, gamma_lim): the curve cannot be
  // tracked at the externally limited speed. Never because of the collision or clearance terms.
  const double limited = std::min(1.0, gamma_lim);
  if (time_varying && p_T >= margin * v_r_ * limited * (1.0 - kMarginRelTol)) {
    s.status |= Status::kAssumption3Clamped;
  }

  const bool eta_branch =
    time_varying && !force_uniform && p_T < margin * g * v_r_ * (1.0 - kMarginRelTol);
  if (eta_branch) {
    // Re-solve eta at gamma v_r and keep Phi_T (math_spec §8.2): ||Phi_gamma|| = gamma v_r.
    const double eta1 = solveEta(s.Phi_S, Phi_T, g * v_r_);
    s.eta = eta1;
    s.Phi_T = Phi_T;
    s.Phi_gamma = eta1 * s.Phi_S + Phi_T;
    s.uniform_shaping = false;
  } else {
    // Static or terminal field, contour blend, or the safety limit: scale the whole field.
    const double eta_nominal = time_varying ? solveEta(s.Phi_S, Phi_T, v_r_) : v_r_;
    s.eta = g * eta_nominal;
    s.Phi_T = g * Phi_T;
    s.Phi_gamma = g * s.Phi;
    s.uniform_shaping = true;
  }
  s.gamma = g;
  s.conv = -s.eta * s.G * s.n;
  s.circ = s.eta * (s.Phi_S + s.G * s.n);
  s.F = s.Phi_gamma;
  if (!fieldIsFinite(s)) {
    invalidate(&s, true);
  }
}

// -----------------------------------------------------------------------------------------------
// Streamlines (math_spec Appendix A).

namespace
{

/// @brief Unit direction of the field at x; false when \f$\lVert F\rVert\f$ is zero or not
/// finite.
bool unitField(FunctionRef<Vec2(const Vec2 &)> field, const Vec2 & x, Vec2 * direction) noexcept
{
  const Vec2 f = field(x);
  const double norm = f.norm();
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return false;
  }
  *direction = f / norm;
  return true;
}

}  // namespace

std::size_t streamline(
  FunctionRef<Vec2(const Vec2 &)> field, const Vec2 & start, double step, Span<Vec2> out) noexcept
{
  if (out.empty() || !field || !isFinite(start)) {
    return 0;
  }
  out[0] = start;
  std::size_t count = 1;
  if (!std::isfinite(step) || step == 0.0) {
    return count;
  }
  Vec2 x = start;
  while (count < out.size()) {
    Vec2 k1;
    Vec2 k2;
    Vec2 k3;
    Vec2 k4;
    if (
      !unitField(field, x, &k1) || !unitField(field, x + (0.5 * step) * k1, &k2) ||
      !unitField(field, x + (0.5 * step) * k2, &k3) || !unitField(field, x + step * k3, &k4)) {
      break;
    }
    x += (step / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    if (!isFinite(x)) {
      break;
    }
    out[count++] = x;
  }
  return count;
}

}  // namespace vectorfield_core
