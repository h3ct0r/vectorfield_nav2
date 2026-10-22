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

// DistanceField3 and the terminal field in R³ (math_spec §4, §5.1, §14.3).

#include "vectorfield_core/d3/field.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace vectorfield_core::d3
{

namespace
{

// Copies the closest point into the sample; false (and the sample invalid) if unusable.
bool fill(const Vec3 & x, const ClosestPoint3 & cp, FieldSample3 * s) noexcept
{
  s->x_eval = x;
  s->status = cp.status_bits;
  if (
    !cp.valid || !isFinite(x) || !isFinite(cp.x_star) || !isFinite(cp.T) || !isFinite(cp.n) ||
    !std::isfinite(cp.D)) {
    s->status |= Status::kNonFinite;
    s->valid = false;
    s->Phi = Vec3::Zero();
    return false;
  }
  s->x_star = cp.x_star;
  s->T = cp.T;
  s->n = cp.n;
  s->D = cp.D;
  s->s_star = cp.s_star;
  return true;
}

void finalize(FieldSample3 * s) noexcept
{
  s->valid = isFinite(s->Phi) && isFinite(s->Phi_S) && std::isfinite(s->eta);
  if (!s->valid) {
    s->status |= Status::kNonFinite;
    s->Phi = Vec3::Zero();
  }
}

bool unitField(FunctionRef<Vec3(const Vec3 &)> field, const Vec3 & x, Vec3 * out) noexcept
{
  const Vec3 f = field(x);
  const double norm = f.norm();
  if (!isFinite(f) || !(norm > 0.0)) {
    return false;
  }
  *out = f / norm;
  return true;
}

}  // namespace

ClampedFeedforward3 clampFeedforward(
  const Vec3 & Phi_T, double v_r, double assumption3_margin) noexcept
{
  ClampedFeedforward3 result;
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

double solveEta(const Vec3 & Phi_S, const Vec3 & Phi_T, double speed) noexcept
{
  const double b = Phi_S.dot(Phi_T);
  const double c = speed * speed - Phi_T.squaredNorm();
  if (!(c > 0.0)) {
    return kNaN;
  }
  const double root = std::sqrt(b * b + c);
  return b > 0.0 ? c / (b + root) : root - b;
}

DistanceField3::DistanceField3(
  const GainFunction & gain, double v_r, int direction, double assumption3_margin) noexcept
: gain_(gain), v_r_(v_r), direction_(direction < 0 ? -1 : 1), eps_v_(assumption3_margin)
{
}

void DistanceField3::evaluate(
  const Vec3 & x, const ClosestPoint3 & cp, const Vec3 & v_c, FieldSample3 * sample) const noexcept
{
  if (sample == nullptr) {
    return;
  }
  FieldSample3 & s = *sample;
  s.terminal = false;
  s.sigma = kNaN;
  s.phi = 1.0;
  s.v_c = v_c;
  if (!fill(x, cp, &s) || !isFinite(v_c)) {
    s.status |= Status::kNonFinite;
    s.valid = false;
    s.Phi = Vec3::Zero();
    return;
  }
  s.T_d = static_cast<double>(direction_) * s.T;
  s.G = gain_.G(s.D);
  s.H = gain_.H(s.D);
  s.Phi_S = -s.G * s.n + s.H * s.T_d;
  const ClampedFeedforward3 clamped = clampFeedforward(feedforward(s.T, v_c), v_r_, eps_v_);
  s.Phi_T = clamped.Phi_T;
  if (clamped.clamped) {
    s.status |= Status::kAssumption3Clamped;
  }
  const bool time_varying = s.Phi_T.squaredNorm() > 0.0;
  s.eta = time_varying ? solveEta(s.Phi_S, s.Phi_T, v_r_) : v_r_;
  s.Phi = s.eta * s.Phi_S + s.Phi_T;
  finalize(&s);
}

void DistanceField3::evaluateTerminal(
  const Vec3 & x, const ClosestPoint3 & cp, double sigma, const TerminalProfile & profile,
  FieldSample3 * sample) const noexcept
{
  if (sample == nullptr) {
    return;
  }
  FieldSample3 & s = *sample;
  s.terminal = true;
  s.v_c = Vec3::Zero();
  s.sigma = sigma;
  const double phi = profile(sigma);
  if (!fill(x, cp, &s) || std::isnan(phi)) {
    s.status |= Status::kNonFinite;
    s.valid = false;
    s.Phi = Vec3::Zero();
    return;
  }
  s.T_d = static_cast<double>(direction_) * s.T;
  s.G = gain_.G(s.D);
  s.H = gain_.H(s.D);
  s.phi = phi;
  s.Phi_S = -s.G * s.n + (s.H * phi) * s.T_d;
  s.Phi_T = Vec3::Zero();
  s.eta = v_r_;
  s.Phi = v_r_ * s.Phi_S;
  finalize(&s);
}

void DistanceField3::evaluateTerminal(
  const Vec3 & x, const ClosestPoint3 & cp, double sigma, const TerminalProfile & profile,
  const Vec3 & v_transport, FieldSample3 * sample) const noexcept
{
  evaluateTerminal(x, cp, sigma, profile, sample);
  if (sample == nullptr || !sample->valid) {
    return;
  }
  FieldSample3 & s = *sample;
  s.v_c = v_transport;
  if (!isFinite(v_transport)) {
    s.status |= Status::kNonFinite;
    s.valid = false;
    s.Phi = Vec3::Zero();
    return;
  }
  const ClampedFeedforward3 clamped = clampFeedforward(v_transport, v_r_, eps_v_);
  s.Phi_T = clamped.Phi_T;
  if (clamped.clamped) {
    s.status |= Status::kAssumption3Clamped;
  }
  s.Phi = v_r_ * s.Phi_S + s.Phi_T;
  finalize(&s);
}

Vec3 DistanceField3::at(const Curve3 & curve, const Vec3 & x, double t) const noexcept
{
  EvalContext3 context;
  context.t = t;
  context.direction = direction_;
  const ClosestPoint3 cp = curve.closestPoint(x, context, CurveHint3{});
  FieldSample3 sample;
  evaluate(x, cp, curve.velocity(cp, t), &sample);
  return sample.Phi;
}

std::size_t streamline(
  FunctionRef<Vec3(const Vec3 &)> field, const Vec3 & start, double step, Span<Vec3> out) noexcept
{
  if (out.empty() || !field || !isFinite(start)) {
    return 0;
  }
  out[0] = start;
  std::size_t count = 1;
  if (!std::isfinite(step) || step == 0.0) {
    return count;
  }
  Vec3 x = start;
  while (count < out.size()) {
    Vec3 k1;
    Vec3 k2;
    Vec3 k3;
    Vec3 k4;
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

}  // namespace vectorfield_core::d3
