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

// ParametricCurve3: the two-stage closest-point search of math_spec §2.5 in R³, the Newton polish
// of §14.1, arc length, and the regularity and closure checks of §2.6.

#include "vectorfield_core/d3/parametric_curve.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/parametric_curve.hpp"

namespace vectorfield_core::d3
{

namespace
{

constexpr std::array<double, kGaussLegendrePoints> kGaussNodes{
  -0.906179845938663992797627, -0.538469310105683091036314, 0.0, 0.538469310105683091036314,
  0.906179845938663992797627};
constexpr std::array<double, kGaussLegendrePoints> kGaussWeights{
  0.236926885056189087514264, 0.478628670499366468041292, 0.568888888888888888888889,
  0.478628670499366468041292, 0.236926885056189087514264};
constexpr double kInvGoldenRatio = 0.618033988749894848204587;
// Closure: |r(u_f) - r(u_i)| <= max(1e-9, 1e-6 L) (the planar rule).
constexpr double kClosureAbsoluteGap = 1e-9;
constexpr double kClosureRelativeGap = 1e-6;
constexpr double kTurnSlack = 1e-9;
// Newton polish: accepted while D grows by at most this much (m).
constexpr double kPolishSlack = 1e-12;

std::string fmt(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(10) << value;
  return stream.str();
}

[[noreturn]] void reject(const std::string & message)
{
  throw CurveError(CurveError::Code::kInvalidCurve, message);
}

double angleBetween(const Vec3 & a, const Vec3 & b) noexcept
{
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

}  // namespace

struct ParametricCurve3::Candidate
{
  // Parameter in the domain (wrapped when closed; the domain end on a ray).
  double u{0.0};
  // Unwrapped parameter of the search bracket (polish bounds).
  double u_raw{0.0};
  // Ray coordinate (0 off the rays); ray = -1 backward, +1 forward, 0 none.
  double tau{0.0};
  int ray{0};
  Vec3 foot{Vec3::Zero()};
  double D{kInf};
  bool valid{false};
};

ParametricCurve3::ParametricCurve3(
  double u_min, double u_max, bool closed, bool time_varying, const CurveOptions & options)
: u_min_(u_min), u_max_(u_max), closed_(closed), time_varying_(time_varying), options_(options)
{
  if (!std::isfinite(u_min) || !std::isfinite(u_max) || !(u_min < u_max)) {
    throw ConfigError(
      "ParametricCurve3: the domain [" + fmt(u_min) + ", " + fmt(u_max) +
      "] must be finite and non-empty");
  }
  const ValidationReport report = validateCurveOptions(options);
  if (!report.ok()) {
    throw ConfigError("ParametricCurve3: " + report.summary());
  }
}

ParametricCurve3::~ParametricCurve3() = default;

void ParametricCurve3::setSpec(std::shared_ptr<const CurveSpec3> spec) noexcept
{
  spec_ = std::move(spec);
}

const CurveSpec3 * ParametricCurve3::spec() const noexcept { return spec_.get(); }

double ParametricCurve3::wrapU(double u) const noexcept
{
  if (!closed_ || !std::isfinite(u)) {
    return u;
  }
  const double period = u_max_ - u_min_;
  double v = std::fmod(u - u_min_, period);
  if (v < 0.0) {
    v += period;
  }
  return u_min_ + (v < period ? v : 0.0);
}

double ParametricCurve3::clampU(double u) const noexcept
{
  return closed_ ? wrapU(u) : std::min(u_max_, std::max(u_min_, u));
}

Vec3 ParametricCurve3::drdu(double u, double t) const noexcept
{
  const double h = kParamDerivativeStepFraction * (u_max_ - u_min_);
  if (!closed_ && u - h < u_min_) {
    return (-3.0 * r(u, t) + 4.0 * r(u + h, t) - r(u + 2.0 * h, t)) / (2.0 * h);
  }
  if (!closed_ && u + h > u_max_) {
    return (3.0 * r(u, t) - 4.0 * r(u - h, t) + r(u - 2.0 * h, t)) / (2.0 * h);
  }
  return (r(u + h, t) - r(u - h, t)) / (2.0 * h);
}

Vec3 ParametricCurve3::drdt(double u, double t) const noexcept
{
  if (!time_varying_) {
    return Vec3::Zero();
  }
  return (r(u, t + kTimeDerivativeStep) - r(u, t - kTimeDerivativeStep)) /
         (2.0 * kTimeDerivativeStep);
}

// ---------------------------------------------------------------------------------------------
// Arc length.

double ParametricCurve3::panelIntegral(double a, double b, double t) const noexcept
{
  if (!std::isfinite(a) || !std::isfinite(b)) {
    return kNaN;
  }
  if (a == b) {
    return 0.0;
  }
  const double period = u_max_ - u_min_;
  const double panels_real =
    std::ceil(static_cast<double>(kArcLengthPanelsPerDomain) * std::abs(b - a) / period);
  const int panels = static_cast<int>(std::min(std::max(panels_real, 1.0), 1e6));
  const double width = (b - a) / panels;
  double sum = 0.0;
  for (int p = 0; p < panels; ++p) {
    const double p0 = a + width * p;
    const double half = 0.5 * width;
    const double mid = p0 + half;
    double panel = 0.0;
    for (int j = 0; j < kGaussLegendrePoints; ++j) {
      const auto index = static_cast<std::size_t>(j);
      panel += kGaussWeights[index] * drdu(clampU(mid + half * kGaussNodes[index]), t).norm();
    }
    sum += half * panel;
  }
  return sum;
}

double ParametricCurve3::arcLength(double u0, double u1, double t) const noexcept
{
  if (
    !time_varying_ && !table_u_.empty() && u0 >= u_min_ && u0 <= u_max_ && u1 >= u_min_ &&
    u1 <= u_max_) {
    return stationAt(u1, t) - stationAt(u0, t);
  }
  return panelIntegral(u0, u1, t);
}

double ParametricCurve3::stationAt(double u, double t) const noexcept
{
  if (!std::isfinite(u)) {
    return kNaN;
  }
  u = clampU(u);
  if (time_varying_ || table_u_.empty()) {
    return panelIntegral(u_min_, u, t);
  }
  const double du = table_u_[1] - table_u_[0];
  const auto last = static_cast<std::ptrdiff_t>(table_u_.size()) - 2;
  const auto k = std::min(
    std::max(static_cast<std::ptrdiff_t>(std::floor((u - u_min_) / du)), std::ptrdiff_t{0}), last);
  const auto index = static_cast<std::size_t>(k);
  return table_s_[index] + panelIntegral(table_u_[index], u, t);
}

double ParametricCurve3::length(double t) const noexcept
{
  if (time_varying_) {
    return panelIntegral(u_min_, u_max_, t);
  }
  return length_;
}

double ParametricCurve3::parameterAt(double s, double t) const noexcept
{
  if (!std::isfinite(s)) {
    return kNaN;
  }
  const double L = length(t);
  if (closed_) {
    s = std::fmod(s, L);
    if (s < 0.0) {
      s += L;
    }
  } else {
    s = std::min(std::max(s, 0.0), L);
  }
  double lo = u_min_;
  double hi = u_max_;
  double u = u_min_ + (u_max_ - u_min_) * (L > 0.0 ? s / L : 0.0);
  if (!time_varying_ && !table_s_.empty()) {
    const auto it = std::upper_bound(table_s_.begin(), table_s_.end(), s);
    const auto k = std::min(
      std::max(static_cast<std::ptrdiff_t>(it - table_s_.begin()) - 1, std::ptrdiff_t{0}),
      static_cast<std::ptrdiff_t>(table_s_.size()) - 2);
    const auto index = static_cast<std::size_t>(k);
    lo = table_u_[index];
    hi = table_u_[index + 1];
    const double span = table_s_[index + 1] - table_s_[index];
    u = lo + (hi - lo) * (span > 0.0 ? (s - table_s_[index]) / span : 0.0);
  }
  // Safeguarded Newton on s(u) = s within [lo, hi].
  for (int iteration = 0; iteration < 30; ++iteration) {
    const double error = stationAt(u, t) - s;
    if (std::abs(error) <= 1e-12 * std::max(1.0, L)) {
      break;
    }
    if (error > 0.0) {
      hi = u;
    } else {
      lo = u;
    }
    const double speed = drdu(u, t).norm();
    double next = speed > 0.0 ? u - error / speed : 0.5 * (lo + hi);
    if (!(next > lo && next < hi)) {
      next = 0.5 * (lo + hi);
    }
    u = next;
  }
  return u;
}

Vec3 ParametricCurve3::pointAt(double s, double t) const noexcept
{
  if (!std::isfinite(s)) {
    return Vec3::Constant(kNaN);
  }
  if (!closed_) {
    const double L = length(t);
    if (s < 0.0) {
      return r(u_min_, t) + s * unitOr(drdu(u_min_, t), Vec3::UnitX());
    }
    if (s > L) {
      return r(u_max_, t) + (s - L) * unitOr(drdu(u_max_, t), Vec3::UnitX());
    }
  }
  return r(parameterAt(s, t), t);
}

Vec3 ParametricCurve3::tangentAt(double s, double t) const noexcept
{
  if (!std::isfinite(s)) {
    return Vec3::Constant(kNaN);
  }
  if (!closed_) {
    if (s <= 0.0) {
      return unitOr(drdu(u_min_, t), Vec3::UnitX());
    }
    if (s >= length(t)) {
      return unitOr(drdu(u_max_, t), Vec3::UnitX());
    }
  }
  return unitOr(drdu(parameterAt(s, t), t), Vec3::UnitX());
}

Vec3 ParametricCurve3::velocity(const ClosestPoint3 & cp, double t) const noexcept
{
  if (!time_varying_ || cp.on_ray || !cp.valid) {
    return Vec3::Zero();
  }
  return drdt(cp.u_star, t);
}

// ---------------------------------------------------------------------------------------------
// Search.

ParametricCurve3::Candidate ParametricCurve3::golden(
  const Vec3 & x, double a, double b, double t) const noexcept
{
  const double tolerance = kGoldenSectionTolerance * std::max(1.0, u_max_ - u_min_);
  const auto f = [&](double u) { return (r(clampU(u), t) - x).squaredNorm(); };
  double c = b - kInvGoldenRatio * (b - a);
  double d = a + kInvGoldenRatio * (b - a);
  double fc = f(c);
  double fd = f(d);
  for (int iteration = 0; iteration < kGoldenSectionMaxIterations && (b - a) > tolerance;
       ++iteration) {
    if (fc < fd) {
      b = d;
      d = c;
      fd = fc;
      c = b - kInvGoldenRatio * (b - a);
      fc = f(c);
    } else {
      a = c;
      c = d;
      fc = fd;
      d = a + kInvGoldenRatio * (b - a);
      fd = f(d);
    }
  }
  // The bracket ends are candidates too (a minimum on the bracket edge).
  double best = 0.5 * (a + b);
  double f_best = f(best);
  for (const double u : {a, b}) {
    const double fu = f(u);
    if (fu < f_best) {
      f_best = fu;
      best = u;
    }
  }
  Candidate out;
  out.u_raw = best;
  out.u = clampU(best);
  out.foot = r(out.u, t);
  out.D = (x - out.foot).norm();
  out.valid = std::isfinite(out.D);
  return out;
}

ParametricCurve3::Candidate ParametricCurve3::withRays(
  const Vec3 & x, const Candidate & c, double t) const noexcept
{
  if (closed_) {
    return c;
  }
  Candidate best = c;
  for (const int ray : {-1, 1}) {
    const double u_end = ray < 0 ? u_min_ : u_max_;
    const Vec3 anchor = r(u_end, t);
    const Vec3 T = unitOr(drdu(u_end, t), Vec3::Zero());
    const double tau_hat = (x - anchor).dot(T);
    if (ray * tau_hat <= 0.0) {
      continue;
    }
    Candidate candidate;
    candidate.ray = ray;
    candidate.tau = tau_hat;
    candidate.u = u_end;
    candidate.u_raw = u_end;
    candidate.foot = anchor + tau_hat * T;
    candidate.D = (x - candidate.foot).norm();
    candidate.valid = std::isfinite(candidate.D);
    if (candidate.valid && (!best.valid || candidate.D < best.D)) {
      best = candidate;
    }
  }
  return best;
}

ParametricCurve3::Candidate ParametricCurve3::globalStage(
  const Vec3 & x, const EvalContext3 & context) const noexcept
{
  const int count = options_.global_search_samples;
  const double period = u_max_ - u_min_;
  const double du = closed_ ? period / count : period / (count - 1);
  const double dir = context.direction < 0 ? -1.0 : 1.0;
  int k_best = -1;
  double d_best = kInf;
  for (int k = 0; k < count; ++k) {
    const double u = !closed_ && k == count - 1 ? u_max_ : u_min_ + du * k;
    const Vec3 p = samples_.empty() ? r(u, context.t) : samples_[static_cast<std::size_t>(k)];
    const double d = (p - x).norm();
    // Ties toward the start of traversal (smallest dir·u).
    if (
      k_best < 0 || d < d_best - kTieTolerance ||
      (isTie(d, d_best) && dir * u < dir * (u_min_ + du * k_best))) {
      k_best = k;
      d_best = d;
    }
  }
  if (k_best < 0 || !std::isfinite(d_best)) {
    return Candidate{};
  }
  double a = u_min_ + du * (k_best - 1);
  double b = u_min_ + du * (k_best + 1);
  if (!closed_) {
    a = std::max(a, u_min_);
    b = std::min(b, u_max_);
  }
  Candidate c = golden(x, a, b, context.t);
  polish(x, a, b, context.t, &c);
  return withRays(x, c, context.t);
}

ParametricCurve3::Candidate ParametricCurve3::localStage(
  const Vec3 & x, double u_prev, double t, bool * on_edge) const noexcept
{
  *on_edge = false;
  const double period = u_max_ - u_min_;
  const double speed = drdu(clampU(u_prev), t).norm();
  double half = speed > 0.0 ? 0.5 * options_.search_window / speed : 0.5 * period;
  half = std::min(half, 0.5 * period);
  const double tolerance = 1e-6 * half;
  double center = u_prev;
  double shifted = 0.0;
  Candidate c;
  for (;;) {
    double a = center - half;
    double b = center + half;
    if (!closed_) {
      a = std::max(a, u_min_);
      b = std::min(b, u_max_);
    }
    c = golden(x, a, b, t);
    polish(x, a, b, t, &c);
    const bool low_edge = c.u_raw - a <= tolerance && (closed_ || a > u_min_);
    const bool high_edge = b - c.u_raw <= tolerance && (closed_ || b < u_max_);
    if (!(low_edge || high_edge) || !c.valid) {
      break;
    }
    if (shifted >= period) {
      *on_edge = true;
      break;
    }
    center += low_edge ? -half : half;
    shifted += half;
  }
  if (!closed_ && (c.u <= u_min_ || c.u >= u_max_)) {
    c = withRays(x, c, t);
  }
  return c;
}

void ParametricCurve3::polish(
  const Vec3 & x, double lo, double hi, double t, Candidate * c) const noexcept
{
  if (!c->valid || c->ray != 0) {
    return;
  }
  if (!closed_) {
    lo = std::max(lo, u_min_);
    hi = std::min(hi, u_max_);
  }
  const double h = kParamDerivativeStepFraction * (u_max_ - u_min_);
  double u = c->u_raw;
  for (int step = 0; step < kNewtonPolishSteps; ++step) {
    const double uc = clampU(u);
    const Vec3 p = r(uc, t);
    const Vec3 ru = drdu(uc, t);
    const Vec3 w = p - x;
    const double g = ru.dot(w);
    const double up = closed_ ? u + h : std::min(u + h, u_max_);
    const double dn = closed_ ? u - h : std::max(u - h, u_min_);
    if (!(up > dn)) {
      return;
    }
    const Vec3 ruu = (drdu(clampU(up), t) - drdu(clampU(dn), t)) / (up - dn);
    const double gp = ru.squaredNorm() + ruu.dot(w);
    if (!(gp > 0.0) || !std::isfinite(g)) {
      return;
    }
    const double next = u - g / gp;
    if (!(next >= lo && next <= hi) || next == u) {
      return;
    }
    const Vec3 foot = r(clampU(next), t);
    const double D = (x - foot).norm();
    // "Does not increase" up to rounding: near the root D changes by O(Δu²), far below the
    // rounding of D itself, so a strict comparison would reject the polish at random.
    if (!(D <= c->D + kPolishSlack)) {
      return;
    }
    u = next;
    c->u_raw = next;
    c->u = clampU(next);
    c->foot = foot;
    c->D = D;
  }
}

ClosestPoint3 ParametricCurve3::closestPoint(
  const Vec3 & x, const EvalContext3 & context, const CurveHint3 & hint) const noexcept
{
  if (!isFinite(x) || !std::isfinite(context.t)) {
    return ClosestPoint3{};
  }
  const double t = context.t;
  if (!hint.valid || !std::isfinite(hint.s)) {
    return finish(x, globalStage(x, context), t);
  }
  bool on_edge = false;
  const Candidate local = localStage(x, hint.s, t, &on_edge);
  if (!local.valid) {
    return finish(x, globalStage(x, context), t);
  }
  if (!on_edge && !(local.D > options_.global_search_trigger)) {
    return finish(x, local, t);
  }
  const Candidate global = globalStage(x, context);
  if (!global.valid) {
    return finish(x, local, t);
  }
  bool adopt = false;
  if (on_edge) {
    adopt = global.D < local.D - kTieTolerance;
  } else {
    const double s_loc = local.ray == 0 ? stationAt(local.u, t) : 0.0;
    const double s_glob = global.ray == 0 ? stationAt(global.u, t) : 0.0;
    const double arc =
      local.ray == 0 && global.ray == 0 ? std::abs(wrapStationDifference(s_glob - s_loc, t)) : kInf;
    const double chord = (global.foot - local.foot).norm();
    adopt = adoptGlobalCandidate(local.D, global.D, arc, chord, options_.global_search_hysteresis);
  }
  ClosestPoint3 cp = finish(x, adopt ? global : local, t);
  if (adopt) {
    cp.status_bits |= Status::kAmbiguousClosest;
  }
  return cp;
}

ClosestPoint3 ParametricCurve3::finish(const Vec3 & x, const Candidate & c, double t) const noexcept
{
  ClosestPoint3 cp;
  if (!c.valid) {
    cp.status_bits |= Status::kNonFinite;
    return cp;
  }
  const Vec3 ru = drdu(c.u, t);
  const double speed = ru.norm();
  if (!std::isfinite(speed) || !isFinite(c.foot)) {
    cp.status_bits |= Status::kNonFinite;
    return cp;
  }
  if (speed < speed_floor_) {
    cp.status_bits |= Status::kNonFinite;
    return cp;
  }
  cp.valid = true;
  cp.T = ru / speed;
  cp.x_star = c.foot;
  cp.D = c.D;
  cp.u_star = c.u;
  cp.segment = -1;
  if (c.ray != 0) {
    cp.on_ray = true;
    cp.piece = c.ray < 0 ? PieceKind::kBackwardRay : PieceKind::kForwardRay;
    cp.s_star = c.ray < 0 ? c.tau : length(t) + c.tau;
  } else {
    cp.piece = PieceKind::kParametric;
    cp.s_star = stationAt(c.u, t);
    if (closed_) {
      const double L = length(t);
      if (cp.s_star >= L) {
        cp.s_star -= L;
      }
    }
  }
  if (c.D < kEpsD) {
    cp.n = Vec3::Zero();
  } else {
    cp.n = unitOr(rejectFrom(x - c.foot, cp.T), Vec3::Zero());
  }
  return cp;
}

// ---------------------------------------------------------------------------------------------
// Initialisation (math_spec §2.6).

void ParametricCurve3::initialize(double t0)
{
  if (!std::isfinite(t0)) {
    reject("t0 = " + fmt(t0) + " must be finite");
  }
  const int count = options_.global_search_samples;
  const double period = u_max_ - u_min_;
  const double du_samples = closed_ ? period / count : period / (count - 1);
  std::vector<Vec3> points(static_cast<std::size_t>(count));
  std::vector<Vec3> derivatives(static_cast<std::size_t>(count));
  for (int k = 0; k < count; ++k) {
    const auto index = static_cast<std::size_t>(k);
    const double u = !closed_ && k == count - 1 ? u_max_ : u_min_ + du_samples * k;
    points[index] = r(u, t0);
    derivatives[index] = drdu(u, t0);
    if (!isFinite(points[index]) || !isFinite(derivatives[index])) {
      reject("the curve is not finite at s = " + fmt(u));
    }
  }

  // u↔s table: M + 1 nodes, 5-point Gauss–Legendre per interval.
  std::vector<double> table_u(static_cast<std::size_t>(count) + 1);
  std::vector<double> table_s(static_cast<std::size_t>(count) + 1);
  const double du_table = period / count;
  table_u[0] = u_min_;
  table_s[0] = 0.0;
  for (int k = 0; k < count; ++k) {
    const auto index = static_cast<std::size_t>(k);
    table_u[index + 1] = k + 1 == count ? u_max_ : u_min_ + du_table * (k + 1);
    table_s[index + 1] = table_s[index] + panelIntegral(table_u[index], table_u[index + 1], t0);
  }
  const double total = table_s.back();
  if (!std::isfinite(total) || !(total > 0.0)) {
    reject("the curve length " + fmt(total) + " is not positive and finite");
  }

  if (closed_) {
    const double h = kParamDerivativeStepFraction * period;
    const Vec3 start = r(u_min_, t0);
    const Vec3 end = r(u_max_, t0);
    const double gap = (end - start).norm();
    const double allowed = std::max(kClosureAbsoluteGap, kClosureRelativeGap * total);
    if (!(gap <= allowed)) {
      reject(
        "the curve is marked closed but does not close: |r(s_max) - r(s_min)| = " + fmt(gap) +
        " > " + fmt(allowed));
    }
    const Vec3 d_start =
      (-3.0 * start + 4.0 * r(u_min_ + h, t0) - r(u_min_ + 2.0 * h, t0)) / (2.0 * h);
    const Vec3 d_end = (3.0 * end - 4.0 * r(u_max_ - h, t0) + r(u_max_ - 2.0 * h, t0)) / (2.0 * h);
    const double angle = angleBetween(d_start, d_end);
    if (!(angle <= kClosureMaxTangentAngle)) {
      reject(
        "the curve is marked closed but its end tangents differ by " + fmt(angle * 180.0 / kPi) +
        " deg (> 1 deg)");
    }
  }

  double sum = 0.0;
  double min_speed = kInf;
  for (const Vec3 & d : derivatives) {
    sum += d.norm();
    min_speed = std::min(min_speed, d.norm());
  }
  const double mean = sum / count;
  if (!(mean > 0.0) || !(min_speed >= kRegularitySpeedRatio * mean)) {
    reject(
      "the curve is not regular: min |dr/ds| = " + fmt(min_speed) + " is below 1e-3 x the mean " +
      fmt(mean) + " (stationary point)");
  }
  const std::size_t pairs = closed_ ? derivatives.size() : derivatives.size() - 1;
  for (std::size_t k = 0; k < pairs; ++k) {
    const double angle = angleBetween(derivatives[k], derivatives[(k + 1) % derivatives.size()]);
    if (angle > kRegularityMaxTurn + kTurnSlack) {
      reject(
        "the curve turns by " + fmt(angle * 180.0 / kPi) +
        " deg between adjacent samples (> 90 deg): a cusp, or too few global_search_samples");
    }
  }

  speed_floor_ = kRegularitySpeedRatio * mean;
  length_ = total;
  if (!time_varying_) {
    table_u_ = std::move(table_u);
    table_s_ = std::move(table_s);
    samples_ = std::move(points);
  }
}

}  // namespace vectorfield_core::d3
