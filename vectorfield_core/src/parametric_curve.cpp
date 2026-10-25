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

// ParametricCurve: closest-point search, arc length and regularity of r(u, t)
// (math_spec §2.4–§2.6).
//
// Invariants kept by this file:
// - r() and drdu() are only ever called with a parameter inside the domain: wrapped into
//   [u_i, u_f) for closed curves, clamped to [u_i, u_f] for open ones (math_spec §2.5: "the
//   expression is never evaluated outside [u_i, u_f]").
// - closestPoint() and every other const method allocate nothing and keep no mutable state, so
//   concurrent calls are safe (ExpressionCurve serialises its own parser).

#include "vectorfield_core/parametric_curve.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/status.hpp"

namespace vectorfield_core
{

namespace
{

// 5-point Gauss–Legendre rule on [-1, 1] (math_spec §2.5).
constexpr std::array<double, kGaussLegendrePoints> kGaussNodes{
  -0.906179845938663992797627, -0.538469310105683091036314, 0.0, 0.538469310105683091036314,
  0.906179845938663992797627};
constexpr std::array<double, kGaussLegendrePoints> kGaussWeights{
  0.236926885056189087514264, 0.478628670499366468041292, 0.568888888888888888888889,
  0.478628670499366468041292, 0.236926885056189087514264};

// 1/φ for the golden-section search.
constexpr double kInvGoldenRatio = 0.618033988749894848204587;

// Polish of a golden-section minimiser (see Search::polish): the secant iteration on the
// stationarity condition stays within ±kPolishTrustFraction·(u_f − u_i) of the golden-section
// result and starts with a step of kPolishStepFraction·(u_f − u_i).
constexpr double kPolishTrustFraction = 1e-5;
constexpr double kPolishStepFraction = 1e-7;
constexpr int kPolishMaxIterations = 8;

// u(s) inversions (Newton with a bisection safeguard): at most kInverseMaxIterations, stopping
// at |s(u) − s| <= kInverseTolerance·max(1, L). The tolerance sits above the rounding noise of
// arc lengths from numerically differentiated expressions (about 1e-12 relative).
constexpr int kInverseMaxIterations = 30;
constexpr double kInverseTolerance = 1e-12;
constexpr double kStepEpsilon = 4.0 * 2.220446049250313e-16;

// Closure tolerances of math_spec §2.6 rule 3.
constexpr double kClosureAbsoluteGap = 1e-9;
// Rounding slack on the "more than 90°" regularity rule, so exactly 90° (four samples on a
// circle, the smallest M allowed) passes as it would in exact arithmetic.
constexpr double kTurnSlack = 1e-9;
constexpr double kClosureRelativeGap = 1e-6;

std::string fmt(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(6) << value;
  return stream.str();
}

[[noreturn]] void reject(const std::string & message)
{
  throw CurveError(CurveError::Code::kInvalidCurve, message);
}

// Domain [lo, hi] of a parametric curve with the wrap (closed) or clamp (open) rule.
struct Domain
{
  double lo{0.0};
  double hi{1.0};
  double period{1.0};
  bool closed{false};

  explicit Domain(const ParametricCurve & curve) noexcept
  : lo(curve.uMin()),
    hi(curve.uMax()),
    period(curve.uMax() - curve.uMin()),
    closed(curve.isClosed())
  {
  }

  // Wraps into [lo, hi) (closed) or clamps into [lo, hi] (open); NaN stays NaN.
  double map(double u) const noexcept
  {
    if (std::isnan(u)) {
      return u;
    }
    if (closed) {
      double v = std::fmod(u - lo, period);
      if (v < 0.0) {
        v += period;
      }
      if (!(v < period)) {
        v = 0.0;
      }
      return lo + v;
    }
    return std::min(hi, std::max(lo, u));
  }
};

// x − P·floor(x/P), in [0, P).
double positiveMod(double x, double period) noexcept
{
  double v = std::fmod(x, period);
  if (v < 0.0) {
    v += period;
  }
  return v < period ? v : 0.0;
}

// ∫_a^b ‖∂r/∂u(u, t)‖ du by one 5-point Gauss–Legendre panel (r' evaluated in the domain).
double gaussPanel(
  const ParametricCurve & curve, const Domain & domain, double a, double b, double t) noexcept
{
  const double half = 0.5 * (b - a);
  const double mid = 0.5 * (a + b);
  double sum = 0.0;
  for (int j = 0; j < kGaussLegendrePoints; ++j) {
    const double u = domain.map(mid + half * kGaussNodes[static_cast<std::size_t>(j)]);
    const Vec2 d = curve.drdu(u, t);
    sum += kGaussWeights[static_cast<std::size_t>(j)] * std::sqrt(d.x() * d.x() + d.y() * d.y());
  }
  return half * sum;
}

// Fixed-panel Gauss–Legendre: ⌈64·|b − a|/(u_f − u_i)⌉ panels (math_spec §2.5), signed.
double panelIntegral(
  const ParametricCurve & curve, const Domain & domain, double a, double b, double t) noexcept
{
  if (!(std::isfinite(a) && std::isfinite(b))) {
    return kNaN;
  }
  if (a == b) {
    return 0.0;
  }
  const double width = std::abs(b - a);
  const double panels_real =
    std::ceil(static_cast<double>(kArcLengthPanelsPerDomain) * width / domain.period);
  // A request spanning many laps of a closed curve is split into whole laps.
  if (domain.closed && panels_real > 4.0 * kArcLengthPanelsPerDomain) {
    const double sign = b > a ? 1.0 : -1.0;
    const double laps = std::floor(width / domain.period);
    const double lap_length = panelIntegral(curve, domain, domain.lo, domain.hi, t);
    const double rest_start = a + sign * laps * domain.period;
    return sign * laps * lap_length + panelIntegral(curve, domain, rest_start, b, t);
  }
  const int panels = std::max(1, static_cast<int>(panels_real));
  const double h = (b - a) / panels;
  double sum = 0.0;
  for (int i = 0; i < panels; ++i) {
    const double p0 = a + h * i;
    const double p1 = i + 1 == panels ? b : a + h * (i + 1);
    sum += gaussPanel(curve, domain, p0, p1, t);
  }
  return sum;
}

// One candidate foot of the closest-point search.
struct Candidate
{
  // Parameter of the foot; the domain end for a ray.
  double u{0.0};
  // Ray coordinate τ (0 off the rays).
  double tau{0.0};
  Vec2 foot{Vec2::Zero()};
  // End tangent (rays only).
  Vec2 ray_tangent{Vec2::UnitX()};
  double D{kInf};
  PieceKind piece{PieceKind::kNone};
  bool valid{false};
};

// A discrete local minimum of the sampled distance in the global stage.
struct SampleCandidate
{
  int k{-1};
  double D{kInf};
};

// Sample minima refined by the global stage (see Search::global).
constexpr std::size_t kGlobalCandidates = 4;

// Result of one golden-section search.
struct Golden
{
  double u{0.0};
  double f{kInf};
  // The minimum is at (or beyond) the lower / upper bracket edge.
  bool at_lo{false};
  bool at_hi{false};
};

// A point as two scalars (the hot loops avoid Eigen expressions, which are slow unoptimised).
struct XY
{
  double x{0.0};
  double y{0.0};
};

// One closest-point query: the curve, the point, the time and the cached samples (interleaved
// x, y of the M global-stage samples; null for time-varying curves).
class Search
{
public:
  Search(
    const ParametricCurve & curve, const double * samples, const Vec2 & x,
    const EvalContext & context) noexcept
  : curve_(curve),
    domain_(curve),
    samples_(samples),
    x_(x),
    px_(x.x()),
    py_(x.y()),
    t_(context.t),
    direction_(context.direction),
    fallback_(unitOr(context.last_tangent, Vec2::UnitX())),
    tolerance_(kGoldenSectionTolerance * std::max(1.0, domain_.period)),
    sample_count_(curve.options().global_search_samples)
  {
  }

  double mapHint(double u) const noexcept { return domain_.map(u); }

  // Local stage (math_spec §2.4): golden section on [u_prev − Δu_b, u_prev + Δu_f], continued
  // by w/2 at a bracket edge that is not a domain end, at most one lap.
  Candidate local(double u_prev, bool * had_edge) const noexcept
  {
    *had_edge = false;
    double center = u_prev;
    double shifted = 0.0;
    Golden golden_result;
    const int max_iterations = 2 * sample_count_ + 4;
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
      const std::pair<double, double> half_widths = window(center);
      double a = center - half_widths.first;
      double b = center + half_widths.second;
      if (!domain_.closed) {
        a = std::max(a, domain_.lo);
        b = std::min(b, domain_.hi);
      }
      if (!(b > a)) {
        golden_result.u = center;
        golden_result.f = dist2(center);
        golden_result.at_lo = false;
        golden_result.at_hi = false;
        break;
      }
      golden_result = golden(a, b);
      const bool lo_edge = golden_result.at_lo && (domain_.closed || a > domain_.lo);
      const bool hi_edge = golden_result.at_hi && (domain_.closed || b < domain_.hi);
      if (!lo_edge && !hi_edge) {
        break;
      }
      *had_edge = true;
      const double step = std::abs(golden_result.u - center);
      if (!(step > 0.0)) {
        break;
      }
      shifted += step;
      if (domain_.closed && shifted >= domain_.period) {
        break;
      }
      center = golden_result.u;
    }
    return refine(golden_result.u);
  }

  // Global stage (thesis §5.1, math_spec §2.5): M samples plus the rays, argmin, golden section
  // on the neighbouring sample interval. Ties go to the hint (when given) or to the start of
  // traversal.
  //
  // At a self-intersection (lemniscate crossing) samples of both branches can tie, and refining
  // only the argmin may refine the wrong branch. Every discrete local minimum of the sampled
  // distance within half a sample spacing of the best one can hold the true foot (the sample
  // nearest the foot is at most half a spacing away from it), so up to kGlobalCandidates such
  // minima are refined and the tie rule is applied to the refined candidates.
  Candidate global(bool has_hint, double u_hint) const noexcept
  {
    const int count = sample_count_;
    const double du = sampleStep();
    const auto sample = [&](int k, XY * p) {
      if (samples_ != nullptr) {
        p->x = samples_[2 * k];
        p->y = samples_[2 * k + 1];
      } else {
        const Vec2 q = point(sampleParameter(k, du));
        p->x = q.x();
        p->y = q.y();
      }
      const double dx = px_ - p->x;
      const double dy = py_ - p->y;
      const double D = std::sqrt(dx * dx + dy * dy);
      return std::isfinite(D) ? D : kInf;
    };
    // Streaming pass with a three-sample window (no storage): D_{k-1}, D_k, D_{k+1}.
    std::array<SampleCandidate, kGlobalCandidates> kept{};
    std::size_t kept_count = 0;
    const auto distance = [](const XY & a, const XY & b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return std::sqrt(dx * dx + dy * dy);
    };
    XY p_first;
    const double D_first = sample(0, &p_first);
    XY p_last = p_first;
    const double D_last = domain_.closed ? sample(count - 1, &p_last) : kInf;
    double D_prev = domain_.closed ? D_last : kInf;
    double D_cur = D_first;
    XY p_cur = p_first;
    double best_D = kInf;
    double max_chord = domain_.closed ? distance(p_first, p_last) : 0.0;
    if (!std::isfinite(max_chord)) {
      max_chord = 0.0;
    }
    for (int k = 0; k < count; ++k) {
      double D_next = kInf;
      XY p_next = p_cur;
      if (k + 1 < count) {
        if (domain_.closed && k + 1 == count - 1) {
          D_next = D_last;
          p_next = p_last;
        } else {
          D_next = sample(k + 1, &p_next);
        }
        const double chord = distance(p_next, p_cur);
        if (std::isfinite(chord)) {
          max_chord = std::max(max_chord, chord);
        }
      } else if (domain_.closed) {
        D_next = D_first;
      }
      if (std::isfinite(D_cur)) {
        best_D = std::min(best_D, D_cur);
        if (D_cur <= D_prev && D_cur <= D_next) {
          keep(SampleCandidate{k, D_cur}, &kept, &kept_count, du, has_hint, u_hint);
        }
      }
      D_prev = D_cur;
      D_cur = D_next;
      p_cur = p_next;
    }
    Candidate best;
    const double margin = 0.6 * max_chord + kTieTolerance;
    for (std::size_t i = 0; i < kept_count; ++i) {
      const SampleCandidate & entry = kept[i];
      if (!(entry.D <= best_D + margin)) {
        continue;
      }
      double a = sampleParameter(entry.k, du) - du;
      double b = sampleParameter(entry.k, du) + du;
      if (!domain_.closed) {
        a = std::max(a, domain_.lo);
        b = std::min(b, domain_.hi);
      }
      const Candidate refined = refine(golden(a, b).u);
      if (refined.valid && better(refined, best, has_hint, u_hint)) {
        best = refined;
      }
    }
    if (!domain_.closed) {
      for (const bool forward : {false, true}) {
        const Candidate ray = rayCandidate(forward);
        if (ray.valid && better(ray, best, has_hint, u_hint)) {
          best = ray;
        }
      }
    }
    return best;
  }

  // Arc length between two feet (closed curves: the shorter way), for the detour test.
  double arcBetween(const Candidate & a, const Candidate & b) const noexcept
  {
    const double ds = station(b) - station(a);
    if (domain_.closed) {
      return std::abs(wrapPeriodic(ds, curve_.length(t_)));
    }
    return std::abs(ds);
  }

  // The ClosestPoint of a candidate: T, the re-orthogonalised n, D and the stations.
  ClosestPoint finalize(const Candidate & candidate, std::uint32_t status) const noexcept
  {
    ClosestPoint cp;
    cp.status_bits = status;
    if (!candidate.valid) {
      cp.valid = false;
      cp.status_bits |= Status::kNonFinite;
      return cp;
    }
    if (candidate.piece == PieceKind::kParametric) {
      const double u = domain_.map(candidate.u);
      const Vec2 d = curve_.drdu(u, t_);
      const double speed = d.norm();
      cp.x_star = curve_.r(u, t_);
      cp.T = unitOr(d, fallback_);
      if (!(speed >= curve_.speedFloor())) {
        cp.status_bits |= Status::kNonFinite;
      }
      cp.u_star = u;
      cp.s_star = curve_.stationAt(u, t_);
      cp.piece = PieceKind::kParametric;
      cp.on_ray = false;
    } else {
      cp.x_star = candidate.foot;
      cp.T = candidate.ray_tangent;
      cp.u_star = candidate.u;
      cp.s_star = station(candidate);
      cp.piece = candidate.piece;
      cp.on_ray = true;
    }
    cp.segment = -1;
    const Vec2 v = x_ - cp.x_star;
    cp.D = v.norm();
    // Re-orthogonalisation (math_spec §2.5), applied twice so n^T T = 0 to rounding.
    Vec2 w = v - v.dot(cp.T) * cp.T;
    w -= w.dot(cp.T) * cp.T;
    const double w_norm = w.norm();
    cp.n = w_norm < kEpsD ? Vec2::Zero() : Vec2(w / w_norm);
    cp.valid = isFinite(cp.x_star) && isFinite(cp.T) && isFinite(cp.n) && std::isfinite(cp.D) &&
               std::isfinite(cp.s_star);
    if (!cp.valid) {
      cp.status_bits |= Status::kNonFinite;
    }
    return cp;
  }

private:
  Vec2 point(double u) const noexcept { return curve_.r(domain_.map(u), t_); }

  // ‖x − r(u)‖², written out in scalars: this is the innermost loop of every search.
  double dist2(double u) const noexcept
  {
    const Vec2 p = point(u);
    const double dx = px_ - p.x();
    const double dy = py_ - p.y();
    return dx * dx + dy * dy;
  }

  // Global-stage sample spacing: closed [u_i, u_f) with M samples, open inclusive.
  double sampleStep() const noexcept
  {
    return domain_.closed ? domain_.period / sample_count_ : domain_.period / (sample_count_ - 1);
  }

  double sampleParameter(int k, double du) const noexcept
  {
    if (!domain_.closed && k == sample_count_ - 1) {
      return domain_.hi;
    }
    return domain_.lo + du * k;
  }

  // Half-widths (Δu_b, Δu_f) of the local window around `center` (math_spec §2.5).
  std::pair<double, double> window(double center) const noexcept
  {
    const double half = 0.5 * curve_.options().search_window;
    const double max_du = domain_.closed ? 0.5 * domain_.period : domain_.period;
    if (curve_.isTimeVarying()) {
      const double speed = curve_.drdu(domain_.map(center), t_).norm();
      double du = speed > 0.0 && std::isfinite(speed) ? half / speed : max_du;
      du = std::min(du, max_du);
      return {du, du};
    }
    const double length = curve_.length(t_);
    if (domain_.closed && 2.0 * half >= length) {
      return {max_du, max_du};
    }
    const double s_center = curve_.stationAt(center, t_);
    if (domain_.closed) {
      const double u_center = domain_.map(center);
      const double u_lo = curve_.parameterAt(s_center - half, t_);
      const double u_hi = curve_.parameterAt(s_center + half, t_);
      double du_b = positiveMod(u_center - u_lo, domain_.period);
      double du_f = positiveMod(u_hi - u_center, domain_.period);
      if (!(du_b + du_f <= domain_.period)) {
        du_b = max_du;
        du_f = max_du;
      }
      return {du_b, du_f};
    }
    const double u_lo = curve_.parameterAt(std::max(0.0, s_center - half), t_);
    const double u_hi = curve_.parameterAt(std::min(length, s_center + half), t_);
    return {std::max(0.0, center - u_lo), std::max(0.0, u_hi - center)};
  }

  // Golden section on [a, b] (math_spec §2.5): stop at a bracket width of 1e-10·max(1, u_f − u_i)
  // or after 100 iterations. Reports whether the minimum sits on an edge.
  Golden golden(double a, double b) const noexcept
  {
    const double a0 = a;
    const double b0 = b;
    double x1 = b - kInvGoldenRatio * (b - a);
    double x2 = a + kInvGoldenRatio * (b - a);
    double f1 = dist2(x1);
    double f2 = dist2(x2);
    bool lo_moved = false;
    bool hi_moved = false;
    for (int iteration = 0; iteration < kGoldenSectionMaxIterations && (b - a) > tolerance_;
         ++iteration) {
      if (f1 < f2) {
        b = x2;
        hi_moved = true;
        x2 = x1;
        f2 = f1;
        x1 = b - kInvGoldenRatio * (b - a);
        f1 = dist2(x1);
      } else {
        a = x1;
        lo_moved = true;
        x1 = x2;
        f1 = f2;
        x2 = a + kInvGoldenRatio * (b - a);
        f2 = dist2(x2);
      }
    }
    Golden result;
    if (f1 <= f2 || std::isnan(f2)) {
      result.u = x1;
      result.f = f1;
    } else {
      result.u = x2;
      result.f = f2;
    }
    if (!lo_moved) {
      const double fa = dist2(a0);
      if (fa <= result.f || std::isnan(result.f)) {
        result.u = a0;
        result.f = fa;
        result.at_lo = true;
      }
    }
    if (!hi_moved) {
      const double fb = dist2(b0);
      if (fb < result.f || std::isnan(result.f)) {
        result.u = b0;
        result.f = fb;
        result.at_lo = false;
        result.at_hi = true;
      }
    }
    return result;
  }

  // Polish of a golden-section minimiser. Golden section on ‖x − r(u)‖² resolves u only to
  // about sqrt(machine epsilon)·D/‖r'‖ (the squared distance is flat at its minimum), which is
  // 1e-8 m at D = 1 m. A few secant steps on the stationarity condition
  // g(u) = (r(u) − x)ᵀ r'(u) = 0, inside a trust region around the golden-section result and
  // only while g increases (a minimum of the distance), recover the foot to rounding.
  double polish(double u_start) const noexcept
  {
    const double trust = kPolishTrustFraction * domain_.period;
    double lo = u_start - trust;
    double hi = u_start + trust;
    if (!domain_.closed) {
      lo = std::max(lo, domain_.lo);
      hi = std::min(hi, domain_.hi);
    }
    const auto g = [this](double u) {
      const double mapped = domain_.map(u);
      return (curve_.r(mapped, t_) - x_).dot(curve_.drdu(mapped, t_));
    };
    double ua = u_start;
    double ga = g(ua);
    if (!std::isfinite(ga) || ga == 0.0) {
      return u_start;
    }
    const double step = kPolishStepFraction * domain_.period;
    double ub = std::min(hi, std::max(lo, ga > 0.0 ? ua - step : ua + step));
    if (ub == ua) {
      return u_start;
    }
    double gb = g(ub);
    double best_u = u_start;
    double best_g = std::abs(ga);
    if (std::isfinite(gb) && std::abs(gb) < best_g) {
      best_u = ub;
      best_g = std::abs(gb);
    }
    for (int iteration = 0; iteration < kPolishMaxIterations; ++iteration) {
      if (!std::isfinite(gb) || gb == 0.0) {
        break;
      }
      const double dg = gb - ga;
      const double du = ub - ua;
      if (dg == 0.0 || du == 0.0) {
        break;
      }
      const double slope = dg / du;
      if (!(slope > 0.0)) {
        break;
      }
      const double next = ub - gb / slope;
      if (!(next >= lo && next <= hi)) {
        break;
      }
      ua = ub;
      ga = gb;
      ub = next;
      gb = g(ub);
      if (std::isfinite(gb) && std::abs(gb) < best_g) {
        best_u = ub;
        best_g = std::abs(gb);
      }
      if (std::abs(ub - ua) <= 4.0 * kMachineEpsilon * std::max(1.0, std::abs(ub))) {
        break;
      }
    }
    return best_u;
  }

  // Polishes an interior minimiser and resolves domain ends of open curves onto the rays.
  Candidate refine(double u) const noexcept
  {
    const bool at_lo = !domain_.closed && u - domain_.lo <= 2.0 * tolerance_;
    const bool at_hi = !domain_.closed && domain_.hi - u <= 2.0 * tolerance_;
    if (at_lo || at_hi) {
      const Candidate ray = rayCandidate(at_hi);
      if (ray.valid) {
        return ray;
      }
    }
    if (std::isfinite(u)) {
      // Near an open end the trust region is clamped to the domain.
      u = polish(u);
    }
    Candidate candidate;
    candidate.u = domain_.map(u);
    candidate.foot = curve_.r(candidate.u, t_);
    candidate.D = (x_ - candidate.foot).norm();
    candidate.piece = PieceKind::kParametric;
    candidate.valid = isFinite(candidate.foot) && std::isfinite(candidate.D);
    return candidate;
  }

  // Foot on a ray of an open curve: r(u_f) + τT(u_f), τ > 0, or r(u_i) + τT(u_i), τ < 0
  // (one-sided end tangents). Invalid when x does not project beyond the anchor.
  Candidate rayCandidate(bool forward) const noexcept
  {
    Candidate candidate;
    if (domain_.closed) {
      return candidate;
    }
    const double u_end = forward ? domain_.hi : domain_.lo;
    const Vec2 anchor = curve_.r(u_end, t_);
    const Vec2 d = curve_.drdu(u_end, t_);
    const double speed = d.norm();
    if (!(speed > 0.0) || !std::isfinite(speed) || !isFinite(anchor)) {
      return candidate;
    }
    const Vec2 T = d / speed;
    const double tau = (x_ - anchor).dot(T);
    if (forward ? !(tau > 0.0) : !(tau < 0.0)) {
      return candidate;
    }
    candidate.u = u_end;
    candidate.tau = tau;
    candidate.foot = anchor + tau * T;
    candidate.ray_tangent = T;
    candidate.D = (x_ - candidate.foot).norm();
    candidate.piece = forward ? PieceKind::kForwardRay : PieceKind::kBackwardRay;
    candidate.valid = isFinite(candidate.foot) && std::isfinite(candidate.D);
    return candidate;
  }

  // Station of a candidate: s(u) on the curve, τ on the backward ray, L + τ on the forward ray.
  double station(const Candidate & candidate) const noexcept
  {
    switch (candidate.piece) {
      case PieceKind::kBackwardRay:
        return candidate.tau;
      case PieceKind::kForwardRay:
        return curve_.length(t_) + candidate.tau;
      default:
        return curve_.stationAt(candidate.u, t_);
    }
  }

  // Distance to the hint along the curve (closed: wrapped), for ties.
  double hintDistance(double s, double s_hint) const noexcept
  {
    const double ds = s - s_hint;
    return domain_.closed ? std::abs(wrapPeriodic(ds, curve_.length(t_))) : std::abs(ds);
  }

  // Tie rule between two samples (math_spec §2.4).
  bool preferSample(int k, int best_k, double du, bool has_hint, double u_hint) const noexcept
  {
    const double u_k = sampleParameter(k, du);
    const double u_best = sampleParameter(best_k, du);
    if (!has_hint) {
      // Stations increase with u, so the smallest dir·s is the smallest dir·u.
      return direction_ * u_k < direction_ * u_best;
    }
    const double s_hint = curve_.stationAt(u_hint, t_);
    return hintDistance(curve_.stationAt(u_k, t_), s_hint) <
           hintDistance(curve_.stationAt(u_best, t_), s_hint);
  }

  // Inserts a sample minimum into `kept`, ordered by D (ties by the tie rule), keeping the best
  // kGlobalCandidates.
  void keep(
    const SampleCandidate & entry, std::array<SampleCandidate, kGlobalCandidates> * kept,
    std::size_t * kept_count, double du, bool has_hint, double u_hint) const noexcept
  {
    const auto before = [&](const SampleCandidate & a, const SampleCandidate & b) {
      if (a.D < b.D - kTieTolerance) {
        return true;
      }
      return isTie(a.D, b.D) && preferSample(a.k, b.k, du, has_hint, u_hint);
    };
    std::size_t position = *kept_count;
    while (position > 0 && before(entry, (*kept)[position - 1])) {
      --position;
    }
    if (position >= kGlobalCandidates) {
      return;
    }
    const std::size_t last = std::min(*kept_count, kGlobalCandidates - 1);
    for (std::size_t i = last; i > position; --i) {
      (*kept)[i] = (*kept)[i - 1];
    }
    (*kept)[position] = entry;
    *kept_count = std::min(*kept_count + 1, kGlobalCandidates);
  }

  // True iff `a` beats `b`: smaller D, or a tie won by the tie rule.
  bool better(const Candidate & a, const Candidate & b, bool has_hint, double u_hint) const noexcept
  {
    if (!b.valid) {
      return a.valid;
    }
    if (!a.valid) {
      return false;
    }
    if (a.D < b.D - kTieTolerance) {
      return true;
    }
    if (!isTie(a.D, b.D)) {
      return false;
    }
    const double s_a = station(a);
    const double s_b = station(b);
    if (!has_hint) {
      return direction_ * s_a < direction_ * s_b;
    }
    const double s_hint = curve_.stationAt(u_hint, t_);
    return hintDistance(s_a, s_hint) < hintDistance(s_b, s_hint);
  }

  static constexpr double kMachineEpsilon = 2.220446049250313e-16;

  const ParametricCurve & curve_;
  Domain domain_;
  const double * samples_;
  Vec2 x_;
  double px_;
  double py_;
  double t_;
  int direction_;
  Vec2 fallback_;
  double tolerance_;
  int sample_count_;
};

}  // namespace

ParametricCurve::ParametricCurve(
  double u_min, double u_max, bool closed, bool time_varying, const CurveOptions & options)
: u_min_(u_min), u_max_(u_max), closed_(closed), time_varying_(time_varying), options_(options)
{
  if (!std::isfinite(u_min) || !std::isfinite(u_max) || !(u_max > u_min)) {
    throw ConfigError(
      "ParametricCurve: the domain [" + fmt(u_min) + ", " + fmt(u_max) +
      "] must be finite with s_min < s_max");
  }
  const ValidationReport report = validateCurveOptions(options);
  if (!report.ok()) {
    throw ConfigError("ParametricCurve: " + report.summary());
  }
}

ParametricCurve::~ParametricCurve() = default;

double ParametricCurve::uMin() const noexcept { return u_min_; }

double ParametricCurve::uMax() const noexcept { return u_max_; }

const CurveOptions & ParametricCurve::options() const noexcept { return options_; }

double ParametricCurve::speedFloor() const noexcept { return speed_floor_; }

void ParametricCurve::setSpec(std::shared_ptr<const CurveSpec> spec) noexcept
{
  spec_ = std::move(spec);
}

const CurveSpec * ParametricCurve::spec() const noexcept { return spec_.get(); }

bool ParametricCurve::isClosed() const noexcept { return closed_; }

bool ParametricCurve::isTimeVarying() const noexcept { return time_varying_; }

double ParametricCurve::length(double t) const noexcept
{
  if (!time_varying_ && !table_s_.empty()) {
    return length_;
  }
  const Domain domain(*this);
  return panelIntegral(*this, domain, u_min_, u_max_, t);
}

double ParametricCurve::stationAt(double u, double t) const noexcept
{
  if (!std::isfinite(u)) {
    return kNaN;
  }
  const Domain domain(*this);
  const double mapped = domain.map(u);
  if (time_varying_ || table_s_.empty()) {
    return panelIntegral(*this, domain, u_min_, mapped, t);
  }
  const std::size_t intervals = table_s_.size() - 1;
  const double du = (u_max_ - u_min_) / static_cast<double>(intervals);
  std::size_t k = 0;
  const double position = (mapped - u_min_) / du;
  if (position > 0.0) {
    k = std::min(intervals - 1, static_cast<std::size_t>(position));
  }
  double s = table_s_[k] + gaussPanel(*this, domain, table_u_[k], mapped, t);
  if (closed_) {
    if (!(s < length_)) {
      s -= length_;
    }
    s = std::max(0.0, s);
  } else {
    s = std::min(length_, std::max(0.0, s));
  }
  return s;
}

double ParametricCurve::parameterAt(double s, double t) const noexcept
{
  if (!std::isfinite(s)) {
    return kNaN;
  }
  const Domain domain(*this);
  const double total = length(t);
  if (!(total > 0.0) || !std::isfinite(total)) {
    return kNaN;
  }
  double target = s;
  if (closed_) {
    target = positiveMod(s, total);
  } else {
    target = std::min(total, std::max(0.0, s));
  }
  const double tolerance = kInverseTolerance * std::max(1.0, total);

  double lo = u_min_;
  double hi = u_max_;
  double base_u = u_min_;
  double base_s = 0.0;
  double u = u_min_;
  if (!time_varying_ && !table_s_.empty()) {
    const auto upper = std::upper_bound(table_s_.begin(), table_s_.end(), target);
    std::size_t k =
      upper == table_s_.begin() ? 0 : static_cast<std::size_t>(upper - table_s_.begin()) - 1;
    k = std::min(k, table_s_.size() - 2);
    lo = table_u_[k];
    hi = table_u_[k + 1];
    base_u = lo;
    base_s = table_s_[k];
    const double span = table_s_[k + 1] - table_s_[k];
    u = span > 0.0 ? lo + (target - base_s) / span * (hi - lo) : lo;
  } else {
    u = u_min_ + target / total * (u_max_ - u_min_);
  }
  for (int iteration = 0; iteration < kInverseMaxIterations; ++iteration) {
    const double g = base_s + panelIntegral(*this, domain, base_u, u, t) - target;
    if (!std::isfinite(g)) {
      return kNaN;
    }
    if (std::abs(g) <= tolerance) {
      break;
    }
    if (g > 0.0) {
      hi = u;
    } else {
      lo = u;
    }
    const double speed = drdu(domain.map(u), t).norm();
    double next = speed > 0.0 && std::isfinite(speed) ? u - g / speed : 0.5 * (lo + hi);
    if (!(next > lo && next < hi)) {
      next = 0.5 * (lo + hi);
    }
    if (std::abs(next - u) <= kStepEpsilon * std::max(1.0, std::abs(u))) {
      u = next;
      break;
    }
    u = next;
  }
  return domain.map(u);
}

double ParametricCurve::arcLength(double u0, double u1, double t) const noexcept
{
  if (!std::isfinite(u0) || !std::isfinite(u1)) {
    return kNaN;
  }
  const Domain domain(*this);
  if (!time_varying_ && !table_s_.empty()) {
    const auto unwrapped = [&](double u) {
      if (!closed_) {
        return stationAt(domain.map(u), t);
      }
      const double laps = std::floor((u - u_min_) / domain.period);
      return laps * length_ + stationAt(u, t);
    };
    return unwrapped(u1) - unwrapped(u0);
  }
  if (!closed_) {
    return panelIntegral(*this, domain, domain.map(u0), domain.map(u1), t);
  }
  return panelIntegral(*this, domain, u0, u1, t);
}

Vec2 ParametricCurve::drdu(double u, double t) const noexcept
{
  const Domain domain(*this);
  const double h = kParamDerivativeStepFraction * domain.period;
  if (closed_) {
    return (r(domain.map(u + h), t) - r(domain.map(u - h), t)) / (2.0 * h);
  }
  const double uc = domain.map(u);
  if (uc - h < u_min_) {
    return (-3.0 * r(uc, t) + 4.0 * r(uc + h, t) - r(uc + 2.0 * h, t)) / (2.0 * h);
  }
  if (uc + h > u_max_) {
    return (3.0 * r(uc, t) - 4.0 * r(uc - h, t) + r(uc - 2.0 * h, t)) / (2.0 * h);
  }
  return (r(uc + h, t) - r(uc - h, t)) / (2.0 * h);
}

Vec2 ParametricCurve::drdt(double u, double t) const noexcept
{
  if (!time_varying_) {
    return Vec2::Zero();
  }
  const Domain domain(*this);
  const double uc = domain.map(u);
  return (r(uc, t + kTimeDerivativeStep) - r(uc, t - kTimeDerivativeStep)) /
         (2.0 * kTimeDerivativeStep);
}

ClosestPoint ParametricCurve::closestPoint(
  const Vec2 & x, const EvalContext & context, const CurveHint & hint) const noexcept
{
  if (!isFinite(x) || !std::isfinite(context.t)) {
    ClosestPoint invalid;
    invalid.valid = false;
    invalid.status_bits = Status::kNonFinite;
    return invalid;
  }
  const Search search(*this, samples_.empty() ? nullptr : samples_.data(), x, context);
  const bool has_hint = hint.valid && std::isfinite(hint.s);
  if (!has_hint) {
    // Initialisation: the global stage, adopted unconditionally (math_spec §2.4).
    return search.finalize(search.global(false, 0.0), Status::kNone);
  }
  const double u_prev = search.mapHint(hint.s);
  bool had_edge = false;
  const Candidate local = search.local(u_prev, &had_edge);
  std::uint32_t status = Status::kNone;
  Candidate result = local;
  if (!local.valid || had_edge || local.D > options_.global_search_trigger) {
    const Candidate global = search.global(true, u_prev);
    if (!local.valid) {
      if (global.valid) {
        result = global;
        status |= Status::kAmbiguousClosest;
      }
    } else if (global.valid && !isTie(global.D, local.D)) {
      // A tie is the same foot (or an equally near one): ties go to the hint, i.e. stay local.
      const double arc = search.arcBetween(local, global);
      const double chord = (local.foot - global.foot).norm();
      if (adoptGlobalCandidate(local.D, global.D, arc, chord, options_.global_search_hysteresis)) {
        result = global;
        status |= Status::kAmbiguousClosest;
      }
    }
  }
  return search.finalize(result, status);
}

Vec2 ParametricCurve::velocity(const ClosestPoint & cp, double t) const noexcept
{
  if (!time_varying_ || !cp.valid || cp.on_ray || cp.piece != PieceKind::kParametric) {
    return Vec2::Zero();
  }
  return drdt(cp.u_star, t);
}

Vec2 ParametricCurve::pointAt(double s, double t) const noexcept
{
  if (!std::isfinite(s)) {
    return Vec2(kNaN, kNaN);
  }
  if (!closed_) {
    const double total = length(t);
    if (s < 0.0 || s > total) {
      const bool forward = s > total;
      const double u_end = forward ? u_max_ : u_min_;
      const Vec2 T = unitOr(drdu(u_end, t), Vec2(kNaN, kNaN));
      return r(u_end, t) + (forward ? s - total : s) * T;
    }
  }
  return r(parameterAt(s, t), t);
}

Vec2 ParametricCurve::tangentAt(double s, double t) const noexcept
{
  if (!std::isfinite(s)) {
    return Vec2(kNaN, kNaN);
  }
  double u = 0.0;
  if (!closed_ && (s < 0.0 || s > length(t))) {
    u = s < 0.0 ? u_min_ : u_max_;
  } else {
    u = parameterAt(s, t);
  }
  return unitOr(drdu(u, t), Vec2(kNaN, kNaN));
}

std::size_t ParametricCurve::sampleStations(
  double s0, double s1, double ds, double t, Span<Vec2> out) const noexcept
{
  if (
    !(ds > 0.0) || !std::isfinite(s0) || !std::isfinite(s1) || !std::isfinite(ds) || out.empty()) {
    return 0;
  }
  if (!closed_) {
    const double total = length(t);
    s0 = std::max(s0, 0.0);
    s1 = std::min(s1, total);
  }
  if (s1 < s0) {
    return 0;
  }
  const Domain domain(*this);
  const double tolerance = kInverseTolerance * std::max({1.0, std::abs(s0), std::abs(s1)});
  std::size_t count = 0;
  double last = s0;
  double u = u_min_;
  for (std::size_t k = 0; count < out.size(); ++k) {
    const double s = s0 + static_cast<double>(k) * ds;
    if (s > s1) {
      break;
    }
    if (!time_varying_ || k == 0) {
      // Static curves invert the u↔s table directly.
      u = parameterAt(s, t);
    } else {
      // Time-varying curves march: solve ∫_{u_prev}^{u} ‖r'‖ = Δs by Newton from the
      // previous station (one Gauss–Legendre panel per small step).
      const double step = s - last;
      double lo = u;
      double hi = u + domain.period;
      const double speed = drdu(domain.map(u), t).norm();
      double next = speed > 0.0 && std::isfinite(speed) ? u + step / speed : u;
      for (int iteration = 0; iteration < kInverseMaxIterations; ++iteration) {
        if (!(next > lo && next < hi)) {
          next = 0.5 * (lo + hi);
        }
        const double g = panelIntegral(*this, domain, u, next, t) - step;
        if (!std::isfinite(g) || std::abs(g) <= tolerance) {
          break;
        }
        if (g > 0.0) {
          hi = next;
        } else {
          lo = next;
        }
        const double next_speed = drdu(domain.map(next), t).norm();
        const double candidate =
          next_speed > 0.0 && std::isfinite(next_speed) ? next - g / next_speed : 0.5 * (lo + hi);
        if (std::abs(candidate - next) <= kStepEpsilon * std::max(1.0, std::abs(next))) {
          next = candidate;
          break;
        }
        next = candidate;
      }
      u = closed_ ? next : domain.map(next);
    }
    out[count++] = r(domain.map(u), t);
    last = s;
  }
  if (count < out.size() && s1 - last > 1e-9) {
    out[count++] = r(parameterAt(s1, t), t);
  }
  return count;
}

void ParametricCurve::initialize(double t0)
{
  if (!std::isfinite(t0)) {
    reject("t0 = " + fmt(t0) + " must be finite");
  }
  const Domain domain(*this);
  const int count = options_.global_search_samples;
  const double period = domain.period;

  // Global-stage samples at t0 (math_spec §2.5): closed [u_i, u_f), open inclusive.
  const double du_samples = closed_ ? period / count : period / (count - 1);
  std::vector<double> sample_u(static_cast<std::size_t>(count));
  std::vector<Vec2> points(static_cast<std::size_t>(count));
  std::vector<Vec2> derivatives(static_cast<std::size_t>(count));
  for (int k = 0; k < count; ++k) {
    const auto index = static_cast<std::size_t>(k);
    sample_u[index] = !closed_ && k == count - 1 ? u_max_ : u_min_ + du_samples * k;
    points[index] = r(sample_u[index], t0);
    derivatives[index] = drdu(sample_u[index], t0);
    if (!isFinite(points[index]) || !isFinite(derivatives[index])) {
      reject(
        "the curve is not finite at s = " + fmt(sample_u[index]) + " (r = (" +
        fmt(points[index].x()) + ", " + fmt(points[index].y()) + "))");
    }
  }

  // u↔s table at t0: M + 1 nodes, 5-point Gauss–Legendre per interval (math_spec §2.5).
  std::vector<double> table_u(static_cast<std::size_t>(count) + 1);
  std::vector<double> table_s(static_cast<std::size_t>(count) + 1);
  const double du_table = period / count;
  table_u[0] = u_min_;
  table_s[0] = 0.0;
  for (int k = 0; k < count; ++k) {
    const auto index = static_cast<std::size_t>(k);
    table_u[index + 1] = k + 1 == count ? u_max_ : u_min_ + du_table * (k + 1);
    table_s[index + 1] =
      table_s[index] + gaussPanel(*this, domain, table_u[index], table_u[index + 1], t0);
  }
  const double total = table_s.back();
  if (!std::isfinite(total) || !(total > 0.0)) {
    reject("the curve length " + fmt(total) + " is not positive and finite");
  }

  // Closure (math_spec §2.6 rule 3), with one-sided end differences so the ends are not mixed.
  if (closed_) {
    const double h = kParamDerivativeStepFraction * period;
    const Vec2 start = r(u_min_, t0);
    const Vec2 end = r(u_max_, t0);
    const double gap = (end - start).norm();
    const double allowed = std::max(kClosureAbsoluteGap, kClosureRelativeGap * total);
    if (!(gap <= allowed)) {
      reject(
        "the curve is marked closed but does not close: |r(s_max) - r(s_min)| = " + fmt(gap) +
        " > " + fmt(allowed));
    }
    const Vec2 d_start =
      (-3.0 * start + 4.0 * r(u_min_ + h, t0) - r(u_min_ + 2.0 * h, t0)) / (2.0 * h);
    const Vec2 d_end = (3.0 * end - 4.0 * r(u_max_ - h, t0) + r(u_max_ - 2.0 * h, t0)) / (2.0 * h);
    const double angle = std::atan2(std::abs(cross2(d_start, d_end)), d_start.dot(d_end));
    if (!(angle <= kClosureMaxTangentAngle)) {
      reject(
        "the curve is marked closed but its end tangents differ by " + fmt(angle * 180.0 / kPi) +
        " deg (> 1 deg)");
    }
  }

  // Regularity (math_spec §2.6 rules 1 and 2) on the M samples.
  double sum = 0.0;
  double min_speed = kInf;
  std::size_t min_index = 0;
  for (std::size_t k = 0; k < derivatives.size(); ++k) {
    const double speed = derivatives[k].norm();
    sum += speed;
    if (speed < min_speed) {
      min_speed = speed;
      min_index = k;
    }
  }
  const double mean = sum / count;
  if (!(mean > 0.0) || !(min_speed >= kRegularitySpeedRatio * mean)) {
    reject(
      "the curve is not regular: |dr/ds| = " + fmt(min_speed) + " at s = " +
      fmt(sample_u[min_index]) + " is below 1e-3 x the mean " + fmt(mean) + " (stationary point)");
  }
  const std::size_t pairs = closed_ ? derivatives.size() : derivatives.size() - 1;
  for (std::size_t k = 0; k < pairs; ++k) {
    const std::size_t next = (k + 1) % derivatives.size();
    const Vec2 & a = derivatives[k];
    const Vec2 & b = derivatives[next];
    const double angle = std::atan2(std::abs(cross2(a, b)), a.dot(b));
    if (angle > kRegularityMaxTurn + kTurnSlack) {
      reject(
        "the curve is not regular: the tangents at s = " + fmt(sample_u[k]) +
        " and s = " + fmt(sample_u[next]) + " differ by " + fmt(angle * 180.0 / kPi) +
        " deg (> 90 deg: a cusp or stationary point between samples)");
    }
  }

  speed_floor_ = kRegularitySpeedRatio * mean;
  if (time_varying_) {
    length_ = total;
    table_u_.clear();
    table_s_.clear();
    samples_.clear();
  } else {
    length_ = total;
    table_u_ = std::move(table_u);
    table_s_ = std::move(table_s);
    samples_.resize(2 * points.size());
    for (std::size_t k = 0; k < points.size(); ++k) {
      samples_[2 * k] = points[k].x();
      samples_[2 * k + 1] = points[k].y();
    }
  }
}

}  // namespace vectorfield_core
