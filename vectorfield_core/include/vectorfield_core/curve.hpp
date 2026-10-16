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
/// @brief The abstract curve interface (plan §3.2; math_spec §2).
///
/// Implementations: PolylineCurve (polyline_curve.hpp), ParametricCurve (parametric_curve.hpp)
/// and, through makeCurve() (curve_spec.hpp), the built-in shapes and ExpressionCurve, which are
/// internal to the library.

#ifndef VECTORFIELD_CORE__CURVE_HPP_
#define VECTORFIELD_CORE__CURVE_HPP_

#include <cmath>
#include <cstddef>

#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

struct CurveSpec;

/// @brief A planar curve \f$r(s, t)\f$ in its own frame, open or closed, possibly time-varying.
///
/// **Frames.** Every input and output is in the curve frame \f$\mathcal F\f$. The Tracker maps
/// the evaluation point in with `CurveFrame::T_eval_curve` and maps \f$x^\star\f$, \f$T\f$,
/// \f$n\f$ and \f$\partial r/\partial t\f$ back (plan §3.1).
///
/// **Open curves are borderless.** They are extended at both ends by semi-infinite rays along
/// the end tangents (math_spec §2.1 [Design]): closestPoint() may return a foot on a ray,
/// pointAt() and tangentAt() extrapolate along them, and sampleStations() never samples them.
/// Arc length is measured from the start (\f$s = 0\f$ at P0 or \f$u_i\f$), negative on the
/// backward ray. Closed curves wrap: stations are taken modulo \f$L\f$ into \f$[0, L)\f$.
///
/// **Time-varying** means the shape depends on \f$t\f$ (an expression whose variables include
/// `t`). Open time-varying curves are rejected by makeCurve() (UNSUPPORTED), so every open curve
/// is static. A moving *frame* does not make the curve itself time-varying.
///
/// **Thread safety.** All const member functions are safe to call concurrently on one object;
/// non-const ones (assign, setSearchPolicy) are not. The exception in spirit is ExpressionCurve:
/// its const methods are still safe, but every muParser evaluation is serialised by an internal
/// mutex (`evalNoThrow()`: mutex, try/catch, NaN on failure), so concurrent callers contend.
/// ExpressionCurve is non-copyable (PIMPL) with no `clone()`; for parallel throughput build a
/// per-thread copy with `makeCurve(*curve.spec(), options)`.
///
/// **Errors.** Every method is `noexcept`. A non-finite input or evaluation yields an invalid
/// ClosestPoint or Status::kNonFinite, never an exception.
class Curve
{
public:
  virtual ~Curve() = default;

  /// @brief Closest point of `x` (curve frame) per the curve's search rules (math_spec §2.4).
  /// @param x Query point in the curve frame.
  /// @param context Time, traversal direction and \f$T_{\mathrm{last}}\f$ (curve frame).
  /// @param hint Previous search state (CurveHint documents what each policy reads).
  /// @return The foot, tangent, normal, distance, stations and flags; `valid == false` for an
  /// empty curve or a non-finite `x`.
  virtual ClosestPoint closestPoint(
    const Vec2 & x, const EvalContext & context, const CurveHint & hint) const noexcept = 0;

  /// @brief True for closed curves (no rays; stations wrap).
  virtual bool isClosed() const noexcept = 0;

  /// @brief True iff the shape depends on \f$t\f$ (see the class comment).
  virtual bool isTimeVarying() const noexcept = 0;

  /// @brief Length \f$L(t)\f$ of the finite curve (open: from start to end, rays excluded;
  /// closed: one lap).
  virtual double length(double t) const noexcept = 0;

  /// @brief Shape velocity \f$\partial r/\partial t\f$ at the foot \f$u^\star\f$ of `cp`
  /// (curve frame), at time t; zero for static curves and on rays (math_spec §4.2).
  /// @param cp A result of closestPoint() on this curve, still in the curve frame.
  /// @param t Time (s).
  virtual Vec2 velocity(const ClosestPoint & cp, double t) const noexcept = 0;

  /// @brief Point \f$r(s, t)\f$ at arc-length station s (rays outside \f$[0, L]\f$ for open
  /// curves; s wrapped for closed curves). Used for \f$g_c = r(s_{\mathrm{goal}})\f$,
  /// \f$g_{\mathrm{ref}}\f$, braking-zone markers and gate samples.
  virtual Vec2 pointAt(double s, double t) const noexcept = 0;

  /// @brief Unit tangent \f$T(s, t)\f$ in the parameter direction at station s (polylines: the
  /// segment whose half-open interval \f$[S_k, S_{k+1})\f$ contains s; the last segment at the
  /// open end; ray tangents beyond).
  virtual Vec2 tangentAt(double s, double t) const noexcept = 0;

  /// @brief Writes \f$r(s_k, t)\f$ for \f$s_k = s_0 + k\,\Delta s\f$, \f$k = 0, 1, \dots\f$
  /// while \f$s_k \le s_1\f$, and then \f$r(s_1, t)\f$ if the last regular station is more than
  /// 1e-9 m before \f$s_1\f$, into `out` (at most `out.size()` points).
  ///
  /// Open curves clamp \f$[s_0, s_1]\f$ to \f$[0, L]\f$ first (rays are never sampled); closed
  /// curves wrap each station. This is the gate sampler of math_spec §6.4 (\f$\Delta s\f$ = one
  /// cell) and the curve drawer of the visualizer. The default calls pointAt() per station;
  /// parametric curves override it with a marching evaluation. Allocation-free.
  /// @return The number of points written; 0 if \f$\Delta s \le 0\f$, the range is empty or an
  /// input is non-finite.
  virtual std::size_t sampleStations(
    double s0, double s1, double ds, double t, Span<Vec2> out) const noexcept
  {
    if (!(ds > 0.0) || !std::isfinite(s0) || !std::isfinite(s1) || !std::isfinite(ds)) {
      return 0;
    }
    if (!isClosed()) {
      const double end = length(t);
      s0 = s0 < 0.0 ? 0.0 : s0;
      s1 = s1 > end ? end : s1;
    }
    if (s1 < s0) {
      return 0;
    }
    std::size_t count = 0;
    double last = s0;
    for (std::size_t k = 0; count < out.size(); ++k) {
      const double s = s0 + static_cast<double>(k) * ds;
      if (s > s1) {
        break;
      }
      out[count++] = pointAt(s, t);
      last = s;
    }
    if (count < out.size() && s1 - last > 1e-9) {
      out[count++] = pointAt(s1, t);
    }
    return count;
  }

  /// @brief The specification this curve was built from by makeCurve(), or null (a
  /// PolylineCurve assigned directly).
  virtual const CurveSpec * spec() const noexcept { return nullptr; }

  /// @brief Mean vertex spacing \f$\bar\ell\f$ (m): the length divided by the segment count for a
  /// vertex-based curve (PolylineCurve; for the plugin, the per-cycle window); 0 for parametric
  /// and expression curves, which have no vertices.
  ///
  /// Heading tracking widens its \f$\dot\psi_d\f$ probe offset to \f$\delta = \max(0.02,
  /// \lvert v\rvert\Delta t, \min(\bar\ell, 0.10))\f$ (math_spec §7.4). On a polyline the field
  /// direction turns only where the probes straddle a vertex, so a span \f$2\delta\f$ shorter than
  /// a vertex spacing sees the whole turning angle on some cycles and nothing on the others. A span
  /// that is a whole number of vertex spacings always contains the same number of vertices: the
  /// finite difference stops alternating, and the CommandLimiter's angular rate limit no longer
  /// rectifies that ripple into a heading bias (a steady offset inside tight curves).
  virtual double vertexSpacing() const noexcept { return 0.0; }

  /// @brief Station of the curve's start: always 0.
  double startStation() const noexcept { return 0.0; }

  /// @brief Station of the curve's end, \f$L(t)\f$ (open curves: the goal station for
  /// \f$\mathrm{dir} = +1\f$, math_spec §5.1).
  double endStation(double t) const noexcept { return length(t); }

  /// @brief Arc-length difference wrapped for closed curves into \f$(-L/2, L/2]\f$ (math_spec
  /// §2.2); unchanged for open curves.
  double wrapStationDifference(double ds, double t) const noexcept
  {
    return isClosed() ? wrapPeriodic(ds, length(t)) : ds;
  }

protected:
  Curve() = default;
  Curve(const Curve &) = default;
  Curve(Curve &&) = default;
  Curve & operator=(const Curve &) = default;
  Curve & operator=(Curve &&) = default;
};

/// @brief \f$\sigma_{\mathrm{end}} = \max(0, S_{\mathrm{end}} - s^\star)\f$: remaining arc
/// length from a foot to the end of an open curve (math_spec §5.5 window-end cap); NaN for
/// closed curves. Plan §3.2 lists \f$\sigma_{\mathrm{end}}\f$ with the closest point; it is
/// derived here instead of stored.
inline double remainingToEnd(const Curve & curve, const ClosestPoint & cp, double t) noexcept
{
  if (curve.isClosed()) {
    return kNaN;
  }
  const double remaining = curve.endStation(t) - cp.s_star;
  return remaining > 0.0 ? remaining : 0.0;
}

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__CURVE_HPP_
