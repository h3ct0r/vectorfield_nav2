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
/// @brief 3D core (experimental): closest-point types and the abstract curve in
/// \f$\mathbb R^3\f$ (math_spec §14.1–§14.2).

#ifndef VECTORFIELD_CORE__D3__CURVE_HPP_
#define VECTORFIELD_CORE__D3__CURVE_HPP_

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core::d3
{

struct CurveSpec3;

/// @brief Search state carried across cycles (the 3D CurveHint): the previous foot and the
/// curve's own parameter there (arc length for polylines, \f$u\f$ for parametric curves).
struct CurveHint3
{
  /// Previous foot point \f$x^\star_{\mathrm{prev}}\f$.
  Vec3 x_star{Vec3::Zero()};
  /// Previous curve parameter (s for polylines, u for parametric curves).
  double s{0.0};
  /// Previous owning segment (polylines), −1 if unknown.
  int segment{-1};
  /// False before the first search: the global stage runs and is adopted unconditionally.
  bool valid{false};
};

/// @brief Per-call context of a closest-point search.
struct EvalContext3
{
  /// Evaluation time t (s); ignored by static curves.
  double t{0.0};
  /// Traversal direction ±1; breaks initialisation ties toward the smallest dir·s.
  int direction{1};
  /// \f$T_{\mathrm{last}}\f$ for the hairpin rule (curve parameter direction); a norm below
  /// \f$\varepsilon_{\mathrm{hp}}\f$ means none.
  Vec3 last_tangent{Vec3::Zero()};
};

/// @brief Result of a closest-point search in \f$\mathbb R^3\f$.
///
/// For a valid result: \f$\lVert T\rVert = 1\f$, \f$n^\top T = 0\f$ to rounding, \f$n = 0\f$ iff
/// \f$D < \varepsilon_D\f$, otherwise \f$n = \Pi_T(x - x^\star)/\lVert\Pi_T(x - x^\star)\rVert\f$.
struct ClosestPoint3
{
  /// Foot point \f$x^\star\f$.
  Vec3 x_star{Vec3::Zero()};
  /// Unit tangent T in the parameter direction (wedge blend or hairpin rule at a vertex).
  Vec3 T{Vec3::UnitX()};
  /// Unit normal n from the foot toward x, or zero on the curve.
  Vec3 n{Vec3::Zero()};
  /// Distance \f$D \ge 0\f$.
  double D{kInf};
  /// Arc-length station of the foot (negative on the backward ray, beyond L on the forward ray;
  /// \f$[0, L)\f$ for closed curves).
  double s_star{0.0};
  /// The curve's own parameter at the foot (s for polylines, \f$u^\star\f$ for parametric curves,
  /// clamped to the domain end on a ray). Becomes CurveHint3::s.
  double u_star{0.0};
  /// Polylines: the owning segment, or the vertex index for kVertex, −1 / N−1 for the rays; −1
  /// for parametric curves.
  int segment{-1};
  /// Owning piece kind (shared with the planar core).
  PieceKind piece{PieceKind::kNone};
  /// The foot lies on a ray.
  bool on_ray{false};
  /// A vertex owns x with \f$D \ge \varepsilon_D\f$ (the wedge blend applies).
  bool in_wedge{false};
  /// The hairpin rule produced T (also Status::kHairpin).
  bool hairpin{false};
  /// False when no closest point exists (empty curve, non-finite input).
  bool valid{false};
  /// Status bits: kHairpin, kAmbiguousClosest (adopted global jump), kNonFinite.
  std::uint32_t status_bits{Status::kNone};
};

/// @brief The hint for the next search from a result.
inline CurveHint3 makeHint(const ClosestPoint3 & cp) noexcept
{
  CurveHint3 hint;
  hint.x_star = cp.x_star;
  hint.s = cp.u_star;
  hint.segment = cp.segment;
  hint.valid = cp.valid;
  return hint;
}

/// @brief A curve \f$r(s, t)\f$ in \f$\mathbb R^3\f$, open or closed, possibly time-varying.
///
/// Same contract as the planar Curve (curve.hpp): open curves are borderless (rays along the
/// end tangents), arc length is measured from the start, closed curves wrap, open
/// time-varying curves are rejected by makeCurve3(), and every method is `noexcept` and safe to
/// call concurrently (ExpressionCurve3 serialises muParser through a mutex). There is no curve
/// frame: inputs and outputs are in the caller's frame.
class Curve3
{
public:
  virtual ~Curve3() = default;

  /// @brief Closest point of `x` (math_spec §14.1).
  virtual ClosestPoint3 closestPoint(
    const Vec3 & x, const EvalContext3 & context, const CurveHint3 & hint) const noexcept = 0;
  /// @brief True for closed curves.
  virtual bool isClosed() const noexcept = 0;
  /// @brief True iff the shape depends on t.
  virtual bool isTimeVarying() const noexcept = 0;
  /// @brief Length \f$L(t)\f$ (open: rays excluded; closed: one lap).
  virtual double length(double t) const noexcept = 0;
  /// @brief \f$\partial r/\partial t\f$ at the foot of `cp` (zero for static curves and on rays).
  virtual Vec3 velocity(const ClosestPoint3 & cp, double t) const noexcept = 0;
  /// @brief \f$r(s, t)\f$ at station s (rays beyond the ends of open curves; wrapped if closed).
  virtual Vec3 pointAt(double s, double t) const noexcept = 0;
  /// @brief Unit tangent at station s.
  virtual Vec3 tangentAt(double s, double t) const noexcept = 0;
  /// @brief The specification this curve was built from by makeCurve3(), or null.
  virtual const CurveSpec3 * spec() const noexcept { return nullptr; }

  /// @brief Writes \f$r(s_0 + k\,\Delta s, t)\f$ while \f$\le s_1\f$, then \f$r(s_1, t)\f$ if
  /// the last station is more than 1e-9 m short (open curves clamp to \f$[0, L]\f$). Same rules
  /// as the planar Curve::sampleStations(). Allocation-free.
  /// @return The number of points written.
  std::size_t sampleStations(
    double s0, double s1, double ds, double t, Span<Vec3> out) const noexcept;

  /// @brief Station of the end, \f$L(t)\f$.
  double endStation(double t) const noexcept { return length(t); }
  /// @brief Arc-length difference wrapped for closed curves into \f$(-L/2, L/2]\f$.
  double wrapStationDifference(double ds, double t) const noexcept
  {
    return isClosed() ? wrapPeriodic(ds, length(t)) : ds;
  }

protected:
  Curve3() = default;
  Curve3(const Curve3 &) = default;
  Curve3(Curve3 &&) = default;
  Curve3 & operator=(const Curve3 &) = default;
  Curve3 & operator=(Curve3 &&) = default;
};

/// @brief Remaining arc length \f$\max(0, L - s^\star)\f$ to the end of an open curve; NaN for
/// closed curves.
inline double remainingToEnd(const Curve3 & curve, const ClosestPoint3 & cp, double t) noexcept
{
  if (curve.isClosed()) {
    return kNaN;
  }
  const double remaining = curve.endStation(t) - cp.s_star;
  return remaining > 0.0 ? remaining : 0.0;
}

static_assert(std::is_trivially_destructible_v<CurveHint3>, "CurveHint3 is Tracker3State data");

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__CURVE_HPP_
