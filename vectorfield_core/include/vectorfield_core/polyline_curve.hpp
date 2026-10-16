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
/// @brief Polyline curves with semi-infinite end rays, wedge tangents and the two search
/// policies (plan §3.2; math_spec §2.2–§2.4).
///
/// Implemented in src/polyline_curve.cpp (M1 wave 2, polyline). The public API is binding;
/// private members are the implementer's choice.

#ifndef VECTORFIELD_CORE__POLYLINE_CURVE_HPP_
#define VECTORFIELD_CORE__POLYLINE_CURVE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Result of PolylineCurve::assign().
enum class AssignStatus : std::uint8_t
{
  /// The polyline was assigned.
  kOk = 0,
  /// Fewer than 2 distinct vertices (closed: fewer than 3); the curve is left empty.
  kTooFew = 1,
  /// A vertex is not finite; the curve is left empty.
  kNonFinite = 2,
};

/// @brief A polyline \f$P_0, \dots, P_{N-1}\f$, open (with semi-infinite rays) or closed
/// (math_spec §2.2, sec-polyline).
///
/// **Pieces** (math_spec §2.2): segment k is \f$P_k + \tau t_k\f$, \f$\tau \in [0, \ell_k]\f$,
/// station \f$S_k + \tau\f$; open curves add the backward ray \f$P_0 + \tau t_0\f$,
/// \f$\tau \le 0\f$, and the forward ray \f$P_{N-1} + \tau t_{N-2}\f$, \f$\tau \ge 0\f$, each
/// clamped only at its anchor. Closed curves add the closing segment \f$P_{N-1} \to P_0\f$ and
/// have \f$s^\star \in [0, L)\f$.
///
/// **Ownership and tangent** (math_spec §2.3, sec-wedge):
/// - a segment or ray owns x when its unclamped \f$\hat\tau\f$ lies in its closed interval;
/// - an interior vertex \f$P_v\f$ (open: v = 1..N−2; closed: every vertex) owns x iff
///   \f$a_1 = (x - P_v)^\top t_{v-1} > 0\f$ and \f$a_2 = -(x - P_v)^\top t_v > 0\f$; collinear
///   joints, including the ray joints, never form wedges;
/// - the closest point is the owner with the smallest D; ties within kTieTolerance go to the
///   hint (WindowScan: foot nearest `hint.x_star`; Hinted: smallest wrapped
///   \f$\lvert s - s_{\mathrm{prev}}\rvert\f$) or, without a valid hint, to the smallest
///   \f$\mathrm{dir}\cdot s\f$ (math_spec §2.4; oracle O8);
/// - wedge tangent \f$T = s_w J n\f$, \f$s_w = \operatorname{sgn}((Jn)^\top(t_{v-1} + t_v))\f$;
/// - hairpin (\f$\lVert t_{v-1} + t_v\rVert < \varepsilon_{\mathrm{hp}}\f$):
///   \f$T = \operatorname{unit}((I - nn^\top)T_{\mathrm{last}})\f$ with
///   `EvalContext::last_tangent`, else \f$+Jn\f$; sets `hairpin` and Status::kHairpin;
/// - on the curve (\f$D < \varepsilon_D\f$): \f$n = 0\f$, \f$T\f$ = the owning piece's tangent,
///   \f$\operatorname{unit}(t_{v-1} + t_v)\f$ exactly at an interior vertex (\f$t_v\f$ for a
///   hairpin vertex).
///
/// A clamped edge point of the candidate set is never returned (it is not a projection and would
/// break \f$n^\top T = 0\f$): the scan extends instead (math_spec §2.4).
///
/// **Allocation.** assign() reuses capacity: after reserve() (or a first assign of the same size)
/// it never allocates, so the plugin can re-assign its window every cycle. closestPoint() never
/// allocates.
class PolylineCurve final : public Curve
{
public:
  /// @brief An empty curve with the default Hinted policy (CurveOptions defaults).
  PolylineCurve() noexcept;

  /// @brief An empty curve with the given search policy.
  explicit PolylineCurve(const SearchPolicy & policy) noexcept;

  /// @brief Replaces the vertices, reusing capacity.
  ///
  /// Vertices closer than kEpsD to the previously kept vertex are dropped (and, when `closed`,
  /// a last vertex within kEpsD of the first), so every segment has a positive length. Callers
  /// dedupe at kDedupeDistance first (conditioning.hpp `dedupe`); this is only a safety net.
  /// @param vertices Polyline vertices in the curve frame.
  /// @param closed True for a closed polyline (\f$P_N \equiv P_0\f$).
  /// @return kOk; kTooFew if fewer than 2 vertices remain (closed: fewer than 3); kNonFinite if
  /// a vertex is not finite. On failure the curve is empty (size() == 0).
  AssignStatus assign(Span<const Vec2> vertices, bool closed) noexcept;

  /// @brief Reserves capacity for `vertex_count` vertices, so later assign() calls of up to that
  /// size never allocate.
  void reserve(std::size_t vertex_count);

  /// @brief Empties the curve (keeps capacity).
  void clear() noexcept;

  /// @brief Sets the search policy (plugin: SearchPolicy::windowScan; follower:
  /// SearchPolicy::hinted).
  void setSearchPolicy(const SearchPolicy & policy) noexcept;

  /// @brief The search policy.
  const SearchPolicy & searchPolicy() const noexcept;

  /// @brief Records the spec this curve was built from (makeCurve only); returned by spec().
  void setSpec(std::shared_ptr<const CurveSpec> spec) noexcept;

  /// @brief Number of vertices N (0 when empty).
  std::size_t size() const noexcept;
  /// @brief True iff no polyline is assigned.
  bool empty() const noexcept;
  /// @brief Number of segments: N − 1 (open) or N (closed).
  std::size_t segmentCount() const noexcept;
  /// @brief The vertices \f$P_0 \dots P_{N-1}\f$.
  Span<const Vec2> vertices() const noexcept;
  /// @brief Station \f$S_k\f$ of vertex k (\f$S_0 = 0\f$); \f$S_N = L\f$ for k = N on a closed
  /// curve. Precondition: k < size(), or k == size() when closed.
  double station(std::size_t k) const noexcept;
  /// @brief Unit tangent \f$t_k\f$ of segment k. Precondition: k < segmentCount().
  Vec2 segmentTangent(std::size_t k) const noexcept;
  /// @brief Length \f$\ell_k\f$ of segment k. Precondition: k < segmentCount().
  double segmentLength(std::size_t k) const noexcept;

  ClosestPoint closestPoint(
    const Vec2 & x, const EvalContext & context, const CurveHint & hint) const noexcept override;
  bool isClosed() const noexcept override;
  /// @brief Always false: polylines are static.
  bool isTimeVarying() const noexcept override;
  double length(double t) const noexcept override;
  /// @brief Always zero: polylines are static.
  Vec2 velocity(const ClosestPoint & cp, double t) const noexcept override;
  Vec2 pointAt(double s, double t) const noexcept override;
  Vec2 tangentAt(double s, double t) const noexcept override;
  std::size_t sampleStations(
    double s0, double s1, double ds, double t, Span<Vec2> out) const noexcept override;
  const CurveSpec * spec() const noexcept override;
  /// @brief \f$\bar\ell = L/\f$ segmentCount() (0 when empty), from the stations stored by assign().
  double vertexSpacing() const noexcept override;

private:
  std::vector<Vec2> vertices_;
  std::vector<Vec2> tangents_;
  std::vector<double> lengths_;
  std::vector<double> stations_;
  bool closed_{false};
  SearchPolicy policy_;
  std::shared_ptr<const CurveSpec> spec_;
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__POLYLINE_CURVE_HPP_
