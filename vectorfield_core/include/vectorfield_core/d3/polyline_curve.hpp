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
/// @brief 3D core (experimental): polylines in \f$\mathbb R^3\f$ (math_spec §2.2, §14.2).

#ifndef VECTORFIELD_CORE__D3__POLYLINE_CURVE_HPP_
#define VECTORFIELD_CORE__D3__POLYLINE_CURVE_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/span.hpp"

namespace vectorfield_core::d3
{

/// @brief A polyline \f$P_0, \dots, P_{N-1}\f$ in \f$\mathbb R^3\f$, open (with rays) or closed.
///
/// Pieces, projections and stations are those of math_spec §2.2. The search is the Hinted policy
/// of §2.4: the pieces meeting \f$[s_{\mathrm{prev}} - w/2, s_{\mathrm{prev}} + w/2]\f$, extended
/// one piece at a time while the winner sits at the window's edge; an exact scan of every piece
/// at initialisation (adopted) or when \f$D_{\mathrm{loc}} > D_{\mathrm{trig}}\f$
/// (adoptGlobalCandidate()). Ownership, the wedge blend and the hairpin rule are math_spec §14.2.
/// Static only. The constructor dedupes consecutive vertices closer than 1 mm (and a closing
/// duplicate); there is no resampling or filleting (the projection is exact on segments).
class PolylineCurve3 final : public Curve3
{
public:
  /// @brief Builds a polyline.
  /// @param points Vertices (curve frame).
  /// @param closed True for a closed polyline (\f$P_N \equiv P_0\f$).
  /// @param options Search window, trigger, hysteresis and `max_curve_points`.
  /// @throws CurveError (kInvalidCurve): non-finite vertex, fewer than 2 distinct vertices (3
  /// when closed), more than `max_curve_points`, or invalid options.
  PolylineCurve3(const std::vector<Vec3> & points, bool closed, const CurveOptions & options = {});

  ClosestPoint3 closestPoint(
    const Vec3 & x, const EvalContext3 & context, const CurveHint3 & hint) const noexcept override;
  bool isClosed() const noexcept override { return closed_; }
  bool isTimeVarying() const noexcept override { return false; }
  double length(double t) const noexcept override;
  Vec3 velocity(const ClosestPoint3 & cp, double t) const noexcept override;
  Vec3 pointAt(double s, double t) const noexcept override;
  Vec3 tangentAt(double s, double t) const noexcept override;
  const CurveSpec3 * spec() const noexcept override { return spec_.get(); }

  /// @brief The vertices after dedupe.
  Span<const Vec3> vertices() const noexcept { return Span<const Vec3>(points_); }
  /// @brief Number of segments (closed: N; open: N − 1).
  int segmentCount() const noexcept { return static_cast<int>(tangents_.size()); }
  /// @brief Records the spec this curve was built from (makeCurve3 only).
  void setSpec(std::shared_ptr<const CurveSpec3> spec) noexcept { spec_ = std::move(spec); }

private:
  struct Candidate;
  // Projection of x onto segment k (unclamped tau is kept for ownership).
  Candidate project(const Vec3 & x, int k) const noexcept;
  Candidate projectRay(const Vec3 & x, bool forward) const noexcept;
  // Best candidate over segments [k0, k1] (wrapping for closed curves), rays included when open
  // and the range touches an end.
  Candidate scan(
    const Vec3 & x, int k0, int count, const EvalContext3 & context,
    const CurveHint3 & hint) const noexcept;
  ClosestPoint3 finish(
    const Vec3 & x, const Candidate & c, const EvalContext3 & context) const noexcept;
  int segmentAt(double s) const noexcept;

  std::vector<Vec3> points_;
  std::vector<Vec3> tangents_;
  std::vector<double> lengths_;
  // stations_[k] = S_k, k = 0..segments (S_segments = L).
  std::vector<double> stations_;
  bool closed_{false};
  CurveOptions options_;
  std::shared_ptr<const CurveSpec3> spec_;
};

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__POLYLINE_CURVE_HPP_
