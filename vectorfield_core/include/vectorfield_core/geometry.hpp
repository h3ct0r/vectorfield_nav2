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
/// @brief Closest-point types shared by every curve: hint, result, search policy and evaluation
/// context (math_spec §2.1–§2.5).

#ifndef VECTORFIELD_CORE__GEOMETRY_HPP_
#define VECTORFIELD_CORE__GEOMETRY_HPP_

#include <cmath>
#include <cstdint>
#include <type_traits>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Search state carried across cycles in `TrackerState::hint` (plan §3.1).
///
/// - `s` holds **the curve's own parameter**: arc length for polylines, \f$u\f$ for parametric
///   curves (frame-invariant). Only the follower's Hinted polyline search and ParametricCurve
///   read `s` and `segment`.
/// - `x_star` is the previous foot point. **Plugin mode is position-only**: the window is
///   re-pruned every cycle, so WindowScan reads only `x_star` (ties go to the foot nearest it)
///   and ignores `s` and `segment` (math_spec §2.4).
/// - In TrackerState, `x_star` is in the evaluation frame; the Tracker maps it into the curve
///   frame before calling Curve::closestPoint().
/// - `valid == false` means initialisation: Hinted and parametric searches run the global stage
///   and adopt its result unconditionally; ties go to the start of traversal.
struct CurveHint
{
  /// Previous foot point \f$x^\star_{\mathrm{prev}}\f$.
  Vec2 x_star{Vec2::Zero()};
  /// Previous curve parameter (s for polylines, u for parametric curves).
  double s{0.0};
  /// Previous owning segment index (polylines), −1 if unknown.
  int segment{-1};
  /// False before the first search.
  bool valid{false};
};

/// @brief Which kind of piece owns a closest point (math_spec §2.2–§2.3).
enum class PieceKind : std::uint8_t
{
  /// No result (empty curve, non-finite input).
  kNone = 0,
  /// The interior or an end of a polyline segment.
  kSegment = 1,
  /// A polyline vertex that owns x (a wedge, \f$a_1 > 0\f$ and \f$a_2 > 0\f$), or x exactly at
  /// a vertex.
  kVertex = 2,
  /// The backward ray of an open curve (\f$s < 0\f$).
  kBackwardRay = 3,
  /// The forward ray of an open curve (\f$s > L\f$).
  kForwardRay = 4,
  /// The interior of a parametric curve.
  kParametric = 5,
};

/// @brief Result of a closest-point search (math_spec §2.1, §2.3, §2.4).
///
/// Curves return it in their own frame; the Tracker maps it with transformed(). Guarantees for a
/// valid result: \f$\lVert T\rVert = 1\f$; \f$n^\top T = 0\f$ to rounding (oracle O1,
/// \f$10^{-12}\f$); \f$n = 0\f$ iff \f$D < \varepsilon_D\f$ (then \f$T\f$ is the owning piece's
/// tangent, \f$\operatorname{unit}(t_{v-1} + t_v)\f$ exactly at an interior vertex); otherwise
/// \f$n = (x - x^\star)/D\f$ (parametric curves re-orthogonalise, math_spec §2.5).
struct ClosestPoint
{
  /// Foot point \f$x^\star\f$.
  Vec2 x_star{Vec2::Zero()};
  /// Unit tangent \f$T\f$ at the foot, in the curve's parameter direction (not multiplied by
  /// dir). In a wedge: \f$s_w J n\f$; hairpin rule of math_spec §2.3.
  Vec2 T{Vec2::UnitX()};
  /// Unit normal \f$n\f$, or zero on the curve.
  Vec2 n{Vec2::Zero()};
  /// Distance \f$D \ge 0\f$.
  double D{kInf};
  /// Arc-length station \f$s^\star\f$ of the foot at the evaluation time: measured from the
  /// curve's start (P0 or \f$u_i\f$); negative on the backward ray, beyond the length on the
  /// forward ray; in \f$[0, L)\f$ for closed curves.
  double s_star{0.0};
  /// The curve's own parameter at the foot: equals `s_star` for polylines; \f$u^\star\f$ for
  /// parametric curves (clamped to the domain end on a ray). Becomes CurveHint::s.
  double u_star{0.0};
  /// Polylines: the owning segment k, or the vertex index v for kVertex, −1 for the backward ray
  /// and N−1 for the forward ray. −1 for parametric curves. Becomes CurveHint::segment.
  int segment{-1};
  /// Owning piece kind.
  PieceKind piece{PieceKind::kNone};
  /// True iff the foot lies on a ray (piece is kBackwardRay or kForwardRay).
  bool on_ray{false};
  /// True iff a vertex owns x with \f$D \ge \varepsilon_D\f$ (the wedge tangent applies).
  bool in_wedge{false};
  /// True iff the hairpin rule produced T (also sets Status::kHairpin).
  bool hairpin{false};
  /// False when no closest point exists (empty curve, non-finite input); other fields are then
  /// meaningless.
  bool valid{false};
  /// Status bits raised by the search: Status::kHairpin, Status::kAmbiguousClosest (an adopted
  /// global jump), Status::kNonFinite (non-finite evaluation, parametric speed floor).
  std::uint32_t status_bits{Status::kNone};
};

/// @brief Maps a curve-frame closest point into the evaluation frame: \f$x^\star\f$ by
/// `T_eval_curve`, \f$T\f$ and \f$n\f$ by its rotation; D, stations and flags are invariant.
inline ClosestPoint transformed(const ClosestPoint & cp, const Isometry2 & T_eval_curve) noexcept
{
  ClosestPoint out = cp;
  out.x_star = T_eval_curve.apply(cp.x_star);
  out.T = T_eval_curve.rotate(cp.T);
  out.n = T_eval_curve.rotate(cp.n);
  return out;
}

/// @brief The hint for the next search from a result in the evaluation frame.
inline CurveHint makeHint(const ClosestPoint & cp_eval) noexcept
{
  CurveHint hint;
  hint.x_star = cp_eval.x_star;
  hint.s = cp_eval.u_star;
  hint.segment = cp_eval.segment;
  hint.valid = cp_eval.valid;
  return hint;
}

/// @brief True iff two candidate distances tie (\f$\lvert D_a - D_b\rvert \le 10^{-9}\f$ m;
/// math_spec §2.4, Appendix C item 1).
inline bool isTie(double D_a, double D_b) noexcept { return std::abs(D_a - D_b) <= kTieTolerance; }

/// @brief Adoption rule of a global candidate (math_spec §2.4 [Design], Hinted and
/// ParametricCurve): adopt iff \f$D_{\mathrm{glob}} < D_{\mathrm{loc}} - h_g\f$ and
/// \f$\operatorname{arc} \le 3\,\mathrm{chord} + 2(D_{\mathrm{loc}} + D_{\mathrm{glob}})\f$.
/// @param D_loc Distance of the local candidate.
/// @param D_glob Distance of the global candidate.
/// @param arc Arc length between the two feet (closed curves: the shorter way).
/// @param chord \f$\lVert x^\star_{\mathrm{loc}} - x^\star_{\mathrm{glob}}\rVert\f$.
/// @param hysteresis \f$h_g\f$ (`global_search_hysteresis`).
inline bool adoptGlobalCandidate(
  double D_loc, double D_glob, double arc, double chord, double hysteresis) noexcept
{
  return D_glob < D_loc - hysteresis && arc <= 3.0 * chord + 2.0 * (D_loc + D_glob);
}

/// @brief Closest-point search policy of a PolylineCurve (math_spec §2.4, sec-search).
///
/// - **WindowScan** (plugin): exact over every piece whose arc-length interval meets
///   [s_min, s_max] (absolute stations from P0; the plugin uses \f$[-(d + 0.3), d + 0.5]\f$),
///   always including the backward ray, extended piece by piece while the winning foot is at
///   the far end of the last included piece (up to the rays). No global stage. Position-only
///   hint.
/// - **Hinted** (follower): the pieces meeting \f$[s_{\mathrm{prev}} - w/2, s_{\mathrm{prev}} +
///   w/2]\f$ (wrapping only when closed), extended like WindowScan; the global stage (an exact
///   scan of every piece) runs at initialisation (adopted unconditionally) and when
///   \f$D_{\mathrm{loc}} > D_{\mathrm{trig}}\f$, then adoptGlobalCandidate().
struct SearchPolicy
{
  /// Policy kind.
  enum class Kind : std::uint8_t
  {
    /// Plugin window scan.
    kWindowScan = 0,
    /// Follower hinted search.
    kHinted = 1,
  };

  /// Policy kind; Hinted by default.
  Kind kind{Kind::kHinted};
  /// WindowScan: lower station bound of the initial candidate set (m).
  double s_min{0.0};
  /// WindowScan: upper station bound of the initial candidate set (m).
  double s_max{0.0};
  /// Hinted: window \f$w\f$ (m).
  double window{1.0};
  /// Hinted: global-search trigger \f$D_{\mathrm{trig}}\f$ (m).
  double global_trigger{0.5};
  /// Hinted: adoption hysteresis \f$h_g\f$ (m).
  double global_hysteresis{0.10};

  /// @brief A WindowScan policy over [s_min, s_max].
  static SearchPolicy windowScan(double s_min, double s_max) noexcept
  {
    SearchPolicy policy;
    policy.kind = Kind::kWindowScan;
    policy.s_min = s_min;
    policy.s_max = s_max;
    return policy;
  }

  /// @brief A Hinted policy with explicit window, trigger and hysteresis.
  static SearchPolicy hinted(double window, double trigger, double hysteresis) noexcept
  {
    SearchPolicy policy;
    policy.kind = Kind::kHinted;
    policy.window = window;
    policy.global_trigger = trigger;
    policy.global_hysteresis = hysteresis;
    return policy;
  }

  /// @brief A Hinted policy from the follower's CurveOptions.
  static SearchPolicy hinted(const CurveOptions & options) noexcept
  {
    return hinted(
      options.search_window, options.global_search_trigger, options.global_search_hysteresis);
  }
};

/// @brief Per-call context of a closest-point search.
struct EvalContext
{
  /// Evaluation time \f$t\f$ (s); ignored by static curves.
  double t{0.0};
  /// Traversal direction \f$\mathrm{dir} = \pm 1\f$; breaks initialisation ties toward the
  /// smallest \f$\mathrm{dir}\cdot s\f$ (math_spec §2.4).
  int direction{1};
  /// \f$T_{\mathrm{last}}\f$ for the hairpin rule, **in the curve frame** and in the curve's
  /// parameter direction (`TrackerState::last_tangent` rotated by the Tracker). A norm below
  /// \f$\varepsilon_{\mathrm{hp}}\f$ means none, and the rule uses \f$+Jn\f$ (math_spec §2.3).
  Vec2 last_tangent{Vec2::Zero()};
};

static_assert(std::is_trivially_destructible_v<CurveHint>, "CurveHint is TrackerState data");

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__GEOMETRY_HPP_
