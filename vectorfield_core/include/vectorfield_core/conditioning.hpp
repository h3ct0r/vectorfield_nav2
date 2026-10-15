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
/// @brief Pure path-conditioning helpers: follower polylines and the plugin window
/// (plan §5.1–§5.3; math_spec §2.8, sec-conditioning).
///
/// Implemented in src/conditioning.cpp (M1 wave 2, polyline). Outputs go to caller-owned
/// `std::vector`s that are cleared and refilled: once their capacity suffices they never
/// allocate, so the plugin's per-cycle helpers (dedupe, tailCondition, kappaPath, onePoseWindow)
/// are allocation-free in steady state. Unless stated otherwise, an output vector must not
/// alias an input span.

#ifndef VECTORFIELD_CORE__CONDITIONING_HPP_
#define VECTORFIELD_CORE__CONDITIONING_HPP_

#include <array>
#include <cstddef>
#include <vector>

#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Drops consecutive vertices closer than `min_spacing` (math_spec §2.8).
///
/// Walks the input keeping a vertex only if it is at least `min_spacing` from the last kept
/// one; the first vertex is always kept. The last vertex is always kept too: if it is within
/// `min_spacing` of the last kept vertex, it replaces that vertex (unless that vertex is the
/// first), so endpoints and the goal never move. When `closed`, a last vertex within
/// `min_spacing` of the first is dropped (the closing duplicate).
/// @param in Input vertices.
/// @param closed Closed polyline.
/// @param out Output vertices (cleared first).
/// @param min_spacing Minimum spacing (m), default 1 mm.
void dedupe(
  Span<const Vec2> in, bool closed, std::vector<Vec2> * out,
  double min_spacing = kDedupeDistance) noexcept;

/// @brief Follower corner fillet (math_spec §2.8 step 2), when \f$\delta > 0\f$.
///
/// At each interior vertex (closed: every vertex) with turning angle \f$\theta \in (0, \pi)\f$
/// the corner is replaced by a circular arc of radius \f$R = \min(R_\delta, R_\ell)\f$ with
/// \f$R_\delta = \delta/(1/\cos(\theta/2) - 1)\f$ and \f$R_\ell = \min(\ell_{v-1},
/// \ell_v)/(2\tan(\theta/2))\f$, tangent at \f$P_v - R\tan(\theta/2)\,t_{v-1}\f$ and
/// \f$P_v + R\tan(\theta/2)\,t_v\f$, centred at \f$P_v + \frac{R}{\cos(\theta/2)}
/// \operatorname{unit}(t_v - t_{v-1})\f$; its deviation from the vertex is at most \f$\delta\f$.
/// The arc is discretised with points at most `arc_spacing` apart along the arc. Collinear
/// vertices and exact reversals (\f$\theta = \pi\f$) are kept as they are. Endpoints of open
/// polylines are preserved. Not on a hot path (once per goal).
/// @param in Deduplicated vertices.
/// @param closed Closed polyline.
/// @param max_corner_deviation \f$\delta\f$ (m); <= 0 copies the input unchanged.
/// @param arc_spacing Maximum spacing of the arc points (m), > 0.
/// @param out Output vertices (cleared first).
void filletCorners(
  Span<const Vec2> in, bool closed, double max_corner_deviation, double arc_spacing,
  std::vector<Vec2> * out);

/// @brief Follower resampling (math_spec §2.8 step 3) that never moves the geometry.
///
/// Every input vertex is kept (corners, fillet tangency points, endpoints), and each segment k
/// is split into \f$\max(1, \lceil\ell_k/\mathrm{spacing}\rceil)\f$ equal pieces, so the result
/// traces exactly the input polyline with spacing at most `spacing`. Closed polylines include
/// the closing segment and do not repeat the first vertex. Not on a hot path.
/// @param in Deduplicated (and filleted) vertices.
/// @param closed Closed polyline.
/// @param spacing Maximum spacing (m), > 0.
/// @param out Output vertices (cleared first).
void resample(Span<const Vec2> in, bool closed, double spacing, std::vector<Vec2> * out);

/// @brief Result of endTangent().
struct EndTangent
{
  /// \f$\hat T_{\mathrm{end}}\f$ (unit), meaningful when `valid`.
  Vec2 tangent{Vec2::UnitX()};
  /// True when a pose at least \f$\ell_c\f$ from the last pose was found.
  bool found_chord{false};
  /// True when `tangent` is defined (a chord was found, or the first pose differs from the last
  /// by at least kDedupeDistance).
  bool valid{false};
};

/// @brief Plugin end tangent of a raw plan, in the plan frame (math_spec §2.8; plan §5.1).
///
/// Walks back from the last pose \f$g\f$ to the first pose \f$p\f$ with \f$\lVert g - p\rVert \ge
/// \ell_c\f$ and returns \f$\operatorname{unit}(g - p)\f$ with `found_chord`. Otherwise returns
/// \f$\operatorname{unit}(g - p_{\mathrm{first}})\f$ (valid, `found_chord` false) or nothing
/// (all poses within 1 mm of \f$g\f$). The caller applies the rule "keep the previous
/// \f$\hat T_{\mathrm{end}}\f$ while goal_seq is unchanged" when `found_chord` is false.
/// @param plan Raw plan positions (plan frame).
/// @param chord_length \f$\ell_c\f$ (chordLength()).
EndTangent endTangent(Span<const Vec2> plan, double chord_length) noexcept;

/// @brief Plugin tail conditioning of a window that ends at the goal (math_spec §2.8).
///
/// Copies `window`, removes the trailing run of vertices (before \f$W_{\mathrm{back}}\f$) that are
/// closer than \f$\ell_c\f$ to \f$W_{\mathrm{back}}\f$, also removes a remaining last vertex within
/// kDedupeDistance of the virtual vertex, then inserts the virtual vertex
/// \f$W_{\mathrm{back}} - \ell_c\,\hat T\f$ before \f$W_{\mathrm{back}}\f$. The last segment of the
/// output is therefore exactly \f$[W_{\mathrm{back}} - \ell_c\hat T, W_{\mathrm{back}}]\f$, at
/// least \f$\ell_c\f$ long and fixed across cycles, and its index is `out->size() - 2`.
/// Precondition (checked by the caller): \f$\lVert W_{\mathrm{back}} -
/// \mathrm{global\_goal}\rVert\f$ < kWindowAtGoalDistance.
/// @param window Window vertices (costmap frame), at least one.
/// @param end_tangent \f$R_{\mathrm{plan}\to\mathrm{costmap}}\hat T_{\mathrm{end}}\f$ (unit,
/// costmap frame).
/// @param chord_length \f$\ell_c\f$.
/// @param out Conditioned window (cleared first).
void tailCondition(
  Span<const Vec2> window, const Vec2 & end_tangent, double chord_length,
  std::vector<Vec2> * out) noexcept;

/// @brief Plugin chord decimation of a raw plan for cusp detection (math_spec §2.8 [Design]).
///
/// Keeps the first pose; keeps a pose only if it is at least \f$\ell_c\f$ from the last kept
/// vertex; before appending the last pose, pops kept vertices (never the first) while the last
/// kept vertex is within \f$\ell_c\f$ of the last pose; then appends the last pose.
/// @param plan Raw plan positions (plan frame).
/// @param chord_length \f$\ell_c\f$.
/// @param out Decimated vertices (cleared first).
void decimateChords(Span<const Vec2> plan, double chord_length, std::vector<Vec2> * out) noexcept;

/// @brief True iff two consecutive chords of a decimated plan have a negative dot product (a
/// cusp; the plugin then throws InvalidPath until M7).
bool hasCusp(Span<const Vec2> decimated) noexcept;

/// @brief Plugin one-pose rule (math_spec §2.8): \f$[p, p_{\mathrm{last}}]\f$, or
/// \f$[p_{\mathrm{last}} - 0.01\,\mathbf h(\psi_{\mathrm{last}}), p_{\mathrm{last}}]\f$ when
/// \f$\lVert p - p_{\mathrm{last}}\rVert\f$ < kDedupeDistance.
/// @param base Base position \f$p\f$ (costmap frame).
/// @param last Last window pose \f$(p_{\mathrm{last}}, \psi_{\mathrm{last}})\f$ (costmap frame).
std::array<Vec2, 2> onePoseWindow(const Vec2 & base, const Pose2 & last) noexcept;

/// @brief \f$\kappa_{\mathrm{path}} = \max_v \theta_v/((\ell_{v-1} + \ell_v)/2)\f$ over interior
/// vertices (closed: every vertex), with \f$\theta_v = \lvert\operatorname{wrap}(\angle t_v -
/// \angle t_{v-1})\rvert\f$; rays excluded (math_spec §2.8). 0 for fewer than 3 vertices.
/// @param vertices Conditioned window or curve vertices (deduplicated).
/// @param closed Closed polyline.
double kappaPath(Span<const Vec2> vertices, bool closed) noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__CONDITIONING_HPP_
