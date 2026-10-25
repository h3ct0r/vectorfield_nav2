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
/// (math_spec §2.8, sec-conditioning).
///
/// Implemented in src/conditioning.cpp. Outputs go to caller-owned
/// `std::vector`s that are cleared and refilled: once their capacity suffices they never
/// allocate, so the plugin's per-cycle helpers (dedupe, tailCondition, kappaPath, onePoseWindow)
/// are allocation-free in steady state. Unless stated otherwise, an output vector must not
/// alias an input span.

#ifndef VECTORFIELD_CORE__CONDITIONING_HPP_
#define VECTORFIELD_CORE__CONDITIONING_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
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

/// @brief Plugin end tangent of a raw plan, in the plan frame (math_spec §2.8).
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

/// @brief Removes the sub-chord backtracks of a plugin window (math_spec §2.8 [Design, 0.2.1]).
///
/// Walks the vertices keeping a stack: before a vertex is pushed, the top is dropped while the
/// polyline would turn there by more than 90° and one of the top's two segments is shorter than
/// `min_length` (\f$\ell_c/2\f$). A NavigateThroughPoses junction of NavFn legs steps back to the
/// next leg's start cell (up to 0.71 cell); at a sharp corner that backward step would make the
/// field there point backward and hold ROTATE_TO_FIELD. A sharp corner sampled finer than
/// `min_length` is cut by a few centimetres. The first and the last vertex are kept; in place.
/// @param points Deduplicated window vertices (rewritten).
/// @param min_length Segment length below which a turn of more than 90° is removed (m).
void despike(std::vector<Vec2> * points, double min_length) noexcept;

/// @brief Plugin chord decimation of a raw plan for cusp detection (math_spec §2.8 [Design]).
///
/// Keeps the first pose; keeps a pose only if it is at least \f$\ell_c\f$ from the last kept
/// vertex; before appending the last pose, pops kept vertices (never the first) while the last
/// kept vertex is within \f$\ell_c\f$ of the last pose; then appends the last pose.
/// @param plan Raw plan positions (plan frame).
/// @param chord_length \f$\ell_c\f$.
/// @param out Decimated vertices (cleared first).
void decimateChords(Span<const Vec2> plan, double chord_length, std::vector<Vec2> * out) noexcept;

/// @brief True iff two consecutive chords of a decimated plan have a negative dot product: a
/// turn of more than 90°, a cusp *candidate*. findCusps() decides which candidates are cusps (a
/// sharp forward corner is not one).
bool hasCusp(Span<const Vec2> decimated) noexcept;

/// @brief A yaw is aligned with a plan step when \f$\lvert\cos(\psi_k - \angle(p_{k+1} -
/// p_k))\rvert\f$ is at least this (within 30°, either way; math_spec §2.8 [Design, 0.2.1]).
inline constexpr double kHeadingAlignCos = 0.8660254037844386;
/// @brief A plan has headings only if aligned yaws cover at least this fraction of its length.
inline constexpr double kHeadingAlignedFraction = 0.9;
/// @brief Without headings, a candidate is a reversal only if its legs, fitted away from the turn,
/// turn by at least 177° (\f$\cos 177°\f$; math_spec §2.8 [Design, 0.2.1]): the headingless
/// reversals Nav2's planners emit (Smac along \f$\pm x\f$) are exact, and 7° above a 170° corner
/// absorb a grid planner's sub-cell wiggle.
inline constexpr double kReversalTurnCos = -0.9986295347545738;

/// @brief True iff the plan's yaws say which way the robot drives (math_spec §2.8 [Design,
/// 0.2.1]).
///
/// Over the steps \f$s_k = p_{k+1} - p_k\f$ (\f$k \le n-2\f$, so the goal yaw is unused) at least
/// 0.1 mm long: yaws aligned with their step (kHeadingAlignCos) cover at least
/// kHeadingAlignedFraction of the length, and some run of consecutive aligned steps with a yaw
/// other than exactly 0 is at least \f$\ell_c\f$ long. NavFn, Theta\* and short Smac 2D plans
/// leave every pose at the identity and NavigateThroughPoses adds one oriented pose per waypoint,
/// so they have none; Smac Hybrid and Lattice (the vehicle yaw) and smoothed Smac 2D plans (yaws
/// along the path) have them.
/// @param plan Raw plan poses (plan frame).
/// @param chord_length \f$\ell_c\f$.
bool planHasHeadings(Span<const Pose2> plan, double chord_length) noexcept;

/// @brief The cusps of a raw plan: where the robot must change its driving direction (math_spec
/// §2.8 [Design, 0.2.1]).
///
/// Candidates come from decimateChords() and hasCusp()'s test; each is moved to the raw pose
/// between the neighbouring chord vertices with the largest projection on the incoming chord, and
/// its legs end at the neighbouring candidates (or the plan's ends). With headings
/// (planHasHeadings()), the drive sign of each leg is that of \f$\sum_k s_k^\top \mathbf
/// h(\psi_k)\f$ over its first \f$2\ell_c\f$ when the magnitude is at least half the length: a
/// candidate is a cusp iff the two signs differ. Otherwise (no headings, or a leg the yaws do not
/// decide) it is a cusp iff the plan doubles back: over \f$L = \min(4\ell_c, L_{\mathrm{in}},
/// L_{\mathrm{out}}) \ge 2\ell_c\f$, the least-squares lines of the legs between \f$\ell_c\f$
/// and \f$L\f$ from the turn (clear of start-cell and goal hooks) turn by at least 177°
/// (kReversalTurnCos) and the outgoing leg's centroid lies within \f$\ell_c/2\f$ (a cell) of the
/// incoming line. A sharp forward corner (tested up to 170°) is no cusp.
/// @param plan Raw plan poses (plan frame).
/// @param chord_length \f$\ell_c\f$.
/// @param cusps Raw indices of the cusps in plan order, strictly between the first and the last
/// pose (cleared first; also the work buffer).
void findCusps(
  Span<const Pose2> plan, double chord_length, std::vector<std::size_t> * cusps) noexcept;

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

/// @brief One segment of a raw plan between cusps (math_spec §9.4 [Design]).
struct PlanSegment
{
  /// Raw index of the first pose (0, or the previous segment's cusp).
  std::size_t first{0};
  /// Raw index of the last pose: the cusp that ends the segment, or the plan's last pose.
  std::size_t last{0};
  /// Drive sign \f$q\f$: +1 body \f$+x\f$ ahead, −1 body \f$-x\f$ ahead.
  std::int8_t drive_sign{1};
  /// Path length of the raw poses first..last (m).
  double length{0.0};
};

/// @brief Splits a raw plan at its cusps and gives each segment a drive sign (math_spec §2.8,
/// §9.4 [Design]).
///
/// Cusps come from findCusps(). With headings (planHasHeadings()), a segment's sign is that of
/// \f$\sum_k (p_{k+1} - p_k)^\top \mathbf h(\psi_k)\f$ when its magnitude is at least half the
/// segment length; otherwise, and without headings, the first segment takes `default_sign` and
/// the sign alternates at cusps.
/// @param plan Raw plan poses (plan frame).
/// @param chord_length \f$\ell_c\f$.
/// @param default_sign Sign of a first segment that headings do not decide (−1 iff
/// `drive_backward`).
/// @param work Work buffer (the cusps).
/// @param out Segments in plan order (cleared first): one for a plan without cusps, none for an
/// empty plan.
/// @return True iff the plan has headings.
bool segmentPlan(
  Span<const Pose2> plan, double chord_length, int default_sign, std::vector<std::size_t> * work,
  std::vector<PlanSegment> * out) noexcept;

/// @brief The segment of a new plan of the same goal to start with (math_spec §9.4 replans): 0,
/// or 1 when segment 0 is shorter than `hysteresis` (\f$2\ell_c\f$) and runs against
/// `active_sign` while segment 1 runs with it (the robot is at that cusp; a ±1 cell jitter of
/// the cusp must not flip the direction twice).
/// @param segments Segments of the new plan.
/// @param active_sign Drive sign in force (0: none, gives 0).
/// @param hysteresis Length below which a contrary first segment is skipped (m).
std::size_t firstActiveSegment(
  Span<const PlanSegment> segments, int active_sign, double hysteresis) noexcept;

/// @brief True iff the plan turns in place before its last pose: two consecutive poses less than
/// 0.1 mm apart (the rule of Nav2's `findFirstPathConstraint`) whose yaws differ by more than
/// 1 mrad, the last pose excluded (planners append the goal yaw there).
bool hasInPlaceRotation(Span<const Pose2> plan) noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__CONDITIONING_HPP_
