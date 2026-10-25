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
/// @brief The control-barrier-function filter of `obstacle_mode: cbf` (math_spec §6.7 [Design]):
/// the constrained sites at a point and the minimal modification of the field. The Tracker
/// composes them (evaluation point, \f$\delta_{\mathrm{safe}}\f$, the hold). All functions are
/// `noexcept` and allocation-free.

#ifndef VECTORFIELD_CORE__CBF_HPP_
#define VECTORFIELD_CORE__CBF_HPP_

#include <cstdint>

#include "vectorfield_core/field.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Half-width (cells) of the window whose features give the extra sites (7 × 7 cells).
inline constexpr int kCbfWindowCells = 3;
/// @brief Two sites are the same obstacle when their normals are within 45°.
inline constexpr double kCbfSameObstacleCos = 0.70710678118654752;
/// @brief The hold sets below this \f$\lVert F^\star\rVert / \lVert F\rVert\f$ ...
inline constexpr double kCbfHoldRatio = 0.05;
/// @brief ... and releases at this one (math_spec §6.7, §10.4).
inline constexpr double kCbfReleaseRatio = 0.10;

/// @brief One obstacle site of the filter.
struct CbfSite
{
  /// Nearest point of the site (the query's \f$x_o^\star\f$, or a site centre).
  Vec2 x_o{Vec2::Zero()};
  /// Unit normal \f$(x - x_o)/D\f$.
  Vec2 n{Vec2::Zero()};
  /// Distance \f$D\f$ (m, > 0).
  double D{kInf};
};

/// @brief The sites the filter constrains at a grid-frame point (math_spec §6.7): site 1 is the
/// continuous query of math_spec §6.1; each further site is the nearest site centre among the
/// features of the \f$(2w+1)^2\f$ cells around x (w = kCbfWindowCells) whose normal is more
/// than 45° from the normals already taken. Sites at distance 0 have no normal and are skipped.
/// @param edt Distance transform over all sites.
/// @param x_grid Point (grid frame).
/// @param count Sites wanted, in [1, kCbfMaxSites] (clamped).
/// @param out Receives up to `count` sites (grid frame): the query's first, then nearest first.
/// @return Sites found (0 off the grid or without sites).
int nearestCbfSites(
  const EuclideanDistanceTransform & edt, const Vec2 & x_grid, int count, CbfSite * out) noexcept;

/// @brief One constraint \f$n^\top F' \ge -\gamma_b h\f$.
struct CbfConstraint
{
  /// Unit normal (away from the obstacle).
  Vec2 n{Vec2::Zero()};
  /// Barrier \f$h = D - \delta_{\mathrm{safe}}\f$ (m).
  double h{kInf};
};

/// @brief The filtered field.
struct CbfSolution
{
  /// \f$F^\star\f$.
  Vec2 F{Vec2::Zero()};
  /// \f$F^\star \ne F\f$: at least one constraint binds.
  bool active{false};
  /// The push-outs (\f$-\gamma_b h_j > 0\f$) conflicted: solved again with
  /// \f$\min(-\gamma_b h_j, 0)\f$.
  bool relaxed{false};
  /// Bit j: constraint j binds at \f$F^\star\f$.
  std::uint8_t binds{0};
};

/// @brief The CBF filter (math_spec §6.7): \f$F^\star = \arg\min\lVert F' - F\rVert^2\f$ s.t.
/// \f$n_j^\top F' \ge -\gamma_b h_j\f$, by enumeration of the active sets (at most two bind in
/// 2D): F, each projection, each pair intersection; the feasible candidate of least cost is the
/// optimum. Infeasible requests (conflicting push-outs) are solved again without the pushes, which
/// is always feasible (\f$F' = 0\f$). Finally \f$\lVert F^\star\rVert \le \lVert F\rVert\f$
/// (binds only while some \f$h_j < 0\f$).
/// @param F Field before the filter.
/// @param constraints Constraints, at most kCbfMaxSites (more are ignored).
/// @param count Number of constraints.
/// @param gain \f$\gamma_b\f$ (> 0).
CbfSolution solveCbf(
  const Vec2 & F, const CbfConstraint * constraints, int count, double gain) noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__CBF_HPP_
