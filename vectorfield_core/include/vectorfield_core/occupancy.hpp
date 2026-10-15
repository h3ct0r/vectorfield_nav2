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
/// @brief Occupancy grid snapshot, Felzenszwalb–Huttenlocher distance transform with feature
/// transform, the bilinear obstacle query and the K flood fill (plan §3.2; math_spec §6.1, §6.4).
///
/// Implemented in src/occupancy.cpp (M1 wave 2, occupancy). The public API is binding; private
/// members are the implementer's choice.
///
/// **Grid conventions.** Cell (ix, iy) has index `iy * width + ix` and covers
/// \f$[o_x + ix\cdot\mathrm{res}, o_x + (ix+1)\mathrm{res}) \times [o_y + iy\cdot\mathrm{res},
/// o_y + (iy+1)\mathrm{res})\f$ in the grid frame, where \f$o\f$ = origin() is the corner of cell
/// (0, 0) (Nav2 Costmap2D convention, no rotation). Its centre is \f$o + (ix + \tfrac12,
/// iy + \tfrac12)\mathrm{res}\f$. Distances are between cell **centres** (math_spec §6.1).

#ifndef VECTORFIELD_CORE__OCCUPANCY_HPP_
#define VECTORFIELD_CORE__OCCUPANCY_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief A snapshot of the obstacle sites of a costmap: a uint8 mask, 1 = site.
///
/// Sites are lethal cells, plus unknown cells when `unknown_is_obstacle` (the ROS layer applies
/// that rule when it builds the mask; math_spec §6.1). The frame name is the caller's business.
class OccupancyGrid
{
public:
  /// @brief An empty grid (0 × 0).
  OccupancyGrid() noexcept;

  /// @brief Reserves storage for `width * height` cells, so setFromMask() of up to that size
  /// never allocates.
  void reserve(int width, int height);

  /// @brief Copies a mask (nonzero = site), reusing capacity.
  ///
  /// The version() changes iff the geometry (size, resolution, origin) or any cell's site flag
  /// changed, so the distance transform is rebuilt only when needed.
  /// @param mask Row-major cells, `mask.size() == width * height`.
  /// @param width Cells along x (> 0).
  /// @param height Cells along y (> 0).
  /// @param resolution Cell size (m, > 0).
  /// @param origin Grid-frame position of the corner of cell (0, 0).
  /// @return False (grid unchanged) on inconsistent or non-finite arguments.
  bool setFromMask(
    Span<const std::uint8_t> mask, int width, int height, double resolution,
    const Vec2 & origin) noexcept;

  /// @brief Cells along x.
  int width() const noexcept;
  /// @brief Cells along y.
  int height() const noexcept;
  /// @brief Cell size (m).
  double resolution() const noexcept;
  /// @brief Grid-frame position of the corner of cell (0, 0).
  const Vec2 & origin() const noexcept;
  /// @brief The mask, row-major, 1 = site.
  Span<const std::uint8_t> mask() const noexcept;
  /// @brief True iff the cell is inside the grid.
  bool inBounds(int ix, int iy) const noexcept;
  /// @brief True iff the cell is inside the grid and a site.
  bool isSite(int ix, int iy) const noexcept;
  /// @brief Centre of a cell (grid frame); the cell need not be in bounds.
  Vec2 cellCenter(int ix, int iy) const noexcept;
  /// @brief The cell containing a grid-frame point (floor); false if outside the grid or
  /// non-finite.
  bool cellAt(const Vec2 & x_grid, int * ix, int * iy) const noexcept;

  /// @brief A process-wide unique stamp of the content: it changes whenever the content changes,
  /// and two grids with equal stamps have equal content (a copy keeps its source's stamp until
  /// either changes). 0 for an empty grid.
  std::uint64_t version() const noexcept;

private:
  std::vector<std::uint8_t> mask_;
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  Vec2 origin_{Vec2::Zero()};
  std::uint64_t version_{0};
};

/// @brief Result of a continuous obstacle query (math_spec §6.1 [Design]).
struct ObstacleQuery
{
  /// \f$D_o = \lVert v\rVert\f$ (m); \f$+\infty\f$ when invalid.
  double D_o{kInf};
  /// \f$n_o = v/D_o\f$; zero when invalid or \f$D_o = 0\f$.
  Vec2 n_o{Vec2::Zero()};
  /// \f$x_o^\star = x - v\f$ (grid frame).
  Vec2 x_o{Vec2::Zero()};
  /// False when the grid has no site or the point is outside the grid.
  bool valid{false};
};

/// @brief Squared Euclidean distance transform with feature transform (Felzenszwalb–Huttenlocher),
/// over all sites of a grid or over a filtered subset (math_spec §6.1, sec-edt).
///
/// Exact in integer cell units (oracle O29: squared distances equal brute force; each feature is
/// a site at that distance). Owns its result and its FH work arrays; they grow on demand and
/// never shrink, so rebuilding a same-size grid never allocates. Const methods are thread-safe.
/// Two instances are used per cycle: \f$\mathrm{EDT}_{\mathrm{all}}\f$ (caller-owned, `update()`)
/// and \f$\mathrm{EDT}_K\f$ (in StepScratch, `build()` with K's mask).
class EuclideanDistanceTransform
{
public:
  /// @brief An empty transform (no grid).
  EuclideanDistanceTransform() noexcept;

  /// @brief Reserves storage for `width * height` cells.
  void reserve(int width, int height);

  /// @brief Rebuilds over all sites iff `grid.version()` differs from the last build (or the
  /// last build was filtered).
  /// @return True iff it rebuilt.
  bool update(const OccupancyGrid & grid) noexcept;

  /// @brief Rebuilds unconditionally over the sites with a nonzero `site_filter` entry.
  /// @param grid The grid.
  /// @param site_filter Row-major cell filter of the grid's size (e.g. K's mask); an empty span
  /// selects all sites.
  void build(const OccupancyGrid & grid, Span<const std::uint8_t> site_filter) noexcept;

  /// @brief True iff at least one site was transformed.
  bool hasSites() const noexcept;
  /// @brief Grid width of the last build.
  int width() const noexcept;
  /// @brief Grid height of the last build.
  int height() const noexcept;
  /// @brief Resolution of the last build (m).
  double resolution() const noexcept;
  /// @brief Origin of the last build (grid frame).
  const Vec2 & origin() const noexcept;
  /// @brief `grid.version()` of the last build (0 if filtered or never built).
  std::uint64_t builtVersion() const noexcept;

  /// @brief Squared distance from the centre of cell (ix, iy) to the nearest site centre, in
  /// cell units² (an exact integer); +inf without sites. Precondition: in bounds.
  double squaredDistanceCells(int ix, int iy) const noexcept;
  /// @brief Distance from the centre of cell (ix, iy) to the nearest site centre (m); +inf
  /// without sites. Precondition: in bounds.
  double distance(int ix, int iy) const noexcept;
  /// @brief Feature \f$f(c)\f$: the nearest site cell of (ix, iy). Precondition: in bounds and
  /// hasSites().
  void feature(int ix, int iy, int * fx, int * fy) const noexcept;

  /// @brief Continuous query at a grid-frame point (math_spec §6.1 [Design]).
  ///
  /// Takes the four cell centres around x (indices clamped to the grid) with bilinear weights
  /// \f$w_{ij}\f$ and \f$v_{ij} = x - f(c_{ij})\f$; if every pair satisfies \f$v_{ij}^\top v_{kl}
  /// \ge 0\f$ and none is zero, \f$v = \sum w_{ij}v_{ij}\f$, else \f$v = x - f(c_{\mathrm{nearest}})\f$.
  /// Outside the grid or without sites: invalid, \f$D_o = +\infty\f$. Allocation-free.
  ObstacleQuery query(const Vec2 & x_grid) const noexcept;

private:
  std::vector<double> squared_;
  std::vector<int> feature_;
  std::vector<double> work_f_;
  std::vector<double> work_z_;
  std::vector<int> work_v_;
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  Vec2 origin_{Vec2::Zero()};
  std::uint64_t built_version_{0};
  bool has_sites_{false};
};

/// @brief K's flood fill (math_spec §6.4 "Identity K"): labels the 8-connected component,
/// containing the seed cell, of the dilated mask \f$\{c : \mathrm{EDT}_{\mathrm{all}}(c) \le
/// r\}\f$.
/// @param edt_all \f$\mathrm{EDT}_{\mathrm{all}}\f$ of the current grid.
/// @param dilation_radius \f$r = r_{\mathrm{insc}}\f$ (m).
/// @param seed_x Seed cell x.
/// @param seed_y Seed cell y.
/// @param component Output mask, resized to width·height: 1 = cell in K.
/// @param queue BFS work buffer (cell indices).
/// @return Number of cells in K; 0 when the seed is out of bounds or not dilated.
std::size_t floodFillDilatedComponent(
  const EuclideanDistanceTransform & edt_all, double dilation_radius, int seed_x, int seed_y,
  std::vector<std::uint8_t> * component, std::vector<int> * queue) noexcept;

/// @brief Nearest dilated cell (\f$\mathrm{EDT}_{\mathrm{all}} \le r\f$) to a grid-frame point
/// within `search_radius` of it, for re-seeding K during an episode (math_spec §6.4: radius
/// \f$r_{\mathrm{insc}} + 2\,\mathrm{res}\f$).
/// @return False if there is none (K is lost).
bool nearestDilatedCell(
  const EuclideanDistanceTransform & edt_all, double dilation_radius, const Vec2 & x_grid,
  double search_radius, int * cell_x, int * cell_y) noexcept;

/// @brief The obstacle inputs of one cycle (plan §3.1): non-owning pointers valid for the call.
///
/// `edt` is \f$\mathrm{EDT}_{\mathrm{all}}\f$ of `grid`, already updated by the caller (rebuilt
/// only when the grid changed). `T_grid_eval` maps evaluation-frame points into the grid frame
/// (plugin: identity, the costmap global frame; follower: costmap ← map).
struct ObstacleMap
{
  /// Occupancy grid.
  const OccupancyGrid * grid{nullptr};
  /// Distance transform over all of `grid`'s sites.
  const EuclideanDistanceTransform * edt{nullptr};
  /// Evaluation frame → grid frame.
  Isometry2 T_grid_eval;
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__OCCUPANCY_HPP_
