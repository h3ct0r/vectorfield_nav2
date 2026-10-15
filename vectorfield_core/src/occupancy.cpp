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

// Occupancy grid snapshot, Felzenszwalb–Huttenlocher distance and feature transform, the bilinear
// obstacle query and K's flood fill (plan §3.2; math_spec §6.1, §6.4).

#include "vectorfield_core/occupancy.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace vectorfield_core
{

namespace
{

/// Largest accepted side length (cells). The lower envelope compares parabola intersections by
/// int64 cross-multiplication of operands below 2^41 and 2^20, which cannot overflow up to here;
/// squared distances (< 2^41) are exact in double. 2^20 cells is 52 km at 5 cm.
constexpr int kMaxSide = 1 << 20;

/// Feature entry of a cell when the transform has no site.
constexpr int kNoFeature = -1;

/// Source of the process-wide content stamps of OccupancyGrid; 0 is reserved for the empty grid.
std::atomic<std::uint64_t> g_next_version{1};

std::uint64_t newVersion() noexcept
{
  return g_next_version.fetch_add(1, std::memory_order_relaxed);
}

/// Centre of cell (ix, iy) for a grid with corner `origin`. Every centre in this file and in
/// OccupancyGrid::cellCenter() comes from this one expression, so they agree bit for bit.
inline Vec2 cellCentre(const Vec2 & origin, double resolution, int ix, int iy) noexcept
{
  return Vec2(origin.x() + (ix + 0.5) * resolution, origin.y() + (iy + 0.5) * resolution);
}

/// The cell containing a grid-frame point: floor((x − o)/res) per axis (half-open cells, as
/// OccupancyGrid::cellAt()). Also returns the continuous cell coordinates (x − o)/res.
/// @return False when the point is non-finite or outside the width × height cells.
bool locateCell(
  const Vec2 & x, const Vec2 & origin, double resolution, int width, int height, double * sx,
  double * sy, int * ix, int * iy) noexcept
{
  if (width <= 0 || height <= 0 || !isFinite(x)) {
    return false;
  }
  *sx = (x.x() - origin.x()) / resolution;
  *sy = (x.y() - origin.y()) / resolution;
  const double fx = std::floor(*sx);
  const double fy = std::floor(*sy);
  if (!(fx >= 0.0 && fx < width && fy >= 0.0 && fy < height)) {
    return false;
  }
  *ix = static_cast<int>(fx);
  *iy = static_cast<int>(fy);
  return true;
}

inline int clampIndex(int i, int n) noexcept { return i < 0 ? 0 : (i >= n ? n - 1 : i); }

/// Pass 1 of the transform: the nearest site of every cell within its own column.
///
/// Writes the squared row offset to that site (cell units², +inf when the column has no site)
/// to `squared` and the site's row (or kNoFeature) to `rows`. Ties go to the lower row. Sweeps
/// whole rows, down and then up, for cache locality; `column_state` (width ints) holds the row
/// of the last site seen in each column.
void nearestSiteInColumns(
  const std::uint8_t * mask, const std::uint8_t * filter, int width, int height, double * squared,
  int * rows, int * column_state) noexcept
{
  auto is_site = [mask, filter](std::size_t i) {
    return mask[i] != 0 && (filter == nullptr || filter[i] != 0);
  };
  std::fill(column_state, column_state + width, kNoFeature);
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    for (int x = 0; x < width; ++x) {
      if (is_site(row + x)) {
        column_state[x] = y;
      }
      rows[row + x] = column_state[x];  // nearest site at or below y, for the upward sweep
    }
  }
  std::fill(column_state, column_state + width, kNoFeature);
  for (int y = height - 1; y >= 0; --y) {
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    for (int x = 0; x < width; ++x) {
      const std::size_t i = row + x;
      if (is_site(i)) {
        column_state[x] = y;
      }
      const int below = rows[i];
      const int above = column_state[x];
      const int d_below = below == kNoFeature ? std::numeric_limits<int>::max() : y - below;
      const int d_above = above == kNoFeature ? std::numeric_limits<int>::max() : above - y;
      if (below == kNoFeature && above == kNoFeature) {
        squared[i] = kInf;
        rows[i] = kNoFeature;
      } else if (d_below <= d_above) {
        squared[i] = static_cast<double>(d_below) * static_cast<double>(d_below);
        rows[i] = below;
      } else {
        squared[i] = static_cast<double>(d_above) * static_cast<double>(d_above);
        rows[i] = above;
      }
    }
  }
}

/// Pass 2 on one row: the lower envelope of the parabolas \f$(q - p)^2 + g(p)\f$ over the finite
/// column distances g of the row (Felzenszwalb and Huttenlocher, "Distance Transforms of Sampled
/// Functions", Theory of Computing 8, 2012, Algorithm 1), tracking the argmin p.
///
/// The envelope is built without its boundary array: with \f$F(p) = g(p) + p^2\f$, the
/// intersection of the parabolas of a < b lies at \f$(F(b) - F(a))/(2(b - a))\f$, and both
/// comparisons of the algorithm are done by exact int64 cross-multiplication. The squared
/// distances are therefore exact integers (oracle O29), and ties go to the leftmost parabola.
///
/// @param squared_row In: g for the row (+inf without a site in that column). Out: squared
/// distance of each cell to its nearest site.
/// @param feature_row In: pass-1 site row of each column. Out: flat index of the nearest site.
/// @param width Row length.
/// @param g Work: copy of the input g (width doubles).
/// @param apex Work: F per column (width doubles).
/// @param vertices Work: columns of the envelope's parabolas (width ints).
/// @param site_rows Work: copy of the input site rows (width ints).
/// @return False iff the row has no finite g, i.e. the transform has no site at all.
bool lowerEnvelopeRow(
  double * squared_row, int * feature_row, int width, double * g, double * apex, int * vertices,
  int * site_rows) noexcept
{
  std::copy(squared_row, squared_row + width, g);
  std::copy(feature_row, feature_row + width, site_rows);
  int k = -1;  // index of the last parabola of the envelope
  for (int q = 0; q < width; ++q) {
    if (!(g[q] < kInf)) {
      continue;
    }
    apex[q] = g[q] + static_cast<double>(q) * static_cast<double>(q);
    const std::int64_t apex_q = static_cast<std::int64_t>(apex[q]);
    // Drop parabola b = vertices[k] while it is nowhere strictly lowest: its intersection with q,
    // s(b, q), lies at or left of its left boundary s(a, b), a = vertices[k − 1].
    while (k >= 1) {
      const int a = vertices[k - 1];
      const int b = vertices[k];
      const std::int64_t apex_a = static_cast<std::int64_t>(apex[a]);
      const std::int64_t apex_b = static_cast<std::int64_t>(apex[b]);
      if ((apex_q - apex_b) * (b - a) <= (apex_b - apex_a) * (q - b)) {
        --k;
      } else {
        break;
      }
    }
    vertices[++k] = q;
  }
  if (k < 0) {
    std::fill(squared_row, squared_row + width, kInf);
    std::fill(feature_row, feature_row + width, kNoFeature);
    return false;
  }
  int j = 0;
  for (int q = 0; q < width; ++q) {
    // Advance while the boundary s(vertices[j], vertices[j + 1]) lies strictly left of q.
    while (j < k) {
      const int a = vertices[j];
      const int b = vertices[j + 1];
      const std::int64_t rise =
        static_cast<std::int64_t>(apex[b]) - static_cast<std::int64_t>(apex[a]);
      if (rise < 2 * static_cast<std::int64_t>(q) * (b - a)) {
        ++j;
      } else {
        break;
      }
    }
    const int p = vertices[j];
    const double dx = static_cast<double>(q - p);
    squared_row[q] = dx * dx + g[p];
    feature_row[q] = site_rows[p] * width + p;
  }
  return true;
}

/// Largest integer squared cell distance s with \f$\sqrt{s}\cdot\mathrm{res} \le r\f$, so that
/// `squaredDistanceCells(c) <= limit` holds exactly when `distance(c) <= r` (sqrt and the product
/// are monotone in s). −1 when no distance qualifies (r negative or NaN); +inf when every finite
/// distance does.
double dilationLimit(double r, double resolution) noexcept
{
  if (!(r >= 0.0) || !(resolution > 0.0)) {
    return -1.0;
  }
  const double ratio = r / resolution;
  const double approx = ratio * ratio;
  if (!(approx < 0x1p52)) {
    return kInf;  // above every exact squared distance of an accepted grid (< 2^41)
  }
  double s = std::floor(approx);
  while (s > 0.0 && std::sqrt(s) * resolution > r) {
    s -= 1.0;
  }
  while (std::sqrt(s + 1.0) * resolution <= r) {
    s += 1.0;
  }
  return s;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// OccupancyGrid

OccupancyGrid::OccupancyGrid() noexcept {}

void OccupancyGrid::reserve(int width, int height)
{
  if (width <= 0 || height <= 0) {
    return;
  }
  mask_.reserve(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
}

bool OccupancyGrid::setFromMask(
  Span<const std::uint8_t> mask, int width, int height, double resolution,
  const Vec2 & origin) noexcept
{
  if (width <= 0 || height <= 0 || width > kMaxSide || height > kMaxSide) {
    return false;
  }
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  // Flat cell indices are int (feature transform, flood-fill queue).
  if (cells > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return false;
  }
  if (mask.size() != cells || mask.data() == nullptr) {
    return false;
  }
  if (!(resolution > 0.0) || !std::isfinite(resolution) || !isFinite(origin)) {
    return false;
  }
  const bool same_geometry = width == width_ && height == height_ && resolution == resolution_ &&
                             origin.x() == origin_.x() && origin.y() == origin_.y() &&
                             mask_.size() == cells;
  bool changed = !same_geometry;
  if (same_geometry) {
    for (std::size_t i = 0; i < cells; ++i) {
      const std::uint8_t site = mask[i] != 0 ? 1 : 0;
      if (site != mask_[i]) {
        mask_[i] = site;
        changed = true;
      }
    }
  } else {
    mask_.resize(cells);
    for (std::size_t i = 0; i < cells; ++i) {
      mask_[i] = mask[i] != 0 ? 1 : 0;
    }
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;
  }
  if (changed) {
    version_ = newVersion();
  }
  return true;
}

int OccupancyGrid::width() const noexcept { return width_; }

int OccupancyGrid::height() const noexcept { return height_; }

double OccupancyGrid::resolution() const noexcept { return resolution_; }

const Vec2 & OccupancyGrid::origin() const noexcept { return origin_; }

Span<const std::uint8_t> OccupancyGrid::mask() const noexcept
{
  return Span<const std::uint8_t>(mask_.data(), mask_.size());
}

bool OccupancyGrid::inBounds(int ix, int iy) const noexcept
{
  return ix >= 0 && ix < width_ && iy >= 0 && iy < height_;
}

bool OccupancyGrid::isSite(int ix, int iy) const noexcept
{
  if (!inBounds(ix, iy)) {
    return false;
  }
  const std::size_t i = static_cast<std::size_t>(iy) * static_cast<std::size_t>(width_) + ix;
  return i < mask_.size() && mask_[i] != 0;
}

Vec2 OccupancyGrid::cellCenter(int ix, int iy) const noexcept
{
  return cellCentre(origin_, resolution_, ix, iy);
}

bool OccupancyGrid::cellAt(const Vec2 & x_grid, int * ix, int * iy) const noexcept
{
  double sx = 0.0;
  double sy = 0.0;
  int cx = 0;
  int cy = 0;
  if (!locateCell(x_grid, origin_, resolution_, width_, height_, &sx, &sy, &cx, &cy)) {
    return false;
  }
  if (ix != nullptr) {
    *ix = cx;
  }
  if (iy != nullptr) {
    *iy = cy;
  }
  return true;
}

std::uint64_t OccupancyGrid::version() const noexcept { return version_; }

// -----------------------------------------------------------------------------------------------
// EuclideanDistanceTransform
//
// Buffers: squared_ and feature_ hold the result (feature_ = flat index of the nearest site, or
// kNoFeature); work_f_ holds a row's column distances, work_z_ the parabola apex constants
// F(p) = g(p) + p² (the envelope needs no boundary array, see lowerEnvelopeRow), and work_v_ the
// envelope's columns followed by a copy of the row's pass-1 site rows (2 × max(width, height)).

EuclideanDistanceTransform::EuclideanDistanceTransform() noexcept {}

void EuclideanDistanceTransform::reserve(int width, int height)
{
  if (width <= 0 || height <= 0) {
    return;
  }
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const std::size_t side = static_cast<std::size_t>(std::max(width, height));
  squared_.reserve(cells);
  feature_.reserve(cells);
  work_f_.reserve(side);
  work_z_.reserve(side);
  work_v_.reserve(2 * side);
}

bool EuclideanDistanceTransform::update(const OccupancyGrid & grid) noexcept
{
  const std::uint64_t version = grid.version();
  if (version == 0) {
    // The empty grid: nothing to do if this transform is already empty.
    if (width_ == 0 && height_ == 0) {
      return false;
    }
  } else if (version == built_version_) {
    const std::size_t cells =
      static_cast<std::size_t>(grid.width()) * static_cast<std::size_t>(grid.height());
    if (squared_.size() == cells && feature_.size() == cells) {
      return false;
    }
  }
  build(grid, Span<const std::uint8_t>());
  return true;
}

void EuclideanDistanceTransform::build(
  const OccupancyGrid & grid, Span<const std::uint8_t> site_filter) noexcept
{
  const int width = grid.width();
  const int height = grid.height();
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  width_ = width;
  height_ = height;
  resolution_ = grid.resolution();
  origin_ = grid.origin();
  built_version_ = site_filter.empty() ? grid.version() : 0;
  has_sites_ = false;
  squared_.resize(cells);
  feature_.resize(cells);
  if (cells == 0) {
    return;
  }
  const Span<const std::uint8_t> mask = grid.mask();
  if (mask.size() != cells || (!site_filter.empty() && site_filter.size() != cells)) {
    // A filter of the wrong size selects nothing (and never passes for an unfiltered build).
    std::fill(squared_.begin(), squared_.end(), kInf);
    std::fill(feature_.begin(), feature_.end(), kNoFeature);
    built_version_ = 0;
    return;
  }
  const std::size_t side = static_cast<std::size_t>(std::max(width, height));
  work_f_.resize(side);
  work_z_.resize(side);
  work_v_.resize(2 * side);

  nearestSiteInColumns(
    mask.data(), site_filter.empty() ? nullptr : site_filter.data(), width, height, squared_.data(),
    feature_.data(), work_v_.data());
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    if (!lowerEnvelopeRow(
          squared_.data() + row, feature_.data() + row, width, work_f_.data(), work_z_.data(),
          work_v_.data(), work_v_.data() + side)) {
      // No column has a site, so neither has any other row: the transform has no site.
      std::fill(squared_.begin(), squared_.end(), kInf);
      std::fill(feature_.begin(), feature_.end(), kNoFeature);
      return;
    }
  }
  has_sites_ = true;
}

bool EuclideanDistanceTransform::hasSites() const noexcept { return has_sites_; }

int EuclideanDistanceTransform::width() const noexcept { return width_; }

int EuclideanDistanceTransform::height() const noexcept { return height_; }

double EuclideanDistanceTransform::resolution() const noexcept { return resolution_; }

const Vec2 & EuclideanDistanceTransform::origin() const noexcept { return origin_; }

std::uint64_t EuclideanDistanceTransform::builtVersion() const noexcept { return built_version_; }

double EuclideanDistanceTransform::squaredDistanceCells(int ix, int iy) const noexcept
{
  return squared_[static_cast<std::size_t>(iy) * static_cast<std::size_t>(width_) + ix];
}

double EuclideanDistanceTransform::distance(int ix, int iy) const noexcept
{
  return std::sqrt(squaredDistanceCells(ix, iy)) * resolution_;
}

void EuclideanDistanceTransform::feature(int ix, int iy, int * fx, int * fy) const noexcept
{
  const int index = feature_[static_cast<std::size_t>(iy) * static_cast<std::size_t>(width_) + ix];
  if (index < 0) {
    *fx = -1;
    *fy = -1;
    return;
  }
  *fx = index % width_;
  *fy = index / width_;
}

ObstacleQuery EuclideanDistanceTransform::query(const Vec2 & x_grid) const noexcept
{
  ObstacleQuery result;
  const std::size_t cells = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  if (!has_sites_ || cells == 0 || feature_.size() < cells) {
    return result;
  }
  double sx = 0.0;
  double sy = 0.0;
  int near_x = 0;
  int near_y = 0;
  if (!locateCell(x_grid, origin_, resolution_, width_, height_, &sx, &sy, &near_x, &near_y)) {
    return result;  // outside the grid (or non-finite): D_o = +inf
  }
  auto site_vector = [this, &x_grid](int ix, int iy) -> Vec2 {
    const int index =
      feature_[static_cast<std::size_t>(iy) * static_cast<std::size_t>(width_) + ix];
    return x_grid - cellCentre(origin_, resolution_, index % width_, index / width_);
  };

  // The four cell centres around x: continuous centre coordinates u = s − 1/2, lower-left cell
  // floor(u), indices clamped to the grid, bilinear weights from the unclamped fractions.
  const double ux = sx - 0.5;
  const double uy = sy - 0.5;
  const double i0 = std::floor(ux);
  const double j0 = std::floor(uy);
  const double tx = ux - i0;
  const double ty = uy - j0;
  const int ia = clampIndex(static_cast<int>(i0), width_);
  const int ib = clampIndex(static_cast<int>(i0) + 1, width_);
  const int ja = clampIndex(static_cast<int>(j0), height_);
  const int jb = clampIndex(static_cast<int>(j0) + 1, height_);
  const Vec2 v[4] = {
    site_vector(ia, ja), site_vector(ib, ja), site_vector(ia, jb), site_vector(ib, jb)};
  const double w[4] = {(1.0 - tx) * (1.0 - ty), tx * (1.0 - ty), (1.0 - tx) * ty, tx * ty};

  bool blend = true;
  for (int a = 0; a < 4 && blend; ++a) {
    if (v[a].x() == 0.0 && v[a].y() == 0.0) {
      blend = false;
    }
    for (int b = a + 1; b < 4 && blend; ++b) {
      if (v[a].dot(v[b]) < 0.0) {
        blend = false;
      }
    }
  }
  Vec2 vec;
  if (blend) {
    vec = w[0] * v[0] + w[1] * v[1] + w[2] * v[2] + w[3] * v[3];
  } else {
    vec = site_vector(near_x, near_y);  // the feature of the cell containing x
  }
  result.D_o = vec.norm();
  result.n_o = result.D_o > 0.0 ? Vec2(vec / result.D_o) : Vec2(Vec2::Zero());
  result.x_o = x_grid - vec;
  result.valid = true;
  return result;
}

// -----------------------------------------------------------------------------------------------
// K identity (math_spec §6.4)

std::size_t floodFillDilatedComponent(
  const EuclideanDistanceTransform & edt_all, double dilation_radius, int seed_x, int seed_y,
  std::vector<std::uint8_t> * component, std::vector<int> * queue) noexcept
{
  if (component == nullptr) {
    return 0;
  }
  const int width = edt_all.width();
  const int height = edt_all.height();
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  component->assign(cells, 0);
  if (
    queue == nullptr || cells == 0 || !edt_all.hasSites() || seed_x < 0 || seed_x >= width ||
    seed_y < 0 || seed_y >= height) {
    return 0;
  }
  const double limit = dilationLimit(dilation_radius, edt_all.resolution());
  if (!(edt_all.squaredDistanceCells(seed_x, seed_y) <= limit)) {
    return 0;
  }
  if (queue->size() < cells) {
    queue->resize(cells);
  }
  std::uint8_t * in_k = component->data();
  int * fifo = queue->data();
  std::size_t head = 0;
  std::size_t tail = 0;
  const int seed = seed_y * width + seed_x;
  in_k[seed] = 1;
  fifo[tail++] = seed;
  // Breadth-first, 8-connected. A cell is marked when queued, so each is queued at most once and
  // the queue never needs more than `cells` entries.
  while (head < tail) {
    const int index = fifo[head++];
    const int x = index % width;
    const int y = index / width;
    const int x_lo = std::max(x - 1, 0);
    const int x_hi = std::min(x + 1, width - 1);
    const int y_lo = std::max(y - 1, 0);
    const int y_hi = std::min(y + 1, height - 1);
    for (int ny = y_lo; ny <= y_hi; ++ny) {
      for (int nx = x_lo; nx <= x_hi; ++nx) {
        const int n = ny * width + nx;
        if (in_k[n] != 0 || !(edt_all.squaredDistanceCells(nx, ny) <= limit)) {
          continue;
        }
        in_k[n] = 1;
        fifo[tail++] = n;
      }
    }
  }
  return tail;
}

bool nearestDilatedCell(
  const EuclideanDistanceTransform & edt_all, double dilation_radius, const Vec2 & x_grid,
  double search_radius, int * cell_x, int * cell_y) noexcept
{
  const int width = edt_all.width();
  const int height = edt_all.height();
  if (
    width <= 0 || height <= 0 || !edt_all.hasSites() || !isFinite(x_grid) ||
    !(search_radius >= 0.0) || !std::isfinite(search_radius)) {
    return false;
  }
  const double resolution = edt_all.resolution();
  const Vec2 & origin = edt_all.origin();
  const double limit = dilationLimit(dilation_radius, resolution);
  if (limit < 0.0) {
    return false;
  }
  // Cells whose centres may lie within the radius, with one cell of slack per side; the exact
  // distance test below decides.
  const double gx = (x_grid.x() - origin.x()) / resolution - 0.5;
  const double gy = (x_grid.y() - origin.y()) / resolution - 0.5;
  const double reach = search_radius / resolution;
  const double x_lo = std::floor(gx - reach) - 1.0;
  const double x_hi = std::ceil(gx + reach) + 1.0;
  const double y_lo = std::floor(gy - reach) - 1.0;
  const double y_hi = std::ceil(gy + reach) + 1.0;
  if (x_hi < 0.0 || y_hi < 0.0 || x_lo > width - 1.0 || y_lo > height - 1.0) {
    return false;
  }
  const int ix_lo = static_cast<int>(std::max(x_lo, 0.0));
  const int ix_hi = static_cast<int>(std::min(x_hi, width - 1.0));
  const int iy_lo = static_cast<int>(std::max(y_lo, 0.0));
  const int iy_hi = static_cast<int>(std::min(y_hi, height - 1.0));
  double best = kInf;
  int best_x = -1;
  int best_y = -1;
  for (int iy = iy_lo; iy <= iy_hi; ++iy) {
    for (int ix = ix_lo; ix <= ix_hi; ++ix) {
      if (!(edt_all.squaredDistanceCells(ix, iy) <= limit)) {
        continue;
      }
      const double d = (cellCentre(origin, resolution, ix, iy) - x_grid).norm();
      if (d <= search_radius && d < best) {  // ties: the first in row-major order
        best = d;
        best_x = ix;
        best_y = iy;
      }
    }
  }
  if (best_x < 0) {
    return false;
  }
  if (cell_x != nullptr) {
    *cell_x = best_x;
  }
  if (cell_y != nullptr) {
    *cell_y = best_y;
  }
  return true;
}

}  // namespace vectorfield_core
