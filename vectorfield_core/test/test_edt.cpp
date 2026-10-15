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

// Felzenszwalb–Huttenlocher distance and feature transform, the bilinear obstacle query and
// change detection (math_spec §6.1; oracles O29, O30).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <thread>
#include <vector>

#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::EuclideanDistanceTransform;
using vf::ObstacleQuery;
using vf::OccupancyGrid;
using vf::Vec2;

namespace
{

constexpr double kDeg = 180.0 / vf::kPi;

// A random mask of one of several kinds: empty, full, a single site, uniform density, sparse,
// dense, or a few random rectangles over a sparse background.
std::vector<std::uint8_t> randomMask(std::mt19937_64 & rng, int width, int height)
{
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(width) * height, 0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  auto fill = [&](double density) {
    for (auto & cell : mask) {
      // Any nonzero value is a site.
      cell = static_cast<std::uint8_t>(unit(rng) < density ? 1 + rng() % 254 : 0);
    }
  };
  switch (rng() % 8) {
    case 0:
      break;  // empty
    case 1:
      fill(1.0);
      break;
    case 2:
      mask[rng() % mask.size()] = 1;
      break;
    case 3:
      fill(unit(rng));
      break;
    case 4:
      fill(0.05 * unit(rng));
      break;
    case 5:
      fill(0.9 + 0.1 * unit(rng));
      break;
    default: {
      fill(0.01 * unit(rng));
      const int rectangles = 1 + static_cast<int>(rng() % 5);
      for (int r = 0; r < rectangles; ++r) {
        const int x0 = static_cast<int>(rng() % width);
        const int y0 = static_cast<int>(rng() % height);
        const int x1 = std::min(width - 1, x0 + static_cast<int>(rng() % 12));
        const int y1 = std::min(height - 1, y0 + static_cast<int>(rng() % 12));
        for (int y = y0; y <= y1; ++y) {
          for (int x = x0; x <= x1; ++x) {
            mask[static_cast<std::size_t>(y) * width + x] = 1;
          }
        }
      }
      break;
    }
  }
  return mask;
}

// Exact squared distance (cell units) from every cell centre to the nearest selected site centre,
// −1 for none: brute force over the site list when it is short, otherwise a ring search that stops
// once the Chebyshev ring is farther than the best distance found. Independent of FH.
std::vector<long long> referenceSquared(
  const std::vector<std::uint8_t> & sites, int width, int height)
{
  std::vector<std::pair<int, int>> list;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (sites[static_cast<std::size_t>(y) * width + x] != 0) {
        list.emplace_back(x, y);
      }
    }
  }
  std::vector<long long> out(static_cast<std::size_t>(width) * height, -1);
  if (list.empty()) {
    return out;
  }
  auto is_site = [&](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height &&
           sites[static_cast<std::size_t>(y) * width + x] != 0;
  };
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      long long best = -1;
      auto consider = [&](int sx, int sy) {
        const long long dx = sx - x;
        const long long dy = sy - y;
        const long long d2 = dx * dx + dy * dy;
        if (best < 0 || d2 < best) {
          best = d2;
        }
      };
      if (list.size() <= 64) {
        for (const auto & s : list) {
          consider(s.first, s.second);
        }
      } else {
        for (int r = 0; r <= std::max(width, height); ++r) {
          if (best >= 0 && best <= static_cast<long long>(r) * r) {
            break;
          }
          if (r == 0) {
            if (is_site(x, y)) {
              consider(x, y);
            }
            continue;
          }
          for (int sx = x - r; sx <= x + r; ++sx) {
            if (is_site(sx, y - r)) {
              consider(sx, y - r);
            }
            if (is_site(sx, y + r)) {
              consider(sx, y + r);
            }
          }
          for (int sy = y - r + 1; sy <= y + r - 1; ++sy) {
            if (is_site(x - r, sy)) {
              consider(x - r, sy);
            }
            if (is_site(x + r, sy)) {
              consider(x + r, sy);
            }
          }
        }
      }
      out[static_cast<std::size_t>(y) * width + x] = best;
    }
  }
  return out;
}

// Checks a transform against the reference: exact squared distances, and every feature a selected
// site at exactly that distance (oracle O29). Returns the number of failures (0 = pass).
int checkTransform(
  const EuclideanDistanceTransform & edt, const std::vector<std::uint8_t> & sites, int width,
  int height)
{
  const std::vector<long long> reference = referenceSquared(sites, width, height);
  const bool any =
    std::any_of(reference.begin(), reference.end(), [](long long d) { return d >= 0; });
  int failures = 0;
  if (edt.hasSites() != any || edt.width() != width || edt.height() != height) {
    ADD_FAILURE() << "hasSites/size mismatch: " << width << " x " << height;
    return 1;
  }
  for (int y = 0; y < height && failures < 5; ++y) {
    for (int x = 0; x < width && failures < 5; ++x) {
      const long long expected = reference[static_cast<std::size_t>(y) * width + x];
      const double got = edt.squaredDistanceCells(x, y);
      if (expected < 0) {
        if (!std::isinf(got)) {
          ADD_FAILURE() << "cell (" << x << ", " << y << "): expected +inf, got " << got;
          ++failures;
        }
        continue;
      }
      if (got != static_cast<double>(expected)) {
        ADD_FAILURE() << "cell (" << x << ", " << y << ") of " << width << " x " << height
                      << ": squared distance " << got << ", brute force " << expected;
        ++failures;
        continue;
      }
      int fx = -1;
      int fy = -1;
      edt.feature(x, y, &fx, &fy);
      const bool feature_is_site = fx >= 0 && fx < width && fy >= 0 && fy < height &&
                                   sites[static_cast<std::size_t>(fy) * width + fx] != 0;
      const long long dx = fx - x;
      const long long dy = fy - y;
      if (!feature_is_site || dx * dx + dy * dy != expected) {
        ADD_FAILURE() << "cell (" << x << ", " << y << "): feature (" << fx << ", " << fy
                      << ") is not a site at squared distance " << expected;
        ++failures;
      }
    }
  }
  return failures;
}

// math_spec §6.1 continuous query, re-derived from the public accessors.
ObstacleQuery referenceQuery(
  const OccupancyGrid & grid, const EuclideanDistanceTransform & edt, const Vec2 & x)
{
  ObstacleQuery out;
  int cx = 0;
  int cy = 0;
  if (!edt.hasSites() || !grid.cellAt(x, &cx, &cy)) {
    return out;
  }
  const double res = grid.resolution();
  const Vec2 & o = grid.origin();
  const double ux = (x.x() - o.x()) / res - 0.5;
  const double uy = (x.y() - o.y()) / res - 0.5;
  const int i0 = static_cast<int>(std::floor(ux));
  const int j0 = static_cast<int>(std::floor(uy));
  const double tx = ux - std::floor(ux);
  const double ty = uy - std::floor(uy);
  auto clamp_index = [](int i, int n) { return std::min(std::max(i, 0), n - 1); };
  const int cols[2] = {clamp_index(i0, grid.width()), clamp_index(i0 + 1, grid.width())};
  const int rows[2] = {clamp_index(j0, grid.height()), clamp_index(j0 + 1, grid.height())};
  auto vector_to_feature = [&](int ix, int iy) {
    int fx = 0;
    int fy = 0;
    edt.feature(ix, iy, &fx, &fy);
    return Vec2(x - grid.cellCenter(fx, fy));
  };
  // Order (i0, j0), (i0 + 1, j0), (i0, j0 + 1), (i0 + 1, j0 + 1).
  Vec2 v[4];
  double w[4];
  for (int b = 0; b < 2; ++b) {
    for (int a = 0; a < 2; ++a) {
      v[2 * b + a] = vector_to_feature(cols[a], rows[b]);
      w[2 * b + a] = (a == 1 ? tx : 1.0 - tx) * (b == 1 ? ty : 1.0 - ty);
    }
  }
  bool blend = true;
  for (int a = 0; a < 4; ++a) {
    blend = blend && !(v[a].x() == 0.0 && v[a].y() == 0.0);
    for (int b = a + 1; b < 4; ++b) {
      blend = blend && v[a].dot(v[b]) >= 0.0;
    }
  }
  const Vec2 vec =
    blend ? Vec2(w[0] * v[0] + w[1] * v[1] + w[2] * v[2] + w[3] * v[3]) : vector_to_feature(cx, cy);
  out.D_o = vec.norm();
  out.n_o = out.D_o > 0.0 ? Vec2(vec / out.D_o) : Vec2(Vec2::Zero());
  out.x_o = x - vec;
  out.valid = true;
  return out;
}

// |angle| (degrees) between two non-zero vectors.
double angleBetweenDeg(const Vec2 & a, const Vec2 & b)
{
  return std::abs(std::atan2(vf::cross2(a, b), a.dot(b))) * kDeg;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// O29: exact squared distances and features.

TEST(Edt, O29_RandomGridsMatchBruteForceExactly)
{
  std::mt19937_64 rng(29);
  int grids_with_sites = 0;
  int grids_empty = 0;
  int grids_full = 0;
  EuclideanDistanceTransform edt;  // reused across sizes: buffers grow and shrink logically
  for (int trial = 0; trial < 10000; ++trial) {
    const int width = 1 + static_cast<int>(rng() % 60);
    const int height = 1 + static_cast<int>(rng() % 60);
    const std::vector<std::uint8_t> mask = randomMask(rng, width, height);
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2(-1.0, 2.0)));
    ASSERT_TRUE(edt.update(grid));
    const int sites = static_cast<int>(
      std::count_if(mask.begin(), mask.end(), [](std::uint8_t c) { return c != 0; }));
    grids_with_sites += sites > 0 ? 1 : 0;
    grids_empty += sites == 0 ? 1 : 0;
    grids_full += sites == width * height ? 1 : 0;
    ASSERT_EQ(checkTransform(edt, mask, width, height), 0) << "trial " << trial;
  }
  // Every kind of grid was exercised.
  EXPECT_GT(grids_empty, 500);
  EXPECT_GT(grids_full, 500);
  EXPECT_GT(grids_with_sites, 8000);
}

TEST(Edt, O29_WorstCaseShapesMatchBruteForce)
{
  // Shapes that stress the lower envelope: one site per row in a diagonal, a single site in each
  // corner, alternating columns, a checkerboard and a 1 x 60 / 60 x 1 strip.
  const int n = 60;
  std::vector<std::vector<std::uint8_t>> masks;
  std::vector<std::uint8_t> diagonal(n * n, 0);
  std::vector<std::uint8_t> corners(n * n, 0);
  std::vector<std::uint8_t> columns(n * n, 0);
  std::vector<std::uint8_t> checker(n * n, 0);
  for (int y = 0; y < n; ++y) {
    diagonal[y * n + (n - 1 - y)] = 1;
    for (int x = 0; x < n; ++x) {
      columns[y * n + x] = x % 2 == 0 ? 1 : 0;
      checker[y * n + x] = (x + y) % 2 == 0 ? 1 : 0;
    }
  }
  corners[0] = corners[n - 1] = corners[(n - 1) * n] = corners[n * n - 1] = 1;
  for (const auto * mask : {&diagonal, &corners, &columns, &checker}) {
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(*mask, n, n, 0.05, Vec2::Zero()));
    EuclideanDistanceTransform edt;
    edt.update(grid);
    EXPECT_EQ(checkTransform(edt, *mask, n, n), 0);
  }
  std::vector<std::uint8_t> strip(n, 0);
  strip[17] = 1;
  strip[42] = 1;
  for (const auto & dims : {std::pair<int, int>{n, 1}, std::pair<int, int>{1, n}}) {
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(strip, dims.first, dims.second, 0.1, Vec2::Zero()));
    EuclideanDistanceTransform edt;
    edt.update(grid);
    EXPECT_EQ(checkTransform(edt, strip, dims.first, dims.second), 0);
  }
}

TEST(Edt, FilteredBuildUsesOnlyFilteredSites)
{
  std::mt19937_64 rng(31);
  EuclideanDistanceTransform edt;
  for (int trial = 0; trial < 2000; ++trial) {
    const int width = 1 + static_cast<int>(rng() % 60);
    const int height = 1 + static_cast<int>(rng() % 60);
    const std::vector<std::uint8_t> mask = randomMask(rng, width, height);
    const std::vector<std::uint8_t> filter = randomMask(rng, width, height);
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2::Zero()));
    edt.build(grid, filter);
    EXPECT_EQ(edt.builtVersion(), 0u);
    std::vector<std::uint8_t> selected(mask.size());
    for (std::size_t i = 0; i < mask.size(); ++i) {
      selected[i] = mask[i] != 0 && filter[i] != 0 ? 1 : 0;
    }
    ASSERT_EQ(checkTransform(edt, selected, width, height), 0) << "trial " << trial;
  }
}

TEST(Edt, AccessorsAndDegenerateInputs)
{
  EuclideanDistanceTransform fresh;
  EXPECT_FALSE(fresh.hasSites());
  EXPECT_EQ(fresh.width(), 0);
  EXPECT_EQ(fresh.height(), 0);
  EXPECT_EQ(fresh.builtVersion(), 0u);
  EXPECT_FALSE(fresh.query(Vec2(0.0, 0.0)).valid);

  const int width = 7;
  const int height = 5;
  std::vector<std::uint8_t> mask(width * height, 0);
  mask[2 * width + 3] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.1, Vec2(1.0, -2.0)));
  EuclideanDistanceTransform edt;
  EXPECT_TRUE(edt.update(grid));
  EXPECT_TRUE(edt.hasSites());
  EXPECT_EQ(edt.width(), width);
  EXPECT_EQ(edt.height(), height);
  EXPECT_DOUBLE_EQ(edt.resolution(), 0.1);
  EXPECT_EQ(edt.origin(), Vec2(1.0, -2.0));
  EXPECT_EQ(edt.builtVersion(), grid.version());
  EXPECT_EQ(edt.squaredDistanceCells(3, 2), 0.0);
  EXPECT_EQ(edt.squaredDistanceCells(0, 0), 13.0);  // (3, 2)
  EXPECT_DOUBLE_EQ(edt.distance(0, 0), std::sqrt(13.0) * 0.1);
  int fx = 0;
  int fy = 0;
  edt.feature(6, 4, &fx, &fy);
  EXPECT_EQ(fx, 3);
  EXPECT_EQ(fy, 2);

  // A filter of the wrong size selects nothing.
  const std::vector<std::uint8_t> wrong(3, 1);
  edt.build(grid, wrong);
  EXPECT_FALSE(edt.hasSites());
  EXPECT_TRUE(std::isinf(edt.squaredDistanceCells(3, 2)));
  EXPECT_TRUE(std::isinf(edt.distance(0, 0)));
  edt.feature(0, 0, &fx, &fy);
  EXPECT_EQ(fx, -1);
  EXPECT_EQ(fy, -1);
  EXPECT_FALSE(edt.query(grid.cellCenter(1, 1)).valid);

  // An all-zero filter selects nothing either.
  const std::vector<std::uint8_t> none(mask.size(), 0);
  edt.build(grid, none);
  EXPECT_FALSE(edt.hasSites());
}

// -----------------------------------------------------------------------------------------------
// Change detection.

TEST(EdtUpdate, RebuildsOnlyWhenTheGridChanged)
{
  const int width = 20;
  const int height = 15;
  std::vector<std::uint8_t> mask(width * height, 0);
  mask[7 * width + 9] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2(0.5, 0.25)));
  EuclideanDistanceTransform edt;
  EXPECT_TRUE(edt.update(grid));
  EXPECT_EQ(edt.builtVersion(), grid.version());
  EXPECT_FALSE(edt.update(grid));  // unchanged

  // Same content again: same stamp, no rebuild.
  const std::uint64_t stamp = grid.version();
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2(0.5, 0.25)));
  EXPECT_EQ(grid.version(), stamp);
  EXPECT_FALSE(edt.update(grid));

  // A copy keeps its stamp, so it needs no rebuild either.
  const OccupancyGrid copy = grid;
  EXPECT_FALSE(edt.update(copy));

  // A cell flip.
  mask[2 * width + 3] = 1;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2(0.5, 0.25)));
  EXPECT_NE(grid.version(), stamp);
  EXPECT_TRUE(edt.update(grid));
  EXPECT_EQ(edt.squaredDistanceCells(3, 2), 0.0);
  EXPECT_FALSE(edt.update(grid));

  // An origin shift (rolling costmap) with the same mask.
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2(0.55, 0.25)));
  EXPECT_TRUE(edt.update(grid));
  EXPECT_EQ(edt.origin(), Vec2(0.55, 0.25));
  EXPECT_FALSE(edt.update(grid));

  // A resolution change.
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.1, Vec2(0.55, 0.25)));
  EXPECT_TRUE(edt.update(grid));
  EXPECT_DOUBLE_EQ(edt.resolution(), 0.1);

  // A filtered build forces the next update to rebuild over all sites ...
  std::vector<std::uint8_t> filter(mask.size(), 0);
  filter[7 * width + 9] = 1;
  edt.build(grid, filter);
  EXPECT_EQ(edt.builtVersion(), 0u);
  EXPECT_NE(edt.squaredDistanceCells(3, 2), 0.0);
  EXPECT_TRUE(edt.update(grid));
  EXPECT_EQ(edt.squaredDistanceCells(3, 2), 0.0);
  // ... but an unfiltered build counts as an update.
  edt.build(grid, vf::Span<const std::uint8_t>());
  EXPECT_EQ(edt.builtVersion(), grid.version());
  EXPECT_FALSE(edt.update(grid));

  // The empty grid: nothing to do on an empty transform, a reset on a built one.
  const OccupancyGrid empty;
  EuclideanDistanceTransform blank;
  EXPECT_FALSE(blank.update(empty));
  EXPECT_TRUE(edt.update(empty));
  EXPECT_EQ(edt.width(), 0);
  EXPECT_FALSE(edt.hasSites());
  EXPECT_FALSE(edt.update(empty));
}

// -----------------------------------------------------------------------------------------------
// The continuous query (math_spec §6.1 [Design]).

TEST(EdtQuery, O30_AxisAlignedWallHasNoBilinearRipple)
{
  // A straight wall, one cell thick, across the whole grid; points on lines parallel to it. The
  // bilinear rule reproduces the (linear) wall-foot coordinate, so n_o has no tangential
  // component; the raw nearest-cell feature switches every cell and sweeps ±atan(res/(2 D_o)).
  const int n = 60;
  const double res = 0.05;
  const Vec2 origin(-1.3, 0.7);
  for (int axis = 0; axis < 2; ++axis) {  // 0: wall along y (column 12), 1: along x (row 12)
    std::vector<std::uint8_t> mask(n * n, 0);
    for (int k = 0; k < n; ++k) {
      mask[axis == 0 ? k * n + 12 : 12 * n + k] = 1;
    }
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(mask, n, n, res, origin));
    EuclideanDistanceTransform edt;
    edt.update(grid);
    const Vec2 wall_point = grid.cellCenter(12, 12);
    const Vec2 normal_axis = axis == 0 ? Vec2(1.0, 0.0) : Vec2(0.0, 1.0);
    const Vec2 along_axis = axis == 0 ? Vec2(0.0, 1.0) : Vec2(1.0, 0.0);
    for (const double D : {0.36, 0.123, 0.2, 0.5}) {
      for (const double side : {1.0, -1.0}) {
        const Vec2 normal = side * normal_axis;
        const Vec2 t_lambda_ideal = vf::J(normal);
        double max_tangential = 0.0;
        double ripple_bilinear = 0.0;
        double ripple_raw = 0.0;
        // Stay a cell away from the grid's ends along the wall: in the outer half-cell the clamped
        // stencil holds one row only, which is the raw feature again.
        const int samples = 4000;
        for (int k = 0; k <= samples; ++k) {
          const double along = along_axis.dot(origin) + res + (n - 2) * res * k / samples;
          const Vec2 x =
            wall_point + D * normal + (along - along_axis.dot(wall_point)) * along_axis;
          const ObstacleQuery q = edt.query(x);
          ASSERT_TRUE(q.valid);
          EXPECT_NEAR(q.D_o, D, 1e-12);
          max_tangential = std::max(max_tangential, std::abs(q.n_o.dot(along_axis)));
          ripple_bilinear =
            std::max(ripple_bilinear, angleBetweenDeg(vf::J(q.n_o), t_lambda_ideal));
          // The raw rule: v = x − f(cell containing x).
          int cx = 0;
          int cy = 0;
          ASSERT_TRUE(grid.cellAt(x, &cx, &cy));
          int fx = 0;
          int fy = 0;
          edt.feature(cx, cy, &fx, &fy);
          const Vec2 raw = (x - grid.cellCenter(fx, fy)).normalized();
          ripple_raw = std::max(ripple_raw, angleBetweenDeg(vf::J(raw), t_lambda_ideal));
        }
        const double sawtooth = std::atan(res / (2.0 * D)) * kDeg;
        EXPECT_LE(max_tangential, 1e-12);
        EXPECT_LT(ripple_bilinear, 0.5);  // oracle O30
        EXPECT_LE(ripple_raw, sawtooth + 1e-9);
        EXPECT_GT(ripple_raw, 0.95 * sawtooth);
        if (axis == 0 && side > 0.0) {
          std::printf(
            "[ O30 ] wall at D_o = %.3f m, res %.2f: T_lambda ripple bilinear %.2e deg, raw "
            "feature sawtooth +/-%.3f deg (atan(res/(2 D_o)) = %.3f deg)\n",
            D, res, ripple_bilinear, ripple_raw, sawtooth);
        }
      }
    }
  }
}

TEST(EdtQuery, MedialAxisFallsBackToTheNearestCellFeature)
{
  // A corridor between two walls (columns 10 and 20): cells left of the medial axis see the left
  // wall, cells right of it the right wall. A naive blend of opposite feature vectors would
  // collapse D_o near the axis; the 90° rule takes the nearest cell's feature instead, so D_o
  // never drops below the distance to the nearer wall.
  const int n = 40;
  const double res = 0.05;
  std::vector<std::uint8_t> mask(n * n, 0);
  for (int y = 0; y < n; ++y) {
    mask[y * n + 10] = 1;
    mask[y * n + 20] = 1;
  }
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, res, Vec2(0.0, 0.0)));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const double left = grid.cellCenter(10, 0).x();
  const double right = grid.cellCenter(20, 0).x();
  int fallbacks = 0;
  int blends = 0;
  for (int i = 0; i <= 400; ++i) {
    for (int j = 0; j <= 40; ++j) {
      const Vec2 x(left + 0.5 * res + (right - left - res) * i / 400.0, 0.5 + 0.9 * j / 40.0);
      const ObstacleQuery q = edt.query(x);
      ASSERT_TRUE(q.valid);
      const double nearer_wall = std::min(x.x() - left, right - x.x());
      EXPECT_GE(q.D_o, nearer_wall - 1e-12) << "x = " << x.transpose();
      // Which rule applied: compare with the nearest-cell feature.
      int cx = 0;
      int cy = 0;
      ASSERT_TRUE(grid.cellAt(x, &cx, &cy));
      int fx = 0;
      int fy = 0;
      edt.feature(cx, cy, &fx, &fy);
      const Vec2 raw = x - grid.cellCenter(fx, fy);
      const ObstacleQuery reference = referenceQuery(grid, edt, x);
      EXPECT_NEAR((q.x_o - reference.x_o).norm(), 0.0, 1e-12);
      if (std::abs(q.n_o.y()) > 1e-9) {
        // A tangential component only appears with the raw feature (the blend reproduces y).
        ++fallbacks;
        EXPECT_NEAR((q.x_o - (x - raw)).norm(), 0.0, 1e-15);
      } else {
        ++blends;
      }
    }
  }
  EXPECT_GT(fallbacks, 100);  // the strip between the two centres where the features flip
  EXPECT_GT(blends, 1000);
}

TEST(EdtQuery, MatchesTheSpecificationOnRandomGrids)
{
  std::mt19937_64 rng(61);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int fallbacks = 0;
  int total = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const int width = 1 + static_cast<int>(rng() % 40);
    const int height = 1 + static_cast<int>(rng() % 40);
    const double res = 0.02 + 0.1 * unit(rng);
    const Vec2 origin(4.0 * unit(rng) - 2.0, 4.0 * unit(rng) - 2.0);
    const std::vector<std::uint8_t> mask = randomMask(rng, width, height);
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(mask, width, height, res, origin));
    EuclideanDistanceTransform edt;
    edt.update(grid);
    for (int k = 0; k < 40; ++k) {
      Vec2 x;
      switch (k % 4) {
        case 0:  // anywhere, including the outer half-cells and a margin outside
          x = origin +
              Vec2((width + 2.0) * res * unit(rng) - res, (height + 2.0) * res * unit(rng) - res);
          break;
        case 1:  // exactly a cell centre
          x = grid.cellCenter(static_cast<int>(rng() % width), static_cast<int>(rng() % height));
          break;
        case 2:  // exactly a cell corner
          x = origin + res * Vec2(
                               static_cast<double>(rng() % (width + 1)),
                               static_cast<double>(rng() % (height + 1)));
          break;
        default:
          x = origin + Vec2(width * res * unit(rng), height * res * unit(rng));
          break;
      }
      const ObstacleQuery q = edt.query(x);
      const ObstacleQuery reference = referenceQuery(grid, edt, x);
      ASSERT_EQ(q.valid, reference.valid) << "x = " << x.transpose();
      ++total;
      if (!q.valid) {
        EXPECT_TRUE(std::isinf(q.D_o));
        EXPECT_EQ(q.n_o, Vec2::Zero());
        continue;
      }
      EXPECT_NEAR(q.D_o, reference.D_o, 1e-12);
      EXPECT_NEAR((q.n_o - reference.n_o).norm(), 0.0, 1e-12);
      EXPECT_NEAR((q.x_o - reference.x_o).norm(), 0.0, 1e-12);
      EXPECT_NEAR((x - q.x_o).norm(), q.D_o, 1e-12);
      if (q.D_o > 0.0) {
        EXPECT_NEAR(q.n_o.norm(), 1.0, 1e-12);
      } else {
        EXPECT_EQ(q.n_o, Vec2::Zero());
      }
      int cx = 0;
      int cy = 0;
      grid.cellAt(x, &cx, &cy);
      int fx = 0;
      int fy = 0;
      edt.feature(cx, cy, &fx, &fy);
      if ((q.x_o - grid.cellCenter(fx, fy)).norm() == 0.0) {
        ++fallbacks;
      }
    }
  }
  EXPECT_GT(total, 10000);
  EXPECT_GT(fallbacks, 0);
}

TEST(EdtQuery, PointOnASiteCentre)
{
  const int n = 9;
  std::vector<std::uint8_t> mask(n * n, 0);
  mask[4 * n + 4] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, 0.05, Vec2(0.3, -0.7)));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const Vec2 centre = grid.cellCenter(4, 4);
  const ObstacleQuery q = edt.query(centre);
  EXPECT_TRUE(q.valid);
  EXPECT_EQ(q.D_o, 0.0);
  EXPECT_EQ(q.n_o, Vec2::Zero());
  EXPECT_EQ(q.x_o, centre);
  // With a single site every feature is that site, so the four vectors coincide and the blend is
  // the exact vector to the site centre (only the centre itself takes the zero-vector rule).
  for (const Vec2 & offset :
       {Vec2(0.01, 0.0), Vec2(-0.02, 0.013), Vec2(0.024, -0.024), Vec2(0.0, 0.001)}) {
    const ObstacleQuery near = edt.query(centre + offset);
    ASSERT_TRUE(near.valid);
    EXPECT_NEAR(near.D_o, offset.norm(), 1e-12);
    EXPECT_NEAR((near.n_o - offset.normalized()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((near.x_o - centre).norm(), 0.0, 1e-12);
  }
}

TEST(EdtQuery, OutsideTheGridOrWithoutSitesIsInvalid)
{
  const int width = 10;
  const int height = 6;
  const double res = 0.1;
  const Vec2 origin(-0.5, 0.25);
  std::vector<std::uint8_t> mask(width * height, 0);
  mask[3 * width + 5] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, res, origin));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const double x_end = origin.x() + width * res;
  const double y_end = origin.y() + height * res;
  for (const Vec2 & x :
       {Vec2(origin.x() - 1e-9, 0.4), Vec2(x_end, 0.4), Vec2(x_end + 1.0, 0.4),
        Vec2(0.0, origin.y() - 1e-9), Vec2(0.0, y_end), Vec2(-100.0, -100.0), Vec2(1e300, 0.4),
        Vec2(vf::kNaN, 0.4), Vec2(0.0, vf::kInf)}) {
    const ObstacleQuery q = edt.query(x);
    EXPECT_FALSE(q.valid) << x.transpose();
    EXPECT_TRUE(std::isinf(q.D_o));
    EXPECT_EQ(q.n_o, Vec2::Zero());
  }
  // The corner of cell (0, 0) is inside.
  EXPECT_TRUE(edt.query(origin).valid);

  // No sites.
  std::vector<std::uint8_t> empty(width * height, 0);
  ASSERT_TRUE(grid.setFromMask(empty, width, height, res, origin));
  edt.update(grid);
  EXPECT_FALSE(edt.hasSites());
  const ObstacleQuery none = edt.query(grid.cellCenter(5, 3));
  EXPECT_FALSE(none.valid);
  EXPECT_TRUE(std::isinf(none.D_o));
}

TEST(EdtQuery, ConcurrentConstQueriesAgree)
{
  std::mt19937_64 rng(7);
  const int n = 60;
  const std::vector<std::uint8_t> mask = randomMask(rng, n, n);
  std::vector<std::uint8_t> sites = mask;
  sites[n * n / 2] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(sites, n, n, 0.05, Vec2(-1.5, -1.5)));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  std::uniform_real_distribution<double> coord(-1.6, 1.6);
  std::vector<Vec2> points(5000);
  for (auto & p : points) {
    p = Vec2(coord(rng), coord(rng));
  }
  std::vector<ObstacleQuery> expected(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    expected[i] = edt.query(points[i]);
  }
  std::vector<int> mismatches(4, 0);
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&, t]() {
      for (int pass = 0; pass < 20; ++pass) {
        for (std::size_t i = 0; i < points.size(); ++i) {
          const ObstacleQuery q = edt.query(points[i]);
          if (q.valid != expected[i].valid || (q.valid && q.x_o != expected[i].x_o)) {
            ++mismatches[t];
          }
        }
      }
    });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  for (const int m : mismatches) {
    EXPECT_EQ(m, 0);
  }
}
