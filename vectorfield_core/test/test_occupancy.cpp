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

// Occupancy grid snapshot, K's flood fill and K re-seeding (math_spec §6.1, §6.4 "Identity K").

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::EuclideanDistanceTransform;
using vf::OccupancyGrid;
using vf::Vec2;

namespace
{

// TC footprint (math_spec §12): 16-gon of circumradius 0.22, r_insc = 0.22 cos(pi/16).
const double kRInsc = 0.22 * std::cos(vf::kPi / 16.0);

std::vector<std::uint8_t> boxMask(
  int width, int height,
  const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> & boxes)
{
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(width) * height, 0);
  for (const auto & box : boxes) {
    for (int y = box.first.second; y <= box.second.second; ++y) {
      for (int x = box.first.first; x <= box.second.first; ++x) {
        mask[static_cast<std::size_t>(y) * width + x] = 1;
      }
    }
  }
  return mask;
}

std::size_t countOnes(const std::vector<std::uint8_t> & v)
{
  return static_cast<std::size_t>(
    std::count_if(v.begin(), v.end(), [](std::uint8_t c) { return c != 0; }));
}

// Dilated mask by brute force over the site list, with the same predicate as the definition
// {c : EDT_all(c) <= r}: sqrt(squared cell distance) * res <= r.
std::vector<std::uint8_t> referenceDilated(
  const std::vector<std::uint8_t> & mask, int width, int height, double r, double res)
{
  std::vector<std::pair<int, int>> sites;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (mask[static_cast<std::size_t>(y) * width + x] != 0) {
        sites.emplace_back(x, y);
      }
    }
  }
  std::vector<std::uint8_t> dilated(mask.size(), 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      long long best = -1;
      for (const auto & s : sites) {
        const long long dx = s.first - x;
        const long long dy = s.second - y;
        if (best < 0 || dx * dx + dy * dy < best) {
          best = dx * dx + dy * dy;
        }
      }
      if (best >= 0 && std::sqrt(static_cast<double>(best)) * res <= r) {
        dilated[static_cast<std::size_t>(y) * width + x] = 1;
      }
    }
  }
  return dilated;
}

// 8-connected component of `seed` in a binary mask, by an explicit stack (a different traversal
// from the library's breadth-first fill).
std::vector<std::uint8_t> referenceComponent(
  const std::vector<std::uint8_t> & dilated, int width, int height, int seed_x, int seed_y)
{
  std::vector<std::uint8_t> component(dilated.size(), 0);
  if (dilated[static_cast<std::size_t>(seed_y) * width + seed_x] == 0) {
    return component;
  }
  std::vector<std::pair<int, int>> stack{{seed_x, seed_y}};
  component[static_cast<std::size_t>(seed_y) * width + seed_x] = 1;
  while (!stack.empty()) {
    const auto [x, y] = stack.back();
    stack.pop_back();
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }
        const std::size_t i = static_cast<std::size_t>(ny) * width + nx;
        if (dilated[i] != 0 && component[i] == 0) {
          component[i] = 1;
          stack.emplace_back(nx, ny);
        }
      }
    }
  }
  return component;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// OccupancyGrid

TEST(OccupancyGrid, EmptyByDefault)
{
  const OccupancyGrid grid;
  EXPECT_EQ(grid.width(), 0);
  EXPECT_EQ(grid.height(), 0);
  EXPECT_EQ(grid.version(), 0u);
  EXPECT_TRUE(grid.mask().empty());
  EXPECT_FALSE(grid.inBounds(0, 0));
  EXPECT_FALSE(grid.isSite(0, 0));
  int ix = 0;
  int iy = 0;
  EXPECT_FALSE(grid.cellAt(Vec2(0.0, 0.0), &ix, &iy));
}

TEST(OccupancyGrid, RejectsInconsistentArguments)
{
  const std::vector<std::uint8_t> mask(12, 1);
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, 4, 3, 0.05, Vec2(1.0, 2.0)));
  const std::uint64_t stamp = grid.version();
  EXPECT_NE(stamp, 0u);
  const std::vector<std::uint8_t> short_mask(11, 1);
  EXPECT_FALSE(grid.setFromMask(short_mask, 4, 3, 0.05, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 3, 3, 0.05, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 0, 12, 0.05, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, -4, -3, 0.05, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 4, 3, 0.0, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 4, 3, -0.05, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 4, 3, vf::kNaN, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 4, 3, vf::kInf, Vec2(1.0, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 4, 3, 0.05, Vec2(vf::kNaN, 2.0)));
  EXPECT_FALSE(grid.setFromMask(mask, 4, 3, 0.05, Vec2(1.0, vf::kInf)));
  EXPECT_FALSE(grid.setFromMask(vf::Span<const std::uint8_t>(), 4, 3, 0.05, Vec2(1.0, 2.0)));
  // Unchanged by every rejection.
  EXPECT_EQ(grid.version(), stamp);
  EXPECT_EQ(grid.width(), 4);
  EXPECT_EQ(grid.height(), 3);
  EXPECT_EQ(grid.origin(), Vec2(1.0, 2.0));
}

TEST(OccupancyGrid, CellConventions)
{
  // Cell (ix, iy) has index iy * width + ix and covers [o + ix res, o + (ix + 1) res) per axis.
  const int width = 5;
  const int height = 3;
  std::vector<std::uint8_t> mask(width * height, 0);
  mask[1 * width + 3] = 200;  // any nonzero value is a site
  OccupancyGrid grid;
  const double res = 0.25;
  const Vec2 origin(-1.0, 0.5);
  ASSERT_TRUE(grid.setFromMask(mask, width, height, res, origin));
  EXPECT_TRUE(grid.isSite(3, 1));
  EXPECT_FALSE(grid.isSite(1, 3));
  EXPECT_FALSE(grid.isSite(-1, 0));
  EXPECT_FALSE(grid.isSite(5, 0));
  EXPECT_EQ(grid.mask()[1 * width + 3], 1);  // normalised to 1
  EXPECT_EQ(countOnes(std::vector<std::uint8_t>(grid.mask().begin(), grid.mask().end())), 1u);
  EXPECT_TRUE(grid.inBounds(4, 2));
  EXPECT_FALSE(grid.inBounds(4, 3));
  EXPECT_EQ(grid.cellCenter(3, 1), Vec2(-1.0 + 3.5 * res, 0.5 + 1.5 * res));
  EXPECT_EQ(grid.cellCenter(-1, 7), Vec2(-1.0 - 0.5 * res, 0.5 + 7.5 * res));  // any index

  int ix = -1;
  int iy = -1;
  ASSERT_TRUE(grid.cellAt(origin, &ix, &iy));  // the corner belongs to cell (0, 0)
  EXPECT_EQ(ix, 0);
  EXPECT_EQ(iy, 0);
  ASSERT_TRUE(grid.cellAt(Vec2(-0.25, 0.75), &ix, &iy));  // a shared corner: the upper cell
  EXPECT_EQ(ix, 3);
  EXPECT_EQ(iy, 1);
  ASSERT_TRUE(grid.cellAt(Vec2(0.2499, 1.2499), &ix, &iy));
  EXPECT_EQ(ix, 4);
  EXPECT_EQ(iy, 2);
  EXPECT_FALSE(grid.cellAt(Vec2(0.25, 1.0), &ix, &iy));  // the far edge is outside
  EXPECT_FALSE(grid.cellAt(Vec2(0.0, 1.25), &ix, &iy));
  EXPECT_FALSE(grid.cellAt(Vec2(-1.0 - 1e-12, 1.0), &ix, &iy));
  EXPECT_FALSE(grid.cellAt(Vec2(vf::kNaN, 1.0), &ix, &iy));
  EXPECT_FALSE(grid.cellAt(Vec2(1e300, 1.0), &ix, &iy));
  EXPECT_TRUE(grid.cellAt(Vec2(-0.9, 0.6), nullptr, nullptr));
}

TEST(OccupancyGrid, VersionStampsTrackContent)
{
  const int width = 6;
  const int height = 4;
  std::vector<std::uint8_t> mask(width * height, 0);
  mask[5] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2::Zero()));
  const std::uint64_t v1 = grid.version();
  EXPECT_NE(v1, 0u);

  // Same site flags (a different nonzero value is still a site): same stamp.
  mask[5] = 77;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2::Zero()));
  EXPECT_EQ(grid.version(), v1);

  // A copy keeps the stamp until either changes.
  OccupancyGrid copy = grid;
  EXPECT_EQ(copy.version(), v1);

  mask[6] = 1;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2::Zero()));
  const std::uint64_t v2 = grid.version();
  EXPECT_NE(v2, v1);
  EXPECT_EQ(copy.version(), v1);

  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2(0.05, 0.0)));  // origin shift
  const std::uint64_t v3 = grid.version();
  EXPECT_NE(v3, v2);
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.1, Vec2(0.05, 0.0)));  // resolution
  const std::uint64_t v4 = grid.version();
  EXPECT_NE(v4, v3);
  ASSERT_TRUE(grid.setFromMask(mask, height, width, 0.1, Vec2(0.05, 0.0)));  // shape
  const std::uint64_t v5 = grid.version();
  EXPECT_NE(v5, v4);

  // Stamps are unique per process: the copy changing never collides with the original.
  std::vector<std::uint8_t> other(width * height, 1);
  ASSERT_TRUE(copy.setFromMask(other, width, height, 0.05, Vec2::Zero()));
  for (const std::uint64_t v : {v1, v2, v3, v4, v5}) {
    EXPECT_NE(copy.version(), v);
  }
}

TEST(OccupancyGrid, ReserveKeepsTheStorage)
{
  OccupancyGrid grid;
  grid.reserve(60, 60);
  std::vector<std::uint8_t> small(30 * 20, 1);
  ASSERT_TRUE(grid.setFromMask(small, 30, 20, 0.05, Vec2::Zero()));
  const std::uint8_t * storage = grid.mask().data();
  std::vector<std::uint8_t> full(60 * 60, 0);
  ASSERT_TRUE(grid.setFromMask(full, 60, 60, 0.05, Vec2::Zero()));
  EXPECT_EQ(grid.mask().data(), storage);
  ASSERT_TRUE(grid.setFromMask(small, 30, 20, 0.05, Vec2::Zero()));
  EXPECT_EQ(grid.mask().data(), storage);
  EXPECT_EQ(grid.mask().size(), 600u);
}

// -----------------------------------------------------------------------------------------------
// K's flood fill

TEST(OccupancyFloodFill, EightConnectivity)
{
  const int width = 12;
  const int height = 10;
  // Two sites touching at a corner only, and a third two cells to the right of the second.
  const std::vector<std::uint8_t> mask =
    boxMask(width, height, {{{3, 3}, {3, 3}}, {{4, 4}, {4, 4}}, {{6, 4}, {6, 4}}});
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  std::vector<std::uint8_t> k;
  std::vector<int> queue;
  // r = 0: the dilated mask is the sites themselves; diagonal neighbours connect.
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, 0.0, 3, 3, &k, &queue), 2u);
  EXPECT_EQ(k[3 * width + 3], 1);
  EXPECT_EQ(k[4 * width + 4], 1);
  EXPECT_EQ(k[4 * width + 6], 0);
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, 0.0, 6, 4, &k, &queue), 1u);
  // r = res: the one-cell gap (5, 4) is dilated and bridges all three; the result is the whole
  // dilation, not only the sites.
  const std::size_t merged = vf::floodFillDilatedComponent(edt, 0.05, 6, 4, &k, &queue);
  EXPECT_EQ(k[3 * width + 3], 1);
  EXPECT_EQ(k[4 * width + 5], 1);
  EXPECT_EQ(merged, countOnes(referenceDilated(mask, width, height, 0.05, 0.05)));
  EXPECT_EQ(k.size(), static_cast<std::size_t>(width * height));
}

TEST(OccupancyFloodFill, GapsNarrowerThanTheDilationMerge)
{
  // Two blocks separated by g empty columns, dilated by the TC r_insc (4.3 cells at 0.05 m): the
  // middle gap cell lies ceil(g/2) cells from the nearer block, so the blocks merge iff
  // ceil(g/2) res <= r_insc, i.e. for gaps up to 8 cells (0.40 m < 2 r_insc = 0.43 m).
  const int width = 60;
  const int height = 30;
  const double res = 0.05;
  for (int gap = 0; gap <= 12; ++gap) {
    const int right_start = 10 + gap;
    const std::vector<std::uint8_t> mask =
      boxMask(width, height, {{{5, 10}, {9, 18}}, {{right_start, 10}, {right_start + 4, 18}}});
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(mask, width, height, res, Vec2(-1.0, -0.5)));
    EuclideanDistanceTransform edt;
    edt.update(grid);
    std::vector<std::uint8_t> k;
    std::vector<int> queue;
    ASSERT_GT(vf::floodFillDilatedComponent(edt, kRInsc, 7, 14, &k, &queue), 0u);
    const bool merged = k[14 * width + right_start + 2] != 0;
    const bool expected = std::ceil(gap / 2.0) * res <= kRInsc;
    EXPECT_EQ(merged, expected) << "gap " << gap << " cells";
    EXPECT_EQ(merged, gap <= 8) << "gap " << gap << " cells";
  }
}

TEST(OccupancyFloodFill, MatchesAReferenceOnRandomGrids)
{
  std::mt19937_64 rng(46);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::vector<std::uint8_t> k;
  std::vector<int> queue;
  int non_trivial = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const int width = 1 + static_cast<int>(rng() % 40);
    const int height = 1 + static_cast<int>(rng() % 40);
    const double res = 0.02 + 0.08 * unit(rng);
    const double density = 0.15 * unit(rng);
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(width) * height);
    for (auto & c : mask) {
      c = unit(rng) < density ? 1 : 0;
    }
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(mask, width, height, res, Vec2(unit(rng), -unit(rng))));
    EuclideanDistanceTransform edt;
    edt.update(grid);
    const double r = 4.0 * res * unit(rng);
    const std::vector<std::uint8_t> dilated = referenceDilated(mask, width, height, r, res);
    for (int s = 0; s < 5; ++s) {
      const int sx = static_cast<int>(rng() % width);
      const int sy = static_cast<int>(rng() % height);
      const std::size_t count = vf::floodFillDilatedComponent(edt, r, sx, sy, &k, &queue);
      const std::vector<std::uint8_t> expected = referenceComponent(dilated, width, height, sx, sy);
      ASSERT_EQ(k, expected) << "trial " << trial << " seed (" << sx << ", " << sy << ")";
      EXPECT_EQ(count, countOnes(expected));
      non_trivial += count > 1 ? 1 : 0;
    }
  }
  EXPECT_GT(non_trivial, 200);
}

TEST(OccupancyFloodFill, SeedOutsideOrNotDilated)
{
  const int width = 10;
  const int height = 8;
  const std::vector<std::uint8_t> mask = boxMask(width, height, {{{2, 2}, {3, 3}}});
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  std::vector<std::uint8_t> k(3, 9);  // stale content is cleared and resized
  std::vector<int> queue;
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, 0.1, -1, 0, &k, &queue), 0u);
  EXPECT_EQ(k.size(), static_cast<std::size_t>(width * height));
  EXPECT_EQ(countOnes(k), 0u);
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, 0.1, 0, height, &k, &queue), 0u);
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, 0.1, 9, 7, &k, &queue), 0u);  // far from the box
  EXPECT_EQ(countOnes(k), 0u);
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, -0.1, 2, 2, &k, &queue), 0u);  // no dilation
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, vf::kNaN, 2, 2, &k, &queue), 0u);
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, 0.1, 2, 2, &k, nullptr), 0u);
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, 0.1, 2, 2, nullptr, &queue), 0u);
  // The seed cell itself, at the dilation boundary (exactly 2 cells = 0.1 m from the box).
  EXPECT_GT(vf::floodFillDilatedComponent(edt, 0.1, 5, 2, &k, &queue), 0u);
  EXPECT_EQ(k[2 * width + 5], 1);
  EXPECT_EQ(k[2 * width + 6], 0);
  // An infinite radius dilates every cell.
  EXPECT_EQ(
    vf::floodFillDilatedComponent(edt, vf::kInf, 9, 7, &k, &queue),
    static_cast<std::size_t>(width * height));
  // No sites: nothing is dilated.
  const std::vector<std::uint8_t> empty(width * height, 0);
  ASSERT_TRUE(grid.setFromMask(empty, width, height, 0.05, Vec2::Zero()));
  edt.update(grid);
  EXPECT_EQ(vf::floodFillDilatedComponent(edt, vf::kInf, 2, 2, &k, &queue), 0u);
}

TEST(OccupancyFloodFill, EdtKMeasuresToTheSitesInK)
{
  // A box near the path and an unrelated wall farther away (math_spec §6.4 rationale): EDT_K over
  // K = the box's dilated component measures to the box's real cells only.
  const int width = 60;
  const int height = 40;
  const double res = 0.05;
  const std::vector<std::uint8_t> mask =
    boxMask(width, height, {{{20, 10}, {25, 15}}, {{0, 30}, {59, 30}}});
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, res, Vec2(-1.5, -1.0)));
  EuclideanDistanceTransform edt_all;
  edt_all.update(grid);
  std::vector<std::uint8_t> k;
  std::vector<int> queue;
  const std::size_t count = vf::floodFillDilatedComponent(edt_all, kRInsc, 22, 12, &k, &queue);
  ASSERT_GT(count, 36u);             // the box and its dilation
  EXPECT_EQ(k[30 * width + 30], 0);  // the wall is another component
  EuclideanDistanceTransform edt_k;
  edt_k.build(grid, k);
  EXPECT_EQ(edt_k.builtVersion(), 0u);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      long long best = -1;
      for (int by = 10; by <= 15; ++by) {
        for (int bx = 20; bx <= 25; ++bx) {
          const long long d2 =
            static_cast<long long>(bx - x) * (bx - x) + static_cast<long long>(by - y) * (by - y);
          best = best < 0 ? d2 : std::min(best, d2);
        }
      }
      ASSERT_EQ(edt_k.squaredDistanceCells(x, y), static_cast<double>(best));
    }
  }
  // Right next to the wall, EDT_all sees the wall and EDT_K the box.
  const Vec2 near_wall = grid.cellCenter(23, 29);
  EXPECT_NEAR(edt_all.query(near_wall).D_o, res, 1e-12);
  EXPECT_NEAR(edt_k.query(near_wall).D_o, 14 * res, 1e-12);
  EXPECT_NEAR(edt_k.query(near_wall).n_o.y(), 1.0, 1e-12);
}

// -----------------------------------------------------------------------------------------------
// Re-seeding K during an episode

TEST(OccupancyNearestDilated, MatchesBruteForce)
{
  std::mt19937_64 rng(53);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int found = 0;
  int missing = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const int width = 1 + static_cast<int>(rng() % 40);
    const int height = 1 + static_cast<int>(rng() % 40);
    const double res = 0.02 + 0.08 * unit(rng);
    const Vec2 origin(2.0 * unit(rng) - 1.0, 2.0 * unit(rng) - 1.0);
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(width) * height);
    const double density = 0.05 * unit(rng);
    for (auto & c : mask) {
      c = unit(rng) < density ? 1 : 0;
    }
    OccupancyGrid grid;
    ASSERT_TRUE(grid.setFromMask(mask, width, height, res, origin));
    EuclideanDistanceTransform edt;
    edt.update(grid);
    for (int s = 0; s < 40; ++s) {
      const double r = 3.0 * res * unit(rng);
      const double radius = r + 2.0 * res * unit(rng) + (s % 7 == 0 ? 0.0 : res);
      const Vec2 x = origin + Vec2(
                                (width + 4.0) * res * unit(rng) - 2.0 * res,
                                (height + 4.0) * res * unit(rng) - 2.0 * res);
      double best = vf::kInf;
      for (int iy = 0; iy < height; ++iy) {
        for (int ix = 0; ix < width; ++ix) {
          const double d = (grid.cellCenter(ix, iy) - x).norm();
          if (edt.distance(ix, iy) <= r && d <= radius) {
            best = std::min(best, d);
          }
        }
      }
      int cx = -1;
      int cy = -1;
      const bool ok = vf::nearestDilatedCell(edt, r, x, radius, &cx, &cy);
      ASSERT_EQ(ok, std::isfinite(best)) << "trial " << trial << " sample " << s;
      if (ok) {
        ++found;
        EXPECT_LE(edt.distance(cx, cy), r);
        EXPECT_DOUBLE_EQ((grid.cellCenter(cx, cy) - x).norm(), best);
      } else {
        ++missing;
      }
    }
  }
  EXPECT_GT(found, 1000);
  EXPECT_GT(missing, 1000);
}

TEST(OccupancyNearestDilated, TheContainingDilatedCellWins)
{
  const int width = 20;
  const int height = 20;
  const std::vector<std::uint8_t> mask = boxMask(width, height, {{{8, 8}, {11, 11}}});
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, width, height, 0.05, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  int cx = -1;
  int cy = -1;
  // Inside a dilated cell (one cell left of the box): that cell.
  const Vec2 x = grid.cellCenter(7, 9) + Vec2(0.02, -0.024);
  ASSERT_TRUE(vf::nearestDilatedCell(edt, kRInsc, x, kRInsc + 0.1, &cx, &cy));
  EXPECT_EQ(cx, 7);
  EXPECT_EQ(cy, 9);
  // Bad inputs.
  EXPECT_FALSE(vf::nearestDilatedCell(edt, kRInsc, x, -1.0, &cx, &cy));
  EXPECT_FALSE(vf::nearestDilatedCell(edt, kRInsc, x, vf::kNaN, &cx, &cy));
  EXPECT_FALSE(vf::nearestDilatedCell(edt, kRInsc, Vec2(vf::kNaN, 0.0), 1.0, &cx, &cy));
  EXPECT_FALSE(vf::nearestDilatedCell(edt, -1.0, x, 1.0, &cx, &cy));
  EXPECT_FALSE(vf::nearestDilatedCell(EuclideanDistanceTransform(), kRInsc, x, 1.0, &cx, &cy));
}

TEST(OccupancyKIdentity, LostWhenTheObstacleIsRemovedOrOutOfReach)
{
  // An episode against a box; k_seed is the last x_o (grid frame). Each cycle it is mapped
  // through the grid and re-seeded within r_insc + 2 res (math_spec §6.4).
  const int width = 60;
  const int height = 60;
  const double res = 0.05;
  const Vec2 origin(-1.5, -1.5);
  const double reach = kRInsc + 2.0 * res;
  const std::vector<std::uint8_t> with_box =
    boxMask(width, height, {{{30, 30}, {35, 35}}, {{0, 55}, {59, 55}}});
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(with_box, width, height, res, origin));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const vf::ObstacleQuery q = edt.query(grid.cellCenter(26, 32) + Vec2(0.01, 0.0));
  ASSERT_TRUE(q.valid);
  const Vec2 k_seed = q.x_o;  // on the box
  int cx = -1;
  int cy = -1;
  ASSERT_TRUE(vf::nearestDilatedCell(edt, kRInsc, k_seed, reach, &cx, &cy));
  std::vector<std::uint8_t> k;
  std::vector<int> queue;
  ASSERT_GT(vf::floodFillDilatedComponent(edt, kRInsc, cx, cy, &k, &queue), 0u);
  EXPECT_EQ(k[32 * width + 32], 1);
  EXPECT_EQ(k[55 * width + 30], 0);

  // The box shrinks by two cells on the seed's side: still within reach.
  const std::vector<std::uint8_t> shrunk =
    boxMask(width, height, {{{32, 30}, {35, 35}}, {{0, 55}, {59, 55}}});
  ASSERT_TRUE(grid.setFromMask(shrunk, width, height, res, origin));
  ASSERT_TRUE(edt.update(grid));
  EXPECT_TRUE(vf::nearestDilatedCell(edt, kRInsc, k_seed, reach, &cx, &cy));

  // The box is removed: K is lost (the far wall is out of reach).
  const std::vector<std::uint8_t> removed = boxMask(width, height, {{{0, 55}, {59, 55}}});
  ASSERT_TRUE(grid.setFromMask(removed, width, height, res, origin));
  ASSERT_TRUE(edt.update(grid));
  EXPECT_FALSE(vf::nearestDilatedCell(edt, kRInsc, k_seed, reach, &cx, &cy));
  // And so is a seed beyond reach of anything, or off the grid.
  EXPECT_FALSE(vf::nearestDilatedCell(edt, kRInsc, grid.cellCenter(30, 20), reach, &cx, &cy));
  EXPECT_FALSE(vf::nearestDilatedCell(edt, kRInsc, Vec2(10.0, 10.0), reach, &cx, &cy));
  // Everything removed: no sites at all.
  const std::vector<std::uint8_t> nothing(width * height, 0);
  ASSERT_TRUE(grid.setFromMask(nothing, width, height, res, origin));
  ASSERT_TRUE(edt.update(grid));
  EXPECT_FALSE(vf::nearestDilatedCell(edt, kRInsc, k_seed, reach, &cx, &cy));
}

TEST(OccupancyKIdentity, FollowsARollingOrigin)
{
  // A rolling costmap: the same world box seen through origins shifted by 1..10 cells. k_seed is
  // a grid-frame point, so it keeps naming the same box, and K covers the same world cells.
  const int width = 60;
  const int height = 60;
  const double res = 0.05;
  const Vec2 origin(-1.5, -1.5);
  const int box_x0 = 28;
  const int box_y0 = 26;
  auto mask_for_shift = [&](int sx, int sy) {
    return boxMask(
      width, height,
      {{{box_x0 - sx, box_y0 - sy}, {box_x0 - sx + 5, box_y0 - sy + 3}},
       {{box_x0 - sx + 15, box_y0 - sy}, {box_x0 - sx + 15, box_y0 - sy + 10}}});
  };
  auto k_cells_world = [&](const OccupancyGrid & grid, const std::vector<std::uint8_t> & k) {
    std::vector<std::pair<long long, long long>> cells;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (k[static_cast<std::size_t>(y) * width + x] != 0) {
          const Vec2 c = grid.cellCenter(x, y);
          cells.emplace_back(std::llround(c.x() / res * 2.0), std::llround(c.y() / res * 2.0));
        }
      }
    }
    std::sort(cells.begin(), cells.end());
    return cells;
  };
  OccupancyGrid grid;
  EuclideanDistanceTransform edt;
  std::vector<std::uint8_t> k;
  std::vector<int> queue;
  const std::vector<std::uint8_t> unshifted = mask_for_shift(0, 0);
  ASSERT_TRUE(grid.setFromMask(unshifted, width, height, res, origin));
  edt.update(grid);
  const Vec2 k_seed = grid.cellCenter(box_x0, box_y0 + 1);  // a box cell, as an x_o would be
  int cx = -1;
  int cy = -1;
  ASSERT_TRUE(vf::nearestDilatedCell(edt, kRInsc, k_seed, kRInsc + 2 * res, &cx, &cy));
  vf::floodFillDilatedComponent(edt, kRInsc, cx, cy, &k, &queue);
  const auto reference = k_cells_world(grid, k);
  ASSERT_FALSE(reference.empty());
  for (int shift = 1; shift <= 10; ++shift) {
    const int sx = shift;
    const int sy = -(shift % 4);
    const Vec2 shifted = origin + res * Vec2(sx, sy);
    const std::vector<std::uint8_t> mask = mask_for_shift(sx, sy);
    ASSERT_TRUE(grid.setFromMask(mask, width, height, res, shifted));
    ASSERT_TRUE(edt.update(grid));
    ASSERT_TRUE(vf::nearestDilatedCell(edt, kRInsc, k_seed, kRInsc + 2 * res, &cx, &cy));
    vf::floodFillDilatedComponent(edt, kRInsc, cx, cy, &k, &queue);
    EXPECT_EQ(k_cells_world(grid, k), reference) << "shift " << shift;
  }
}
