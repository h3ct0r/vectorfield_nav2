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

// Steady-state allocation check of the occupancy module: once its
// buffers have grown, a grid update, the distance-transform rebuild, K's flood fill, the EDT_K
// build, obstacle queries and footprint checks perform no heap allocation. Every global
// operator new is counted, so this file is not built under the sanitizers (the test_noalloc
// prefix; math_spec O55).

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <random>
#include <vector>

#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"

namespace
{

std::atomic<long long> g_allocations{0};

void * countedAlloc(std::size_t size) noexcept
{
  g_allocations.fetch_add(1, std::memory_order_relaxed);
  return std::malloc(size == 0 ? 1 : size);
}

void * countedAlignedAlloc(std::size_t size, std::align_val_t alignment) noexcept
{
  g_allocations.fetch_add(1, std::memory_order_relaxed);
  std::size_t align = static_cast<std::size_t>(alignment);
  if (align < sizeof(void *)) {
    align = sizeof(void *);
  }
  void * p = nullptr;
  return posix_memalign(&p, align, size == 0 ? 1 : size) == 0 ? p : nullptr;
}

void * orThrow(void * p)
{
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

}  // namespace

// Replacements of every global allocation function (C++17), all counted.
void * operator new(std::size_t size) { return orThrow(countedAlloc(size)); }
void * operator new[](std::size_t size) { return orThrow(countedAlloc(size)); }
void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  return countedAlloc(size);
}
void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  return countedAlloc(size);
}
void * operator new(std::size_t size, std::align_val_t alignment)
{
  return orThrow(countedAlignedAlloc(size, alignment));
}
void * operator new[](std::size_t size, std::align_val_t alignment)
{
  return orThrow(countedAlignedAlloc(size, alignment));
}
void * operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return countedAlignedAlloc(size, alignment);
}
void * operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return countedAlignedAlloc(size, alignment);
}
void operator delete(void * p) noexcept { std::free(p); }
void operator delete[](void * p) noexcept { std::free(p); }
void operator delete(void * p, std::size_t) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t) noexcept { std::free(p); }
void operator delete(void * p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void * p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete(void * p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void * p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void * p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete(void * p, std::align_val_t, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void * p, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(p);
}

namespace vf = vectorfield_core;
using vf::Vec2;

namespace
{

constexpr int kSide = 60;
constexpr double kRes = 0.05;
const double kRInsc = 0.22 * std::cos(vf::kPi / 16.0);

std::vector<std::uint8_t> sceneMask(int width, int height, int box_x)
{
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(width) * height, 0);
  for (int y = height / 2; y < height / 2 + 6; ++y) {
    for (int x = box_x; x < box_x + 6; ++x) {
      mask[static_cast<std::size_t>(y) * width + x] = 1;
    }
  }
  for (int x = 0; x < width; ++x) {
    mask[static_cast<std::size_t>(height - 5) * width + x] = 1;  // an unrelated wall
  }
  return mask;
}

std::vector<Vec2> sixteenGon(double circumradius)
{
  std::vector<Vec2> polygon;
  for (int k = 0; k < 16; ++k) {
    polygon.push_back(circumradius * vf::heading(vf::kTwoPi * k / 16.0));
  }
  return polygon;
}

}  // namespace

TEST(NoAllocOccupancy, TheCounterSeesLibraryAllocations)
{
  // A zero count below is meaningful only if allocations made inside libvectorfield_core reach
  // the replaced operator new (it is a shared library by default).
  const std::vector<std::uint8_t> mask = sceneMask(kSide, kSide, 30);
  vf::OccupancyGrid grid;
  const long long before = g_allocations.load();
  ASSERT_TRUE(grid.setFromMask(mask, kSide, kSide, kRes, Vec2::Zero()));  // first growth
  EXPECT_GT(g_allocations.load(), before);
}

TEST(NoAllocOccupancy, ReserveMakesTheFirstBuildAllocationFree)
{
  const std::vector<std::uint8_t> mask = sceneMask(kSide, kSide, 30);
  vf::OccupancyGrid grid;
  vf::EuclideanDistanceTransform edt_all;
  vf::EuclideanDistanceTransform edt_k;
  std::vector<std::uint8_t> k_mask;
  std::vector<int> queue;
  grid.reserve(kSide, kSide);
  edt_all.reserve(kSide, kSide);
  edt_k.reserve(kSide, kSide);
  k_mask.reserve(kSide * kSide);
  queue.reserve(kSide * kSide);
  const long long before = g_allocations.load();
  const bool set = grid.setFromMask(mask, kSide, kSide, kRes, Vec2::Zero());
  const bool rebuilt = edt_all.update(grid);
  const std::size_t k_cells =
    vf::floodFillDilatedComponent(edt_all, kRInsc, 32, kSide / 2 + 2, &k_mask, &queue);
  edt_k.build(grid, k_mask);
  const long long allocations = g_allocations.load() - before;
  EXPECT_TRUE(set);
  EXPECT_TRUE(rebuilt);
  EXPECT_GT(k_cells, 36u);
  EXPECT_TRUE(edt_k.hasSites());
  EXPECT_EQ(allocations, 0);
}

TEST(NoAllocOccupancy, SteadyStateIsAllocationFree)
{
  // Two same-size scenes (the box moves by a cell), a smaller grid every fifth cycle (buffers
  // never shrink) and a rolling origin, so every cycle rebuilds the transform.
  const std::vector<std::uint8_t> scene_a = sceneMask(kSide, kSide, 30);
  const std::vector<std::uint8_t> scene_b = sceneMask(kSide, kSide, 31);
  const std::vector<std::uint8_t> small = sceneMask(40, 30, 12);
  const Vec2 origin(-1.5, -1.5);
  const Vec2 k_seed = origin + Vec2(32.5, 32.5) * kRes;  // a box cell centre, grid frame

  vf::OccupancyGrid grid;
  vf::EuclideanDistanceTransform edt_all;
  vf::EuclideanDistanceTransform edt_k;
  std::vector<std::uint8_t> k_mask;
  std::vector<int> queue;
  const std::vector<Vec2> gon = sixteenGon(0.22);
  const vf::FootprintChecker polygon(vf::Footprint::polygon(gon));
  const vf::FootprintChecker disc(vf::Footprint::circle(0.22));
  const vf::FootprintChecker point;

  std::mt19937_64 rng(55);
  std::uniform_real_distribution<double> coord(-1.6, 1.6);
  std::uniform_real_distribution<double> yaw(-vf::kPi, vf::kPi);
  std::uniform_real_distribution<double> ring(0.15, 0.45);
  const Vec2 box_centre = origin + Vec2(33.0, 33.0) * kRes;
  std::vector<Vec2> points(64);
  std::vector<vf::Pose2> poses(64);
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i] = Vec2(coord(rng), coord(rng));
    // Half the poses on rings around the box, so the rasterised ground truth runs too.
    const Vec2 near_box = box_centre + ring(rng) * vf::heading(yaw(rng));
    poses[i] = vf::Pose2{i % 2 == 0 ? Vec2(coord(rng), coord(rng)) : near_box, yaw(rng)};
  }

  auto cycle = [&](int i) -> long long {
    long long work = 0;
    if (i % 5 == 4) {
      work += grid.setFromMask(small, 40, 30, kRes, origin) ? 1 : 0;
    } else {
      const Vec2 rolled = origin + Vec2((i % 3) * kRes, 0.0);
      work += grid.setFromMask(i % 2 == 0 ? scene_a : scene_b, kSide, kSide, kRes, rolled) ? 1 : 0;
    }
    work += edt_all.update(grid) ? 1 : 0;
    int sx = 0;
    int sy = 0;
    if (vf::nearestDilatedCell(edt_all, kRInsc, k_seed, kRInsc + 2.0 * kRes, &sx, &sy)) {
      work += static_cast<long long>(
        vf::floodFillDilatedComponent(edt_all, kRInsc, sx, sy, &k_mask, &queue));
      edt_k.build(grid, k_mask);
    }
    for (const Vec2 & x : points) {
      work += edt_all.query(x).valid ? 1 : 0;
      work += edt_k.query(x).valid ? 1 : 0;
    }
    for (const vf::Pose2 & pose : poses) {
      work += polygon.collides(pose, grid, edt_all) ? 1 : 0;
      work += static_cast<long long>(polygon.classify(pose, grid, edt_all));
      work += polygon.collidesGroundTruth(pose, grid) ? 1 : 0;
      work += polygon.offGrid(pose, grid) ? 1 : 0;
      work += disc.collides(pose, grid, edt_all) ? 1 : 0;
      work += point.collides(pose, grid, edt_all) ? 1 : 0;
    }
    return work;
  };

  for (int i = 0; i < 10; ++i) {  // warm-up: every buffer reaches its largest size
    cycle(i);
  }
  // The poses exercise every rule of the checker (outside the counted window).
  int decisions[5] = {0, 0, 0, 0, 0};
  cycle(0);
  for (const vf::Pose2 & pose : poses) {
    ++decisions[static_cast<int>(polygon.classify(pose, grid, edt_all))];
  }
  for (int d = 0; d < 5; ++d) {
    EXPECT_GT(decisions[d], 0) << "FootprintDecision " << d;
  }
  const long long before = g_allocations.load();
  long long work = 0;
  for (int i = 0; i < 1000; ++i) {
    work += cycle(i);
  }
  const long long allocations = g_allocations.load() - before;
  EXPECT_EQ(allocations, 0);
  EXPECT_GT(work, 1000 * 64);
}
