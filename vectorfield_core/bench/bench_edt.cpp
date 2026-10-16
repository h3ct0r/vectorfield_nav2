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

// Obstacle distance: the Felzenszwalb–Huttenlocher distance and feature transform and the
// bilinear query (plan §3.5 target: < 100 µs for the transform of a 60 x 60 grid, the stock
// 3 x 3 m local costmap at 0.05 m).

#include <benchmark/benchmark.h>

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;

namespace
{

/// A local-costmap-like mask: a wall along one side, two boxes and 2% scattered cells (fixed seed).
std::vector<std::uint8_t> costmapMask(int size)
{
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(size * size), 0);
  const auto set = [&](int ix, int iy) { mask[static_cast<std::size_t>(iy * size + ix)] = 1; };
  for (int ix = 0; ix < size; ++ix) {
    set(ix, size - 3);
  }
  for (int iy = size / 4; iy < size / 4 + 6; ++iy) {
    for (int ix = size / 2; ix < size / 2 + 8; ++ix) {
      set(ix, iy);
    }
  }
  for (int iy = size / 2; iy < size / 2 + 4; ++iy) {
    for (int ix = size / 5; ix < size / 5 + 4; ++ix) {
      set(ix, iy);
    }
  }
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> cell(0, size * size - 1);
  for (int k = 0; k < size * size / 50; ++k) {
    mask[static_cast<std::size_t>(cell(rng))] = 1;
  }
  return mask;
}

vf::OccupancyGrid makeGrid(int size)
{
  const std::vector<std::uint8_t> mask = costmapMask(size);
  vf::OccupancyGrid grid;
  grid.setFromMask(vf::Span<const std::uint8_t>(mask), size, size, 0.05, vf::Vec2(-1.5, -1.5));
  return grid;
}

// A full rebuild over all sites (what a changed costmap costs once per cycle).
void BM_Edt_Build(benchmark::State & state)
{
  const int size = static_cast<int>(state.range(0));
  const vf::OccupancyGrid grid = makeGrid(size);
  vf::EuclideanDistanceTransform edt;
  edt.reserve(size, size);
  for (auto _ : state) {
    edt.build(grid, vf::Span<const std::uint8_t>());
    benchmark::DoNotOptimize(edt.hasSites());
  }
  state.counters["cells"] = static_cast<double>(size * size);
}
BENCHMARK(BM_Edt_Build)->Arg(60)->Arg(200)->Unit(benchmark::kMicrosecond);

// update() on an unchanged grid: the version check that skips the rebuild.
void BM_Edt_UpdateUnchanged(benchmark::State & state)
{
  const vf::OccupancyGrid grid = makeGrid(60);
  vf::EuclideanDistanceTransform edt;
  edt.update(grid);
  for (auto _ : state) {
    benchmark::DoNotOptimize(edt.update(grid));
  }
}
BENCHMARK(BM_Edt_UpdateUnchanged);

// The continuous bilinear query (distance and normal), on points spread over the grid.
void BM_Edt_Query(benchmark::State & state)
{
  const vf::OccupancyGrid grid = makeGrid(60);
  vf::EuclideanDistanceTransform edt;
  edt.update(grid);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> coordinate(-1.45, 1.45);
  std::vector<vf::Vec2> points(1024);
  for (vf::Vec2 & p : points) {
    p = vf::Vec2(coordinate(rng), coordinate(rng));
  }
  std::size_t k = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(edt.query(points[k++ & 1023U]));
  }
}
BENCHMARK(BM_Edt_Query);

}  // namespace
