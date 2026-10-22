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

// The experimental 3D core (math_spec §14): closest point + field on a built-in helix and Viviani
// curve (hint carried, as in Tracker3), the global stage, a 3D polyline, and one Tracker3::step.

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "vectorfield_core/d3.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace d3 = vectorfield_core::d3;
using d3::Vec3;

std::shared_ptr<const d3::Curve3> builtin(const std::string & name)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kBuiltin;
  spec.builtin_shape = name;
  return d3::makeCurve3(spec, {});
}

const d3::DistanceField3 & field()
{
  static const d3::DistanceField3 f(vf::GainFunction(vf::GainType::kAtan, 2.0), 0.5, 1, 0.1);
  return f;
}

// Evaluates the field at points walking along the curve 2 cm off it, carrying the hint.
void walk(benchmark::State & state, const d3::Curve3 & curve, bool hinted)
{
  const double L = curve.length(0.0);
  std::vector<Vec3> points;
  for (int k = 0; k < 256; ++k) {
    const double s = 0.5 + (L - 1.0) * k / 256.0;
    points.push_back(curve.pointAt(s, 0.0) + Vec3(0.0, 0.0, 0.02));
  }
  d3::CurveHint3 hint;
  d3::FieldSample3 sample;
  std::size_t k = 0;
  for (auto _ : state) {
    const Vec3 & x = points[k];
    k = (k + 1) % points.size();
    const d3::ClosestPoint3 cp = curve.closestPoint(x, d3::EvalContext3{}, hint);
    field().evaluate(x, cp, Vec3::Zero(), &sample);
    benchmark::DoNotOptimize(sample.Phi);
    hint = hinted && k != 0 ? d3::makeHint(cp) : d3::CurveHint3{};
  }
}

void BM_D3FieldEval_Helix_Hinted(benchmark::State & state) { walk(state, *builtin("helix"), true); }
BENCHMARK(BM_D3FieldEval_Helix_Hinted);

void BM_D3FieldEval_Helix_GlobalStage(benchmark::State & state)
{
  walk(state, *builtin("helix"), false);
}
BENCHMARK(BM_D3FieldEval_Helix_GlobalStage);

void BM_D3FieldEval_Viviani_Hinted(benchmark::State & state)
{
  walk(state, *builtin("viviani"), true);
}
BENCHMARK(BM_D3FieldEval_Viviani_Hinted);

void BM_D3FieldEval_Polyline2k_Hinted(benchmark::State & state)
{
  std::vector<Vec3> points;
  for (int k = 0; k < 2000; ++k) {
    const double u = 0.01 * k;
    points.emplace_back(std::cos(u) * (3.0 + 0.1 * u), std::sin(u) * 3.0, 0.05 * u);
  }
  const d3::PolylineCurve3 curve(points, false);
  walk(state, curve, true);
}
BENCHMARK(BM_D3FieldEval_Polyline2k_Hinted);

void BM_D3TrackerStep_Helix(benchmark::State & state)
{
  const auto curve = builtin("helix");
  const d3::Tracker3 tracker{d3::Tracker3Config{}};
  d3::Tracker3Input input;
  input.curve = curve.get();
  input.pose.p = curve->pointAt(5.0, 0.0) + Vec3(0.0, 0.0, 0.02);
  d3::Tracker3State warm = d3::Tracker3::initialState();
  for (int k = 0; k < 5; ++k) {
    tracker.step(input, warm, &warm);
  }
  for (auto _ : state) {
    d3::Tracker3State next;
    d3::Tracker3Output out = tracker.step(input, warm, &next);
    benchmark::DoNotOptimize(out.cmd);
  }
}
BENCHMARK(BM_D3TrackerStep_Helix);

// The 3D CBF filter alone (math_spec §14.8): four sites in general position (floor, two walls,
// a sphere), every active set enumerated.
void BM_D3CbfSolve_FourSites(benchmark::State & state)
{
  d3::Cbf3Constraint c[4];
  c[0] = {Vec3::UnitZ(), 0.02};
  c[1] = {Vec3(0.0, 1.0, 0.0), 0.05};
  c[2] = {Vec3(0.0, -1.0, 0.0), 0.3};
  c[3] = {Vec3(-0.6, 0.0, 0.8), 0.04};
  Vec3 F(0.4, -0.2, -0.25);
  for (auto _ : state) {
    benchmark::DoNotOptimize(F);
    d3::Cbf3Solution s = d3::solveCbf3(F, c, 4, 2.0);
    benchmark::DoNotOptimize(s.F);
  }
}
BENCHMARK(BM_D3CbfSolve_FourSites);

// One Tracker3::step on the helix with the CBF filter active: a sphere beside the body, a floor
// and a pole (primitives), and the same plus a 64^3 voxel grid (window features).
void cbfStep(benchmark::State & state, bool voxels)
{
  const auto curve = builtin("helix");
  d3::Tracker3Config config;
  config.obstacles.obstacle_mode = vf::ObstacleMode::kCbf;
  const d3::Tracker3 tracker{config};
  const Vec3 p = curve->pointAt(5.0, 0.0) + Vec3(0.0, 0.0, 0.02);
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::sphere(p + 0.55 * curve->tangentAt(5.0, 0.0), 0.2));
  obstacles.add(d3::Primitive3::plane(Vec3(0.0, 0.0, p.z() - 0.32), Vec3::UnitZ()));
  obstacles.add(d3::Primitive3::cylinder(Vec3(0.0, 0.0, 0.0), 0.1));
  if (voxels) {
    const int n = 64;
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(n * n * n), 0);
    for (int iz = 0; iz < n; ++iz) {
      for (int iy = 0; iy < n; ++iy) {
        mask[static_cast<std::size_t>(40 + n * (iy + n * iz))] = 1;  // a wall at x = p.x + 0.33
      }
    }
    d3::VoxelGrid3 grid;
    grid.setFromMask(mask, n, n, n, 0.05, p - Vec3(2.0 - 0.025, 1.6, 1.6));
    obstacles.setVoxels(grid);
  }
  d3::Tracker3Input input;
  input.curve = curve.get();
  input.obstacles = &obstacles;
  input.pose.p = p;
  d3::Tracker3State warm = d3::Tracker3::initialState();
  for (int k = 0; k < 5; ++k) {
    tracker.step(input, warm, &warm);
  }
  for (auto _ : state) {
    d3::Tracker3State next;
    d3::Tracker3Output out = tracker.step(input, warm, &next);
    benchmark::DoNotOptimize(out.cmd);
  }
  state.counters["sites"] = tracker.step(input, warm, nullptr).cbf.count;
}

void BM_D3TrackerStep_Helix_CbfPrimitives(benchmark::State & state) { cbfStep(state, false); }
BENCHMARK(BM_D3TrackerStep_Helix_CbfPrimitives);

void BM_D3TrackerStep_Helix_CbfVoxels(benchmark::State & state) { cbfStep(state, true); }
BENCHMARK(BM_D3TrackerStep_Helix_CbfVoxels);

// The voxel EDT of a 64^3 grid (262 144 voxels): random 5 % occupancy, rebuilt every iteration.
void BM_D3VoxelEdt_64(benchmark::State & state)
{
  const int n = 64;
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(n * n * n), 0);
  std::uint32_t seed = 12345U;
  for (auto & m : mask) {
    seed = seed * 1664525U + 1013904223U;
    m = (seed >> 24U) < 13U ? 1 : 0;
  }
  d3::VoxelGrid3 grid;
  grid.setFromMask(mask, n, n, n, 0.05, Vec3::Zero());
  d3::DistanceTransform3 edt;
  edt.build(grid);
  for (auto _ : state) {
    edt.build(grid);
    benchmark::DoNotOptimize(edt.squaredDistanceCells(10, 20, 30));
  }
  state.counters["voxels/s"] = benchmark::Counter(
    static_cast<double>(n * n * n), benchmark::Counter::kIsIterationInvariantRate);
}
BENCHMARK(BM_D3VoxelEdt_64)->Unit(benchmark::kMillisecond);

}  // namespace
