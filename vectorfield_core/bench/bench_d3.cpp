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

}  // namespace
