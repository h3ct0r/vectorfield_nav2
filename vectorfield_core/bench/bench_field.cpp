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

// Field evaluation: closest point plus DistanceField::evaluate, the per-call cost of every field
// query (target: < 5 µs on a 2000-point polyline).
//
// The evaluation point walks along the curve 1 cm per call, 0.1 m off it, and the hint is carried
// from call to call the way the Tracker carries it, so the hinted search sees the access pattern
// of a robot driving the curve.

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;

namespace
{

/// y = 0.5 sin(x) sampled every 0.05 m in x: `count` vertices.
std::vector<vf::Vec2> sineVertices(std::size_t count)
{
  std::vector<vf::Vec2> vertices;
  vertices.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    const double x = 0.05 * static_cast<double>(k);
    vertices.emplace_back(x, 0.5 * std::sin(x));
  }
  return vertices;
}

/// A point 0.1 m to the left of the sine curve at abscissa x.
vf::Vec2 probeNear(double x)
{
  const vf::Vec2 tangent = vf::Vec2(1.0, 0.5 * std::cos(x)).normalized();
  return vf::Vec2(x, 0.5 * std::sin(x)) + 0.1 * vf::Vec2(-tangent.y(), tangent.x());
}

/// Runs the walking field query against `curve` over abscissas [0, x_end).
void walkCurve(benchmark::State & state, const vf::Curve & curve, double x_end)
{
  const vf::DistanceField field(vf::GainFunction(vf::GainType::kAtan, 2.0), 0.4, 1, 0.1);
  vf::EvalContext context;
  vf::CurveHint hint;
  vf::FieldSample sample;
  double x = 0.0;
  for (auto _ : state) {
    const vf::Vec2 p = probeNear(x);
    const vf::ClosestPoint cp = curve.closestPoint(p, context, hint);
    sample = vf::FieldSample{};
    field.evaluate(p, cp, vf::Vec2::Zero(), &sample);
    benchmark::DoNotOptimize(sample.F);
    hint = vf::makeHint(cp);
    context.last_tangent = cp.T;
    x += 0.01;
    if (x >= x_end) {
      x = 0.0;
      hint = vf::CurveHint{};  // restart: the next call runs the global stage again
    }
  }
}

// The follower: a 2000-vertex polyline with the Hinted search (CurveOptions defaults).
void BM_FieldEval_Polyline2k_Hinted(benchmark::State & state)
{
  const std::vector<vf::Vec2> vertices = sineVertices(2000);
  vf::PolylineCurve curve(vf::SearchPolicy::hinted(vf::CurveOptions{}));
  if (curve.assign(vf::Span<const vf::Vec2>(vertices), false) != vf::AssignStatus::kOk) {
    state.SkipWithError("assign failed");
    return;
  }
  walkCurve(state, curve, 0.05 * 1999.0 - 1.0);
}
BENCHMARK(BM_FieldEval_Polyline2k_Hinted);

// The follower's first call of a goal: the hint is invalid, so the global stage scans all pieces.
void BM_FieldEval_Polyline2k_GlobalStage(benchmark::State & state)
{
  const std::vector<vf::Vec2> vertices = sineVertices(2000);
  vf::PolylineCurve curve(vf::SearchPolicy::hinted(vf::CurveOptions{}));
  if (curve.assign(vf::Span<const vf::Vec2>(vertices), false) != vf::AssignStatus::kOk) {
    state.SkipWithError("assign failed");
    return;
  }
  const vf::DistanceField field(vf::GainFunction(vf::GainType::kAtan, 2.0), 0.4, 1, 0.1);
  const vf::EvalContext context;
  const vf::CurveHint no_hint;
  const vf::Vec2 p = probeNear(50.0);
  vf::FieldSample sample;
  for (auto _ : state) {
    const vf::ClosestPoint cp = curve.closestPoint(p, context, no_hint);
    sample = vf::FieldSample{};
    field.evaluate(p, cp, vf::Vec2::Zero(), &sample);
    benchmark::DoNotOptimize(sample.F);
  }
}
BENCHMARK(BM_FieldEval_Polyline2k_GlobalStage);

// The plugin: a 100-vertex (5 m) window searched with WindowScan over [-(d + 0.3), d + 0.5]
// with d = 0.2, re-assigned every 100 calls the way the plugin re-assigns each cycle's window.
void BM_FieldEval_Window100_WindowScan(benchmark::State & state)
{
  const std::vector<vf::Vec2> vertices = sineVertices(100);
  vf::PolylineCurve curve(vf::SearchPolicy::windowScan(-0.5, 0.7));
  curve.reserve(vertices.size());
  const vf::DistanceField field(vf::GainFunction(vf::GainType::kAtan, 2.0), 0.4, 1, 0.1);
  vf::EvalContext context;
  vf::CurveHint hint;
  vf::FieldSample sample;
  std::size_t calls = 0;
  for (auto _ : state) {
    if (calls++ % 100 == 0) {
      benchmark::DoNotOptimize(curve.assign(vf::Span<const vf::Vec2>(vertices), false));
    }
    const vf::Vec2 p = probeNear(0.1);
    const vf::ClosestPoint cp = curve.closestPoint(p, context, hint);
    sample = vf::FieldSample{};
    field.evaluate(p, cp, vf::Vec2::Zero(), &sample);
    benchmark::DoNotOptimize(sample.F);
    hint = vf::makeHint(cp);
  }
}
BENCHMARK(BM_FieldEval_Window100_WindowScan);

// A built-in parametric curve (ellipse 2 x 1): windowed golden section with secant polish.
void BM_FieldEval_BuiltinEllipse(benchmark::State & state)
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kBuiltin;
  spec.builtin_shape = "ellipse";
  spec.parameter_names = {"a", "b"};
  spec.parameter_values = {2.0, 1.0};
  const auto curve = vf::makeCurve(spec, vf::CurveOptions{});
  const vf::DistanceField field(vf::GainFunction(vf::GainType::kAtan, 2.0), 0.4, 1, 0.1);
  vf::EvalContext context;
  vf::CurveHint hint;
  vf::FieldSample sample;
  double u = 0.0;
  for (auto _ : state) {
    const vf::Vec2 p(2.1 * std::cos(u), 1.1 * std::sin(u));
    const vf::ClosestPoint cp = curve->closestPoint(p, context, hint);
    sample = vf::FieldSample{};
    field.evaluate(p, cp, vf::Vec2::Zero(), &sample);
    benchmark::DoNotOptimize(sample.F);
    hint = vf::makeHint(cp);
    context.last_tangent = cp.T;
    u += 0.005;
  }
}
BENCHMARK(BM_FieldEval_BuiltinEllipse);

}  // namespace
