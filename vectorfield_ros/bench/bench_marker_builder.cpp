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

// MarkerBuilder cost per cycle: the full `markers` set (target < 0.2 ms) and the
// 12 x 12 `field_grid` (reported separately), on a plugin-style cycle in a contour episode, so
// K's cells, the rings and the prediction are all drawn.

#include <benchmark/benchmark.h>

#include <memory>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_ros/marker_builder.hpp"
#include "vectorfield_ros/viz_config.hpp"
#include "viz_test_support.hpp"

namespace
{

namespace vf = vectorfield_core;
using vf::Pose2;
using vf::Vec2;

/// @brief A contour episode on a 3 m window of 61 vertices (the plugin's typical window).
struct Fixture
{
  Fixture() : world(Vec2(-2.5, -2.5), 9.5, 5.0)
  {
    world.addDisc(Vec2(2.0, 0.0), 0.3);
    world.commit();
    std::vector<Vec2> window;
    for (int k = 0; k <= 60; ++k) {
      window.emplace_back(-1.0 + 0.05 * k, 0.0);
    }
    window.emplace_back(6.0, 0.0);
    scenario = std::make_unique<vf_viz_test::Scenario>(
      vf_viz_test::contourConfig(), window, Pose2{Vec2(-1.0, 0.05), 0.0});
    scenario->input().goal = vf_viz_test::goalAt(Vec2(6.0, 0.0));
    scenario->input().obstacles = world.map();
    scenario->runUntil([](const vf::TrackerOutput & out) { return out.field.contour.active; }, 400);
  }

  vf_viz_test::World world;
  std::unique_ptr<vf_viz_test::Scenario> scenario;
};

void BM_Markers(benchmark::State & state)
{
  const Fixture fixture;
  const vectorfield_ros::MarkerContext context = fixture.scenario->context();
  vectorfield_ros::MarkerBuilder builder;
  for (auto _ : state) {
    benchmark::DoNotOptimize(&builder.markers(context));
  }
  state.counters["contour_active"] = fixture.scenario->output().field.contour.active ? 1.0 : 0.0;
}
BENCHMARK(BM_Markers)->Unit(benchmark::kMicrosecond);

void BM_FieldGrid(benchmark::State & state)
{
  const Fixture fixture;
  const vectorfield_ros::MarkerContext context = fixture.scenario->context();
  vectorfield_ros::MarkerBuilder builder;
  vectorfield_ros::VizConfig config;  // 3.0 m at 0.25 m: 12 x 12
  config.field_grid_style = static_cast<vectorfield_ros::FieldGridStyle>(state.range(0));
  for (auto _ : state) {
    benchmark::DoNotOptimize(&builder.fieldGrid(context, config));
  }
}
BENCHMARK(BM_FieldGrid)->Arg(0)->Arg(1)->ArgNames({"streamlines"})->Unit(benchmark::kMicrosecond);

}  // namespace
