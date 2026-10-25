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

// One control cycle: Tracker::step with the plugin's setup (target: < 1 ms for a
// 100-vertex window, the 1.5 s collision horizon and contour on). The episode start, which adds
// the two side-probe rollouts, is reported separately.
//
// Each benchmark captures one (input, state) pair from a closed-loop run of the defaults (TB3
// footprint, 20 Hz, obstacle_mode contour) and replays that cycle: step() is a pure function of
// its input and state, so every iteration does the same work.

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;

namespace
{

constexpr int kCells = 80;  // 4 x 4 m at 0.05 m, covering [-2, 2]^2
constexpr double kRes = 0.05;

/// Which cycle of the closed-loop run a benchmark replays.
enum class Cycle
{
  kFreePath,
  kEpisodeStart,
  kActiveEpisode
};

/// A straight 100-vertex (4.95 m) window from (-1.5, 0) along +x, a disc of radius `disc_radius`
/// at (0.5, 0) (none when 0), and the closed loop that drives the defaults toward it.
class Scenario
{
public:
  explicit Scenario(double disc_radius)
  : tracker_(contourConfig()),
    curve_(vf::SearchPolicy::windowScan(-0.5, 0.7)),
    mask_(static_cast<std::size_t>(kCells * kCells), 0)
  {
    std::vector<vf::Vec2> window;
    for (int k = 0; k < 100; ++k) {
      window.emplace_back(-1.5 + 0.05 * k, 0.0);
    }
    curve_.assign(vf::Span<const vf::Vec2>(window), false);
    for (int iy = 0; iy < kCells; ++iy) {
      for (int ix = 0; ix < kCells; ++ix) {
        const vf::Vec2 c(-2.0 + (ix + 0.5) * kRes, -2.0 + (iy + 0.5) * kRes);
        if (disc_radius > 0.0 && (c - vf::Vec2(0.5, 0.0)).norm() <= disc_radius) {
          mask_[static_cast<std::size_t>(iy * kCells + ix)] = 1;
        }
        if (iy == 2) {  // a wall along the bottom edge, far from the path
          mask_[static_cast<std::size_t>(iy * kCells + ix)] = 1;
        }
      }
    }
    grid_.setFromMask(
      vf::Span<const std::uint8_t>(mask_), kCells, kCells, kRes, vf::Vec2(-2.0, -2.0));
    edt_.update(grid_);
    scratch_.reserve(static_cast<std::size_t>(kCells * kCells), 64, 128);
  }

  /// Runs the closed loop from (-1.4, 0.05) until `cycle` and stores that cycle's input and
  /// state. Returns false if the run never reaches it.
  bool capture(Cycle cycle)
  {
    vf::TrackerInput input;
    input.curve = &curve_;
    input.dt = 0.05;
    input.cap_at_curve_end = true;
    vf::GoalInfo goal;
    goal.pose = vf::Pose2{vf::Vec2(3.45, 0.0), 0.0};
    input.goal = goal;
    input.obstacles = vf::ObstacleMap{&grid_, &edt_, vf::Isometry2()};
    input.pose = vf::Pose2{vf::Vec2(-1.4, 0.05), 0.0};

    vf::TrackerState state = vf::Tracker::initialState();
    vf::TrackerState next;
    for (int k = 0; k < 400; ++k) {
      input.t = 0.05 * k;
      const vf::TrackerOutput out = tracker_.step(input, state, &next, scratch_);
      const bool starts = !state.contour.active && next.contour.active;
      const bool ready =
        (cycle == Cycle::kFreePath && k == 20) || (cycle == Cycle::kEpisodeStart && starts) ||
        (cycle == Cycle::kActiveEpisode && state.contour.active && next.contour.active);
      if (ready) {
        input_ = input;
        state_ = state;
        return true;
      }
      state = next;
      input.pose = vf::sim::integrate(tracker_.kinematics(), input.pose, out.cmd, input.dt);
      input.measured = out.cmd;
    }
    return false;
  }

  /// Replays the captured cycle once; returns its output.
  vf::TrackerOutput replay() { return tracker_.step(input_, state_, &next_, scratch_); }

  const vf::TrackerState & capturedState() const { return state_; }

private:
  static vf::CoreConfig contourConfig()
  {
    vf::CoreConfig config;
    config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
    return config;
  }

  vf::Tracker tracker_;
  vf::PolylineCurve curve_;
  std::vector<std::uint8_t> mask_;
  vf::OccupancyGrid grid_;
  vf::EuclideanDistanceTransform edt_;
  vf::StepScratch scratch_;
  vf::TrackerInput input_;
  vf::TrackerState state_{vf::Tracker::initialState()};
  vf::TrackerState next_;
};

void runCycle(benchmark::State & state, double disc_radius, Cycle cycle)
{
  Scenario scenario(disc_radius);
  if (!scenario.capture(cycle)) {
    state.SkipWithError("the closed-loop run never reached the requested cycle");
    return;
  }
  for (auto _ : state) {
    vf::TrackerOutput out = scenario.replay();
    benchmark::DoNotOptimize(out.cmd);
  }
  state.counters["phase"] = static_cast<double>(scenario.capturedState().phase);
}

// FOLLOW on a free path: rollout over the horizon, gate closed (the wall is off the path).
void BM_TrackerStep_FreePath(benchmark::State & state) { runCycle(state, 0.0, Cycle::kFreePath); }
BENCHMARK(BM_TrackerStep_FreePath)->Unit(benchmark::kMicrosecond);

// A cycle inside a contour episode: K flood fill, EDT_K and the rollout under the contour field.
void BM_TrackerStep_ActiveEpisode(benchmark::State & state)
{
  runCycle(state, 0.15, Cycle::kActiveEpisode);
}
BENCHMARK(BM_TrackerStep_ActiveEpisode)->Unit(benchmark::kMicrosecond);

// The cycle that starts an episode: everything above plus the two side-probe rollouts.
void BM_TrackerStep_EpisodeStart(benchmark::State & state)
{
  runCycle(state, 0.15, Cycle::kEpisodeStart);
}
BENCHMARK(BM_TrackerStep_EpisodeStart)->Unit(benchmark::kMicrosecond);

}  // namespace
