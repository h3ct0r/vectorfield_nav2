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

// Contour oracle O38, the watchdog on a fixed curve and on re-pruned windows (math_spec §6.4,
// §12.5).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

}  // namespace

namespace
{

struct WatchdogResult
{
  bool tripped{false};
  double travelled{vf::kNaN};  // ContourSample::travelled_m at the trip cycle
  double odometry{vf::kNaN};   // base path length from the start to the trip cycle
  Vec2 position{Vec2::Zero()};
  bool stopped_after{false};
};

// A wall perpendicular to the path at x = 3 that extends beyond the grid (y in [−7, 7], grid |y| <= 6),
// path y = 0 → (8, 0).
// `windowed`: plugin-style re-pruned windows (raw plan every 2.5 cm, prune 2 m, WindowScan,
// cap_at_curve_end); else the fixed follower curve. Runs until kContourWatchdog or 40 s.
WatchdogResult runWatchdog(bool windowed)
{
  ObstacleWorld world(Vec2(-2.0, -6.0), 8.0, 12.0);
  // The wall runs past both grid edges (|y| > 6 m): no circulation can reach an end within the
  // 5 m watchdog length (rounding the end of a 4 m half-wall takes about 4.9 m of travel).
  world.addBox(Vec2(3.0, -7.0), Vec2(3.1, 7.0));
  world.commit();
  const auto fixed_curve = makePolyline({Vec2(-1.0, 0.0), Vec2(8.0, 0.0)}, false);
  PrunedWindow window(linePoints(Vec2(-1.0, 0.0), Vec2(8.0, 0.0), 0.025), 2.0);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.stop_when_stopped = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0});
  runner.input().goal = goalAt(Vec2(8.0, 0.0));
  runner.input().obstacles = world.map();
  if (windowed) {
    runner.input().cap_at_curve_end = true;
    runner.setPreStep(
      [&window](Runner & r, vf::TrackerInput & in) { in.curve = &window.update(r.pose().p); });
  } else {
    runner.input().curve = fixed_curve.get();
  }
  WatchdogResult w;
  runner.run(
    40.0, [](const Cycle & c) { return Status::has(c.out.status, Status::kContourWatchdog); });
  const auto & cs = runner.cycles();
  const std::size_t k = firstIndex(
    cs, [](const Cycle & c) { return Status::has(c.out.status, Status::kContourWatchdog); });
  if (k < cs.size()) {
    w.tripped = true;
    w.travelled = cs[k].out.field.contour.travelled_m;
    w.odometry = odometry(cs, k);
    w.position = cs[k].rec.pose.p;
    w.stopped_after = cs[k].out.phase == Phase::kStopped;
  }
  return w;
}

}  // namespace

// O38 Watchdog (math_spec §6.4 "Watchdog"): a perpendicular wall longer than the limit trips
// kContourWatchdog (→ STOPPED) after > 5 m of episode travel, at the same travelled distance
// (episode travel and odometry, ±1 cell) and the same place (±1 cell) on a fixed curve and on
// re-pruned windows. Timeout 40 s per run.
TEST(OracleContour, O38_WatchdogSameOnFixedCurveAndWindows)
{
  const WatchdogResult fixed = runWatchdog(false);
  const WatchdogResult windowed = runWatchdog(true);
  ASSERT_TRUE(fixed.tripped);
  ASSERT_TRUE(windowed.tripped);
  EXPECT_TRUE(fixed.stopped_after);
  EXPECT_TRUE(windowed.stopped_after);
  EXPECT_GT(fixed.travelled, 5.0);
  EXPECT_LT(fixed.travelled, 5.0 + kRes);
  EXPECT_NEAR(fixed.travelled, windowed.travelled, kRes);
  EXPECT_NEAR(fixed.odometry, windowed.odometry, kRes);
  EXPECT_LE((fixed.position - windowed.position).norm(), kRes);
}
