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

// Obstacle scenarios not covered by the O3x oracles (math_spec §6.3–§6.4): a
// perpendicular wall is bypassed (the latch removes the perpendicular-wall trap), and a valid
// path through a door never starts the contour (the `path_blocked` gate rationale). The too-long
// wall is O38 (test_oracle_contour_tracking.cpp); the box with a parallel wall is O34.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

std::vector<Cycle> runToGoal(
  ObstacleWorld & world, const Pose2 & start, const Vec2 & goal, double timeout)
{
  const auto curve = makePolyline({Vec2(start.p.x() - 1.0, 0.0), goal}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(start);
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(goal);
  runner.input().obstacles = world.map();
  runner.run(timeout, [](const Cycle & c) { return c.out.settled; });
  return runner.cycles();
}

}  // namespace

// Perpendicular wall (math_spec §6.3: "A wall perpendicular to the path always traps" the
// unlatched rule): a 1.2 m wall across the path, start 2 cm off the path. The latched contour
// bypasses it: exactly one episode with constant σ_c, no watchdog, no kBlocked, the base passes
// the wall, rejoins the path (|y| < 0.05 m) and settles at the goal. Timeout 40 s.
TEST(ScenarioObstacles, PerpendicularWallBypassed)
{
  ObstacleWorld world(Vec2(-2.0, -3.0), 10.0, 6.0);
  world.addBox(Vec2(3.0, -0.6), Vec2(3.1, 0.6));
  world.commit();
  const std::vector<Cycle> cs = runToGoal(world, Pose2{Vec2(0.0, 0.02), 0.0}, Vec2(7.0, 0.0), 40.0);
  const std::vector<Episode> eps = episodes(cs);
  ASSERT_EQ(eps.size(), 1U);
  EXPECT_TRUE(eps[0].sigma_constant);
  EXPECT_FALSE(anyStatus(cs, Status::kContourWatchdog));
  EXPECT_FALSE(anyStatus(cs, Status::kBlocked));
  const std::size_t k_pass = firstIndex(cs, [](const Cycle & c) { return c.rec.pose.p.x() > 3.6; });
  ASSERT_LT(k_pass, cs.size()) << "never passed the wall";
  bool rejoined = false;
  for (std::size_t k = k_pass; k < cs.size(); ++k) {
    rejoined = rejoined || std::abs(cs[k].rec.pose.p.y()) < 0.05;
  }
  EXPECT_TRUE(rejoined);
  EXPECT_TRUE(cs.back().out.settled);
}

// Door passage (math_spec §6.4 gate rationale; corridor_door: zero contour
// activations): a straight path through a 0.9 m door (jambs 0.45 m from the path, 0.23 m of
// footprint clearance): the gate never opens, no episode starts, no kBlocked, and the goal beyond
// the door is reached. Timeout 30 s.
TEST(ScenarioObstacles, DoorPassageNeverStartsContour)
{
  ObstacleWorld world(Vec2(-2.0, -3.0), 10.0, 6.0);
  world.addBox(Vec2(3.0, 0.45), Vec2(3.2, 2.5));
  world.addBox(Vec2(3.0, -2.5), Vec2(3.2, -0.45));
  world.commit();
  const std::vector<Cycle> cs = runToGoal(world, Pose2{Vec2(0.0, 0.0), 0.0}, Vec2(6.0, 0.0), 30.0);
  for (const Cycle & c : cs) {
    EXPECT_FALSE(c.out.field.contour.gate_open) << "t = " << c.rec.t;
    EXPECT_FALSE(c.after.contour.active) << "t = " << c.rec.t;
    EXPECT_FALSE(Status::has(c.out.status, Status::kBlocked)) << "t = " << c.rec.t;
  }
  EXPECT_TRUE(cs.back().out.settled);
}
