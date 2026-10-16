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

// Contour oracle O56, dead ends (math_spec §6.4 "Side probe", §10.4, §12.5).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

// TC with contour (`activation`), path y = 0 from (−1, 0) to a free goal (6, 0) behind a blockage
// at x = 3, robot at (0.5, 0) heading +x. Runs until kBlocked has been set for 3 s, or 15 s.
std::vector<Cycle> runIntoBlockage(const ObstacleWorld & world, vf::ObstacleActivation activation)
{
  vf::CoreConfig config = tcContourConfig();
  config.obstacles.obstacle_activation = activation;
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(6.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.stop_when_stopped = false;
  Runner runner(config, options);
  runner.reset(Pose2{Vec2(0.5, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(6.0, 0.0));
  runner.input().obstacles = world.map();
  double blocked_at = -1.0;
  runner.run(15.0, [&blocked_at](const Cycle & c) {
    if (blocked_at < 0.0 && Status::has(c.out.status, Status::kBlocked)) {
      blocked_at = c.rec.t;
    }
    return blocked_at >= 0.0 && c.rec.t > blocked_at + 3.0;
  });
  return runner.cycles();
}

// O56 with `path_blocked`: no episode, never a collision, kBlocked set and kept, stopped at least
// m_s − res and less than 3 m_s short of the wall (ground-truth translation to contact along +x).
void expectStopsShort(const ObstacleWorld & world)
{
  const std::vector<Cycle> cs = runIntoBlockage(world, vf::ObstacleActivation::kPathBlocked);
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(tcFootprint()));
  for (const Cycle & c : cs) {
    ASSERT_FALSE(c.after.contour.active) << "episode at t = " << c.rec.t;
    ASSERT_FALSE(checker.collidesGroundTruth(c.pose_after, world.grid())) << "t = " << c.rec.t;
  }
  const std::size_t k =
    firstIndex(cs, [](const Cycle & c) { return Status::has(c.out.status, Status::kBlocked); });
  ASSERT_LT(k, cs.size()) << "never kBlocked";
  for (std::size_t i = k; i < cs.size(); ++i) {
    EXPECT_TRUE(Status::has(cs[i].out.status, Status::kBlocked)) << "t = " << cs[i].rec.t;
  }
  EXPECT_GT(cs.back().rec.t, cs[k].rec.t + 2.9) << "kBlocked not kept for 3 s";
  EXPECT_LT(std::abs(cs.back().rec.cmd.vx), 1e-3) << "did not stop";
  const double gap = travelToContact(checker, world, cs.back().pose_after, Vec2(1.0, 0.0));
  EXPECT_GE(gap, kStopMargin - kRes);
  EXPECT_LT(gap, 3.0 * kStopMargin) << "stopped early";
}

// A 1.2 m corridor (0.1 m walls from x = −1) closed by a wall across the path at x = 3.
ObstacleWorld deadEndCorridor()
{
  ObstacleWorld world = centredWorld(-1.5, 6.5, -2.0, 2.0);
  world.addBox(Vec2(-1.0, 0.6), Vec2(3.1, 0.7));
  world.addBox(Vec2(-1.0, -0.7), Vec2(3.1, -0.6));
  world.addBox(Vec2(3.0, -0.7), Vec2(3.1, 0.7));
  world.commit();
  return world;
}

}  // namespace

// O56 Dead end (math_spec §6.4 "Side probe", past the blockage): a straight path into a 1.2 m
// corridor closed by a wall across it, `path_blocked`. Neither side probe gets past the wall, so
// no episode starts: the cap stops the robot short of the wall, and kBlocked follows (§10.4) and
// stays set.
TEST(OracleContour, O56_DeadEndCorridorStops) { expectStopsShort(deadEndCorridor()); }

// O56 with the O38 wall: a wall across the path far wider than v_r T_side is a blockage, not a
// contour, with `path_blocked`.
TEST(OracleContour, O56_LongWallAcrossPathStops)
{
  ObstacleWorld world(Vec2(-2.0, -6.0), 8.0, 12.0);
  world.addBox(Vec2(3.0, -7.0), Vec2(3.1, 7.0));
  world.commit();
  expectStopsShort(world);
}

// The dead end with `always`, safety only. `always` keeps the plain exit test, and in a corridor
// narrower than 2 D'_in the side walls alone start episodes (the paper's nearest obstacle), so the
// episodes chatter at the end wall and liveness (stop, kBlocked) is not part of its contract; the
// cap still keeps the footprint free.
TEST(OracleContour, O56_DeadEndCorridorAlwaysNeverCollides)
{
  const ObstacleWorld world = deadEndCorridor();
  const std::vector<Cycle> cs = runIntoBlockage(world, vf::ObstacleActivation::kAlways);
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(tcFootprint()));
  for (const Cycle & c : cs) {
    ASSERT_FALSE(checker.collidesGroundTruth(c.pose_after, world.grid())) << "t = " << c.rec.t;
  }
}
