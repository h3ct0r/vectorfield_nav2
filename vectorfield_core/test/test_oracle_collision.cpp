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

// Closed-loop oracles of collision prediction (math_spec §10, §12.8): O50 cancel with an
// obstacle, O51 kBlocked hysteresis, O52 free arc length.
//
// Geometry: TC footprint (16-gon, front vertex exactly 0.22 m ahead of base_link), boxes whose
// faces lie on grid lines, so the analytic contact distance along a straight approach is
// x_face − 0.22 − p_x.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

vf::FootprintChecker tcChecker()
{
  return vf::FootprintChecker(vf::Footprint::fromSpec(tcFootprint()));
}

// A grid line of `world` at or below x.
double gridLineBelow(const ObstacleWorld & world, double x)
{
  return world.origin().x() +
         std::floor((x - world.origin().x()) / world.resolution()) * world.resolution();
}

}  // namespace

// O50 Cancel with an obstacle (math_spec §8.4, §9.2, §10.2): a_can = 0.3, a_dec = 1.0, cancel at
// 0.4 m/s on a straight; one cycle later a box appears at most 0.2 m (at least 0.15 m: its face is
// snapped down to a grid line) ahead of the footprint front, inside the 0.267 m gentle stopping
// distance and outside the hard one plus a cycle: the robot stops with the footprint free,
// STATUS_COLLISION_AHEAD is set, |v| < 0.01 is reached, and the cancel completes. Timeout 3 s.
TEST(OracleCollision, O50_CancelWithObstacle)
{
  vf::CoreConfig config = tcConfig();
  config.phases.cancel_deceleration = 0.3;
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(20.0, 0.0)}, false);
  ObstacleWorld world(Vec2(-2.0, -2.0), 10.0, 4.0);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.stop_when_stopped = false;
  Runner runner(config, options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0}, 0.0, Twist2{0.4, 0.0, 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(20.0, 0.0));
  runner.input().obstacles = world.map();
  runner.input().cancel = true;
  runner.step();
  const double front = runner.pose().p.x() + kRCirc;
  const double face = gridLineBelow(world, front + 0.2);
  ASSERT_GT(face - front, 0.15 - 1e-9);
  ASSERT_LE(face - front, 0.2 + 1e-9);
  world.addBox(Vec2(face, -0.5), Vec2(face + 0.5, 0.5));
  world.commit();
  runner.run(3.0, [](const Cycle & c) { return c.out.cancel_complete; });
  const auto & cs = runner.cycles();
  const vf::FootprintChecker checker = tcChecker();
  bool collision_ahead = false;
  for (std::size_t k = 1; k < cs.size(); ++k) {
    EXPECT_EQ(cs[k].out.phase, Phase::kStopped);
    EXPECT_FALSE(checker.collidesGroundTruth(cs[k].pose_after, world.grid()))
      << "t = " << cs[k].rec.t;
    collision_ahead = collision_ahead || Status::has(cs[k].out.status, Status::kCollisionAhead);
  }
  EXPECT_TRUE(collision_ahead);
  EXPECT_LT(std::abs(cs.back().rec.cmd.vx), 0.01);
  EXPECT_TRUE(cs.back().out.cancel_complete);
  EXPECT_LT(runner.pose().p.x() + kRCirc, face);
}

// O51 kBlocked hysteresis (math_spec §10.4): a blocked robot at rest; the lethal edge in front of
// it flickers by ±1 cell every cycle, so the gap to the footprint alternates between 0.08 m
// (blocked: s_free < m_s + v_stop²/(2a_dec)) and 0.13 m (s_free may exceed that, but stays below
// the release level m_s + h_b = 0.2 m): kBlocked stays set and the commanded speed stays 0.
TEST(OracleCollision, O51_BlockedHysteresisWithFlicker)
{
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(10.0, 0.0)}, false);
  ObstacleWorld world(Vec2(-2.0, -2.0), 8.0, 4.0);
  const int wall = world.addBox(Vec2(0.30, -1.0), Vec2(0.60, 1.0));
  world.commit();
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcConfig(), options);
  const Pose2 start{Vec2(0.0, 0.0), 0.0};
  runner.reset(start);
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(10.0, 0.0));
  runner.input().obstacles = world.map();
  int cycle = 0;
  runner.setPreStep([&](Runner &, vf::TrackerInput &) {
    const double face = cycle % 2 == 0 ? 0.30 : 0.35;
    world.moveBox(wall, Vec2(face, -1.0), Vec2(face + 0.30, 1.0));
    world.commit();
    ++cycle;
  });
  runner.run(3.0);
  for (const Cycle & c : runner.cycles()) {
    EXPECT_TRUE(Status::has(c.out.status, Status::kBlocked)) << "t = " << c.rec.t;
    EXPECT_LE(std::abs(c.rec.cmd.vx), 1e-12) << "t = " << c.rec.t;
  }
  EXPECT_LE((runner.pose().p - start.p).norm(), 1e-12);
}

// O52 Free arc length (math_spec §10.2): straight approach to a box at 0.4 m/s. Whenever a
// collision is predicted, s_free lies within one sub-step (<= res) below the analytic last-free
// arc x_face − 0.22 − p_x and never above it; while the box is beyond what a 1.5 s rollout at
// v_r can reach, s_free = +inf; the robot stops >= m_s − res short of contact. Timeout 12 s.
TEST(OracleCollision, O52_FreeArcLengthStraightApproach)
{
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(10.0, 0.0)}, false);
  ObstacleWorld world(Vec2(-2.0, -2.0), 10.0, 4.0);
  const double face = 2.0;
  world.addBox(Vec2(face, -0.5), Vec2(face + 0.5, 0.5));
  world.commit();
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0}, 0.0, Twist2{0.4, 0.0, 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(10.0, 0.0));
  runner.input().obstacles = world.map();
  int still = 0;
  runner.run(12.0, [&](const Cycle & c) {
    still = std::abs(c.rec.cmd.vx) < 1e-6 ? still + 1 : 0;
    return still >= 20;
  });
  int finite = 0;
  for (const Cycle & c : runner.cycles()) {
    const double contact = face - kRCirc - c.rec.pose.p.x();
    if (std::isfinite(c.out.s_free)) {
      ++finite;
      EXPECT_LE(c.out.s_free, contact + 1e-9) << "t = " << c.rec.t;
      EXPECT_GE(c.out.s_free, contact - kRes - 1e-9) << "t = " << c.rec.t;
    } else {
      EXPECT_GT(contact, 0.5) << "collision-free rollout although contact is near, t = " << c.rec.t;
    }
    if (contact > kVr * kHorizon + 0.05) {
      EXPECT_TRUE(std::isinf(c.out.s_free)) << "t = " << c.rec.t;
    }
  }
  EXPECT_GT(finite, 10);
  const double final_gap = face - kRCirc - runner.pose().p.x();
  EXPECT_GE(final_gap, kStopMargin - kRes);
  EXPECT_GT(final_gap, 0.0);
}

// O52, rotation (math_spec §10.2): a rectangular robot (±0.3 × ±0.15 m) 0.05 m beside a wall
// parallel to its long axis, path behind it: ROTATE_TO_FIELD starts, the swept rotation collides,
// s_free = 0 and kBlocked is set (rotation rule of math_spec §10.4).
TEST(OracleCollision, O52_RotationIntoWallGivesZero)
{
  vf::CoreConfig config = tcConfig();
  config.node.footprint.polygon = {
    Vec2(0.3, 0.15), Vec2(-0.3, 0.15), Vec2(-0.3, -0.15), Vec2(0.3, -0.15)};
  const auto curve = makePolyline({Vec2(0.0, 0.0), Vec2(-3.0, 0.0)}, false);
  ObstacleWorld world(Vec2(-2.0, -2.0), 4.0, 4.0);
  world.addBox(Vec2(-1.0, 0.2), Vec2(1.0, 0.5));
  world.commit();
  const vf::Tracker tracker(config);
  vf::StepScratch scratch;
  vf::TrackerInput input;
  input.pose = Pose2{Vec2(0.0, 0.0), 0.0};
  input.dt = kPeriod;
  input.curve = curve.get();
  input.goal = goalAt(Vec2(-3.0, 0.0));
  input.obstacles = world.map();
  vf::TrackerState state = vf::Tracker::initialState();
  const vf::TrackerOutput out = tracker.step(input, state, &state, scratch);
  ASSERT_EQ(out.phase, Phase::kRotateToField);
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(config.node.footprint));
  ASSERT_FALSE(checker.collidesGroundTruth(input.pose, world.grid()));
  EXPECT_EQ(out.s_free, 0.0);
  EXPECT_TRUE(Status::has(out.status, Status::kBlocked));
  EXPECT_LE(std::abs(out.cmd.vx), 1e-12);
}
