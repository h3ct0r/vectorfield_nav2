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

// Unit tests of the Tracker's obstacle handling: rollout reuse, cap, kBlocked hysteresis, the
// settled precedence, a cancel collision and a contour smoke run (math_spec §6, §10).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/simulator.hpp"
#include "vectorfield_core/tracker.hpp"

namespace vectorfield_core
{
namespace
{

constexpr double kRes = 0.05;

class StraightCurve
{
public:
  explicit StraightCurve(double length)
  {
    const std::vector<Vec2> points = {Vec2(0.0, 0.0), Vec2(length, 0.0)};
    EXPECT_EQ(curve_.assign(Span<const Vec2>(points), false), AssignStatus::kOk);
  }
  const Curve * get() const { return &curve_; }

private:
  PolylineCurve curve_;
};

/// 80 x 80 cells of 0.05 m covering [-2, 2]^2.
class GridWorld
{
public:
  GridWorld() : mask_(80 * 80, 0) {}

  void site(double x, double y)
  {
    const int ix = static_cast<int>(std::floor((x + 2.0) / kRes));
    const int iy = static_cast<int>(std::floor((y + 2.0) / kRes));
    mask_[static_cast<std::size_t>(iy * 80 + ix)] = 1;
  }
  void wallAtX(double x)
  {
    for (int iy = 0; iy < 80; ++iy) {
      site(x, -2.0 + (iy + 0.5) * kRes);
    }
  }
  void disc(const Vec2 & centre, double radius)
  {
    for (int iy = 0; iy < 80; ++iy) {
      for (int ix = 0; ix < 80; ++ix) {
        const Vec2 c(-2.0 + (ix + 0.5) * kRes, -2.0 + (iy + 0.5) * kRes);
        if ((c - centre).norm() <= radius) {
          mask_[static_cast<std::size_t>(iy * 80 + ix)] = 1;
        }
      }
    }
  }
  ObstacleMap map()
  {
    EXPECT_TRUE(grid_.setFromMask(Span<const std::uint8_t>(mask_), 80, 80, kRes, Vec2(-2, -2)));
    edt_.update(grid_);
    return ObstacleMap{&grid_, &edt_, Isometry2()};
  }
  const OccupancyGrid & grid() const { return grid_; }
  const EuclideanDistanceTransform & edt() const { return edt_; }

private:
  std::vector<std::uint8_t> mask_;
  OccupancyGrid grid_;
  EuclideanDistanceTransform edt_;
};

GoalInfo goalAt(const Vec2 & position)
{
  GoalInfo goal;
  goal.pose = Pose2{position, 0.0};
  goal.xy_tol = 0.25;
  goal.yaw_tol = 0.25;
  return goal;
}

TrackerInput inputAt(const Curve * curve, const Pose2 & pose, const GoalInfo & goal)
{
  TrackerInput input;
  input.curve = curve;
  input.pose = pose;
  input.dt = 0.05;
  input.goal = goal;
  return input;
}

bool isZero(const Twist2 & u) { return u.vx == 0.0 && u.vy == 0.0 && u.wz == 0.0; }

TEST(TrackerSafety, ReusesTheRolloutsFirstStepWhenTheCapDoesNotBind)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve(3.0);
  GridWorld world;  // no sites
  const ObstacleMap map = world.map();
  TrackerInput with_map = inputAt(curve.get(), Pose2{Vec2(0.5, 0.2), 0.1}, goalAt(Vec2(3, 0)));
  with_map.measured = Twist2{0.1, 0.0, 0.0};
  with_map.obstacles = map;
  TrackerInput without_map = with_map;
  without_map.obstacles.reset();
  TrackerState a = Tracker::initialState();
  TrackerState b = a;
  StepScratch scratch_a;
  StepScratch scratch_b;
  for (int k = 0; k < 3; ++k) {
    const TrackerOutput out_a = tracker.step(with_map, a, &a, scratch_a);
    const TrackerOutput out_b = tracker.step(without_map, b, &b, scratch_b);
    EXPECT_EQ(out_a.cmd.vx, out_b.cmd.vx);
    EXPECT_EQ(out_a.cmd.wz, out_b.cmd.wz);
    EXPECT_EQ(out_a.s_free, kInf);
    EXPECT_EQ(out_a.cap, kInf);
    EXPECT_FALSE(out_a.predicted.empty());
    EXPECT_EQ(out_a.first_collision, -1);
    EXPECT_TRUE(out_b.predicted.empty());
  }
}

TEST(TrackerSafety, KBlockedStopsShortOfAWallAndReleasesWithHysteresis)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve(3.0);
  GridWorld world;
  world.wallAtX(1.01);  // first column [1.0, 1.05]: a 0.22 m disc touches it at x = 0.78
  const ObstacleMap map = world.map();
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  auto stepAt = [&](double x, TrackerState * st) {
    TrackerInput input = inputAt(curve.get(), Pose2{Vec2(x, 0.0), 0.0}, goalAt(Vec2(3, 0)));
    input.obstacles = map;
    return tracker.step(input, *st, st, scratch);
  };

  TrackerOutput out = stepAt(0.70, &state);  // 0.08 m of free arc < m_s
  EXPECT_NE(out.status & Status::kBlocked, 0U);
  EXPECT_TRUE(state.blocked);
  EXPECT_TRUE(isZero(out.cmd_target));
  EXPECT_LE(out.s_free, 0.08 + 1e-9);
  EXPECT_GE(out.s_free, 0.08 - kRes - 1e-9);
  EXPECT_EQ(out.cap, 0.0);
  EXPECT_FALSE(out.settled);
  ASSERT_GE(out.first_collision, 0);
  EXPECT_LT(static_cast<std::size_t>(out.first_collision), out.predicted.size());

  out = stepAt(0.66, &state);  // about 0.12 m: above the set threshold, below m_s + h_b = 0.2
  EXPECT_NE(out.status & Status::kBlocked, 0U);
  out = stepAt(0.50, &state);  // about 0.28 m: released
  EXPECT_EQ(out.status & Status::kBlocked, 0U);
  EXPECT_FALSE(state.blocked);

  TrackerState fresh = Tracker::initialState();
  out = stepAt(0.66, &fresh);  // without the latch, 0.12 m does not block
  EXPECT_EQ(out.status & Status::kBlocked, 0U);
  EXPECT_NE(out.status & Status::kCollisionAhead, 0U);  // but the cap binds
  EXPECT_LT(out.cap, 0.4);
}

TEST(TrackerSafety, ACapThatStopsTheRobotInsideXyTolSettlesInsteadOfBlocking)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve(0.9);
  GridWorld world;
  // First column [0.95, 1.0]: contact at x = 0.73, before sigma_stop is reached at x = 0.775
  // (a wall beyond that would let the rollout settle first, and the cap would never bind).
  world.wallAtX(0.96);
  const ObstacleMap map = world.map();
  GoalInfo goal = goalAt(Vec2(0.9, 0.0));
  goal.xy_reached = true;  // 0.2 m from the goal: inside xy_tol, outside sigma_stop
  TrackerInput input = inputAt(curve.get(), Pose2{Vec2(0.70, 0.0), 0.0}, goal);
  input.obstacles = map;
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  const TrackerOutput out = tracker.step(input, state, &state, scratch);
  EXPECT_TRUE(out.settled);
  EXPECT_EQ(out.status & Status::kBlocked, 0U);
  EXPECT_NE(out.status & Status::kCollisionAhead, 0U);
  EXPECT_TRUE(state.holding);
  EXPECT_TRUE(isZero(out.cmd_target));
}

TEST(TrackerSafety, RotateToHeadingChecksTheSweptRotation)
{
  // A long nose (0.6 m x 0.2 m) settled at the goal; turning left to the goal yaw sweeps the nose
  // into a wall at y = 0.3 (math_spec §10.4: the rotation rule of kBlocked).
  CoreConfig config;
  config.node.footprint.polygon = {
    Vec2(-0.1, -0.1), Vec2(0.5, -0.1), Vec2(0.5, 0.1), Vec2(-0.1, 0.1)};
  const Tracker tracker{config};
  const StraightCurve curve(1.0);
  GoalInfo goal = goalAt(Vec2(1.0, 0.0));
  goal.pose.yaw = kHalfPi;
  goal.has_yaw = true;
  goal.xy_reached = true;
  TrackerInput input = inputAt(curve.get(), Pose2{Vec2(0.98, 0.0), 0.0}, goal);
  TrackerState state = Tracker::initialState();
  StepScratch scratch;

  GridWorld open;  // no wall: the rotation proceeds
  input.obstacles = open.map();
  TrackerOutput out = tracker.step(input, state, &state, scratch);
  EXPECT_EQ(out.phase, Phase::kRotateToHeading);
  EXPECT_EQ(out.status & Status::kBlocked, 0U);
  EXPECT_GT(out.cmd_target.wz, 0.0);
  EXPECT_GT(out.predicted.size(), 1U);  // the rotation was swept, not cut at settled

  GridWorld walled;
  for (double x = 0.6; x < 1.8; x += kRes) {
    walled.site(x, 0.31);
  }
  input.obstacles = walled.map();
  state = Tracker::initialState();
  out = tracker.step(input, state, &state, scratch);
  EXPECT_EQ(out.phase, Phase::kRotateToHeading);
  EXPECT_NE(out.status & Status::kBlocked, 0U);
  EXPECT_TRUE(isZero(out.cmd_target));
  EXPECT_EQ(out.s_free, 0.0);  // a rotation-only collision
}

TEST(TrackerSafety, RotateToFieldDecisionIgnoresTheCollisionCap)
{
  // A 0.6 m x 0.3 m robot 0.05 m beside a wall with the path behind it: the rollout enters
  // ROTATE_TO_FIELD, the swept rotation collides at once (s_free = 0, cap 0). The capped re-run
  // must decide the same phase although its shaped field is zero.
  CoreConfig config;
  config.node.footprint.polygon = {
    Vec2(0.3, 0.15), Vec2(-0.3, 0.15), Vec2(-0.3, -0.15), Vec2(0.3, -0.15)};
  const Tracker tracker{config};
  const std::vector<Vec2> points = {Vec2(0.0, 0.0), Vec2(-3.0, 0.0)};
  PolylineCurve curve;
  ASSERT_EQ(curve.assign(Span<const Vec2>(points), false), AssignStatus::kOk);
  GridWorld world;
  for (double x = -1.0 + 0.5 * kRes; x < 1.0; x += kRes) {
    for (double y = 0.2 + 0.5 * kRes; y < 0.5; y += kRes) {
      world.site(x, y);
    }
  }
  TrackerInput input = inputAt(&curve, Pose2{Vec2(0.0, 0.0), 0.0}, goalAt(Vec2(-3.0, 0.0)));
  input.obstacles = world.map();
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  const TrackerOutput out = tracker.step(input, state, &state, scratch);
  EXPECT_EQ(out.phase, Phase::kRotateToField);
  EXPECT_EQ(state.phase, Phase::kRotateToField);
  EXPECT_EQ(out.s_free, 0.0);
  EXPECT_NE(out.status & Status::kBlocked, 0U);
  EXPECT_TRUE(isZero(out.cmd_target));
}

TEST(TrackerSafety, WithoutCollisionDetectionOnlyTheCurrentPoseBlocks)
{
  CoreConfig config;
  config.safety.use_collision_detection = false;
  const Tracker tracker{config};
  const StraightCurve curve(3.0);
  GridWorld ahead;
  ahead.wallAtX(1.01);
  TrackerInput input = inputAt(curve.get(), Pose2{Vec2(0.70, 0.0), 0.0}, goalAt(Vec2(3, 0)));
  input.obstacles = ahead.map();
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  TrackerOutput out = tracker.step(input, state, &state, scratch);
  EXPECT_EQ(out.status & Status::kBlocked, 0U);
  EXPECT_EQ(out.s_free, kInf);
  EXPECT_TRUE(out.predicted.empty());

  GridWorld under;
  under.site(0.70, 0.0);
  input.obstacles = under.map();
  out = tracker.step(input, state, &state, scratch);
  EXPECT_NE(out.status & Status::kBlocked, 0U);
  EXPECT_TRUE(isZero(out.cmd_target));
}

TEST(TrackerSafety, ACancelCollisionRaisesTheDecelerationAndIsReported)
{
  CoreConfig config;
  config.phases.cancel_deceleration = 0.3;  // gentle stop: about 0.26 m from 0.4 m/s
  const Tracker tracker{config};
  const StraightCurve curve(3.0);
  GridWorld world;
  world.wallAtX(0.81);  // a 0.22 m disc at x = 0.4 touches it after 0.18 m
  const ObstacleMap map = world.map();
  TrackerInput input = inputAt(curve.get(), Pose2{Vec2(0.4, 0.0), 0.0}, goalAt(Vec2(3, 0)));
  input.obstacles = map;
  input.cancel = true;
  input.measured = Twist2{0.4, 0.0, 0.0};
  TrackerState state = Tracker::initialState();
  state.limiter.u_prev = input.measured;
  state.limiter.valid = true;
  StepScratch scratch;
  TrackerOutput out = tracker.step(input, state, &state, scratch);
  EXPECT_EQ(out.phase, Phase::kStopped);
  EXPECT_NE(out.status & Status::kCollisionAhead, 0U);
  EXPECT_EQ(out.status & Status::kBlocked, 0U);
  EXPECT_TRUE(state.limiter.cancel_hard);
  EXPECT_NEAR(out.cmd.vx, 0.4 - 1.0 * 0.05, 1e-12);  // max(a_can, a_dec) already this cycle

  // Simulated to the end: the robot stops free of the wall and the cancel completes.
  sim::SimulationOptions options;
  options.stop_when_stopped = true;
  sim::ClosedLoopSimulator simulator(tracker, options);
  simulator.reset(Pose2{Vec2(0.4, 0.0), 0.0}, 0.0, Twist2{0.4, 0.0, 0.0});
  TrackerInput template_input = input;
  simulator.run(template_input, 3.0);
  const FootprintChecker checker(Footprint::circle(0.22));
  bool completed = false;
  for (const sim::SimulationRecord & record : simulator.records()) {
    EXPECT_FALSE(checker.collides(record.pose, world.grid(), world.edt()));
    completed = completed || (std::abs(record.cmd.vx) < kStopLinearSpeed);
  }
  EXPECT_TRUE(completed);
  EXPECT_FALSE(checker.collides(simulator.pose(), world.grid(), world.edt()));
}

TEST(TrackerSafety, ContourEpisodeStartsWithALatchedSideAroundTheBlockingDisc)
{
  CoreConfig config;
  config.obstacles.obstacle_mode = ObstacleMode::kContour;
  const Tracker tracker{config};
  const StraightCurve curve(3.5);
  GridWorld world;
  world.disc(Vec2(1.0, 0.0), 0.15);
  const ObstacleMap map = world.map();
  TrackerInput input = inputAt(curve.get(), Pose2{Vec2(-1.2, 0.0), 0.0}, goalAt(Vec2(3.5, 0)));
  input.obstacles = map;

  sim::ClosedLoopSimulator simulator(tracker);
  simulator.reset(Pose2{Vec2(-1.2, 0.0), 0.0});
  const FootprintChecker checker(Footprint::circle(0.22));
  bool started = false;
  bool ended_after_start = false;
  int episodes = 0;
  bool was_active = false;
  std::int8_t side = 0;
  for (int k = 0; k < 400 && simulator.pose().p.x() < 1.6; ++k) {
    const sim::SimulationRecord & record = simulator.step(input);
    EXPECT_FALSE(checker.collides(record.pose, world.grid(), world.edt())) << "t " << record.t;
    EXPECT_EQ(record.status & Status::kBlocked, 0U) << "t " << record.t;
    const ContourState & contour = simulator.state().contour;
    if (contour.active) {
      if (!was_active) {
        ++episodes;
        side = contour.sigma_c;
        EXPECT_TRUE(side == 1 || side == -1);
        EXPECT_TRUE(simulator.scratch().k_valid);
      }
      EXPECT_EQ(contour.sigma_c, side);  // latched for the whole episode
      started = true;
    } else if (was_active) {
      ended_after_start = true;
    }
    was_active = contour.active;
  }
  EXPECT_TRUE(started);
  EXPECT_EQ(episodes, 1);
  EXPECT_GT(simulator.pose().p.x(), 1.6);  // passed the disc
  (void)ended_after_start;
}

}  // namespace
}  // namespace vectorfield_core
