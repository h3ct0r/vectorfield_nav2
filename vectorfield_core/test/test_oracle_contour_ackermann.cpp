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

// Contour oracle O57, Ackermann (math_spec §6.6 [Design, M7], §12.5).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

constexpr int kObstacleTag = 1;

/// A car: turning radius and the rear axle's body-x from base_link (`reference_point_offset`).
struct Car
{
  double r_min;
  double x_r;
};

/// The cars of O57: base_link mid-body, the rear axle 0.2 m behind it.
constexpr Car kCars[] = {{0.5, -0.2}, {1.0, -0.2}};

std::string label(const Car & car)
{
  return "R_min " + std::to_string(car.r_min) + ", x_r " + std::to_string(car.x_r);
}

/// A wheelbase-like 0.7 x 0.4 m body: the rear bumper 0.15 m behind the rear axle (at x_r), the
/// front bumper 0.55 m ahead of it (w_lat 0.2).
vf::FootprintSpec carFootprint(double x_r)
{
  vf::FootprintSpec spec;
  spec.polygon = {
    Vec2(x_r - 0.15, -0.2), Vec2(x_r + 0.55, -0.2), Vec2(x_r + 0.55, 0.2), Vec2(x_r - 0.15, 0.2)};
  return spec;
}

/// TC as Ackermann with contour (auto lambda, D_in, D'_in, lookahead). The watchdog length covers
/// the R_min = 1 circulation (math_spec §6.6 watchdog warning).
vf::CoreConfig carConfig(const Car & car)
{
  vf::CoreConfig c = tcConfig(vf::MotionModel::kAckermann, vf::TrackingLaw::kHeadingTracking);
  c.motion.min_turning_radius = car.r_min;
  c.motion.reference_point_offset = car.x_r;
  c.node.footprint = carFootprint(car.x_r);
  c.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  c.obstacles.obstacle_max_contour_length = 8.0;
  return c;
}

/// Metrics of a run.
struct RunStats
{
  std::vector<Cycle> cycles;
  std::vector<Episode> eps;
  /// Smallest distance from the evaluation point (rear axle) to the obstacle's sites.
  double min_clearance{vf::kInf};
  /// Base lateral position when it passes the obstacle's x (NaN if never).
  double y_at_obstacle{vf::kNaN};
  int saturated{0};
  int infeasible{0};
  int collisions{0};
  bool settled{false};
};

RunStats measure(
  const std::vector<Cycle> & cycles, const ObstacleWorld & world, const vf::CoreConfig & config,
  double x_obstacle)
{
  RunStats r;
  r.cycles = cycles;
  r.eps = episodes(cycles);
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(config.node.footprint));
  for (const Cycle & c : cycles) {
    r.min_clearance =
      std::min(r.min_clearance, world.distanceToSites(c.out.field.x_eval, kObstacleTag));
    r.saturated += Status::has(c.out.status, Status::kCurvatureSaturated) ? 1 : 0;
    r.infeasible += Status::has(c.out.status, Status::kHeadingInfeasible) ? 1 : 0;
    r.collisions += checker.collidesGroundTruth(c.pose_after, world.grid()) ? 1 : 0;
    if (
      std::isnan(r.y_at_obstacle) && c.rec.pose.p.x() <= x_obstacle &&
      c.pose_after.p.x() > x_obstacle) {
      r.y_at_obstacle = 0.5 * (c.rec.pose.p.y() + c.pose_after.p.y());
    }
    r.settled = r.settled || c.out.settled;
  }
  return r;
}

/// O57 disc: a disc of radius R_o at (0, y_o) on the path y = 0; the car starts D'_in + R_o + 1
/// before it heading +x (before the gate can open); the goal is 4 m + 2 lambda past it.
void expectDiscPassed(const Car & car, double R_o, double y_o)
{
  SCOPED_TRACE(label(car) + ", R_o " + std::to_string(R_o) + ", y_o " + std::to_string(y_o));
  const vf::CoreConfig config = carConfig(car);
  const vf::CoreConfig resolved = vf::resolveAutoValues(config);
  const double lambda = resolved.obstacles.obstacle_contour_distance;
  const double x_start = -(resolved.obstacles.obstacle_blend_start_distance + R_o + 1.0);
  const double x_goal = 4.0 + 2.0 * lambda;
  const double half = R_o + lambda + 1.0;
  ObstacleWorld world = centredWorld(x_start - 1.0, x_goal + 1.0, -half, half);
  world.addDisc(Vec2(0.0, y_o), R_o, kObstacleTag);
  world.commit();
  const StraightRun run = runStraight(config, world, Pose2{Vec2(x_start, 0.0), 0.0}, x_goal, 60.0);
  const RunStats r = measure(run.cycles, world, config, 0.0);
  std::printf(
    "[O57 disc %s, R_o %.2f, y_o %.2f] lambda %.3f: min clearance %.3f, y at disc %.3f, "
    "%d saturated cycles, end %.1f s\n",
    label(car).c_str(), R_o, y_o, lambda, r.min_clearance, r.y_at_obstacle, r.saturated,
    r.cycles.back().rec.t);
  ASSERT_GE(r.eps.size(), 1U) << "the contour never engaged";
  EXPECT_EQ(r.eps.size(), 1U);
  EXPECT_TRUE(r.eps[0].sigma_constant);
  EXPECT_GE(r.min_clearance, lambda - kRes);
  EXPECT_EQ(r.collisions, 0);
  EXPECT_EQ(r.infeasible, 0) << "kHeadingInfeasible";
  ASSERT_FALSE(std::isnan(r.y_at_obstacle)) << "never passed the disc";
  // Beside the disc, on the free side when it is offset (the larger gap is at y < 0).
  EXPECT_GT(std::abs(r.y_at_obstacle - y_o), R_o + lambda - kRes - std::abs(car.x_r));
  if (y_o > 0.0) {
    EXPECT_LT(r.y_at_obstacle, 0.0);
  }
  // Rejoined: the goal is settled on the path.
  EXPECT_TRUE(r.settled) << "never settled";
  EXPECT_LT(std::abs(r.cycles.back().out.field.D), 0.05);
}

/// Runs `config` along y = 0 from (x_start, 0) toward the goal (x_goal, 0) until kBlocked has
/// been set for 3 s, or 40 s.
std::vector<Cycle> runIntoBlockage(
  const vf::CoreConfig & config, const ObstacleWorld & world, double x_start, double x_goal)
{
  const auto curve = makePolyline({Vec2(x_start - 1.0, 0.0), Vec2(x_goal, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.stop_when_stopped = false;
  Runner runner(config, options);
  runner.reset(Pose2{Vec2(x_start, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(x_goal, 0.0));
  runner.input().obstacles = world.map();
  double blocked_at = -1.0;
  runner.run(40.0, [&blocked_at](const Cycle & c) {
    if (blocked_at < 0.0 && Status::has(c.out.status, Status::kBlocked)) {
      blocked_at = c.rec.t;
    }
    return blocked_at >= 0.0 && c.rec.t > blocked_at + 3.0;
  });
  return runner.cycles();
}

/// No episode, never a collision, kBlocked set and kept, stopped at least m_s - res and less
/// than 3 m_s short of the blockage (ground-truth translation to contact along +x).
void expectStopsShort(const Car & car, const ObstacleWorld & world, double x_start, double x_goal)
{
  const vf::CoreConfig config = carConfig(car);
  const std::vector<Cycle> cs = runIntoBlockage(config, world, x_start, x_goal);
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(config.node.footprint));
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
  std::printf(
    "[O57 stop %s] stopped %.3f m short at t %.1f\n", label(car).c_str(), gap, cs.back().rec.t);
  EXPECT_GE(gap, kStopMargin - kRes);
  EXPECT_LT(gap, 3.0 * kStopMargin) << "stopped early";
}

/// A 0.6 m box across the path from a wall at y = -1.2 up to y = 0.3 (that side is closed), and a
/// wall parallel to the path at `gap` above it; both walls from x = x0 to 7.
ObstacleWorld boxWithGap(double gap, double x0)
{
  ObstacleWorld world = centredWorld(x0 - 1.0, 8.0, -2.0, 2.0 + gap);
  world.addBox(Vec2(2.7, -1.2), Vec2(3.3, 0.3), kObstacleTag);
  world.addBox(Vec2(x0, -1.3), Vec2(7.0, -1.2));
  world.addBox(Vec2(x0, 0.3 + gap), Vec2(7.0, 0.4 + gap));
  world.commit();
  return world;
}

/// The gap of O57 for a car: 0.15 m narrower than the car's lambda + w_lat = R_min + 0.45, and
/// wider than TC's lambda + w_lat = 0.69 m.
double narrowGap(const Car & car) { return car.r_min + 0.3; }

}  // namespace

// O57 Disc (math_spec §6.6): discs of radius 0.15 and 0.3 m on a straight path, centred and 0.1 m
// off it, for R_min 0.5 and 1 m: one episode with a constant side, the rear axle keeps
// lambda - res from the disc, no collision, never kHeadingInfeasible, the car passes beside the
// disc (on the free side when it is offset) and settles at the goal on the path.
TEST(OracleContourAckermann, O57_DiscRMin050)
{
  for (const double R_o : {0.15, 0.3}) {
    for (const double y_o : {0.0, 0.1}) {
      expectDiscPassed(kCars[0], R_o, y_o);
    }
  }
}

TEST(OracleContourAckermann, O57_DiscRMin100Centred)
{
  for (const double R_o : {0.15, 0.3}) {
    expectDiscPassed(kCars[1], R_o, 0.0);
  }
}

TEST(OracleContourAckermann, O57_DiscRMin100Offset)
{
  for (const double R_o : {0.15, 0.3}) {
    expectDiscPassed(kCars[1], R_o, 0.1);
  }
}

// O57 Gap (math_spec §6.6, §6.4 "Side probe"): a box across the path, closed on one side by a
// wall, with a gap on the other side narrower than the car's lambda + w_lat (R_min + 0.3). Both
// side probes, which roll the car's kinematics, fail: no episode, the cap stops the car short of
// the box and kBlocked follows.
TEST(OracleContourAckermann, O57_GapTooNarrowForTheTurningRadiusStops)
{
  for (const Car & car : kCars) {
    SCOPED_TRACE(label(car));
    const double x_start =
      2.7 - vf::resolveAutoValues(carConfig(car)).obstacles.obstacle_blend_start_distance - 1.0;
    expectStopsShort(car, boxWithGap(narrowGap(car), x_start - 1.0), x_start, 6.5);
  }
}

// TC (differential, FL) contours through the gap that stops the R_min = 0.5 car.
TEST(OracleContourAckermann, O57_GapPassableForTc)
{
  ObstacleWorld world = boxWithGap(narrowGap(kCars[0]), -1.0);
  const StraightRun run =
    runStraight(tcContourConfig(), world, Pose2{Vec2(0.5, 0.0), 0.0}, 6.5, 40.0);
  const RunStats r = measure(run.cycles, world, tcContourConfig(), 3.0);
  EXPECT_GE(r.eps.size(), 1U);
  EXPECT_EQ(r.collisions, 0);
  EXPECT_TRUE(r.settled);
}

// O57 Dead end (math_spec §6.6, O56): the 1.2 m corridor closed across the path stops the car.
TEST(OracleContourAckermann, O57_DeadEndCorridorStops)
{
  for (const Car & car : kCars) {
    SCOPED_TRACE(label(car));
    const double x_start =
      3.0 - vf::resolveAutoValues(carConfig(car)).obstacles.obstacle_blend_start_distance - 1.0;
    ObstacleWorld world = centredWorld(x_start - 1.5, 6.5, -2.0, 2.0);
    world.addBox(Vec2(x_start - 1.0, 0.6), Vec2(3.1, 0.7));
    world.addBox(Vec2(x_start - 1.0, -0.7), Vec2(3.1, -0.6));
    world.addBox(Vec2(3.0, -0.7), Vec2(3.1, 0.7), kObstacleTag);
    world.commit();
    expectStopsShort(car, world, x_start, 6.0);
  }
}

// O57 Late start (math_spec §6.6 "Turn-in"): the system test's case, a 0.4 m box whose near face
// is 0.8 m ahead of the car at rest, well inside D_in. The car turns at R_min at once: no
// kHeadingInfeasible, no collision, it passes and settles at the goal 4 m ahead.
TEST(OracleContourAckermann, O57_BoxInsideTheTurnInBand)
{
  for (const double offset : {0.0, 0.1}) {
    SCOPED_TRACE("offset " + std::to_string(offset));
    vf::CoreConfig config = carConfig(Car{0.5, 0.0});
    config.node.footprint = vf::FootprintSpec{};  // TB3: circle of 0.22 m
    ObstacleWorld world = centredWorld(-1.5, 5.5, -2.5, 2.5);
    world.addBox(Vec2(0.8, offset - 0.2), Vec2(1.2, offset + 0.2), kObstacleTag);
    world.commit();
    const StraightRun run = runStraight(config, world, Pose2{Vec2(0.0, 0.0), 0.0}, 4.0, 60.0);
    const RunStats r = measure(run.cycles, world, config, 1.0);
    std::printf(
      "[O57 late start, offset %.2f] min clearance %.3f, y at box %.3f, %d saturated, t %.1f\n",
      offset, r.min_clearance, r.y_at_obstacle, r.saturated, r.cycles.back().rec.t);
    ASSERT_GE(r.eps.size(), 1U);
    EXPECT_EQ(r.collisions, 0);
    EXPECT_EQ(r.infeasible, 0);
    EXPECT_TRUE(r.settled);
  }
}

// O57 Validation (math_spec §6.6): lambda < R_min + 0.05 is a ConfigError.
TEST(OracleContourAckermann, O57_LambdaBelowTurningRadiusIsAConfigError)
{
  vf::CoreConfig config = carConfig(kCars[0]);
  config.obstacles.obstacle_contour_distance = 0.5;
  EXPECT_THROW(vf::Tracker{config}, vf::ConfigError);
  config.obstacles.obstacle_contour_distance = 0.6;
  EXPECT_NO_THROW(vf::Tracker{config});
}
