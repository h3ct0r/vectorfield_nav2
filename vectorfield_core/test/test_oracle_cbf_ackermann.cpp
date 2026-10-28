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

// CBF oracle O59, Ackermann (math_spec §6.7 [Design, 0.3.0], §12.5): closed loops of a
// 0.7 × 0.4 m car along y = 0 with the filter at the rear axle and the turning margin.

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

constexpr double kMargin = 0.05;

/// A car: turning radius and the rear axle's body-x from base_link (`reference_point_offset`).
struct Car
{
  double r_min;
  double x_r;
};

/// The cars of O59 (those of O57): base_link mid-body, the rear axle 0.2 m behind it.
constexpr Car kCars[] = {{0.5, -0.2}, {1.0, -0.2}};

/// The 0.7 × 0.4 m body of O57: the rear bumper 0.15 m behind the rear axle, the front bumper
/// 0.55 m ahead of it.
vf::FootprintSpec carFootprint(double x_r)
{
  vf::FootprintSpec spec;
  spec.polygon = {
    Vec2(x_r - 0.15, -0.2), Vec2(x_r + 0.55, -0.2), Vec2(x_r + 0.55, 0.2), Vec2(x_r - 0.15, 0.2)};
  return spec;
}

/// δ_safe at the rear axle: the front corners are the farthest, hypot(0.55, 0.2) + m.
constexpr double kDeltaCar = 0.5852349955359814 + kMargin;

vf::CoreConfig carConfig(const Car & car)
{
  vf::CoreConfig c = tcConfig(vf::MotionModel::kAckermann, vf::TrackingLaw::kHeadingTracking);
  c.motion.min_turning_radius = car.r_min;
  c.motion.reference_point_offset = car.x_r;
  c.node.footprint = carFootprint(car.x_r);
  c.obstacles.obstacle_mode = vf::ObstacleMode::kCbf;
  c.obstacles.cbf_margin = kMargin;
  return c;
}

std::string label(const Car & car)
{
  return "R_min " + std::to_string(car.r_min) + ", x_r " + std::to_string(car.x_r);
}

/// The body polygon's boundary sampled every 5 cm (body frame; the distance to a site centre
/// 0.3 m away is then within 1 mm of the edge's).
std::vector<Vec2> boundarySamples(const vf::FootprintSpec & spec)
{
  std::vector<Vec2> out;
  const std::vector<Vec2> & poly = spec.polygon;
  for (std::size_t k = 0; k < poly.size(); ++k) {
    const Vec2 & a = poly[k];
    const Vec2 & b = poly[(k + 1) % poly.size()];
    const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.05)));
    for (int i = 0; i < n; ++i) {
      out.emplace_back(a + (static_cast<double>(i) / n) * (b - a));
    }
  }
  return out;
}

/// Brute-force clearance of the footprint at `pose`: the smallest distance from its boundary to
/// a site centre (the cell squares are res/√2 closer at most).
double footprintClearance(
  const ObstacleWorld & world, const std::vector<Vec2> & boundary, const Pose2 & pose)
{
  double best = vf::kInf;
  for (const Vec2 & b : boundary) {
    best = std::min(best, world.distanceToSites(pose.p + vf::rotate(pose.yaw, b)));
  }
  return best;
}

/// Metrics of one run.
struct CarRun
{
  /// Brute-force barrier at the rear axle, min over the cycles with the filter on.
  double min_h{vf::kInf};
  /// The δ_safe the Tracker used.
  double delta{0.0};
  /// Smallest footprint-to-site-centre distance (boundaryClearance()).
  double min_clearance{vf::kInf};
  double max_abs_D{0.0};
  double final_abs_D{vf::kInf};
  bool settled{false};
  int blocked_cycles{0};
  int hold_cycles{0};
  int active_cycles{0};
  int collisions{0};
  int infeasible{0};
  /// Commands with |ω| > |v|/R_min.
  int curvature_violations{0};
  /// Smallest forward speed with the rear axle in [x_lo, x_hi].
  double min_speed{vf::kInf};
  double final_speed{vf::kInf};
  Pose2 final_pose;
  std::size_t cycles{0};
};

/// Runs runStraight() and measures it; stops once kBlocked has lasted `blocked_stop` cycles.
CarRun runCar(
  const vf::CoreConfig & config, ObstacleWorld & world, const Pose2 & start, double x_goal,
  double timeout, int blocked_stop = 20, double x_lo = -vf::kInf, double x_hi = vf::kInf)
{
  int blocked = 0;
  const StraightRun r = runStraight(
    config, world, start, x_goal, timeout, {}, [&blocked, blocked_stop](const Cycle & c) {
      blocked = vf::Status::has(c.out.status, vf::Status::kBlocked) ? blocked + 1 : 0;
      return blocked >= blocked_stop;
    });
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(config.node.footprint));
  const std::vector<Vec2> boundary = boundarySamples(config.node.footprint);
  const double r_min = config.motion.min_turning_radius;
  CarRun m;
  m.cycles = r.cycles.size();
  for (const Cycle & c : r.cycles) {
    const vf::CbfSample & cbf = c.out.field.cbf;
    if (cbf.on) {
      if (m.delta == 0.0) {
        m.delta = cbf.delta;
      }
      m.min_h = std::min(m.min_h, world.distanceToSites(c.out.field.x_eval) - cbf.delta);
    }
    // The disc of radius δ_safe − m about the rear axle contains the footprint: brute force only
    // where that bound leaves less than 0.6 m.
    const Vec2 rear =
      c.pose_after.p + config.motion.reference_point_offset * vf::heading(c.pose_after.yaw);
    const double bound = world.distanceToSites(rear) - (kDeltaCar - kMargin);
    m.min_clearance = std::min(
      m.min_clearance, bound < 0.6 ? footprintClearance(world, boundary, c.pose_after) : bound);
    m.max_abs_D = std::max(m.max_abs_D, std::abs(c.out.field.D));
    m.settled = m.settled || c.out.settled;
    m.blocked_cycles += vf::Status::has(c.out.status, vf::Status::kBlocked) ? 1 : 0;
    m.hold_cycles += cbf.hold ? 1 : 0;
    m.active_cycles += cbf.active ? 1 : 0;
    m.collisions += checker.collidesGroundTruth(c.pose_after, world.grid()) ? 1 : 0;
    m.infeasible += vf::Status::has(c.out.status, vf::Status::kHeadingInfeasible) ? 1 : 0;
    m.curvature_violations +=
      std::abs(c.out.cmd.wz) > std::abs(c.out.cmd.vx) / r_min + 1e-9 ? 1 : 0;
    const double x = c.out.field.x_eval.x();
    if (x >= x_lo && x <= x_hi) {
      m.min_speed = std::min(m.min_speed, c.rec.measured.vx);
    }
  }
  if (!r.cycles.empty()) {
    const Cycle & last = r.cycles.back();
    m.final_abs_D = std::abs(last.out.field.D);
    m.final_speed = std::abs(last.out.cmd.vx);
    m.final_pose = last.pose_after;
  }
  return m;
}

std::string describe(const CarRun & m)
{
  char buffer[512];
  std::snprintf(
    buffer, sizeof(buffer),
    "min h %.4f, delta %.3f, clearance %.3f, max |D| %.3f, final |D| %.3f, settled %d, "
    "blocked %d, hold %d, active %d, collisions %d, infeasible %d, curvature %d, min speed %.3f, "
    "final speed %.3f, final x %.3f, cycles %zu",
    m.min_h, m.delta, m.min_clearance, m.max_abs_D, m.final_abs_D, m.settled ? 1 : 0,
    m.blocked_cycles, m.hold_cycles, m.active_cycles, m.collisions, m.infeasible,
    m.curvature_violations, m.min_speed, m.final_speed, m.final_pose.p.x(), m.cycles);
  return buffer;
}

/// Safety of every run: no collision, never kHeadingInfeasible, |ω| <= |v|/R_min.
void expectSafe(const CarRun & m)
{
  EXPECT_EQ(m.collisions, 0) << describe(m);
  EXPECT_EQ(m.infeasible, 0) << "kHeadingInfeasible: " << describe(m);
  EXPECT_EQ(m.curvature_violations, 0) << describe(m);
  EXPECT_NEAR(m.delta, kDeltaCar, 1e-9) << describe(m);
}

}  // namespace

// O59 (a) Offset disc: a 0.15 m disc 0.2, 0.35 and 0.5 m off the path (the middle one on the other
// side). The turning margin starts the turn about R_min earlier than γ_b h alone would, so both
// cars pass: settled at the goal, never kBlocked, the filter binds, min h >= −res at the rear
// axle (so the footprint stays m − res from the site centres), and the car rejoins the path.
TEST(OracleCbfAckermann, O59a_OffsetDiscPasses)
{
  for (const Car & car : kCars) {
    for (const double offset : {0.2, -0.35, 0.5}) {
      SCOPED_TRACE(label(car) + ", offset " + std::to_string(offset));
      ObstacleWorld world = centredWorld(-2.5, 9.0, -3.0, 3.0);
      world.addDisc(Vec2(3.0, offset), 0.15);
      world.commit();
      const CarRun m = runCar(carConfig(car), world, Pose2{Vec2(-1.0, 0.0), 0.0}, 7.0, 60.0);
      std::printf("[O59a %s, offset %.2f] %s\n", label(car).c_str(), offset, describe(m).c_str());
      expectSafe(m);
      EXPECT_TRUE(m.settled) << describe(m);
      EXPECT_EQ(m.blocked_cycles, 0) << describe(m);
      EXPECT_GE(m.active_cycles, 1) << describe(m);
      EXPECT_GE(m.min_h, -kRes) << describe(m);
      EXPECT_GE(m.min_clearance, kMargin - kRes) << describe(m);
      EXPECT_LT(m.final_abs_D, 0.02) << describe(m);
    }
  }
}

// O59 (b) A wall parallel to the path with its site centres 0.60 m from it, 0.035 m inside δ_safe;
// the car starts 0.3 m off the path on the other side, so the path field turns it toward the
// wall. The filter binds and the car levels off at h ≈ 0, about 0.035 m short of the path, and
// follows the wall there without kBlocked at no less than 90 % of v_r, with min h >= −res.
TEST(OracleCbfAckermann, O59b_ParallelWallIsFollowed)
{
  for (const Car & car : kCars) {
    SCOPED_TRACE(label(car));
    const double y_wall = 0.6 - 0.5 * kRes;  // the first row of cell centres at 0.60
    ObstacleWorld world = centredWorld(-2.5, 9.0, -3.0, 3.0);
    world.addBox(Vec2(-1.0, y_wall), Vec2(8.0, y_wall + 0.2));
    world.commit();
    const CarRun m =
      runCar(carConfig(car), world, Pose2{Vec2(-1.0, -0.3), 0.0}, 7.0, 60.0, 20, 3.0, 5.5);
    std::printf("[O59b %s] %s\n", label(car).c_str(), describe(m).c_str());
    expectSafe(m);
    EXPECT_TRUE(m.settled) << describe(m);
    EXPECT_EQ(m.blocked_cycles, 0) << describe(m);
    EXPECT_GE(m.min_h, -kRes) << describe(m);
    EXPECT_GE(m.active_cycles, 1) << describe(m);
    EXPECT_GE(m.min_speed, 0.9 * kVr) << describe(m);
    EXPECT_LT(m.final_abs_D, 0.05) << describe(m);
  }
}

// O59 (c) A 0.4 × 1.2 m box across the path: F ∥ −n at the face, so F* → 0 where the turning
// margin is used up, i.e. R_min + δ_safe from the face. The hold raises kBlocked (kept for 100
// cycles), the car is at rest, never collides, h >= 0, and it stopped at least m_s − res and at
// most R_min + δ_safe short of the box (ground-truth translation to contact along +x).
TEST(OracleCbfAckermann, O59c_BoxAheadHoldsAndBlocks)
{
  for (const Car & car : kCars) {
    SCOPED_TRACE(label(car));
    const vf::CoreConfig config = carConfig(car);
    ObstacleWorld world = centredWorld(-2.5, 9.0, -3.0, 3.0);
    world.addBox(Vec2(3.0, -0.6), Vec2(3.4, 0.6));
    world.commit();
    const CarRun m = runCar(config, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 7.0, 30.0, 100);
    const vf::FootprintChecker checker(vf::Footprint::fromSpec(config.node.footprint));
    const double gap = travelToContact(checker, world, m.final_pose, Vec2(1.0, 0.0), 3.0);
    std::printf("[O59c %s] stop %.3f m short; %s\n", label(car).c_str(), gap, describe(m).c_str());
    expectSafe(m);
    EXPECT_FALSE(m.settled) << describe(m);
    EXPECT_GE(m.hold_cycles, 1) << describe(m);
    EXPECT_GE(m.blocked_cycles, 100) << "kBlocked must persist: " << describe(m);
    EXPECT_GE(m.min_h, 0.0) << describe(m);
    EXPECT_LT(m.final_speed, 1e-9) << describe(m);
    EXPECT_GE(gap, kStopMargin - kRes) << describe(m);
    EXPECT_LE(gap, car.r_min + kDeltaCar) << describe(m);
  }
}

// O59 (d) A gap the car's body fits through (0.39 m of free space on each side) but narrower than
// the barrier's 2 δ_safe: two 0.15 m discs with their site centres δ_safe − 0.05 off the path. The
// car holds before the gap with h >= 0 and kBlocked, never collides; Nav2 replans.
TEST(OracleCbfAckermann, O59d_GapTooTightHolds)
{
  for (const Car & car : kCars) {
    SCOPED_TRACE(label(car));
    const double y = 0.15 + kDeltaCar - 0.05;
    ObstacleWorld world = centredWorld(-2.5, 9.0, -3.0, 3.0);
    world.addDisc(Vec2(3.0, y), 0.15);
    world.addDisc(Vec2(3.0, -y), 0.15);
    world.commit();
    const CarRun m = runCar(carConfig(car), world, Pose2{Vec2(-1.0, 0.0), 0.0}, 7.0, 30.0, 40);
    std::printf("[O59d %s] %s\n", label(car).c_str(), describe(m).c_str());
    expectSafe(m);
    EXPECT_FALSE(m.settled) << describe(m);
    EXPECT_GE(m.hold_cycles, 1) << describe(m);
    EXPECT_GE(m.blocked_cycles, 40) << describe(m);
    EXPECT_GE(m.min_h, 0.0) << describe(m);
    EXPECT_LT(m.final_pose.p.x(), 3.0) << describe(m);
  }
}

// O59 (g) A car whose heading lags the field beside a box (a real car's steering and drive lag
// behind the commands): a 0.3 m box whose near face is 0.15 or 0.30 m from the path, the car on
// the path 0.85 or 1.25 m before it, heading 0.6–1.2 rad toward the box's side. The filtered
// field points up to about 115° from the heading; the car turns at R_min toward it at no less
// than half the field speed (kObstacleTurnSpeedFactor) instead of stopping with
// kHeadingInfeasible, passes the box without collision and settles at the goal. Six of these
// nine starts stopped with kHeadingInfeasible before the turn rule. Pointed at the box from
// 0.85 m at 0.9 rad or more the car only creeps under the collision cap (it turns at v/R_min) and
// may not clear the box in time; it never collides.
TEST(OracleCbfAckermann, O59g_LaggingHeadingTurnsBesideABox)
{
  struct Start
  {
    double near_face;
    double x0;
    double yaw0;
  };
  const Start starts[] = {
    {0.15, 1.6, 0.6}, {0.15, 1.6, 0.9}, {0.15, 1.6, 1.2}, {0.15, 2.0, 0.6}, {0.30, 1.6, 0.6},
    {0.30, 1.6, 0.9}, {0.30, 1.6, 1.2}, {0.30, 2.0, 0.6}, {0.30, 2.0, 0.9},
  };
  const Car & car = kCars[0];
  for (const Start & start : starts) {
    SCOPED_TRACE(
      "near face " + std::to_string(start.near_face) + ", x0 " + std::to_string(start.x0) +
      ", yaw0 " + std::to_string(start.yaw0));
    ObstacleWorld world(Vec2(-1.525, -2.525), 10.0, 5.0);
    world.addBox(Vec2(2.85, start.near_face), Vec2(3.15, start.near_face + 0.3));
    world.commit();
    const CarRun m =
      runCar(carConfig(car), world, Pose2{Vec2(start.x0, 0.0), start.yaw0}, 7.0, 40.0);
    expectSafe(m);
    EXPECT_TRUE(m.settled) << describe(m);
    EXPECT_GT(m.active_cycles, 0) << "the filter never bound: " << describe(m);
  }
}

// O59 (f) The turning margin itself: one step of the car (R_min = 1) at the origin, heading 0.5
// rad, toward a wall whose face is across +x at x = 2. The first site is the face's nearest point
// with n = −x (math_spec §6.1), so c = cos 0.5 and the margin is R_min (1 − sin 0.5); h_j stays
// D − δ_safe. Heading along the face (π/2) or away from it, and every other motion model: 0.
TEST(OracleCbfAckermann, O59f_TurningMarginOfOneStep)
{
  const auto step = [](const vf::CoreConfig & config, double yaw) {
    ObstacleWorld world = centredWorld(-2.5, 4.0, -3.0, 3.0);
    world.addBox(Vec2(2.0, -2.5), Vec2(2.5, 2.5));
    world.commit();
    const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(5.0, 0.0)}, false);
    Runner runner(config);
    runner.reset(Pose2{Vec2::Zero(), yaw});
    runner.input().curve = curve.get();
    runner.input().goal = goalAt(Vec2(5.0, 0.0));
    runner.input().obstacles = world.map();
    return runner.step().out.field.cbf;
  };
  const vf::CoreConfig car = carConfig({1.0, -0.2});
  const vf::CbfSample cbf = step(car, 0.5);
  ASSERT_TRUE(cbf.on);
  ASSERT_GE(cbf.count, 1);
  EXPECT_NEAR(cbf.n[0].x(), -1.0, 1e-9);
  EXPECT_NEAR(cbf.turn_margin[0], 1.0 - std::sin(0.5), 1e-9);
  const Vec2 rear = -0.2 * vf::heading(0.5);
  EXPECT_NEAR(cbf.h_j[0], (2.0 - rear.x()) - kDeltaCar, 1e-9);
  EXPECT_NEAR(step(car, 0.5 * vf::kPi).turn_margin[0], 0.0, 1e-12);
  EXPECT_EQ(step(car, vf::kPi).turn_margin[0], 0.0);
  vf::CoreConfig diff = tcConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking);
  diff.obstacles.obstacle_mode = vf::ObstacleMode::kCbf;
  EXPECT_EQ(step(diff, 0.5).turn_margin[0], 0.0);
}
