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

// CBF oracle O58 (math_spec §6.7 [Design, M7], §12.5): closed loops of the TC robot along y = 0.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

/// δ_safe of the TC 16-gon: r_circ + margin at base_link, r_circ + d at the FL control point
/// (the vertex on the body x-axis is the farthest from p_c).
constexpr double kMargin = 0.05;
constexpr double kDeltaBase = kRCirc + kMargin;
constexpr double kDeltaFl = kRCirc + kD + kMargin;

vf::CoreConfig cbfConfig(vf::MotionModel model, vf::TrackingLaw law, double speed = kVr)
{
  vf::CoreConfig config = tcConfig(model, law);
  config.obstacles.obstacle_mode = vf::ObstacleMode::kCbf;
  config.obstacles.cbf_margin = kMargin;
  config.field.desired_speed = speed;
  return config;
}

vf::CoreConfig flConfig(double speed = kVr)
{
  return cbfConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kFeedbackLinearization, speed);
}

vf::CoreConfig omniConfig(double speed = kVr)
{
  return cbfConfig(vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic, speed);
}

vf::CoreConfig htConfig(double speed = kVr)
{
  return cbfConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking, speed);
}

double deltaOf(const vf::CoreConfig & config)
{
  return config.motion.tracking_law == vf::TrackingLaw::kFeedbackLinearization ? kDeltaFl
                                                                               : kDeltaBase;
}

/// Metrics of one straight run.
struct CbfRun
{
  /// Brute-force barrier at the evaluation point, min over the cycles with the filter on.
  double min_h{vf::kInf};
  /// Same, as the Tracker reported it.
  double min_h_reported{vf::kInf};
  /// The δ_safe the Tracker used (constant along a run without phase changes).
  double delta{0.0};
  double max_abs_D{0.0};
  double final_abs_D{vf::kInf};
  bool settled{false};
  int blocked_cycles{0};
  int hold_cycles{0};
  int active_cycles{0};
  int contour_cycles{0};
  /// Smallest base speed over the cycles with x in [x_lo, x_hi].
  double min_speed{vf::kInf};
  /// Largest change of the target ω and v_y between consecutive cycles with the filter active.
  double max_dw{0.0};
  double max_dvy{0.0};
  double x_end{0.0};
  double final_speed{vf::kInf};
  std::size_t cycles{0};
};

/// Runs runStraight() and measures it; stops once kBlocked has lasted `blocked_stop` cycles.
CbfRun runCbf(
  const vf::CoreConfig & config, ObstacleWorld & world, const Pose2 & start, double x_goal,
  double timeout, double x_lo = -vf::kInf, double x_hi = vf::kInf, int blocked_stop = 20)
{
  int blocked = 0;
  const StraightRun r = runStraight(
    config, world, start, x_goal, timeout, {}, [&blocked, blocked_stop](const Cycle & c) {
      blocked = vf::Status::has(c.out.status, vf::Status::kBlocked) ? blocked + 1 : 0;
      return blocked >= blocked_stop;
    });
  CbfRun m;
  m.cycles = r.cycles.size();
  const Cycle * previous = nullptr;
  for (const Cycle & c : r.cycles) {
    const vf::CbfSample & cbf = c.out.field.cbf;
    if (cbf.on) {
      if (m.delta == 0.0) {
        m.delta = cbf.delta;  // the first cycle's (FOLLOW; FINAL_APPROACH switches FL to base_link)
      }
      m.min_h = std::min(m.min_h, world.distanceToSites(c.out.field.x_eval) - cbf.delta);
      m.min_h_reported = std::min(m.min_h_reported, cbf.h);
    }
    m.max_abs_D = std::max(m.max_abs_D, std::abs(c.out.field.D));
    m.settled = m.settled || c.out.settled;
    m.blocked_cycles += vf::Status::has(c.out.status, vf::Status::kBlocked) ? 1 : 0;
    m.hold_cycles += cbf.hold ? 1 : 0;
    m.active_cycles += cbf.active ? 1 : 0;
    m.contour_cycles += c.out.field.contour.active ? 1 : 0;
    const double x = c.input.pose.p.x();
    if (x >= x_lo && x <= x_hi) {
      m.min_speed = std::min(m.min_speed, std::hypot(c.rec.measured.vx, c.rec.measured.vy));
    }
    if (previous != nullptr && (cbf.active || previous->out.field.cbf.active)) {
      m.max_dw = std::max(m.max_dw, std::abs(c.out.cmd_target.wz - previous->out.cmd_target.wz));
      m.max_dvy = std::max(m.max_dvy, std::abs(c.out.cmd_target.vy - previous->out.cmd_target.vy));
    }
    previous = &c;
  }
  if (!r.cycles.empty()) {
    const Cycle & last = r.cycles.back();
    m.final_abs_D = std::abs(last.out.field.D);
    m.x_end = last.pose_after.p.x();
    m.final_speed = std::hypot(last.out.cmd.vx, last.out.cmd.vy);
  }
  return m;
}

std::string describe(const CbfRun & m)
{
  return "min h " + std::to_string(m.min_h) + " (reported " + std::to_string(m.min_h_reported) +
         "), delta " + std::to_string(m.delta) + ", max |D| " + std::to_string(m.max_abs_D) +
         ", final |D| " + std::to_string(m.final_abs_D) + ", settled " + std::to_string(m.settled) +
         ", blocked " + std::to_string(m.blocked_cycles) + ", hold " +
         std::to_string(m.hold_cycles) + ", active " + std::to_string(m.active_cycles) +
         ", min speed " + std::to_string(m.min_speed) + ", max dw " + std::to_string(m.max_dw) +
         ", max dvy " + std::to_string(m.max_dvy) + ", x_end " + std::to_string(m.x_end) +
         ", cycles " + std::to_string(m.cycles);
}

}  // namespace

// O58 (a) Offset disc: a 0.15 m disc 0.1, 0.2 and 0.3 m off the path, FL, HT and omni. The robot
// passes (settles at the goal) without kBlocked, the brute-force barrier at the evaluation point
// stays above −res, the filter binds, and the robot rejoins the path.
TEST(OracleCbf, O58a_OffsetDiscPassesAndRejoins)
{
  for (const vf::CoreConfig & config : {flConfig(), htConfig(), omniConfig()}) {
    for (const double offset : {0.1, 0.2, 0.3}) {
      SCOPED_TRACE(
        std::string(vf::toString(config.motion.tracking_law)) + ", offset " +
        std::to_string(offset));
      ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
      world.addDisc(Vec2(2.0, offset), 0.15);
      world.commit();
      const CbfRun m = runCbf(config, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 5.0, 40.0);
      EXPECT_TRUE(m.settled) << describe(m);
      EXPECT_EQ(m.blocked_cycles, 0) << describe(m);
      EXPECT_GE(m.active_cycles, 1) << describe(m);
      EXPECT_GE(m.min_h, -kRes) << describe(m);
      EXPECT_NEAR(m.delta, deltaOf(config), 1e-12) << describe(m);
      EXPECT_LT(m.final_abs_D, 0.02) << describe(m);
    }
  }
}

// O58 (b) Doorway: a wall across the path with an opening of half-width δ_safe + 0.03 m. The
// contour gate never opens (no sample within w_lat − 0.05 of a jamb), so the contour mode does
// nothing. The CBF filter binds at the jambs (both in the QP, so their lateral pushes cancel):
// |D| stays within 0.02 m and the robot passes at no less than 60 % of v_r (about
// γ_b sqrt(2 · 0.03 · half) for a point jamb: 0.36 m/s FL, 0.29 m/s at base_link).
TEST(OracleCbf, O58b_DoorwayWithoutContourKeepsThePath)
{
  for (const vf::CoreConfig & config : {flConfig(), htConfig(), omniConfig()}) {
    SCOPED_TRACE(vf::toString(config.motion.tracking_law));
    const double half = deltaOf(config) + 0.03;
    ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
    world.addBox(Vec2(2.0, -2.4), Vec2(2.1, -half));
    world.addBox(Vec2(2.0, half), Vec2(2.1, 2.4));
    world.commit();
    const CbfRun m = runCbf(config, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 5.0, 40.0, 0.5, 3.5);
    EXPECT_TRUE(m.settled) << describe(m);
    EXPECT_EQ(m.blocked_cycles, 0) << describe(m);
    EXPECT_LT(m.max_abs_D, 0.02) << describe(m);
    EXPECT_GE(m.min_h, -kRes) << describe(m);
    EXPECT_GE(m.active_cycles, 1) << describe(m);
    EXPECT_GE(m.min_speed, 0.6 * kVr) << describe(m);

    vf::CoreConfig contour = config;
    contour.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
    const CbfRun c = runCbf(contour, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 5.0, 40.0);
    EXPECT_TRUE(c.settled) << describe(c);
    EXPECT_EQ(c.contour_cycles, 0) << "the contour engaged in the doorway";
  }
}

// O58 (c) Centred disc on a symmetric grid (the path on a row of cell centres, the disc centred
// on it): F ∥ −n, so the filter stops the robot (the accepted deadlock, math_spec §6.7). The hold
// raises kBlocked, which keeps the robot at rest in front of the disc with h ≥ 0.
TEST(OracleCbf, O58c_CentredDiscHoldsAndBlocks)
{
  for (const vf::CoreConfig & config : {flConfig(), omniConfig()}) {
    SCOPED_TRACE(vf::toString(config.motion.tracking_law));
    ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
    world.addDisc(Vec2(2.0, 0.0), 0.3);
    world.commit();
    const CbfRun m =
      runCbf(config, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 5.0, 15.0, -vf::kInf, vf::kInf, 120);
    EXPECT_FALSE(m.settled) << describe(m);
    EXPECT_GE(m.hold_cycles, 1) << describe(m);
    EXPECT_GE(m.blocked_cycles, 100) << "kBlocked must persist: " << describe(m);
    EXPECT_GE(m.min_h, 0.0) << describe(m);
    EXPECT_LT(m.x_end, 2.0 - 0.3 - deltaOf(config) + 0.05) << describe(m);
    EXPECT_LT(m.final_speed, 1e-9) << describe(m);
    EXPECT_LT(m.max_abs_D, 1e-9) << "the symmetric run left the path: " << describe(m);
  }
}

// O58 (d) A wall parallel to the path at 0.45 m: the robot keeps at least 90 % of v_r over the
// wall; |D| stays within 0.05 m (FL: δ_safe = 0.47 m > 0.45 m, so the filter holds the control
// point 2–3 cm off the path; omni and HT: inactive).
TEST(OracleCbf, O58d_ParallelWallDoesNotSlowDown)
{
  for (const vf::CoreConfig & config : {flConfig(), htConfig(), omniConfig()}) {
    SCOPED_TRACE(vf::toString(config.motion.tracking_law));
    ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
    world.addBox(Vec2(-1.0, 0.45), Vec2(6.0, 0.6));
    world.commit();
    const CbfRun m = runCbf(config, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 5.0, 40.0, 0.5, 3.5);
    EXPECT_TRUE(m.settled) << describe(m);
    EXPECT_EQ(m.blocked_cycles, 0) << describe(m);
    EXPECT_GE(m.min_speed, 0.9 * kVr) << describe(m);
    EXPECT_LT(m.max_abs_D, 0.05) << describe(m);
    EXPECT_GE(m.min_h, -kRes) << describe(m);
  }
}

// O58 (e) Two 0.15 m discs, the robot starting 3 cm off their bisector. Wide gap (site centres
// 2 δ_safe + 0.06 m apart): with three sites the robot passes, and the target ω (FL) and v_y
// (omni) change by at most 0.05 per cycle while the filter binds. Narrow gap (2 δ_safe − 0.04 m,
// below the barrier's width): three sites hold the robot before the waist with h ≥ 0 (kBlocked);
// one site, whose nearest site jumps from one disc to the other at the bisector, lets FL squeeze
// through with ω jumping by more than ten times as much.
TEST(OracleCbf, O58e_TwoDiscsPassWithoutChattering)
{
  for (const vf::CoreConfig & base : {flConfig(), omniConfig()}) {
    SCOPED_TRACE(vf::toString(base.motion.tracking_law));
    const bool fl = base.motion.tracking_law == vf::TrackingLaw::kFeedbackLinearization;
    const double delta = deltaOf(base);
    const Pose2 start{Vec2(-1.0, 0.03), 0.0};
    const auto world = [delta](double clearance) {
      ObstacleWorld w = centredWorld(-2.5, 7.0, -2.5, 2.5);
      w.addDisc(Vec2(2.0, 0.15 + delta + clearance), 0.15);
      w.addDisc(Vec2(2.0, -0.15 - delta - clearance), 0.15);
      w.commit();
      return w;
    };
    ObstacleWorld wide = world(0.01);  // nearest sites at δ_safe + 0.03 from the bisector
    const CbfRun m = runCbf(base, wide, start, 5.0, 40.0);
    EXPECT_TRUE(m.settled) << describe(m);
    EXPECT_EQ(m.blocked_cycles, 0) << describe(m);
    EXPECT_GE(m.active_cycles, 1) << describe(m);
    EXPECT_GE(m.min_h, -kRes) << describe(m);
    EXPECT_LE(fl ? m.max_dw : m.max_dvy, 0.05) << describe(m);

    ObstacleWorld narrow = world(-0.02);  // nearest sites at δ_safe − 0.02
    const CbfRun held = runCbf(base, narrow, start, 5.0, 20.0);
    EXPECT_FALSE(held.settled) << describe(held);
    EXPECT_GE(held.hold_cycles, 1) << describe(held);
    EXPECT_GE(held.blocked_cycles, 20) << describe(held);
    EXPECT_GE(held.min_h, 0.0) << describe(held);
    EXPECT_LT(held.x_end, 2.0) << describe(held);
    if (fl) {
      vf::CoreConfig one = base;
      one.obstacles.cbf_sites = 1;
      const CbfRun single = runCbf(one, narrow, start, 5.0, 20.0);
      EXPECT_GT(single.max_dw, 10.0 * held.max_dw)
        << "one site: " << describe(single) << "\nthree sites: " << describe(held);
    }
  }
}

// O58 (f) Zero-order hold at 20 Hz, FL and omni at 0.4 and 0.5 m/s: the brute-force barrier stays
// above −res for a head-on wall, a centred disc and discs 0.1 and 0.2 m off the path.
TEST(OracleCbf, O58f_BarrierUnderZeroOrderHold)
{
  for (const double speed : {0.4, 0.5}) {
    for (const vf::CoreConfig & config : {flConfig(speed), omniConfig(speed)}) {
      for (int scenario = 0; scenario < 4; ++scenario) {
        SCOPED_TRACE(
          std::string(vf::toString(config.motion.tracking_law)) + ", speed " +
          std::to_string(speed) + ", scenario " + std::to_string(scenario));
        ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
        if (scenario == 0) {
          world.addBox(Vec2(2.0, -1.0), Vec2(2.2, 1.0));
        } else {
          world.addDisc(Vec2(2.0, 0.1 * (scenario - 1)), 0.2);
        }
        world.commit();
        const CbfRun m = runCbf(config, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 5.0, 25.0);
        EXPECT_GE(m.min_h, -kRes) << describe(m);
        EXPECT_GE(m.active_cycles, 1) << describe(m);
      }
    }
  }
}

// O58 Reversing (math_spec §6.7 "Interaction"): with drive_backward (q = −1) the FL control point
// is behind base_link and the filter runs there, with the same δ_safe for the symmetric 16-gon.
// A 0.15 m disc 0.2 m off the path is passed backwards with the barrier above −res.
TEST(OracleCbf, O58_DriveBackwardFiltersAtTheRearControlPoint)
{
  vf::CoreConfig config = flConfig();
  config.motion.drive_backward = true;
  ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
  world.addDisc(Vec2(2.0, 0.2), 0.15);
  world.commit();
  const CbfRun m = runCbf(config, world, Pose2{Vec2(-1.0, 0.0), vf::kPi}, 5.0, 40.0);
  EXPECT_TRUE(m.settled) << describe(m);
  EXPECT_EQ(m.blocked_cycles, 0) << describe(m);
  EXPECT_GE(m.active_cycles, 1) << describe(m);
  EXPECT_GE(m.min_h, -kRes) << describe(m);
  EXPECT_NEAR(m.delta, kDeltaFl, 1e-12) << describe(m);
}
