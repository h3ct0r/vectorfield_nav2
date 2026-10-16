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

// Closed-loop oracles of the phases (math_spec §8.4, §9, §12.8): O49 cancel, O54 settled.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

// Cycles needed by the acceleration stage to bring |u| to zero at rate `rate`, plus 2
// (math_spec O49: "within v/a_can + 2 cycles").
std::size_t allowedCycles(double speed, double rate)
{
  return static_cast<std::size_t>(std::ceil(speed / (rate * kPeriod) - 1e-9)) + 2U;
}

// O54: in the translation phases, inside xy_tol settled iff one of its clauses holds and outside
// never settled; ROTATE_TO_HEADING is never entered without settled (this cycle or the previous).
// Clauses (revised rule): ‖p − g‖ <= σ_stop, the cap binds, and for Ackermann only σ_base <= σ_stop.
// XY reached is the follower latch, re-derived here from the poses (math_spec §9.1).
// Returns the number of cycles where the predicate was checked inside xy_tol.
int checkSettledPredicate(
  const std::vector<Cycle> & cycles, const Vec2 & g, double xy_tol, bool ackermann)
{
  const double sigma_stop = 0.5 * xy_tol;
  bool latch = false;
  int inside = 0;
  for (std::size_t k = 0; k < cycles.size(); ++k) {
    const Cycle & c = cycles[k];
    const double dist = (c.rec.pose.p - g).norm();
    latch = latch || dist <= xy_tol;
    if (
      k > 0 && c.out.phase == Phase::kRotateToHeading &&
      cycles[k - 1].out.phase != Phase::kRotateToHeading) {
      EXPECT_TRUE(c.out.settled || cycles[k - 1].out.settled)
        << "ROTATE_TO_HEADING entered without settled at t = " << c.rec.t;
    }
    // The predicate is checked in the translation phases, where it is the transition trigger; in
    // the rotation phases the cap comes from a rotation-only rollout.
    if (!vf::isTranslationPhase(c.out.phase)) {
      continue;
    }
    const double sigma = c.out.field.sigma;
    EXPECT_FALSE(std::isnan(sigma)) << "σ_base unset in a translation phase at t = " << c.rec.t;
    if (
      std::abs(dist - xy_tol) < 1e-9 || std::abs(dist - sigma_stop) < 1e-9 ||
      std::abs(sigma - sigma_stop) < 1e-9) {
      continue;  // floating-point boundary: either answer is acceptable
    }
    // Settled rule as revised in review (replacing math_spec §9.1's σ clause for non-Ackermann):
    // settled := XY reached ∧ (‖p − g‖ <= σ_stop ∨ cap binds ∨ (Ackermann ∧ σ_base <= σ_stop)).
    const bool clause =
      dist <= sigma_stop || c.out.shaping.capBinds() || (ackermann && sigma <= sigma_stop);
    EXPECT_EQ(c.out.settled, latch && clause)
      << "t = " << c.rec.t << ", ‖p − g‖ = " << dist << ", σ_base = " << sigma
      << ", cap binds = " << c.out.shaping.capBinds() << ", XY latch = " << latch;
    if (latch) {
      ++inside;
    }
  }
  return inside;
}

}  // namespace

// O49 Cancel, Ackermann (math_spec §8.4, §9.2 STOPPED): moving at 0.3 m/s with
// speed_limit_scale 0.1, a cancel reaches |v| < 0.01 within v/a_can + 2 cycles (the STOPPED
// target bypasses the v_ack band and the speed-limit floor), stays STOPPED and completes.
TEST(OraclePhases, O49_CancelAckermann)
{
  vf::CoreConfig config = tcConfig(vf::MotionModel::kAckermann, vf::TrackingLaw::kAuto);
  config.motion.min_turning_radius = 0.5;
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(20.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.stop_when_stopped = false;
  Runner runner(config, options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0}, 0.0, Twist2{0.3, 0.0, 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(20.0, 0.0));
  runner.input().speed_limit_scale = 0.1;
  runner.input().cancel = true;
  runner.run(2.0);
  const auto & cs = runner.cycles();
  const std::size_t k_stop =
    firstIndex(cs, [](const Cycle & c) { return std::abs(c.rec.cmd.vx) < 0.01; });
  ASSERT_LT(k_stop, cs.size());
  EXPECT_LE(k_stop + 1, allowedCycles(0.3, config.phases.cancel_deceleration));
  for (const Cycle & c : cs) {
    EXPECT_EQ(c.out.phase, Phase::kStopped) << "t = " << c.rec.t;
  }
  EXPECT_TRUE(cs.back().out.cancel_complete);
}

// O49 Cancel, skid in ROTATE_TO_FIELD with min_in_place_angular_vel 0.3 (math_spec §7.6, §9.2):
// once rotating, a cancel reaches |ω| < 0.01 within ω/α_max + 2 cycles (the rotate floor is
// bypassed in STOPPED).
TEST(OraclePhases, O49_CancelSkidRotateToField)
{
  vf::CoreConfig config = tcConfig(vf::MotionModel::kSkidSteer, vf::TrackingLaw::kAuto);
  config.motion.min_in_place_angular_vel = 0.3;
  const auto curve = makePolyline({Vec2(0.0, 0.0), Vec2(-3.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.stop_when_stopped = false;
  Runner runner(config, options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(-3.0, 0.0));
  // Rotate for 6 cycles first.
  int rotating = 0;
  for (int k = 0; k < 40 && rotating < 6; ++k) {
    if (runner.step().out.phase == Phase::kRotateToField) {
      ++rotating;
    }
  }
  ASSERT_EQ(rotating, 6) << "never rotated to the field";
  const double w_cancel = std::abs(runner.cycles().back().rec.cmd.wz);
  ASSERT_GT(w_cancel, 0.3);
  const std::size_t k_cancel = runner.cycles().size();
  runner.input().cancel = true;
  runner.run(2.0);
  const auto & cs = runner.cycles();
  std::size_t k_stop = cs.size();
  for (std::size_t k = k_cancel; k < cs.size(); ++k) {
    EXPECT_EQ(cs[k].out.phase, Phase::kStopped);
    EXPECT_LT(std::abs(cs[k].rec.cmd.vx), 0.01);
    if (k_stop == cs.size() && std::abs(cs[k].rec.cmd.wz) < 0.01) {
      k_stop = k;
    }
  }
  ASSERT_LT(k_stop, cs.size());
  EXPECT_LE(k_stop - k_cancel + 1, allowedCycles(w_cancel, config.motion.max_angular_accel));
  EXPECT_TRUE(cs.back().out.cancel_complete);
}

// O54 Settled (math_spec §9.1, rule as revised in review): inside xy_tol, settled iff
// ‖p − g‖ <= σ_stop, or the cap binds, or (Ackermann only) σ_base <= σ_stop; never settled before
// XY is reached; ROTATE_TO_HEADING is never entered without settled. Runs: straight approaches
// with lateral offsets 0 and ±0.3 m, a lateral 0.3 m goal, an Ackermann approach from 0.3 m off the
// path (σ clause), and a wall whose face is 0.08 m beyond the goal (cap clause). The face must lie
// between 0.07 and 0.095 m beyond the goal: contact (0.14 m before the goal) then precedes the
// settle point, so the rollout predicts it, and the stop m_s short of contact (0.24 m from the goal)
// is inside xy_tol. A face <= 0.05 m beyond the goal stops the robot 0.27 m away, outside xy_tol
// (never settled, kBlocked); a face 0.10 m beyond lets the nominal rollout settle first (s_free =
// +inf, math_spec §10.2), see O54_GoalNearWallSettlesWithoutCap. Timeout 20 s per run.
TEST(OraclePhases, O54_SettledPredicate)
{
  struct Case
  {
    std::string name;
    std::vector<Vec2> path;
    Vec2 goal;
    Pose2 start;
    bool wall;
    bool ackermann;
  };
  const std::vector<Case> cases = {
    {"straight",
     {Vec2(-1.0, 0.0), Vec2(2.0, 0.0)},
     Vec2(2.0, 0.0),
     Pose2{Vec2(0.0, 0.0), 0.0},
     false,
     false},
    {"offset+0.3",
     {Vec2(-1.0, 0.0), Vec2(2.0, 0.0)},
     Vec2(2.0, 0.0),
     Pose2{Vec2(0.0, 0.3), 0.0},
     false,
     false},
    {"offset-0.3",
     {Vec2(-1.0, 0.0), Vec2(2.0, 0.0)},
     Vec2(2.0, 0.0),
     Pose2{Vec2(0.0, -0.3), 0.0},
     false,
     false},
    {"lateral",
     {Vec2(0.0, 0.0), Vec2(0.0, 0.3)},
     Vec2(0.0, 0.3),
     Pose2{Vec2(0.0, 0.0), 0.0},
     false,
     false},
    {"cap",
     {Vec2(-1.0, 0.0), Vec2(2.0, 0.0)},
     Vec2(2.0, 0.0),
     Pose2{Vec2(0.0, 0.0), 0.0},
     true,
     false},
    {"ackermann offset+0.3",
     {Vec2(-1.0, 0.0), Vec2(2.0, 0.0)},
     Vec2(2.0, 0.0),
     Pose2{Vec2(-0.5, 0.3), 0.0},
     false,
     true},
  };
  int total_inside = 0;
  bool cap_clause_seen = false;
  for (const Case & tc : cases) {
    SCOPED_TRACE(tc.name);
    const auto curve = makePolyline(tc.path, false);
    // Grid lines at x = 0.03 + 0.05 k so that the wall face lies exactly at x = 2.08.
    ObstacleWorld world(Vec2(-3.02, -3.0), 8.0, 6.0);
    if (tc.wall) {
      world.addBox(Vec2(2.08, -1.0), Vec2(2.58, 1.0));
      world.commit();
    }
    vf::sim::SimulationOptions options;
    options.stop_when_settled = false;
    vf::CoreConfig config =
      tc.ackermann ? tcConfig(vf::MotionModel::kAckermann, vf::TrackingLaw::kAuto) : tcConfig();
    Runner runner(config, options);
    runner.reset(tc.start);
    runner.input().curve = curve.get();
    // Ackermann has no ROTATE_TO_HEADING (goal hold); its goal yaw is left unchecked.
    runner.input().goal = goalAt(tc.goal, 0.0, !tc.ackermann);
    runner.input().obstacles = world.map();
    double settled_at = -1.0;
    runner.run(20.0, [&](const Cycle & c) {
      if (c.out.settled && settled_at < 0.0) {
        settled_at = c.rec.t;
      }
      return settled_at >= 0.0 && c.rec.t > settled_at + 3.0;
    });
    ASSERT_GE(settled_at, 0.0) << "never settled";
    total_inside += checkSettledPredicate(runner.cycles(), tc.goal, kXyTol, tc.ackermann);
    for (const Cycle & c : runner.cycles()) {
      if (c.out.settled && c.out.shaping.capBinds()) {
        cap_clause_seen = true;
      }
      if (tc.wall) {
        EXPECT_FALSE(Status::has(c.out.status, Status::kBlocked)) << "t = " << c.rec.t;
      }
    }
  }
  EXPECT_GT(total_inside, 20) << "too few cycles inside xy_tol: the oracle is vacuous";
  EXPECT_TRUE(cap_clause_seen) << "the wall case never settled through the cap clause";
}

// O54, goal within m_s of a wall but collision-free (accepted behaviour, design review): the wall
// face is 0.10 m beyond the goal, so the nominal rollout reaches the settle point (‖p − g‖ <=
// σ_stop) before any contact: s_free = +inf, the cap never binds, and the robot settles through the
// distance clause with its footprint free. Never kBlocked. Timeout 20 s.
TEST(OraclePhases, O54_GoalNearWallSettlesWithoutCap)
{
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(2.0, 0.0)}, false);
  ObstacleWorld world(Vec2(-3.0, -3.0), 8.0, 6.0);
  world.addBox(Vec2(2.10, -1.0), Vec2(2.60, 1.0));
  world.commit();
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(2.0, 0.0), 0.0, true);
  runner.input().obstacles = world.map();
  runner.run(20.0);
  const auto & cs = runner.cycles();
  const std::size_t k = firstIndex(cs, [](const Cycle & c) { return c.out.settled; });
  ASSERT_LT(k, cs.size()) << "never settled";
  EXPECT_FALSE(cs[k].out.shaping.capBinds()) << "settled through the cap";
  EXPECT_LE((cs[k].rec.pose.p - Vec2(2.0, 0.0)).norm(), kSigmaStop + 1e-9);
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(tcFootprint()));
  for (const Cycle & c : cs) {
    EXPECT_FALSE(Status::has(c.out.status, Status::kBlocked)) << "t = " << c.rec.t;
    EXPECT_FALSE(checker.collidesGroundTruth(c.pose_after, world.grid())) << "t = " << c.rec.t;
  }
}
