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

// Closed-loop oracles of the terminal field (math_spec §5, §9, §12.4): O24 ZOH never crosses,
// O25 monotone speed, O27 Ackermann reference shift, O28 final approach.
//
// Unless stated otherwise: TC (math_spec §12), follower-style inputs (open curve built by
// makeCurve, goal = r(s_goal), `GoalInfo::xy_reached` = nullopt so the core latch decides,
// `cap_at_curve_end` false), no obstacles.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

struct Approach
{
  std::vector<Vec2> path;
  Vec2 goal{Vec2::Zero()};
  double goal_yaw{0.0};
  bool has_yaw{true};
  Pose2 start;
};

struct ApproachResult
{
  std::vector<Cycle> cycles;
  std::size_t first_settled{0};  // cycles.size() if never
  std::size_t first_rth{0};      // first ROTATE_TO_HEADING cycle, cycles.size() if never
};

// Runs until `extra` seconds after the first settled cycle (or after the first ROTATE_TO_HEADING
// cycle when `stop_after_rth`), or the timeout.
ApproachResult runApproach(
  const vf::CoreConfig & config, const Approach & a, double timeout, double extra = 1.0,
  bool stop_after_rth = false)
{
  const auto curve = makePolyline(a.path, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(config, options);
  runner.reset(a.start);
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(a.goal, a.goal_yaw, a.has_yaw);
  double mark = -1.0;
  runner.run(timeout, [&](const Cycle & c) {
    const bool event = stop_after_rth ? c.out.phase == Phase::kRotateToHeading : c.out.settled;
    if (event && mark < 0.0) {
      mark = c.rec.t;
    }
    return mark >= 0.0 && c.rec.t >= mark + extra;
  });
  ApproachResult r;
  r.cycles = runner.cycles();
  r.first_settled = firstIndex(r.cycles, [](const Cycle & c) { return c.out.settled; });
  r.first_rth =
    firstIndex(r.cycles, [](const Cycle & c) { return c.out.phase == Phase::kRotateToHeading; });
  return r;
}

// Straight path along +x ending at the goal (2, 0); start 2 m before the goal with a lateral
// offset, aligned with the path (O24, O25).
Approach straightApproach(double lateral)
{
  Approach a;
  a.path = {Vec2(-1.0, 0.0), Vec2(2.0, 0.0)};
  a.goal = Vec2(2.0, 0.0);
  a.goal_yaw = 0.0;
  a.has_yaw = true;
  a.start = Pose2{Vec2(0.0, lateral), 0.0};
  return a;
}

// O25 check: once σ_base <= σ_b and until settled, |v| never increases by more than 1e-9
// between consecutive cycles, except at the v_min floor (|v| <= v_min) and at most one increase
// within one cycle of the FINAL_APPROACH switch.
void expectMonotoneSpeed(const ApproachResult & r, double v_floor, bool allow_switch_lag)
{
  const auto & cs = r.cycles;
  const std::size_t k0 =
    firstIndex(cs, [](const Cycle & c) { return c.out.field.sigma <= kSigmaB; });
  ASSERT_LT(k0, cs.size()) << "σ_base never reached σ_b";
  const std::size_t k_fa =
    firstIndex(cs, [](const Cycle & c) { return c.out.phase == Phase::kFinalApproach; });
  int switch_excuses = 0;
  for (std::size_t k = k0 + 1; k < cs.size() && k <= r.first_settled; ++k) {
    const double prev = std::abs(cs[k - 1].rec.cmd.vx);
    const double now = std::abs(cs[k].rec.cmd.vx);
    if (now <= prev + 1e-9 || now <= v_floor + 1e-9) {
      continue;
    }
    if (
      allow_switch_lag && k_fa < cs.size() && (k == k_fa || k == k_fa + 1) && switch_excuses == 0) {
      ++switch_excuses;
      continue;
    }
    ADD_FAILURE() << "|v| increased from " << prev << " to " << now << " at t = " << cs[k].rec.t
                  << " (σ_base = " << cs[k].out.field.sigma << ", phase "
                  << vf::phaseName(cs[k].out.phase) << ")";
    return;
  }
}

}  // namespace

// O24 ZOH never crosses (math_spec §5.3–§5.4): TC, FL, straight polyline, start 2 m before the
// goal with lateral offsets 0 and ±0.3 m: σ_base > 0 on every cycle (the Tracker's value and the
// analytic one, 2 − p_x) and settled fires within 15 s (reference 5.65–5.7 s, Appendix B).
// Each run continues 2 s after settling. Timeout 17 s.
TEST(OracleTerminal, O24_ZohNeverCrosses)
{
  for (const double lateral : {0.0, 0.3, -0.3}) {
    SCOPED_TRACE("lateral offset " + std::to_string(lateral));
    const ApproachResult r = runApproach(tcConfig(), straightApproach(lateral), 17.0, 2.0);
    ASSERT_LT(r.first_settled, r.cycles.size()) << "never settled";
    EXPECT_LE(r.cycles[r.first_settled].rec.t, 15.0);
    for (const Cycle & c : r.cycles) {
      // The Tracker's σ_base in the translation phases (it may be unset while rotating), the
      // analytic one (2 − p_x on this straight path) on every cycle.
      if (vf::isTranslationPhase(c.out.phase)) {
        ASSERT_GT(c.out.field.sigma, 0.0) << "t = " << c.rec.t;
      }
      ASSERT_GT(2.0 - c.rec.pose.p.x(), 0.0) << "t = " << c.rec.t;
      ASSERT_GT(2.0 - c.pose_after.p.x(), 0.0) << "t = " << c.rec.t;
    }
  }
}

// O25 Monotone speed (math_spec §5.4 "Feasible braking"): same runs as O24; once σ_base <= σ_b
// the commanded |v| never increases by more than 1e-9 from one cycle to the next, apart from
// the v_min floor and one cycle of acceleration lag at the FINAL_APPROACH switch.
TEST(OracleTerminal, O25_MonotoneSpeed)
{
  for (const double lateral : {0.0, 0.3, -0.3}) {
    SCOPED_TRACE("lateral offset " + std::to_string(lateral));
    const ApproachResult r = runApproach(tcConfig(), straightApproach(lateral), 17.0, 0.5);
    ASSERT_LT(r.first_settled, r.cycles.size()) << "never settled";
    expectMonotoneSpeed(r, kVmin, true);
  }
}

// O27 Reference shift (math_spec §5.2, §7.5): Ackermann (R_min 0.5) with reference_point_offset
// in {−0.3, 0, +0.3} on a straight aligned approach from 2 m: speed non-increasing over the last
// σ_b (allowing the v_ack floor), final ‖p − g‖ <= 0.6 xy_tol, settled. Runs continue 1 s after
// settling (goal hold). Timeout 20 s.
TEST(OracleTerminal, O27_AckermannReferenceShift)
{
  for (const double x_r : {-0.3, 0.0, 0.3}) {
    SCOPED_TRACE("reference_point_offset " + std::to_string(x_r));
    vf::CoreConfig config = tcConfig(vf::MotionModel::kAckermann, vf::TrackingLaw::kAuto);
    config.motion.min_turning_radius = 0.5;
    config.motion.reference_point_offset = x_r;
    const ApproachResult r = runApproach(config, straightApproach(0.0), 20.0, 1.0);
    ASSERT_LT(r.first_settled, r.cycles.size()) << "never settled";
    expectMonotoneSpeed(r, config.motion.ackermann_min_speed, false);
    const Cycle & last = r.cycles.back();
    EXPECT_LE((last.pose_after.p - Vec2(2.0, 0.0)).norm(), 0.6 * kXyTol);
    EXPECT_FALSE(vf::Status::has(last.out.status, Status::kHeadingInfeasible));
    for (const Cycle & c : r.cycles) {
      ASSERT_NE(c.out.phase, Phase::kStopped) << "t = " << c.rec.t;
      ASSERT_NE(c.out.phase, Phase::kRotateToHeading) << "Ackermann has no ROTATE_TO_HEADING";
    }
  }
}

namespace
{

struct LawCase
{
  const char * name;
  vf::MotionModel model;
  vf::TrackingLaw law;
  vf::OmniHeadingPolicy policy;
};

const LawCase kLawCases[] = {
  {"FL", vf::MotionModel::kDifferential, vf::TrackingLaw::kFeedbackLinearization,
   vf::OmniHeadingPolicy::kPathTangent},
  {"HT", vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking,
   vf::OmniHeadingPolicy::kPathTangent},
  {"omni_path_tangent", vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic,
   vf::OmniHeadingPolicy::kPathTangent},
  {"omni_fixed", vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic,
   vf::OmniHeadingPolicy::kFixed},
};

// The O28 goals. "60" and "90": straight path along +x from 2 m before the goal at the origin,
// goal yaw 60° / 90° from the arrival heading. The lateral 0.3 m goal in its two readings (design
// review): "beside": straight path ending at the goal, the robot starts 0.3 m beside the goal,
// parallel to the path (σ_base = 0 at the start); "sidestep": the path ends in a 0.3 m sidestep
// ([−2.5, 0] → [0, 0] → [0, 0.3]), start 2 m before the step. "lateral": a path that is only the
// 0.3 m sidestep from the robot ([p, p + 0.3 J h]). Goal yaw = the start heading in all three.
Approach finalApproachCase(const std::string & which, double heading_offset = 0.0)
{
  Approach a;
  a.has_yaw = true;
  if (which == "lateral") {
    a.path = {Vec2(0.0, 0.0), Vec2(0.0, 0.3)};
    a.goal = Vec2(0.0, 0.3);
    a.goal_yaw = 0.0;
    a.start = Pose2{Vec2(0.0, 0.0), 0.0};
    return a;
  }
  if (which == "beside") {
    a.path = {Vec2(-2.5, 0.0), Vec2(0.0, 0.0)};
    a.goal = Vec2(0.0, 0.0);
    a.goal_yaw = 0.0;
    a.start = Pose2{Vec2(0.0, 0.3), 0.0};
    return a;
  }
  if (which == "sidestep") {
    a.path = {Vec2(-2.5, 0.0), Vec2(0.0, 0.0), Vec2(0.0, 0.3)};
    a.goal = Vec2(0.0, 0.3);
    a.goal_yaw = 0.0;
    a.start = Pose2{Vec2(-2.0, 0.0), 0.0};
    return a;
  }
  const double angle = which == "60" ? vf::kPi / 3.0 : vf::kHalfPi;
  a.path = {Vec2(-2.5, 0.0), Vec2(0.0, 0.0)};
  a.goal = Vec2(0.0, 0.0);
  a.goal_yaw = vf::wrapAngle(heading_offset + angle);
  a.start = Pose2{Vec2(-2.0, 0.0), heading_offset};
  return a;
}

}  // namespace

// O28 Final approach (math_spec §5.2, §9.2): goals at 60° and 90° from the arrival heading and a
// lateral 0.3 m goal (three variants: started 0.3 m beside the goal, a path ending in a 0.3 m
// sidestep, a pure 0.3 m sidestep), for d in {0.1, 0.3} and every law with a ROTATE_TO_HEADING
// phase (FL, HT,
// holonomic with path_tangent and fixed; Ackermann has none): ROTATE_TO_HEADING starts with
// ‖p − g‖ <= 0.6 xy_tol; no v < 0 for the unicycle laws; omni with `fixed` keeps its heading
// (|Δψ| < 1e-9) until ROTATE_TO_HEADING. Timeout 25 s per run.
TEST(OracleTerminal, O28_FinalApproach)
{
  for (const LawCase & law : kLawCases) {
    for (const double d : {0.1, 0.3}) {
      for (const char * which : {"60", "90", "lateral", "beside", "sidestep"}) {
        SCOPED_TRACE(std::string(law.name) + ", d = " + std::to_string(d) + ", goal " + which);
        vf::CoreConfig config = tcConfig(law.model, law.law);
        config.motion.control_point_offset = d;
        config.motion.omni_heading_policy = law.policy;
        const Approach a = finalApproachCase(which);
        const ApproachResult r = runApproach(config, a, 25.0, 0.5, true);
        ASSERT_LT(r.first_rth, r.cycles.size()) << "ROTATE_TO_HEADING never started";
        const Cycle & entry = r.cycles[r.first_rth];
        EXPECT_LE((entry.rec.pose.p - a.goal).norm(), 0.6 * kXyTol);
        for (std::size_t k = 0; k < r.cycles.size(); ++k) {
          const Cycle & c = r.cycles[k];
          if (law.model != vf::MotionModel::kOmnidirectional) {
            ASSERT_GE(c.rec.cmd.vx, -1e-12) << "v < 0 at t = " << c.rec.t;
            ASSERT_GE(c.out.cmd_target.vx, -1e-12) << "target v < 0 at t = " << c.rec.t;
          }
          if (law.policy == vf::OmniHeadingPolicy::kFixed && k < r.first_rth) {
            ASSERT_LT(std::abs(c.pose_after.yaw - a.start.yaw), 1e-9) << "t = " << c.rec.t;
          }
        }
      }
    }
  }
}

// O28, drive_backward (math_spec §7.1, §7.3–§7.4): diff with drive_backward (FL and HT) starts
// with its back toward the goal and arrives without a 180° turn (|wrap(ψ − ψ0)| < π/2 until
// ROTATE_TO_HEADING), never drives forward (q v >= 0, i.e. v <= 0) and starts ROTATE_TO_HEADING
// within 0.6 xy_tol. Goals at 60° and 90° from the (backward) arrival heading.
TEST(OracleTerminal, O28_FinalApproachDriveBackward)
{
  for (const vf::TrackingLaw law :
       {vf::TrackingLaw::kFeedbackLinearization, vf::TrackingLaw::kHeadingTracking}) {
    for (const double d : {0.1, 0.3}) {
      for (const char * which : {"60", "90"}) {
        SCOPED_TRACE(
          std::string(law == vf::TrackingLaw::kHeadingTracking ? "HT" : "FL") +
          ", d = " + std::to_string(d) + ", goal " + which);
        vf::CoreConfig config = tcConfig(vf::MotionModel::kDifferential, law);
        config.motion.control_point_offset = d;
        config.motion.drive_backward = true;
        const Approach a = finalApproachCase(which, vf::kPi);
        const ApproachResult r = runApproach(config, a, 25.0, 0.5, true);
        ASSERT_LT(r.first_rth, r.cycles.size()) << "ROTATE_TO_HEADING never started";
        EXPECT_LE((r.cycles[r.first_rth].rec.pose.p - a.goal).norm(), 0.6 * kXyTol);
        for (std::size_t k = 0; k < r.cycles.size(); ++k) {
          const Cycle & c = r.cycles[k];
          ASSERT_LE(c.rec.cmd.vx, 1e-12) << "forward motion at t = " << c.rec.t;
          if (k < r.first_rth) {
            ASSERT_LT(std::abs(vf::wrapAngle(c.pose_after.yaw - a.start.yaw)), vf::kHalfPi)
              << "turned around at t = " << c.rec.t;
          }
        }
      }
    }
  }
}
