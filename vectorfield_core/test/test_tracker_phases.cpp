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

// Unit tests of the Tracker's cycle structure and phase machine (plan §3.1, §3.3;
// math_spec §5.5, §9). Closed-loop oracles and scenarios live in test_oracle_* / test_scenario_*.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/simulator.hpp"
#include "vectorfield_core/tracker.hpp"

namespace vectorfield_core
{
namespace
{

/// An open straight polyline from the origin along +x.
class StraightCurve
{
public:
  explicit StraightCurve(double length = 3.0)
  {
    const std::vector<Vec2> points = {Vec2(0.0, 0.0), Vec2(length, 0.0)};
    EXPECT_EQ(curve_.assign(Span<const Vec2>(points), false), AssignStatus::kOk);
  }
  const Curve * get() const { return &curve_; }

private:
  PolylineCurve curve_;
};

GoalInfo goalAt(const Vec2 & position, double yaw = 0.0, bool has_yaw = false)
{
  GoalInfo goal;
  goal.pose = Pose2{position, yaw};
  goal.has_yaw = has_yaw;
  goal.xy_tol = 0.25;
  goal.yaw_tol = 0.25;
  return goal;
}

TrackerInput inputAt(const Curve * curve, const Pose2 & pose, const GoalInfo & goal)
{
  TrackerInput input;
  input.curve = curve;
  input.pose = pose;
  input.t = 0.0;
  input.dt = 0.05;
  input.goal = goal;
  return input;
}

bool isZero(const Twist2 & u) { return u.vx == 0.0 && u.vy == 0.0 && u.wz == 0.0; }

TEST(TrackerConstruction, ResolvesAutoValuesAndRejectsErrors)
{
  const Tracker tracker{CoreConfig{}};
  EXPECT_NEAR(tracker.config().obstacles.obstacle_contour_distance, 0.47, 1e-12);
  EXPECT_NEAR(tracker.config().obstacles.obstacle_blend_start_distance, 0.82, 1e-12);
  EXPECT_EQ(tracker.config().motion.tracking_law, TrackingLaw::kFeedbackLinearization);
  EXPECT_TRUE(tracker.validation().ok());
  EXPECT_NEAR(tracker.footprint().rCirc(), 0.22, 1e-15);

  CoreConfig too_fast;
  too_fast.field.desired_speed = 0.6;  // above max_linear_vel
  EXPECT_THROW(Tracker{too_fast}, ConfigError);
}

TEST(TrackerConstruction, InitialStateFollowsAndRequestsAReseed)
{
  const TrackerState state = Tracker::initialState();
  EXPECT_EQ(state.phase, Phase::kFollow);
  EXPECT_FALSE(state.hint.valid);
  EXPECT_FALSE(state.limiter.valid);
  EXPECT_FALSE(state.contour.active);
  TrackerState reseeded = state;
  reseeded.limiter.valid = true;
  requestLimiterReseed(&reseeded);
  EXPECT_FALSE(reseeded.limiter.valid);
}

TEST(TrackerStep, OutputStateMayAliasTheInputState)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  TrackerInput input = inputAt(curve.get(), Pose2{Vec2(0.3, 0.2), 0.3}, goalAt(Vec2(3.0, 0.0)));
  input.measured = Twist2{0.2, 0.0, 0.1};
  TrackerState aliased = Tracker::initialState();
  TrackerState source = aliased;
  TrackerState separate;
  StepScratch scratch_a;
  StepScratch scratch_b;
  for (int k = 0; k < 5; ++k) {
    const TrackerOutput a = tracker.step(input, aliased, &aliased, scratch_a);
    const TrackerOutput b = tracker.step(input, source, &separate, scratch_b);
    source = separate;
    EXPECT_EQ(a.cmd.vx, b.cmd.vx);
    EXPECT_EQ(a.cmd.wz, b.cmd.wz);
    EXPECT_EQ(a.status, b.status);
    EXPECT_EQ(aliased.phase, separate.phase);
    EXPECT_EQ(aliased.hint.s, separate.hint.s);
    EXPECT_EQ(aliased.limiter.u_prev.vx, separate.limiter.u_prev.vx);
    EXPECT_EQ(aliased.limiter.u_prev.wz, separate.limiter.u_prev.wz);
  }
}

TEST(TrackerStep, TurnsTowardTheCurveAndReportsTheFieldAtTheControlPoint)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  const TrackerOutput out = tracker.step(
    inputAt(curve.get(), Pose2{Vec2(0.5, 0.3), 0.0}, goalAt(Vec2(3.0, 0.0))), state, &state,
    scratch);
  EXPECT_EQ(out.phase, Phase::kFollow);
  EXPECT_GT(out.cmd.vx, 0.0);
  EXPECT_LT(out.cmd.wz, 0.0);
  EXPECT_EQ(out.status & Status::kNonFinite, 0U);
  // FL evaluates at p_c = p + d h(psi) on the field shifted by x_ref = d (math_spec §5.2).
  EXPECT_NEAR((out.field.x_eval - Vec2(0.7, 0.3)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(out.field.D, 0.3, 1e-12);
  EXPECT_EQ(out.field.x_ref, 0.2);
  EXPECT_TRUE(out.field.terminal);
  EXPECT_NEAR(out.field.sigma, 2.5, 1e-12);  // measured at base_link, unshifted
  EXPECT_TRUE(state.hint.valid);
  EXPECT_TRUE(state.limiter.valid);
}

TEST(TrackerPhases, RotatesToTheFieldWhenFacingAwayAndReleasesOnce)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  const TrackerInput input =
    inputAt(curve.get(), Pose2{Vec2(0.5, 0.0), kPi}, goalAt(Vec2(3.0, 0.0)));
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  const TrackerOutput first = tracker.step(input, state, &state, scratch);
  EXPECT_EQ(first.phase, Phase::kRotateToField);
  EXPECT_EQ(first.cmd.vx, 0.0);
  EXPECT_GT(first.cmd.wz, 0.0);
  EXPECT_TRUE(first.field.has_target_heading);
  EXPECT_NEAR(first.field.target_heading, 0.0, 1e-12);

  sim::SimulationOptions options;
  options.stop_when_settled = false;
  sim::ClosedLoopSimulator simulator(tracker, options);
  simulator.reset(Pose2{Vec2(0.5, 0.0), kPi});
  simulator.run(input, 5.0);
  int entries = 0;
  Phase previous = Phase::kFollow;
  for (const sim::SimulationRecord & record : simulator.records()) {
    if (record.phase == Phase::kRotateToField && previous != Phase::kRotateToField) {
      ++entries;
    }
    previous = record.phase;
    EXPECT_GE(record.cmd.vx, 0.0);  // never reverses (backstop, math_spec §9.2)
  }
  EXPECT_EQ(entries, 1);
  EXPECT_EQ(simulator.records().back().phase, Phase::kFollow);
}

TEST(TrackerPhases, CancelEntersStoppedAndCompletesAtTheCancelDeceleration)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  TrackerInput input = inputAt(curve.get(), Pose2{Vec2(1.0, 0.0), 0.0}, goalAt(Vec2(3.0, 0.0)));
  input.measured = Twist2{0.4, 0.0, 0.0};
  input.cancel = true;
  TrackerState state = Tracker::initialState();
  state.limiter.u_prev = input.measured;
  state.limiter.valid = true;
  StepScratch scratch;
  int cycles = 0;
  double previous = 0.4;
  bool complete = false;
  while (!complete && cycles < 20) {
    const TrackerOutput out = tracker.step(input, state, &state, scratch);
    ++cycles;
    EXPECT_EQ(out.phase, Phase::kStopped);
    EXPECT_LE(out.cmd.vx, previous + 1e-12);
    EXPECT_GE(previous - out.cmd.vx, -1e-12);
    EXPECT_LE(previous - out.cmd.vx, 1.0 * 0.05 + 1e-12);  // cancel_deceleration * dt
    previous = out.cmd.vx;
    complete = out.cancel_complete;
  }
  EXPECT_TRUE(complete);
  EXPECT_LE(cycles, static_cast<int>(std::ceil(0.4 / 1.0 / 0.05)) + 2);

  // STOPPED stays latched after the cancel flag drops, until goal_seq changes.
  input.cancel = false;
  TrackerOutput out = tracker.step(input, state, &state, scratch);
  EXPECT_EQ(out.phase, Phase::kStopped);
  EXPECT_TRUE(isZero(out.cmd));
  input.goal_seq = 1;
  out = tracker.step(input, state, &state, scratch);
  EXPECT_NE(out.phase, Phase::kStopped);
  EXPECT_GT(out.cmd_target.vx, 0.0);
}

TEST(TrackerPhases, TerminalModeHasHysteresisAndResetsOnANewGoal)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  const Pose2 pose{Vec2(1.0, 0.0), 0.0};
  auto terminalWith = [&](const Vec2 & goal, std::uint32_t goal_seq) {
    TrackerInput input = inputAt(curve.get(), pose, goalAt(goal));
    input.goal_seq = goal_seq;
    const TrackerOutput out = tracker.step(input, state, &state, scratch);
    EXPECT_EQ(out.field.terminal, state.terminal_mode);
    return out.field.terminal;
  };
  EXPECT_TRUE(terminalWith(Vec2(3.0, 0.04), 0));   // enters below 5 cm
  EXPECT_TRUE(terminalWith(Vec2(3.0, 0.20), 0));   // stays up to 25 cm
  EXPECT_FALSE(terminalWith(Vec2(3.0, 0.30), 0));  // leaves beyond 25 cm
  EXPECT_FALSE(terminalWith(Vec2(3.0, 0.20), 0));  // no re-entry until below 5 cm
  EXPECT_FALSE(terminalWith(Vec2(3.0, 0.20), 1));  // a new goal starts outside
  EXPECT_TRUE(terminalWith(Vec2(3.0, 0.00), 1));

  TrackerInput far = inputAt(curve.get(), pose, goalAt(Vec2(3.0, 0.3)));
  far.goal_seq = 2;
  const TrackerOutput out = tracker.step(far, state, &state, scratch);
  EXPECT_EQ(out.field.sigma, kInf);  // outside terminal mode sigma_base := +inf
}

TEST(TrackerPhases, FinalApproachHandsFeedbackLinearizationOverAtBaseLink)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  StepScratch scratch;
  // Auto final_approach_distance for FL with xy_tol 0.25: max(2d, xy_tol + d) = 0.45.
  TrackerState far_state = Tracker::initialState();
  const TrackerOutput far = tracker.step(
    inputAt(curve.get(), Pose2{Vec2(2.4, 0.0), 0.0}, goalAt(Vec2(3.0, 0.0))), far_state, &far_state,
    scratch);
  EXPECT_EQ(far.phase, Phase::kFollow);
  EXPECT_EQ(far.field.x_ref, 0.2);

  TrackerState near_state = Tracker::initialState();
  const TrackerOutput near = tracker.step(
    inputAt(curve.get(), Pose2{Vec2(2.6, 0.0), 0.0}, goalAt(Vec2(3.0, 0.0))), near_state,
    &near_state, scratch);
  EXPECT_EQ(near.phase, Phase::kFinalApproach);
  EXPECT_EQ(near.field.x_ref, 0.0);
  EXPECT_NEAR((near.field.x_eval - Vec2(2.6, 0.0)).norm(), 0.0, 1e-15);
}

TEST(TrackerPhases, SettlesAndHoldsWithoutARotatePhase)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  // Follower style: no yaw, the core XY latch.
  TrackerOutput out = tracker.step(
    inputAt(curve.get(), Pose2{Vec2(2.95, 0.0), 0.0}, goalAt(Vec2(3.0, 0.0))), state, &state,
    scratch);
  EXPECT_TRUE(out.settled);
  EXPECT_TRUE(state.holding);
  EXPECT_TRUE(state.xy_latched);
  EXPECT_TRUE(isZero(out.cmd_target));

  // The follower's latch keeps XY reached, so the hold persists away from the goal.
  out = tracker.step(
    inputAt(curve.get(), Pose2{Vec2(2.5, 0.0), 0.0}, goalAt(Vec2(3.0, 0.0))), state, &state,
    scratch);
  EXPECT_FALSE(out.settled);
  EXPECT_TRUE(state.holding);
  EXPECT_TRUE(isZero(out.cmd_target));

  // Plugin style: the goal checker no longer reports XY reached, so the hold ends.
  GoalInfo lost = goalAt(Vec2(3.0, 0.0));
  lost.xy_reached = false;
  out =
    tracker.step(inputAt(curve.get(), Pose2{Vec2(2.5, 0.0), 0.0}, lost), state, &state, scratch);
  EXPECT_FALSE(state.holding);
  EXPECT_GT(out.cmd_target.vx, 0.0);

  // A new goal resets the goal-scoped latches.
  TrackerInput next = inputAt(curve.get(), Pose2{Vec2(2.95, 0.0), 0.0}, goalAt(Vec2(3.0, 0.0)));
  next.goal_seq = 7;
  state.holding = true;
  state.xy_latched = true;
  state.ackermann_band_lifted = true;
  next.goal->xy_reached = false;
  tracker.step(next, state, &state, scratch);
  EXPECT_EQ(state.goal_seq, 7U);
  EXPECT_FALSE(state.holding);
  EXPECT_FALSE(state.ackermann_band_lifted);
}

// Settled (math_spec §9.1): XY reached and ||p - g|| <= sigma_stop = xy_tol / 2 (or the cap
// binds). The along-track test sigma_base <= sigma_stop settles only Ackermann, which cannot
// close a lateral offset in place; the others drive on to sigma_stop.
TEST(TrackerPhases, SettledNeedsTheStopRadiusExceptForAckermann)
{
  const StraightCurve curve;
  CoreConfig omni;
  omni.motion.motion_model = MotionModel::kOmnidirectional;
  omni.motion.tracking_law = TrackingLaw::kHolonomic;
  CoreConfig ackermann;
  ackermann.motion.motion_model = MotionModel::kAckermann;
  ackermann.motion.tracking_law = TrackingLaw::kHeadingTracking;
  ackermann.motion.min_turning_radius = 0.5;
  struct Case
  {
    const char * name;
    CoreConfig config;
    bool along_track_settles;
  };
  const Case cases[] = {
    {"diff", CoreConfig{}, false}, {"omni", omni, false}, {"ackermann", ackermann, true}};
  GoalInfo goal = goalAt(Vec2(3.0, 0.0));
  goal.xy_reached = true;  // plugin: the goal checker's answer
  for (const Case & c : cases) {
    SCOPED_TRACE(c.name);
    const Tracker tracker{c.config};
    // sigma_base = 0 and 0.1 <= sigma_stop = 0.125, but ||p - g|| = 0.2 and 0.224.
    for (const Vec2 & p : {Vec2(3.0, 0.2), Vec2(2.9, -0.2)}) {
      TrackerState state = Tracker::initialState();
      StepScratch scratch;
      const TrackerOutput out =
        tracker.step(inputAt(curve.get(), Pose2{p, 0.0}, goal), state, &state, scratch);
      EXPECT_EQ(out.settled, c.along_track_settles) << "p = " << p.transpose();
    }
    // Inside sigma_stop every model settles.
    TrackerState state = Tracker::initialState();
    StepScratch scratch;
    const TrackerOutput out =
      tracker.step(inputAt(curve.get(), Pose2{Vec2(2.95, 0.1), 0.0}, goal), state, &state, scratch);
    EXPECT_TRUE(out.settled);
  }
}

TEST(TrackerPhases, RotatesToTheGoalHeadingOnceSettled)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  TrackerState state = Tracker::initialState();
  StepScratch scratch;
  GoalInfo goal = goalAt(Vec2(3.0, 0.0), kHalfPi, true);
  goal.xy_reached = true;  // plugin: the goal checker's answer
  TrackerOutput out =
    tracker.step(inputAt(curve.get(), Pose2{Vec2(2.97, 0.0), 0.0}, goal), state, &state, scratch);
  EXPECT_TRUE(out.settled);
  EXPECT_EQ(out.phase, Phase::kRotateToHeading);
  EXPECT_NEAR(out.cmd_target.wz, 1.5, 1e-12);  // min(rotate vel, sqrt(2 alpha |e|))
  EXPECT_EQ(out.cmd_target.vx, 0.0);
  EXPECT_NEAR(out.field.target_heading, kHalfPi, 1e-15);

  // Inside yaw_tol / 2 the rotate law commands zero.
  out = tracker.step(
    inputAt(curve.get(), Pose2{Vec2(2.97, 0.0), kHalfPi - 0.1}, goal), state, &state, scratch);
  EXPECT_EQ(out.phase, Phase::kRotateToHeading);
  EXPECT_EQ(out.cmd_target.wz, 0.0);

  // XY lost: back to FINAL_APPROACH (sigma_base below the entry distance).
  goal.xy_reached = false;
  out =
    tracker.step(inputAt(curve.get(), Pose2{Vec2(2.7, 0.0), 0.0}, goal), state, &state, scratch);
  EXPECT_EQ(out.phase, Phase::kFinalApproach);
}

TEST(TrackerPhases, ProbesDoNotChangeTheNextStep)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve;
  const TrackerInput input =
    inputAt(curve.get(), Pose2{Vec2(0.4, 0.25), 0.2}, goalAt(Vec2(3.0, 0.0)));
  TrackerState probed = Tracker::initialState();
  TrackerState plain = probed;
  StepScratch scratch_probed;
  StepScratch scratch_plain;
  tracker.step(input, probed, &probed, scratch_probed);
  tracker.step(input, plain, &plain, scratch_plain);
  for (const Vec2 & x : {Vec2(0.0, 1.0), Vec2(2.0, -0.5), Vec2(5.0, 0.3), Vec2(-1.0, 0.0)}) {
    const ProbeSample probe = tracker.probe(x, input, probed, scratch_probed);
    ASSERT_TRUE(std::isfinite(probe.psi_d));
    EXPECT_EQ(probe.psi_d, angleOf(probe.F));
    EXPECT_GT(probe.F.norm(), 0.0);
  }
  const TrackerOutput a = tracker.step(input, probed, &probed, scratch_probed);
  const TrackerOutput b = tracker.step(input, plain, &plain, scratch_plain);
  EXPECT_EQ(a.cmd.vx, b.cmd.vx);
  EXPECT_EQ(a.cmd.wz, b.cmd.wz);
  EXPECT_EQ(probed.hint.s, plain.hint.s);
}

TEST(TrackerStep, RejectsUnusableInputWithoutThrowing)
{
  const Tracker tracker{CoreConfig{}};
  TrackerState state = Tracker::initialState();
  state.limiter.u_prev = Twist2{0.3, 0.0, 0.2};
  state.limiter.valid = true;
  StepScratch scratch;
  TrackerInput input;  // no curve
  const TrackerOutput out = tracker.step(input, state, &state, scratch);
  EXPECT_NE(out.status & Status::kNonFinite, 0U);
  EXPECT_TRUE(isZero(out.cmd));
  EXPECT_TRUE(isZero(state.limiter.u_prev));
}

}  // namespace
}  // namespace vectorfield_core
