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

// GoalSupervisor (plan §6): the precedence table (cancel/deactivation > kBlocked and the progress
// watchdog > the Assumption-3 timer, paused while kBlocked), the timers of the holds, and the
// deceleration ramp.

#include <gtest/gtest.h>

#include <cstdint>

#include "vectorfield_core/status.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_curve_follower::belowStopThresholds;
using vectorfield_curve_follower::GoalSupervisor;
using vectorfield_curve_follower::rampDown;
using vectorfield_curve_follower::SupervisorAction;
using vectorfield_curve_follower::SupervisorConfig;
using vectorfield_curve_follower::SupervisorDecision;
using vectorfield_curve_follower::SupervisorInput;
namespace result_code = vectorfield_curve_follower::result_code;

constexpr std::uint32_t kBlocked = vf::Status::kBlocked;
constexpr std::uint32_t kClamped = vf::Status::kAssumption3Clamped;

/// A healthy cycle at time `t` where the Tracker ran with `status`, the robot at x = `x`.
SupervisorInput ran(double t, std::uint32_t status = 0U, double x = 0.0)
{
  SupervisorInput input;
  input.now = t;
  input.tracker_ran = true;
  input.status = status;
  input.position = vf::Vec2(x, 0.0);
  input.nominal_speed = 0.4;
  return input;
}

void expectAbort(const SupervisorDecision & decision, std::uint16_t code, bool decelerate)
{
  EXPECT_EQ(decision.action, SupervisorAction::kAbort);
  EXPECT_EQ(decision.code, code);
  EXPECT_EQ(decision.decelerate, decelerate);
}

TEST(GoalSupervisor, PrecedenceTable)
{
  struct Row
  {
    const char * name;
    SupervisorInput input;
    SupervisorAction action;
    std::uint16_t code;
  };
  auto make = [](auto edit) {
    SupervisorInput input = ran(0.0, kBlocked | kClamped | vf::Status::kHeadingInfeasible);
    input.cancel_requested = true;
    input.deactivating = true;
    input.nav2_active = true;
    input.config_invalid = true;
    input.curve_frame_ok = false;
    input.inputs_stale = true;
    edit(input);
    return input;
  };
  const Row rows[] = {
    {"deactivation beats everything", make([](SupervisorInput &) {}), SupervisorAction::kAbort,
     result_code::kUnknown},
    {"then cancel", make([](SupervisorInput & in) { in.deactivating = false; }),
     SupervisorAction::kCancel, result_code::kNone},
    {"then a Nav2 action", make([](SupervisorInput & in) {
       in.deactivating = false;
       in.cancel_requested = false;
     }),
     SupervisorAction::kAbort, result_code::kNav2Active},
    {"then an invalid configuration", make([](SupervisorInput & in) {
       in.deactivating = false;
       in.cancel_requested = false;
       in.nav2_active = false;
     }),
     SupervisorAction::kAbort, result_code::kUnknown},
    {"then the curve frame (zero)", make([](SupervisorInput & in) {
       in.deactivating = false;
       in.cancel_requested = false;
       in.nav2_active = false;
       in.config_invalid = false;
     }),
     SupervisorAction::kZero, result_code::kNone},
    {"then stale inputs (decelerate)", make([](SupervisorInput & in) {
       in.deactivating = false;
       in.cancel_requested = false;
       in.nav2_active = false;
       in.config_invalid = false;
       in.curve_frame_ok = true;
     }),
     SupervisorAction::kDecelerate, result_code::kNone},
    {"then heading infeasible", make([](SupervisorInput & in) {
       in.deactivating = false;
       in.cancel_requested = false;
       in.nav2_active = false;
       in.config_invalid = false;
       in.curve_frame_ok = true;
       in.inputs_stale = false;
     }),
     SupervisorAction::kAbort, result_code::kUnsupported},
    {"blocked waits before the A3 timer", make([](SupervisorInput & in) {
       in.deactivating = false;
       in.cancel_requested = false;
       in.nav2_active = false;
       in.config_invalid = false;
       in.curve_frame_ok = true;
       in.inputs_stale = false;
       in.status = kBlocked | kClamped;
     }),
     SupervisorAction::kContinue, result_code::kNone},
  };
  for (const Row & row : rows) {
    GoalSupervisor supervisor(SupervisorConfig{}, 0.0);
    const SupervisorDecision decision = supervisor.update(row.input);
    EXPECT_EQ(decision.action, row.action) << row.name;
    EXPECT_EQ(decision.code, row.code) << row.name;
    EXPECT_NE(decision.reason, nullptr) << row.name;
  }
}

TEST(GoalSupervisor, ImmediateAbortsSendZeroNowAndTimeoutDecelerates)
{
  GoalSupervisor supervisor(SupervisorConfig{}, 0.0);
  SupervisorInput nav2 = ran(0.1);
  nav2.nav2_active = true;
  expectAbort(supervisor.update(nav2), result_code::kNav2Active, false);
  expectAbort(supervisor.update(ran(0.2, vf::Status::kNonFinite)), result_code::kUnknown, false);
  expectAbort(
    supervisor.update(ran(0.3, vf::Status::kContourWatchdog)), result_code::kBlocked, true);

  SupervisorConfig config;
  config.max_duration = 5.0;
  GoalSupervisor timed(config, 10.0);
  EXPECT_EQ(timed.update(ran(14.9, 0U, 0.0)).action, SupervisorAction::kContinue);
  expectAbort(timed.update(ran(15.0, 0U, 1.0)), result_code::kTimeout, true);
}

TEST(GoalSupervisor, HoldsAbortAfterStaleAbortTimeout)
{
  SupervisorConfig config;
  config.stale_abort_timeout = 2.0;
  GoalSupervisor supervisor(config, 0.0);
  SupervisorInput stale;
  stale.inputs_stale = true;
  for (double t = 0.0; t < 1.95; t += 0.05) {
    stale.now = t;
    EXPECT_EQ(supervisor.update(stale).action, SupervisorAction::kDecelerate) << t;
  }
  // Fresh again: the timer restarts.
  EXPECT_EQ(supervisor.update(ran(2.0)).action, SupervisorAction::kContinue);
  stale.now = 2.05;
  EXPECT_EQ(supervisor.update(stale).action, SupervisorAction::kDecelerate);
  stale.now = 4.1;
  expectAbort(supervisor.update(stale), result_code::kStaleInput, true);

  // A stale moving frame is the same hold.
  GoalSupervisor frame(config, 0.0);
  SupervisorInput frame_stale;
  frame_stale.frame_stale = true;
  EXPECT_EQ(frame.update(frame_stale).action, SupervisorAction::kDecelerate);
  frame_stale.now = 2.0;
  expectAbort(frame.update(frame_stale), result_code::kStaleInput, true);

  // The curve frame: zero, then TF_ERROR (no deceleration).
  GoalSupervisor tf(config, 0.0);
  SupervisorInput tf_error;
  tf_error.curve_frame_ok = false;
  EXPECT_EQ(tf.update(tf_error).action, SupervisorAction::kZero);
  tf_error.now = 2.0;
  expectAbort(tf.update(tf_error), result_code::kTfError, false);
}

TEST(GoalSupervisor, BlockedResumesOrAbortsAfterBlockedTimeout)
{
  SupervisorConfig config;
  config.blocked_timeout = 3.0;
  config.progress_timeout = 100.0;
  GoalSupervisor supervisor(config, 0.0);
  // A short block resumes by itself.
  for (double t = 0.0; t < 2.9; t += 0.1) {
    EXPECT_EQ(supervisor.update(ran(t, kBlocked)).action, SupervisorAction::kContinue);
  }
  EXPECT_EQ(supervisor.update(ran(3.0, 0U, 1.0)).action, SupervisorAction::kContinue);
  // A persistent block aborts.
  for (double t = 3.1; t < 6.05; t += 0.1) {
    EXPECT_EQ(supervisor.update(ran(t, kBlocked, 1.0)).action, SupervisorAction::kContinue);
  }
  expectAbort(supervisor.update(ran(6.15, kBlocked, 1.0)), result_code::kBlocked, false);
}

TEST(GoalSupervisor, ProgressWatchdog)
{
  SupervisorConfig config;
  config.progress_radius = 0.5;
  config.progress_timeout = 10.0;
  GoalSupervisor supervisor(config, 0.0);
  // r_p = min(0.5, 0.5 · 0.4 · 10) = 0.5; slow drift of 0.4 m in 9.9 s is no progress.
  EXPECT_DOUBLE_EQ(supervisor.progressRadius(0.4), 0.5);
  EXPECT_DOUBLE_EQ(supervisor.progressRadius(0.05), 0.25);
  for (int k = 0; k <= 99; ++k) {
    EXPECT_EQ(supervisor.update(ran(0.1 * k, 0U, 0.004 * k)).action, SupervisorAction::kContinue);
  }
  expectAbort(supervisor.update(ran(10.0, 0U, 0.4)), result_code::kBlocked, false);

  // Moving more than r_p re-anchors.
  GoalSupervisor moving(config, 0.0);
  for (int k = 0; k <= 300; ++k) {
    EXPECT_EQ(moving.update(ran(0.1 * k, 0U, 0.04 * k)).action, SupervisorAction::kContinue);
  }
  // Holds re-anchor the watchdog.
  GoalSupervisor held(config, 0.0);
  EXPECT_EQ(held.update(ran(0.0)).action, SupervisorAction::kContinue);
  SupervisorInput stale = ran(9.0);
  stale.tracker_ran = false;
  stale.inputs_stale = true;
  EXPECT_EQ(held.update(stale).action, SupervisorAction::kDecelerate);
  EXPECT_EQ(held.update(ran(9.5)).action, SupervisorAction::kContinue);
  EXPECT_EQ(held.update(ran(19.0)).action, SupervisorAction::kContinue);
  expectAbort(held.update(ran(19.5)), result_code::kBlocked, false);
}

TEST(GoalSupervisor, ProgressWatchdogCountsHeight)
{
  // A 3D caller (vectorfield_follower_3d) climbing straight up makes progress.
  SupervisorConfig config;
  config.progress_radius = 0.5;
  config.progress_timeout = 10.0;
  GoalSupervisor climbing(config, 0.0);
  for (int k = 0; k <= 300; ++k) {
    SupervisorInput input = ran(0.1 * k);
    input.position_z = 0.04 * k;
    EXPECT_EQ(climbing.update(input).action, SupervisorAction::kContinue) << k;
  }
  // Hovering at one height does not.
  GoalSupervisor hovering(config, 0.0);
  for (int k = 0; k <= 99; ++k) {
    SupervisorInput input = ran(0.1 * k);
    input.position_z = 2.0;
    EXPECT_EQ(hovering.update(input).action, SupervisorAction::kContinue);
  }
  SupervisorInput last = ran(10.0);
  last.position_z = 2.0;
  expectAbort(hovering.update(last), result_code::kBlocked, false);
}

TEST(GoalSupervisor, AssumptionThreeTimerPausesWhileBlocked)
{
  SupervisorConfig config;
  config.assumption3_abort_time = 1.05;
  config.blocked_timeout = 100.0;
  config.progress_timeout = 100.0;
  GoalSupervisor supervisor(config, 0.0);
  double x = 0.0;
  double t = 0.0;
  auto step = [&](std::uint32_t status) {
    t += 0.1;
    x += 0.04;
    return supervisor.update(ran(t, status, x));
  };
  EXPECT_EQ(step(0U).action, SupervisorAction::kContinue);
  for (int k = 0; k < 5; ++k) {
    EXPECT_EQ(step(kClamped).action, SupervisorAction::kContinue);
  }
  EXPECT_NEAR(supervisor.assumption3Time(), 0.5, 1e-9);
  // Blocked for 5 s with the clamp still on: paused, not reset.
  for (int k = 0; k < 50; ++k) {
    EXPECT_EQ(step(kBlocked | kClamped).action, SupervisorAction::kContinue);
  }
  EXPECT_NEAR(supervisor.assumption3Time(), 0.5, 1e-9);
  for (int k = 0; k < 5; ++k) {
    EXPECT_EQ(step(kClamped).action, SupervisorAction::kContinue);
  }
  expectAbort(step(kClamped), result_code::kAssumption3Violated, true);

  // Clearing the clamp resets the timer.
  GoalSupervisor reset(config, 0.0);
  t = 0.0;
  for (int k = 0; k < 8; ++k) {
    t += 0.1;
    EXPECT_EQ(reset.update(ran(t, kClamped, 0.04 * k)).action, SupervisorAction::kContinue);
  }
  t += 0.1;
  EXPECT_EQ(reset.update(ran(t, 0U, 1.0)).action, SupervisorAction::kContinue);
  EXPECT_EQ(reset.assumption3Time(), 0.0);
}

TEST(GoalSupervisor, RampDownDecelerates)
{
  const vf::Twist2 start{0.4, 0.0, 1.0};
  const vf::Twist2 next = rampDown(start, 0.05, 1.0, 4.0);
  EXPECT_NEAR(next.vx, 0.35, 1e-12);
  EXPECT_NEAR(next.wz, 0.8, 1e-12);
  // Direction of the planar velocity is kept.
  const vf::Twist2 lateral = rampDown(vf::Twist2{0.3, 0.4, -0.5}, 0.1, 1.0, 1.0);
  EXPECT_NEAR(lateral.vx, 0.24, 1e-12);
  EXPECT_NEAR(lateral.vy, 0.32, 1e-12);
  EXPECT_NEAR(lateral.wz, -0.4, 1e-12);
  // Below the thresholds: exactly zero.
  const vf::Twist2 last = rampDown(vf::Twist2{0.05, 0.0, 0.0}, 0.05, 1.0, 1.0);
  EXPECT_EQ(last.vx, 0.0);
  EXPECT_TRUE(belowStopThresholds(last));
  EXPECT_FALSE(belowStopThresholds(vf::Twist2{0.0, 0.0, 0.02}));
  // Invalid limits stop at once.
  EXPECT_EQ(rampDown(start, 0.05, 0.0, 1.0).vx, 0.0);
  int steps = 0;
  vf::Twist2 twist = start;
  while (!belowStopThresholds(twist) && steps < 100) {
    twist = rampDown(twist, 0.05, 1.0, 4.0);
    ++steps;
  }
  EXPECT_EQ(steps, 8);
}

}  // namespace
