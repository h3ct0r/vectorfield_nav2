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

// Layer 2 (plan §9), through the server harness: localization corrections and replans. A map→odom
// step with replan jitter, a settle-then-rotate approach disturbed by map→odom steps and replans
// (math_spec §9.1), a goal end that moves at every replan during a contour episode (goal identity
// keeps the episode and the limiter, math_spec §9.1), consecutive goals (Humble's reset
// emulation, plan §5.1) and Lyrical's skew between the window and the goal (nav2_api_matrix.md
// §4.1).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "harness_fixtures.hpp"
#include "vectorfield_ros/compat.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::WindowKind;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::describe;
using vectorfield_nav2_controller::test::finalDistance;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::modelOptions;
using vectorfield_nav2_controller::test::pluginParameter;
using vectorfield_nav2_controller::test::ServerHarness;
using vectorfield_nav2_controller::test::straightPlan;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

constexpr double kDeg = vf::kPi / 180.0;

/// The robot's pose in the map frame now (the last record's pose is from the start of its cycle).
vf::Pose2 robotInMap(const ServerHarness & harness, const vf::Pose2 & map_to_odom)
{
  const CycleRecord & last = harness.history().back();
  return vf::Isometry2::fromPose(map_to_odom).apply(last.pose_odom);
}

/// Number of cycles that threw anything.
std::size_t countThrows(const ServerHarness & harness)
{
  const std::vector<CycleRecord> & history = harness.history();
  return static_cast<std::size_t>(std::count_if(
    history.begin(), history.end(), [](const CycleRecord & r) { return !r.exception.empty(); }));
}

TEST(Localization, AMapToOdomStepAndReplanJitterNeverThrowAndSucceed)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  const vf::Vec2 goal(2.0, 0.0);
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal, 0.05, 0.0)));

  // A 0.2 m / 5° correction after 1 s; replans every 0.5 s from the robot, whose start the
  // planner puts up to 3 cm to either side.
  const vf::Pose2 step{vf::Vec2(0.12, 0.16), 5.0 * kDeg};
  const std::array<double, 4> jitter = {0.03, -0.03, 0.015, -0.015};
  const double start = harness.now();
  vf::Pose2 map_to_odom;
  bool stepped = false;
  double next_replan = start + 0.5;
  std::size_t replans = 0;
  const GoalStatus status = harness.runUntilDone(20.0, [&](const CycleRecord &) {
    if (!stepped && harness.now() >= start + 1.0 - 1e-9) {
      map_to_odom = step;
      harness.setMapToOdom(map_to_odom);
      stepped = true;
    }
    if (harness.now() >= next_replan - 1e-9) {
      const vf::Pose2 robot = robotInMap(harness, map_to_odom);
      const vf::Vec2 normal = vf::heading(vf::angleOf(goal - robot.p) + 0.5 * vf::kPi);
      const vf::Vec2 from = robot.p + jitter[replans % jitter.size()] * normal;
      harness.replan(mapPlan(straightPlan(from, goal, 0.05, 0.0)));
      ++replans;
      next_replan += 0.5;
    }
  });
  ASSERT_EQ(status, GoalStatus::kSucceeded) << describe(harness);
  EXPECT_TRUE(stepped);
  EXPECT_GE(replans, 4U);
  EXPECT_EQ(countThrows(harness), 0U);
  EXPECT_LE(
    (harness.history().back().pose_map.p - goal).norm(), harness.options().xy_goal_tolerance);
}

class SettleThenRotate : public ::testing::TestWithParam<std::string>
{
};

TEST_P(SettleThenRotate, MapToOdomStepsAndReplansNeverUndoTheRotation)
{
  const std::string law = GetParam();
  HarnessOptions options =
    law == "holonomic" ? modelOptions("omnidirectional", law) : modelOptions("differential", law);
  // The robot stopped 0.24 m before the goal on its approach: inside xy_tol, outside σ_stop.
  options.initial_pose = vf::Pose2{vf::Vec2(0.76, 0.0), 0.0};
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  const vf::Pose2 goal{vf::Vec2(1.0, 0.0), 90.0 * kDeg};
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal.p, 0.05, goal.yaw)));

  // A 3 cm map→odom shift and a replan from the robot after 3 cycles; then, 0.5 s into
  // ROTATE_TO_HEADING (the residual approach speed has decayed), three more 3 cm steps, each with
  // a replan, 0.2 s apart.
  vf::Pose2 map_to_odom;
  const auto shiftAndReplan = [&]() {
    map_to_odom.p.y() += 0.03;
    harness.setMapToOdom(map_to_odom);
    const vf::Pose2 robot = robotInMap(harness, map_to_odom);
    harness.replan(mapPlan(straightPlan(robot.p, goal.p, 0.05, goal.yaw)));
  };
  std::size_t cycles = 0;
  std::size_t steps = 0;
  double next_step = -1.0;
  const GoalStatus status = harness.runUntilDone(15.0, [&](const CycleRecord & record) {
    if (++cycles == 3) {
      shiftAndReplan();
    }
    const bool rotating =
      record.computed && record.diagnostics.output.phase == vf::Phase::kRotateToHeading;
    if (rotating && next_step < 0.0) {
      next_step = harness.now() + 0.5;
    }
    if (next_step > 0.0 && steps < 3 && harness.now() >= next_step - 1e-9) {
      shiftAndReplan();
      ++steps;
      next_step += 0.2;
    }
  });
  ASSERT_EQ(status, GoalStatus::kSucceeded) << describe(harness);
  EXPECT_EQ(steps, 3U) << "the goal ended before the three steps";
  EXPECT_EQ(countThrows(harness), 0U);

  // No translation while rotating: the rotation starts with the approach's residual speed, which
  // the limiter (all axes on one line, math_spec §8.4) only brings down; once zero it stays zero
  // through every step and replan.
  const double xy_tol = options.xy_goal_tolerance;
  const double period = 1.0 / options.controller_frequency;
  bool rotating = false;
  bool stopped = false;
  double previous_speed = vf::kInf;
  double travelled = 0.0;
  for (const CycleRecord & record : harness.history()) {
    if (!record.computed) {
      continue;
    }
    const vf::Phase phase = record.diagnostics.output.phase;
    if (!rotating && phase == vf::Phase::kRotateToHeading) {
      rotating = true;
      EXPECT_LE((record.pose_map.p - goal.p).norm(), 0.6 * xy_tol)
        << "rotation started at " << record.pose_map.p.transpose();
    }
    if (!rotating) {
      continue;
    }
    EXPECT_NE(phase, vf::Phase::kFollow) << "FOLLOW re-entered at t = " << record.t;
    if (phase == vf::Phase::kRotateToHeading) {
      const double speed = std::hypot(record.published.linear.x, record.published.linear.y);
      EXPECT_LE(speed, previous_speed + 1e-12) << "translation grew at t = " << record.t;
      if (stopped) {
        EXPECT_EQ(speed, 0.0) << "translation resumed at t = " << record.t;
      }
      stopped |= speed == 0.0;
      previous_speed = speed;
      travelled += speed * period;
    }
  }
  EXPECT_TRUE(rotating) << "ROTATE_TO_HEADING never started";
  EXPECT_TRUE(stopped) << "the linear command never reached zero while rotating";
  EXPECT_LE(travelled, 0.03);
}

INSTANTIATE_TEST_SUITE_P(
  Layer2, SettleThenRotate,
  ::testing::Values("feedback_linearization", "heading_tracking", "holonomic"));

TEST(MovingGoal, AGoalEndMovingAtEveryReplanKeepsTheContourEpisodeAndTheSpeed)
{
  HarnessOptions options = modelOptions();
  options.parameters.push_back(pluginParameter(options, "obstacle_mode", std::string("contour")));
  options.initial_pose = vf::Pose2{vf::Vec2(-1.0, 0.0), 0.0};
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  // A box across the straight line from the start to every goal.
  harness.setLethalBox(vf::Vec2(0.30, -0.25), vf::Vec2(0.50, 0.25));
  const vf::Vec2 start = options.initial_pose.p;
  const auto goalAt = [](std::size_t k) { return vf::Vec2(1.8, 0.05 * static_cast<double>(k)); };
  std::size_t moves = 0;
  harness.followPath(mapPlan(straightPlan(start, goalAt(0), 0.05, 0.0)));

  // The goal end moves 5 cm per plan at 1 Hz, eight times.
  double next_replan = harness.now() + 1.0;
  std::vector<double> replan_times;
  const GoalStatus status = harness.runUntilDone(40.0, [&](const CycleRecord &) {
    if (moves < 8 && harness.now() >= next_replan - 1e-9) {
      ++moves;
      harness.replan(mapPlan(straightPlan(start, goalAt(moves), 0.05, 0.0)));
      replan_times.push_back(harness.now());
      next_replan += 1.0;
    }
  });
  ASSERT_EQ(status, GoalStatus::kSucceeded) << describe(harness);
  EXPECT_EQ(countThrows(harness), 0U);
  EXPECT_LE(
    (harness.history().back().pose_map.p - goalAt(moves)).norm(),
    harness.options().xy_goal_tolerance);

  // σ_c: one episode, whose side never changes, spanning at least two goal moves.
  const double a_dec = harness.controller()->appliedConfig().motion.max_linear_decel;
  const double period = 1.0 / options.controller_frequency;
  std::int8_t sigma_c = 0;
  std::size_t episodes = 0;
  bool active = false;
  double episode_start = 0.0;
  double episode_end = 0.0;
  const CycleRecord * previous = nullptr;
  for (const CycleRecord & record : harness.history()) {
    if (!record.computed) {
      continue;
    }
    const auto & contour = record.diagnostics.state.contour;
    if (contour.active) {
      if (!active) {
        ++episodes;
        episode_start = record.t;
      }
      episode_end = record.t;
      if (sigma_c == 0) {
        sigma_c = contour.sigma_c;
      }
      EXPECT_EQ(contour.sigma_c, sigma_c) << "σ_c changed at t = " << record.t;
    }
    active = contour.active;
    // The limiter is never re-seeded: the speed drops by at most one deceleration step.
    if (previous != nullptr) {
      EXPECT_GE(record.published.linear.x, previous->published.linear.x - a_dec * period - 1e-9)
        << "speed drop at t = " << record.t;
    }
    previous = &record;
  }
  EXPECT_EQ(episodes, 1U);
  EXPECT_NE(sigma_c, 0);
  const auto during = std::count_if(replan_times.begin(), replan_times.end(), [&](double t) {
    return t >= episode_start && t < episode_end;
  });
  EXPECT_GE(during, 2) << "episode [" << episode_start << ", " << episode_end << "]";
}

TEST(ConsecutiveGoals, AGoalRightAfterAnotherAndOneAfterAPauseSucceed)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  const double xy_tol = harness.options().xy_goal_tolerance;
  const auto robot = [&]() { return harness.history().back().pose_odom.p; };

  // 1. Ahead.
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(1.0, 0.0), 0.05, 0.0)));
  ASSERT_EQ(harness.runUntilDone(10.0), GoalStatus::kSucceeded) << describe(harness);
  const std::size_t first_end = harness.history().size();

  // 2. At once, to the left (Humble: no reset from the server and none due in setPlan).
  const vf::Vec2 second(1.0, 0.8);
  harness.followPath(mapPlan(straightPlan(robot(), second, 0.05, 0.5 * vf::kPi)));
  ASSERT_EQ(harness.runUntilDone(15.0), GoalStatus::kSucceeded) << describe(harness);
  EXPECT_LE(finalDistance(harness, second), xy_tol);
  const std::size_t second_end = harness.history().size();

  // 3. After a pause of 0.5 s (Humble: the reset emulation runs in setPlan).
  for (int i = 0; i < 10; ++i) {
    harness.spinOnce();
  }
  const vf::Vec2 third(0.0, 0.8);
  harness.followPath(mapPlan(straightPlan(robot(), third, 0.05, vf::kPi)));
  ASSERT_EQ(harness.runUntilDone(15.0), GoalStatus::kSucceeded) << describe(harness);
  EXPECT_LE(finalDistance(harness, third), xy_tol);
  EXPECT_EQ(countThrows(harness), 0U);

  // Each goal starts with its goal-scoped state reset: no goal phase carried over.
  for (const std::size_t begin : {first_end, second_end + 10}) {
    const CycleRecord & first = harness.history()[begin];
    ASSERT_TRUE(first.computed);
    EXPECT_NE(first.diagnostics.output.phase, vf::Phase::kRotateToHeading);
    EXPECT_NE(first.diagnostics.output.phase, vf::Phase::kStopped);
  }
}

class GoalSkew : public ::testing::TestWithParam<double>
{
};

TEST_P(GoalSkew, ASkewBetweenTheWindowAndTheGoalDoesNotFlicker)
{
  if (vectorfield_ros::compat::kApiGeneration != 'C') {
    GTEST_SKIP() << "only Gen C reads the window and the goal at different stamps";
  }
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  harness.setGoalSkew(vf::Pose2{vf::Vec2(0.0, GetParam()), 0.0});
  const vf::Vec2 goal(1.5, 0.0);
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal, 0.05, 0.5 * vf::kPi)));
  ASSERT_EQ(harness.runUntilDone(20.0), GoalStatus::kSucceeded) << describe(harness);
  EXPECT_EQ(countThrows(harness), 0U);

  // Latches only move forward: the tail stays conditioned, terminal mode stays on, and the phase
  // never steps back once the final approach started.
  bool tail = false;
  bool terminal = false;
  int rank = 0;
  for (const CycleRecord & record : harness.history()) {
    if (!record.computed) {
      continue;
    }
    const auto & diagnostics = record.diagnostics;
    if (tail) {
      EXPECT_EQ(diagnostics.window_kind, WindowKind::kTailConditioned) << "t = " << record.t;
    }
    tail |= diagnostics.window_kind == WindowKind::kTailConditioned;
    if (terminal) {
      EXPECT_TRUE(diagnostics.state.terminal_mode) << "t = " << record.t;
    }
    terminal |= diagnostics.state.terminal_mode;
    const vf::Phase phase = diagnostics.output.phase;
    const int phase_rank = phase == vf::Phase::kRotateToHeading ? 2
                           : phase == vf::Phase::kFinalApproach ? 1
                                                                : 0;
    EXPECT_GE(phase_rank, rank) << "phase " << vf::phaseName(phase) << " at t = " << record.t;
    rank = std::max(rank, phase_rank);
  }
  EXPECT_TRUE(tail);
  EXPECT_TRUE(terminal);
  EXPECT_EQ(rank, 2);
}

INSTANTIATE_TEST_SUITE_P(Layer2, GoalSkew, ::testing::Values(0.0, 0.01, 0.02, 0.03));

}  // namespace
