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

// Layer 2 (plan §9), through the server harness: goals in free space. The straight approach, goals
// behind the robot, the one-pose rotate goal (plan §5.3), the final approach to a rotated or
// lateral goal for each tracking law (settled before ROTATE_TO_HEADING, math_spec §9.1), and a
// real cusp rejected as InvalidPath (plan §5.1).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <tuple>
#include <vector>

#include "harness_fixtures.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::test::countExceptions;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::describe;
using vectorfield_nav2_controller::test::finalDistance;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::modelOptions;
using vectorfield_nav2_controller::test::reversingPlan;
using vectorfield_nav2_controller::test::ServerHarness;
using vectorfield_nav2_controller::test::straightPlan;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

constexpr double kDeg = vf::kPi / 180.0;

/// Runs a straight plan from the origin to `goal` (yaw `goal_yaw`) to the end.
GoalStatus runStraight(
  ServerHarness & harness, const vf::Vec2 & goal, double goal_yaw, double timeout)
{
  harness.configure();
  harness.activate();
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal, 0.05, goal_yaw)));
  return harness.runUntilDone(timeout);
}

TEST(Approach, FreeSpaceStraightPlanSucceedsWithoutNoValidControl)
{
  ServerHarness harness(modelOptions());
  const vf::Vec2 goal(1.5, 0.0);
  ASSERT_EQ(runStraight(harness, goal, 0.0, 15.0), GoalStatus::kSucceeded) << describe(harness);
  EXPECT_EQ(countExceptions(harness, "NoValidControl"), 0U);
  EXPECT_LE(finalDistance(harness, goal), harness.options().xy_goal_tolerance);
}

class GoalBehind : public ::testing::TestWithParam<double>
{
};

TEST_P(GoalBehind, RotatesInPlaceWithoutReversingAndSucceeds)
{
  const double angle = GetParam() * kDeg;
  ServerHarness harness(modelOptions());
  const vf::Vec2 goal = 1.0 * vf::heading(angle);
  ASSERT_EQ(runStraight(harness, goal, angle, 20.0), GoalStatus::kSucceeded) << describe(harness);
  bool rotated_to_field = false;
  for (const CycleRecord & record : harness.history()) {
    EXPECT_GE(record.published.linear.x, -1e-9) << "v < 0 at t = " << record.t;
    rotated_to_field |=
      record.computed && record.diagnostics.output.phase == vf::Phase::kRotateToField;
  }
  EXPECT_TRUE(rotated_to_field);
  EXPECT_EQ(countExceptions(harness, "NoValidControl"), 0U);
}

INSTANTIATE_TEST_SUITE_P(Layer2, GoalBehind, ::testing::Values(120.0, 180.0));

TEST(OnePoseGoal, RotatesInPlaceAtOnceAndSucceeds)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  const std::vector<vf::Pose2> plan = {vf::Pose2{vf::Vec2::Zero(), 0.5 * vf::kPi}};
  harness.followPath(mapPlan(plan));
  ASSERT_EQ(harness.runUntilDone(10.0), GoalStatus::kSucceeded) << describe(harness);
  std::size_t computed = 0;
  for (const CycleRecord & record : harness.history()) {
    if (!record.computed) {
      continue;
    }
    ++computed;
    EXPECT_EQ(record.diagnostics.window_kind, vectorfield_nav2_controller::WindowKind::kOnePose);
    EXPECT_EQ(record.diagnostics.output.phase, vf::Phase::kRotateToHeading) << "t = " << record.t;
    EXPECT_NEAR(record.published.linear.x, 0.0, 1e-9);
  }
  EXPECT_GT(computed, 0U);
  EXPECT_LT(finalDistance(harness, vf::Vec2::Zero()), 1e-6);
}

/// (tracking law, case): case "yaw60" and "yaw90" approach (1, 0) with the goal yaw 60° / 90° off
/// the approach; "lateral" is a goal 0.3 m to the robot's left with its current yaw.
using FinalApproachCase = std::tuple<std::string, std::string>;

class FinalApproach : public ::testing::TestWithParam<FinalApproachCase>
{
};

TEST_P(FinalApproach, SettlesBeforeRotatingToTheGoalHeading)
{
  const auto & [law, shape] = GetParam();
  HarnessOptions options =
    law == "holonomic" ? modelOptions("omnidirectional", law) : modelOptions("differential", law);
  ServerHarness harness(options);
  vf::Vec2 goal(1.0, 0.0);
  double goal_yaw = 0.0;
  if (shape == "yaw60") {
    goal_yaw = 60.0 * kDeg;
  } else if (shape == "yaw90") {
    goal_yaw = 90.0 * kDeg;
  } else {
    goal = vf::Vec2(0.0, 0.3);
  }
  ASSERT_EQ(runStraight(harness, goal, goal_yaw, 20.0), GoalStatus::kSucceeded)
    << describe(harness);
  EXPECT_EQ(countExceptions(harness, "NoValidControl"), 0U);

  const double xy_tol = options.xy_goal_tolerance;
  const CycleRecord * first_rotation = nullptr;
  for (const CycleRecord & record : harness.history()) {
    if (record.computed && record.diagnostics.output.phase == vf::Phase::kRotateToHeading) {
      first_rotation = &record;
      break;
    }
  }
  ASSERT_NE(first_rotation, nullptr) << "ROTATE_TO_HEADING never started";
  EXPECT_LE((first_rotation->pose_odom.p - goal).norm(), 0.6 * xy_tol)
    << "rotation started at " << first_rotation->pose_odom.p.transpose();
  EXPECT_LE(finalDistance(harness, goal), xy_tol);
}

INSTANTIATE_TEST_SUITE_P(
  Layer2, FinalApproach,
  ::testing::Combine(
    ::testing::Values("feedback_linearization", "heading_tracking", "holonomic"),
    ::testing::Values("yaw60", "yaw90", "lateral")),
  [](const ::testing::TestParamInfo<FinalApproachCase> & case_info) {
    return std::get<0>(case_info.param) + "_" + std::get<1>(case_info.param);
  });

TEST(Cusp, ARealReversalIsRejectedAsInvalidPath)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  harness.followPath(mapPlan(reversingPlan(vf::Pose2{}, 0.6, 0.4, 0.05)));
  EXPECT_EQ(harness.status(), GoalStatus::kAborted);
  EXPECT_EQ(harness.result().reason, "INVALID_PATH") << describe(harness);
  EXPECT_NE(harness.result().message.find("reversing"), std::string::npos) << describe(harness);
  const CycleRecord & record = harness.spinOnce();
  EXPECT_FALSE(record.computed);
  EXPECT_EQ(record.published.linear.x, 0.0);
}

}  // namespace
