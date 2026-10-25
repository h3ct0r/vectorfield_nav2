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

// Layer 2, through the server harness: goal checkers other than SimpleGoalChecker.
// Ackermann with PositionGoalChecker and failure_tolerance 0 (its identity orientation means "yaw
// unchecked"; Humble computes once more on arrival, nav2_api_matrix.md §4.1) and AxisGoalChecker
// final approaches (Lyrical and later). The rotation shim has its own binary: Jazzy's shim library
// links PositionGoalChecker's, and class_loader cannot create a plugin whose library another
// plugin's library already loaded.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <tuple>

#include "harness_fixtures.hpp"
#include "vectorfield_ros/compat.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace compat = vectorfield_ros::compat;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::describe;
using vectorfield_nav2_controller::test::finalDistance;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::modelOptions;
using vectorfield_nav2_controller::test::ServerHarness;
using vectorfield_nav2_controller::test::straightPlan;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

/// Every cycle threw nothing.
void expectNoThrow(const ServerHarness & harness)
{
  for (const CycleRecord & record : harness.history()) {
    EXPECT_TRUE(record.exception.empty()) << "t = " << record.t << ": " << record.message;
  }
}

/// Ackermann goals: "straight" 1.5 m ahead, "offset" 1.5 m ahead and 0.3 m to the left.
class AckermannPositionChecker : public ::testing::TestWithParam<std::string>
{
};

TEST_P(AckermannPositionChecker, SucceedsWithoutPatience)
{
  HarnessOptions options = modelOptions("ackermann");
  options.goal_checker = "nav2_controller::PositionGoalChecker";
  ASSERT_EQ(options.failure_tolerance, 0.0);
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  const vf::Vec2 goal = GetParam() == "straight" ? vf::Vec2(1.5, 0.0) : vf::Vec2(1.5, 0.3);
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal, 0.05, 0.0)));
  ASSERT_EQ(harness.runUntilDone(20.0), GoalStatus::kSucceeded) << describe(harness);
  expectNoThrow(harness);
  EXPECT_LE(finalDistance(harness, goal), options.xy_goal_tolerance);
  // The last compute (Humble: in the cycle that reaches the goal) returned a sane command.
  for (const CycleRecord & record : harness.history()) {
    if (record.computed) {
      EXPECT_GE(record.published.linear.x, -1e-9) << "t = " << record.t;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Layer2, AckermannPositionChecker, ::testing::Values("straight", "offset"));

/// (tracking law, case): "straight" 1.5 m ahead, "lateral" 0.3 m to the left.
using AxisCase = std::tuple<std::string, std::string>;

class AxisChecker : public ::testing::TestWithParam<AxisCase>
{
};

TEST_P(AxisChecker, FinalApproachEndsWithinTheAlongAndCrossTolerances)
{
  if (compat::kApiGeneration != 'C') {
    GTEST_SKIP() << "AxisGoalChecker exists on Lyrical and later";
  }
  const auto & [law, shape] = GetParam();
  HarnessOptions options =
    law == "holonomic" ? modelOptions("omnidirectional", law) : modelOptions("differential", law);
  options.goal_checker = "nav2_controller::AxisGoalChecker";
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  const vf::Vec2 goal = shape == "straight" ? vf::Vec2(1.5, 0.0) : vf::Vec2(0.0, 0.3);
  const double yaw = vf::angleOf(goal);
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal, 0.05, yaw)));
  ASSERT_EQ(harness.runUntilDone(20.0), GoalStatus::kSucceeded) << describe(harness);
  expectNoThrow(harness);
  // along_path_tolerance and cross_track_tolerance default to 0.25 m, the axis is the plan's end.
  const vf::Vec2 error = harness.history().back().pose_odom.p - goal;
  const vf::Vec2 axis = vf::heading(yaw);
  EXPECT_LT(std::abs(error.dot(axis)), 0.25);
  EXPECT_LT(std::abs(vf::cross2(axis, error)), 0.25);
}

INSTANTIATE_TEST_SUITE_P(
  Layer2, AxisChecker,
  ::testing::Combine(
    ::testing::Values("feedback_linearization", "heading_tracking", "holonomic"),
    ::testing::Values("straight", "lateral")),
  [](const ::testing::TestParamInfo<AxisCase> & case_info) {
    return std::get<0>(case_info.param) + "_" + std::get<1>(case_info.param);
  });

}  // namespace
