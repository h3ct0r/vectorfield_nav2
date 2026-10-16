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

// Layer 2 (plan §9), through the server harness: the footprint against walls. In-place rotations
// beside a wall (the rollout checks the swept rotation; kBlocked holds while blocked, plan §5.2
// step 6), a footprint changed mid-run through setRobotFootprintPolygon (the transactional rebuild
// of plan §4: auto λ and the prediction follow it, an explicit λ that now breaks its rule stops the
// robot), and the goal veto (a binding cap inside xy_tol settles the robot, math_spec §9.2
// "Precedence").

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "harness_fixtures.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/status.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::describe;
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

/// A `length` × `width` rectangle centred on base_link.
std::vector<geometry_msgs::msg::Point> rectangle(double length, double width)
{
  constexpr std::array<std::array<double, 2>, 4> kCorners = {{
    {0.5, 0.5},
    {-0.5, 0.5},
    {-0.5, -0.5},
    {0.5, -0.5},
  }};
  std::vector<geometry_msgs::msg::Point> polygon;
  for (const auto & corner : kCorners) {
    geometry_msgs::msg::Point point;
    point.x = corner[0] * length;
    point.y = corner[1] * width;
    polygon.push_back(point);
  }
  return polygon;
}

/// The auto λ of a resolved configuration (math_spec §11).
double autoLambda(const vf::CoreConfig & config)
{
  return vf::autoContourDistance(
    vf::computeFootprintRadii(config.node.footprint), vf::evaluationOffset(config));
}

TEST(WallRotation, AHalfTurnThreeTenthsFromAWallSucceedsInPlace)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  // The wall's edge is 0.3 m to the robot's left; the padded 0.22 m disc sweeps nothing new.
  harness.setLethalBox(vf::Vec2(-1.0, 0.3), vf::Vec2(1.0, 0.6));
  harness.followPath(mapPlan({vf::Pose2{vf::Vec2::Zero(), vf::kPi}}));
  ASSERT_EQ(harness.runUntilDone(15.0), GoalStatus::kSucceeded) << describe(harness);
  for (const CycleRecord & record : harness.history()) {
    EXPECT_TRUE(record.exception.empty()) << "t = " << record.t << ": " << record.message;
    EXPECT_NEAR(record.published.linear.x, 0.0, 1e-9);
  }
  EXPECT_LT(harness.history().back().pose_odom.p.norm(), 1e-9);
}

TEST(WallRotation, ARectangleBesideAWallIsBlockedFromTurningAndNeverMoves)
{
  HarnessOptions options = modelOptions();
  options.footprint = rectangle(0.6, 0.4);
  options.failure_tolerance = 0.5;
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  // 9 cm beside the padded long side; a quarter turn sweeps the corners out to 0.37 m.
  harness.setLethalBox(vf::Vec2(-1.0, 0.28), vf::Vec2(1.0, 0.6));
  harness.followPath(mapPlan({vf::Pose2{vf::Vec2::Zero(), 90.0 * kDeg}}));
  ASSERT_EQ(harness.runUntilDone(5.0), GoalStatus::kAborted) << describe(harness);
  EXPECT_EQ(harness.result().reason, "PATIENCE_EXCEEDED") << describe(harness);
  std::size_t computed = 0;
  for (const CycleRecord & record : harness.history()) {
    EXPECT_LT(record.pose_odom.p.norm(), 1e-12);
    EXPECT_EQ(record.pose_odom.yaw, 0.0);
    if (!record.computed) {
      continue;
    }
    ++computed;
    // kBlocked stays set while blocked: every cycle, not only the first.
    EXPECT_NE(record.diagnostics.output.status & vf::Status::kBlocked, 0U) << "t = " << record.t;
    EXPECT_EQ(record.exception, "NoValidControl") << "t = " << record.t;
    EXPECT_NE(record.message.find("BLOCKED"), std::string::npos) << record.message;
    EXPECT_EQ(record.published.linear.x, 0.0);
    EXPECT_EQ(record.published.angular.z, 0.0);
  }
  EXPECT_GE(computed, 10U);
}

TEST(FootprintChange, AutoLambdaAndThePredictionFollowTheNewFootprint)
{
  HarnessOptions options = modelOptions();
  options.robot_radius = 0.10;
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  // A corridor 0.3 m wide from x = 0.9: open to the 0.11 m padded disc, closed to a 0.42 m
  // wide padded rectangle.
  harness.setLethalBox(vf::Vec2(0.9, 0.15), vf::Vec2(2.4, 0.6));
  harness.setLethalBox(vf::Vec2(0.9, -0.6), vf::Vec2(2.4, -0.15));
  const vf::CoreConfig before = harness.controller()->appliedConfig();
  EXPECT_NEAR(before.obstacles.obstacle_contour_distance, autoLambda(before), 1e-12);
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(2.0, 0.0), 0.05, 0.0)));

  // Until the robot is 0.45 m along, the disc's rollout reaches into the corridor freely.
  while (harness.status() == GoalStatus::kRunning && harness.history().size() < 200 &&
         (harness.history().empty() || harness.history().back().pose_odom.p.x() < 0.45)) {
    const CycleRecord & record = harness.spinOnce();
    ASSERT_TRUE(record.exception.empty()) << record.message;
    EXPECT_EQ(record.diagnostics.output.first_collision, -1) << "t = " << record.t;
  }
  ASSERT_EQ(harness.status(), GoalStatus::kRunning) << describe(harness);

  harness.setFootprint(rectangle(0.6, 0.4));
  bool predicted = false;
  for (int i = 0; i < 3 && !predicted; ++i) {
    const CycleRecord & record = harness.spinOnce();
    ASSERT_TRUE(record.computed) << describe(harness);
    predicted = record.diagnostics.output.first_collision >= 0;
  }
  EXPECT_TRUE(predicted) << "the rollout did not use the rectangle";
  const vf::CoreConfig after = harness.controller()->appliedConfig();
  EXPECT_EQ(after.node.footprint.polygon.size(), 4U);
  EXPECT_NEAR(after.obstacles.obstacle_contour_distance, autoLambda(after), 1e-12);
  EXPECT_GT(after.obstacles.obstacle_contour_distance, before.obstacles.obstacle_contour_distance);
  EXPECT_FALSE(harness.history().back().diagnostics.config_invalid);
}

TEST(FootprintChange, AnExplicitLambdaThatNoLongerFitsStopsTheRobot)
{
  HarnessOptions options = modelOptions();
  options.robot_radius = 0.10;
  options.failure_tolerance = 0.5;
  // λ − d = 0.16 m: above the disc's inscribed radius, below the rectangle's 0.21 m.
  options.parameters.push_back(pluginParameter(options, "obstacle_mode", std::string("contour")));
  options.parameters.push_back(pluginParameter(options, "obstacle_contour_distance", 0.36));
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(2.0, 0.0), 0.05, 0.0)));
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(harness.spinOnce().exception.empty()) << describe(harness);
  }
  harness.setFootprint(rectangle(0.6, 0.4));
  const std::size_t changed = harness.history().size();
  ASSERT_EQ(harness.runUntilDone(5.0), GoalStatus::kAborted) << describe(harness);
  EXPECT_EQ(harness.result().reason, "PATIENCE_EXCEEDED") << describe(harness);

  const vf::Pose2 stop = harness.history()[changed + 1].pose_odom;
  for (std::size_t i = changed; i < harness.history().size(); ++i) {
    const CycleRecord & record = harness.history()[i];
    ASSERT_TRUE(record.computed);
    EXPECT_EQ(record.exception, "NoValidControl") << "t = " << record.t;
    EXPECT_NE(record.message.find("obstacle_contour_distance"), std::string::npos)
      << record.message;
    EXPECT_TRUE(record.diagnostics.config_invalid);
    EXPECT_EQ(record.published.linear.x, 0.0);
    EXPECT_EQ(record.published.angular.z, 0.0);
    if (i > changed) {
      EXPECT_LT((record.pose_odom.p - stop.p).norm(), 1e-12) << "moved at t = " << record.t;
    }
  }
  // The rejected footprint is not applied: the Tracker keeps the disc.
  EXPECT_NE(harness.controller()->appliedConfig().node.footprint.polygon.size(), 4U);
}

TEST(GoalVeto, ACapThatStopsTheRobotInsideTheToleranceSettlesIt)
{
  HarnessOptions options = modelOptions();
  ASSERT_EQ(options.failure_tolerance, 0.0);
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  // A wall 0.1 m past the goal: the padded disc stops short of σ_stop 0.125 (the approach alone
  // settles at about 0.12 m), still inside xy_tol 0.25.
  harness.setLethalBox(vf::Vec2(1.10, -0.6), vf::Vec2(1.4, 0.6));
  const vf::Pose2 goal{vf::Vec2(1.0, 0.0), 90.0 * kDeg};
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal.p, 0.05, goal.yaw)));
  ASSERT_EQ(harness.runUntilDone(15.0), GoalStatus::kSucceeded) << describe(harness);

  const double xy_tol = options.xy_goal_tolerance;
  const CycleRecord * first_rotation = nullptr;
  for (const CycleRecord & record : harness.history()) {
    EXPECT_TRUE(record.exception.empty()) << "t = " << record.t << ": " << record.message;
    if (
      first_rotation == nullptr && record.computed &&
      record.diagnostics.output.phase == vf::Phase::kRotateToHeading) {
      first_rotation = &record;
    }
  }
  ASSERT_NE(first_rotation, nullptr) << "ROTATE_TO_HEADING never started";
  const double distance = (first_rotation->pose_odom.p - goal.p).norm();
  EXPECT_GT(distance, 0.5 * xy_tol) << "the wall did not veto the approach";
  EXPECT_LE(distance, xy_tol);
}

}  // namespace
