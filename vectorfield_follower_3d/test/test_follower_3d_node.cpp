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

// The 3D follower in process (test/follower_3d_harness.hpp): goal validation, an open helix,
// laps, a cancel, a frame moving through Odometry, the CBF hold → BLOCKED, the configuration
// checks and the body-frame output.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "follower_3d_harness.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"

namespace vectorfield_follower_3d_test
{
namespace
{

namespace vf = vectorfield_core;
namespace rc = vectorfield_curve_follower::result_code;
using Curve3 = vectorfield_msgs::msg::Curve3;
using rclcpp_action::ResultCode;

FollowCurve3::Goal builtin(
  const std::string & shape, const std::vector<std::string> & names,
  const std::vector<double> & values, std::uint32_t laps = 0)
{
  FollowCurve3::Goal goal;
  goal.curve.type = Curve3::BUILTIN;
  goal.curve.builtin_shape = shape;
  goal.curve.parameter_names = names;
  goal.curve.parameter_values = values;
  goal.laps = laps;
  return goal;
}

d3::Pose3 at(double x, double y, double z, double yaw = 0.0)
{
  d3::Pose3 pose;
  pose.p = d3::Vec3(x, y, z);
  pose.yaw = yaw;
  return pose;
}

TEST_F(Follower3DTest, InvalidGoalEndsWithoutACommand)
{
  start(RobotSetup{at(0.0, 0.0, 1.0), {}});
  const auto result = run(builtin("no_such_shape", {}, {}), 2.0);
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->error_code, rc::kInvalidCurve) << result.result->error_msg;
  const auto laps_on_open = run(builtin("helix", {"turns"}, {1.0}, 2), 2.0);
  ASSERT_TRUE(laps_on_open.result);
  EXPECT_EQ(laps_on_open.result->error_code, rc::kInvalidCurve);
  EXPECT_EQ(robot_->commands(), 0U);
  expectSilence(0.5);
}

TEST_F(Follower3DTest, OpenHelixSucceedsNearItsEnd)
{
  start(RobotSetup{at(0.7, -0.1, 0.1), {}});
  // One turn of radius 0.5 rising 0.4: about 3.2 m.
  const auto result = run(builtin("helix", {"radius", "lead", "turns"}, {0.5, 0.4, 1.0}), 30.0);
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->error_code, rc::kNone) << result.result->error_msg;
  const d3::Vec3 end(0.5, 0.0, 0.4);
  EXPECT_LT((robot_->pose().p - end).norm(), 0.1) << robot_->pose().p.transpose();
  EXPECT_GT(result.result->distance_travelled, 2.8);
  EXPECT_EQ(robot_->trailingZeros(), 1U) << "exactly one zero at the end";
  expectSilence();
}

TEST_F(Follower3DTest, LapsOnATiltedCircle)
{
  start(RobotSetup{at(0.6, 0.0, 1.0), {}});
  // A circle of radius 0.5 at z = 1, pitched by 0.4 rad: 2 laps at 0.5 m/s.
  FollowCurve3::Goal goal = builtin("circle", {"radius", "cz", "pitch"}, {0.5, 1.0, 0.4}, 2);
  const auto handle = send(goal);
  ASSERT_TRUE(handle);
  const auto future = resultOf(handle);
  double max_after_lap_1 = 0.0;
  const bool ended = tickUntil(
    [&]() {
      const CycleSnapshot3 s = follower_->lastCycle();
      if (s.laps_completed >= 1 && s.tracker_ran && s.executing) {
        max_after_lap_1 = std::max(max_after_lap_1, s.distance);
      }
      return ready(future) || !s.executing;
    },
    40.0);
  ASSERT_TRUE(ended);
  const auto result = waitResult(future, 1.0);
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->laps_completed, 2U);
  EXPECT_LT(max_after_lap_1, 0.05);
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(Follower3DTest, CancelDeceleratesThenOneZeroThenSilence)
{
  start(RobotSetup{at(1.0, 0.0, 1.0), {}});
  const auto handle = send(builtin("circle", {"cz"}, {1.0}));
  ASSERT_TRUE(handle);
  tickFor(4.0);
  const std::size_t before = robot_->commands();
  ASSERT_GT(robot_->log().back().v.norm(), 0.3);
  cancel(handle);
  const auto result = waitResult(resultOf(handle), 5.0);
  EXPECT_EQ(result.code, ResultCode::CANCELED);
  const auto log = robot_->log();
  ASSERT_GT(log.size(), before);
  for (std::size_t k = before + 1; k < log.size(); ++k) {
    EXPECT_LE(log[k].v.norm(), log[k - 1].v.norm() + 1e-9) << "command " << k << " speeds up";
  }
  EXPECT_TRUE(isZero(log.back()));
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(Follower3DTest, HoldBetweenGoalsStreamsZeroSetpoints)
{
  start(RobotSetup{at(0.7, -0.1, 0.1), {rclcpp::Parameter("hold_between_goals", true)}});
  // Before the first goal: a zero setpoint every control period (20 Hz). On simulated time a
  // timer can skip periods when /clock updates arrive together (a loaded machine), so this only
  // asks for a steady stream, well above PX4's 2 Hz offboard minimum.
  const std::size_t idle_start = robot_->commands();
  tickFor(1.0);
  const std::size_t idle = robot_->commands() - idle_start;
  EXPECT_GE(idle, 5U);
  EXPECT_LE(idle, 21U);
  for (const d3::Twist3 & command : robot_->log()) {
    EXPECT_TRUE(isZero(command));
  }
  // The goal runs as without the hold, and no zero lands among its commands.
  const std::size_t goal_start = robot_->commands();
  const auto result = run(builtin("helix", {"radius", "lead", "turns"}, {0.5, 0.4, 1.0}), 30.0);
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  const auto log = robot_->log();
  std::size_t first = log.size();
  std::size_t last = 0;
  for (std::size_t k = goal_start; k < log.size(); ++k) {
    if (!isZero(log[k])) {
      first = std::min(first, k);
      last = k;
    }
  }
  ASSERT_LT(first, log.size()) << "the goal sent no motion";
  for (std::size_t k = first; k <= last; ++k) {
    EXPECT_FALSE(isZero(log[k])) << "a zero among the goal's commands at " << k;
  }
  // Afterwards the zeros resume.
  const std::size_t after = robot_->commands();
  tickFor(1.0);
  EXPECT_GE(robot_->commands() - after, 5U);
  EXPECT_TRUE(isZero(robot_->log().back()));
}

TEST_F(Follower3DTest, FrameMovingThroughOdometry)
{
  start(RobotSetup{at(0.7, 0.0, 0.0), {}});
  MovingFrame frame;
  frame.topic = "platform/odom";
  frame.child_frame_id = "platform";
  frame.twist.v = d3::Vec3(0.1, 0.0, 0.05);
  frame.twist.w = d3::Vec3(0.0, 0.0, 0.1);
  frame.start_ns = t_ns_;
  robot_->startMovingFrame(frame);
  FollowCurve3::Goal goal = builtin("circle", {"radius"}, {0.6});
  goal.curve.header.frame_id = "platform";
  goal.frame_motion = FollowCurve3::Goal::FRAME_MOVING;
  goal.frame_odom_topic = "platform/odom";
  const auto handle = send(goal);
  ASSERT_TRUE(handle);
  tickFor(10.0);
  double worst = 0.0;
  for (int k = 0; k < 80 && !HasFatalFailure(); ++k) {
    tick();
    const CycleSnapshot3 s = follower_->lastCycle();
    ASSERT_TRUE(s.executing) << describe(s);
    ASSERT_TRUE(s.tracker_ran) << describe(s);
    // The true distance to the moving circle, from the frame's exact pose.
    const d3::Isometry3 T = robot_->framePoseAt(t_ns_);
    const d3::Vec3 local = T.inverse() * robot_->pose().p;
    const double distance = std::hypot(std::hypot(local.x(), local.y()) - 0.6, local.z());
    worst = std::max(worst, distance);
  }
  EXPECT_LT(worst, 0.05) << "the robot rides with the circle";
  EXPECT_GT(robot_->pose().p.z(), 0.5) << "the frame rose 0.05 m/s for over 14 s";
  cancel(handle);
  EXPECT_EQ(waitResult(resultOf(handle), 5.0).code, ResultCode::CANCELED);
  expectSilence();
}

TEST_F(Follower3DTest, CentredSphereHoldsThenBlocked)
{
  start(
    RobotSetup{
      at(0.0, 0.0, 1.0),
      {rclcpp::Parameter("obstacle_mode", "cbf"), rclcpp::Parameter("body_radius", 0.2),
       rclcpp::Parameter("blocked_timeout", 2.0),
       rclcpp::Parameter("obstacles", std::vector<std::string>{"sphere 2 0 1 0.3"})}});
  FollowCurve3::Goal goal;
  goal.curve.type = Curve3::POLYLINE;
  goal.curve.points.resize(2);
  goal.curve.points[0].z = 1.0;
  goal.curve.points[1].x = 4.0;
  goal.curve.points[1].z = 1.0;
  const auto handle = send(goal);
  ASSERT_TRUE(handle);
  const auto future = resultOf(handle);
  double least = 1e9;
  bool blocked = false;
  const bool ended = tickUntil(
    [&]() {
      const double clearance = (robot_->pose().p - d3::Vec3(2.0, 0.0, 1.0)).norm() - 0.3 - 0.2;
      least = std::min(least, clearance);
      const CycleSnapshot3 s = follower_->lastCycle();
      blocked = blocked || vf::Status::has(s.status, vf::Status::kBlocked);
      return ready(future) || !s.executing;
    },
    30.0);
  ASSERT_TRUE(ended);
  const auto result = waitResult(future, 1.0);
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->error_code, rc::kBlocked) << result.result->error_msg;
  EXPECT_TRUE(blocked);
  EXPECT_GT(least, 0.0) << "the body never touched the sphere";
  EXPECT_LT(robot_->pose().p.x(), 1.6);
  EXPECT_TRUE(isZero(robot_->log().back()));
  expectSilence();
}

TEST_F(Follower3DTest, InvalidConfigurationFailsConfigure)
{
  const auto state = start(
    RobotSetup{
      at(0.0, 0.0, 1.0), {rclcpp::Parameter("obstacles", std::vector<std::string>{"sphere 1 2"})}},
    false);
  EXPECT_EQ(state, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(Follower3DTest, CrossParameterRuleFailsConfigure)
{
  // min_approach_speed > desired_speed: validateTracker3Config refuses it.
  const auto state = start(
    RobotSetup{
      at(0.0, 0.0, 1.0),
      {rclcpp::Parameter("desired_speed", 0.1), rclcpp::Parameter("min_approach_speed", 0.2)}},
    false);
  EXPECT_EQ(state, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(Follower3DTest, ObstacleUpdatesAreCheckedAndBodyFrameOutput)
{
  start(RobotSetup{at(0.0, 0.5, 1.0, 0.5 * vf::kPi), {rclcpp::Parameter("cmd_frame", "body")}});
  // An invalid entry is refused; a valid one is taken.
  EXPECT_FALSE(
    follower_->set_parameter(rclcpp::Parameter("obstacles", std::vector<std::string>{"cone"}))
      .successful);
  EXPECT_TRUE(follower_
                ->set_parameter(
                  rclcpp::Parameter("obstacles", std::vector<std::string>{"plane 0 0 -5 0 0 1"}))
                .successful);
  // Body frame: the commands carry robot_base_frame (the rotation itself is a helper test).
  const auto handle = send(builtin("circle", {"radius", "cz"}, {0.5, 1.0}));
  ASSERT_TRUE(handle);
  tick();
  tick();
  EXPECT_EQ(robot_->lastFrame(), "base_link");
  cancel(handle);
  waitResult(resultOf(handle), 10.0);
}

}  // namespace
}  // namespace vectorfield_follower_3d_test
