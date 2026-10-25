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

// The follower in process, goal validation and mutual exclusion (the
// failure table): every INVALID_CURVE, INVALID_GOAL and UNSUPPORTED row ends before the first
// command; a Nav2 action active at goal start ends NAV2_ACTIVE with no message, and one that
// becomes active mid-run gets exactly one zero; the interlock and the output topic follow the
// node's namespace.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "follower_test_harness.hpp"

namespace
{

using namespace vectorfield_curve_follower_test;  // NOLINT(google-build-using-namespace)
using action_msgs::msg::GoalStatus;

class FollowerGoalChecks : public FollowerTest
{
protected:
  /// Runs a goal that must end with `code` before any command.
  void expectRefused(const FollowCurve::Goal & goal, std::uint16_t code, const std::string & text)
  {
    const std::size_t before = robot_->commands();
    const auto result = run(goal, 10.0);
    ASSERT_TRUE(result.result);
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(result.result->error_code, code) << result.result->error_msg;
    EXPECT_NE(result.result->error_msg.find(text), std::string::npos)
      << "'" << result.result->error_msg << "' does not mention '" << text << "'";
    settle();
    EXPECT_EQ(robot_->commands(), before) << "a refused goal published a command";
  }

  /// A circle goal (laps 0) runs: the robot receives commands; then it is canceled.
  void expectRunsThenCancel()
  {
    const auto handle = send(circleGoal(0.5, 0));
    ASSERT_TRUE(handle);
    const ResultFuture result = resultOf(handle);
    tickFor(1.0);
    ASSERT_FALSE(ready(result)) << "the goal ended: " << result.get().result->error_msg;
    EXPECT_GT(robot_->commands(), 5U);
    cancel(handle);
    const auto canceled = waitResult(result, 10.0);
    ASSERT_TRUE(canceled.result);
    EXPECT_EQ(canceled.code, rclcpp_action::ResultCode::CANCELED);
  }

  /// Waits until the follower's graph sees a publisher on `topic`, and lets a transient-local
  /// sample arrive.
  void waitPublisherSeen(const std::string & topic)
  {
    ASSERT_TRUE(waitFor([&]() { return follower_->count_publishers(topic) > 0; })) << topic;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  /// A circle goal runs 1.5 s, then `action` goes EXECUTING: NAV2_ACTIVE, exactly one zero.
  void expectOneZeroWhenActiveMidRun(const std::string & action)
  {
    ActionStatusStub stub(robot_->node(), action);
    ASSERT_TRUE(waitFor([&]() { return stub.subscribers() > 0; })) << stub.topic();
    const auto handle = send(circleGoal(0.5, 0));
    ASSERT_TRUE(handle);
    const ResultFuture future = resultOf(handle);
    tickFor(1.5);
    ASSERT_GT(robot_->commands(), 10U) << describe(follower_->lastCycle());
    ASSERT_FALSE(isZero(robot_->lastCommand()));
    stub.publish(GoalStatus::STATUS_EXECUTING);
    const auto result = waitResult(future, 5.0);
    ASSERT_TRUE(result.result);
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(result.result->error_code, FollowCurve::Result::NAV2_ACTIVE)
      << result.result->error_msg;
    settle();
    EXPECT_EQ(robot_->trailingZeros(), 1U) << "exactly one zero after NAV2_ACTIVE";
    expectSilence();
  }
};

TEST_F(FollowerGoalChecks, InvalidCurvesEndBeforeAnyCommand)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double two_pi = 2.0 * vf::kPi;
  struct Case
  {
    const char * name;
    FollowCurve::Goal goal;
    const char * text;
  };
  std::vector<Case> cases = {
    {"fewer than 2 points", polylineGoal({vf::Vec2(0.0, 0.0)}), "at least 2 points"},
    {"non-finite point", polylineGoal({vf::Vec2(0.0, 0.0), vf::Vec2(nan, 1.0)}), "not finite"},
    {"s_min >= s_max", expressionGoal("s", "0", 1.0, 1.0, false), "s_min < s_max"},
    {"parse error", expressionGoal("cos(s", "sin(s)", 0.0, two_pi, true), "x_expression"},
    {"expression too long", expressionGoal("s", "0", 0.0, 1.0, false), "limit is 1024"},
    {"too many points", polylineGoal({vf::Vec2(0.0, 0.0), vf::Vec2(2000.0, 0.0)}),
     "max_curve_points"},
    {"unknown builtin", circleGoal(), "unknown builtin_shape"},
    {"unknown parameter", circleGoal(), "unknown parameter 'c'"},
    {"not regular", expressionGoal("s^2", "s^3", -1.0, 1.0, false), "not regular"},
    {"closed but not closing", expressionGoal("s", "0.1*s", 0.0, 1.0, true), "does not close"},
    {"laps > 0 on an open curve", polylineGoal({vf::Vec2(0.0, 0.0), vf::Vec2(1.0, 0.0)}),
     "closed curve"},
  };
  std::string too_long = "s";
  while (too_long.size() <= 1024U) {
    too_long += "+0";
  }
  cases[4].goal.curve.x_expression = too_long;
  cases[6].goal.curve.builtin_shape = "hexagon";
  cases[7].goal.curve.parameter_names = {"a", "c"};
  cases[10].goal.laps = 2;
  for (const Case & c : cases) {
    SCOPED_TRACE(c.name);
    expectRefused(c.goal, FollowCurve::Result::INVALID_CURVE, c.text);
  }
  EXPECT_EQ(robot_->commands(), 0U);
}

TEST_F(FollowerGoalChecks, RejectedGoalScopedConfigurationIsInvalidGoal)
{
  ASSERT_NO_FATAL_FAILURE(start());
  // desired_speed above max_linear_vel (0.5).
  FollowCurve::Goal fast = circleGoal();
  fast.desired_speed = 0.6;
  expectRefused(fast, FollowCurve::Result::INVALID_GOAL, "goal configuration rejected");
  EXPECT_EQ(robot_->commands(), 0U);
}

TEST_F(FollowerGoalChecks, UnsupportedGoalsEndBeforeAnyCommand)
{
  FollowerSetup setup;
  // The frame source is checked once its first message arrives (after discovery).
  setup.parameters = {rclcpp::Parameter("stale_abort_timeout", 30.0)};
  ASSERT_NO_FATAL_FAILURE(start(setup));

  {
    SCOPED_TRACE("open and time-varying");
    expectRefused(
      expressionGoal("s + 0.1*t", "0", 0.0, 1.0, false), FollowCurve::Result::UNSUPPORTED,
      "time-varying");
  }
  {
    SCOPED_TRACE("TF differencing without moving_frame_reference");
    FollowCurve::Goal goal = circleGoal();
    goal.curve.header.frame_id = "platform";
    goal.frame_motion = FollowCurve::Goal::FRAME_MOVING;
    expectRefused(goal, FollowCurve::Result::UNSUPPORTED, "moving_frame_reference");
  }
  {
    SCOPED_TRACE("Odometry child_frame_id is not the curve frame");
    robot_->startMovingFrame(MovingFrame{"platform_odom", "trailer", vf::Vec2::Zero(), t_ns_});
    FollowCurve::Goal goal = circleGoal();
    goal.curve.header.frame_id = "platform";
    goal.frame_motion = FollowCurve::Goal::FRAME_MOVING;
    goal.frame_odom_topic = "platform_odom";
    const auto handle = send(goal);
    ASSERT_TRUE(handle);
    const ResultFuture future = resultOf(handle);
    ASSERT_TRUE(waitFor([&]() { return robot_->frameSubscribers() > 0 || ready(future); }));
    const auto result = waitResult(future, 10.0);
    ASSERT_TRUE(result.result);
    EXPECT_EQ(result.result->error_code, FollowCurve::Result::UNSUPPORTED)
      << result.result->error_msg;
    EXPECT_NE(result.result->error_msg.find("trailer"), std::string::npos)
      << result.result->error_msg;
  }
  settle();
  EXPECT_EQ(robot_->commands(), 0U);
}

TEST_F(FollowerGoalChecks, InterlockMustSeeFollowPath)
{
  FollowerSetup setup;
  setup.parameters = {rclcpp::Parameter("require_nav2_interlock", true)};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  expectRefused(circleGoal(), FollowCurve::Result::UNSUPPORTED, "/follow_path/_action/status");

  // A (stub) follow_path in the follower's namespace: the goal runs.
  const ActionStatusStub follow_path(robot_->node(), "follow_path");
  ASSERT_NO_FATAL_FAILURE(waitPublisherSeen(follow_path.topic()));
  expectRunsThenCancel();
}

// The follower in /robot1: the default output topic is /robot1/cmd_vel_nav, and a follow_path in
// another namespace does not satisfy the interlock (UNSUPPORTED, no message).
TEST_F(FollowerGoalChecks, NamespacedFollowerUsesItsNamespace)
{
  FollowerSetup setup;
  setup.ns = "/robot1";
  setup.parameters = {rclcpp::Parameter("require_nav2_interlock", true)};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  EXPECT_TRUE(waitFor([&]() {
    const auto publishers = follower_->get_publishers_info_by_topic("/robot1/cmd_vel_nav");
    return std::any_of(publishers.begin(), publishers.end(), [](const auto & info) {
      return info.node_name() == "curve_follower" && info.node_namespace() == "/robot1";
    });
  }))
    << "no /robot1/cmd_vel_nav publisher";
  EXPECT_EQ(follower_->count_publishers("/cmd_vel_nav"), 0U);

  const ActionStatusStub elsewhere(robot_->node(), "/follow_path");
  ASSERT_NO_FATAL_FAILURE(waitPublisherSeen("/follow_path/_action/status"));
  expectRefused(
    circleGoal(), FollowCurve::Result::UNSUPPORTED, "/robot1/follow_path/_action/status");
  EXPECT_EQ(robot_->commands(), 0U);

  const ActionStatusStub here(robot_->node(), "follow_path");
  ASSERT_EQ(here.topic(), "/robot1/follow_path/_action/status");
  ASSERT_NO_FATAL_FAILURE(waitPublisherSeen(here.topic()));
  // The robot in /robot1 receives the commands.
  expectRunsThenCancel();
}

TEST_F(FollowerGoalChecks, Nav2ActiveAtGoalStartPublishesNothing)
{
  ASSERT_NO_FATAL_FAILURE(start());
  ActionStatusStub navigate(robot_->node(), "navigate_to_pose");
  ASSERT_TRUE(waitFor([&]() { return navigate.subscribers() > 0; })) << navigate.topic();
  navigate.publish(GoalStatus::STATUS_EXECUTING);
  ASSERT_NO_FATAL_FAILURE(waitPublisherSeen(navigate.topic()));
  expectRefused(circleGoal(), FollowCurve::Result::NAV2_ACTIVE, "navigate_to_pose");

  // Once it is no longer active, a goal runs.
  navigate.publish(GoalStatus::STATUS_SUCCEEDED);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  expectRunsThenCancel();
}

TEST_F(FollowerGoalChecks, Nav2StatusEndingJustAfterGoalStartIsWaitedFor)
{
  // A client that starts the curve as soon as a Nav2 action's result arrives, before its
  // terminal status: the follower waits for the status instead of refusing the goal.
  ASSERT_NO_FATAL_FAILURE(start());
  ActionStatusStub navigate(robot_->node(), "navigate_to_pose");
  ASSERT_TRUE(waitFor([&]() { return navigate.subscribers() > 0; })) << navigate.topic();
  navigate.publish(GoalStatus::STATUS_EXECUTING);
  ASSERT_NO_FATAL_FAILURE(waitPublisherSeen(navigate.topic()));
  std::thread ending([&navigate]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    navigate.publish(GoalStatus::STATUS_SUCCEEDED);
  });
  expectRunsThenCancel();
  ending.join();
}

TEST_F(FollowerGoalChecks, NavigateToPoseBecomingActiveMidRunSendsExactlyOneZero)
{
  ASSERT_NO_FATAL_FAILURE(start());
  expectOneZeroWhenActiveMidRun("navigate_to_pose");
}

TEST_F(FollowerGoalChecks, DockRobotBecomingActiveMidRunSendsExactlyOneZero)
{
  ASSERT_NO_FATAL_FAILURE(start());
  expectOneZeroWhenActiveMidRun("dock_robot");
}

}  // namespace
