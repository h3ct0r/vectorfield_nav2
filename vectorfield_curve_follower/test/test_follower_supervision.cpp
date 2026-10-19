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

// The follower in process (plan §9 layer 2b), supervision of a running goal (plan §6 "Failure
// modes"): TF_ERROR, STALE_INPUT (odometry, costmap and moving-frame gaps), BLOCKED (kBlocked,
// the progress watchdog, the contour watchdog), UNSUPPORTED from kHeadingInfeasible,
// ASSUMPTION3_VIOLATED, UNKNOWN on deactivation, TIMEOUT and cancel, each with its stop
// behaviour (decelerate, zero, one zero then silence).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>

#include "follower_test_harness.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"

namespace
{

using namespace vectorfield_curve_follower_test;  // NOLINT(google-build-using-namespace)
using vectorfield_curve_follower::SupervisorAction;

double toSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

class FollowerSupervision : public FollowerTest
{
protected:
  /// Expects an aborted result with `code` whose message mentions `text`.
  static void expectAborted(
    const WrappedResult & result, std::uint16_t code, const std::string & text)
  {
    ASSERT_TRUE(result.result);
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(result.result->error_code, code) << result.result->error_msg;
    EXPECT_NE(result.result->error_msg.find(text), std::string::npos)
      << "'" << result.result->error_msg << "' does not mention '" << text << "'";
  }

  bool decelerating() const
  {
    return follower_->lastCycle().decision.action == SupervisorAction::kDecelerate;
  }

  bool blocked() const
  {
    return vf::Status::has(follower_->lastCycle().status, vf::Status::kBlocked);
  }

  /// Starts a circle goal (laps 0) and runs it for `duration`; the robot is moving.
  ResultFuture startRunning(const FollowCurve::Goal & goal, double duration)
  {
    const auto handle = send(goal);
    if (!handle) {
      return ResultFuture{};
    }
    handle_ = handle;
    ResultFuture future = resultOf(handle);
    tickFor(duration);
    EXPECT_FALSE(ready(future)) << "the goal ended early";
    EXPECT_GT(robot_->commands(), 10U) << describe(follower_->lastCycle());
    EXPECT_FALSE(isZero(robot_->lastCommand()));
    return future;
  }

  /// A gap starts: once the follower decelerates, the commands only slow down; STALE_INPUT
  /// after stale_abort_timeout, then silence.
  void expectStaleInputAfterGap(const ResultFuture & future, const std::string & text)
  {
    ASSERT_TRUE(tickUntil([this]() { return decelerating(); }, 2.0)) << "no deceleration";
    const std::size_t first = robot_->commands() - 2;
    const double since = now();
    const auto result = waitResult(future, 5.0);
    expectAborted(result, FollowCurve::Result::STALE_INPUT, text);
    EXPECT_GE(now() - since, 2.0 - 1e-6) << "before stale_abort_timeout";
    settle();
    EXPECT_TRUE(deceleratesFrom(first));
    EXPECT_TRUE(isZero(robot_->lastCommand()));
    expectSilence();
  }

  std::shared_ptr<GoalHandle> handle_;
};

/// A box on the circle x² + y² = 0.5², about 0.5 m ahead of the start pose.
Box boxOnTheCircle() { return boxAt(vf::Vec2(0.5 * std::cos(1.0), 0.5 * std::sin(1.0)), 0.08); }

TEST_F(FollowerSupervision, UntransformableCurveFrameIsTfErrorWithoutCommands)
{
  ASSERT_NO_FATAL_FAILURE(start());
  FollowCurve::Goal goal = circleGoal();
  goal.curve.header.frame_id = "nowhere";
  const auto result = run(goal, 5.0);
  expectAborted(result, FollowCurve::Result::TF_ERROR, "not transformable");
  ASSERT_TRUE(result.result);
  EXPECT_GE(toSeconds(result.result->total_elapsed_time), 2.0 - 1e-6);
  EXPECT_LT(toSeconds(result.result->total_elapsed_time), 2.5);
  settle();
  EXPECT_EQ(robot_->commands(), 0U);
}

TEST_F(FollowerSupervision, NoCostmapIsStaleInputWithoutCommands)
{
  FollowerSetup setup;
  setup.costmap = false;
  ASSERT_NO_FATAL_FAILURE(start(setup));
  const auto result = run(circleGoal(), 5.0);
  expectAborted(result, FollowCurve::Result::STALE_INPUT, "an input is stale");
  ASSERT_TRUE(result.result);
  EXPECT_GE(toSeconds(result.result->total_elapsed_time), 2.0 - 1e-6);
  settle();
  EXPECT_EQ(robot_->commands(), 0U);
}

TEST_F(FollowerSupervision, OdometryGapDeceleratesThenStaleInput)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const ResultFuture future = startRunning(circleGoal(0.5, 0), 1.5);
  ASSERT_FALSE(HasFailure());
  robot_->setOdometryPaused(true);
  expectStaleInputAfterGap(future, "an input is stale");
}

TEST_F(FollowerSupervision, CostmapGapDeceleratesThenStaleInput)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const ResultFuture future = startRunning(circleGoal(0.5, 0), 1.5);
  ASSERT_FALSE(HasFailure());
  const double gap = now();
  robot_->setCostmapPaused(true);
  expectStaleInputAfterGap(future, "an input is stale");
  // The costmap counts as stale only after costmap_timeout (0.5 s).
  EXPECT_GE(now() - gap, 0.5 + 2.0 - 0.15);
}

// FRAME_MOVING with an Odometry source that stops: the frame is held (no extrapolation, no
// frame velocity), the follower decelerates and aborts STALE_INPUT.
TEST_F(FollowerSupervision, StaleMovingFrameInputDeceleratesThenStaleInput)
{
  ASSERT_NO_FATAL_FAILURE(start());
  robot_->startMovingFrame(MovingFrame{"platform_odom", "platform", vf::Vec2(0.1, 0.0), t_ns_});
  FollowCurve::Goal goal = circleGoal(0.5, 0);
  goal.curve.header.frame_id = "platform";
  goal.frame_motion = FollowCurve::Goal::FRAME_MOVING;
  goal.frame_odom_topic = "platform_odom";
  const auto handle = send(goal);
  ASSERT_TRUE(handle);
  const ResultFuture future = resultOf(handle);
  ASSERT_TRUE(waitFor([&]() { return robot_->frameSubscribers() > 0; }));
  tickFor(2.0);
  ASSERT_FALSE(ready(future)) << future.get().result->error_msg;
  ASSERT_GT(robot_->commands(), 10U);
  robot_->setFramePaused(true);
  expectStaleInputAfterGap(future, "moving frame");
}

TEST_F(FollowerSupervision, PersistentBlockIsBlockedAfterBlockedTimeout)
{
  FollowerSetup setup;
  setup.parameters = {
    rclcpp::Parameter("blocked_timeout", 3.0), rclcpp::Parameter("progress_timeout", 60.0)};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  robot_->setBoxes({boxOnTheCircle()});
  const auto handle = send(circleGoal());
  ASSERT_TRUE(handle);
  const ResultFuture future = resultOf(handle);
  ASSERT_TRUE(tickUntil([this]() { return blocked(); }, 5.0)) << "kBlocked never set";
  const double since = now();
  const auto result = waitResult(future, 10.0);
  expectAborted(result, FollowCurve::Result::BLOCKED, "blocked_timeout");
  EXPECT_GE(now() - since, 3.0 - 0.1);
  settle();
  EXPECT_TRUE(isZero(robot_->lastCommand()));
  expectSilence();
}

TEST_F(FollowerSupervision, NoProgressIsBlockedAfterProgressTimeout)
{
  FollowerSetup setup;
  setup.parameters = {
    rclcpp::Parameter("progress_timeout", 2.0), rclcpp::Parameter("blocked_timeout", 60.0)};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  // Stuck wheels: the robot never moves, though nothing blocks it.
  robot_->setFrozen(true);
  const auto result = run(circleGoal(), 5.0);
  expectAborted(result, FollowCurve::Result::BLOCKED, "progress_timeout");
  ASSERT_TRUE(result.result);
  EXPECT_GE(toSeconds(result.result->total_elapsed_time), 2.0 - 1e-6);
  settle();
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(FollowerSupervision, ShortBlockResumesAndSucceeds)
{
  ASSERT_NO_FATAL_FAILURE(start());
  robot_->setBoxes({boxOnTheCircle()});
  const auto handle = send(circleGoal());
  ASSERT_TRUE(handle);
  const ResultFuture future = resultOf(handle);
  ASSERT_TRUE(tickUntil([this]() { return blocked(); }, 5.0)) << "kBlocked never set";
  tickFor(3.0);
  ASSERT_FALSE(ready(future)) << future.get().result->error_msg;
  EXPECT_TRUE(blocked());
  robot_->setBoxes({});
  const auto result = waitResult(future, 30.0);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->error_msg;
  EXPECT_EQ(result.result->laps_completed, 1U);
}

TEST_F(FollowerSupervision, ContourWatchdogIsBlocked)
{
  FollowerSetup setup;
  setup.start = vf::Pose2{vf::Vec2(0.0, 0.0), 0.0};
  setup.parameters = {
    rclcpp::Parameter("obstacle_mode", "contour"),
    rclcpp::Parameter("obstacle_activation", "always"),
    rclcpp::Parameter("obstacle_max_contour_time", 1.0),
    rclcpp::Parameter("blocked_timeout", 60.0),
    rclcpp::Parameter("progress_timeout", 60.0),
  };
  ASSERT_NO_FATAL_FAILURE(start(setup));
  // A 0.2 m box on the path, 2 m ahead: an episode starts and lasts longer than 1 s.
  robot_->setBoxes({boxAt(vf::Vec2(2.0, 0.0), 0.1)});
  const auto result = run(polylineGoal({vf::Vec2(0.0, 0.0), vf::Vec2(2.9, 0.0)}), 20.0);
  expectAborted(result, FollowCurve::Result::BLOCKED, "contour watchdog");
  settle();
  EXPECT_TRUE(isZero(robot_->lastCommand()));
  expectSilence();
}

// An Ackermann robot facing against an open curve: kHeadingInfeasible at the first cycle.
TEST_F(FollowerSupervision, HeadingInfeasibleIsUnsupported)
{
  vf::CoreConfig car;
  car.motion.motion_model = vf::MotionModel::kAckermann;
  car.motion.min_turning_radius = 0.5;
  FollowerSetup setup;
  setup.start = vf::Pose2{vf::Vec2(0.0, 0.0), vf::kPi};
  setup.kinematics = vf::Kinematics::fromConfig(car);
  setup.parameters = {
    rclcpp::Parameter("motion_model", "ackermann"),
    rclcpp::Parameter("min_turning_radius", 0.5),
  };
  ASSERT_NO_FATAL_FAILURE(start(setup));
  const auto result = run(polylineGoal({vf::Vec2(0.0, 0.0), vf::Vec2(2.5, 0.0)}), 5.0);
  expectAborted(result, FollowCurve::Result::UNSUPPORTED, "kHeadingInfeasible");
  settle();
  EXPECT_EQ(robot_->commands(), 0U);
}

// A circle translating at 1 m/s, faster than max_linear_vel: the clamp holds and the goal aborts
// after assumption3_abort_time, decelerating to one zero.
TEST_F(FollowerSupervision, Assumption3ClampHeldTooLongIsViolated)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const auto result =
    run(expressionGoal("0.5*cos(s) + 1.0*t", "0.5*sin(s)", 0.0, 2.0 * vf::kPi, true), 5.0);
  expectAborted(result, FollowCurve::Result::ASSUMPTION3_VIOLATED, "Assumption 3");
  ASSERT_TRUE(result.result);
  EXPECT_GE(toSeconds(result.result->total_elapsed_time), 1.0);
  settle();
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

// A parameter change the node accepts but the goal's overrides cannot use (max_linear_vel below
// the goal's desired_speed): the transactional rebuild fails, UNKNOWN with one zero.
TEST_F(FollowerSupervision, RejectedRebuildIsUnknownWithOneZero)
{
  ASSERT_NO_FATAL_FAILURE(start());
  FollowCurve::Goal goal = circleGoal(0.5, 0);
  goal.desired_speed = 0.45;
  const ResultFuture future = startRunning(goal, 1.5);
  ASSERT_FALSE(HasFailure());
  ASSERT_TRUE(follower_->set_parameter(rclcpp::Parameter("max_linear_vel", 0.42)).successful);
  const auto result = waitResult(future, 2.0);
  expectAborted(result, FollowCurve::Result::UNKNOWN, "configuration was rejected");
  settle();
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(FollowerSupervision, DeactivationDuringAGoalIsUnknownWithOneZero)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const ResultFuture future = startRunning(circleGoal(0.5, 0), 1.5);
  ASSERT_FALSE(HasFailure());
  ASSERT_EQ(follower_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(future.wait_for(kWallTimeout), std::future_status::ready);
  const WrappedResult & result = future.get();
  expectAborted(result, FollowCurve::Result::UNKNOWN, "deactivating");
  settle();
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(FollowerSupervision, MaxDurationIsTimeout)
{
  ASSERT_NO_FATAL_FAILURE(start());
  FollowCurve::Goal goal = circleGoal(0.5, 0);
  goal.max_duration.sec = 1;
  goal.max_duration.nanosec = 500'000'000U;
  const auto result = run(goal, 5.0);
  expectAborted(result, FollowCurve::Result::TIMEOUT, "max_duration");
  ASSERT_TRUE(result.result);
  EXPECT_GE(toSeconds(result.result->total_elapsed_time), 1.5 - 1e-6);
  settle();
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(FollowerSupervision, CancelDeceleratesSendsOneZeroThenSilence)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const ResultFuture future = startRunning(circleGoal(0.5, 0), 2.0);
  ASSERT_FALSE(HasFailure());
  const std::size_t first = robot_->commands() - 1;
  client_->async_cancel_goal(handle_);
  const auto result = waitResult(future, 5.0);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->error_code, FollowCurve::Result::NONE);
  settle();
  EXPECT_TRUE(deceleratesFrom(first));
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

}  // namespace
