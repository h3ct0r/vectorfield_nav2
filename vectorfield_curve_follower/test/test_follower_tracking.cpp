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

// The follower in process (plan §9 layer 2b), goals that succeed: laps done and an open curve
// settled (one zero, then silence), reverse_direction, lap counting under ±5 cm dither from a
// start 5 cm before s = 0, preemption (the TrackerState is kept only for an identical goal), and
// a translating circle blocked by a box for 3 s.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "follower_test_harness.hpp"
#include "vectorfield_core/status.hpp"

namespace
{

using namespace vectorfield_curve_follower_test;  // NOLINT(google-build-using-namespace)

class FollowerTracking : public FollowerTest
{
protected:
  static void expectSucceeded(const WrappedResult & result)
  {
    ASSERT_TRUE(result.result);
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->error_msg;
    EXPECT_EQ(result.result->error_code, FollowCurve::Result::NONE);
  }

  /// The robot starts on the circle x² + y² = 0.5² at (0.5, 0), 2.2 rad off its tangent: the
  /// first goal rotates it in place (ROTATE_TO_FIELD) until the error is inside the hysteresis
  /// band (rotate_to_field_release_angle 0.785, rotate_to_field_angle 1.57).
  std::shared_ptr<GoalHandle> rotateIntoTheHysteresisBand()
  {
    FollowerSetup setup;
    setup.start = vf::Pose2{vf::Vec2(0.5, 0.0), 0.5 * vf::kPi + 2.2};
    start(setup);
    if (HasFatalFailure()) {
      return nullptr;
    }
    auto handle = send(circleGoal(0.5, 0));
    if (!handle) {
      return nullptr;
    }
    const bool in_band = tickUntil(
      [this]() {
        return follower_->lastCycle().phase == vf::Phase::kRotateToField &&
               headingError(robot_->pose()) < 1.3;
      },
      5.0);
    EXPECT_TRUE(in_band) << "never in ROTATE_TO_FIELD inside the band";
    EXPECT_GT(headingError(robot_->pose()), 1.0);
    return handle;
  }

  /// Preempts with `goal` and runs one cycle; returns the phase of that cycle.
  vf::Phase preemptWith(const FollowCurve::Goal & goal, const ResultFuture & previous)
  {
    const CycleSnapshot before = follower_->lastCycle();
    // Pending once send() returns: the next cycle takes it, and the action server aborts the
    // preempted goal before that cycle runs.
    const auto handle = send(goal);
    EXPECT_TRUE(handle);
    tick();
    EXPECT_TRUE(waitFor([&]() { return ready(previous); })) << "the preempted goal was not aborted";
    if (ready(previous)) {
      EXPECT_EQ(previous.get().code, rclcpp_action::ResultCode::ABORTED);
    }
    const CycleSnapshot snapshot = follower_->lastCycle();
    EXPECT_EQ(snapshot.goal, before.goal + 1) << "the cycle did not run the new goal";
    EXPECT_EQ(snapshot.cycles, before.cycles + 1);
    EXPECT_TRUE(snapshot.tracker_ran);
    if (handle) {
      cancel(handle);
      const auto canceled = waitResult(resultOf(handle), 10.0);
      EXPECT_EQ(canceled.code, rclcpp_action::ResultCode::CANCELED);
    }
    return snapshot.phase;
  }
};

TEST_F(FollowerTracking, LapsDoneSucceedsWithOneZeroThenSilence)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const auto result = run(circleGoal(0.5, 1), 30.0);
  expectSucceeded(result);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->laps_completed, 1U);
  settle();
  EXPECT_GT(robot_->commands(), 20U);
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(FollowerTracking, OpenCurveSettledSucceedsWithOneZero)
{
  FollowerSetup setup;
  setup.start = vf::Pose2{vf::Vec2(0.0, 0.0), 0.0};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  const auto result = run(polylineGoal({vf::Vec2(0.0, 0.0), vf::Vec2(1.5, 0.0)}), 30.0);
  expectSucceeded(result);
  settle();
  // goal_tolerance 0.10.
  EXPECT_LT((robot_->pose().p - vf::Vec2(1.5, 0.0)).norm(), 0.15);
  EXPECT_EQ(robot_->trailingZeros(), 1U);
  expectSilence();
}

TEST_F(FollowerTracking, ReverseDirectionSucceedsNearTheFirstPoint)
{
  FollowerSetup setup;
  setup.start = vf::Pose2{vf::Vec2(1.5, 0.0), vf::kPi};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  FollowCurve::Goal goal = polylineGoal({vf::Vec2(0.0, 0.0), vf::Vec2(1.5, 0.0)});
  goal.reverse_direction = true;
  const auto result = run(goal, 30.0);
  expectSucceeded(result);
  settle();
  EXPECT_LT((robot_->pose().p - vf::Vec2(0.0, 0.0)).norm(), 0.15);
  EXPECT_GT(robot_->distance(), 1.2);
  EXPECT_EQ(robot_->trailingZeros(), 1U);
}

// The robot starts 5 cm before s = 0 and its published pose dithers ±5 cm along the heading
// for the whole run: neither the seam crossings at the start nor the dither count a lap, and two
// laps end after two turns around the circle. Heading tracking evaluates the field at base_link,
// so s* is the robot's own station (feedback linearization would use a point 0.2 m ahead).
TEST_F(FollowerTracking, LapDitherAndAStartBeforeTheSeamDoNotMiscount)
{
  FollowerSetup setup;
  constexpr double kRadius = 0.5;
  const double before = 0.05 / kRadius;
  setup.start = vf::Pose2{
    vf::Vec2(kRadius * std::cos(before), -kRadius * std::sin(before)), 0.5 * vf::kPi - before};
  setup.parameters = {rclcpp::Parameter("tracking_law", "heading_tracking")};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  robot_->setDither(0.05);
  const auto result = run(circleGoal(kRadius, 2), 60.0);
  expectSucceeded(result);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->laps_completed, 2U);
  // Two turns from 0.1 rad before the seam, within the dither (0.1 rad) and a cycle's travel.
  EXPECT_NEAR(robot_->sweptAngle(), 2.0 * vf::kTwoPi, 0.25);
}

// An identical goal keeps the TrackerState: still in ROTATE_TO_FIELD inside the band.
TEST_F(FollowerTracking, PreemptionWithAnIdenticalGoalKeepsTheTrackerState)
{
  const auto first = rotateIntoTheHysteresisBand();
  ASSERT_TRUE(first);
  ASSERT_FALSE(HasFailure());
  const ResultFuture previous = resultOf(first);
  EXPECT_EQ(preemptWith(circleGoal(0.5, 0), previous), vf::Phase::kRotateToField);
}

// A changed CurveSpec restarts from the initial state: inside the band, a fresh state follows.
TEST_F(FollowerTracking, PreemptionWithAChangedCurveRestartsTheTrackerState)
{
  const auto first = rotateIntoTheHysteresisBand();
  ASSERT_TRUE(first);
  ASSERT_FALSE(HasFailure());
  const ResultFuture previous = resultOf(first);
  EXPECT_EQ(preemptWith(circleGoal(0.55, 0), previous), vf::Phase::kFollow);
}

// A changed override (desired_speed) restarts from the initial state too.
TEST_F(FollowerTracking, PreemptionWithAChangedOverrideRestartsTheTrackerState)
{
  const auto first = rotateIntoTheHysteresisBand();
  ASSERT_TRUE(first);
  ASSERT_FALSE(HasFailure());
  const ResultFuture previous = resultOf(first);
  FollowCurve::Goal goal = circleGoal(0.5, 0);
  goal.desired_speed = 0.3;
  EXPECT_EQ(preemptWith(goal, previous), vf::Phase::kFollow);
}

// A circle translating at 0.1 m/s (FRAME_MOVING, Odometry source) with a box on it for 3 s: the
// Assumption-3 timer is paused while blocked, and the robot resumes and completes the lap.
TEST_F(FollowerTracking, TranslatingCircleBlockedByABoxResumes)
{
  FollowerSetup setup;
  // The frame source starts after discovery.
  setup.parameters = {rclcpp::Parameter("stale_abort_timeout", 10.0)};
  ASSERT_NO_FATAL_FAILURE(start(setup));
  const vf::Vec2 velocity(0.1, 0.0);
  robot_->startMovingFrame(MovingFrame{"platform_odom", "platform", velocity, t_ns_});
  FollowCurve::Goal goal = circleGoal(0.5, 1);
  goal.curve.header.frame_id = "platform";
  goal.frame_motion = FollowCurve::Goal::FRAME_MOVING;
  goal.frame_odom_topic = "platform_odom";
  const auto handle = send(goal);
  ASSERT_TRUE(handle);
  const ResultFuture future = resultOf(handle);
  ASSERT_TRUE(waitFor([&]() { return robot_->frameSubscribers() > 0; }));
  tickFor(1.0);
  ASSERT_FALSE(ready(future)) << future.get().result->error_msg;

  // A box on the circle, 1 rad ahead of the robot where the circle will be in about 1 s.
  const vf::Vec2 centre = robot_->frameOriginAt(t_ns_);
  const vf::Vec2 offset = robot_->pose().p - centre;
  const double angle = std::atan2(offset.y(), offset.x()) + 1.0;
  robot_->setBoxes(
    {boxAt(centre + velocity + 0.5 * vf::Vec2(std::cos(angle), std::sin(angle)), 0.1)});
  const bool blocked = tickUntil(
    [this]() { return vf::Status::has(follower_->lastCycle().status, vf::Status::kBlocked); }, 5.0);
  ASSERT_TRUE(blocked) << "kBlocked never set";
  tickFor(3.0);
  ASSERT_FALSE(ready(future)) << future.get().result->error_msg;
  robot_->setBoxes({});
  const double resumed = robot_->distance();

  const auto result = waitResult(future, 40.0);
  expectSucceeded(result);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->error_code, FollowCurve::Result::ASSUMPTION3_VIOLATED);
  EXPECT_EQ(result.result->laps_completed, 1U);
  EXPECT_GT(robot_->distance() - resumed, 1.0) << "the robot did not resume";
}

}  // namespace
