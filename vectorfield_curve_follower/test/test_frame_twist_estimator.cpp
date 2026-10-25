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

// FrameTwistEstimator: exact log_SE(2) differencing across ±π, TF source equal
// to the Odometry source under a constant twist, the Δt window, outliers, the LPF and the
// extrapolation limit.

#include <gtest/gtest.h>

#include <cmath>

#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_curve_follower/frame_twist_estimator.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_curve_follower::FrameEstimate;
using vectorfield_curve_follower::FrameTwistConfig;
using vectorfield_curve_follower::FrameTwistEstimator;
using vectorfield_curve_follower::SampleResult;

/// The frame pose after `t` seconds of a constant twist, with the yaw wrapped as TF reports it.
vf::Pose2 poseAt(const vf::Pose2 & start, const vf::Twist2 & twist, double t)
{
  vf::Pose2 pose = vf::sim::integrateTwist(start, twist, t);
  pose.yaw = vf::wrapAngle(pose.yaw);
  return pose;
}

void expectPoseNear(const vf::Pose2 & a, const vf::Pose2 & b, double tolerance)
{
  EXPECT_NEAR(a.p.x(), b.p.x(), tolerance);
  EXPECT_NEAR(a.p.y(), b.p.y(), tolerance);
  EXPECT_NEAR(vf::wrapAngle(a.yaw - b.yaw), 0.0, tolerance);
}

TEST(FrameTwistEstimator, BodyTwistIsExactForAConstantTwistAcrossPi)
{
  const vf::Twist2 twist{0.3, -0.1, 0.8};
  const vf::Pose2 start{vf::Vec2(1.0, -2.0), 3.0};
  for (const double dt : {1e-4, 0.05, 0.5, 2.0}) {
    const vf::Twist2 estimated =
      FrameTwistEstimator::bodyTwist(start, poseAt(start, twist, dt), dt);
    EXPECT_NEAR(estimated.vx, twist.vx, 1e-9) << dt;
    EXPECT_NEAR(estimated.vy, twist.vy, 1e-9) << dt;
    EXPECT_NEAR(estimated.wz, twist.wz, 1e-9) << dt;
  }
  // Pure translation (the small-angle branch) and a non-positive Δt.
  const vf::Twist2 straight = FrameTwistEstimator::bodyTwist(
    vf::Pose2{vf::Vec2::Zero(), vf::kPi}, vf::Pose2{vf::Vec2(-0.2, 0.0), -vf::kPi}, 0.1);
  EXPECT_NEAR(straight.vx, 2.0, 1e-12);
  EXPECT_NEAR(straight.vy, 0.0, 1e-12);
  EXPECT_NEAR(straight.wz, 0.0, 1e-12);
  const vf::Twist2 none = FrameTwistEstimator::bodyTwist(start, start, 0.0);
  EXPECT_EQ(none.vx, 0.0);
  EXPECT_EQ(none.wz, 0.0);
}

// Layer 2b requirement: TF-source velocity equals Odometry-source velocity to 1e-6 under a
// constant twist ((0.1, 0) m/s, 0.3 rad/s) with the frame yaw starting at 3.1 rad.
TEST(FrameTwistEstimator, TfSourceEqualsOdometrySourceUnderAConstantTwist)
{
  const vf::Twist2 twist{0.1, 0.0, 0.3};
  const vf::Pose2 start{vf::Vec2(0.5, -0.3), 3.1};
  FrameTwistEstimator tf_source;
  FrameTwistEstimator odom_source;
  const double t0 = 100.0;
  const double period = 0.05;
  bool crossed = false;
  double previous_yaw = start.yaw;
  for (int k = 0; k <= 60; ++k) {
    const double t = t0 + period * k;
    const vf::Pose2 pose = poseAt(start, twist, t - t0);
    crossed = crossed || (previous_yaw > 0.0 && pose.yaw < 0.0);
    previous_yaw = pose.yaw;
    const SampleResult tf_result = tf_source.addPose(t, pose);
    EXPECT_EQ(tf_result, k == 0 ? SampleResult::kStarted : SampleResult::kAccepted);
    EXPECT_EQ(odom_source.addOdometry(t, pose, twist), SampleResult::kAccepted);
    if (k == 0) {
      EXPECT_FALSE(tf_source.estimate(t).ready);
      continue;
    }
    const double query = t + 0.02;
    const FrameEstimate a = tf_source.estimate(query);
    const FrameEstimate b = odom_source.estimate(query);
    ASSERT_TRUE(a.ready && !a.stale);
    ASSERT_TRUE(b.ready && !b.stale);
    EXPECT_NEAR(a.twist.vx, b.twist.vx, 1e-6);
    EXPECT_NEAR(a.twist.vy, b.twist.vy, 1e-6);
    EXPECT_NEAR(a.twist.wz, b.twist.wz, 1e-6);
    EXPECT_NEAR(b.twist.vx, 0.1, 1e-12);
    EXPECT_NEAR(b.twist.wz, 0.3, 1e-12);
    expectPoseNear(a.pose, b.pose, 1e-6);
    expectPoseNear(a.pose, poseAt(start, twist, query - t0), 1e-6);
  }
  EXPECT_TRUE(crossed) << "the yaw must cross ±π";
}

TEST(FrameTwistEstimator, DuplicateAndCloseStampsDoNotDifferentiate)
{
  FrameTwistEstimator estimator;
  const vf::Pose2 a{vf::Vec2::Zero(), 0.0};
  EXPECT_EQ(estimator.addPose(1.0, a), SampleResult::kStarted);
  EXPECT_EQ(estimator.addPose(1.0, vf::Pose2{vf::Vec2(5.0, 0.0), 0.0}), SampleResult::kDuplicate);
  EXPECT_EQ(estimator.addPose(0.9, a), SampleResult::kInvalid);
  EXPECT_EQ(
    estimator.addPose(1.005, vf::Pose2{vf::Vec2(0.0005, 0.0), 0.0}), SampleResult::kTooClose);
  // The pair is measured from the base (t = 1.0), not from the close sample.
  EXPECT_EQ(estimator.addPose(1.1, vf::Pose2{vf::Vec2(0.01, 0.0), 0.0}), SampleResult::kAccepted);
  const FrameEstimate estimate = estimator.estimate(1.1);
  EXPECT_NEAR(estimate.twist.vx, 0.1, 1e-12);
  EXPECT_EQ(estimator.addPose(2.0, vf::Pose2{vf::Vec2::Zero(), 0.0}), SampleResult::kStarted)
    << "a gap longer than max_dt restarts the estimator";
  EXPECT_FALSE(estimator.estimate(2.0).ready);
}

TEST(FrameTwistEstimator, OutliersAreDroppedAndTheBaseKept)
{
  FrameTwistEstimator estimator;
  const vf::Twist2 twist{0.2, 0.0, 0.0};
  const vf::Pose2 start{vf::Vec2::Zero(), 0.0};
  ASSERT_EQ(estimator.addPose(0.0, start), SampleResult::kStarted);
  ASSERT_EQ(estimator.addPose(0.05, poseAt(start, twist, 0.05)), SampleResult::kAccepted);
  // A 1 m jump in 50 ms (20 m/s) is an outlier.
  vf::Pose2 jump = poseAt(start, twist, 0.1);
  jump.p.x() += 1.0;
  EXPECT_EQ(estimator.addPose(0.1, jump), SampleResult::kOutlier);
  EXPECT_EQ(estimator.outliers(), 1U);
  EXPECT_NEAR(estimator.estimate(0.1).twist.vx, 0.2, 1e-12);
  // A yaw-rate outlier.
  vf::Pose2 spin = poseAt(start, twist, 0.1);
  spin.yaw += 1.0;
  EXPECT_EQ(estimator.addPose(0.1, spin), SampleResult::kOutlier);
  // The next good sample pairs with the kept base (t = 0.05).
  EXPECT_EQ(estimator.addPose(0.15, poseAt(start, twist, 0.15)), SampleResult::kAccepted);
  EXPECT_NEAR(estimator.estimate(0.15).twist.vx, 0.2, 1e-12);
  // Odometry outliers too.
  FrameTwistEstimator odom;
  EXPECT_EQ(odom.addOdometry(0.0, start, vf::Twist2{3.0, 0.0, 0.0}), SampleResult::kOutlier);
  EXPECT_FALSE(odom.estimate(0.0).ready);
}

TEST(FrameTwistEstimator, TheLowPassFiltersVelocities)
{
  FrameTwistConfig config;
  config.cutoff_hz = 2.0;
  FrameTwistEstimator estimator(config);
  const double dt = 0.05;
  vf::Pose2 pose{vf::Vec2::Zero(), 0.0};
  ASSERT_EQ(estimator.addPose(0.0, pose), SampleResult::kStarted);
  pose = vf::sim::integrateTwist(pose, vf::Twist2{0.2, 0.0, 0.0}, dt);
  ASSERT_EQ(estimator.addPose(dt, pose), SampleResult::kAccepted);
  EXPECT_NEAR(estimator.estimate(dt).twist.vx, 0.2, 1e-12) << "the first pair initialises";
  pose = vf::sim::integrateTwist(pose, vf::Twist2{0.6, 0.0, 0.5}, dt);
  ASSERT_EQ(estimator.addPose(2 * dt, pose), SampleResult::kAccepted);
  const double alpha = 1.0 - std::exp(-2.0 * vf::kPi * config.cutoff_hz * dt);
  const FrameEstimate estimate = estimator.estimate(2 * dt);
  EXPECT_NEAR(estimate.twist.vx, 0.2 + alpha * 0.4, 1e-9);
  EXPECT_NEAR(estimate.twist.wz, alpha * 0.5, 1e-9);

  // Cutoff 0 disables the filter.
  config.cutoff_hz = 0.0;
  estimator.setConfig(config);
  pose = vf::sim::integrateTwist(pose, vf::Twist2{0.1, 0.0, 0.0}, dt);
  ASSERT_EQ(estimator.addPose(3 * dt, pose), SampleResult::kAccepted);
  EXPECT_NEAR(estimator.estimate(3 * dt).twist.vx, 0.1, 1e-9);
}

TEST(FrameTwistEstimator, ExtrapolatesAtMostInputTimeout)
{
  FrameTwistConfig config;
  config.input_timeout = 0.3;
  FrameTwistEstimator estimator(config);
  const vf::Twist2 twist{0.1, 0.0, 0.3};
  const vf::Pose2 start{vf::Vec2(1.0, 1.0), 0.5};
  ASSERT_EQ(estimator.addOdometry(10.0, start, twist), SampleResult::kAccepted);

  const FrameEstimate inside = estimator.estimate(10.2);
  EXPECT_TRUE(inside.ready);
  EXPECT_FALSE(inside.stale);
  EXPECT_NEAR(inside.age, 0.2, 1e-12);
  expectPoseNear(inside.pose, vf::sim::integrateTwist(start, twist, 0.2), 1e-12);
  EXPECT_DOUBLE_EQ(inside.twist.wz, 0.3);

  // Beyond input_timeout: hold the last pose, zero twist.
  const FrameEstimate stale = estimator.estimate(10.31);
  EXPECT_TRUE(stale.ready);
  EXPECT_TRUE(stale.stale);
  expectPoseNear(stale.pose, start, 0.0);
  EXPECT_EQ(stale.twist.vx, 0.0);
  EXPECT_EQ(stale.twist.wz, 0.0);

  // A query before the sample does not extrapolate backwards.
  expectPoseNear(estimator.estimate(9.9).pose, start, 0.0);

  estimator.reset();
  EXPECT_FALSE(estimator.estimate(10.0).ready);
  EXPECT_TRUE(estimator.estimate(10.0).stale);
}

}  // namespace
