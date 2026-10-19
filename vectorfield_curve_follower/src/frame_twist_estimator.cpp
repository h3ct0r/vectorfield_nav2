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

#include "vectorfield_curve_follower/frame_twist_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "vectorfield_core/kinematics.hpp"

namespace vectorfield_curve_follower
{

namespace vf = vectorfield_core;

namespace
{

/// Below this |Δθ| the series of (Δθ/2)·cot(Δθ/2) is used.
constexpr double kSmallAngle = 1e-6;

}  // namespace

FrameTwistEstimator::FrameTwistEstimator(const FrameTwistConfig & config) noexcept : config_(config)
{
}

void FrameTwistEstimator::setConfig(const FrameTwistConfig & config) noexcept { config_ = config; }

void FrameTwistEstimator::reset() noexcept
{
  has_latest_ = false;
  has_base_ = false;
  has_twist_ = false;
  twist_ = vf::Twist2{};
  outliers_ = 0;
}

vf::Twist2 FrameTwistEstimator::bodyTwist(
  const vf::Pose2 & a, const vf::Pose2 & b, double dt) noexcept
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return vf::Twist2{};
  }
  const double dtheta = vf::wrapAngle(b.yaw - a.yaw);
  // Displacement in a's axes.
  const vf::Isometry2 T_a(a.yaw, a.p);
  const vf::Vec2 local = T_a.rotateInverse(b.p - a.p);
  // V(Δθ)⁻¹ = [[A, B], [−B, A]] with A = (Δθ/2)·cot(Δθ/2), B = Δθ/2.
  const double half = 0.5 * dtheta;
  const double A =
    std::abs(dtheta) < kSmallAngle ? 1.0 - dtheta * dtheta / 12.0 : half / std::tan(half);
  const double B = half;
  vf::Twist2 twist;
  twist.vx = (A * local.x() + B * local.y()) / dt;
  twist.vy = (-B * local.x() + A * local.y()) / dt;
  twist.wz = dtheta / dt;
  return twist;
}

bool FrameTwistEstimator::isOutlier(const vf::Twist2 & twist) const noexcept
{
  return !vf::isFinite(twist) || std::hypot(twist.vx, twist.vy) > config_.max_speed ||
         std::abs(twist.wz) > config_.max_angular_rate;
}

void FrameTwistEstimator::filter(const vf::Twist2 & twist, double dt) noexcept
{
  if (!has_twist_ || !(config_.cutoff_hz > 0.0)) {
    twist_ = twist;
    has_twist_ = true;
    return;
  }
  const double alpha = 1.0 - std::exp(-2.0 * vf::kPi * config_.cutoff_hz * dt);
  twist_.vx += alpha * (twist.vx - twist_.vx);
  twist_.vy += alpha * (twist.vy - twist_.vy);
  twist_.wz += alpha * (twist.wz - twist_.wz);
}

SampleResult FrameTwistEstimator::addPose(double stamp, const vf::Pose2 & pose) noexcept
{
  if (!std::isfinite(stamp) || !vf::isFinite(pose)) {
    return SampleResult::kInvalid;
  }
  if (has_latest_) {
    if (stamp == latest_stamp_) {
      return SampleResult::kDuplicate;
    }
    if (stamp < latest_stamp_) {
      return SampleResult::kInvalid;
    }
  }
  const double dt = stamp - base_stamp_;
  if (!has_base_ || dt > config_.max_dt) {
    // First sample, or a gap: restart (no twist across the gap).
    has_base_ = true;
    base_stamp_ = stamp;
    base_pose_ = pose;
    has_latest_ = true;
    latest_stamp_ = stamp;
    latest_pose_ = pose;
    has_twist_ = false;
    twist_ = vf::Twist2{};
    return SampleResult::kStarted;
  }
  if (dt < config_.min_dt) {
    has_latest_ = true;
    latest_stamp_ = stamp;
    latest_pose_ = pose;
    return SampleResult::kTooClose;
  }
  const vf::Twist2 twist = bodyTwist(base_pose_, pose, dt);
  if (isOutlier(twist)) {
    ++outliers_;
    return SampleResult::kOutlier;
  }
  filter(twist, dt);
  base_stamp_ = stamp;
  base_pose_ = pose;
  latest_stamp_ = stamp;
  latest_pose_ = pose;
  return SampleResult::kAccepted;
}

SampleResult FrameTwistEstimator::addOdometry(
  double stamp, const vf::Pose2 & pose, const vf::Twist2 & twist) noexcept
{
  if (!std::isfinite(stamp) || !vf::isFinite(pose)) {
    return SampleResult::kInvalid;
  }
  if (has_latest_) {
    if (stamp == latest_stamp_) {
      return SampleResult::kDuplicate;
    }
    if (stamp < latest_stamp_) {
      return SampleResult::kInvalid;
    }
  }
  if (isOutlier(twist)) {
    ++outliers_;
    return SampleResult::kOutlier;
  }
  has_latest_ = true;
  latest_stamp_ = stamp;
  latest_pose_ = pose;
  twist_ = twist;
  has_twist_ = true;
  return SampleResult::kAccepted;
}

FrameEstimate FrameTwistEstimator::estimate(double now) const noexcept
{
  FrameEstimate out;
  if (!has_latest_) {
    return out;
  }
  out.age = now - latest_stamp_;
  out.pose = latest_pose_;
  out.ready = has_twist_;
  if (!has_twist_ || !(out.age <= config_.input_timeout)) {
    out.stale = true;
    return out;
  }
  out.stale = false;
  out.twist = twist_;
  out.pose = vf::sim::integrateTwist(latest_pose_, twist_, std::max(0.0, out.age));
  return out;
}

}  // namespace vectorfield_curve_follower
