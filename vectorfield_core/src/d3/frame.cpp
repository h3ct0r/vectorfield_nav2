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

// Moving curve frames: SE(3) exponential and logarithm, FrameTwistEstimator3
// (math_spec §14.7 [Design, 0.3.0]).

#include "vectorfield_core/d3/frame.hpp"

#include <algorithm>
#include <cmath>

namespace vectorfield_core::d3
{

namespace
{

/// Below this rotation angle the coefficients use their Taylor series (truncation < 1e-17).
constexpr double kSeriesAngle = 1e-2;
/// Tolerance of the rotation check of isRigid().
constexpr double kRigidTolerance = 1e-6;

}  // namespace

bool isRigid(const Isometry3 & T) noexcept
{
  if (!T.matrix().allFinite()) {
    return false;
  }
  const Mat3 R = T.linear();
  return (R.transpose() * R - Mat3::Identity()).cwiseAbs().maxCoeff() <= kRigidTolerance &&
         R.determinant() > 0.0;
}

Mat3 skew(const Vec3 & w) noexcept
{
  Mat3 K;
  K << 0.0, -w.z(), w.y(),  //
    w.z(), 0.0, -w.x(),     //
    -w.y(), w.x(), 0.0;
  return K;
}

Isometry3 expSE3(const FrameTwist3 & xi, double tau) noexcept
{
  const Vec3 phi = tau * xi.w;
  const Vec3 rho = tau * xi.v;
  const double theta = phi.norm();
  // R = I + a K + b K^2, V = I + b K + c K^2 with a = sin(θ)/θ, b = (1 - cos θ)/θ²,
  // c = (θ - sin θ)/θ³; b is computed as (sin(θ/2)/(θ/2))²/2, free of cancellation.
  double a = 1.0;
  double b = 0.5;
  double c = 1.0 / 6.0;
  if (theta > 0.0) {
    const double half = 0.5 * theta;
    const double sinc_half = std::sin(half) / half;
    a = std::sin(theta) / theta;
    b = 0.5 * sinc_half * sinc_half;
    const double t2 = theta * theta;
    c = theta < kSeriesAngle ? 1.0 / 6.0 - t2 / 120.0 + t2 * t2 / 5040.0
                             : (theta - std::sin(theta)) / (t2 * theta);
  }
  const Mat3 K = skew(phi);
  const Mat3 K2 = K * K;
  Isometry3 T = Isometry3::Identity();
  T.linear() = Mat3::Identity() + a * K + b * K2;
  T.translation() = (Mat3::Identity() + b * K + c * K2) * rho;
  return T;
}

FrameTwist3 logSE3(const Isometry3 & T) noexcept
{
  const Eigen::AngleAxisd aa(Eigen::Quaterniond(Mat3(T.linear())));
  const double theta = aa.angle();
  const Vec3 phi = theta * aa.axis();
  // V^{-1} = I - K/2 + d K^2 with d = (1 - (θ/2) cot(θ/2))/θ².
  const double t2 = theta * theta;
  const double half = 0.5 * theta;
  const double d = theta < kSeriesAngle ? 1.0 / 12.0 + t2 / 720.0 + t2 * t2 / 30240.0
                                        : (1.0 - half * std::cos(half) / std::sin(half)) / t2;
  const Mat3 K = skew(phi);
  FrameTwist3 xi;
  xi.w = phi;
  xi.v = (Mat3::Identity() - 0.5 * K + d * (K * K)) * T.translation();
  return xi;
}

FrameTwist3 bodyTwist(const Isometry3 & a, const Isometry3 & b, double dt) noexcept
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return FrameTwist3{};
  }
  // a^{-1} b for a rigid a: (R_aᵀ R_b, R_aᵀ (c_b - c_a)).
  Isometry3 relative = Isometry3::Identity();
  relative.linear() = a.linear().transpose() * b.linear();
  relative.translation() = a.linear().transpose() * (b.translation() - a.translation());
  FrameTwist3 xi = logSE3(relative);
  xi.v /= dt;
  xi.w /= dt;
  return xi;
}

Isometry3 extrapolate(const CurveFrame3 & frame, double tau) noexcept
{
  if (!frame.moving || tau == 0.0 || !std::isfinite(tau)) {
    return frame.T_eval_curve;
  }
  return frame.T_eval_curve * expSE3(frame.twist, tau);
}

Vec3 frameVelocity(const CurveFrame3 & frame, const Vec3 & x_curve) noexcept
{
  if (!frame.moving) {
    return Vec3::Zero();
  }
  return frame.T_eval_curve.linear() * (frame.twist.v + frame.twist.w.cross(x_curve));
}

// -----------------------------------------------------------------------------------------------
// FrameTwistEstimator3

FrameTwistEstimator3::FrameTwistEstimator3(const FrameTwistConfig3 & config) noexcept
: config_(config)
{
}

void FrameTwistEstimator3::reset() noexcept
{
  has_latest_ = false;
  has_base_ = false;
  has_twist_ = false;
  twist_ = FrameTwist3{};
  outliers_ = 0;
}

bool FrameTwistEstimator3::isOutlier(const FrameTwist3 & twist) const noexcept
{
  return !isFinite(twist) || twist.v.norm() > config_.max_speed ||
         twist.w.norm() > config_.max_angular_rate;
}

void FrameTwistEstimator3::filter(const FrameTwist3 & twist, double dt) noexcept
{
  if (!has_twist_ || !(config_.cutoff_hz > 0.0)) {
    twist_ = twist;
    has_twist_ = true;
    return;
  }
  const double alpha = 1.0 - std::exp(-2.0 * kPi * config_.cutoff_hz * dt);
  twist_.v += alpha * (twist.v - twist_.v);
  twist_.w += alpha * (twist.w - twist_.w);
}

FrameSampleResult3 FrameTwistEstimator3::checkStamp(
  double stamp, const Isometry3 & pose) const noexcept
{
  if (!std::isfinite(stamp) || !isRigid(pose)) {
    return FrameSampleResult3::kInvalid;
  }
  if (has_latest_) {
    if (stamp == latest_stamp_) {
      return FrameSampleResult3::kDuplicate;
    }
    if (stamp < latest_stamp_) {
      return FrameSampleResult3::kInvalid;
    }
  }
  return FrameSampleResult3::kAccepted;
}

FrameSampleResult3 FrameTwistEstimator3::addPose(double stamp, const Isometry3 & pose) noexcept
{
  const FrameSampleResult3 check = checkStamp(stamp, pose);
  if (check != FrameSampleResult3::kAccepted) {
    return check;
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
    twist_ = FrameTwist3{};
    return FrameSampleResult3::kStarted;
  }
  if (dt < config_.min_dt) {
    has_latest_ = true;
    latest_stamp_ = stamp;
    latest_pose_ = pose;
    return FrameSampleResult3::kTooClose;
  }
  const FrameTwist3 twist = bodyTwist(base_pose_, pose, dt);
  if (isOutlier(twist)) {
    ++outliers_;
    return FrameSampleResult3::kOutlier;
  }
  filter(twist, dt);
  base_stamp_ = stamp;
  base_pose_ = pose;
  latest_stamp_ = stamp;
  latest_pose_ = pose;
  return FrameSampleResult3::kAccepted;
}

FrameSampleResult3 FrameTwistEstimator3::addTwist(
  double stamp, const Isometry3 & pose, const FrameTwist3 & twist) noexcept
{
  const FrameSampleResult3 check = checkStamp(stamp, pose);
  if (check != FrameSampleResult3::kAccepted) {
    return check;
  }
  if (isOutlier(twist)) {
    ++outliers_;
    return FrameSampleResult3::kOutlier;
  }
  has_latest_ = true;
  latest_stamp_ = stamp;
  latest_pose_ = pose;
  twist_ = twist;
  has_twist_ = true;
  return FrameSampleResult3::kAccepted;
}

FrameEstimate3 FrameTwistEstimator3::estimate(double now) const noexcept
{
  FrameEstimate3 out;
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
  out.pose = latest_pose_ * expSE3(twist_, std::max(0.0, out.age));
  return out;
}

}  // namespace vectorfield_core::d3
