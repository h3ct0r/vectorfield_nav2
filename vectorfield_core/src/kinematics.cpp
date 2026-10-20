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

#include "vectorfield_core/kinematics.hpp"

#include <cmath>

namespace vectorfield_core
{

Kinematics Kinematics::fromConfig(const CoreConfig & config) noexcept
{
  const MotionConfig & motion = config.motion;
  Kinematics kinematics;
  kinematics.type = motion.motion_model;
  kinematics.max_linear_vel = motion.max_linear_vel;
  kinematics.max_lateral_vel = motion.max_lateral_vel;
  kinematics.max_angular_vel = motion.max_angular_vel;
  kinematics.max_linear_accel = motion.max_linear_accel;
  kinematics.max_linear_decel = motion.max_linear_decel;
  kinematics.max_angular_accel = motion.max_angular_accel;
  kinematics.cancel_deceleration = config.phases.cancel_deceleration;
  kinematics.control_point_offset = motion.control_point_offset;
  kinematics.reference_point_offset = motion.reference_point_offset;
  kinematics.min_turning_radius = motion.min_turning_radius;
  kinematics.ackermann_min_speed = motion.ackermann_min_speed;
  kinematics.drive_backward = motion.drive_backward;
  kinematics.allow_reversing = motion.allow_reversing;
  kinematics.skid_yaw_rate_efficiency = motion.skid_yaw_rate_efficiency;
  kinematics.skid_icr_offset = motion.skid_icr_offset;
  return kinematics;
}

Twist2 Kinematics::slipCompensated(
  const Twist2 & command, double lever, bool * saturated) const noexcept
{
  Twist2 compensated = command;
  bool clamped = false;
  // validateConfig() keeps eta in (0, 1] and |x_ICR| < d / 2; eta = 1, x_ICR = 0 is the identity.
  const double eta = skid_yaw_rate_efficiency;
  const double arm = lever - skid_icr_offset;
  const bool icr = lever != 0.0 && skid_icr_offset != 0.0 && lever * arm > 0.0;
  if (type == MotionModel::kSkidSteer && eta > 0.0 && (eta < 1.0 || icr)) {
    const double omega = command.wz * (icr ? lever / arm : 1.0) / eta;
    clamped = std::abs(omega) > max_angular_vel;
    compensated.wz = clamp(omega, -max_angular_vel, max_angular_vel);
  }
  if (saturated != nullptr) {
    *saturated = clamped;
  }
  return compensated;
}

namespace sim
{

namespace
{

// Coefficients of the exact arc: a = sin(w dt)/w and b = (1 - cos(w dt))/w, so the body-frame
// displacement is a v + b J v. Below |w dt| = 1e-4 the Taylor series is used (truncation below
// 1e-28 relative); above, b uses 2 sin^2(x/2)/w, which has no cancellation.
void arcCoefficients(double w, double dt, double * a, double * b) noexcept
{
  const double x = w * dt;
  if (std::abs(x) < 1e-4) {
    const double x2 = x * x;
    *a = dt * (1.0 - x2 / 6.0 * (1.0 - x2 / 20.0));
    *b = dt * (0.5 * x) * (1.0 - x2 / 12.0 * (1.0 - x2 / 30.0));
    return;
  }
  const double half_sine = std::sin(0.5 * x);
  *a = std::sin(x) / w;
  *b = 2.0 * half_sine * half_sine / w;
}

}  // namespace

Twist2 feasibleTwist(const Kinematics & kinematics, const Twist2 & command) noexcept
{
  switch (kinematics.type) {
    case MotionModel::kOmnidirectional:
      return command;
    case MotionModel::kDifferential:
    case MotionModel::kSkidSteer:
      return Twist2{command.vx, 0.0, command.wz};
    case MotionModel::kAckermann: {
      Twist2 rear{command.vx, 0.0, command.wz};
      if (kinematics.min_turning_radius > 0.0) {
        const double limit = std::abs(command.vx) / kinematics.min_turning_radius;
        rear.wz = clamp(rear.wz, -limit, limit);
      }
      return rear;
    }
  }
  return command;
}

Pose2 integrateTwist(const Pose2 & pose, const Twist2 & twist, double dt) noexcept
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return pose;
  }
  double a = 0.0;
  double b = 0.0;
  arcCoefficients(twist.wz, dt, &a, &b);
  const Vec2 v(twist.vx, twist.vy);
  const Vec2 body = a * v + b * J(v);
  return Pose2{pose.p + rotate(pose.yaw, body), pose.yaw + twist.wz * dt};
}

Pose2 integrate(
  const Kinematics & kinematics, const Pose2 & pose, const Twist2 & command, double dt,
  const SlipParams & slip) noexcept
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return pose;
  }
  Twist2 twist = feasibleTwist(kinematics, command);
  twist.wz *= slip.eta_psi;
  twist.vy -= slip.x_icr * twist.wz;

  if (kinematics.type != MotionModel::kAckermann) {
    return integrateTwist(pose, twist, dt);
  }
  // Bicycle: integrate the rear axle (no lateral velocity), then map back to base_link.
  const double x_r = kinematics.reference_point_offset;
  const Pose2 rear{pose.p + x_r * heading(pose.yaw), pose.yaw};
  const Pose2 rear_end = integrateTwist(rear, twist, dt);
  return Pose2{rear_end.p - x_r * heading(rear_end.yaw), rear_end.yaw};
}

}  // namespace sim

}  // namespace vectorfield_core
