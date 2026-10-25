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

// Kinematics and exact planar integration (math_spec §10.2, §7.5).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::Kinematics;
using vf::MotionModel;
using vf::Pose2;
using vf::Twist2;
using vf::Vec2;

namespace
{

// Neumaier (improved Kahan) summation.
class CompensatedSum
{
public:
  void add(double x)
  {
    const double t = sum_ + x;
    if (std::abs(sum_) >= std::abs(x)) {
      compensation_ += (sum_ - t) + x;
    } else {
      compensation_ += (x - t) + sum_;
    }
    sum_ = t;
  }
  double value() const { return sum_ + compensation_; }

private:
  double sum_ = 0.0;
  double compensation_ = 0.0;
};

// Reference: p(dt) = p + integral of R(psi + w tau) (vx, vy) over [0, dt] by composite Simpson
// with `intervals` (even) intervals and compensated sums.
Pose2 quadrature(const Pose2 & pose, const Twist2 & twist, double dt, int intervals)
{
  const double h = dt / intervals;
  CompensatedSum sum_x;
  CompensatedSum sum_y;
  const Vec2 v(twist.vx, twist.vy);
  for (int i = 0; i <= intervals; ++i) {
    double weight = 2.0;
    if (i == 0 || i == intervals) {
      weight = 1.0;
    } else if (i % 2 == 1) {
      weight = 4.0;
    }
    const Vec2 velocity = vf::rotate(pose.yaw + twist.wz * (h * i), v);
    sum_x.add(weight * velocity.x());
    sum_y.add(weight * velocity.y());
  }
  return Pose2{pose.p + (h / 3.0) * Vec2(sum_x.value(), sum_y.value()), pose.yaw + twist.wz * dt};
}

double distance(const Pose2 & a, const Pose2 & b)
{
  return std::max((a.p - b.p).norm(), std::abs(a.yaw - b.yaw));
}

Kinematics model(MotionModel type)
{
  Kinematics kinematics;
  kinematics.type = type;
  return kinematics;
}

}  // namespace

TEST(Kinematics, FromConfigCopiesTheMotionGroup)
{
  vf::CoreConfig config;
  config.motion.motion_model = MotionModel::kAckermann;
  config.motion.max_linear_vel = 0.7;
  config.motion.max_lateral_vel = 0.3;
  config.motion.max_angular_vel = 1.1;
  config.motion.max_linear_accel = 0.9;
  config.motion.max_linear_decel = 1.2;
  config.motion.max_angular_accel = 2.5;
  config.motion.control_point_offset = 0.25;
  config.motion.reference_point_offset = -0.3;
  config.motion.min_turning_radius = 0.8;
  config.motion.ackermann_min_speed = 0.07;
  config.motion.drive_backward = true;
  config.motion.allow_reversing = true;
  config.phases.cancel_deceleration = 0.4;

  const Kinematics k = Kinematics::fromConfig(config);
  EXPECT_EQ(k.type, MotionModel::kAckermann);
  EXPECT_DOUBLE_EQ(k.max_linear_vel, 0.7);
  EXPECT_DOUBLE_EQ(k.max_lateral_vel, 0.3);
  EXPECT_DOUBLE_EQ(k.max_angular_vel, 1.1);
  EXPECT_DOUBLE_EQ(k.max_linear_accel, 0.9);
  EXPECT_DOUBLE_EQ(k.max_linear_decel, 1.2);
  EXPECT_DOUBLE_EQ(k.max_angular_accel, 2.5);
  EXPECT_DOUBLE_EQ(k.cancel_deceleration, 0.4);
  EXPECT_DOUBLE_EQ(k.control_point_offset, 0.25);
  EXPECT_DOUBLE_EQ(k.reference_point_offset, -0.3);
  EXPECT_DOUBLE_EQ(k.min_turning_radius, 0.8);
  EXPECT_DOUBLE_EQ(k.ackermann_min_speed, 0.07);
  // Ackermann ignores the configured drive_backward; only plan segments reverse it.
  EXPECT_FALSE(k.drive_backward);
  EXPECT_TRUE(k.allow_reversing);
  config.motion.motion_model = MotionModel::kDifferential;
  EXPECT_TRUE(Kinematics::fromConfig(config).drive_backward);
  EXPECT_DOUBLE_EQ(k.driveSign(), 1.0);
  EXPECT_TRUE(k.isAckermann());
  EXPECT_FALSE(k.isHolonomic());
  EXPECT_FALSE(k.isUnicycle());
  EXPECT_DOUBLE_EQ(k.angularLimit(0.5), 0.55);
  EXPECT_DOUBLE_EQ(k.skid_yaw_rate_efficiency, 1.0);
  EXPECT_DOUBLE_EQ(k.skid_icr_offset, 0.0);

  config.motion.skid_yaw_rate_efficiency = 0.8;
  config.motion.skid_icr_offset = 0.04;
  EXPECT_DOUBLE_EQ(Kinematics::fromConfig(config).skid_yaw_rate_efficiency, 0.8);
  EXPECT_DOUBLE_EQ(Kinematics::fromConfig(config).skid_icr_offset, 0.04);
}

// math_spec §8.4 [Design, 0.1.1]: omega_cmd = clamp(lambda omega / eta, +-omega_max), lambda =
// s / (s - x_ICR) for a steered point s != 0; the identity off skid steer or when off.
TEST(Kinematics, SkidSlipCompensation)
{
  Kinematics skid;
  skid.type = MotionModel::kSkidSteer;
  skid.max_angular_vel = 1.5;
  bool saturated = true;
  const vf::Twist2 u{0.3, 0.0, 0.6};
  vf::Twist2 c = skid.slipCompensated(u, 0.2, &saturated);
  EXPECT_DOUBLE_EQ(c.wz, 0.6) << "off by default";
  EXPECT_FALSE(saturated);
  skid.skid_yaw_rate_efficiency = 0.75;
  c = skid.slipCompensated(u, 0.0, &saturated);
  EXPECT_DOUBLE_EQ(c.vx, 0.3);
  EXPECT_DOUBLE_EQ(c.wz, 0.8);
  EXPECT_FALSE(saturated);
  skid.skid_icr_offset = 0.05;
  EXPECT_DOUBLE_EQ(skid.slipCompensated(u, 0.0).wz, 0.8) << "no steered point: eta only";
  EXPECT_DOUBLE_EQ(skid.slipCompensated(u, 0.2).wz, 0.6 * (0.2 / 0.15) / 0.75);
  EXPECT_DOUBLE_EQ(skid.slipCompensated(u, -0.2).wz, 0.6 * (0.2 / 0.25) / 0.75) << "q = -1";
  c = skid.slipCompensated(vf::Twist2{0.0, 0.0, -1.4}, 0.0, &saturated);
  EXPECT_DOUBLE_EQ(c.wz, -1.5) << "clamped to omega_max";
  EXPECT_TRUE(saturated);

  Kinematics diff = skid;
  diff.type = MotionModel::kDifferential;
  c = diff.slipCompensated(u, 0.2, &saturated);
  EXPECT_DOUBLE_EQ(c.wz, 0.6);
  EXPECT_FALSE(saturated);
}

// math_spec §10.2 and Appendix B: the exact SE(2) step agrees with a 2e5-step quadrature.
TEST(Integrate, ExactArcMatchesQuadrature)
{
  const std::vector<double> rates{0.0,      1e-300, 1e-12, 1e-9, 1e-6, 1e-5, 0.999e-4,
                                  1.001e-4, 2e-4,   0.3,   -1.7, 5.0,  -25.0};
  const std::vector<double> durations{0.05, 1.0, 2.0};
  double worst = 0.0;
  for (double w : rates) {
    for (double dt : durations) {
      const Pose2 start{Vec2(1.5, -0.5), 2.6};
      const Twist2 twist{0.45, -0.2, w};
      const Pose2 exact = vf::sim::integrateTwist(start, twist, dt);
      const Pose2 reference = quadrature(start, twist, dt, 200000);
      worst = std::max(worst, distance(exact, reference));
      ASSERT_LE(distance(exact, reference), 1e-12) << "w = " << w << ", dt = " << dt;

      // The omnidirectional plant integrates the same twist.
      const Pose2 omni = vf::sim::integrate(model(MotionModel::kOmnidirectional), start, twist, dt);
      ASSERT_LE(distance(omni, exact), 0.0) << "w = " << w << ", dt = " << dt;
    }
  }
  char text[32];
  std::snprintf(text, sizeof(text), "%.3e", worst);
  RecordProperty("max_error", text);
}

TEST(Integrate, SmallRateLimitIsTheStraightLine)
{
  const Pose2 start{Vec2(-1.0, 2.0), -0.4};
  const Twist2 straight{0.3, 0.1, 0.0};
  const double dt = 0.5;
  const Pose2 line{start.p + dt * vf::rotate(start.yaw, Vec2(0.3, 0.1)), start.yaw};
  EXPECT_LE(distance(vf::sim::integrateTwist(start, straight, dt), line), 1e-16);
  for (double w : {1e-300, 1e-15, 1e-12}) {
    const Twist2 twist{0.3, 0.1, w};
    EXPECT_LE(distance(vf::sim::integrateTwist(start, twist, dt), line), 1e-12) << "w = " << w;
  }
  // Continuity across the series branch at |w dt| = 1e-4: the two sides differ only by the
  // first-order effect of their 4e-13 rad/s rate difference (about 2e-14 m), not by a jump.
  const double w_switch = 1e-4 / dt;
  const double w_below = w_switch * (1 - 1e-9);
  const double w_above = w_switch * (1 + 1e-9);
  const Pose2 below = vf::sim::integrateTwist(start, Twist2{0.3, 0.1, w_below}, dt);
  const Pose2 above = vf::sim::integrateTwist(start, Twist2{0.3, 0.1, w_above}, dt);
  EXPECT_LE((below.p - above.p).norm(), 1e-13);
  EXPECT_NEAR(above.yaw - below.yaw, (w_above - w_below) * dt, 1e-16);
}

TEST(Integrate, ConstantTwistComposes)
{
  const Pose2 start{Vec2(0.2, 0.1), 1.0};
  const Twist2 twist{0.4, 0.05, -0.9};
  const Pose2 split =
    vf::sim::integrateTwist(vf::sim::integrateTwist(start, twist, 0.3), twist, 0.45);
  const Pose2 joint = vf::sim::integrateTwist(start, twist, 0.75);
  EXPECT_LE(distance(split, joint), 1e-12);
}

TEST(Integrate, NonPositiveOrNonFiniteDurationKeepsThePose)
{
  const Pose2 start{Vec2(0.2, 0.1), 1.0};
  const Twist2 twist{0.4, 0.0, 0.3};
  for (double dt : {0.0, -0.1, vf::kNaN, vf::kInf}) {
    const Pose2 kept = vf::sim::integrate(model(MotionModel::kDifferential), start, twist, dt);
    EXPECT_EQ(kept.p, start.p);
    EXPECT_EQ(kept.yaw, start.yaw);
  }
  const Pose2 broken = vf::sim::integrateTwist(start, Twist2{vf::kNaN, 0.0, 0.0}, 0.1);
  EXPECT_FALSE(vf::isFinite(broken));
}

TEST(Integrate, UnicycleDropsLateralVelocity)
{
  const Pose2 start{Vec2(0.0, 0.0), 0.3};
  const Twist2 command{0.3, 0.2, 0.4};
  for (MotionModel type : {MotionModel::kDifferential, MotionModel::kSkidSteer}) {
    const Pose2 moved = vf::sim::integrate(model(type), start, command, 0.5);
    const Pose2 expected = vf::sim::integrateTwist(start, Twist2{0.3, 0.0, 0.4}, 0.5);
    EXPECT_LE(distance(moved, expected), 0.0);
  }
  const Pose2 omni = vf::sim::integrate(model(MotionModel::kOmnidirectional), start, command, 0.5);
  EXPECT_LE(distance(omni, vf::sim::integrateTwist(start, command, 0.5)), 0.0);
}

// math_spec §7.5 and §10.2: the bicycle is integrated at the rear axle (no lateral velocity) and
// mapped back to base_link; base_link then has lateral velocity -x_r w.
TEST(Integrate, BicycleIntegratesTheRearAxle)
{
  Kinematics car = model(MotionModel::kAckermann);
  car.min_turning_radius = 0.5;
  const Pose2 start{Vec2(1.0, -1.0), 0.7};
  const double dt = 0.8;
  for (double x_r : {-0.3, 0.0, 0.25}) {
    car.reference_point_offset = x_r;
    const Twist2 command{0.4, 0.0, 0.5};  // |w| <= |v|/R_min = 0.8: feasible
    const Pose2 base_end = vf::sim::integrate(car, start, command, dt);

    // Rigid body: the base moves with the constant body twist (v, -x_r w, w).
    const Pose2 rigid = vf::sim::integrateTwist(start, Twist2{0.4, -x_r * 0.5, 0.5}, dt);
    EXPECT_LE(distance(base_end, rigid), 1e-12) << "x_r = " << x_r;

    // The rear axle follows the unicycle arc of radius v/w.
    const Pose2 rear_start{start.p + x_r * vf::heading(start.yaw), start.yaw};
    const Pose2 rear_end = vf::sim::integrateTwist(rear_start, Twist2{0.4, 0.0, 0.5}, dt);
    EXPECT_LE((base_end.p + x_r * vf::heading(base_end.yaw) - rear_end.p).norm(), 1e-12);
    const Vec2 centre = rear_start.p + (0.4 / 0.5) * vf::J(vf::heading(start.yaw));
    EXPECT_NEAR((rear_end.p - centre).norm(), 0.8, 1e-12);
  }

  // Curvature saturation: |w| is limited to |v|/R_min; a car cannot turn in place.
  car.reference_point_offset = -0.3;
  const Pose2 saturated = vf::sim::integrate(car, start, Twist2{0.4, 0.0, 2.0}, dt);
  const Pose2 limited = vf::sim::integrate(car, start, Twist2{0.4, 0.0, 0.8}, dt);
  EXPECT_LE(distance(saturated, limited), 0.0);
  const Pose2 in_place = vf::sim::integrate(car, start, Twist2{0.0, 0.0, 1.0}, dt);
  EXPECT_LE(distance(in_place, start), 1e-15);
  const Pose2 lateral = vf::sim::integrate(car, start, Twist2{0.4, 0.3, 0.5}, dt);
  EXPECT_LE(distance(lateral, vf::sim::integrate(car, start, Twist2{0.4, 0.0, 0.5}, dt)), 0.0);

  const Twist2 feasible = vf::sim::feasibleTwist(car, Twist2{-0.2, 0.1, -3.0});
  EXPECT_DOUBLE_EQ(feasible.vx, -0.2);
  EXPECT_DOUBLE_EQ(feasible.vy, 0.0);
  EXPECT_DOUBLE_EQ(feasible.wz, -0.4);
}

TEST(Integrate, SlipScalesYawRateAndShiftsTheIcr)
{
  const vf::sim::SlipParams slip{0.75, 0.05};
  const Pose2 start{Vec2(0.0, 0.5), -1.2};
  const Twist2 command{0.3, 0.0, 0.8};
  const Pose2 slipped =
    vf::sim::integrate(model(MotionModel::kSkidSteer), start, command, 0.6, slip);
  // w' = 0.75 * 0.8 = 0.6 and vy' = -x_icr w' = -0.03.
  const Pose2 expected = vf::sim::integrateTwist(start, Twist2{0.3, -0.03, 0.6}, 0.6);
  EXPECT_LE(distance(slipped, expected), 1e-15);

  const Pose2 nominal = vf::sim::integrate(model(MotionModel::kSkidSteer), start, command, 0.6);
  EXPECT_LE(distance(nominal, vf::sim::integrateTwist(start, command, 0.6)), 0.0);
}
