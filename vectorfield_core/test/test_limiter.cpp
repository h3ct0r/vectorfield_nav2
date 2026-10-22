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

// Speed-shaping terms, the kinematic box and the CommandLimiter (math_spec §8.2–§8.4): oracles
// O44 and O48, the law/limiter parts of O45, O46, O47 and O49, and UpstreamRegression.U25.
//
// Closest points are built analytically (lines and circles, a translating circle for the
// time-varying cases), so these tests do not depend on the curve or Tracker implementations.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/limiter.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracking_law.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::BoxResult;
using vf::ClosestPoint;
using vf::CommandLimiter;
using vf::DistanceField;
using vf::FieldSample;
using vf::GainFunction;
using vf::GainType;
using vf::Interval;
using vf::Kinematics;
using vf::LimiterState;
using vf::MotionModel;
using vf::Phase;
using vf::Pose2;
using vf::ShapingTerm;
using vf::SpeedShaping;
using vf::Status;
using vf::TrackingLaw;
using vf::Twist2;
using vf::Vec2;

namespace
{

constexpr double kVr = 0.4;
constexpr double kDt = 0.05;

class Rng
{
public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}
  double uniform(double lo, double hi)
  {
    return std::uniform_real_distribution<double>(lo, hi)(engine_);
  }
  bool chance(double p) { return uniform(0.0, 1.0) < p; }
  bool coin() { return chance(0.5); }
  Vec2 unit() { return vf::heading(uniform(-vf::kPi, vf::kPi)); }

private:
  std::mt19937_64 engine_;
};

ClosestPoint lineClosestPoint(const Vec2 & x, const Vec2 & p0, const Vec2 & t)
{
  ClosestPoint cp;
  const Vec2 left = vf::J(t);
  const double y = (x - p0).dot(left);
  cp.T = t;
  cp.D = std::abs(y);
  cp.n = cp.D < vf::kEpsD ? Vec2(Vec2::Zero()) : Vec2(vf::sgn(y) * left);
  cp.x_star = x - y * left;
  cp.s_star = (x - p0).dot(t);
  cp.u_star = cp.s_star;
  cp.valid = true;
  return cp;
}

ClosestPoint circleClosestPoint(const Vec2 & x, const Vec2 & c, double R)
{
  ClosestPoint cp;
  const Vec2 r = x - c;
  const double rho = r.norm();
  const Vec2 u = r / rho;
  cp.x_star = c + R * u;
  cp.D = std::abs(rho - R);
  cp.n = cp.D < vf::kEpsD ? Vec2(Vec2::Zero()) : Vec2(vf::sgn(rho - R) * u);
  cp.T = vf::J(u);
  double angle = std::atan2(u.y(), u.x());
  if (angle < 0.0) {
    angle += vf::kTwoPi;
  }
  cp.s_star = R * angle;
  cp.u_star = angle;
  cp.valid = true;
  return cp;
}

// u(eta) = a + eta b on the box axes: (v, omega) for FL, (v_x, v_y) for holonomic.
Vec2 boxAxes(const Twist2 & cmd, bool holonomic)
{
  return holonomic ? Vec2(cmd.vx, cmd.vy) : Vec2(cmd.vx, cmd.wz);
}

// The time-varying sample of a circle of radius R translating at `velocity` (v_c = velocity),
// evaluated at x.
FieldSample translatingCircleSample(
  const DistanceField & field, const Vec2 & x, const Vec2 & centre, double R, const Vec2 & velocity)
{
  FieldSample sample;
  field.evaluate(x, circleClosestPoint(x, centre, R), velocity, &sample);
  return sample;
}

Kinematics omniKinematics()
{
  Kinematics k;
  k.type = MotionModel::kOmnidirectional;
  return k;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Speed-shaping terms (math_spec §8.2, §9.1, §10.3).

TEST(LimiterShaping, GammaAndBindingTerm)
{
  SpeedShaping shaping;
  EXPECT_EQ(shaping.gamma(), 1.0);
  EXPECT_EQ(shaping.binding(), ShapingTerm::kNone);
  EXPECT_FALSE(shaping.capBinds());
  EXPECT_FALSE(shaping.collisionAhead());

  shaping.gamma_lim = 0.5;
  shaping.gamma_cl = 0.7;
  shaping.gamma_ap = 0.9;
  shaping.gamma_col = 0.8;
  EXPECT_EQ(shaping.gamma(), 0.5);
  EXPECT_EQ(shaping.binding(), ShapingTerm::kSpeedLimit);
  EXPECT_FALSE(shaping.collisionAhead());

  shaping.gamma_col = 0.3;
  EXPECT_EQ(shaping.gamma(), 0.3);
  EXPECT_EQ(shaping.binding(), ShapingTerm::kCollision);
  EXPECT_TRUE(shaping.capBinds());
  EXPECT_TRUE(shaping.collisionAhead());

  shaping.gamma_col = 1.0;
  shaping.gamma_cl = 0.2;
  EXPECT_EQ(shaping.binding(), ShapingTerm::kClearance);
  EXPECT_FALSE(shaping.capBinds());
  EXPECT_TRUE(shaping.collisionAhead()) << "STATUS_COLLISION_AHEAD also for the clearance term";

  shaping.gamma_cl = 1.0;
  shaping.gamma_ap = 0.1;
  EXPECT_EQ(shaping.binding(), ShapingTerm::kApproach);
  EXPECT_FALSE(shaping.collisionAhead());
}

// Ties resolve in the order limit, approach, clearance, collision: the cap binds only when it is
// strictly the smallest term (math_spec §9.1 [Spec]).
TEST(LimiterShaping, TiesNeverBlameTheCollisionCap)
{
  SpeedShaping shaping;
  shaping.gamma_lim = 0.4;
  shaping.gamma_col = 0.4;
  EXPECT_EQ(shaping.binding(), ShapingTerm::kSpeedLimit);
  EXPECT_FALSE(shaping.capBinds());
  EXPECT_FALSE(shaping.collisionAhead());

  shaping.gamma_lim = 1.0;
  shaping.gamma_ap = 0.4;
  EXPECT_EQ(shaping.binding(), ShapingTerm::kApproach);
  EXPECT_FALSE(shaping.capBinds());

  shaping.gamma_ap = 1.0;
  shaping.gamma_cl = 0.4;
  EXPECT_EQ(shaping.binding(), ShapingTerm::kClearance);
  EXPECT_FALSE(shaping.capBinds());

  // A cap at 1 never binds.
  SpeedShaping free_path;
  free_path.gamma_col = 1.0;
  EXPECT_FALSE(free_path.capBinds());

  // Randomised: capBinds() iff the binding term is the collision cap.
  Rng rng(92);
  for (int i = 0; i < 20000; ++i) {
    SpeedShaping s;
    const double levels[] = {0.2, 0.5, 1.0};
    s.gamma_lim = levels[static_cast<int>(rng.uniform(0.0, 3.0))];
    s.gamma_cl = levels[static_cast<int>(rng.uniform(0.0, 3.0))];
    s.gamma_ap = levels[static_cast<int>(rng.uniform(0.0, 3.0))];
    s.gamma_col = levels[static_cast<int>(rng.uniform(0.0, 3.0))];
    ASSERT_EQ(s.capBinds(), s.binding() == ShapingTerm::kCollision);
    ASSERT_EQ(s.gamma(), std::min({1.0, s.gamma_lim, s.gamma_cl, s.gamma_ap, s.gamma_col}));
  }

  SpeedShaping broken;
  broken.gamma_col = vf::kNaN;
  EXPECT_TRUE(std::isnan(broken.gamma()));
  EXPECT_EQ(broken.binding(), ShapingTerm::kNone);
  EXPECT_FALSE(broken.capBinds());
}

TEST(LimiterShaping, ClearanceApproachAndCollisionScales)
{
  // gamma_cl: 1 when off; min(1, k_cl max(0, EDT - w_lat) / d_cl).
  EXPECT_EQ(vf::clearanceScale(0.3, 0.22, 0.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(vf::clearanceScale(0.42, 0.22, 0.5, 1.0), 0.4);
  EXPECT_DOUBLE_EQ(vf::clearanceScale(0.42, 0.22, 0.5, 2.0), 0.8);
  EXPECT_EQ(vf::clearanceScale(0.1, 0.22, 0.5, 1.0), 0.0);
  EXPECT_EQ(vf::clearanceScale(5.0, 0.22, 0.5, 1.0), 1.0);
  EXPECT_EQ(vf::clearanceScale(vf::kInf, 0.22, 0.5, 1.0), 1.0) << "no obstacle";
  EXPECT_EQ(vf::clearanceScale(vf::kNaN, 0.22, 0.5, 1.0), 0.0) << "conservative";

  // gamma_ap = min(1, sqrt(v_min^2 + 2 a_dec sigma_end) / v_r); 1 for closed curves (NaN).
  EXPECT_EQ(vf::approachScale(vf::kNaN, 0.05, 1.0, kVr), 1.0);
  EXPECT_DOUBLE_EQ(vf::approachScale(0.0, 0.05, 1.0, kVr), 0.05 / kVr);
  EXPECT_DOUBLE_EQ(vf::approachScale(0.02, 0.05, 1.0, kVr), std::sqrt(0.0025 + 0.04) / kVr);
  EXPECT_EQ(vf::approachScale(1.0, 0.05, 1.0, kVr), 1.0);
  EXPECT_EQ(vf::approachScale(vf::kInf, 0.05, 1.0, kVr), 1.0);
  EXPECT_DOUBLE_EQ(vf::approachScale(-0.3, 0.05, 1.0, kVr), 0.05 / kVr) << "max(0, sigma_end)";

  // gamma_col = min(1, v_cap / v_r).
  EXPECT_EQ(vf::collisionScale(vf::kInf, kVr), 1.0);
  EXPECT_DOUBLE_EQ(vf::collisionScale(0.1, kVr), 0.25);
  EXPECT_EQ(vf::collisionScale(0.0, kVr), 0.0);
  EXPECT_EQ(vf::collisionScale(0.9, kVr), 1.0);
  EXPECT_TRUE(std::isnan(vf::collisionScale(vf::kNaN, kVr))) << "a failed prediction";
}

// -----------------------------------------------------------------------------------------------
// Interval arithmetic of the eta-box (math_spec §8.3).

TEST(LimiterEtaBox, AxisIntervals)
{
  Interval i = vf::boxAxisInterval(0.0, 2.0, 1.0);
  EXPECT_DOUBLE_EQ(i.lo, -0.5);
  EXPECT_DOUBLE_EQ(i.hi, 0.5);
  i = vf::boxAxisInterval(1.0, -2.0, 1.5);  // |1 - 2 eta| <= 1.5
  EXPECT_DOUBLE_EQ(i.lo, -0.25);
  EXPECT_DOUBLE_EQ(i.hi, 1.25);
  // b = 0: all of R when |a| <= L, else empty.
  i = vf::boxAxisInterval(0.5, 0.0, 1.0);
  EXPECT_FALSE(i.empty());
  EXPECT_EQ(i.lo, -vf::kInf);
  EXPECT_EQ(i.hi, vf::kInf);
  EXPECT_FALSE(vf::boxAxisInterval(-1.0, 0.0, 1.0).empty());
  EXPECT_TRUE(vf::boxAxisInterval(1.5, 0.0, 1.0).empty());
  EXPECT_TRUE(vf::boxAxisInterval(vf::kNaN, 1.0, 1.0).empty());
  EXPECT_TRUE(vf::boxAxisInterval(0.0, 1.0, -0.1).empty());
  const Interval point = vf::boxAxisInterval(0.3, 2.0, 0.0);
  EXPECT_FALSE(point.empty());
  EXPECT_DOUBLE_EQ(point.lo, -0.15);
  EXPECT_DOUBLE_EQ(point.hi, -0.15);
  // Intersection.
  EXPECT_TRUE((Interval{0.0, 1.0}.intersect(Interval{1.5, 2.0}).empty()));
  const Interval both = Interval{0.0, 1.0}.intersect(Interval{0.5, 2.0});
  EXPECT_EQ(both.lo, 0.5);
  EXPECT_EQ(both.hi, 1.0);

  // Membership equals the constraint.
  Rng rng(83);
  for (int i2 = 0; i2 < 20000; ++i2) {
    const double a = rng.uniform(-2.0, 2.0);
    const double b = rng.chance(0.1) ? 0.0 : rng.uniform(-2.0, 2.0);
    const double L = rng.uniform(0.0, 1.5);
    const Interval interval = vf::boxAxisInterval(a, b, L);
    const double eta = rng.uniform(-5.0, 5.0);
    const double value = std::abs(a + eta * b);
    if (std::abs(value - L) < 1e-9) {
      continue;
    }
    const bool inside = !interval.empty() && eta >= interval.lo && eta <= interval.hi;
    ASSERT_EQ(inside, value <= L) << a << " " << b << " " << L << " " << eta;
    if (!interval.empty() && b != 0.0) {
      ASSERT_NEAR(std::abs(a + interval.lo * b), L, 1e-12);
      ASSERT_NEAR(std::abs(a + interval.hi * b), L, 1e-12);
    }
  }
}

// The omni norm interval: ||a + eta b|| <= L, from the quadratic B eta^2 + 2 A eta + C <= 0.
TEST(LimiterEtaBox, OmniNormInterval)
{
  Interval i = vf::boxNormInterval(Vec2::Zero(), Vec2(1.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(i.lo, -1.0);
  EXPECT_DOUBLE_EQ(i.hi, 1.0);
  i = vf::boxNormInterval(Vec2(2.0, 0.0), Vec2(1.0, 0.0), 1.0);  // |2 + eta| <= 1
  EXPECT_DOUBLE_EQ(i.lo, -3.0);
  EXPECT_DOUBLE_EQ(i.hi, -1.0);
  i = vf::boxNormInterval(Vec2(-2.0, 0.0), Vec2(1.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(i.lo, 1.0);
  EXPECT_DOUBLE_EQ(i.hi, 3.0);
  // The line a + eta b misses the disc: Delta < 0.
  EXPECT_TRUE(vf::boxNormInterval(Vec2(0.0, 2.0), Vec2(1.0, 0.0), 1.0).empty());
  // B = 0: all of R iff ||a|| <= L.
  EXPECT_EQ(vf::boxNormInterval(Vec2(0.3, 0.39), Vec2::Zero(), 0.5).lo, -vf::kInf);
  EXPECT_TRUE(vf::boxNormInterval(Vec2(0.3, 0.41), Vec2::Zero(), 0.5).empty());
  // Tangent line: a single point.
  const Interval touch = vf::boxNormInterval(Vec2(0.0, 1.0), Vec2(1.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(touch.lo, 0.0);
  EXPECT_DOUBLE_EQ(touch.hi, 0.0);

  Rng rng(84);
  for (int k = 0; k < 20000; ++k) {
    const Vec2 a(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
    const Vec2 b =
      rng.chance(0.05) ? Vec2(Vec2::Zero()) : Vec2(rng.uniform(0.01, 2.0) * rng.unit());
    const double L = rng.uniform(0.0, 1.2);
    const Interval interval = vf::boxNormInterval(a, b, L);
    // Against the textbook roots (-A -+ sqrt(Delta)) / B of math_spec §8.3.
    const double B = b.squaredNorm();
    const double A = a.dot(b);
    const double C = a.squaredNorm() - L * L;
    const double Delta = A * A - B * C;
    if (B > 0.0 && Delta >= 0.0) {
      ASSERT_FALSE(interval.empty());
      ASSERT_NEAR(interval.lo, (-A - std::sqrt(Delta)) / B, 1e-12 * (1.0 + std::abs(A) / B));
      ASSERT_NEAR(interval.hi, (-A + std::sqrt(Delta)) / B, 1e-12 * (1.0 + std::abs(A) / B));
      ASSERT_NEAR((a + interval.lo * b).norm(), L, 1e-12);
      ASSERT_NEAR((a + interval.hi * b).norm(), L, 1e-12);
    }
    for (int j = 0; j < 4; ++j) {
      const double eta = rng.uniform(-6.0, 6.0);
      const double norm = (a + eta * b).norm();
      if (std::abs(norm - L) < 1e-9) {
        continue;
      }
      const bool inside = !interval.empty() && eta >= interval.lo && eta <= interval.hi;
      ASSERT_EQ(inside, norm <= L);
    }
  }
}

// -----------------------------------------------------------------------------------------------
// Uniform c (math_spec §8.3).

TEST(LimiterUniformBox, DiffSkidAckermann)
{
  Kinematics diff;
  diff.max_linear_vel = 0.5;
  const double omega_bar = 1.2;
  BoxResult box = vf::uniformBox(Twist2{0.8, 0.0, 0.6}, diff, omega_bar);
  EXPECT_DOUBLE_EQ(box.c, 0.5 / 0.8);
  EXPECT_DOUBLE_EQ(box.cmd.vx, 0.5);
  EXPECT_DOUBLE_EQ(box.cmd.wz, 0.6 * 0.5 / 0.8) << "omega scaled with v: the direction is kept";
  box = vf::uniformBox(Twist2{0.2, 0.0, -2.4}, diff, omega_bar);
  EXPECT_DOUBLE_EQ(box.c, 0.5);
  EXPECT_DOUBLE_EQ(box.cmd.wz, -1.2);
  EXPECT_DOUBLE_EQ(box.cmd.vx, 0.1);
  // Zero denominators are dropped; inside the box c = 1.
  box = vf::uniformBox(Twist2{0.0, 0.0, 3.0}, diff, omega_bar);
  EXPECT_DOUBLE_EQ(box.c, 0.4);
  box = vf::uniformBox(Twist2{0.3, 0.0, 0.3}, diff, omega_bar);
  EXPECT_EQ(box.c, 1.0);
  EXPECT_EQ(box.cmd.vx, 0.3);
  EXPECT_EQ(box.status, Status::kNone) << "no Assumption-3 flag from the uniform box";
  EXPECT_FALSE(box.eta_box);
  // A lateral component is not commandable by these models.
  box = vf::uniformBox(Twist2{0.3, 0.2, 0.0}, diff, omega_bar);
  EXPECT_EQ(box.cmd.vy, 0.0);
  Kinematics car = diff;
  car.type = MotionModel::kAckermann;
  box = vf::uniformBox(Twist2{0.8, 0.0, 0.6}, car, omega_bar);
  EXPECT_DOUBLE_EQ(box.c, 0.5 / 0.8);
}

// max_reverse_vel (math_spec §8.3, §9.4 [Design, 0.3.0]): with drive_backward the box bounds |v|
// by min(v_max, v_rev); forward it stays v_max, and omni ignores it.
TEST(LimiterUniformBox, ReverseSpeedBound)
{
  Kinematics diff;
  diff.max_linear_vel = 0.5;
  diff.max_reverse_vel = 0.2;
  EXPECT_EQ(diff.linearLimit(), 0.5);
  BoxResult box = vf::uniformBox(Twist2{-0.4, 0.0, 0.4}, diff, 1.2);
  EXPECT_DOUBLE_EQ(box.cmd.vx, -0.4) << "forward drive: v_max only";
  diff.drive_backward = true;
  EXPECT_EQ(diff.linearLimit(), 0.2);
  box = vf::uniformBox(Twist2{-0.4, 0.0, 0.4}, diff, 1.2);
  EXPECT_DOUBLE_EQ(box.c, 0.5);
  EXPECT_DOUBLE_EQ(box.cmd.vx, -0.2);
  EXPECT_DOUBLE_EQ(box.cmd.wz, 0.2) << "omega scaled with v";
  // The eta box intersects |a_x + eta b_x| <= v_rev.
  box = vf::etaBox(Vec2(-0.1, 0.0), Vec2(-0.3, 0.0), 1.0, 0.0, Twist2{}, diff, 1.2);
  EXPECT_TRUE(box.eta_box);
  EXPECT_NEAR(box.eta, (0.2 - 0.1) / 0.3, 1e-12);
  EXPECT_NEAR(box.cmd.vx, -0.2, 1e-12);
  Kinematics defaulted;  // +inf: v_max
  defaulted.drive_backward = true;
  EXPECT_EQ(defaulted.linearLimit(), defaulted.max_linear_vel);
  Kinematics omni = omniKinematics();
  omni.drive_backward = true;
  omni.max_reverse_vel = 0.1;
  EXPECT_EQ(omni.linearLimit(), omni.max_linear_vel);
}

// Holonomic omega never enters c: it is clamped on its own (math_spec §8.3).
TEST(LimiterUniformBox, HolonomicOmegaIsExcluded)
{
  Kinematics omni = omniKinematics();
  omni.max_linear_vel = 0.5;
  omni.max_lateral_vel = 0.3;
  BoxResult box = vf::uniformBox(Twist2{0.2, 0.1, 9.0}, omni, 1.5);
  EXPECT_EQ(box.c, 1.0);
  EXPECT_EQ(box.cmd.vx, 0.2);
  EXPECT_EQ(box.cmd.vy, 0.1);
  EXPECT_EQ(box.cmd.wz, 1.5);
  // ||v_xy|| binds.
  box = vf::uniformBox(Twist2{0.6, 0.0, -0.4}, omni, 1.5);
  EXPECT_DOUBLE_EQ(box.c, 0.5 / 0.6);
  EXPECT_EQ(box.cmd.wz, -0.4) << "omega is not scaled";
  box = vf::uniformBox(Twist2{0.3, 0.4, 0.0}, omni, 1.5);
  EXPECT_DOUBLE_EQ(box.c, 0.75) << "both ||v_xy|| (0.5/0.5) and v_y (0.3/0.4) terms";
  EXPECT_NEAR(std::hypot(box.cmd.vx, box.cmd.vy), 0.375, 1e-15);
}

// -----------------------------------------------------------------------------------------------
// The eta-box (math_spec §8.3).

// O44: pure box, FL, d = 0.2, v_max 0.5, omega_max 1.5, v_r 0.4, psi = 0, Phi_S at D = 0.2 (atan,
// k_f = 2) and Phi_T = (0, 0.35): eta_1 = 0.126613, the omega interval is [-2.68331, -0.20641],
// the set is empty, uniform c = 0.788083, box output v = 0.096809 >= 0, omega = 1.5,
// kAssumption3Clamped (all 1e-6). Without the [0, eta_1] bound the robot would back away.
TEST(LimiterEtaBox, O44EtaLowerBound)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const DistanceField field(gain, kVr, 1, 0.1);
  const Vec2 x(0.0, -0.2);
  FieldSample sample;
  field.evaluate(x, lineClosestPoint(x, Vec2::Zero(), Vec2::UnitX()), Vec2(0.0, 0.35), &sample);
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.Phi_S.x(), 0.970217, 1e-6);
  EXPECT_NEAR(sample.Phi_S.y(), 0.242238, 1e-6);
  EXPECT_NEAR((sample.Phi_T - Vec2(0.0, 0.35)).norm(), 0.0, 1e-15);
  field.shape(1.0, 1.0, false, &sample);
  ASSERT_FALSE(sample.uniform_shaping);
  EXPECT_FALSE(Status::has(sample.status, Status::kAssumption3Clamped));
  const double eta1 = sample.eta;
  EXPECT_NEAR(eta1, 0.126613, 1e-6);

  Kinematics k;
  k.max_linear_vel = 0.5;
  k.max_angular_vel = 1.5;
  k.control_point_offset = 0.2;
  const double omega_bar = 1.5;
  const vf::Mat2 M = vf::flMatrix(0.0, 0.2, 1.0);
  const Vec2 a = M * sample.Phi_T;
  const Vec2 b = M * sample.Phi_S;
  const Interval omega_interval = vf::boxAxisInterval(a.y(), b.y(), omega_bar);
  EXPECT_NEAR(omega_interval.lo, -2.683313, 1e-6);
  EXPECT_NEAR(omega_interval.hi, -0.206409, 1e-6);

  const vf::LawInput input{Pose2{x, 0.0}, x,    sample.F, sample.T_d, kDt,
                           omega_bar,     0.02, vf::kNaN, vf::kInf,   true};
  const vf::LawOutput law = vf::feedbackLinearizationLaw(input, 0.2, 1.0);
  const BoxResult box = vf::etaBox(a, b, eta1, 0.0, law.cmd, k, omega_bar);
  EXPECT_TRUE(box.fallback);
  EXPECT_FALSE(box.eta_box);
  EXPECT_TRUE(std::isnan(box.eta));
  EXPECT_TRUE(Status::has(box.status, Status::kAssumption3Clamped));
  EXPECT_NEAR(box.c, 0.788083, 1e-6);
  EXPECT_NEAR(box.cmd.vx, 0.096809, 1e-6);
  EXPECT_GE(box.cmd.vx, 0.0);
  EXPECT_NEAR(box.cmd.wz, 1.5, 1e-6);

  // The unbounded rule (no [0, eta_1]): eta = -0.206409, v = -0.200261, D-dot = +0.05.
  const Interval unbounded = vf::boxAxisInterval(a.x(), b.x(), k.max_linear_vel)
                               .intersect(omega_interval)
                               .intersect(Interval{-vf::kInf, eta1});
  ASSERT_FALSE(unbounded.empty());
  EXPECT_NEAR(unbounded.hi, -0.206409, 1e-6);
  const Vec2 u = a + unbounded.hi * b;
  EXPECT_NEAR(u.x(), -0.200261, 1e-6);
  const Vec2 control_point_velocity = vf::flInverse(0.0, 0.2, 1.0) * u;
  EXPECT_NEAR(sample.n.dot(control_point_velocity - sample.v_c), 0.05, 1e-6);
}

// eta* is the largest feasible eta in [0, eta_1]: never negative, and either eta_1 or a
// constraint is active there; an empty set falls back to uniform c with the flag.
TEST(LimiterEtaBox, MaximumOfTheIntersectionNeverNegative)
{
  Rng rng(85);
  int fallbacks = 0;
  int bound_below_eta1 = 0;
  for (int i = 0; i < 20000; ++i) {
    const bool holonomic = rng.coin();
    Kinematics k = holonomic ? omniKinematics() : Kinematics{};
    k.max_linear_vel = rng.uniform(0.2, 1.0);
    k.max_lateral_vel = rng.uniform(0.1, 1.0);
    const double omega_bar = rng.uniform(0.2, 2.0);
    const Vec2 a(rng.uniform(-0.6, 0.6), rng.uniform(-1.0, 1.0));
    const Vec2 b(rng.uniform(-2.0, 2.0), rng.uniform(-3.0, 3.0));
    const double eta1 = rng.uniform(0.0, 1.0);
    const Twist2 fallback{
      rng.uniform(-1.0, 1.0), holonomic ? rng.uniform(-1.0, 1.0) : 0.0, rng.uniform(-3.0, 3.0)};
    const double holo_omega = rng.uniform(-4.0, 4.0);
    const BoxResult box = vf::etaBox(a, b, eta1, holo_omega, fallback, k, omega_bar);
    const Vec2 limits =
      holonomic ? Vec2(k.max_linear_vel, k.max_lateral_vel) : Vec2(k.max_linear_vel, omega_bar);
    const auto feasible = [&](double eta) {
      const Vec2 u = a + eta * b;
      bool ok = std::abs(u.x()) <= limits.x() + 1e-12 && std::abs(u.y()) <= limits.y() + 1e-12;
      if (holonomic) {
        ok = ok && u.norm() <= k.max_linear_vel + 1e-12;
      }
      return ok;
    };
    if (box.fallback) {
      ++fallbacks;
      ASSERT_TRUE(Status::has(box.status, Status::kAssumption3Clamped));
      // Only when no eta in [0, eta_1] is feasible.
      for (int j = 0; j <= 200; ++j) {
        ASSERT_FALSE(feasible(eta1 * j / 200.0) && feasible(eta1 * j / 200.0 - 1e-9));
      }
      const BoxResult uniform = vf::uniformBox(fallback, k, omega_bar);
      ASSERT_EQ(box.cmd.vx, uniform.cmd.vx);
      ASSERT_EQ(box.cmd.wz, uniform.cmd.wz);
      continue;
    }
    ASSERT_TRUE(box.eta_box);
    ASSERT_EQ(box.status, Status::kNone);
    ASSERT_GE(box.eta, 0.0);
    ASSERT_LE(box.eta, eta1);
    ASSERT_TRUE(feasible(box.eta));
    const Vec2 u = a + box.eta * b;
    ASSERT_NEAR((boxAxes(box.cmd, holonomic) - u).norm(), 0.0, 1e-15);
    if (box.eta < eta1) {
      ++bound_below_eta1;
      ASSERT_FALSE(feasible(box.eta + 1e-6)) << "eta* is the maximum";
    }
    if (holonomic) {
      ASSERT_DOUBLE_EQ(box.cmd.wz, std::clamp(holo_omega, -omega_bar, omega_bar));
    } else {
      ASSERT_EQ(box.cmd.vy, 0.0);
    }
  }
  EXPECT_GT(fallbacks, 500);
  EXPECT_GT(bound_below_eta1, 500);
  // NaN eta_1: fallback.
  EXPECT_TRUE(
    vf::etaBox(Vec2::Zero(), Vec2::UnitX(), vf::kNaN, 0.0, Twist2{}, Kinematics{}, 1.5).fallback);
}

// O45 (law/limiter part): a translating circle (0.1 m/s) under FL with omega_max lowered until
// omega binds: D-dot = -eta* G <= 1e-12 and no kAssumption3Clamped while bound.
TEST(LimiterEtaBox, O45TranslatingCircleUnderFeedbackLinearization)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const DistanceField field(gain, kVr, 1, 0.1);
  const Vec2 velocity = 0.1 * Vec2(0.8, 0.6);
  Rng rng(45);
  int omega_bound = 0;
  for (int i = 0; i < 20000; ++i) {
    const Vec2 centre(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
    const double R = 1.5;
    const Vec2 p_c = centre + rng.uniform(0.3, 2.5) * rng.unit();
    const double psi = rng.uniform(-vf::kPi, vf::kPi);
    Kinematics k;
    k.max_linear_vel = 0.5;
    // d omega_bar > ||Phi_T|| keeps M Phi_T inside the box, so the set is never empty.
    k.max_angular_vel = rng.uniform(0.6, 1.2);
    k.control_point_offset = 0.2;
    FieldSample sample = translatingCircleSample(field, p_c, centre, R, velocity);
    field.shape(1.0, 1.0, false, &sample);
    ASSERT_FALSE(sample.uniform_shaping);
    const vf::Mat2 M = vf::flMatrix(psi, 0.2, 1.0);
    const vf::LawInput input{
      Pose2{p_c - 0.2 * vf::heading(psi), psi},
      p_c,
      sample.F,
      sample.T_d,
      kDt,
      k.max_angular_vel,
      0.02,
      vf::kNaN,
      vf::kInf,
      true};
    const vf::LawOutput law = vf::feedbackLinearizationLaw(input, 0.2, 1.0);
    const BoxResult box = vf::etaBox(
      M * sample.Phi_T, M * sample.Phi_S, sample.eta, 0.0, law.cmd, k, k.max_angular_vel);
    ASSERT_FALSE(box.fallback);
    ASSERT_EQ(box.status, Status::kNone);
    const Vec2 control_point_velocity = vf::flInverse(psi, 0.2, 1.0) * Vec2(box.cmd.vx, box.cmd.wz);
    const double D_dot = sample.n.dot(control_point_velocity - sample.v_c);
    ASSERT_LE(D_dot, 1e-12);
    ASSERT_NEAR(D_dot, -box.eta * sample.G, 1e-12);
    if (std::abs(std::abs(box.cmd.wz) - k.max_angular_vel) < 1e-12) {
      ++omega_bound;
    }
  }
  EXPECT_GT(omega_bound, 2000) << "omega binds";
}

// O46 (law/limiter part): omni on a translating circle, path_tangent, 180 deg heading error:
// |omega| <= omega_max, no kAssumption3Clamped while only omega saturates, D-dot <= 1e-12.
TEST(LimiterEtaBox, O46OmniHeadingSaturationIsNotAnAssumption3Violation)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const DistanceField field(gain, kVr, 1, 0.1);
  const Kinematics k = omniKinematics();
  Rng rng(46);
  for (int i = 0; i < 20000; ++i) {
    const Vec2 centre(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
    const Vec2 velocity = rng.uniform(0.0, 0.15) * rng.unit();
    const Vec2 p = centre + rng.uniform(0.3, 2.5) * rng.unit();
    FieldSample sample = translatingCircleSample(field, p, centre, 1.5, velocity);
    field.shape(1.0, 1.0, false, &sample);
    ASSERT_FALSE(sample.uniform_shaping);
    const double psi = vf::angleOf(sample.T_d) + vf::kPi;
    const vf::LawInput input{Pose2{p, psi},     p,    sample.F, sample.T_d, kDt,
                             k.max_angular_vel, 0.02, vf::kNaN, vf::kInf,   true};
    const vf::LawOutput law = vf::holonomicLaw(input, vf::OmniHeadingPolicy::kPathTangent, 2.0);
    ASSERT_DOUBLE_EQ(std::abs(law.cmd.wz), k.max_angular_vel) << "omega saturates";
    const vf::Mat2 Rt = vf::rotation(psi).transpose();
    const BoxResult box = vf::etaBox(
      Rt * sample.Phi_T, Rt * sample.Phi_S, sample.eta, law.cmd.wz, law.cmd, k, k.max_angular_vel);
    ASSERT_TRUE(box.eta_box);
    ASSERT_EQ(box.status, Status::kNone);
    ASSERT_DOUBLE_EQ(box.eta, sample.eta) << "translation is not reduced for omega";
    ASSERT_LE(std::abs(box.cmd.wz), k.max_angular_vel);
    const Vec2 base_velocity = vf::rotate(psi, Vec2(box.cmd.vx, box.cmd.vy));
    const double D_dot = sample.n.dot(base_velocity - sample.v_c);
    ASSERT_LE(D_dot, 1e-12);
  }
}

// O47 (law/limiter part): during a contour episode with theta < 1 on a moving curve the shaping
// is uniform and the box is uniform c: the command stays in the box, ||v_xy|| <= gamma v_r +
// 1e-12 and nothing raises kAssumption3Clamped.
TEST(LimiterUniformBox, O47ContourWithAMovingCurve)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const DistanceField field(gain, kVr, 1, 0.1);
  Rng rng(47);
  for (int i = 0; i < 20000; ++i) {
    const bool holonomic = rng.coin();
    Kinematics k = holonomic ? omniKinematics() : Kinematics{};
    k.max_lateral_vel = rng.uniform(0.1, 0.5);
    const Vec2 centre(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
    const Vec2 p = centre + rng.uniform(0.3, 2.5) * rng.unit();
    FieldSample sample =
      translatingCircleSample(field, p, centre, 1.5, rng.uniform(0.0, 0.2) * rng.unit());
    const double gamma = rng.uniform(0.0, 1.0);
    field.shape(gamma, 1.0, true, &sample);
    ASSERT_TRUE(sample.uniform_shaping);
    ASSERT_FALSE(Status::has(sample.status, Status::kAssumption3Clamped));
    // The contour turns F but keeps ||F|| = ||Phi_gamma|| (math_spec §6.4).
    const Vec2 F = sample.Phi_gamma.norm() * rng.unit();
    const double psi = rng.uniform(-vf::kPi, vf::kPi);
    const double omega_bar = k.max_angular_vel;
    const vf::LawInput input{Pose2{p, psi}, p,    F,        sample.T_d, kDt,
                             omega_bar,     0.02, vf::kNaN, vf::kInf,   true};
    const vf::LawOutput law = holonomic
                                ? vf::holonomicLaw(input, vf::OmniHeadingPolicy::kField, 2.0)
                                : vf::feedbackLinearizationLaw(input, 0.2, 1.0);
    const BoxResult box = vf::uniformBox(law.cmd, k, omega_bar);
    ASSERT_EQ(box.status, Status::kNone);
    ASSERT_LE(std::abs(box.cmd.vx), k.max_linear_vel + 1e-12);
    ASSERT_LE(std::abs(box.cmd.wz), omega_bar + 1e-12);
    if (holonomic) {
      ASSERT_LE(std::abs(box.cmd.vy), k.max_lateral_vel + 1e-12);
      ASSERT_LE(std::hypot(box.cmd.vx, box.cmd.vy), gamma * kVr + 1e-12);
    } else {
      const Vec2 control_point_velocity =
        vf::flInverse(psi, 0.2, 1.0) * Vec2(box.cmd.vx, box.cmd.wz);
      ASSERT_LE(control_point_velocity.norm(), gamma * kVr + 1e-12);
    }
  }
}

// -----------------------------------------------------------------------------------------------
// CommandLimiter (math_spec §8.4).

namespace
{

struct AxisBounds
{
  double x;
  double y;
  double w;
};

// The per-axis bounds A_i of math_spec §8.4 for a step from `from` to `to`.
AxisBounds expectedBounds(
  const Twist2 & from, const Twist2 & to, const Kinematics & k, bool cancel, bool hard)
{
  const auto linear = [&](double previous, double target) {
    if (cancel) {
      return hard ? std::max(k.cancel_deceleration, k.max_linear_decel) : k.cancel_deceleration;
    }
    return previous * (target - previous) < 0.0 ? k.max_linear_decel : k.max_linear_accel;
  };
  return AxisBounds{linear(from.vx, to.vx), linear(from.vy, to.vy), k.max_angular_accel};
}

}  // namespace

// O48: random u*, u_prev: every axis change is within its bound (1e-12) and u - u_prev is
// parallel to u* - u_prev (1e-12); alpha is the largest admissible.
TEST(LimiterCommand, O48AccelerationStage)
{
  const CommandLimiter limiter;
  Rng rng(48);
  double worst_bound = -vf::kInf;
  double worst_parallel = 0.0;
  for (int i = 0; i < 20000; ++i) {
    Kinematics k = omniKinematics();
    k.max_linear_accel = rng.uniform(0.1, 3.0);
    k.max_linear_decel = rng.uniform(0.1, 3.0);
    k.max_angular_accel = rng.uniform(0.2, 6.0);
    k.cancel_deceleration = rng.uniform(0.1, 3.0);
    const bool cancel = rng.chance(0.2);
    LimiterState state;
    state.valid = true;
    state.cancel_hard = rng.coin();
    state.u_prev = Twist2{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-2.0, 2.0)};
    const Twist2 target =
      cancel ? Twist2{}
             : Twist2{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-2.0, 2.0)};
    const double dt = rng.uniform(0.01, 0.2);
    double alpha = -1.0;
    std::uint32_t status = Status::kNone;
    LimiterState next;
    const Twist2 u = limiter.limit(target, state, &next, dt, k, cancel, &status, &alpha);
    ASSERT_EQ(status, Status::kNone);
    ASSERT_GE(alpha, 0.0);
    ASSERT_LE(alpha, 1.0);
    const AxisBounds bounds = expectedBounds(state.u_prev, target, k, cancel, state.cancel_hard);
    const double change[] = {
      u.vx - state.u_prev.vx, u.vy - state.u_prev.vy, u.wz - state.u_prev.wz};
    const double delta[] = {
      target.vx - state.u_prev.vx, target.vy - state.u_prev.vy, target.wz - state.u_prev.wz};
    const double limit[] = {bounds.x * dt, bounds.y * dt, bounds.w * dt};
    double usage = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      worst_bound = std::max(worst_bound, std::abs(change[axis]) - limit[axis]);
      usage = std::max(usage, std::abs(change[axis]) / limit[axis]);
      ASSERT_NEAR(change[axis], alpha * delta[axis], 4e-15);
    }
    // Parallel: the cross product of the change and the step direction vanishes.
    const double cross[] = {
      change[1] * delta[2] - change[2] * delta[1], change[2] * delta[0] - change[0] * delta[2],
      change[0] * delta[1] - change[1] * delta[0]};
    const double delta_norm =
      std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    const double cross_norm =
      std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
    worst_parallel = std::max(worst_parallel, cross_norm / std::max(1e-300, delta_norm));
    // Largest alpha: the target is reached, or some axis uses its whole bound.
    ASSERT_TRUE(alpha == 1.0 || usage >= 1.0 - 1e-12);
    ASSERT_EQ(next.u_prev.vx, u.vx);
    ASSERT_EQ(next.u_prev.wz, u.wz);
    ASSERT_EQ(next.cancel_hard, state.cancel_hard);
    ASSERT_TRUE(next.valid);
  }
  EXPECT_LE(worst_bound, 1e-12);
  EXPECT_LE(worst_parallel, 1e-12);
}

// O48 with brake_translation (math_spec §8.4 [Design, 0.1.1]): random u_prev, target (0, 0, w*):
// every axis change is within its bound, the translation moves along its own straight line with
// the largest admissible alpha, and omega takes the largest admissible step on its own.
TEST(LimiterCommand, O48AccelerationStageBrakingTranslation)
{
  const CommandLimiter limiter;
  Rng rng(4848);
  for (int i = 0; i < 20000; ++i) {
    Kinematics k = omniKinematics();
    k.max_linear_accel = rng.uniform(0.1, 3.0);
    k.max_linear_decel = rng.uniform(0.1, 3.0);
    k.max_angular_accel = rng.uniform(0.2, 6.0);
    k.cancel_deceleration = rng.uniform(0.1, 3.0);
    const bool cancel = rng.chance(0.2);
    LimiterState state;
    state.cancel_hard = rng.coin();
    state.u_prev = Twist2{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-2.0, 2.0)};
    const Twist2 target{0.0, 0.0, cancel ? 0.0 : rng.uniform(-2.0, 2.0)};
    const double dt = rng.uniform(0.01, 0.2);
    double alpha = -1.0;
    const Twist2 u = limiter.limit(target, state, nullptr, dt, k, cancel, nullptr, &alpha, true);
    const AxisBounds bounds = expectedBounds(state.u_prev, target, k, cancel, state.cancel_hard);
    const Twist2 & p = state.u_prev;
    ASSERT_LE(std::abs(u.vx - p.vx), bounds.x * dt + 1e-12);
    ASSERT_LE(std::abs(u.vy - p.vy), bounds.y * dt + 1e-12);
    ASSERT_LE(std::abs(u.wz - p.wz), bounds.w * dt + 1e-12);
    // Translation: u = (1 - a) u_prev (towards zero, parallel), a the largest admissible.
    const double a_lin =
      std::min({1.0, bounds.x * dt / std::abs(p.vx), bounds.y * dt / std::abs(p.vy)});
    ASSERT_NEAR(u.vx, (1.0 - a_lin) * p.vx, 4e-15);
    ASSERT_NEAR(u.vy, (1.0 - a_lin) * p.vy, 4e-15);
    // Omega: the largest admissible step, whatever the translation does.
    const double dw = target.wz - p.wz;
    ASSERT_NEAR(u.wz, p.wz + vf::sgn(dw) * std::min(std::abs(dw), bounds.w * dt), 4e-15);
    ASSERT_GE(alpha, 0.0);
    ASSERT_LE(alpha, 1.0);
  }
}

// Appendix C item 11: a_dec when u_prev,i Delta_i < 0, else a_acc (also across zero); alpha_max for
// omega; all axes share alpha.
TEST(LimiterCommand, BoundSelectionBySign)
{
  const CommandLimiter limiter;
  Kinematics k;
  k.max_linear_accel = 0.5;
  k.max_linear_decel = 2.0;
  k.max_angular_accel = 4.0;
  const double dt = 0.1;
  const auto step = [&](double from, double to) {
    LimiterState state;
    state.u_prev = Twist2{from, 0.0, 0.0};
    return limiter.limit(Twist2{to, 0.0, 0.0}, state, nullptr, dt, k, false, nullptr).vx;
  };
  EXPECT_DOUBLE_EQ(step(0.3, 1.0), 0.35) << "speeding up: a_acc";
  EXPECT_DOUBLE_EQ(step(0.3, 0.0), 0.1) << "slowing down: a_dec";
  EXPECT_DOUBLE_EQ(step(0.3, -1.0), 0.1) << "reversing: u_prev Delta < 0 selects a_dec";
  EXPECT_DOUBLE_EQ(step(-0.3, -1.0), -0.35);
  EXPECT_DOUBLE_EQ(step(-0.3, 0.0), -0.1);
  EXPECT_DOUBLE_EQ(step(0.0, -1.0), -0.05) << "from rest: a_acc";
  EXPECT_DOUBLE_EQ(step(0.3, 0.31), 0.31) << "within the bound: reached";

  // One alpha for all axes: omega limited by alpha_max slows v proportionally.
  LimiterState state;
  const Twist2 u = limiter.limit(Twist2{0.04, 0.0, 2.0}, state, nullptr, dt, k, false, nullptr);
  EXPECT_DOUBLE_EQ(u.wz, 0.4);
  EXPECT_DOUBLE_EQ(u.vx, 0.04 * 0.2);
}

// math_spec §8.4 [Design, 0.1.1]: entering a rotation phase at v = 0.09 m/s with the target
// (0, 0, 1.5), the translation brakes at a_dec on its own (v = 0 within ceil(v / (a_dec dt))
// cycles) while omega ramps at alpha_max; the shared alpha would hold v for ~9 cycles.
TEST(LimiterCommand, BrakeTranslationDecouplesTheLinearStopFromTheOmegaRamp)
{
  const CommandLimiter limiter;
  Kinematics k;
  k.max_linear_decel = 1.0;
  k.max_angular_accel = 3.2;
  const Twist2 target{0.0, 0.0, 1.5};
  const double v0 = 0.09;
  LimiterState state;
  state.u_prev = Twist2{v0, 0.0, 0.0};
  int cycles = 0;
  for (; state.u_prev.vx > 0.0 && cycles < 100; ++cycles) {
    const double v = state.u_prev.vx;
    const double w = state.u_prev.wz;
    double alpha = -1.0;
    limiter.limit(target, state, &state, kDt, k, false, nullptr, &alpha, true);
    EXPECT_DOUBLE_EQ(state.u_prev.vx, std::max(0.0, v - k.max_linear_decel * kDt));
    EXPECT_DOUBLE_EQ(state.u_prev.wz, std::min(1.5, w + k.max_angular_accel * kDt));
    EXPECT_LT(alpha, 1.0);
  }
  EXPECT_LE(cycles, static_cast<int>(std::ceil(v0 / (k.max_linear_decel * kDt))));

  // Without the flag the axes share omega's alpha and v decays only geometrically.
  state = LimiterState{};
  state.u_prev = Twist2{v0, 0.0, 0.0};
  cycles = 0;
  for (; state.u_prev.vx >= vf::kStopLinearSpeed && cycles < 100; ++cycles) {
    limiter.limit(target, state, &state, kDt, k, false, nullptr);
  }
  EXPECT_GT(cycles, static_cast<int>(std::ceil(v0 / (k.max_linear_decel * kDt))));

  // A cancel brakes at a_can on its own; omni lateral shares the translational alpha.
  Kinematics omni;
  omni.type = MotionModel::kOmnidirectional;
  omni.cancel_deceleration = 0.4;
  state = LimiterState{};
  state.u_prev = Twist2{0.06, 0.08, 1.0};
  const Twist2 u = limiter.limit(Twist2{}, state, nullptr, kDt, omni, true, nullptr, nullptr, true);
  EXPECT_NEAR(u.vy, 0.08 - 0.4 * kDt, 1e-15) << "a_can on the binding (lateral) axis";
  EXPECT_NEAR(u.vx * 0.08 - u.vy * 0.06, 0.0, 1e-15) << "translation keeps its direction";
  EXPECT_DOUBLE_EQ(u.wz, 1.0 - omni.max_angular_accel * kDt);
}

TEST(LimiterCommand, StoppedAndCancelDecelerations)
{
  const CommandLimiter limiter;
  Kinematics k;
  k.max_linear_decel = 1.0;
  k.cancel_deceleration = 0.3;
  k.max_angular_accel = 3.2;
  LimiterState state;
  state.u_prev = Twist2{0.4, 0.0, 0.0};
  // STOPPED without a cancel: the maximum deceleration (by the sign rule).
  EXPECT_DOUBLE_EQ(limiter.limit(Twist2{}, state, nullptr, kDt, k, false, nullptr).vx, 0.35);
  // Cancel: a_can.
  EXPECT_DOUBLE_EQ(limiter.limit(Twist2{}, state, nullptr, kDt, k, true, nullptr).vx, 0.385);
  // Cancel after a rollout found a collision: max(a_can, a_dec).
  state.cancel_hard = true;
  LimiterState next;
  EXPECT_DOUBLE_EQ(limiter.limit(Twist2{}, state, &next, kDt, k, true, nullptr).vx, 0.35);
  EXPECT_TRUE(next.cancel_hard);
  // The angular bound of a cancel is alpha_max.
  state.u_prev = Twist2{0.0, 0.0, 1.0};
  EXPECT_DOUBLE_EQ(limiter.limit(Twist2{}, state, nullptr, kDt, k, true, nullptr).wz, 0.84);
}

// O49 (limiter part): Ackermann at 0.3 m/s reaches |v| < 0.01 within v/a_can + 2 cycles; skid at
// the 0.3 rad/s rotate floor reaches |omega| < 0.01 within omega/alpha_max + 2 cycles.
TEST(LimiterCommand, O49CancelRamps)
{
  const CommandLimiter limiter;
  Kinematics car;
  car.type = MotionModel::kAckermann;
  car.min_turning_radius = 0.5;
  car.cancel_deceleration = 1.0;
  LimiterState state;
  state.u_prev = Twist2{0.3, 0.0, 0.2};
  int cycles = 0;
  while (std::abs(state.u_prev.vx) >= vf::kStopLinearSpeed && cycles < 100) {
    limiter.limit(Twist2{}, state, &state, kDt, car, true, nullptr);
    ASSERT_LE(std::abs(state.u_prev.wz), std::abs(state.u_prev.vx) / 0.5 + 1e-15);
    ++cycles;
  }
  EXPECT_LE(cycles, static_cast<int>(std::ceil(0.3 / 1.0 / kDt)) + 2);

  Kinematics skid;
  skid.type = MotionModel::kSkidSteer;
  state = LimiterState{};
  state.u_prev = Twist2{0.0, 0.0, 0.3};
  cycles = 0;
  while (std::abs(state.u_prev.wz) >= vf::kStopAngularSpeed && cycles < 100) {
    limiter.limit(Twist2{}, state, &state, kDt, skid, true, nullptr);
    ++cycles;
  }
  EXPECT_LE(cycles, static_cast<int>(std::ceil(0.3 / skid.max_angular_accel / kDt)) + 2);
}

TEST(LimiterCommand, AckermannCurvatureClamp)
{
  const CommandLimiter limiter;
  Kinematics car;
  car.type = MotionModel::kAckermann;
  car.min_turning_radius = 0.5;
  car.max_angular_accel = 100.0;
  LimiterState state;
  state.u_prev = Twist2{0.2, 0.0, 0.0};
  std::uint32_t status = Status::kNone;
  Twist2 u = limiter.limit(Twist2{0.2, 0.0, -1.0}, state, nullptr, kDt, car, false, &status);
  EXPECT_DOUBLE_EQ(u.wz, -0.4) << "|omega| <= |v| / R_min, sign kept";
  EXPECT_TRUE(Status::has(status, Status::kCurvatureSaturated));
  status = Status::kNone;
  u = limiter.limit(Twist2{0.2, 0.0, 0.3}, state, nullptr, kDt, car, false, &status);
  EXPECT_DOUBLE_EQ(u.wz, 0.3);
  EXPECT_EQ(status, Status::kNone);
  // Diff/skid have no curvature limit; Ackermann with R_min <= 0 neither.
  Kinematics diff;
  diff.max_angular_accel = 100.0;
  status = Status::kNone;
  u = limiter.limit(Twist2{0.2, 0.0, -1.0}, state, nullptr, kDt, diff, false, &status);
  EXPECT_DOUBLE_EQ(u.wz, -1.0);
  EXPECT_EQ(status, Status::kNone);
  car.min_turning_radius = 0.0;
  u = limiter.limit(Twist2{0.2, 0.0, -1.0}, state, nullptr, kDt, car, false, &status);
  EXPECT_DOUBLE_EQ(u.wz, -1.0);
}

TEST(LimiterCommand, NonFiniteGuardAndDegenerateSteps)
{
  const CommandLimiter limiter;
  const Kinematics k = omniKinematics();
  LimiterState state;
  state.u_prev = Twist2{0.2, 0.1, 0.3};
  for (const Twist2 & bad :
       {Twist2{vf::kNaN, 0.0, 0.0}, Twist2{0.0, vf::kInf, 0.0}, Twist2{0.0, 0.0, -vf::kInf}}) {
    std::uint32_t status = Status::kNone;
    double alpha = -1.0;
    LimiterState next;
    const Twist2 u = limiter.limit(bad, state, &next, kDt, k, false, &status, &alpha);
    EXPECT_EQ(u.vx, 0.0);
    EXPECT_EQ(u.vy, 0.0);
    EXPECT_EQ(u.wz, 0.0);
    EXPECT_TRUE(Status::has(status, Status::kNonFinite));
    EXPECT_EQ(next.u_prev.vx, 0.0);
    EXPECT_EQ(alpha, 0.0);
  }
  // A corrupted u_prev, a NaN or infinite dt: zero and kNonFinite.
  LimiterState corrupt;
  corrupt.u_prev = Twist2{vf::kNaN, 0.0, 0.0};
  std::uint32_t status = Status::kNone;
  EXPECT_EQ(limiter.limit(Twist2{}, corrupt, nullptr, kDt, k, false, &status).vx, 0.0);
  EXPECT_TRUE(Status::has(status, Status::kNonFinite));
  for (const double dt : {vf::kNaN, vf::kInf}) {
    status = Status::kNone;
    const Twist2 u = limiter.limit(Twist2{0.1, 0.0, 0.0}, state, nullptr, dt, k, false, &status);
    EXPECT_EQ(u.vx, 0.0);
    EXPECT_TRUE(Status::has(status, Status::kNonFinite));
  }
  // A non-positive dt keeps u_prev.
  double alpha = -1.0;
  status = Status::kNone;
  Twist2 u = limiter.limit(Twist2{1.0, 1.0, 1.0}, state, nullptr, 0.0, k, false, &status, &alpha);
  EXPECT_EQ(u.vx, 0.2);
  EXPECT_EQ(u.vy, 0.1);
  EXPECT_EQ(u.wz, 0.3);
  EXPECT_EQ(alpha, 0.0);
  EXPECT_EQ(status, Status::kNone);
  u = limiter.limit(Twist2{1.0, 1.0, 1.0}, state, nullptr, -0.1, k, false, &status);
  EXPECT_EQ(u.vx, 0.2);
  // Reaching the target: alpha = 1.
  u = limiter.limit(state.u_prev, state, nullptr, kDt, k, false, &status, &alpha);
  EXPECT_EQ(alpha, 1.0);
  EXPECT_EQ(u.wz, 0.3);
}

TEST(LimiterCommand, OutputMayAliasTheInputState)
{
  const CommandLimiter limiter;
  const Kinematics k;
  LimiterState state;
  state.u_prev = Twist2{0.1, 0.0, 0.0};
  state.cancel_hard = true;
  state.valid = false;
  const Twist2 u = limiter.limit(Twist2{0.5, 0.0, 0.0}, state, &state, kDt, k, false, nullptr);
  EXPECT_DOUBLE_EQ(u.vx, 0.15);
  EXPECT_DOUBLE_EQ(state.u_prev.vx, 0.15);
  EXPECT_TRUE(state.cancel_hard);
  EXPECT_TRUE(state.valid);
  // The next step continues from the stored output.
  limiter.limit(Twist2{0.5, 0.0, 0.0}, state, &state, kDt, k, false, nullptr);
  EXPECT_DOUBLE_EQ(state.u_prev.vx, 0.2);
}

// Unicycle and Ackermann models have no lateral axis: a lateral u_prev (re-seeded from a slipping
// odometry twist) is not commanded and does not slow the other axes.
TEST(LimiterCommand, LateralAxisOnlyForHolonomicModels)
{
  const CommandLimiter limiter;
  Kinematics skid;
  skid.type = MotionModel::kSkidSteer;
  LimiterState state;
  state.u_prev = Twist2{0.2, 0.15, 0.0};
  double alpha = -1.0;
  const Twist2 u =
    limiter.limit(Twist2{0.22, 0.0, 0.0}, state, nullptr, kDt, skid, false, nullptr, &alpha);
  EXPECT_EQ(u.vy, 0.0);
  EXPECT_DOUBLE_EQ(u.vx, 0.22);
  EXPECT_EQ(alpha, 1.0);
  // Omni keeps the lateral axis under the same linear bounds.
  const Twist2 omni = limiter.limit(
    Twist2{0.22, 0.0, 0.0}, state, nullptr, kDt, omniKinematics(), false, nullptr, &alpha);
  EXPECT_DOUBLE_EQ(omni.vy, 0.1);
  EXPECT_DOUBLE_EQ(alpha, 0.05 / 0.15);
}

// -----------------------------------------------------------------------------------------------
// Upstream regression U25 (docs/design/upstream_audit.md): upstream had no velocity or
// acceleration limits and did not validate d.

namespace
{

// One randomised configuration and state of the nominal pipeline of math_spec §8.1 for FOLLOW and
// FINAL_APPROACH: field -> speed shaping -> (contour) -> law -> Ackermann band -> kinematic box
// -> backstop -> CommandLimiter.
struct PipelineCase
{
  vf::CoreConfig config;
  double v_r{0.4};
  double gamma_lim{1.0};
};

}  // namespace

TEST(UpstreamRegression, U25_CommandsRespectLimits)
{
  const CommandLimiter limiter;
  Rng rng(25);
  const MotionModel models[] = {
    MotionModel::kDifferential, MotionModel::kSkidSteer, MotionModel::kOmnidirectional,
    MotionModel::kAckermann};
  int eta_boxes = 0;
  int fallbacks = 0;
  for (int i = 0; i < 20000; ++i) {
    PipelineCase c;
    vf::MotionConfig & motion = c.config.motion;
    motion.motion_model = models[static_cast<int>(rng.uniform(0.0, 4.0))];
    const bool unicycle = motion.motion_model == MotionModel::kDifferential ||
                          motion.motion_model == MotionModel::kSkidSteer;
    motion.tracking_law =
      unicycle && rng.coin() ? TrackingLaw::kHeadingTracking : TrackingLaw::kAuto;
    motion.max_linear_vel = rng.uniform(0.2, 1.2);
    motion.max_lateral_vel = rng.uniform(0.1, 1.0);
    motion.max_angular_vel = rng.uniform(0.3, 3.0);
    motion.max_linear_accel = rng.uniform(0.2, 3.0);
    motion.max_linear_decel = rng.uniform(0.2, 3.0);
    motion.max_angular_accel = rng.uniform(0.5, 6.0);
    motion.control_point_offset = rng.uniform(0.05, 0.5);
    motion.drive_backward = unicycle && rng.chance(0.2);
    motion.allow_reversing = rng.chance(0.3);
    motion.heading_gain = rng.uniform(0.5, 4.0);
    motion.omni_heading_policy =
      static_cast<vf::OmniHeadingPolicy>(static_cast<int>(rng.uniform(0.0, 3.0)));
    motion.min_turning_radius = rng.uniform(0.2, 1.5);
    motion.reference_point_offset = rng.uniform(-0.4, 0.4);
    motion.ackermann_min_speed = 0.05;
    c.config.phases.cancel_deceleration = rng.uniform(0.2, 3.0);
    c.v_r = rng.uniform(0.1, motion.max_linear_vel);
    const vf::ControlLaw law(c.config);
    const Kinematics & k = law.kinematics();

    // Speed shaping terms; Ackermann floors the external limit at v_ack / v_r.
    SpeedShaping shaping;
    c.gamma_lim = rng.uniform(0.05, 1.0);
    shaping.gamma_lim =
      k.isAckermann() ? std::max(c.gamma_lim, k.ackermann_min_speed / c.v_r) : c.gamma_lim;
    shaping.gamma_cl = rng.chance(0.5) ? 1.0 : rng.uniform(0.0, 1.0);
    shaping.gamma_ap = rng.chance(0.5) ? 1.0 : rng.uniform(0.0, 1.0);
    shaping.gamma_col = rng.chance(0.5) ? 1.0 : rng.uniform(0.0, 1.0);
    const double omega_bar = k.angularLimit(shaping.gamma_lim);

    // A random (possibly moving, possibly terminal) field at the law's point.
    const Phase phase = rng.chance(0.8) ? Phase::kFollow : Phase::kFinalApproach;
    const Pose2 pose{
      Vec2(rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)), rng.uniform(-vf::kPi, vf::kPi)};
    const vf::EvaluationPoint point = law.evaluationPoint(pose, phase);
    const GainFunction gain(
      rng.coin() ? GainType::kAtan : GainType::kRational, rng.uniform(0.5, 6.0));
    const DistanceField field(gain, c.v_r, rng.coin() ? 1 : -1, 0.1);
    const bool circle = rng.coin();
    const Vec2 centre(rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0));
    const Vec2 tangent = rng.unit();
    const auto closest = [&](const Vec2 & x) {
      return circle ? circleClosestPoint(x, centre, 1.5) : lineClosestPoint(x, centre, tangent);
    };
    const ClosestPoint cp = closest(point.x);
    FieldSample sample;
    if (!circle && rng.chance(0.3)) {
      const vf::TerminalProfile profile(c.v_r, 0.05, 0.5, 0.125);
      field.evaluateTerminal(point.x, cp, rng.uniform(-1.0, 2.0), profile, &sample);
    } else {
      const Vec2 v_c =
        rng.chance(0.5) ? Vec2(Vec2::Zero()) : Vec2(rng.uniform(0.0, 1.3 * c.v_r) * rng.unit());
      field.evaluate(point.x, cp, v_c, &sample);
    }
    ASSERT_TRUE(sample.valid);
    const bool contour_blend = rng.chance(0.15);
    field.shape(shaping.gamma(), shaping.gamma_lim, contour_blend, &sample);
    if (contour_blend) {
      sample.F = sample.Phi_gamma.norm() * rng.unit();
    }

    // The law, with a static probe of the same curve for heading tracking.
    const auto probe = [&](const Vec2 & x, double) {
      FieldSample probe_sample;
      field.evaluate(x, closest(x), Vec2::Zero(), &probe_sample);
      vf::ProbeSample out;
      out.F = probe_sample.F;
      out.psi_d = vf::angleOf(probe_sample.F);
      out.s_star = probe_sample.s_star;
      out.D = probe_sample.D;
      return out;
    };
    vf::LawInput input;
    input.pose = pose;
    input.x = point.x;
    input.F = sample.F;
    input.T_d = sample.T_d;
    input.dt = rng.uniform(0.02, 0.1);
    input.omega_bar = omega_bar;
    input.hold_speed = vf::kHoldSpeedFraction * c.v_r * shaping.gamma_lim;
    input.last_psi_d = rng.uniform(-vf::kPi, vf::kPi);
    input.curve_length = circle ? vf::kTwoPi * 1.5 : vf::kInf;
    const vf::LawOutput out = law.track(phase, input, probe);
    Twist2 cmd = out.cmd;
    if (k.isAckermann() && phase == Phase::kFollow) {
      cmd.vx = vf::applyAckermannBand(cmd.vx, k.ackermann_min_speed);
    }

    // Kinematic box: the eta-box for FL and holonomic on the eta branch, else uniform c.
    const TrackingLaw active = law.activeLaw(phase);
    BoxResult box;
    if (
      !sample.uniform_shaping &&
      (active == TrackingLaw::kFeedbackLinearization || active == TrackingLaw::kHolonomic)) {
      const vf::Mat2 M = active == TrackingLaw::kHolonomic
                           ? vf::Mat2(vf::rotation(pose.yaw).transpose())
                           : vf::flMatrix(pose.yaw, k.control_point_offset, k.driveSign());
      box = vf::etaBox(M * sample.Phi_T, M * sample.Phi_S, sample.eta, cmd.wz, cmd, k, omega_bar);
      eta_boxes += box.eta_box ? 1 : 0;
      fallbacks += box.fallback ? 1 : 0;
    } else {
      box = vf::uniformBox(cmd, k, omega_bar);
    }
    Twist2 target = box.cmd;
    const double q = k.isAckermann() ? 1.0 : k.driveSign();
    if (!k.allow_reversing) {
      target = vf::applyReversingBackstop(target, q);
    }

    // The target is in the box.
    const double tol = 1e-12;
    ASSERT_LE(std::abs(target.vx), k.max_linear_vel + tol);
    ASSERT_LE(std::abs(target.wz), omega_bar + tol);
    if (k.isHolonomic()) {
      ASSERT_LE(std::abs(target.vy), k.max_lateral_vel + tol);
      ASSERT_LE(std::hypot(target.vx, target.vy), k.max_linear_vel + tol);
    } else {
      ASSERT_EQ(target.vy, 0.0);
    }

    // The limiter, from a previous output inside the same box.
    LimiterState state;
    state.valid = true;
    const double v_prev = rng.uniform(-1.0, 1.0) * k.max_linear_vel;
    double w_prev = rng.uniform(-1.0, 1.0) * omega_bar;
    double vy_prev = 0.0;
    if (k.isHolonomic()) {
      const double room =
        std::sqrt(std::max(0.0, k.max_linear_vel * k.max_linear_vel - v_prev * v_prev));
      vy_prev = rng.uniform(-1.0, 1.0) * std::min(room, k.max_lateral_vel);
    }
    if (k.isAckermann()) {
      w_prev = std::clamp(
        w_prev, -std::abs(v_prev) / k.min_turning_radius, std::abs(v_prev) / k.min_turning_radius);
    }
    state.u_prev = Twist2{v_prev, vy_prev, w_prev};
    const bool cancel = rng.chance(0.1);
    const Twist2 goal = cancel ? Twist2{} : target;
    std::uint32_t status = Status::kNone;
    const double dt = input.dt;
    const Twist2 u = limiter.limit(goal, state, &state, dt, k, cancel, &status);
    ASSERT_FALSE(Status::has(status, Status::kNonFinite));
    ASSERT_LE(std::abs(u.vx), k.max_linear_vel + tol);
    ASSERT_LE(std::abs(u.wz), omega_bar + tol);
    if (k.isHolonomic()) {
      ASSERT_LE(std::abs(u.vy), k.max_lateral_vel + tol);
      ASSERT_LE(std::hypot(u.vx, u.vy), k.max_linear_vel + tol);
    } else {
      ASSERT_EQ(u.vy, 0.0);
    }
    // Acceleration bounds. For Ackermann the yaw rate is slaved to the speed by the curvature
    // clamp that follows the acceleration stage (math_spec §8.4 order), so only |omega| <=
    // |v| / R_min is checked there.
    const Twist2 from{v_prev, vy_prev, w_prev};
    const AxisBounds bounds = expectedBounds(from, goal, k, cancel, false);
    ASSERT_LE(std::abs(u.vx - v_prev), bounds.x * dt + tol);
    ASSERT_LE(std::abs(u.vy - vy_prev), bounds.y * dt + tol);
    if (k.isAckermann()) {
      ASSERT_LE(std::abs(u.wz), std::abs(u.vx) / k.min_turning_radius + tol);
    } else {
      ASSERT_LE(std::abs(u.wz - w_prev), bounds.w * dt + tol);
    }
  }
  EXPECT_GT(eta_boxes, 1000);
  EXPECT_GT(fallbacks, 10);

  // FL with d = 0 is rejected (upstream divided by zero).
  vf::CoreConfig zero_offset;
  zero_offset.motion.control_point_offset = 0.0;
  EXPECT_THROW(vf::ControlLaw{zero_offset}, vf::ConfigError);
}
