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

// Tracking laws (math_spec §7): the evaluation-point table, holonomic, feedback linearisation
// (O39), heading tracking with the psi_d-dot probes (O40 law part, O41, U28), the Ackermann rules,
// the rotate law and the allow-reversing backstop.
//
// Field probes are synthetic: closest points on lines, circles, a U-turn, crossing branches and a
// three-vertex polyline are built analytically, so these tests do not depend on the curve or
// Tracker implementations.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <vector>

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
using vf::ClosestPoint;
using vf::DistanceField;
using vf::FieldSample;
using vf::GainFunction;
using vf::Kinematics;
using vf::LawInput;
using vf::LawOutput;
using vf::MotionModel;
using vf::Phase;
using vf::Pose2;
using vf::ProbeSample;
using vf::Status;
using vf::TrackingLaw;
using vf::Twist2;
using vf::Vec2;

namespace
{

constexpr double kVr = 0.4;
constexpr double kOmegaBar = 1.5;
constexpr double kDt = 0.05;
constexpr double kKpsi = 2.0;

class Rng
{
public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}
  double uniform(double lo, double hi)
  {
    return std::uniform_real_distribution<double>(lo, hi)(engine_);
  }
  bool coin() { return uniform(0.0, 1.0) < 0.5; }
  Vec2 unit() { return vf::heading(uniform(-vf::kPi, vf::kPi)); }

private:
  std::mt19937_64 engine_;
};

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

// Closest point on the segment or ray a + tau t, tau in [lo, hi] (station s0 + tau).
struct PieceFoot
{
  Vec2 foot;
  double tau{0.0};
  double D{0.0};
};

PieceFoot projectPiece(const Vec2 & x, const Vec2 & a, const Vec2 & t, double lo, double hi)
{
  PieceFoot result;
  result.tau = std::clamp((x - a).dot(t), lo, hi);
  result.foot = a + result.tau * t;
  result.D = (x - result.foot).norm();
  return result;
}

ClosestPoint fromFoot(const Vec2 & x, const Vec2 & foot, const Vec2 & T, double s_star)
{
  ClosestPoint cp;
  cp.x_star = foot;
  cp.D = (x - foot).norm();
  cp.n = cp.D < vf::kEpsD ? Vec2(Vec2::Zero()) : Vec2((x - foot) / cp.D);
  // Exactly orthogonal frame (the field re-orthogonalises anyway).
  cp.T = cp.D < vf::kEpsD ? T : Vec2(vf::sgn(vf::J(cp.n).dot(T)) * vf::J(cp.n));
  cp.s_star = s_star;
  cp.u_star = s_star;
  cp.valid = true;
  return cp;
}

// U-turn of half-width w/2 (legs w apart, math_spec O9): leg A on y = +w/2 traversed in +x up to
// the apex x = 0, a clockwise semicircle about the origin, then leg B on y = -w/2 in -x. Stations
// start at 10 m on leg A's x = -10.
ClosestPoint uTurnClosestPoint(const Vec2 & x, double w)
{
  const double r = 0.5 * w;
  const double s_apex = 10.0;
  if (x.x() > 0.0) {
    const Vec2 u = x.norm() > 0.0 ? Vec2(x.normalized()) : Vec2(Vec2::UnitX());
    const double theta = std::atan2(u.y(), u.x());
    return fromFoot(x, r * u, Vec2(u.y(), -u.x()), s_apex + r * (vf::kHalfPi - theta));
  }
  if (x.y() >= 0.0) {
    return fromFoot(x, Vec2(x.x(), r), Vec2::UnitX(), s_apex + x.x());
  }
  return fromFoot(x, Vec2(x.x(), -r), -Vec2::UnitX(), s_apex + vf::kPi * r - x.x());
}

// Two straight branches crossing at the origin at +-alpha from +x (a lemniscate crossing, Gerono
// a = 4, b = 1.5 crosses at 106.3 deg); the second branch is a lobe (12 m) further along the curve.
ClosestPoint crossingClosestPoint(const Vec2 & x, double alpha)
{
  const Vec2 t1(std::cos(alpha), std::sin(alpha));
  const Vec2 t2(std::cos(alpha), -std::sin(alpha));
  const PieceFoot f1 = projectPiece(x, Vec2::Zero(), t1, -vf::kInf, vf::kInf);
  const PieceFoot f2 = projectPiece(x, Vec2::Zero(), t2, -vf::kInf, vf::kInf);
  if (f1.D <= f2.D) {
    return fromFoot(x, f1.foot, t1, 5.0 + f1.tau);
  }
  return fromFoot(x, f2.foot, t2, 17.0 + f2.tau);
}

// Polyline P0 = (-2, 0), P1 = (0, 0), P2 = (0, 2) (a 90 deg left turn) with semi-infinite rays.
// The vertex owns x when a1 > 0 and a2 > 0; its wedge tangent is s_w J n (math_spec §2.3).
ClosestPoint cornerClosestPoint(const Vec2 & x)
{
  const Vec2 t0 = Vec2::UnitX();
  const Vec2 t1 = Vec2::UnitY();
  const double a1 = x.dot(t0);
  const double a2 = -x.dot(t1);
  if (a1 > 0.0 && a2 > 0.0) {
    ClosestPoint cp = fromFoot(x, Vec2::Zero(), t0, 2.0);
    const Vec2 Jn = vf::J(cp.n);
    cp.T = vf::sgn(Jn.dot(t0 + t1)) * Jn;
    cp.in_wedge = true;
    return cp;
  }
  const PieceFoot f0 = projectPiece(x, Vec2(-2.0, 0.0), t0, -vf::kInf, 2.0);
  const PieceFoot f1 = projectPiece(x, Vec2::Zero(), t1, 0.0, vf::kInf);
  if (f0.D <= f1.D) {
    return fromFoot(x, f0.foot, t0, f0.tau);
  }
  return fromFoot(x, f1.foot, t1, 2.0 + f1.tau);
}

// A field probe over a closest-point function: the static field of math_spec §4.1, psi_d = NaN
// below the hold speed, as the Tracker's probe returns it.
template <typename ClosestFn>
class StaticProbe
{
public:
  StaticProbe(ClosestFn closest, double hold_speed = 0.02)
  : closest_(std::move(closest)), field_(GainFunction(), kVr, 1, 0.1), hold_speed_(hold_speed)
  {
  }

  ProbeSample operator()(const Vec2 & x, double /*time_offset*/) const
  {
    ++calls;
    FieldSample sample;
    field_.evaluate(x, closest_(x), Vec2::Zero(), &sample);
    ProbeSample probe;
    probe.F = sample.F;
    probe.psi_d = sample.F.norm() < hold_speed_ ? vf::kNaN : vf::angleOf(sample.F);
    probe.s_star = sample.s_star;
    probe.D = sample.D;
    return probe;
  }

  Vec2 field(const Vec2 & x) const
  {
    FieldSample sample;
    field_.evaluate(x, closest_(x), Vec2::Zero(), &sample);
    return sample.F;
  }

  mutable int calls{0};

private:
  ClosestFn closest_;
  DistanceField field_;
  double hold_speed_;
};

template <typename ClosestFn>
StaticProbe<ClosestFn> makeProbe(ClosestFn closest)
{
  return StaticProbe<ClosestFn>(closest);
}

LawInput lawInput(const Pose2 & pose, const Vec2 & x, const Vec2 & F)
{
  LawInput input;
  input.pose = pose;
  input.x = x;
  input.F = F;
  input.T_d = Vec2::UnitX();
  input.dt = kDt;
  input.omega_bar = kOmegaBar;
  input.hold_speed = 0.05 * kVr;
  input.last_psi_d = vf::kNaN;
  input.curve_length = vf::kInf;
  input.time_varying = false;
  return input;
}

// Records every probe call.
struct ProbeCall
{
  Vec2 x;
  double time_offset;
};

}  // namespace

// -----------------------------------------------------------------------------------------------
// Evaluation points (math_spec §7.1).

TEST(LawEvaluationPoint, Table)
{
  const Pose2 base{Vec2(1.0, -2.0), 0.7};
  const Vec2 h = vf::heading(base.yaw);
  const Phase phases[] = {
    Phase::kRotateToField, Phase::kFollow, Phase::kFinalApproach, Phase::kRotateToHeading,
    Phase::kStopped};

  Kinematics diff;
  diff.control_point_offset = 0.25;
  for (const Phase phase : phases) {
    SCOPED_TRACE(vf::phaseName(phase));
    const vf::EvaluationPoint fl =
      vf::evaluationPoint(base, phase, TrackingLaw::kFeedbackLinearization, diff);
    if (phase == Phase::kFinalApproach) {
      EXPECT_EQ(fl.x, base.p);
      EXPECT_EQ(fl.x_ref, 0.0);
      EXPECT_EQ(fl.law, TrackingLaw::kHeadingTracking);
    } else {
      EXPECT_NEAR((fl.x - (base.p + 0.25 * h)).norm(), 0.0, 1e-15);
      EXPECT_EQ(fl.x_ref, 0.25);
      EXPECT_EQ(fl.law, TrackingLaw::kFeedbackLinearization);
    }
    // auto resolves to FL for diff and skid.
    EXPECT_EQ(vf::evaluationPoint(base, phase, TrackingLaw::kAuto, diff).x, fl.x);

    const vf::EvaluationPoint ht =
      vf::evaluationPoint(base, phase, TrackingLaw::kHeadingTracking, diff);
    EXPECT_EQ(ht.x, base.p);
    EXPECT_EQ(ht.x_ref, 0.0);
    EXPECT_EQ(ht.law, TrackingLaw::kHeadingTracking);
  }

  // drive_backward: p_c = p - d h, still x_ref = d (the shift is along the drive direction).
  Kinematics backward = diff;
  backward.drive_backward = true;
  const vf::EvaluationPoint back =
    vf::evaluationPoint(base, Phase::kFollow, TrackingLaw::kFeedbackLinearization, backward);
  EXPECT_NEAR((back.x - (base.p - 0.25 * h)).norm(), 0.0, 1e-15);
  EXPECT_EQ(back.x_ref, 0.25);

  Kinematics omni;
  omni.type = MotionModel::kOmnidirectional;
  for (const Phase phase : phases) {
    const vf::EvaluationPoint holo = vf::evaluationPoint(base, phase, TrackingLaw::kAuto, omni);
    EXPECT_EQ(holo.x, base.p);
    EXPECT_EQ(holo.x_ref, 0.0);
    EXPECT_EQ(holo.law, TrackingLaw::kHolonomic);
  }

  Kinematics car;
  car.type = MotionModel::kAckermann;
  car.reference_point_offset = -0.3;
  for (const Phase phase : phases) {
    const vf::EvaluationPoint rear = vf::evaluationPoint(base, phase, TrackingLaw::kAuto, car);
    EXPECT_NEAR((rear.x - (base.p - 0.3 * h)).norm(), 0.0, 1e-15);
    EXPECT_EQ(rear.x_ref, -0.3);
    EXPECT_EQ(rear.law, TrackingLaw::kHeadingTracking);
  }
}

// -----------------------------------------------------------------------------------------------
// Feedback linearisation (math_spec §7.3).

// O39: random psi, F, q, d > 0: ||v h + q d omega J h - F|| <= 1e-12.
TEST(LawFeedbackLinearization, O39ControlPointFollowsTheField)
{
  Rng rng(39);
  double worst = 0.0;
  for (int i = 0; i < 20000; ++i) {
    const double psi = rng.uniform(-10.0, 10.0);
    const Vec2 F(rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0));
    const double q = rng.coin() ? 1.0 : -1.0;
    const double d = rng.uniform(0.01, 2.0);
    LawInput input = lawInput(Pose2{Vec2::Zero(), psi}, Vec2::Zero(), F);
    const LawOutput out = vf::feedbackLinearizationLaw(input, d, q);
    const Vec2 h = vf::heading(psi);
    const Vec2 velocity = out.cmd.vx * h + q * d * out.cmd.wz * vf::J(h);
    worst = std::max(worst, (velocity - F).norm());
    ASSERT_EQ(out.cmd.vy, 0.0);

    // (v, omega) = M F and M^-1 maps it back.
    const vf::Mat2 M = vf::flMatrix(psi, d, q);
    const Vec2 u = M * F;
    ASSERT_NEAR(u.x(), out.cmd.vx, 1e-12);
    ASSERT_NEAR(u.y(), out.cmd.wz, 1e-12 * std::max(1.0, std::abs(u.y())));
    ASSERT_NEAR((vf::flInverse(psi, d, q) * u - F).norm(), 0.0, 1e-12);
    ASSERT_NEAR((M * vf::flInverse(psi, d, q) - vf::Mat2::Identity()).norm(), 0.0, 1e-14);

    // q v = ||F|| cos e_c, omega = ||F|| sin e_c / d with e_c from q h to F.
    const double e_c = out.heading_error;
    ASSERT_NEAR(q * out.cmd.vx, F.norm() * std::cos(e_c), 1e-12);
    ASSERT_NEAR(out.cmd.wz, F.norm() * std::sin(e_c) / d, 1e-10);
  }
  EXPECT_LE(worst, 1e-12);
}

TEST(LawFeedbackLinearization, ZeroFieldGivesZeroCommand)
{
  const LawOutput out =
    vf::feedbackLinearizationLaw(lawInput(Pose2{}, Vec2::Zero(), Vec2::Zero()), 0.2, 1.0);
  EXPECT_EQ(out.cmd.vx, 0.0);
  EXPECT_EQ(out.cmd.wz, 0.0);
  EXPECT_EQ(out.heading_error, 0.0);
  EXPECT_TRUE(std::isnan(out.psi_d));
}

// -----------------------------------------------------------------------------------------------
// Holonomic (math_spec §7.2).

TEST(LawHolonomic, BaseMovesWithTheFieldWhateverOmega)
{
  Rng rng(72);
  for (int i = 0; i < 5000; ++i) {
    const double psi = rng.uniform(-4.0, 4.0);
    const Vec2 F(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
    LawInput input = lawInput(Pose2{Vec2(1.0, 2.0), psi}, Vec2(1.0, 2.0), F);
    input.T_d = rng.unit();
    input.omega_bar = rng.uniform(0.1, 3.0);
    for (const vf::OmniHeadingPolicy policy :
         {vf::OmniHeadingPolicy::kPathTangent, vf::OmniHeadingPolicy::kField,
          vf::OmniHeadingPolicy::kFixed}) {
      const LawOutput out = vf::holonomicLaw(input, policy, kKpsi);
      const Vec2 world = vf::rotate(psi, Vec2(out.cmd.vx, out.cmd.vy));
      ASSERT_NEAR((world - F).norm(), 0.0, 1e-15);
      ASSERT_LE(std::abs(out.cmd.wz), input.omega_bar);
      double target = vf::kNaN;
      if (policy == vf::OmniHeadingPolicy::kPathTangent) {
        target = vf::angleOf(input.T_d);
      } else if (policy == vf::OmniHeadingPolicy::kField && F.norm() >= input.hold_speed) {
        target = vf::angleOf(F);
      }
      if (std::isnan(target)) {
        ASSERT_EQ(out.cmd.wz, 0.0);
      } else {
        const double e = vf::wrapAngle(target - psi);
        ASSERT_DOUBLE_EQ(out.heading_error, e);
        ASSERT_DOUBLE_EQ(out.cmd.wz, std::clamp(kKpsi * e, -input.omega_bar, input.omega_bar));
      }
    }
  }
}

TEST(LawHolonomic, FieldPolicyHoldsBelowTheHoldSpeed)
{
  LawInput input = lawInput(Pose2{Vec2::Zero(), 0.0}, Vec2::Zero(), Vec2(0.0, 0.01));
  input.hold_speed = 0.02;
  input.last_psi_d = 0.3;
  LawOutput out = vf::holonomicLaw(input, vf::OmniHeadingPolicy::kField, kKpsi);
  EXPECT_TRUE(out.held);
  EXPECT_DOUBLE_EQ(out.psi_d, 0.3);
  EXPECT_DOUBLE_EQ(out.cmd.wz, kKpsi * 0.3);
  // No previous target: no heading feedback.
  input.last_psi_d = vf::kNaN;
  out = vf::holonomicLaw(input, vf::OmniHeadingPolicy::kField, kKpsi);
  EXPECT_EQ(out.cmd.wz, 0.0);
  // Above the hold speed the field direction is the target.
  input.F = Vec2(0.0, 0.3);
  out = vf::holonomicLaw(input, vf::OmniHeadingPolicy::kField, kKpsi);
  EXPECT_FALSE(out.held);
  EXPECT_DOUBLE_EQ(out.psi_d, vf::kHalfPi);
  EXPECT_DOUBLE_EQ(out.cmd.wz, kOmegaBar) << "k_psi pi/2 > omega_bar: clamped on its own";
}

// -----------------------------------------------------------------------------------------------
// Heading tracking (math_spec §7.4).

TEST(LawHeadingTracking, SpeedAndFeedbackWithoutProbes)
{
  Rng rng(74);
  for (int i = 0; i < 5000; ++i) {
    const double psi = rng.uniform(-4.0, 4.0);
    const Vec2 F = rng.uniform(0.05, 0.6) * rng.unit();
    const double q = rng.coin() ? 1.0 : -1.0;
    const LawInput input = lawInput(Pose2{Vec2::Zero(), psi}, Vec2::Zero(), F);
    const LawOutput out = vf::headingTrackingLaw(input, vf::FieldProbe(), kKpsi, q, 0.0);
    const double psi_dr = q < 0.0 ? psi + vf::kPi : psi;
    const double e = vf::wrapAngle(vf::angleOf(F) - psi_dr);
    ASSERT_NEAR(out.heading_error, e, 1e-12);
    ASSERT_NEAR(out.cmd.vx, q * F.norm() * std::max(0.0, std::cos(e)), 1e-12);
    ASSERT_GE(q * out.cmd.vx, 0.0);
    ASSERT_NEAR(out.cmd.wz, kKpsi * e, 1e-12);
    ASSERT_EQ(out.cmd.vy, 0.0);
    ASSERT_DOUBLE_EQ(out.psi_d, vf::angleOf(F));
    ASSERT_EQ(out.psi_d_dot, 0.0);
    ASSERT_FALSE(out.held);
    ASSERT_EQ(out.status, Status::kNone);
  }
}

// The hold threshold is LawInput::hold_speed = 0.05 v_r gamma_lim: a low external speed limit
// never freezes psi_d (math_spec §7.4).
TEST(LawHeadingTracking, HoldRule)
{
  std::vector<ProbeCall> calls;
  const auto probe = [&calls](const Vec2 & x, double offset) {
    calls.push_back(ProbeCall{x, offset});
    ProbeSample sample;
    sample.psi_d = 0.0;
    sample.s_star = 0.0;
    sample.D = 0.0;
    return sample;
  };
  const double gamma_lim = 0.1;
  LawInput input = lawInput(Pose2{Vec2::Zero(), 0.2}, Vec2::Zero(), Vec2(0.0, 0.01));
  input.hold_speed = vf::kHoldSpeedFraction * kVr * gamma_lim;  // 0.002
  input.last_psi_d = -0.5;
  LawOutput out = vf::headingTrackingLaw(input, probe, kKpsi, 1.0, 0.0);
  EXPECT_FALSE(out.held) << "||F|| = 0.01 is below 0.05 v_r but above 0.05 v_r gamma_lim";
  EXPECT_DOUBLE_EQ(out.psi_d, vf::kHalfPi);
  EXPECT_EQ(calls.size(), 2U);

  // Below the threshold: psi_d is held at last_psi_d, psi_d-dot = 0 and nothing is probed.
  calls.clear();
  input.F = Vec2(0.0, 0.001);
  out = vf::headingTrackingLaw(input, probe, kKpsi, 1.0, 0.0);
  EXPECT_TRUE(out.held);
  EXPECT_DOUBLE_EQ(out.psi_d, -0.5);
  EXPECT_DOUBLE_EQ(out.heading_error, -0.7);
  EXPECT_EQ(out.psi_d_dot, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.wz, kKpsi * -0.7);
  EXPECT_TRUE(calls.empty());

  // Held with no previous psi_d: hold the drive direction (no turn).
  input.last_psi_d = vf::kNaN;
  out = vf::headingTrackingLaw(input, probe, kKpsi, -1.0, 0.0);
  EXPECT_TRUE(out.held);
  EXPECT_DOUBLE_EQ(out.heading_error, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.wz, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.vx, -0.001);
}

TEST(LawHeadingTracking, ProbesAlongTheHeadingAtDeltaAndHalfPeriod)
{
  std::vector<ProbeCall> calls;
  const auto probe = [&calls](const Vec2 & x, double offset) {
    calls.push_back(ProbeCall{x, offset});
    ProbeSample sample;
    sample.psi_d = 0.1 * x.x() + 0.05 * offset;
    sample.s_star = x.x();
    sample.D = 0.1;
    return sample;
  };
  const Vec2 x(0.5, -0.25);
  for (const double speed : {0.2, 0.4, 0.9}) {
    calls.clear();
    LawInput input = lawInput(Pose2{Vec2::Zero(), 0.0}, x, Vec2(speed, 0.0));
    input.time_varying = true;
    input.dt = 0.05;
    const LawOutput out = vf::headingTrackingLaw(input, probe, kKpsi, 1.0, 0.0);
    const double delta = std::max(0.02, speed * input.dt);
    ASSERT_EQ(calls.size(), 4U);
    EXPECT_NEAR((calls[0].x - (x + delta * Vec2::UnitX())).norm(), 0.0, 1e-15);
    EXPECT_NEAR((calls[1].x - (x - delta * Vec2::UnitX())).norm(), 0.0, 1e-15);
    EXPECT_EQ(calls[0].time_offset, 0.0);
    EXPECT_EQ(calls[1].time_offset, 0.0);
    EXPECT_EQ(calls[2].x, x);
    EXPECT_EQ(calls[3].x, x);
    EXPECT_DOUBLE_EQ(calls[2].time_offset, 0.025);
    EXPECT_DOUBLE_EQ(calls[3].time_offset, -0.025);
    // psi_d-dot = v d(psi_d)/dx + d_t psi_d = 0.1 v + 0.05.
    EXPECT_NEAR(out.psi_d_dot, 0.1 * speed + 0.05, 1e-12);
    EXPECT_FALSE(out.jump);
  }
}

// The probe offset covers the curve's mean vertex spacing, capped at 0.10 m:
// delta = max(0.02, |v| dt, min(l_bar, 0.10)) (math_spec §7.4). NaN or negative spacings are
// ignored, and a faster robot's |v| dt still wins.
TEST(LawHeadingTracking, ProbeOffsetCoversTheVertexSpacing)
{
  std::vector<ProbeCall> calls;
  const auto probe = [&calls](const Vec2 & x, double offset) {
    calls.push_back(ProbeCall{x, offset});
    ProbeSample sample;
    sample.psi_d = 0.1 * x.x();
    sample.s_star = x.x();
    sample.D = 0.1;
    return sample;
  };
  struct Case
  {
    double spacing;
    double speed;
    double delta;
  };
  const Case cases[] = {{0.0, 0.2, 0.02},   {0.035, 0.2, 0.035},   {0.08, 0.2, 0.08},
                        {0.25, 0.2, 0.10},  {vf::kNaN, 0.2, 0.02}, {-1.0, 0.2, 0.02},
                        {0.035, 0.9, 0.045}};
  const Vec2 x(0.5, -0.25);
  for (const Case & c : cases) {
    SCOPED_TRACE("spacing " + std::to_string(c.spacing) + ", speed " + std::to_string(c.speed));
    calls.clear();
    LawInput input = lawInput(Pose2{Vec2::Zero(), 0.0}, x, Vec2(c.speed, 0.0));
    input.dt = 0.05;
    input.vertex_spacing = c.spacing;
    const LawOutput out = vf::headingTrackingLaw(input, probe, kKpsi, 1.0, 0.0);
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_NEAR((calls[0].x - (x + c.delta * Vec2::UnitX())).norm(), 0.0, 1e-15);
    EXPECT_NEAR((calls[1].x - (x - c.delta * Vec2::UnitX())).norm(), 0.0, 1e-15);
    // A linear psi_d: the difference quotient is the gradient whatever delta is.
    EXPECT_NEAR(out.psi_d_dot, 0.1 * c.speed, 1e-12);
    EXPECT_FALSE(out.jump);
  }
}

// O40 (law part): on a smooth circle, unsaturated, |psi_d-dot - grad(psi_d)^T (v h)| <= 1e-3
// rad/s, and psi_d-dot is never zeroed.
TEST(LawHeadingTracking, O40FeedforwardAlongTheMotionOnASmoothCircle)
{
  Rng rng(40);
  double worst = 0.0;
  int samples = 0;
  for (const double R : {1.0, 2.0}) {
    const Vec2 centre(0.3, -0.1);
    const auto closest = [centre, R](const Vec2 & x) { return circleClosestPoint(x, centre, R); };
    const auto probe = makeProbe(closest);
    for (int i = 0; i < 5000; ++i) {
      const double D = rng.uniform(0.0, 0.3);
      const Vec2 x = centre + (R + (rng.coin() ? D : -D)) * rng.unit();
      const Vec2 F = probe.field(x);
      const double q = rng.coin() ? 1.0 : -1.0;
      const double e = rng.uniform(-1.2, 1.2);
      const double psi = vf::angleOf(F) + e + (q < 0.0 ? vf::kPi : 0.0);
      LawInput input = lawInput(Pose2{x, psi}, x, F);
      input.curve_length = vf::kTwoPi * R;
      const LawOutput out = vf::headingTrackingLaw(input, probe, kKpsi, q, 0.0);
      ASSERT_FALSE(out.jump);
      ASSERT_LT(std::abs(out.psi_d_dot), kOmegaBar) << "unsaturated";

      // grad(psi_d) by a fine central difference of the exact field.
      constexpr double kH = 1e-6;
      const auto psi_d = [&probe](const Vec2 & p) { return vf::angleOf(probe.field(p)); };
      const Vec2 grad(
        vf::wrapAngle(psi_d(x + kH * Vec2::UnitX()) - psi_d(x - kH * Vec2::UnitX())) / (2.0 * kH),
        vf::wrapAngle(psi_d(x + kH * Vec2::UnitY()) - psi_d(x - kH * Vec2::UnitY())) / (2.0 * kH));
      const double expected = grad.dot(out.cmd.vx * vf::heading(psi));
      worst = std::max(worst, std::abs(out.psi_d_dot - expected));
      ++samples;
      // omega = clamp(psi_d-dot) + k_psi e.
      ASSERT_NEAR(out.cmd.wz, out.psi_d_dot + kKpsi * out.heading_error, 1e-12);
    }
  }
  EXPECT_EQ(samples, 10000);
  EXPECT_LE(worst, 1e-3);
}

// O40 (zeroing part): on a circle discretised at 2.5 and 5 cm, probes 2 delta >= 4 cm apart
// straddle vertices on most cycles, and the jump test never fires.
TEST(LawHeadingTracking, O40VertexChangesAreNotJumps)
{
  Rng rng(400);
  for (const double R : {0.5, 1.0, 2.0}) {
    for (const double spacing : {0.025, 0.05}) {
      const int n = static_cast<int>(std::ceil(vf::kTwoPi * R / spacing));
      const double chord = 2.0 * R * std::sin(vf::kPi / n);
      const auto closest = [R, n, chord](const Vec2 & x) {
        // Brute force over the closed polygon's segments.
        ClosestPoint best;
        best.D = vf::kInf;
        for (int k = 0; k < n; ++k) {
          const Vec2 a = R * vf::heading(vf::kTwoPi * k / n);
          const Vec2 b = R * vf::heading(vf::kTwoPi * (k + 1) / n);
          const Vec2 t = (b - a).normalized();
          const PieceFoot foot = projectPiece(x, a, t, 0.0, chord);
          if (foot.D < best.D) {
            best = fromFoot(x, foot.foot, t, k * chord + foot.tau);
          }
        }
        return best;
      };
      const auto probe = makeProbe(closest);
      int straddles = 0;
      for (int i = 0; i < 300; ++i) {
        const double D = rng.uniform(0.0, std::min(0.3, 0.5 * R));
        const Vec2 x = (R + (rng.coin() ? D : -D)) * rng.unit();
        const Vec2 F = probe.field(x);
        const double psi = vf::angleOf(F) + rng.uniform(-1.0, 1.0);
        LawInput input = lawInput(Pose2{x, psi}, x, F);
        input.curve_length = n * chord;
        const LawOutput out = vf::headingTrackingLaw(input, probe, kKpsi, 1.0, 0.0);
        ASSERT_FALSE(out.jump) << "R " << R << " spacing " << spacing << " x " << x.transpose();
        ASSERT_TRUE(std::isfinite(out.psi_d_dot));
        const ClosestPoint plus = closest(x + 0.02 * vf::heading(psi));
        const ClosestPoint minus = closest(x - 0.02 * vf::heading(psi));
        straddles += std::floor(plus.s_star / chord) != std::floor(minus.s_star / chord) ? 1 : 0;
      }
      EXPECT_GT(straddles, 100) << "the probes straddle vertices on most cycles";
    }
  }
}

// O41: probes straddling U-turn legs or lemniscate branches see a closest-point jump, so
// psi_d-dot = 0.
TEST(LawHeadingTracking, O41JumpZeroing)
{
  // U-turn, legs 0.8 m apart; the robot crosses the midline before the apex.
  const double w = 0.8;
  const auto uturn = makeProbe([w](const Vec2 & x) { return uTurnClosestPoint(x, w); });
  for (const double before : {0.1, 0.4, 1.0, 2.5}) {
    const Vec2 x(-before, 0.0);
    const Vec2 F = uturn.field(x + Vec2(0.0, 1e-9));
    const LawInput input = lawInput(Pose2{x, vf::kHalfPi}, x, F);
    const LawOutput out = vf::headingTrackingLaw(input, uturn, kKpsi, 1.0, 0.0);
    EXPECT_TRUE(out.jump) << before;
    EXPECT_EQ(out.psi_d_dot, 0.0);
    EXPECT_DOUBLE_EQ(out.cmd.wz, kKpsi * out.heading_error);
  }

  // Lemniscate crossing (106 deg): the robot on the bisector, heading across it.
  const double alpha = 0.5 * 106.3 * vf::kPi / 180.0;
  const auto crossing =
    makeProbe([alpha](const Vec2 & x) { return crossingClosestPoint(x, alpha); });
  for (const double along : {0.05, 0.3, 1.0}) {
    const Vec2 x(along, 0.0);
    const Vec2 F = crossing.field(x + Vec2(0.0, 1e-9));
    const LawInput input = lawInput(Pose2{x, vf::kHalfPi}, x, F);
    const LawOutput out = vf::headingTrackingLaw(input, crossing, kKpsi, 1.0, 0.0);
    EXPECT_TRUE(out.jump) << along;
    EXPECT_EQ(out.psi_d_dot, 0.0);
  }

  // Moving along leg A away from the switch: no jump, the feedforward is kept.
  const Vec2 on_leg(-1.0, 0.3);
  const LawInput along_leg = lawInput(Pose2{on_leg, 0.0}, on_leg, uturn.field(on_leg));
  EXPECT_FALSE(vf::headingTrackingLaw(along_leg, uturn, kKpsi, 1.0, 0.0).jump);
}

// On a straight piece with the robot on the path and heading along it, the feet are exactly
// 2 delta apart with D = 0: the threshold itself. Rounding of the stations must not make that a
// jump.
TEST(LawHeadingTracking, OnTheStraightPathIsNotAJump)
{
  const auto line = makeProbe([](const Vec2 & x) {
    const PieceFoot foot = projectPiece(x, Vec2::Zero(), Vec2::UnitX(), -vf::kInf, vf::kInf);
    return fromFoot(x, foot.foot, Vec2::UnitX(), foot.tau);
  });
  Rng rng(41);
  for (int i = 0; i < 2000; ++i) {
    const Vec2 x(rng.uniform(-300.0, 300.0), 0.0);
    const LawInput input = lawInput(Pose2{x, 0.0}, x, line.field(x));
    const LawOutput out = vf::headingTrackingLaw(input, line, kKpsi, 1.0, 0.0);
    ASSERT_FALSE(out.jump) << x.x();
    ASSERT_EQ(out.psi_d_dot, 0.0);
  }
}

// A vertex change between the probes is not a jump (math_spec §7.4): outside a 90 deg corner the
// feet move from segment 0 to the vertex, and psi_d-dot follows the turning field.
TEST(LawHeadingTracking, VertexChangeKeepsTheFeedforward)
{
  const auto corner = makeProbe([](const Vec2 & x) { return cornerClosestPoint(x); });
  const Vec2 x(0.0, -0.3);
  const Vec2 F = corner.field(x);
  const LawInput input = lawInput(Pose2{x, 0.0}, x, F);
  const LawOutput out = vf::headingTrackingLaw(input, corner, kKpsi, 1.0, 0.0);
  EXPECT_FALSE(out.jump);
  EXPECT_GT(std::abs(out.psi_d_dot), 0.05);
  // It matches the central difference it is defined by.
  const double delta = std::max(0.02, std::abs(out.cmd.vx) * kDt);
  const double psi_plus = vf::angleOf(corner.field(x + Vec2(delta, 0.0)));
  const double psi_minus = vf::angleOf(corner.field(x - Vec2(delta, 0.0)));
  EXPECT_NEAR(
    out.psi_d_dot, out.cmd.vx * vf::wrapAngle(psi_plus - psi_minus) / (2.0 * delta), 1e-12);
}

TEST(LawHeadingTracking, ContourChangesAndNaNProbesZeroTheFeedforward)
{
  const auto contour_flip = [](const Vec2 & x, double) {
    ProbeSample sample;
    sample.psi_d = x.y();
    sample.s_star = 0.0;
    sample.D = 0.1;
    sample.contour_active = x.y() > 0.0;
    sample.sigma_c = x.y() > 0.0 ? 1 : 0;
    return sample;
  };
  const auto side_flip = [](const Vec2 & x, double) {
    ProbeSample sample;
    sample.psi_d = x.y();
    sample.s_star = 0.0;
    sample.D = 0.1;
    sample.contour_active = true;
    sample.sigma_c = x.y() > 0.0 ? 1 : -1;
    return sample;
  };
  const auto nan_ahead = [](const Vec2 & x, double) {
    ProbeSample sample;
    sample.psi_d = x.y() > 0.0 ? vf::kNaN : 0.0;
    sample.s_star = 0.0;
    sample.D = 0.1;
    return sample;
  };
  const auto smooth = [](const Vec2 & x, double) {
    ProbeSample sample;
    sample.psi_d = x.y();
    sample.s_star = 0.0;
    sample.D = 0.1;
    return sample;
  };
  const LawInput input = lawInput(Pose2{Vec2::Zero(), vf::kHalfPi}, Vec2::Zero(), Vec2(0.0, 0.4));
  const LawOutput a = vf::headingTrackingLaw(input, contour_flip, kKpsi, 1.0, 0.0);
  EXPECT_TRUE(a.jump);
  EXPECT_EQ(a.psi_d_dot, 0.0);
  const LawOutput b = vf::headingTrackingLaw(input, side_flip, kKpsi, 1.0, 0.0);
  EXPECT_TRUE(b.jump);
  EXPECT_EQ(b.psi_d_dot, 0.0);
  const LawOutput c = vf::headingTrackingLaw(input, nan_ahead, kKpsi, 1.0, 0.0);
  EXPECT_FALSE(c.jump);
  EXPECT_EQ(c.psi_d_dot, 0.0);
  const LawOutput d = vf::headingTrackingLaw(input, smooth, kKpsi, 1.0, 0.0);
  EXPECT_FALSE(d.jump);
  EXPECT_NEAR(d.psi_d_dot, 0.4, 1e-12) << "v * d(psi_d)/dy = 0.4 * 1";

  // Temporal probes: a NaN or a contour change also zeroes psi_d-dot.
  LawInput moving = input;
  moving.time_varying = true;
  const auto nan_later = [](const Vec2 & x, double offset) {
    ProbeSample sample;
    sample.psi_d = offset > 0.0 ? vf::kNaN : x.y();
    sample.s_star = 0.0;
    sample.D = 0.1;
    return sample;
  };
  EXPECT_EQ(vf::headingTrackingLaw(moving, nan_later, kKpsi, 1.0, 0.0).psi_d_dot, 0.0);
  const auto contour_later = [](const Vec2 & x, double offset) {
    ProbeSample sample;
    sample.psi_d = x.y();
    sample.s_star = 0.0;
    sample.D = 0.1;
    sample.contour_active = offset > 0.0;
    return sample;
  };
  const LawOutput e = vf::headingTrackingLaw(moving, contour_later, kKpsi, 1.0, 0.0);
  EXPECT_TRUE(e.jump);
  EXPECT_EQ(e.psi_d_dot, 0.0);
}

TEST(LawHeadingTracking, ClosedCurveArcDifferenceIsWrapped)
{
  // Probes on either side of a closed curve's seam: s* = 0.01 and L - 0.01.
  const double L = 6.0;
  const auto seam = [L](const Vec2 & x, double) {
    ProbeSample sample;
    sample.psi_d = 0.2 * x.x();
    sample.s_star = x.x() > 0.0 ? 0.01 : L - 0.01;
    sample.D = 0.0;
    return sample;
  };
  LawInput input = lawInput(Pose2{Vec2::Zero(), 0.0}, Vec2::Zero(), Vec2(0.4, 0.0));
  input.curve_length = L;
  const LawOutput closed = vf::headingTrackingLaw(input, seam, kKpsi, 1.0, 0.0);
  EXPECT_FALSE(closed.jump);
  EXPECT_NEAR(closed.psi_d_dot, 0.4 * 0.2, 1e-12);
  input.curve_length = vf::kInf;
  EXPECT_TRUE(vf::headingTrackingLaw(input, seam, kKpsi, 1.0, 0.0).jump);
}

TEST(LawHeadingTracking, TimeDerivativeOfARotatingField)
{
  // A uniform field rotating at 0.3 rad/s: no spatial gradient, d_t psi_d = 0.3.
  const auto rotating = [](const Vec2 &, double offset) {
    ProbeSample sample;
    sample.psi_d = vf::wrapAngle(3.1 + 0.3 * offset);
    sample.s_star = 1.0;
    sample.D = 0.0;
    return sample;
  };
  LawInput input = lawInput(Pose2{Vec2::Zero(), 3.1}, Vec2::Zero(), 0.4 * vf::heading(3.1));
  input.time_varying = true;
  const LawOutput out = vf::headingTrackingLaw(input, rotating, kKpsi, 1.0, 0.0);
  EXPECT_NEAR(out.psi_d_dot, 0.3, 1e-12) << "wrapped across +-pi";
  EXPECT_NEAR(out.cmd.wz, 0.3, 1e-12);
  // Static fields skip the temporal probes; a non-positive dt skips them too.
  input.time_varying = false;
  EXPECT_EQ(vf::headingTrackingLaw(input, rotating, kKpsi, 1.0, 0.0).psi_d_dot, 0.0);
  input.time_varying = true;
  input.dt = 0.0;
  EXPECT_EQ(vf::headingTrackingLaw(input, rotating, kKpsi, 1.0, 0.0).psi_d_dot, 0.0);
}

TEST(LawHeadingTracking, FeedforwardIsClampedToOmegaC)
{
  const auto steep = [](const Vec2 & x, double) {
    ProbeSample sample;
    sample.psi_d = 10.0 * x.x();
    sample.s_star = 0.0;
    sample.D = 0.0;
    return sample;
  };
  const LawInput input = lawInput(Pose2{Vec2::Zero(), 0.0}, Vec2::Zero(), Vec2(0.4, 0.0));
  const LawOutput diff = vf::headingTrackingLaw(input, steep, kKpsi, 1.0, 0.0);
  EXPECT_GT(diff.psi_d_dot, kOmegaBar) << "reported before the clamp";
  EXPECT_DOUBLE_EQ(diff.cmd.wz, kOmegaBar);
  // Ackermann: omega_c = |v| / R_min.
  const LawOutput car = vf::headingTrackingLaw(input, steep, kKpsi, 1.0, 0.5);
  EXPECT_DOUBLE_EQ(car.cmd.wz, 0.4 / 0.5);
}

// -----------------------------------------------------------------------------------------------
// Ackermann (math_spec §7.5).

TEST(LawAckermann, HeadingInfeasibleBeyondNinetyDegrees)
{
  for (const double e_deg : {-170.0, -95.0, -89.0, 0.0, 60.0, 89.9, 90.1, 135.0}) {
    const double e = e_deg * vf::kPi / 180.0;
    const Vec2 F = 0.4 * vf::heading(e);
    const LawInput input = lawInput(Pose2{Vec2::Zero(), 0.0}, Vec2::Zero(), F);
    const LawOutput car = vf::headingTrackingLaw(input, vf::FieldProbe(), kKpsi, 1.0, 0.5);
    EXPECT_EQ(Status::has(car.status, Status::kHeadingInfeasible), std::abs(e) > vf::kHalfPi)
      << e_deg;
    EXPECT_GE(car.cmd.vx, 0.0) << "forward only";
    // diff/skid never raise it.
    const LawOutput diff = vf::headingTrackingLaw(input, vf::FieldProbe(), kKpsi, 1.0, 0.0);
    EXPECT_EQ(diff.status, Status::kNone);
  }
}

TEST(LawAckermann, SpeedBandRaisesSmallForwardSpeeds)
{
  EXPECT_DOUBLE_EQ(vf::applyAckermannBand(0.01, 0.05), 0.05);
  EXPECT_DOUBLE_EQ(vf::applyAckermannBand(0.049, 0.05), 0.05);
  EXPECT_DOUBLE_EQ(vf::applyAckermannBand(0.05, 0.05), 0.05);
  EXPECT_DOUBLE_EQ(vf::applyAckermannBand(0.3, 0.05), 0.3);
  EXPECT_EQ(vf::applyAckermannBand(0.0, 0.05), 0.0) << "the cap and STOPPED command 0";
  EXPECT_DOUBLE_EQ(vf::applyAckermannBand(-0.01, 0.05), -0.01);
  EXPECT_TRUE(std::isnan(vf::applyAckermannBand(vf::kNaN, 0.05)));
}

// The speed-limit floor: with gamma_lim floored at v_ack/v_r (SpeedShaping, math_spec §8.2) a
// low external limit still leaves the car v_ack, so |omega| <= |v|/R_min never deadlocks.
TEST(LawAckermann, SpeedLimitFloorKeepsVAck)
{
  const double v_ack = 0.05;
  const GainFunction gain;
  const DistanceField field(gain, kVr, 1, 0.1);
  const Vec2 x(0.0, -0.1);
  ClosestPoint cp;
  cp.x_star = Vec2::Zero();
  cp.T = Vec2::UnitX();
  cp.n = -Vec2::UnitY();
  cp.D = 0.1;
  cp.valid = true;
  vf::SpeedShaping shaping;
  shaping.gamma_lim = std::max(0.02, v_ack / kVr);  // external limit 2 % floored to 12.5 %
  FieldSample sample;
  field.evaluate(x, cp, Vec2::Zero(), &sample);
  field.shape(shaping.gamma(), shaping.gamma_lim, false, &sample);
  EXPECT_NEAR(sample.F.norm(), v_ack, 1e-15);
  LawInput input = lawInput(Pose2{x, vf::angleOf(sample.F) + 0.3}, x, sample.F);
  input.hold_speed = vf::kHoldSpeedFraction * kVr * shaping.gamma_lim;
  const LawOutput out = vf::headingTrackingLaw(input, vf::FieldProbe(), kKpsi, 1.0, 0.5);
  EXPECT_FALSE(out.held);
  // The law speed ||F|| cos e is raised back to v_ack by the FOLLOW band.
  EXPECT_LT(out.cmd.vx, v_ack);
  EXPECT_DOUBLE_EQ(vf::applyAckermannBand(out.cmd.vx, v_ack), v_ack);
}

// -----------------------------------------------------------------------------------------------
// Rotate law (math_spec §7.6) and the backstop (§9.2).

TEST(LawRotate, StoppingTermFloorAndDeadBand)
{
  vf::RotateLawParams params;
  params.omega_floor = 0.3;
  params.omega_rot = 1.2;
  params.alpha_max = 3.2;
  params.dead_band = 0.0;
  // Large errors: omega_rot; mid errors: the stopping term sqrt(2 alpha |e|); small: the floor.
  EXPECT_DOUBLE_EQ(vf::rotateLawOmega(2.0, params), 1.2);
  EXPECT_DOUBLE_EQ(vf::rotateLawOmega(-2.0, params), -1.2);
  EXPECT_DOUBLE_EQ(vf::rotateLawOmega(0.1, params), std::sqrt(2.0 * 3.2 * 0.1));
  EXPECT_DOUBLE_EQ(vf::rotateLawOmega(-0.1, params), -std::sqrt(2.0 * 3.2 * 0.1));
  EXPECT_DOUBLE_EQ(vf::rotateLawOmega(0.005, params), 0.3);
  EXPECT_EQ(vf::rotateLawOmega(0.0, params), 0.0);
  EXPECT_TRUE(std::isnan(vf::rotateLawOmega(vf::kNaN, params)));

  // ROTATE_TO_HEADING: omega* = 0 inside yaw_tol/2, the floor outside.
  params.dead_band = 0.125;
  EXPECT_EQ(vf::rotateLawOmega(0.125, params), 0.0);
  EXPECT_EQ(vf::rotateLawOmega(-0.1, params), 0.0);
  EXPECT_DOUBLE_EQ(vf::rotateLawOmega(0.126, params), std::sqrt(2.0 * 3.2 * 0.126));
  params.alpha_max = 0.1;
  EXPECT_DOUBLE_EQ(vf::rotateLawOmega(0.126, params), 0.3);
}

// The stopping term brakes within alpha_max: under the CommandLimiter the rotation reaches the
// yaw_tol/2 band without overshooting the goal yaw.
TEST(LawRotate, ReachesTheBandWithoutOvershoot)
{
  vf::CoreConfig config;
  config.motion.min_in_place_angular_vel = 0.25;
  const vf::ControlLaw law(config);
  const vf::CommandLimiter limiter;
  for (const double start : {3.0, 1.0, 0.4, -2.5}) {
    double e = start;
    vf::LimiterState state;
    state.valid = true;
    bool reached = false;
    for (int k = 0; k < 400; ++k) {
      const double target = law.rotate(Phase::kRotateToHeading, e, kOmegaBar, 0.25);
      const Twist2 u = limiter.limit(
        Twist2{0.0, 0.0, target}, state, &state, kDt, law.kinematics(), false, nullptr);
      ASSERT_LE(std::abs(u.wz), kOmegaBar + 1e-12);
      e -= u.wz * kDt;
      ASSERT_GE(e * vf::sgn(start), -0.125) << "start " << start << " overshoot at step " << k;
      if (std::abs(e) <= 0.125 && std::abs(u.wz) < 1e-9) {
        reached = true;
        break;
      }
    }
    EXPECT_TRUE(reached) << start;
  }
}

TEST(LawRotate, ControlLawPhases)
{
  vf::CoreConfig config;
  config.motion.min_in_place_angular_vel = 0.3;
  config.phases.rotate_to_heading_angular_vel = 1.0;
  const vf::ControlLaw law(config);
  // omega_rot = min(omega_rth, omega_bar).
  EXPECT_DOUBLE_EQ(law.rotate(Phase::kRotateToField, 3.0, 1.5, 0.25), 1.0);
  EXPECT_DOUBLE_EQ(law.rotate(Phase::kRotateToField, 3.0, 0.6, 0.25), 0.6);
  // ROTATE_TO_FIELD has no dead band: the floor applies down to 0.
  EXPECT_DOUBLE_EQ(law.rotate(Phase::kRotateToField, 0.01, 1.5, 0.25), 0.3);
  EXPECT_EQ(law.rotate(Phase::kRotateToHeading, 0.01, 1.5, 0.25), 0.0);
  EXPECT_DOUBLE_EQ(law.rotate(Phase::kRotateToHeading, -0.2, 1.5, 0.25), -1.0);
  EXPECT_DOUBLE_EQ(law.rotate(Phase::kRotateToHeading, -0.15, 1.5, 0.25), -std::sqrt(6.4 * 0.15));
  // STOPPED bypasses the floors.
  EXPECT_EQ(law.rotate(Phase::kStopped, 2.0, 1.5, 0.25), 0.0);
}

TEST(LawBackstop, ClampsReverseDriveSpeedOnly)
{
  const Twist2 back{-0.2, 0.05, 0.7};
  const Twist2 forward = vf::applyReversingBackstop(back, 1.0);
  EXPECT_EQ(forward.vx, 0.0);
  EXPECT_EQ(forward.vy, 0.05);
  EXPECT_EQ(forward.wz, 0.7);
  const Twist2 kept = vf::applyReversingBackstop(back, -1.0);
  EXPECT_EQ(kept.vx, -0.2) << "drive_backward: q v = 0.2 >= 0";
  EXPECT_EQ(vf::applyReversingBackstop(Twist2{0.3, 0.0, 0.1}, -1.0).vx, 0.0);
  EXPECT_EQ(vf::applyReversingBackstop(Twist2{0.3, 0.0, 0.1}, 1.0).vx, 0.3);
}

// -----------------------------------------------------------------------------------------------
// ControlLaw.

TEST(LawControlLaw, ConstructionAndDispatch)
{
  const vf::CoreConfig config{};
  const vf::ControlLaw fl(config);
  EXPECT_EQ(fl.law(), TrackingLaw::kFeedbackLinearization);
  EXPECT_EQ(fl.activeLaw(Phase::kFollow), TrackingLaw::kFeedbackLinearization);
  EXPECT_EQ(fl.activeLaw(Phase::kFinalApproach), TrackingLaw::kHeadingTracking);
  const Pose2 base{Vec2(0.5, 0.5), 0.3};
  EXPECT_EQ(
    fl.evaluationPoint(base, Phase::kFollow).x,
    vf::evaluationPoint(base, Phase::kFollow, TrackingLaw::kFeedbackLinearization, fl.kinematics())
      .x);

  // FOLLOW runs FL; FINAL_APPROACH runs heading tracking at base_link.
  const LawInput input = lawInput(base, base.p, Vec2(0.0, 0.4));
  const LawOutput follow = fl.track(Phase::kFollow, input, vf::FieldProbe());
  EXPECT_NEAR(follow.cmd.vx, vf::heading(0.3).dot(Vec2(0.0, 0.4)), 1e-15);
  const LawOutput final_approach = fl.track(Phase::kFinalApproach, input, vf::FieldProbe());
  EXPECT_NEAR(final_approach.heading_error, vf::kHalfPi - 0.3, 1e-15);
  EXPECT_DOUBLE_EQ(final_approach.cmd.wz, kKpsi * (vf::kHalfPi - 0.3));

  // drive_backward is honoured by the hand-over.
  vf::CoreConfig backward = config;
  backward.motion.drive_backward = true;
  const LawOutput back =
    vf::ControlLaw(backward).track(Phase::kFinalApproach, input, vf::FieldProbe());
  EXPECT_LE(back.cmd.vx, 0.0);

  // Ackermann: forward only (q = +1) whatever drive_backward, omega_c = |v|/R_min.
  vf::CoreConfig car;
  car.motion.motion_model = MotionModel::kAckermann;
  car.motion.min_turning_radius = 0.5;
  car.motion.drive_backward = true;
  const vf::ControlLaw ackermann(car);
  EXPECT_EQ(ackermann.law(), TrackingLaw::kHeadingTracking);
  const LawOutput forward = ackermann.track(Phase::kFollow, input, vf::FieldProbe());
  EXPECT_GE(forward.cmd.vx, 0.0);

  vf::CoreConfig omni;
  omni.motion.motion_model = MotionModel::kOmnidirectional;
  omni.motion.omni_heading_policy = vf::OmniHeadingPolicy::kFixed;
  const vf::ControlLaw holonomic(omni);
  EXPECT_EQ(holonomic.law(), TrackingLaw::kHolonomic);
  const LawOutput holo = holonomic.track(Phase::kFinalApproach, input, vf::FieldProbe());
  EXPECT_NEAR(
    (vf::rotate(base.yaw, Vec2(holo.cmd.vx, holo.cmd.vy)) - Vec2(0.0, 0.4)).norm(), 0.0, 1e-15);
  EXPECT_EQ(holo.cmd.wz, 0.0);
}

TEST(LawControlLaw, RejectsInvalidConfigurations)
{
  vf::CoreConfig config;
  for (const double d : {0.0, -0.1, vf::kNaN}) {
    config.motion.control_point_offset = d;
    EXPECT_THROW(vf::ControlLaw{config}, vf::ConfigError) << d;
  }
  // Heading tracking does not use d.
  config.motion.tracking_law = TrackingLaw::kHeadingTracking;
  EXPECT_NO_THROW(vf::ControlLaw{config});

  vf::CoreConfig omni;
  omni.motion.motion_model = MotionModel::kOmnidirectional;
  omni.motion.tracking_law = TrackingLaw::kFeedbackLinearization;
  EXPECT_THROW(vf::ControlLaw{omni}, vf::ConfigError);
  vf::CoreConfig diff;
  diff.motion.tracking_law = TrackingLaw::kHolonomic;
  EXPECT_THROW(vf::ControlLaw{diff}, vf::ConfigError);
  vf::CoreConfig car;
  car.motion.motion_model = MotionModel::kAckermann;
  car.motion.tracking_law = TrackingLaw::kFeedbackLinearization;
  EXPECT_THROW(vf::ControlLaw{car}, vf::ConfigError);
  vf::CoreConfig skid;
  skid.motion.motion_model = MotionModel::kSkidSteer;
  skid.motion.tracking_law = TrackingLaw::kHeadingTracking;
  EXPECT_NO_THROW(vf::ControlLaw{skid});
}

// -----------------------------------------------------------------------------------------------
// Upstream regression U28 (docs/design/upstream_audit.md): upstream differentiated the field by a
// forward difference that spiked as ||Delta f||/1e-4 whenever the nearest sample switched.

TEST(UpstreamRegression, U28_FeedforwardIgnoresClosestPointJump)
{
  // Heading tracking near a U-turn apex (legs 0.8 m apart): the robot cuts across the midline,
  // so the probes x -+ delta h straddle the switch from leg A to leg B.
  const double w = 0.8;
  const auto uturn = makeProbe([w](const Vec2 & x) { return uTurnClosestPoint(x, w); });
  Kinematics diff;
  for (const double before : {0.05, 0.3, 0.8}) {
    // psi_d = 25.8 deg on leg A's side; these headings keep v = ||F|| cos e > 0.
    for (const double heading_deg : {50.0, 70.0, 90.0}) {
      const Vec2 x(-before, 0.0);
      const double psi = heading_deg * vf::kPi / 180.0;
      const Vec2 F = uturn.field(x + Vec2(0.0, 1e-9));
      const LawInput input = lawInput(Pose2{x, psi}, x, F);
      const LawOutput out = vf::headingTrackingLaw(input, uturn, kKpsi, 1.0, 0.0);

      // The naive central difference would command a spike far above omega_bar...
      const double delta = std::max(0.02, std::abs(out.cmd.vx) * kDt);
      const ProbeSample plus = uturn(x + delta * vf::heading(psi), 0.0);
      const ProbeSample minus = uturn(x - delta * vf::heading(psi), 0.0);
      const double naive = out.cmd.vx * vf::wrapAngle(plus.psi_d - minus.psi_d) / (2.0 * delta);
      EXPECT_GT(std::abs(naive), kOmegaBar);

      // ...but the jump test zeroes psi_d-dot this cycle,
      EXPECT_TRUE(out.jump);
      EXPECT_EQ(out.psi_d_dot, 0.0);
      EXPECT_DOUBLE_EQ(out.cmd.wz, kKpsi * out.heading_error);
      // and the command stays within the limits after the kinematic box.
      const vf::BoxResult box = vf::uniformBox(out.cmd, diff, kOmegaBar);
      EXPECT_LE(std::abs(box.cmd.wz), kOmegaBar + 1e-12);
      EXPECT_LE(std::abs(box.cmd.vx), diff.max_linear_vel + 1e-12);
    }
  }
}
