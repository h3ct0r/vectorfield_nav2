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

// Closed-loop oracles of the distance field (math_spec §4.2–§4.4, §12.3): O20 translating
// circle, O21 rigid moving frame, O22 ultimate bound.
//
// "Single integrator ẋ = Φ with no limits" is realised two ways, both allowed by the headers:
// (1) on the field and curve APIs (Curve::closestPoint + Curve::velocity + DistanceField),
// integrated by explicit Euler with step 1e-3 s; (2) through the Tracker with an omnidirectional
// plant at 1 kHz, holonomic law, `omni_heading_policy: fixed` (ω = 0) and limits so large that
// neither the kinematic box nor the acceleration stage acts, so the base moves with exactly Φ
// over each 1 ms period.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include "test_support.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/gains.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

// O20: circle of radius 1.5 m translating at u = (0.1, 0.05) m/s, centred at the origin at t = 0.
constexpr double kR20 = 1.5;
const Vec2 kU20(0.1, 0.05);

Vec2 centre20(double t) { return kU20 * t; }

// Analytic distance to the translating circle and its counter-clockwise tangent at the foot.
double distance20(const Vec2 & x, double t) { return std::abs((x - centre20(t)).norm() - kR20); }
Vec2 tangent20(const Vec2 & x, double t) { return vf::J((x - centre20(t)).normalized()); }

// (I − T Tᵀ) u.
Vec2 projectNormal(const Vec2 & T, const Vec2 & u) { return u - T.dot(u) * T; }

vf::DistanceField tcField()
{
  return vf::DistanceField(vf::GainFunction(vf::GainType::kAtan, kKf), kVr, 1, 0.1);
}

// An omnidirectional configuration with no effective limits, for ẋ = Φ through the Tracker.
vf::CoreConfig unlimitedOmniConfig(double frequency)
{
  vf::CoreConfig c = tcConfig(vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic);
  c.motion.omni_heading_policy = vf::OmniHeadingPolicy::kFixed;
  c.motion.max_linear_vel = 100.0;
  c.motion.max_lateral_vel = 100.0;
  c.motion.max_angular_vel = 100.0;
  c.motion.max_linear_accel = 1.0e6;
  c.motion.max_linear_decel = 1.0e6;
  c.motion.max_angular_accel = 1.0e6;
  c.phases.rotate_to_heading_angular_vel = 1.5;
  c.node.controller_frequency = frequency;
  return c;
}

// Frame of the O20 moving-frame variant: a static circle translated with the frame.
vf::CurveFrame translatingFrame(double t)
{
  vf::CurveFrame frame;
  frame.T_eval_curve = vf::Isometry2(0.0, centre20(t));
  frame.twist_in_curve = Twist2{kU20.x(), kU20.y(), 0.0};
  frame.moving = true;
  return frame;
}

// Pose of a frame moving with a constant body twist (v, ω) from pose0 for τ seconds (τ may be
// negative): the closed form of math_spec §10.2, written out independently.
Pose2 constantTwistPose(const Pose2 & pose0, const Vec2 & v, double w, double tau)
{
  const double a = w * tau;
  const double s = std::abs(w) > 1e-12 ? std::sin(a) / w : tau;
  const double c = std::abs(w) > 1e-12 ? (1.0 - std::cos(a)) / w : 0.0;
  const Vec2 body(s * v.x() - c * v.y(), c * v.x() + s * v.y());
  return Pose2{pose0.p + vf::rotate(pose0.yaw, body), pose0.yaw + a};
}

}  // namespace

// O20, first part (math_spec §4.2 [Derived] Φ_T = Π_T v_c): an expression circle translating at
// u gives Φ_T = (I − TTᵀ)u to 1e-9, with T the analytic tangent of the true closest point.
// Field level: Curve::closestPoint, Curve::velocity (δ_t = 1e-4 central difference) and
// DistanceField::evaluate. 500 seeded random (x, t) samples away from the centre.
TEST(OracleField, O20_FeedforwardIsProjectedTranslation_Expression)
{
  const auto curve =
    makeExpression("1.5*cos(s) + 0.1*t", "1.5*sin(s) + 0.05*t", 0.0, vf::kTwoPi, true);
  ASSERT_TRUE(curve->isTimeVarying());
  const vf::DistanceField field = tcField();
  std::mt19937_64 rng(2020);
  std::uniform_real_distribution<double> t_dist(0.0, 20.0);
  std::uniform_real_distribution<double> r_dist(0.4, 3.0);
  std::uniform_real_distribution<double> a_dist(-vf::kPi, vf::kPi);
  double worst = 0.0;
  for (int i = 0; i < 500; ++i) {
    const double t = t_dist(rng);
    const double a = a_dist(rng);
    const Vec2 x = centre20(t) + r_dist(rng) * Vec2(std::cos(a), std::sin(a));
    vf::EvalContext ctx;
    ctx.t = t;
    ctx.direction = 1;
    const vf::ClosestPoint cp = curve->closestPoint(x, ctx, vf::CurveHint{});
    ASSERT_TRUE(cp.valid);
    vf::FieldSample s;
    field.evaluate(x, cp, curve->velocity(cp, t), &s);
    ASSERT_FALSE(Status::has(s.status, Status::kAssumption3Clamped));
    const Vec2 expected = projectNormal(tangent20(x, t), kU20);
    worst = std::max(worst, (s.Phi_T - expected).norm());
    EXPECT_NEAR(cp.D, distance20(x, t), 1e-9);
  }
  EXPECT_LE(worst, 1e-9);
}

// O20, first part, through the Tracker (math_spec §4.2 "Sources of v_c"): a static built-in
// circle in a frame translating at u (FRAME_MOVING) gives FieldSample::Phi_T = (I − TTᵀ)u to
// 1e-9 at the law's point (holonomic: base_link). 200 seeded random single steps.
TEST(OracleField, O20_FeedforwardIsProjectedTranslation_MovingFrame)
{
  const auto curve = makeBuiltin("ellipse", {{"a", kR20}, {"b", kR20}});
  const vf::Tracker tracker(
    tcConfig(vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic));
  vf::StepScratch scratch;
  std::mt19937_64 rng(2021);
  std::uniform_real_distribution<double> t_dist(0.0, 20.0);
  std::uniform_real_distribution<double> r_dist(0.4, 3.0);
  std::uniform_real_distribution<double> a_dist(-vf::kPi, vf::kPi);
  double worst = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double t = t_dist(rng);
    const double a = a_dist(rng);
    vf::TrackerInput input;
    input.pose = Pose2{centre20(t) + r_dist(rng) * Vec2(std::cos(a), std::sin(a)), a_dist(rng)};
    input.t = t;
    input.dt = kPeriod;
    input.curve = curve.get();
    input.frame = translatingFrame(t);
    const vf::TrackerState in = vf::Tracker::initialState();
    vf::TrackerState out_state;
    const vf::TrackerOutput out = tracker.step(input, in, &out_state, scratch);
    ASSERT_TRUE(out.field.valid);
    ASSERT_FALSE(Status::has(out.status, Status::kAssumption3Clamped));
    EXPECT_NEAR((out.field.v_c - kU20).norm(), 0.0, 1e-9);
    const Vec2 expected = projectNormal(tangent20(out.field.x_eval, t), kU20);
    worst = std::max(worst, (out.field.Phi_T - expected).norm());
  }
  EXPECT_LE(worst, 1e-9);
}

// O20, second part (math_spec §4.4 Proposition 2): single integrator ẋ = Φ with no limits,
// explicit Euler step 1e-3 s, from D0 = 0.5 outside the translating expression circle:
// D < 1e-3 after 20 s (reference 8.3e-5, math_spec Appendix B).
TEST(OracleField, O20_SingleIntegratorConverges_Expression)
{
  const auto curve =
    makeExpression("1.5*cos(s) + 0.1*t", "1.5*sin(s) + 0.05*t", 0.0, vf::kTwoPi, true);
  const vf::DistanceField field = tcField();
  constexpr double h = 1e-3;
  Vec2 x = centre20(0.0) + (kR20 + 0.5) * Vec2(1.0, 0.0);
  vf::CurveHint hint;
  double t = 0.0;
  for (int k = 0; k < 20000; ++k) {
    t = k * h;
    vf::EvalContext ctx;
    ctx.t = t;
    ctx.direction = 1;
    const vf::ClosestPoint cp = curve->closestPoint(x, ctx, hint);
    ASSERT_TRUE(cp.valid) << "k = " << k;
    vf::FieldSample s;
    field.evaluate(x, cp, curve->velocity(cp, t), &s);
    x += h * s.Phi;
    hint = vf::makeHint(cp);
  }
  EXPECT_LT(distance20(x, 20.0), 1e-3);
}

// O20, second part, through the Tracker: an omnidirectional plant with no effective limits at
// 1 kHz (ZOH step 1e-3 s) on the built-in circle in a translating frame, from D0 = 0.5 inside
// the circle: D < 1e-3 after 20 s. Timeout: exactly 20 s of simulated time.
TEST(OracleField, O20_SingleIntegratorConverges_TrackerOmniUnlimited)
{
  const auto curve = makeBuiltin("ellipse", {{"a", kR20}, {"b", kR20}});
  vf::sim::SimulationOptions options;
  options.period = 1e-3;
  options.stop_when_settled = false;
  Runner runner(unlimitedOmniConfig(1000.0), options);
  const double a0 = 2.0;
  runner.reset(Pose2{centre20(0.0) + (kR20 - 0.5) * Vec2(std::cos(a0), std::sin(a0)), 0.3});
  runner.input().curve = curve.get();
  runner.setPreStep(
    [](Runner & r, vf::TrackerInput & in) { in.frame = translatingFrame(r.time()); });
  runner.run(20.0);
  ASSERT_EQ(runner.cycles().size(), 20000U);
  for (const Cycle & c : runner.cycles()) {
    ASSERT_FALSE(Status::has(c.out.status, Status::kAssumption3Clamped));
    ASSERT_FALSE(Status::has(c.out.status, Status::kNonFinite));
  }
  EXPECT_NEAR(runner.pose().yaw, 0.3, 1e-12) << "fixed heading policy commands ω = 0";
  EXPECT_LT(distance20(runner.pose().p, runner.time()), 1e-3);
}

// O21 Rigid moving frame (math_spec §4.2): R_F = R(π/2), p_F = (1, 2), v_F = (0.1, 0) in F,
// ω_F = 0.3. FieldSample::v_c equals the central difference (δ = 1e-5 s) of the world point
// p_F(t) + R_F(t) r_F(u*) within 1e-6, with r_F(u*) the foot mapped back into F. A static
// built-in ellipse, 50 seeded random robot positions.
TEST(OracleField, O21_RigidMovingFrameVelocity)
{
  const auto curve = makeBuiltin("ellipse", {{"a", 0.6}, {"b", 0.4}});
  const vf::Tracker tracker(tcConfig());
  vf::StepScratch scratch;
  const Pose2 frame_pose{Vec2(1.0, 2.0), vf::kHalfPi};
  const Vec2 v_f(0.1, 0.0);
  const double w_f = 0.3;
  constexpr double delta = 1e-5;
  std::mt19937_64 rng(2121);
  std::uniform_real_distribution<double> r_dist(0.1, 1.5);
  std::uniform_real_distribution<double> a_dist(-vf::kPi, vf::kPi);
  for (int i = 0; i < 50; ++i) {
    vf::TrackerInput input;
    const double a = a_dist(rng);
    input.pose = Pose2{frame_pose.p + r_dist(rng) * Vec2(std::cos(a), std::sin(a)), a_dist(rng)};
    input.t = 3.0;
    input.dt = kPeriod;
    input.curve = curve.get();
    input.frame.T_eval_curve = vf::Isometry2::fromPose(frame_pose);
    input.frame.twist_in_curve = Twist2{v_f.x(), v_f.y(), w_f};
    input.frame.moving = true;
    vf::TrackerState next;
    const vf::TrackerOutput out = tracker.step(input, vf::Tracker::initialState(), &next, scratch);
    ASSERT_TRUE(out.field.valid);
    const Vec2 foot_in_f = input.frame.T_eval_curve.applyInverse(out.field.x_star);
    const Pose2 plus = constantTwistPose(frame_pose, v_f, w_f, delta);
    const Pose2 minus = constantTwistPose(frame_pose, v_f, w_f, -delta);
    const Vec2 w_plus = plus.p + vf::rotate(plus.yaw, foot_in_f);
    const Vec2 w_minus = minus.p + vf::rotate(minus.yaw, foot_in_f);
    const Vec2 expected = (w_plus - w_minus) / (2.0 * delta);
    EXPECT_LE((out.field.v_c - expected).norm(), 1e-6) << "sample " << i;
  }
}

// O21 with a time-varying shape in the moving frame (math_spec §4.2: v_c = R_F(v_F + ω_F J x*_F
// + ∂_t r_F)): the central difference of p_F(t) + R_F(t) r_F(u*, t) at fixed u*.
TEST(OracleField, O21_RigidMovingFrameVelocity_TimeVaryingShape)
{
  const auto curve = makeExpression("0.6*cos(s) + 0.05*t", "0.4*sin(s)", 0.0, vf::kTwoPi, true);
  const vf::Tracker tracker(tcConfig());
  vf::StepScratch scratch;
  const Pose2 frame_pose{Vec2(1.0, 2.0), vf::kHalfPi};
  const Vec2 v_f(0.1, 0.0);
  const double w_f = 0.3;
  const double t0 = 2.0;
  constexpr double delta = 1e-5;
  std::mt19937_64 rng(2122);
  std::uniform_real_distribution<double> r_dist(0.1, 1.5);
  std::uniform_real_distribution<double> a_dist(-vf::kPi, vf::kPi);
  for (int i = 0; i < 20; ++i) {
    vf::TrackerInput input;
    const double a = a_dist(rng);
    input.pose = Pose2{frame_pose.p + r_dist(rng) * Vec2(std::cos(a), std::sin(a)), a_dist(rng)};
    input.t = t0;
    input.dt = kPeriod;
    input.curve = curve.get();
    input.frame.T_eval_curve = vf::Isometry2::fromPose(frame_pose);
    input.frame.twist_in_curve = Twist2{v_f.x(), v_f.y(), w_f};
    input.frame.moving = true;
    vf::TrackerState next;
    const vf::TrackerOutput out = tracker.step(input, vf::Tracker::initialState(), &next, scratch);
    ASSERT_TRUE(out.field.valid);
    // u* of the foot: the shape is (0.6 cos u + 0.05 t, 0.4 sin u) in F.
    const Vec2 foot_in_f = input.frame.T_eval_curve.applyInverse(out.field.x_star);
    const double u = std::atan2(foot_in_f.y() / 0.4, (foot_in_f.x() - 0.05 * t0) / 0.6);
    const auto shape = [u](double t) {
      return Vec2(0.6 * std::cos(u) + 0.05 * t, 0.4 * std::sin(u));
    };
    const Pose2 plus = constantTwistPose(frame_pose, v_f, w_f, delta);
    const Pose2 minus = constantTwistPose(frame_pose, v_f, w_f, -delta);
    const Vec2 w_plus = plus.p + vf::rotate(plus.yaw, shape(t0 + delta));
    const Vec2 w_minus = minus.p + vf::rotate(minus.yaw, shape(t0 - delta));
    const Vec2 expected = (w_plus - w_minus) / (2.0 * delta);
    EXPECT_LE((out.field.v_c - expected).norm(), 1e-6) << "sample " << i;
  }
}

// O22 Ultimate bound (math_spec §4.4, [Thesis 3.38, 5.4]): ẋ = Φ + δ on a static circle of radius
// 1 m with δ = 0.1 n (n the analytic normal, pointing away from the curve), Euler step 1e-3 s:
// after 20 s, D ≤ tan(π/2 · 0.25)/k_f + 1e-3 = 0.2081 m. Three starts (outside, inside, near).
TEST(OracleField, O22_UltimateBound_FieldLevel)
{
  const auto curve = makeBuiltin("ellipse", {{"a", 1.0}, {"b", 1.0}});
  const vf::DistanceField field = tcField();
  const double bound = std::tan(vf::kHalfPi * 0.25) / kKf + 1e-3;
  ASSERT_NEAR(bound, 0.2081, 1e-4);
  const std::vector<Vec2> starts = {
    2.0 * Vec2(std::cos(0.3), std::sin(0.3)), 0.5 * Vec2(std::cos(2.0), std::sin(2.0)),
    1.1 * Vec2(std::cos(-2.5), std::sin(-2.5))};
  for (const Vec2 & start : starts) {
    Vec2 x = start;
    vf::CurveHint hint;
    constexpr double h = 1e-3;
    for (int k = 0; k < 20000; ++k) {
      vf::EvalContext ctx;
      ctx.t = k * h;
      const vf::ClosestPoint cp = curve->closestPoint(x, ctx, hint);
      ASSERT_TRUE(cp.valid);
      vf::FieldSample s;
      field.evaluate(x, cp, Vec2::Zero(), &s);
      const double r = x.norm();
      const Vec2 n = (r > 1.0 ? 1.0 : -1.0) * x / r;
      x += h * (s.Phi + 0.1 * n);
      hint = vf::makeHint(cp);
    }
    EXPECT_LE(std::abs(x.norm() - 1.0), bound) << "start " << start.transpose();
  }
}

// O22 through the Tracker: the unlimited omnidirectional plant at 1 kHz with the simulator's
// world-frame disturbance δ(t, pose) = 0.1 n(pose); after 20 s, D ≤ 0.2081 m.
TEST(OracleField, O22_UltimateBound_TrackerWithDisturbance)
{
  const auto curve = makeBuiltin("ellipse", {{"a", 1.0}, {"b", 1.0}});
  vf::sim::SimulationOptions options;
  options.period = 1e-3;
  options.stop_when_settled = false;
  options.disturbance = [](double, const Pose2 & pose) {
    const double r = pose.p.norm();
    return Vec2(((r > 1.0 ? 0.1 : -0.1) / r) * pose.p);
  };
  Runner runner(unlimitedOmniConfig(1000.0), options);
  runner.reset(Pose2{1.8 * Vec2(std::cos(1.0), std::sin(1.0)), 0.0});
  runner.input().curve = curve.get();
  runner.run(20.0);
  const double bound = std::tan(vf::kHalfPi * 0.25) / kKf + 1e-3;
  EXPECT_LE(std::abs(runner.pose().p.norm() - 1.0), bound);
}
