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

// Moving curve frames in the 3D core (math_spec §14.7, oracles O3D-10 to O3D-14): the SE(3)
// exponential and logarithm, the feedforward of a translating frame, the curve velocity against
// a finite difference of the moving frame, closed loops on a translating circle and on a helix
// in a rotating frame, the goal hold riding with the frame, FrameTwistEstimator3, and the static
// frame's equality with the frame-free tracker.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "vectorfield_core/d3.hpp"
#include "vectorfield_core/errors.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace d3 = vectorfield_core::d3;
using d3::Isometry3;
using d3::Vec3;

d3::CurveSpec3 builtin(
  const std::string & name, const std::vector<std::string> & names = {},
  const std::vector<double> & values = {})
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kBuiltin;
  spec.builtin_shape = name;
  spec.parameter_names = names;
  spec.parameter_values = values;
  return spec;
}

Isometry3 pose(double yaw, double pitch, double roll, const Vec3 & c)
{
  Isometry3 T = Isometry3::Identity();
  T.linear() = d3::rotationZYX(yaw, pitch, roll);
  T.translation() = c;
  return T;
}

d3::CurveFrame3 moving(const Vec3 & v, const Vec3 & w, const Isometry3 & T = Isometry3::Identity())
{
  d3::CurveFrame3 frame;
  frame.T_eval_curve = T;
  frame.twist.v = v;
  frame.twist.w = w;
  frame.moving = true;
  return frame;
}

double maxAbs(const Isometry3 & a, const Isometry3 & b)
{
  return (a.matrix() - b.matrix()).cwiseAbs().maxCoeff();
}

struct LoopRun
{
  std::vector<d3::sim::SimulationRecord3> records;
  d3::Tracker3Output last;
  d3::Pose3 pose;
};

LoopRun simulate(
  const d3::Curve3 & curve, const Vec3 & start, double duration, const d3::CurveFrame3 & frame,
  bool stop_when_settled = true)
{
  d3::Tracker3Config config;
  config.desired_speed = 0.5;
  d3::sim::SimulationOptions3 options;
  options.period = 0.05;
  options.frame = frame;
  options.stop_when_settled = stop_when_settled;
  d3::sim::ClosedLoopSimulator3 simulator(d3::Tracker3(config), options);
  d3::Pose3 p;
  p.p = start;
  simulator.reset(p);
  d3::Tracker3Input input;
  input.curve = &curve;
  simulator.run(input, duration);
  return LoopRun{simulator.records(), simulator.lastOutput(), simulator.pose()};
}

std::uint32_t statusUnion(const LoopRun & run)
{
  std::uint32_t bits = 0;
  for (const auto & r : run.records) {
    bits |= r.status;
  }
  return bits;
}

// ------------------------------------------------------------------------------------------------
// SE(3)

TEST(D3Frame, ExpLogRoundTripAndSeriesContinuity)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (int i = 0; i < 2000; ++i) {
    d3::FrameTwist3 xi;
    xi.v = Vec3(unit(rng), unit(rng), unit(rng));
    const Vec3 axis = Vec3(unit(rng), unit(rng), unit(rng)).normalized();
    // Angles from 1e-9 to just below pi, both sides of the series switch at 1e-2.
    const double theta = std::pow(10.0, -9.0 + 9.4 * (0.5 + 0.5 * unit(rng)));
    xi.w = std::min(theta, 3.1) * axis;
    const Isometry3 T = d3::expSE3(xi, 1.0);
    ASSERT_TRUE(d3::isRigid(T));
    const d3::FrameTwist3 back = d3::logSE3(T);
    EXPECT_LT((back.w - xi.w).norm(), 1e-12) << "theta " << theta;
    EXPECT_LT((back.v - xi.v).norm(), 1e-11) << "theta " << theta;
    // exp(a xi) exp(b xi) = exp((a + b) xi).
    EXPECT_LT(maxAbs(d3::expSE3(xi, 0.3) * d3::expSE3(xi, 0.7), d3::expSE3(xi, 1.0)), 1e-14 * 50);
  }
  // Continuity across the series switch.
  d3::FrameTwist3 xi;
  xi.v = Vec3(0.3, -0.2, 0.5);
  xi.w = Vec3(0.0, 0.6, 0.8);
  const Isometry3 below = d3::expSE3(xi, 1e-2 * (1.0 - 1e-12));
  const Isometry3 above = d3::expSE3(xi, 1e-2 * (1.0 + 1e-12));
  EXPECT_LT(maxAbs(below, above), 1e-13);
  // Near pi the quaternion route keeps the angle.
  xi.w = Vec3(0.0, 0.0, vf::kPi - 1e-9);
  EXPECT_NEAR(d3::logSE3(d3::expSE3(xi, 1.0)).w.z(), vf::kPi - 1e-9, 1e-12);
}

// ------------------------------------------------------------------------------------------------
// Field oracles

// O3D-10: a closed curve in a frame translating at u (arbitrary placement):
// Phi_T = (I - T T^T) u to 1e-9, ||Phi|| = v_r.
TEST(D3Frame, TranslatingFrameFeedforwardIsTheNormalPartOfU)
{
  const auto circle = d3::makeCurve3(builtin("circle", {"radius"}, {1.5}), {});
  const Isometry3 T = pose(0.4, 0.3, -0.2, Vec3(0.5, -1.0, 0.3));
  const Vec3 v_F(0.1, 0.05, -0.08);
  const d3::CurveFrame3 frame = moving(v_F, Vec3::Zero(), T);
  const Vec3 u = T.linear() * v_F;
  const d3::Tracker3 tracker{d3::Tracker3Config{}};
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int checked = 0;
  for (int i = 0; i < 500; ++i) {
    const double s = 4.0 * vf::kPi * (0.5 + 0.5 * unit(rng));
    const Vec3 offset = 0.5 * Vec3(unit(rng), unit(rng), unit(rng));
    d3::Tracker3Input input;
    input.curve = circle.get();
    input.frame = frame;
    input.pose.p = T * (circle->pointAt(s, 0.0) + offset);
    const d3::Tracker3Output out = tracker.step(input, d3::Tracker3::initialState(), nullptr);
    ASSERT_TRUE(out.field.valid);
    if (out.field.D > 0.7) {
      continue;  // stay inside the reach (1.5 m)
    }
    const Vec3 tangent = out.field.T;
    const Vec3 expected = u - tangent.dot(u) * tangent;
    EXPECT_LT((out.field.Phi_T - expected).norm(), 1e-9);
    EXPECT_LT((out.field.v_c - u).norm(), 1e-12);
    EXPECT_NEAR(out.field.Phi.norm(), 0.5, 1e-12);
    EXPECT_EQ(out.status & vf::Status::kAssumption3Clamped, 0U);
    // The foot in F is the curve-frame closest point of the mapped body.
    EXPECT_LT((T * out.x_star_curve - out.field.x_star).norm(), 1e-12);
    ++checked;
  }
  EXPECT_GT(checked, 200);
}

// O3D-11: v_c = R (v_F + w_F x r_F + dr_F/dt) equals the central difference (1e-5 s) of the world
// foot under the exact frame motion, for a rotating, translating frame and a time-varying curve.
TEST(D3Frame, CurveVelocityMatchesTheMovingFrameDifference)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kExpression;
  spec.x_expression = "2*cos(s) + 0.05*t";
  spec.y_expression = "sin(s)";
  spec.z_expression = "0.3*sin(2*s) + 0.02*t";
  spec.s_max = 2.0 * vf::kPi;
  spec.closed = true;
  const auto curve = d3::makeCurve3(spec, {});
  const Vec3 dr_dt(0.05, 0.0, 0.02);
  const d3::CurveFrame3 frame =
    moving(Vec3(0.1, 0.0, 0.05), Vec3(0.1, -0.2, 0.3), pose(1.0, 0.2, 0.1, Vec3(1.0, 2.0, 0.5)));
  const d3::Tracker3 tracker{d3::Tracker3Config{}};
  const double t = 3.0;
  const double delta = 1e-5;
  for (double s : {0.3, 1.7, 2.9, 4.4, 5.8}) {
    d3::Tracker3Input input;
    input.curve = curve.get();
    input.frame = frame;
    input.t = t;
    input.pose.p = frame.T_eval_curve * (curve->pointAt(s, t) + Vec3(0.05, -0.1, 0.08));
    const d3::Tracker3Output out = tracker.step(input, d3::Tracker3::initialState(), nullptr);
    ASSERT_TRUE(out.field.valid);
    const Vec3 r_F = out.x_star_curve;
    const Vec3 ahead = d3::extrapolate(frame, delta) * (r_F + delta * dr_dt);
    const Vec3 behind = d3::extrapolate(frame, -delta) * (r_F - delta * dr_dt);
    const Vec3 difference = (ahead - behind) / (2.0 * delta);
    EXPECT_LT((out.field.v_c - difference).norm(), 1e-6) << "s " << s;
  }
}

// ------------------------------------------------------------------------------------------------
// Closed loops (Tracker3, 0.5 m/s, 20 Hz ZOH)

// O3D-12: a circle (radius 2 m) in a frame translating at constant velocity: D < 0.01 m after the
// transient. The ZOH residual grows with the speed relative to the curve (math_spec §14.7).
TEST(D3Frame, ClosedLoopOnATranslatingCircle)
{
  const auto circle = d3::makeCurve3(builtin("circle", {"radius"}, {2.0}), {});
  const d3::CurveFrame3 frame =
    moving(Vec3(0.15, 0.05, 0.03), Vec3::Zero(), pose(0.3, 0.2, 0.0, Vec3(0.2, 0.0, 0.5)));
  const Vec3 start = frame.T_eval_curve * Vec3(2.5, 0.3, 0.3);
  const LoopRun run = simulate(*circle, start, 60.0, frame);
  ASSERT_GT(run.records.size(), 1000U);
  double worst = 0.0;
  for (const auto & r : run.records) {
    if (r.t >= 15.0) {
      worst = std::max(worst, r.D);
    }
  }
  EXPECT_LT(worst, 0.01) << "max D after the transient";
  EXPECT_EQ(statusUnion(run) & vf::Status::kAssumption3Clamped, 0U);
  // The frame recorded by the simulator moved with the twist.
  const auto & last = run.records.back();
  EXPECT_LT(
    (last.T_eval_curve.translation() - frame.T_eval_curve.translation() -
     last.t * (frame.T_eval_curve.linear() * frame.twist.v))
      .norm(),
    1e-12);
}

// O3D-13: a helix (open, offset 0.8 m from the rotation axis) in a frame rotating about a
// vertical and about a tilted axis: D < 0.02 m after the transient, no Assumption-3 flag, settled
// at the moving goal, and the goal hold rides with the frame.
void expectHelixInRotatingFrame(const Vec3 & w)
{
  const auto helix = d3::makeCurve3(builtin("helix", {"cx"}, {0.8}), {});
  const d3::CurveFrame3 frame = moving(Vec3::Zero(), w, pose(0.0, 0.0, 0.0, Vec3(0.0, 0.0, 0.2)));
  const Vec3 start = frame.T_eval_curve * Vec3(2.0, -0.2, -0.3);
  const LoopRun run = simulate(*helix, start, 120.0, frame, false);
  ASSERT_FALSE(run.records.empty());
  double worst = 0.0;
  double settled_at = vf::kNaN;
  for (const auto & r : run.records) {
    if (r.t >= 15.0 && std::isnan(settled_at)) {
      worst = std::max(worst, r.D);
    }
    if (r.settled && std::isnan(settled_at)) {
      settled_at = r.t;
    }
  }
  EXPECT_LT(worst, 0.02) << "max D after the transient";
  EXPECT_EQ(statusUnion(run) & vf::Status::kAssumption3Clamped, 0U);
  ASSERT_TRUE(std::isfinite(settled_at)) << "never settled";
  // Hold: every step after settling stays within sigma_stop of the moving goal.
  int held = 0;
  for (const auto & r : run.records) {
    if (r.t > settled_at + 1.0) {
      EXPECT_TRUE(r.settled) << "t " << r.t;
      ++held;
    }
  }
  EXPECT_GT(held, 100);
  EXPECT_TRUE(run.last.settled);
  EXPECT_LT((run.pose.p - run.last.goal).norm(), 0.05);
  EXPECT_GT(run.last.frame_velocity.norm(), 0.05);
}

TEST(D3Frame, ClosedLoopOnAHelixInAVerticallyRotatingFrame)
{
  expectHelixInRotatingFrame(Vec3(0.0, 0.0, 0.15));
}

TEST(D3Frame, ClosedLoopOnAHelixInATiltedRotatingFrame)
{
  expectHelixInRotatingFrame(0.15 * Vec3(1.0, 0.0, 2.0).normalized());
}

// A frame faster than (1 - eps_v) v_r at the curve raises the Assumption-3 flag.
TEST(D3Frame, FastFrameRaisesTheAssumption3Flag)
{
  const auto circle = d3::makeCurve3(builtin("circle", {"radius"}, {1.5}), {});
  const d3::CurveFrame3 frame = moving(Vec3(0.0, 0.6, 0.0), Vec3::Zero());
  const d3::Tracker3 tracker{d3::Tracker3Config{}};
  d3::Tracker3Input input;
  input.curve = circle.get();
  input.frame = frame;
  input.pose.p = Vec3(1.7, 0.0, 0.0);  // T = (0, 1, 0): u is tangential, Phi_T = 0
  EXPECT_EQ(
    tracker.step(input, d3::Tracker3::initialState(), nullptr).status &
      vf::Status::kAssumption3Clamped,
    0U);
  input.pose.p = Vec3(0.0, 1.7, 0.0);  // T = (-1, 0, 0): u is normal, ||Phi_T|| = 0.6 > 0.45
  const d3::Tracker3Output out = tracker.step(input, d3::Tracker3::initialState(), nullptr);
  EXPECT_NE(out.status & vf::Status::kAssumption3Clamped, 0U);
  EXPECT_NEAR(out.field.Phi_T.norm(), 0.45, 1e-12);
}

// ------------------------------------------------------------------------------------------------
// Static frames

// A static identity frame (default or explicit) and a moving identity frame with a zero twist
// give the frame-free results exactly; a rigidly displaced static frame gives the same D.
TEST(D3Frame, StaticFrameReproducesTheFrameFreeTracker)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kExpression;
  spec.x_expression = "2*cos(s) + 0.05*t";
  spec.y_expression = "sin(s)";
  spec.z_expression = "0.3*sin(2*s) + 0.02*t";
  spec.s_max = 2.0 * vf::kPi;
  spec.closed = true;
  const auto expression = d3::makeCurve3(spec, {});
  const auto viviani = d3::makeCurve3(builtin("viviani"), {});
  const auto helix = d3::makeCurve3(builtin("helix"), {});
  struct Case
  {
    const d3::Curve3 * curve;
    Vec3 start;
  };
  for (const Case & c :
       {Case{expression.get(), Vec3(2.4, 0.0, 0.2)}, Case{viviani.get(), Vec3(2.5, 0.5, 0.6)},
        Case{helix.get(), Vec3(1.6, -0.4, -0.5)}}) {
    const LoopRun reference = simulate(*c.curve, c.start, 30.0, d3::CurveFrame3{});
    d3::CurveFrame3 zero_twist;
    zero_twist.moving = true;
    const LoopRun mapped = simulate(*c.curve, c.start, 30.0, zero_twist);
    const Isometry3 T = pose(0.7, -0.3, 0.2, Vec3(3.0, -1.0, 2.0));
    d3::CurveFrame3 displaced;
    displaced.T_eval_curve = T;
    const LoopRun moved = simulate(*c.curve, T * c.start, 30.0, displaced);
    ASSERT_EQ(reference.records.size(), mapped.records.size());
    ASSERT_GT(reference.records.size(), 100U);
    for (std::size_t k = 0; k < reference.records.size(); ++k) {
      const auto & a = reference.records[k];
      const auto & b = mapped.records[k];
      // The goal hold of a moving frame brakes differently; compare up to the first settle.
      if (a.settled) {
        break;
      }
      ASSERT_EQ(a.pose.p, b.pose.p) << "k " << k;
      ASSERT_EQ(a.cmd.v, b.cmd.v) << "k " << k;
      ASSERT_EQ(a.cmd.wz, b.cmd.wz) << "k " << k;
      ASSERT_EQ(a.D, b.D) << "k " << k;
      ASSERT_EQ(a.F, b.F) << "k " << k;
      ASSERT_EQ(a.status, b.status) << "k " << k;
    }
    // The first step of a rigidly displaced static frame: the same D and the rotated field (the
    // per-axis limits make later steps frame-dependent).
    ASSERT_FALSE(moved.records.empty());
    EXPECT_NEAR(reference.records[0].D, moved.records[0].D, 1e-12);
    EXPECT_LT((T.linear() * reference.records[0].F - moved.records[0].F).norm(), 1e-11);
  }
}

// ------------------------------------------------------------------------------------------------
// FrameTwistEstimator3 (O3D-14)

TEST(D3Frame, EstimatorRecoversAConstantTwistAcrossPi)
{
  d3::FrameTwist3 xi;
  xi.v = Vec3(0.3, -0.1, 0.2);
  xi.w = Vec3(0.2, -0.1, 1.2);
  // Start 0.05 rad short of a half turn: the absolute rotation angle crosses pi.
  const Isometry3 T0 = pose(vf::kPi - 0.05, 0.1, -0.2, Vec3(1.0, -2.0, 0.5));
  d3::FrameTwistEstimator3 estimator;
  ASSERT_FALSE(estimator.estimate(0.0).ready);
  int accepted = 0;
  for (int k = 0; k <= 100; ++k) {
    const double t = 0.05 * k;
    const Isometry3 T = T0 * d3::expSE3(xi, t);
    const d3::FrameSampleResult3 result = estimator.addPose(t, T);
    if (k == 0) {
      EXPECT_EQ(result, d3::FrameSampleResult3::kStarted);
      continue;
    }
    ASSERT_EQ(result, d3::FrameSampleResult3::kAccepted) << "k " << k;
    ++accepted;
    const d3::FrameEstimate3 estimate = estimator.estimate(t + 0.02);
    ASSERT_TRUE(estimate.ready);
    ASSERT_FALSE(estimate.stale);
    EXPECT_LT((estimate.twist.v - xi.v).norm(), 1e-6) << "k " << k;
    EXPECT_LT((estimate.twist.w - xi.w).norm(), 1e-6) << "k " << k;
    EXPECT_LT(maxAbs(estimate.pose, T0 * d3::expSE3(xi, t + 0.02)), 1e-9);
    const d3::CurveFrame3 frame = estimate.curveFrame();
    EXPECT_TRUE(frame.moving);
  }
  EXPECT_EQ(accepted, 100);
  EXPECT_EQ(estimator.outliers(), 0U);
  // The heading did cross +-pi: atan2 of the rotated x axis jumps from about +pi to -pi.
  const auto heading = [&](double t) {
    const Vec3 x = (T0 * d3::expSE3(xi, t)).linear().col(0);
    return std::atan2(x.y(), x.x());
  };
  EXPECT_GT(heading(0.0), 3.0);
  EXPECT_LT(heading(0.1), -3.0);
}

TEST(D3Frame, EstimatorRejectsOutliersAndRestartsAfterGaps)
{
  d3::FrameTwist3 xi;
  xi.v = Vec3(0.2, 0.0, 0.0);
  xi.w = Vec3(0.0, 0.0, 0.3);
  d3::FrameTwistEstimator3 estimator;
  const auto at = [&](double t) { return d3::expSE3(xi, t); };
  EXPECT_EQ(estimator.addPose(0.0, at(0.0)), d3::FrameSampleResult3::kStarted);
  EXPECT_EQ(estimator.addPose(0.05, at(0.05)), d3::FrameSampleResult3::kAccepted);
  EXPECT_EQ(estimator.addPose(0.05, at(0.05)), d3::FrameSampleResult3::kDuplicate);
  EXPECT_EQ(estimator.addPose(0.04, at(0.04)), d3::FrameSampleResult3::kInvalid);
  // A 1 m jump in 50 ms (20 m/s) is an outlier; the base is kept.
  Isometry3 jump = at(0.1);
  jump.translation() += Vec3(1.0, 0.0, 0.0);
  EXPECT_EQ(estimator.addPose(0.1, jump), d3::FrameSampleResult3::kOutlier);
  EXPECT_EQ(estimator.outliers(), 1U);
  EXPECT_EQ(estimator.addPose(0.15, at(0.15)), d3::FrameSampleResult3::kAccepted);
  EXPECT_LT((estimator.estimate(0.15).twist.w - xi.w).norm(), 1e-9);
  EXPECT_EQ(estimator.addPose(0.155, at(0.155)), d3::FrameSampleResult3::kTooClose);
  // A non-rigid pose is invalid.
  Isometry3 sheared = at(0.2);
  sheared.linear()(0, 1) += 0.1;
  EXPECT_EQ(estimator.addPose(0.2, sheared), d3::FrameSampleResult3::kInvalid);
  // Stale beyond input_timeout: pose held, twist zero, not moving.
  const d3::FrameEstimate3 stale = estimator.estimate(1.0);
  EXPECT_TRUE(stale.stale);
  EXPECT_EQ(stale.twist.w, Vec3::Zero());
  EXPECT_FALSE(stale.curveFrame().moving);
  // A gap restarts.
  EXPECT_EQ(estimator.addPose(1.0, at(1.0)), d3::FrameSampleResult3::kStarted);
  EXPECT_FALSE(estimator.estimate(1.0).ready);
  // Twist source.
  EXPECT_EQ(estimator.addTwist(1.05, at(1.05), xi), d3::FrameSampleResult3::kAccepted);
  d3::FrameTwist3 fast = xi;
  fast.w.z() = 10.0;
  EXPECT_EQ(estimator.addTwist(1.1, at(1.1), fast), d3::FrameSampleResult3::kOutlier);
  // The low-pass filter converges to a new constant twist.
  estimator.reset();
  d3::FrameTwist3 other = xi;
  other.v = Vec3(0.0, 0.4, 0.1);
  for (int k = 0; k <= 40; ++k) {
    estimator.addPose(
      0.05 * k, k < 20 ? at(0.05 * k) : at(0.95) * d3::expSE3(other, 0.05 * (k - 19)));
  }
  EXPECT_LT((estimator.estimate(2.0).twist.v - other.v).norm(), 0.05);
}

TEST(D3Frame, SimulatorFollowsAFrameCallbackAndRejectsBadFrames)
{
  const auto circle = d3::makeCurve3(builtin("circle", {"radius"}, {1.5}), {});
  d3::sim::SimulationOptions3 options;
  options.frame_motion = [](double t) {
    d3::CurveFrame3 frame;
    frame.T_eval_curve.translation() = Vec3(0.1 * t, 0.0, 0.0);
    frame.twist.v = Vec3(0.1, 0.0, 0.0);
    frame.moving = true;
    return frame;
  };
  d3::sim::ClosedLoopSimulator3 simulator(d3::Tracker3(d3::Tracker3Config{}), options);
  simulator.reset(d3::Pose3{Vec3(1.7, 0.0, 0.0), 0.0});
  d3::Tracker3Input input;
  input.curve = circle.get();
  simulator.run(input, 1.0);
  EXPECT_NEAR(simulator.records().back().T_eval_curve.translation().x(), 0.095, 1e-12);
  EXPECT_NEAR(simulator.frameAt(2.0).T_eval_curve.translation().x(), 0.2, 1e-12);

  d3::sim::SimulationOptions3 bad;
  bad.frame.T_eval_curve.linear()(0, 0) = 2.0;
  EXPECT_THROW(
    d3::sim::ClosedLoopSimulator3(d3::Tracker3(d3::Tracker3Config{}), bad), vf::ConfigError);
  // A non-rigid frame makes the tracker output zero with kNonFinite.
  d3::Tracker3Input sheared;
  sheared.curve = circle.get();
  sheared.pose.p = Vec3(1.7, 0.0, 0.0);
  sheared.frame.T_eval_curve.linear()(0, 1) = 0.5;
  const d3::Tracker3Output out =
    d3::Tracker3(d3::Tracker3Config{}).step(sheared, d3::Tracker3::initialState(), nullptr);
  EXPECT_FALSE(out.field.valid);
  EXPECT_NE(out.status & vf::Status::kNonFinite, 0U);
}

}  // namespace
