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

// Closed-loop upstream regressions about evaluation and execution (docs/design/upstream_audit.md
// §3; plan §3.5 naming): U19 evaluation has no side effects, U25 commands respect the limits
// (closed-loop part), U30 concurrent const evaluation. Split from
// test_upstream_regressions_closed_loop.cpp to keep each executable under 20 s in Debug.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "test_support.hpp"
#include "vectorfield_core/field.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

}  // namespace

// U19 (upstream: field evaluation mutated the tracking state): two identical closed loops (200 cycles, covering the contour episode; HT,
// contour on, a box on the path); between the steps of the second one, closestPoint at arbitrary
// points (chained real hints), probes on a 5 × 5 field grid at time offsets 0 and Δt/2, and a
// streamline through the probe. TrackerState and outputs stay bit-identical every cycle.
TEST(UpstreamRegression, U19_EvaluationHasNoSideEffects)
{
  ObstacleWorld world(Vec2(-2.0, -2.5), 9.0, 5.0);
  world.addBox(Vec2(2.7, -0.3), Vec2(3.3, 0.3));
  world.commit();
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(6.5, 0.0)}, false);
  vf::CoreConfig config =
    tcConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking);
  config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner a(config, options);
  Runner b(config, options);
  for (Runner * r : {&a, &b}) {
    r->reset(Pose2{Vec2(0.0, 0.02), 0.0});
    r->input().curve = curve.get();
    r->input().goal = goalAt(Vec2(6.5, 0.0));
    r->input().obstacles = world.map();
  }
  std::mt19937_64 rng(1919);
  std::uniform_real_distribution<double> ux(-1.0, 7.0);
  std::uniform_real_distribution<double> uy(-2.0, 2.0);
  vf::CurveHint chained;
  int active = 0;
  for (int k = 0; k < 200; ++k) {
    const Cycle & ca = a.step();
    const Cycle & cb = b.step();
    ASSERT_TRUE(bitEqual(ca.after, cb.after)) << "TrackerState differs at cycle " << k;
    ASSERT_TRUE(bitEqual(ca.out, cb.out)) << "TrackerOutput differs at cycle " << k;
    active += cb.after.contour.active ? 1 : 0;
    for (int i = 0; i < 10; ++i) {
      vf::EvalContext ctx;
      ctx.t = cb.input.t;
      const vf::ClosestPoint cp = curve->closestPoint(Vec2(ux(rng), uy(rng)), ctx, chained);
      chained = vf::makeHint(cp);
    }
    for (int ix = 0; ix < 5; ++ix) {
      for (int iy = 0; iy < 5; ++iy) {
        const Vec2 x = cb.rec.pose.p + Vec2(-0.75 + 0.375 * ix, -0.75 + 0.375 * iy);
        for (const double offset : {0.0, 0.5 * kPeriod}) {
          (void)b.tracker().probe(x, cb.input, b.sim().state(), b.sim().scratch(), offset);
        }
      }
    }
    const auto grid_field = [&](const Vec2 & x) {
      return b.tracker().probe(x, cb.input, b.sim().state(), b.sim().scratch()).F;
    };
    std::array<Vec2, 10> line{};
    (void)vf::streamline(
      grid_field, cb.rec.pose.p + Vec2(0.3, 0.3), 0.125, vf::Span<Vec2>(line.data(), line.size()));
    if (cb.out.settled) {
      break;
    }
  }
  EXPECT_GT(active, 0) << "the contour never engaged; the probes did not exercise it";
}

namespace
{

struct LimitModel
{
  vf::MotionModel model;
  vf::TrackingLaw law;
  vf::OmniHeadingPolicy policy;
};

}  // namespace

// U25 closed-loop part (upstream: no saturation or acceleration limits): 24 seeded closed loops
// over every (model, law, heading policy) pair, random ellipses, starts, speeds, speed limits
// (>= 0.3, so the Ackermann floor never binds), a box on the curve in half of the runs, contour in
// half of the non-Ackermann runs and a cancel after 3 s in a quarter of them: on every cycle the
// final command is finite, |v_x| <= v_max, |v_y| <= v_y,max and ‖v_xy‖ <= v_max (omni; v_y = 0
// otherwise), |ω| <= ω_max·γ_lim, each axis changes by at most max(a_acc, a_dec)·Δt (ω: α_max·Δt)
// from the previous command, and Ackermann keeps |ω| <= |v|/R_min (all + 1e-12). FL with d = 0 is
// a ConfigError. 6 s per run.
TEST(UpstreamRegression, U25_CommandsRespectLimits_ClosedLoop)
{
  vf::CoreConfig zero_d = tcConfig();
  zero_d.motion.control_point_offset = 0.0;
  EXPECT_THROW(vf::Tracker{zero_d}, vf::ConfigError);

  const LimitModel models[] = {
    {vf::MotionModel::kDifferential, vf::TrackingLaw::kFeedbackLinearization,
     vf::OmniHeadingPolicy::kPathTangent},
    {vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking,
     vf::OmniHeadingPolicy::kPathTangent},
    {vf::MotionModel::kSkidSteer, vf::TrackingLaw::kFeedbackLinearization,
     vf::OmniHeadingPolicy::kPathTangent},
    {vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic,
     vf::OmniHeadingPolicy::kPathTangent},
    {vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic, vf::OmniHeadingPolicy::kField},
    {vf::MotionModel::kAckermann, vf::TrackingLaw::kHeadingTracking,
     vf::OmniHeadingPolicy::kPathTangent},
  };
  std::mt19937_64 rng(2525);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (int run = 0; run < 24; ++run) {
    const LimitModel & m = models[run % 6];
    SCOPED_TRACE("run " + std::to_string(run));
    vf::CoreConfig config = tcConfig(m.model, m.law);
    config.motion.omni_heading_policy = m.policy;
    config.field.desired_speed = 0.2 + 0.3 * unit(rng);
    const bool ackermann = m.model == vf::MotionModel::kAckermann;
    if (ackermann) {
      config.motion.min_turning_radius = 0.5;
    }
    const bool contour = !ackermann && unit(rng) < 0.5;
    if (contour) {
      config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
    }
    const double a = 1.0 + unit(rng);
    const double b = 0.7 + 0.8 * unit(rng);
    const auto curve = makeBuiltin("ellipse", {{"a", a}, {"b", b}});
    const double phi0 = vf::kTwoPi * unit(rng);
    const Vec2 on_curve(a * std::cos(phi0), b * std::sin(phi0));
    const Vec2 start = on_curve + (unit(rng) * 1.3 - 0.5) * on_curve.normalized();
    ObstacleWorld world(Vec2(-4.0, -4.0), 8.0, 8.0);
    if (unit(rng) < 0.5) {
      const double phi_box = phi0 + vf::kHalfPi;
      const Vec2 c(a * std::cos(phi_box), b * std::sin(phi_box));
      world.addBox(c - Vec2(0.15, 0.15), c + Vec2(0.15, 0.15));
    }
    world.commit();
    const bool cancel = unit(rng) < 0.25;
    const double scale = 0.3 + 0.7 * unit(rng);
    vf::sim::SimulationOptions options;
    options.stop_when_settled = false;
    options.stop_when_stopped = false;
    Runner runner(config, options);
    runner.reset(Pose2{start, vf::kTwoPi * unit(rng) - vf::kPi});
    runner.input().curve = curve.get();
    runner.input().obstacles = world.map();
    runner.input().speed_limit_scale = scale;
    runner.setPreStep(
      [cancel](Runner & r, vf::TrackerInput & in) { in.cancel = cancel && r.time() >= 3.0; });
    runner.run(6.0);
    const vf::Kinematics & k = runner.tracker().kinematics();
    const double w_bar = k.max_angular_vel * scale;
    const double dv = std::max(k.max_linear_accel, k.max_linear_decel) * kPeriod;
    const double dw = k.max_angular_accel * kPeriod;
    Twist2 prev;
    for (const Cycle & c : runner.cycles()) {
      const Twist2 & u = c.rec.cmd;
      ASSERT_TRUE(vf::isFinite(u)) << "t = " << c.rec.t;
      EXPECT_LE(std::abs(u.vx), k.max_linear_vel + 1e-12) << "t = " << c.rec.t;
      if (k.isHolonomic()) {
        EXPECT_LE(std::abs(u.vy), k.max_lateral_vel + 1e-12) << "t = " << c.rec.t;
        EXPECT_LE(std::hypot(u.vx, u.vy), k.max_linear_vel + 1e-12) << "t = " << c.rec.t;
      } else {
        EXPECT_EQ(u.vy, 0.0) << "t = " << c.rec.t;
      }
      EXPECT_LE(std::abs(u.wz), w_bar + 1e-12) << "t = " << c.rec.t;
      EXPECT_LE(std::abs(u.vx - prev.vx), dv + 1e-12) << "t = " << c.rec.t;
      EXPECT_LE(std::abs(u.vy - prev.vy), dv + 1e-12) << "t = " << c.rec.t;
      EXPECT_LE(std::abs(u.wz - prev.wz), dw + 1e-12) << "t = " << c.rec.t;
      if (ackermann) {
        EXPECT_LE(std::abs(u.wz), std::abs(u.vx) / k.min_turning_radius + 1e-12)
          << "t = " << c.rec.t;
      }
      prev = u;
    }
  }
}

namespace
{

struct Snap
{
  Twist2 cmd;
  std::uint32_t status{0};
  Phase phase{Phase::kFollow};
  Vec2 F{Vec2::Zero()};
  double D{0.0};
  bool active{false};
};

// A closed loop on a shared const Tracker with its own state and scratch (no simulator object).
std::vector<Snap> sharedTrackerLoop(
  const vf::Tracker & tracker, const vf::TrackerInput & tmpl, Pose2 pose, int steps)
{
  vf::StepScratch scratch;
  vf::TrackerState state = vf::Tracker::initialState();
  Twist2 measured;
  double t = 0.0;
  std::vector<Snap> out;
  out.reserve(static_cast<std::size_t>(steps));
  for (int k = 0; k < steps; ++k) {
    vf::TrackerInput input = tmpl;
    input.pose = pose;
    input.measured = measured;
    input.t = t;
    input.dt = kPeriod;
    const vf::TrackerOutput o = tracker.step(input, state, &state, scratch);
    out.push_back(Snap{o.cmd, o.status, o.phase, o.field.F, o.field.D, state.contour.active});
    pose = vf::sim::integrate(tracker.kinematics(), pose, o.cmd, kPeriod);
    measured = o.cmd;
    t += kPeriod;
  }
  return out;
}

bool sameSnaps(const std::vector<Snap> & a, const std::vector<Snap> & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t k = 0; k < a.size(); ++k) {
    if (
      !bitEqual(a[k].cmd, b[k].cmd) || a[k].status != b[k].status || a[k].phase != b[k].phase ||
      !bitEqual(a[k].F, b[k].F) || !bitEqual(a[k].D, b[k].D) || a[k].active != b[k].active) {
      return false;
    }
  }
  return true;
}

}  // namespace

// U30 (upstream: shared mutable state across callbacks): four closed loops on four threads share
// one const Tracker, the curves (a polyline and a built-in ellipse), the grid and its EDT, each
// with its own TrackerState and StepScratch: race-free (run it under TSan) and bit-identical to a
// serial run of the same loops. 200 cycles each.
TEST(UpstreamRegression, U30_ConcurrentConstEvaluation)
{
  ObstacleWorld world(Vec2(-2.0, -2.5), 9.0, 5.0);
  world.addBox(Vec2(2.7, -0.3), Vec2(3.3, 0.3));
  world.commit();
  const vf::Tracker tracker(tcContourConfig());
  const auto polyline = makePolyline({Vec2(-1.0, 0.0), Vec2(6.5, 0.0)}, false);
  const auto ellipse = makeBuiltin("ellipse", {{"a", 1.5}, {"b", 1.0}});
  vf::TrackerInput open_input;
  open_input.curve = polyline.get();
  open_input.goal = goalAt(Vec2(6.5, 0.0));
  open_input.obstacles = world.map();
  vf::TrackerInput closed_input;
  closed_input.curve = ellipse.get();
  closed_input.obstacles = world.map();
  struct Job
  {
    const vf::TrackerInput * input;
    Pose2 start;
  };
  const std::vector<Job> jobs = {
    {&open_input, Pose2{Vec2(0.0, 0.02), 0.0}},
    {&open_input, Pose2{Vec2(0.0, -0.03), 0.1}},
    {&closed_input, Pose2{Vec2(1.6, 0.1), vf::kHalfPi}},
    {&closed_input, Pose2{Vec2(-1.7, 0.0), -vf::kHalfPi}},
  };
  constexpr int kSteps = 200;
  std::vector<std::vector<Snap>> serial;
  for (const Job & job : jobs) {
    serial.push_back(sharedTrackerLoop(tracker, *job.input, job.start, kSteps));
  }
  std::vector<std::vector<Snap>> concurrent(jobs.size());
  std::vector<std::thread> threads;
  for (std::size_t j = 0; j < jobs.size(); ++j) {
    threads.emplace_back([&tracker, &jobs, &concurrent, j]() {
      concurrent[j] = sharedTrackerLoop(tracker, *jobs[j].input, jobs[j].start, kSteps);
    });
  }
  for (std::thread & thread : threads) {
    thread.join();
  }
  bool any_active = false;
  for (std::size_t j = 0; j < jobs.size(); ++j) {
    EXPECT_TRUE(sameSnaps(serial[j], concurrent[j]))
      << "job " << j << " differs from the serial run";
    for (const Snap & s : serial[j]) {
      any_active = any_active || s.active;
    }
  }
  EXPECT_TRUE(any_active) << "no contour episode in any job";
}
