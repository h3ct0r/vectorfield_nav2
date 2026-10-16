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

// Closed-loop limiter oracles (math_spec §7.7, §8.2–§8.3, §12.7): O45 η-box keeps Ḋ <= 0,
// O46 omni on a translating circle, O47 contour with a moving curve.
//
// The translating circle: a built-in circle of radius 1.5 m in a curve frame translating at
// u = (0.1, 0) m/s (FRAME_MOVING, twist in the frame), so the field is time-varying with
// ‖Φ_T‖ <= 0.1 m/s. Ḋ is the math_spec §12 quantity n^T(ẋ − v_c) from cmd_target.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

const Vec2 kU(0.1, 0.0);
constexpr double kR = 1.5;

vf::CurveFrame frameAt(double t)
{
  vf::CurveFrame frame;
  frame.T_eval_curve = vf::Isometry2(0.0, kU * t);
  frame.twist_in_curve = Twist2{kU.x(), kU.y(), 0.0};
  frame.moving = true;
  return frame;
}

std::vector<Cycle> runTranslatingCircle(
  const vf::CoreConfig & config, const Pose2 & start, double duration,
  const ObstacleWorld * world = nullptr)
{
  const auto curve = makeBuiltin("ellipse", {{"a", kR}, {"b", kR}});
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(config, options);
  runner.reset(start);
  runner.input().curve = curve.get();
  if (world != nullptr) {
    runner.input().obstacles = world->map();
  }
  runner.setPreStep([](Runner & r, vf::TrackerInput & in) { in.frame = frameAt(r.time()); });
  runner.run(duration);
  return runner.cycles();
}

}  // namespace

// O45 η-box keeps Ḋ <= 0 (math_spec §8.3, §7.7): FL on the translating circle with ω_max lowered
// to 0.6 rad/s, so ω binds during the initial turn while M Φ_T alone (|ω| <= 0.5 rad/s) never
// violates the box: on every FOLLOW cycle under the η-box (non-uniform shaping, backstop inactive)
// Ḋ <= 1e-12, and kAssumption3Clamped is never set. Duration 15 s.
TEST(OracleLimiter, O45_EtaBoxKeepsDistanceNonIncreasing)
{
  vf::CoreConfig config = tcConfig();
  config.motion.max_angular_vel = 0.6;
  const Pose2 start{Vec2(kR + 0.6, 0.0), vf::kHalfPi + 1.0};
  const std::vector<Cycle> cs = runTranslatingCircle(config, start, 15.0);
  int bound = 0;
  int checked = 0;
  for (const Cycle & c : cs) {
    EXPECT_FALSE(Status::has(c.out.status, Status::kAssumption3Clamped)) << "t = " << c.rec.t;
    if (
      c.out.phase != Phase::kFollow || c.out.field.uniform_shaping || c.out.cmd_target.vx <= 0.0) {
      continue;
    }
    ++checked;
    if (std::abs(c.out.cmd_target.wz) >= 0.6 - 1e-9) {
      ++bound;
    }
    EXPECT_LE(flDdot(c, kD), 1e-12) << "t = " << c.rec.t;
  }
  EXPECT_GT(checked, 100);
  EXPECT_GE(bound, 3) << "ω never bound; the oracle is vacuous";
}

// O46 Omni (math_spec §7.2, §8.3 "Holonomic ω"): holonomic law with `path_tangent` on the
// translating circle, 180° initial heading error: |ω| <= ω_max on every cycle (it saturates at the
// start), no kAssumption3Clamped while only ω saturates, and Ḋ <= 1e-12. Duration 10 s.
TEST(OracleLimiter, O46_OmniHeadingSaturationKeepsDistanceNonIncreasing)
{
  const vf::CoreConfig config =
    tcConfig(vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic);
  const Pose2 start{Vec2(kR + 0.3, 0.0), -vf::kHalfPi};
  const std::vector<Cycle> cs = runTranslatingCircle(config, start, 10.0);
  int saturated = 0;
  for (const Cycle & c : cs) {
    EXPECT_LE(std::abs(c.rec.cmd.wz), kWmax + 1e-12) << "t = " << c.rec.t;
    EXPECT_LE(std::abs(c.out.cmd_target.wz), kWmax + 1e-12) << "t = " << c.rec.t;
    EXPECT_FALSE(Status::has(c.out.status, Status::kAssumption3Clamped)) << "t = " << c.rec.t;
    EXPECT_LE(holonomicDdot(c), 1e-12) << "t = " << c.rec.t;
    if (std::abs(c.out.cmd_target.wz) >= kWmax - 1e-9) {
      ++saturated;
    }
  }
  EXPECT_GE(saturated, 3) << "ω never saturated; the oracle is vacuous";
}

// O47 Contour with a moving curve (math_spec §6.4 "Speed uses ‖Φ‖", §8.2–§8.3): the translating
// circle sweeps over a static 0.3 m box, FL and holonomic: during the episode the command stays in
// the kinematic box (final and target), ‖v_xy‖ <= γ v_r + 1e-12 (FL: control-point velocity
// M⁻¹u; holonomic: base velocity), and kAssumption3Clamped is never set. Duration 25 s.
TEST(OracleLimiter, O47_ContourWithMovingCurveStaysInBox)
{
  ObstacleWorld world(Vec2(-2.5, -3.0), 7.0, 6.0);
  world.addBox(Vec2(2.0, -0.15), Vec2(2.3, 0.15));
  world.commit();
  for (const bool holonomic : {false, true}) {
    SCOPED_TRACE(holonomic ? "holonomic" : "FL");
    vf::CoreConfig config =
      holonomic ? tcConfig(vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic)
                : tcConfig();
    config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
    const std::vector<Cycle> cs =
      runTranslatingCircle(config, Pose2{Vec2(0.0, -kR), 0.0}, 25.0, &world);
    int active = 0;
    for (const Cycle & c : cs) {
      if (!c.after.contour.active) {
        continue;
      }
      ++active;
      EXPECT_FALSE(Status::has(c.out.status, Status::kAssumption3Clamped)) << "t = " << c.rec.t;
      for (const Twist2 & u : {c.rec.cmd, c.out.cmd_target}) {
        EXPECT_LE(std::abs(u.vx), kVmax + 1e-12) << "t = " << c.rec.t;
        EXPECT_LE(std::abs(u.wz), kWmax + 1e-12) << "t = " << c.rec.t;
        if (holonomic) {
          EXPECT_LE(std::abs(u.vy), kVmax + 1e-12) << "t = " << c.rec.t;
          EXPECT_LE(std::hypot(u.vx, u.vy), kVmax + 1e-12) << "t = " << c.rec.t;
        } else {
          EXPECT_EQ(u.vy, 0.0);
        }
      }
      const Twist2 & u = c.out.cmd_target;
      const double speed = holonomic ? std::hypot(u.vx, u.vy)
                                     : (u.vx * vf::heading(c.input.pose.yaw) +
                                        kD * u.wz * vf::J(vf::heading(c.input.pose.yaw)))
                                         .norm();
      EXPECT_LE(speed, c.out.field.gamma * kVr + 1e-12) << "t = " << c.rec.t;
    }
    EXPECT_GT(active, 10) << "no contour episode on the moving curve";
  }
}
