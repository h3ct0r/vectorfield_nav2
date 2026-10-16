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

// Closed-loop law oracles (math_spec §7.4–§7.5, §12.6): O40 heading-tracking feedforward on
// discretised circles (closed-loop part), O43 Ackermann from a large heading error.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

}  // namespace

// O40 HT feedforward, closed loop (math_spec §7.4 "Jump-only zeroing"): HT (diff) on closed
// polyline circles of R in {0.5, 1, 2} m with vertices every 2.5 and 5 cm (kept exactly), starting
// on the circle along the tangent: after a 5 s transient |D| < 0.01 m (the Tracker's D at base_link
// and the brute-force distance to the polyline), and ψ̇_d is zeroed on 0% of cycles
// (FieldSample::psi_d_dot != 0 from t = 1 s on). Duration 20 s per run.
TEST(OracleLaws, O40_HeadingTrackingCirclesClosedLoop)
{
  for (const double R : {0.5, 1.0, 2.0}) {
    for (const double spacing : {0.025, 0.05}) {
      SCOPED_TRACE("R = " + std::to_string(R) + ", spacing = " + std::to_string(spacing));
      const std::vector<Vec2> points = circlePoints(Vec2::Zero(), R, spacing);
      const auto curve = makePolylineExact(points, true);
      vf::sim::SimulationOptions options;
      options.stop_when_settled = false;
      Runner runner(
        tcConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking), options);
      runner.reset(Pose2{Vec2(R, 0.0), vf::kHalfPi});
      runner.input().curve = curve.get();
      runner.run(20.0);
      int zeroed = 0;
      int counted = 0;
      double worst = 0.0;
      for (const Cycle & c : runner.cycles()) {
        EXPECT_EQ(c.out.phase, Phase::kFollow) << "t = " << c.rec.t;
        if (c.rec.t >= 1.0) {
          ++counted;
          zeroed += c.out.field.psi_d_dot == 0.0 ? 1 : 0;
        }
        if (c.rec.t >= 5.0) {
          worst = std::max(worst, std::max(c.rec.D, polylineDistance(points, true, c.rec.pose.p)));
        }
      }
      EXPECT_GT(counted, 300);
      EXPECT_EQ(zeroed, 0) << "ψ̇_d zeroed on " << zeroed << " of " << counted << " cycles";
      EXPECT_LT(worst, 0.01);
    }
  }
}

// O43 Ackermann (math_spec §7.5 "Speed band"): R_min = 0.5, start on a straight path at a heading
// error of 85°: the command never stalls at zero speed (v > 0 on every cycle), no
// kHeadingInfeasible or STOPPED, and it converges (|y| < 0.05 m and |ψ| < 0.1 rad over the last
// 5 s of a 45 s run).
TEST(OracleLaws, O43_AckermannConvergesWithoutStall)
{
  vf::CoreConfig config = tcConfig(vf::MotionModel::kAckermann, vf::TrackingLaw::kAuto);
  config.motion.min_turning_radius = 0.5;
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(40.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(config, options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 85.0 * vf::kPi / 180.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(40.0, 0.0));
  runner.run(45.0);
  for (const Cycle & c : runner.cycles()) {
    EXPECT_GT(c.rec.cmd.vx, 0.0) << "zero-speed stall at t = " << c.rec.t;
    EXPECT_FALSE(Status::has(c.out.status, Status::kHeadingInfeasible)) << "t = " << c.rec.t;
    EXPECT_NE(c.out.phase, Phase::kStopped) << "t = " << c.rec.t;
    if (c.rec.t >= 40.0) {
      EXPECT_LT(std::abs(c.rec.pose.p.y()), 0.05) << "t = " << c.rec.t;
      EXPECT_LT(std::abs(vf::wrapAngle(c.rec.pose.yaw)), 0.1) << "t = " << c.rec.t;
    }
  }
}
