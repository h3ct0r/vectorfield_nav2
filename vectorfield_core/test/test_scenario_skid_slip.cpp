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

// Skid slip scenario of plan §3.5 (plan §8: vectorfield_sim's skid SlipParams η_ψ = 0.75,
// x_ICR = 0.05; math_spec §4.4 ultimate bound). Skid steer with FL; the plant applies the slip,
// the Tracker sees ideal odometry. The spec states no tolerance; the bounds below come from an
// independent ideal-FL reference with the TC limits and the same slip (straight: D → 2e-6 m;
// circle R = 1.5: steady D at the control point ≈ 0.058 m) with about 1.7× margin.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

vf::sim::SimulationOptions slipOptions()
{
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  options.slip.eta_psi = 0.75;
  options.slip.x_icr = 0.05;
  return options;
}

}  // namespace

// Straight path, start 0.5 m off it: under slip the base converges (|y| < 0.01 m from t = 20 s
// to the end of a 30 s run).
TEST(ScenarioSkidSlip, ConvergesOnStraightPath)
{
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(40.0, 0.0)}, false);
  Runner runner(tcConfig(vf::MotionModel::kSkidSteer, vf::TrackingLaw::kAuto), slipOptions());
  runner.reset(Pose2{Vec2(0.0, 0.5), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(40.0, 0.0));
  runner.run(30.0);
  for (const Cycle & c : runner.cycles()) {
    EXPECT_NE(c.out.phase, Phase::kStopped);
    if (c.rec.t >= 20.0) {
      EXPECT_LT(std::abs(c.rec.pose.p.y()), 0.01) << "t = " << c.rec.t;
    }
  }
}

// Circle of radius 1.5 m, start 0.5 m outside: the steady distance at the law's point stays below
// 0.1 m from t = 20 s on (reference 0.058 m), and the robot keeps circulating (at least half a lap
// of progress in 40 s).
TEST(ScenarioSkidSlip, BoundedOffsetOnCircle)
{
  const auto curve = makeBuiltin("ellipse", {{"a", 1.5}, {"b", 1.5}});
  Runner runner(tcConfig(vf::MotionModel::kSkidSteer, vf::TrackingLaw::kAuto), slipOptions());
  runner.reset(Pose2{Vec2(2.0, 0.0), vf::kHalfPi});
  runner.input().curve = curve.get();
  runner.run(40.0);
  for (const Cycle & c : runner.cycles()) {
    if (c.rec.t >= 20.0) {
      EXPECT_LT(c.rec.D, 0.1) << "t = " << c.rec.t;
    }
  }
  const double length = curve->length(0.0);
  EXPECT_GE(closedProgress(runner.cycles(), length), 0.5 * length);
}
