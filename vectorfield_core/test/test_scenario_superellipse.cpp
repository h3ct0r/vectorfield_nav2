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

// Superellipse runs (math_spec §2.7 polar superellipse): built-in superellipse with
// a = 2, b = 1.5 and n in {2, 4, 8} (perimeters 11.1, 12.3, 13.1 m; peak curvature 0.89, 1.51,
// 3.18 1/m), TC with FL, started on the curve along its tangent: one lap within 1.5× its time at
// v_r, never STOPPED or non-finite, and after a 5 s transient the distance at the law's point
// stays below 0.02 m (an ideal-FL reference with the TC limits stays below 3.3 mm).

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

}  // namespace

TEST(ScenarioSuperellipse, RunsForN2N4N8)
{
  for (const double n : {2.0, 4.0, 8.0}) {
    SCOPED_TRACE("n = " + std::to_string(n));
    const auto curve = makeBuiltin("superellipse", {{"a", 2.0}, {"b", 1.5}, {"n", n}});
    const double length = curve->length(0.0);
    vf::sim::SimulationOptions options;
    options.stop_when_settled = false;
    Runner runner(tcConfig(), options);
    runner.reset(Pose2{Vec2(2.0, 0.0), vf::kHalfPi});
    runner.input().curve = curve.get();
    runner.run(1.5 * length / kVr, [&runner, length](const Cycle &) {
      return closedProgress(runner.cycles(), length) >= length;
    });
    const auto & cs = runner.cycles();
    EXPECT_GE(closedProgress(cs, length), length) << "lap not completed";
    double worst = 0.0;
    for (const Cycle & c : cs) {
      EXPECT_FALSE(Status::has(c.out.status, Status::kNonFinite)) << "t = " << c.rec.t;
      EXPECT_NE(c.out.phase, Phase::kStopped) << "t = " << c.rec.t;
      if (c.rec.t >= 5.0) {
        worst = std::max(worst, c.rec.D);
      }
    }
    EXPECT_LT(worst, 0.02);
  }
}
