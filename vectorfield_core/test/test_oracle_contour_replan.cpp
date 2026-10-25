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

// Contour gate hysteresis (math_spec §6.4 "Gate"): a replan during an episode that grazes the
// obstacle it is contouring, as Nav2 replans from the displaced pose when its global costmap does
// not see the obstacle.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

// The footprint's half width (the 16-gon has a vertex on body ±y).
constexpr double kWLat = kRCirc;

struct ReplanResult
{
  std::vector<Cycle> cycles;
  bool replanned{false};
  double clearance{0.0};  // least distance of the replanned path to the box sites
  int blocked_cycles{0};
  double final_x{0.0};
};

// A 1.8 m long, 0.6 m wide box across the path y = 0 → (6.5, 0), x in [2.7, 4.5]; FL from
// (0, 0.02). Once the episode has moved the robot beside the box (base x >= 2.2), the curve is
// replaced by a plan from the robot's position that runs along the box `grazing` metres closer
// than w_lat (inside the gate tolerance, so a closed gate would not open for it), then rejoins
// y = 0 past the box. The box is long enough for the robot to converge onto that plan.
ReplanResult runReplan(double grazing, double timeout = 30.0)
{
  ObstacleWorld world(Vec2(-1.5, -2.25), 9.0, 4.5);
  world.addBox(Vec2(2.7, -0.3), Vec2(4.5, 0.3));
  world.commit();
  const Vec2 goal(6.5, 0.0);
  const auto curve = makePolyline({Vec2(-1.0, 0.0), goal}, false);
  std::shared_ptr<const vf::Curve> replan;
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.02), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(goal);
  runner.input().obstacles = world.map();
  ReplanResult r;
  const std::size_t steps = static_cast<std::size_t>(std::lround(timeout / kPeriod));
  for (std::size_t k = 0; k < steps; ++k) {
    const Pose2 pose = runner.pose();
    if (
      !r.replanned && !runner.cycles().empty() && runner.cycles().back().after.contour.active &&
      pose.p.x() >= 2.2) {
      const double side = pose.p.y() >= 0.0 ? 1.0 : -1.0;
      const double y = side * (0.3 + kWLat - grazing);
      replan = makePolyline({pose.p, Vec2(2.5, y), Vec2(5.0, y), goal}, false);
      runner.input().curve = replan.get();
      r.replanned = true;
      r.clearance = std::abs(y) - 0.3;
    }
    const Cycle & c = runner.step();
    r.blocked_cycles += Status::has(c.out.status, Status::kBlocked) ? 1 : 0;
    if (c.pose_after.p.x() > 6.0) {
      break;
    }
  }
  r.cycles = runner.cycles();
  r.final_x = runner.pose().p.x();
  return r;
}

}  // namespace

// The replanned path passes the box 0.03 m inside w_lat: the gate, open for the episode, stays
// open (the path is not yet clear by obstacle_gate_release_margin), so the episode carries the
// robot past the box instead of ending and leaving it blocked against the box.
TEST(OracleContour, GateStaysOpenForAGrazingReplan)
{
  const ReplanResult r = runReplan(0.03);
  ASSERT_TRUE(r.replanned) << "no episode beside the box";
  EXPECT_NEAR(r.clearance, kWLat - 0.03, 1e-9);
  EXPECT_EQ(r.blocked_cycles, 0);
  EXPECT_GT(r.final_x, 6.0) << "did not get past the box";
}

// A replanned path that clears the box by more than the release margin lets the gate close as
// before: the episode ends and the robot follows the new path.
TEST(OracleContour, GateClosesForAClearReplan)
{
  const ReplanResult r = runReplan(-0.15);
  ASSERT_TRUE(r.replanned);
  EXPECT_EQ(r.blocked_cycles, 0);
  EXPECT_GT(r.final_x, 6.0);
  const std::vector<Episode> eps = episodes(r.cycles);
  ASSERT_FALSE(eps.empty());
  EXPECT_LT(eps.back().end, r.cycles.size()) << "the episode never ended";
}
