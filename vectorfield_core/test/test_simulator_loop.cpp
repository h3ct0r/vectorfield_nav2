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

// Unit tests of the closed-loop simulator (simulator.hpp).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/simulator.hpp"

namespace vectorfield_core
{
namespace
{

class StraightCurve
{
public:
  explicit StraightCurve(double length)
  {
    const std::vector<Vec2> points = {Vec2(0.0, 0.0), Vec2(length, 0.0)};
    EXPECT_EQ(curve_.assign(Span<const Vec2>(points), false), AssignStatus::kOk);
  }
  const Curve * get() const { return &curve_; }

private:
  PolylineCurve curve_;
};

TrackerInput inputFor(const Curve * curve, const Vec2 & goal_position)
{
  TrackerInput input;
  input.curve = curve;
  GoalInfo goal;
  goal.pose = Pose2{goal_position, 0.0};
  goal.xy_tol = 0.25;
  input.goal = goal;
  return input;
}

TEST(BoundedRandomDisturbance, IsBoundedReproducibleAndCopyable)
{
  sim::Disturbance a = sim::boundedRandomDisturbance(0.05, 42);
  sim::Disturbance b = sim::boundedRandomDisturbance(0.05, 42);
  sim::Disturbance c = sim::boundedRandomDisturbance(0.05, 43);
  bool differs = false;
  double largest = 0.0;
  for (int i = 0; i < 1000; ++i) {
    const Vec2 da = a(0.05 * i, Pose2{});
    const Vec2 db = b(0.05 * i, Pose2{});
    const Vec2 dc = c(0.05 * i, Pose2{});
    EXPECT_EQ(da, db);
    differs = differs || da != dc;
    largest = std::max(largest, da.norm());
  }
  EXPECT_TRUE(differs);
  EXPECT_LE(largest, 0.05 + 1e-15);
  EXPECT_GT(largest, 0.045);  // the disc is filled
  sim::Disturbance copy = a;  // a copy replays the same sequence from here
  EXPECT_EQ(copy(0.0, Pose2{}), a(0.0, Pose2{}));
  EXPECT_EQ(sim::boundedRandomDisturbance(0.0, 1)(0.0, Pose2{}), Vec2::Zero());
}

TEST(ClosedLoopSimulator, IsDeterministicAndIntegratesTheHeldCommand)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve(3.0);
  const TrackerInput input = inputFor(curve.get(), Vec2(3.0, 0.0));
  sim::SimulationOptions options;
  options.stop_when_settled = false;
  sim::ClosedLoopSimulator a(tracker, options);
  sim::ClosedLoopSimulator b(tracker, options);
  a.reset(Pose2{Vec2(0.0, 0.4), 0.5});
  b.reset(Pose2{Vec2(0.0, 0.4), 0.5});
  EXPECT_EQ(a.run(input, 4.0), 80U);
  EXPECT_EQ(b.run(input, 4.0), 80U);
  EXPECT_NEAR(a.time(), 4.0, 1e-9);
  ASSERT_EQ(a.records().size(), 80U);
  for (std::size_t k = 0; k < a.records().size(); ++k) {
    const sim::SimulationRecord & ra = a.records()[k];
    const sim::SimulationRecord & rb = b.records()[k];
    EXPECT_EQ(ra.pose.p, rb.pose.p);
    EXPECT_EQ(ra.cmd.vx, rb.cmd.vx);
    EXPECT_EQ(ra.cmd.wz, rb.cmd.wz);
    EXPECT_NEAR(ra.t, 0.05 * static_cast<double>(k), 1e-9);
    if (k > 0) {
      // Ideal odometry: the measured twist is the command held over the previous period.
      const sim::SimulationRecord & previous = a.records()[k - 1];
      EXPECT_EQ(ra.measured.vx, previous.cmd.vx);
      const Pose2 expected =
        sim::integrate(tracker.kinematics(), previous.pose, previous.cmd, 0.05);
      EXPECT_NEAR((ra.pose.p - expected.p).norm(), 0.0, 1e-12);
      EXPECT_NEAR(wrapAngle(ra.pose.yaw - expected.yaw), 0.0, 1e-12);
    }
  }
  // The robot converged toward the line.
  EXPECT_LT(std::abs(a.pose().p.y()), 0.4);
}

TEST(ClosedLoopSimulator, StopsWhenSettledAndResets)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve(1.0);
  const TrackerInput input = inputFor(curve.get(), Vec2(1.0, 0.0));
  sim::ClosedLoopSimulator simulator(tracker);
  simulator.reset(Pose2{Vec2(0.0, 0.1), 0.0});
  const std::size_t periods = simulator.run(input, 30.0);
  EXPECT_LT(periods, 600U);
  ASSERT_FALSE(simulator.records().empty());
  EXPECT_TRUE(simulator.records().back().settled);
  EXPECT_LE((simulator.records().back().pose.p - Vec2(1.0, 0.0)).norm(), 0.25);
  EXPECT_TRUE(simulator.lastOutput().settled);

  simulator.reset(Pose2{}, 2.0);
  EXPECT_TRUE(simulator.records().empty());
  EXPECT_EQ(simulator.time(), 2.0);
  EXPECT_EQ(simulator.state().phase, Phase::kFollow);
  EXPECT_FALSE(simulator.state().hint.valid);
}

TEST(ClosedLoopSimulator, AppliesTheDisturbanceAndTheUpdateHook)
{
  const Tracker tracker{CoreConfig{}};
  const StraightCurve curve(3.0);
  TrackerInput input = inputFor(curve.get(), Vec2(3.0, 0.0));
  input.cancel = true;  // STOPPED from rest: the command stays zero
  sim::SimulationOptions options;
  options.stop_when_stopped = false;
  options.disturbance = [](double, const Pose2 &) -> Vec2 { return Vec2(0.1, -0.2); };
  sim::ClosedLoopSimulator simulator(tracker, options);
  simulator.reset(Pose2{Vec2(0.5, 0.5), 0.0});
  EXPECT_EQ(simulator.run(input, 1.0), 20U);
  EXPECT_NEAR((simulator.pose().p - Vec2(0.6, 0.3)).norm(), 0.0, 1e-12);

  // The hook's edits persist: a new goal at t >= 0.5 s releases STOPPED.
  simulator.reset(Pose2{Vec2(0.5, 0.5), 0.0});
  simulator.run(input, 1.0, [](double t, TrackerInput * edited) {
    if (t >= 0.5 - 1e-9) {
      edited->cancel = false;
      edited->goal_seq = 1;
    }
  });
  for (const sim::SimulationRecord & record : simulator.records()) {
    EXPECT_EQ(record.phase == Phase::kStopped, record.t < 0.5 - 1e-9) << "t " << record.t;
  }
}

}  // namespace
}  // namespace vectorfield_core
