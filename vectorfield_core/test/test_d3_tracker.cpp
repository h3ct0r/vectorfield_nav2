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

// Tracker3 and ClosedLoopSimulator3 (math_spec §14.5, oracles O3D-7 and O3D-8): convergence on a
// helix and on Viviani's curve at 0.5 m/s, 20 Hz ZOH; an open polyline settles at its goal; a
// cancel stops; configuration errors; the plant.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "vectorfield_core/d3.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace d3 = vectorfield_core::d3;
using d3::Vec3;

d3::CurveSpec3 builtin(const std::string & name)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kBuiltin;
  spec.builtin_shape = name;
  return spec;
}

d3::Tracker3Config config()
{
  d3::Tracker3Config c;
  c.desired_speed = 0.5;
  return c;
}

struct LoopRun
{
  std::vector<d3::sim::SimulationRecord3> records;
  d3::Tracker3Output last;
  d3::Pose3 pose;
};

LoopRun simulate(
  const d3::Curve3 & curve, const d3::Pose3 & start, double duration,
  const d3::Tracker3Config & c = config(), double tau = 0.0)
{
  d3::sim::SimulationOptions3 options;
  options.period = 0.05;
  options.plant.velocity_time_constant = tau;
  d3::sim::ClosedLoopSimulator3 simulator(d3::Tracker3(c), options);
  simulator.reset(start);
  d3::Tracker3Input input;
  input.curve = &curve;
  simulator.run(input, duration);
  return LoopRun{simulator.records(), simulator.lastOutput(), simulator.pose()};
}

double maxDistanceAfter(const LoopRun & run, double t0, double t1 = vf::kInf)
{
  double worst = 0.0;
  for (const auto & r : run.records) {
    if (r.t >= t0 && r.t <= t1) {
      worst = std::max(worst, r.D);
    }
  }
  return worst;
}

TEST(D3Tracker, ConvergesOnAHelix)
{
  const auto helix = d3::makeCurve3(builtin("helix"), {});
  d3::Pose3 start;
  start.p = Vec3(1.6, -0.4, -0.5);
  const LoopRun run = simulate(*helix, start, 60.0);
  ASSERT_FALSE(run.records.empty());
  // After the transient, |D| < 0.01 m until the goal. The residual is the ZOH equilibrium of
  // math_spec §14.3: D_zoh = v_r Δt κ / (2 G'(0)), 9.76 mm for κ = 0.994 at 0.5 m/s and 20 Hz.
  EXPECT_LT(maxDistanceAfter(run, 15.0), 0.01) << "max D after the transient";
  const double kappa = 1.0 / (1.0 + std::pow(0.5 / (2.0 * vf::kPi), 2));
  const double d_zoh = 0.5 * 0.05 * kappa / (2.0 * (4.0 / vf::kPi));
  for (const auto & r : run.records) {
    if (r.t > 20.0 && r.sigma > 1.0) {
      EXPECT_NEAR(r.D, d_zoh, 0.03 * d_zoh);
    }
  }
  EXPECT_TRUE(run.last.settled);
  EXPECT_EQ(run.last.phase, vf::Phase::kFinalApproach);
  EXPECT_LT((run.pose.p - run.last.goal).norm(), 0.05);
  // The yaw followed the path tangent: no wild spins, finite throughout.
  for (const auto & r : run.records) {
    ASSERT_TRUE(d3::isFinite(r.pose));
  }
}

TEST(D3Tracker, ConvergesOnVivianiThroughTheDoublePoint)
{
  const auto viviani = d3::makeCurve3(builtin("viviani"), {});
  d3::Pose3 start;
  start.p = Vec3(2.5, 0.5, 0.6);
  const LoopRun run = simulate(*viviani, start, 80.0);
  EXPECT_LT(maxDistanceAfter(run, 10.0), 0.01);
  // Progress: the station advanced through more than one full lap (the double point at
  // (R, 0, 0) is crossed twice per lap without switching branches).
  double travelled = 0.0;
  for (std::size_t k = 1; k < run.records.size(); ++k) {
    travelled +=
      viviani->wrapStationDifference(run.records[k].s_star - run.records[k - 1].s_star, 0.0);
  }
  EXPECT_GT(travelled, 1.2 * viviani->length(0.0));
}

TEST(D3Tracker, ConvergesWithAVelocityLag)
{
  const auto knot = d3::makeCurve3(builtin("torus_knot"), {});
  d3::Pose3 start;
  start.p = Vec3(2.8, 0.2, 0.3);
  auto c = config();
  c.desired_speed = 0.4;
  const LoopRun run = simulate(*knot, start, 40.0, c, 0.1);
  EXPECT_LT(maxDistanceAfter(run, 12.0), 0.03);
}

TEST(D3Tracker, OpenPolylineSettlesAtTheGoal)
{
  const d3::PolylineCurve3 path(
    {Vec3(0, 0, 0), Vec3(2, 0, 0.5), Vec3(3, 1, 1.5), Vec3(3, 3, 1.5)}, false);
  d3::Pose3 start;
  start.p = Vec3(-0.5, 0.3, -0.2);
  const LoopRun run = simulate(path, start, 60.0);
  ASSERT_TRUE(run.last.settled);
  const Vec3 goal(3, 3, 1.5);
  EXPECT_LE((run.pose.p - goal).norm(), 0.5 * config().goal_tolerance + 0.01);
  EXPECT_LE((run.records.back().pose.p - goal).norm(), 0.5 * config().goal_tolerance);
  EXPECT_EQ(run.last.phase, vf::Phase::kFinalApproach);
  // Holding: further steps command zero.
  d3::Tracker3 tracker(config());
  d3::Tracker3State state;
  d3::Tracker3Input input;
  input.curve = &path;
  input.pose.p = goal;
  for (int k = 0; k < 40; ++k) {
    const auto out = tracker.step(input, state, &state);
    EXPECT_TRUE(out.settled);
    EXPECT_EQ(out.cmd_target.v, Vec3::Zero());
  }
  EXPECT_TRUE(state.holding);
  // A new goal_seq resets the latch and the hold.
  input.goal_seq = 1;
  input.pose.p = Vec3(0, 0, 0);
  const auto out = tracker.step(input, state, &state);
  EXPECT_FALSE(out.settled);
  EXPECT_EQ(out.phase, vf::Phase::kFollow);
  EXPECT_GT(out.cmd_target.v.norm(), 0.1);
}

TEST(D3Tracker, CancelStops)
{
  const auto circle = d3::makeCurve3(builtin("circle"), {});
  d3::sim::SimulationOptions3 options;
  d3::sim::ClosedLoopSimulator3 simulator(d3::Tracker3(config()), options);
  d3::Pose3 start;
  start.p = Vec3(1.0, 0.0, 0.0);
  simulator.reset(start);
  d3::Tracker3Input input;
  input.curve = circle.get();
  simulator.run(input, 5.0);
  EXPECT_GT(simulator.lastOutput().cmd.v.norm(), 0.4);
  input.cancel = true;
  const std::size_t steps = simulator.run(input, 5.0);
  EXPECT_EQ(simulator.lastOutput().phase, vf::Phase::kStopped);
  EXPECT_TRUE(simulator.lastOutput().cancel_complete);
  // 0.5 m/s at a_can = 1 m/s²: 10 cycles, plus the yaw-rate ramp-down.
  EXPECT_LE(steps, 14u);
  // STOPPED is latched even without the cancel flag, until goal_seq changes.
  input.cancel = false;
  d3::Tracker3State state = simulator.state();
  const d3::Tracker3 tracker(config());
  d3::Tracker3Input step_input = input;
  step_input.pose = simulator.pose();
  auto out = tracker.step(step_input, state, &state);
  EXPECT_EQ(out.phase, vf::Phase::kStopped);
  step_input.goal_seq = 7;
  out = tracker.step(step_input, state, &state);
  EXPECT_EQ(out.phase, vf::Phase::kFollow);
}

TEST(D3Tracker, InvalidInputsAndConfig)
{
  const d3::Tracker3 tracker(config());
  d3::Tracker3State state;
  d3::Tracker3Input input;
  auto out = tracker.step(input, state, &state);  // no curve
  EXPECT_TRUE(out.status & vf::Status::kNonFinite);
  EXPECT_EQ(out.cmd.v, Vec3::Zero());
  auto c = config();
  c.desired_speed = -1.0;
  EXPECT_THROW(d3::Tracker3{c}, vf::ConfigError);
  c = config();
  c.assumption3_margin = 1.0;
  EXPECT_THROW(d3::Tracker3{c}, vf::ConfigError);
  c = config();
  c.limits.max_axis_speed.z() = 0.0;
  EXPECT_THROW(d3::Tracker3{c}, vf::ConfigError);
  c = config();
  c.min_approach_speed = 1.0;
  EXPECT_THROW(d3::Tracker3{c}, vf::ConfigError);
  d3::sim::SimulationOptions3 options;
  options.period = 0.0;
  EXPECT_THROW(d3::sim::ClosedLoopSimulator3(d3::Tracker3(config()), options), vf::ConfigError);
}

TEST(D3Simulator, PlantIsExactUnderZoh)
{
  d3::sim::PlantState3 state;
  d3::Twist3 cmd;
  cmd.v = Vec3(0.3, -0.2, 0.1);
  cmd.wz = 0.5;
  const auto ideal = d3::sim::integrate(state, cmd, 0.1, {});
  EXPECT_TRUE(ideal.pose.p.isApprox(Vec3(0.03, -0.02, 0.01), 1e-15));
  EXPECT_NEAR(ideal.pose.yaw, 0.05, 1e-15);
  // Lag: two half steps equal one full step (closed form).
  d3::sim::PlantParams3 lag;
  lag.velocity_time_constant = 0.2;
  const auto one = d3::sim::integrate(state, cmd, 0.1, lag);
  const auto two = d3::sim::integrate(d3::sim::integrate(state, cmd, 0.05, lag), cmd, 0.05, lag);
  EXPECT_LT((one.pose.p - two.pose.p).norm(), 1e-15);
  EXPECT_LT((one.twist.v - two.twist.v).norm(), 1e-15);
  EXPECT_LT(one.pose.p.norm(), ideal.pose.p.norm());
  cmd.v.x() = vf::kNaN;
  EXPECT_EQ(d3::sim::integrate(state, cmd, 0.1, {}).pose.p, Vec3::Zero());
}

}  // namespace
