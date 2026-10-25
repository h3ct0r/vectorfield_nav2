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

// Closed-loop oracles of the 3D CBF filter (math_spec §14.8, O3D-17 to O3D-24): Tracker3 at
// 0.5 m/s and 20 Hz ZOH with the default limits, body radius 0.2 m and margin 0.05 m.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "vectorfield_core/d3.hpp"

namespace
{

namespace d3 = vectorfield_core::d3;
using d3::Vec3;
using vectorfield_core::ObstacleMode;
using vectorfield_core::Phase;
using vectorfield_core::Status;

constexpr double kBody = 0.2;
constexpr double kMargin = 0.05;
constexpr double kDelta = kBody + kMargin;

d3::Tracker3Config cbfConfig()
{
  d3::Tracker3Config config;
  config.obstacles.obstacle_mode = ObstacleMode::kCbf;
  config.obstacles.body_radius = kBody;
  config.obstacles.cbf_margin = kMargin;
  return config;
}

using Records = std::vector<d3::sim::SimulationRecord3>;

Records simulate(
  const d3::Curve3 & curve, const Vec3 & start, const d3::ObstacleSet3 & obstacles, double duration,
  const d3::Tracker3Config & config = cbfConfig())
{
  d3::sim::SimulationOptions3 options;
  options.obstacles = obstacles;
  d3::sim::ClosedLoopSimulator3 sim(d3::Tracker3(config), options);
  d3::Pose3 pose;
  pose.p = start;
  sim.reset(pose);
  d3::Tracker3Input input;
  input.curve = &curve;
  sim.run(input, duration);
  return sim.records();
}

/// Least barrier \f$\min_k \min_j D_j(p_k) - \delta\f$ over a run, from the analytic primitives.
double minBarrier(const Records & records, const d3::ObstacleSet3 & obstacles)
{
  double h = vectorfield_core::kInf;
  for (const auto & r : records) {
    h = std::min(h, obstacles.nearest(r.pose.p).D - kDelta);
  }
  return h;
}

std::shared_ptr<const d3::Curve3> helix(
  const std::vector<std::string> & names, const std::vector<double> & values)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kBuiltin;
  spec.builtin_shape = "helix";
  spec.parameter_names = names;
  spec.parameter_values = values;
  return d3::makeCurve3(spec, {});
}

bool anyStatus(const Records & records, std::uint32_t flag)
{
  return std::any_of(
    records.begin(), records.end(), [flag](const auto & r) { return Status::has(r.status, flag); });
}

}  // namespace

// O3D-17: a sphere 0.2 m off a straight line is passed with h >= -1e-3, and the robot rejoins.
TEST(D3CbfOracle, SphereOffTheLine)
{
  const d3::PolylineCurve3 line({Vec3(-1, 0, 1), Vec3(8, 0, 1)}, false);
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::sphere(Vec3(2, 0.2, 1.05), 0.5));
  const Records run = simulate(line, Vec3(-1, 0, 1), obstacles, 40.0);
  ASSERT_TRUE(run.back().settled);
  EXPECT_GE(minBarrier(run, obstacles), -1e-3);
  RecordProperty("min_barrier", std::to_string(minBarrier(run, obstacles)));
  EXPECT_FALSE(anyStatus(run, Status::kBlocked));
  // It deviated (the line passes 0.3 m from the centre, inside the 0.75 m clearance) ...
  double deviation = 0.0;
  for (const auto & r : run) {
    deviation = std::max(deviation, r.D);
    EXPECT_LE(r.h, obstacles.nearest(r.pose.p).D - kDelta + 1e-12);
  }
  EXPECT_GT(deviation, 0.3);
  // ... and rejoined: D < 0.01 m from x = 6 on (D decays at about G'(0) v_r = 0.64 /s).
  for (const auto & r : run) {
    if (r.pose.p.x() > 6.0) {
      EXPECT_LT(r.D, 0.01) << r.pose.p.transpose();
    }
  }
}

// O3D-18: a sphere off the helix is passed and the helix rejoined.
TEST(D3CbfOracle, SphereOffTheHelix)
{
  const auto curve = helix({}, {});
  d3::ObstacleSet3 obstacles;
  // Helix point at s = 2 pi: (1, 0, 0.5); the sphere sits 0.35 m outward of it, so the body
  // (clearance 0.55 m from the centre) passes 0.2 m inside the helix.
  obstacles.add(d3::Primitive3::sphere(Vec3(1.35, 0.0, 0.5), 0.3));
  const Records run = simulate(*curve, Vec3(1.0, 0.0, 0.0), obstacles, 60.0);
  ASSERT_TRUE(run.back().settled);
  EXPECT_GE(minBarrier(run, obstacles), -1e-3);
  RecordProperty("min_barrier", std::to_string(minBarrier(run, obstacles)));
  bool deviated = false;
  for (const auto & r : run) {
    deviated = deviated || r.D > 0.15;
    if (r.s_star > 2.0 * M_PI + 2.5) {
      EXPECT_LT(r.D, 0.015) << r.s_star;
    }
  }
  EXPECT_TRUE(deviated);
}

// O3D-19: a sphere centred on the line holds the robot: kBlocked, at rest, h ~ 0 (>= 0).
TEST(D3CbfOracle, CentredSphereHolds)
{
  const d3::PolylineCurve3 line({Vec3(-1, 0, 1), Vec3(5, 0, 1)}, false);
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::sphere(Vec3(2, 0, 1), 0.5));
  const Records run = simulate(line, Vec3(-1, 0, 1), obstacles, 30.0);
  const auto & last = run.back();
  EXPECT_FALSE(last.settled);
  EXPECT_EQ(last.phase, Phase::kFollow);
  EXPECT_TRUE(Status::has(last.status, Status::kBlocked));
  EXPECT_LT(last.cmd.v.norm(), 1e-6);
  EXPECT_GE(last.h, 0.0);
  EXPECT_LT(last.h, 0.02);
  EXPECT_GE(minBarrier(run, obstacles), 0.0);
  RecordProperty("min_barrier", std::to_string(minBarrier(run, obstacles)));
  // The hold is steady: once set it stays set (no toggling).
  bool set = false;
  for (const auto & r : run) {
    const bool blocked = Status::has(r.status, Status::kBlocked);
    EXPECT_FALSE(set && !blocked) << r.t;
    set = set || blocked;
  }
}

// O3D-20: a ground plane under a descending helix (lead -0.5, the last turn below z = 0): the
// body never goes below the plane (z >= delta - 1e-3), and it follows the helix down to it.
TEST(D3CbfOracle, GroundPlaneUnderADescendingHelix)
{
  const auto curve = helix({"lead", "cz"}, {-0.5, 1.0});
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::plane(Vec3::Zero(), Vec3::UnitZ()));
  const Records run = simulate(*curve, Vec3(1.0, 0.0, 1.0), obstacles, 60.0);
  double z_min = vectorfield_core::kInf;
  double xy_error = 0.0;
  for (const auto & r : run) {
    z_min = std::min(z_min, r.pose.p.z());
    if (r.t > 5.0) {
      // On the floor the robot still circles under the helix (its horizontal distance stays small).
      xy_error = std::max(xy_error, std::abs(std::hypot(r.pose.p.x(), r.pose.p.y()) - 1.0));
    }
  }
  EXPECT_GE(z_min, kDelta - 1e-3);
  RecordProperty("min_barrier", std::to_string(z_min - kDelta));
  EXPECT_LT(z_min, kDelta + 0.01);  // it did reach the floor
  EXPECT_LT(xy_error, 0.08);
  EXPECT_GT(run.back().s_star, 6.0 * M_PI - 0.1);  // it got to the end, above the goal
}

// O3D-21: a corridor of two walls (|y| <= 0.6) narrower than the helix (|y| <= 1): the robot
// follows inside with h >= -1e-3 and keeps moving.
TEST(D3CbfOracle, CorridorNarrowerThanTheHelix)
{
  const auto curve = helix({}, {});
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::plane(Vec3(0, -0.6, 0), Vec3::UnitY()));
  obstacles.add(d3::Primitive3::plane(Vec3(0, 0.6, 0), -Vec3::UnitY()));
  const Records run = simulate(*curve, Vec3(1.0, 0.0, 0.0), obstacles, 60.0);
  EXPECT_GE(minBarrier(run, obstacles), -1e-3);
  RecordProperty("min_barrier", std::to_string(minBarrier(run, obstacles)));
  double y_max = 0.0;
  for (const auto & r : run) {
    y_max = std::max(y_max, std::abs(r.pose.p.y()));
  }
  EXPECT_GT(y_max, 0.6 - kDelta - 0.01);  // it used the corridor
  EXPECT_FALSE(anyStatus(run, Status::kBlocked));
  EXPECT_TRUE(run.back().settled);  // the goal (1, 0, 1.5) is inside the corridor
}

// O3D-22: between two spheres 0.97 m apart (a 0.07 m gap for the 0.9 m clearance): no
// chattering. Both constraints are in the filter, so F* changes boundedly and monotonically
// (one slowdown, one recovery) while it acts.
TEST(D3CbfOracle, TwoSpheresNoChattering)
{
  const d3::PolylineCurve3 line({Vec3(-1, 0, 1), Vec3(5, 0, 1)}, false);
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::sphere(Vec3(2, 0.48, 1), 0.2));
  obstacles.add(d3::Primitive3::sphere(Vec3(2.05, -0.49, 1.0), 0.2));
  d3::sim::SimulationOptions3 options;
  options.obstacles = obstacles;
  d3::sim::ClosedLoopSimulator3 sim(d3::Tracker3(cbfConfig()), options);
  d3::Pose3 pose;
  pose.p = Vec3(-1, 0.05, 1);
  sim.reset(pose);
  d3::Tracker3Input input;
  input.curve = &line;
  Vec3 previous = Vec3::Zero();
  bool was_active = false;
  double max_jump = 0.0;
  double last_change = 0.0;
  int reversals = 0;
  int both = 0;
  int steps = 0;
  while (steps < 600 && !sim.lastOutput().settled) {
    sim.step(input);
    const d3::Tracker3Output & out = sim.lastOutput();
    if (out.cbf.active || was_active) {
      max_jump = std::max(max_jump, (out.cmd_target.v - previous).norm());
      const double change = out.cmd_target.v.x() - previous.x();
      if (std::abs(change) > 1e-3) {
        reversals += change * last_change < 0.0 ? 1 : 0;
        last_change = change;
      }
    }
    previous = out.cmd_target.v;
    was_active = out.cbf.active;
    both += (out.cbf.active && out.cbf.binds == 3U) ? 1 : 0;
    ++steps;
  }
  EXPECT_TRUE(sim.lastOutput().settled);
  EXPECT_GE(minBarrier(sim.records(), obstacles), -1e-3);
  EXPECT_GT(both, 5);         // both spheres constrain the field at once
  EXPECT_LT(max_jump, 0.08);  // 16 % of v_r per step (the release as the gap is crossed)
  EXPECT_LE(reversals, 2);    // slow down, speed up: no alternation
  EXPECT_FALSE(anyStatus(sim.records(), Status::kBlocked));
  RecordProperty("max_target_jump", std::to_string(max_jump));
  RecordProperty("steps_with_both_binding", both);
}

// O3D-23: a voxelised sphere (0.05 m voxels, 64^3) is passed with the true barrier >= 0: voxel
// sites count from the voxel cubes (half a diagonal inside the centre distance).
TEST(D3CbfOracle, VoxelSphereOffTheLine)
{
  const d3::PolylineCurve3 line({Vec3(-1, 0, 1), Vec3(5, 0, 1)}, false);
  const Vec3 centre(2, 0.2, 1.05);
  const double radius = 0.5;
  const int n = 64;
  const double res = 0.05;
  const Vec3 origin = centre - Vec3::Constant(0.5 * n * res);
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(n * n * n), 0);
  d3::VoxelGrid3 grid;
  for (int iz = 0; iz < n; ++iz) {
    for (int iy = 0; iy < n; ++iy) {
      for (int ix = 0; ix < n; ++ix) {
        const Vec3 c = origin + res * Vec3(ix + 0.5, iy + 0.5, iz + 0.5);
        mask[static_cast<std::size_t>(ix + n * (iy + n * iz))] = (c - centre).norm() <= radius;
      }
    }
  }
  ASSERT_TRUE(grid.setFromMask(mask, n, n, n, res, origin));
  d3::ObstacleSet3 obstacles;
  obstacles.setVoxels(grid);
  d3::ObstacleSet3 analytic;
  analytic.add(d3::Primitive3::sphere(centre, radius));
  const Records run = simulate(line, Vec3(-1, 0, 1), obstacles, 30.0);
  ASSERT_TRUE(run.back().settled);
  EXPECT_GE(minBarrier(run, analytic), 0.0);
  EXPECT_FALSE(anyStatus(run, Status::kBlocked));
  RecordProperty("min_true_barrier", std::to_string(minBarrier(run, analytic)));
}

// O3D-24: without obstacles (or with ones the filter never meets) Tracker3 is bit-identical to
// the tracker without the filter, on the helix and in a moving frame.
TEST(D3CbfOracle, ObstacleFreeIsBitIdentical)
{
  const auto curve = helix({}, {});
  d3::ObstacleSet3 none;
  d3::ObstacleSet3 far;
  far.add(d3::Primitive3::sphere(Vec3(30, 30, 30), 1.0));
  d3::Tracker3Config plain;
  for (const bool moving : {false, true}) {
    d3::sim::SimulationOptions3 options;
    options.frame.moving = moving;
    options.frame.twist.w = Vec3(0, 0, moving ? 0.1 : 0.0);
    const auto run = [&](const d3::Tracker3Config & config, const d3::ObstacleSet3 & set) {
      d3::sim::SimulationOptions3 o = options;
      o.obstacles = set;
      d3::sim::ClosedLoopSimulator3 sim(d3::Tracker3(config), o);
      d3::Pose3 pose;
      pose.p = Vec3(1.3, 0.2, -0.3);
      sim.reset(pose);
      d3::Tracker3Input input;
      input.curve = curve.get();
      sim.run(input, 50.0);
      return sim.records();
    };
    const Records reference = run(plain, none);
    for (const Records & other : {run(cbfConfig(), none), run(cbfConfig(), far)}) {
      ASSERT_EQ(other.size(), reference.size());
      for (std::size_t k = 0; k < reference.size(); ++k) {
        ASSERT_EQ(other[k].pose.p, reference[k].pose.p) << k;
        ASSERT_EQ(other[k].pose.yaw, reference[k].pose.yaw) << k;
        ASSERT_EQ(other[k].cmd.v, reference[k].cmd.v) << k;
        ASSERT_EQ(other[k].status, reference[k].status) << k;
      }
    }
  }
}

// Interaction with the goal hold: a cancel and a goal_seq change clear the CBF hold.
TEST(D3CbfOracle, CancelAndGoalChangeClearTheHold)
{
  const d3::PolylineCurve3 line({Vec3(-1, 0, 1), Vec3(5, 0, 1)}, false);
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::sphere(Vec3(0.5, 0, 1), 0.3));
  const d3::Tracker3 tracker(cbfConfig());
  d3::Tracker3State state = d3::Tracker3::initialState();
  d3::Tracker3Input input;
  input.curve = &line;
  input.obstacles = &obstacles;
  input.pose.p = Vec3(0.5 - 0.3 - kDelta - 0.005, 0, 1);  // h = 0.005: F* ~ 0.01 m/s
  d3::Tracker3Output out = tracker.step(input, state, &state);
  EXPECT_TRUE(Status::has(out.status, Status::kBlocked));
  EXPECT_TRUE(state.blocked);
  EXPECT_EQ(out.cmd_target.v, Vec3::Zero());
  input.cancel = true;
  out = tracker.step(input, state, &state);
  EXPECT_EQ(out.phase, Phase::kStopped);
  EXPECT_FALSE(Status::has(out.status, Status::kBlocked));  // not evaluated in STOPPED
  input.cancel = false;
  input.goal_seq = 1;
  input.obstacles = nullptr;
  out = tracker.step(input, state, &state);
  EXPECT_EQ(out.phase, Phase::kFollow);
  EXPECT_FALSE(state.blocked);
  EXPECT_GT(out.cmd_target.v.x(), 0.4);
}
