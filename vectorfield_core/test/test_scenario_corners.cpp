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

// O42 Rotate to field (math_spec §9.2 "Why π/2 and π/4, measured at the tracked point", §12.6) and
// the corner scenarios of plan §3.5: FL corners of 60–170°, closed triangle, square and pentagon,
// a U-turn of radius 0.15 m and goals behind the robot. TC, FL, no obstacles.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

struct TransitResult
{
  double time{vf::kInf};  // transit time; +inf when not completed within the timeout
  int entries{0};         // ROTATE_TO_FIELD entries up to the transit
  std::vector<Cycle> cycles;
};

// Runs FL until `done(cycle)` holds or the timeout, with or without ROTATE_TO_FIELD.
TransitResult runTransit(
  const vf::Curve & curve, const std::optional<vf::GoalInfo> & goal, const Pose2 & start,
  bool rotate_to_field, double timeout, const std::function<bool(const Cycle &)> & done)
{
  vf::CoreConfig config = tcConfig();
  config.phases.use_rotate_to_field = rotate_to_field;
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(config, options);
  runner.reset(start);
  runner.input().curve = &curve;
  runner.input().goal = goal;
  TransitResult r;
  runner.run(timeout, [&](const Cycle & c) {
    if (done(c)) {
      r.time = c.rec.t + kPeriod;
      return true;
    }
    return false;
  });
  r.cycles = runner.cycles();
  r.entries = countEntries(r.cycles, Phase::kRotateToField);
  return r;
}

}  // namespace

// O42, corners: an open path turning left by 60°, 90°, 120°, 150° and 170° at the origin (legs of
// 3 m), FL from 2 m before the corner: at most one ROTATE_TO_FIELD entry, and the transit time
// (until the base is 1 m past the corner along the outgoing leg) is at most 1.2× the run without
// the phase. Timeout 30 s per run.
TEST(ScenarioCorners, O42_CornersOneEntryAndFastTransit)
{
  for (const double deg : {60.0, 90.0, 120.0, 150.0, 170.0}) {
    SCOPED_TRACE("turn " + std::to_string(deg) + "°");
    const double th = deg * vf::kPi / 180.0;
    const Vec2 dir(std::cos(th), std::sin(th));
    const Vec2 end = 3.0 * dir;
    const auto curve = makePolyline({Vec2(-3.0, 0.0), Vec2(0.0, 0.0), end}, false);
    // Transit: the base is 1 m along the outgoing leg and within 0.5 m of its line.
    const auto done = [dir](const Cycle & c) {
      const double along = c.pose_after.p.dot(dir);
      return along >= 1.0 && (c.pose_after.p - along * dir).norm() < 0.5;
    };
    const Pose2 start{Vec2(-2.0, 0.0), 0.0};
    const TransitResult with = runTransit(*curve, goalAt(end), start, true, 30.0, done);
    const TransitResult without = runTransit(*curve, goalAt(end), start, false, 30.0, done);
    ASSERT_TRUE(std::isfinite(with.time)) << "did not pass the corner with ROTATE_TO_FIELD";
    EXPECT_LE(with.entries, 1);
    if (std::isfinite(without.time)) {
      EXPECT_LE(with.time, 1.2 * without.time)
        << "with " << with.time << " s, without " << without.time << " s";
    }
  }
}

// O42, closed polygons with 2 m sides (triangle, square, pentagon), start mid-side along it: one
// lap with at most one ROTATE_TO_FIELD entry per corner, and the lap time at most 1.2× the run
// without the phase. Progress is the unwrapped station of the base's brute-force projection.
// Timeout 60 s per run.
TEST(ScenarioCorners, O42_PolygonsOneEntryPerCorner)
{
  for (const int sides : {3, 4, 5}) {
    SCOPED_TRACE(std::to_string(sides) + " sides");
    const double R = 2.0 / (2.0 * std::sin(vf::kPi / sides));
    const std::vector<Vec2> points = regularPolygon(R, sides, -vf::kPi / 2.0 - vf::kPi / sides);
    const double perimeter = polylineLength(points, true);
    const auto curve = makePolyline(points, true);
    const Vec2 mid = 0.5 * (points[0] + points[1]);
    const Vec2 along = (points[1] - points[0]).normalized();
    const Pose2 start{mid, std::atan2(along.y(), along.x())};
    const auto make_done = [&points, perimeter]() {
      auto last = std::make_shared<double>(vf::kNaN);
      auto total = std::make_shared<double>(0.0);
      return [&points, perimeter, last, total](const Cycle & c) {
        double station = 0.0;
        polylineDistance(points, true, c.pose_after.p, &station);
        if (!std::isnan(*last)) {
          *total += vf::wrapPeriodic(station - *last, perimeter);
        }
        *last = station;
        return *total >= perimeter;
      };
    };
    const TransitResult with = runTransit(*curve, std::nullopt, start, true, 60.0, make_done());
    const TransitResult without = runTransit(*curve, std::nullopt, start, false, 60.0, make_done());
    ASSERT_TRUE(std::isfinite(with.time)) << "lap not completed";
    EXPECT_LE(with.entries, sides);
    if (std::isfinite(without.time)) {
      EXPECT_LE(with.time, 1.2 * without.time)
        << "with " << with.time << " s, without " << without.time << " s";
    }
  }
}

// O42, U-turn of radius 0.15 m (legs 0.3 m apart, the turn discretised every 2.5 cm): FL with
// ROTATE_TO_FIELD completes it (settled at the goal). Timeout 40 s.
TEST(ScenarioCorners, O42_TightUTurnCompletes)
{
  std::vector<Vec2> points = linePoints(Vec2(-2.0, 0.0), Vec2(0.0, 0.0), 0.5);
  appendPoints(&points, arcPoints(Vec2(0.0, 0.15), 0.15, -vf::kHalfPi, vf::kHalfPi, 0.025));
  appendPoints(&points, linePoints(Vec2(0.0, 0.3), Vec2(-2.0, 0.3), 0.5));
  const auto curve = makePolyline(points, false);
  const Vec2 goal(-2.0, 0.3);
  const TransitResult r = runTransit(
    *curve, goalAt(goal), Pose2{Vec2(-1.5, 0.0), 0.0}, true, 40.0,
    [](const Cycle & c) { return c.out.settled; });
  ASSERT_TRUE(std::isfinite(r.time)) << "U-turn not completed";
  EXPECT_LE((r.cycles.back().rec.pose.p - goal).norm(), kXyTol);
}

// O42, goals behind (math_spec §9.2): paths from the robot to goals 2 m away at 120° and 180°
// from its heading: no v < 0 on any cycle (final command and box target), exactly one
// ROTATE_TO_FIELD entry, and the goal is reached (settled). Timeout 20 s.
TEST(ScenarioCorners, O42_GoalsBehindNeverReverse)
{
  for (const double deg : {120.0, 180.0}) {
    SCOPED_TRACE("goal at " + std::to_string(deg) + "°");
    const double b = deg * vf::kPi / 180.0;
    const Vec2 goal = 2.0 * Vec2(std::cos(b), std::sin(b));
    const auto curve = makePolyline({Vec2(0.0, 0.0), goal}, false);
    const TransitResult r = runTransit(
      *curve, goalAt(goal), Pose2{Vec2(0.0, 0.0), 0.0}, true, 20.0,
      [](const Cycle & c) { return c.out.settled; });
    ASSERT_TRUE(std::isfinite(r.time)) << "goal not reached";
    EXPECT_EQ(r.entries, 1);
    for (const Cycle & c : r.cycles) {
      EXPECT_GE(c.rec.cmd.vx, -1e-12) << "t = " << c.rec.t;
      EXPECT_GE(c.out.cmd_target.vx, -1e-12) << "t = " << c.rec.t;
    }
  }
}
