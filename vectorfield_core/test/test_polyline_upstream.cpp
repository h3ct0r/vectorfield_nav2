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

// Regressions for the points-mode defects of adrianomcr/vectorfield_stack listed in
// docs/design/upstream_audit.md (rows U08, U09, U11, U12, U13, U14). Test names follow the
// audit's `UpstreamRegression.UNN_Name` scheme.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::AssignStatus;
using vf::ClosestPoint;
using vf::CurveHint;
using vf::EvalContext;
using vf::PolylineCurve;
using vf::SearchPolicy;
using vf::Status;
using vf::Vec2;

namespace
{

Vec2 randomPoint(std::mt19937_64 & rng, std::uniform_real_distribution<double> & dist)
{
  const double a = dist(rng);
  const double b = dist(rng);
  return Vec2(a, b);
}

CurveHint stationHint(double s)
{
  CurveHint hint;
  hint.s = s;
  hint.valid = true;
  return hint;
}

CurveHint positionHint(const Vec2 & x_star)
{
  CurveHint hint;
  hint.x_star = x_star;
  hint.valid = true;
  return hint;
}

EvalContext context(int direction = 1)
{
  EvalContext ctx;
  ctx.direction = direction;
  return ctx;
}

bool sameBits(const Vec2 & a, const Vec2 & b) { return a.x() == b.x() && a.y() == b.y(); }

bool wellFormed(const ClosestPoint & cp)
{
  return cp.valid && vf::isFinite(cp.x_star) && vf::isFinite(cp.T) && vf::isFinite(cp.n) &&
         std::isfinite(cp.D) && std::isfinite(cp.s_star) && std::abs(cp.n.dot(cp.T)) <= 1e-12 &&
         std::abs(cp.T.norm() - 1.0) <= 1e-12;
}

}  // namespace

// U08: the upstream ±10-sample index window overran short paths. Polylines of 2-12 vertices
// with the robot anywhere in a 10 m box give a finite closest point; 1 vertex gives kTooFew; a
// NaN vertex gives kNonFinite.
TEST(UpstreamRegression, U08_ShortPathIndexError)
{
  std::mt19937_64 rng(8);
  std::uniform_real_distribution<double> plan_box(-1.0, 1.0);
  std::uniform_real_distribution<double> robot_box(-5.0, 5.0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int samples = 0;
  int bad = 0;
  for (int n = 2; n <= 12; ++n) {
    for (int trial = 0; trial < 40; ++trial) {
      std::vector<Vec2> points;
      for (int i = 0; i < n; ++i) {
        points.push_back(randomPoint(rng, plan_box));
      }
      for (const SearchPolicy & policy :
           {SearchPolicy::hinted(1.0, 0.5, 0.1), SearchPolicy::windowScan(-0.5, 0.7)}) {
        PolylineCurve curve(policy);
        ASSERT_EQ(curve.assign(points, false), AssignStatus::kOk);
        for (int q = 0; q < 10; ++q) {
          const Vec2 x = randomPoint(rng, robot_box);
          const double s_hint = curve.length(0.0) * (2.0 * unit(rng) - 0.5);
          for (const CurveHint & hint : {CurveHint{}, stationHint(s_hint), positionHint(x)}) {
            bad += wellFormed(curve.closestPoint(x, context(), hint)) ? 0 : 1;
            ++samples;
          }
        }
      }
    }
  }
  EXPECT_EQ(bad, 0);
  EXPECT_GE(samples, 10000);

  PolylineCurve curve;
  const std::vector<Vec2> one{{0.3, 0.4}};
  EXPECT_EQ(curve.assign(one, false), AssignStatus::kTooFew);
  EXPECT_FALSE(curve.closestPoint(Vec2(0.0, 0.0), context(), CurveHint{}).valid);
  const std::vector<Vec2> with_nan{{0.0, 0.0}, {vf::kNaN, 1.0}, {2.0, 0.0}};
  EXPECT_EQ(curve.assign(with_nan, false), AssignStatus::kNonFinite);
  EXPECT_FALSE(curve.closestPoint(Vec2(0.0, 0.0), context(), CurveHint{}).valid);
  // Upstream also crashed on all-NaN distances: a NaN robot position gives an invalid result.
  const std::vector<Vec2> two{{0.0, 0.0}, {1.0, 0.0}};
  ASSERT_EQ(curve.assign(two, false), AssignStatus::kOk);
  EXPECT_FALSE(curve.closestPoint(Vec2(vf::kNaN, vf::kNaN), context(), CurveHint{}).valid);
}

// U09: the upstream window wrapped on open paths, so a robot at the start of a nearly closed
// open path latched onto its end. On an open C-shaped polyline whose ends are 0.2 m apart, a
// robot at the start gets s* ~ 0 and tracks the whole length without jumping to the end (the
// FINAL_APPROACH part of this regression is a Tracker test).
TEST(UpstreamRegression, U09_OpenPathWindowDoesNotWrap)
{
  const double radius = 1.0;
  const double gap_angle = std::asin(0.1);  // ends 2 R sin(gap_angle) = 0.2 m apart
  const double arc = radius * (vf::kTwoPi - 2.0 * gap_angle);
  const int pieces = static_cast<int>(std::ceil(arc / 0.05));
  std::vector<Vec2> points;
  for (int i = 0; i <= pieces; ++i) {
    const double angle = gap_angle + (vf::kTwoPi - 2.0 * gap_angle) * i / pieces;
    points.push_back(radius * vf::heading(angle));
  }
  ASSERT_NEAR((points.front() - points.back()).norm(), 0.2, 1e-12);

  for (const SearchPolicy & policy :
       {SearchPolicy::hinted(1.0, 0.5, 0.1), SearchPolicy::windowScan(-0.5, 0.7)}) {
    PolylineCurve curve(policy);
    ASSERT_EQ(curve.assign(points, false), AssignStatus::kOk);
    const double length = curve.length(0.0);
    // The robot at the start.
    for (const CurveHint & hint : {CurveHint{}, stationHint(0.0), positionHint(points[0])}) {
      EXPECT_EQ(curve.closestPoint(points[0], context(), hint).s_star, 0.0);
    }
    // Slightly into the gap, still nearer the start: s* stays near 0, far from L.
    const Vec2 toward_end = (points.back() - points.front()).normalized();
    const ClosestPoint near_start =
      curve.closestPoint(points[0] + 0.05 * toward_end, context(), CurveHint{});
    EXPECT_LT(std::abs(near_start.s_star), 0.06);
    // Tracking from the start: a robot nearer the end (0.08 m) than the start (0.12 m) keeps
    // s* at the start, because the open window never contains the end.
    const ClosestPoint tracked = curve.closestPoint(
      points[0] + 0.12 * toward_end, context(),
      policy.kind == SearchPolicy::Kind::kHinted ? stationHint(0.0) : positionHint(points[0]));
    EXPECT_LT(tracked.s_star, 0.5 * length);
  }

  // Follow the C from start to end (0.05 m inside), carrying the hint: s* never jumps to the end
  // early and covers the full length.
  PolylineCurve curve(SearchPolicy::hinted(1.0, 0.5, 0.1));
  ASSERT_EQ(curve.assign(points, false), AssignStatus::kOk);
  const double length = curve.length(0.0);
  CurveHint hint;
  double previous = -vf::kInf;
  int backwards = 0;
  int off_track = 0;
  for (double s = 0.0; s <= length; s += 0.02) {
    const Vec2 robot = curve.pointAt(s, 0.0) + 0.05 * vf::J(curve.tangentAt(s, 0.0));
    const ClosestPoint cp = curve.closestPoint(robot, context(), hint);
    backwards += cp.s_star < previous - 1e-12 ? 1 : 0;
    off_track += std::abs(cp.s_star - s) > 5e-3 ? 1 : 0;
    previous = cp.s_star;
    hint = vf::makeHint(cp);
  }
  EXPECT_EQ(backwards, 0);
  EXPECT_EQ(off_track, 0);
  EXPECT_GT(previous, length - 0.03);
}

// U11: upstream resampling dropped the last point, moving the goal. After preprocessing (dedupe,
// fillet, resample) open polylines of 2-50 points at any spacing keep their first and last
// vertices bit for bit.
TEST(UpstreamRegression, U11_ResamplePreservesEndpoints)
{
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> origin(-10.0, 10.0);
  int moved = 0;
  int trials = 0;
  for (int trial = 0; trial < 2000; ++trial) {
    const int n = 2 + trial % 49;
    std::vector<Vec2> raw{randomPoint(rng, origin)};
    double heading = vf::kTwoPi * unit(rng);
    for (int i = 1; i < n; ++i) {
      heading += (unit(rng) - 0.5) * 3.0;
      const double length = 2e-4 * std::pow(2.0 / 2e-4, unit(rng));  // 0.2 mm .. 2 m
      raw.push_back(raw.back() + length * vf::heading(heading));
    }
    const double deviation = trial % 4 == 0 ? 0.0 : 0.01 + 0.3 * unit(rng);
    const double spacing = 0.01 * std::pow(50.0, unit(rng));  // 1 cm .. 0.5 m
    std::vector<Vec2> deduped;
    std::vector<Vec2> filleted;
    std::vector<Vec2> resampled;
    vf::dedupe(raw, false, &deduped);
    vf::filletCorners(deduped, false, deviation, spacing, &filleted);
    vf::resample(filleted, false, spacing, &resampled);
    for (const std::vector<Vec2> * stage : {&deduped, &filleted, &resampled}) {
      if (!sameBits(stage->front(), raw.front()) || !sameBits(stage->back(), raw.back())) {
        ++moved;
      }
    }
    PolylineCurve curve;
    ASSERT_EQ(curve.assign(resampled, false), AssignStatus::kOk);
    if (!sameBits(curve.pointAt(curve.length(0.0), 0.0), raw.back())) {
      ++moved;
    }
    ++trials;
  }
  EXPECT_EQ(trials, 2000);
  EXPECT_EQ(moved, 0);
}

// U12: upstream smoothing pulled the endpoints inward and dropped a last point equal to the
// first. An open polyline whose last point equals its first keeps both through preprocessing,
// the goal stays at the last input point, and at start-up the tie between s = 0 and s = L goes
// to the start of traversal (audit item O2).
TEST(UpstreamRegression, U12_OpenPathEndpointsUnchanged)
{
  const std::vector<Vec2> raw{{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.5}, {0.5, 1.5}, {0.0, 0.0}};
  std::vector<Vec2> deduped;
  std::vector<Vec2> filleted;
  std::vector<Vec2> resampled;
  vf::dedupe(raw, false, &deduped);
  ASSERT_EQ(deduped.size(), raw.size());
  vf::filletCorners(deduped, false, 0.05, 0.05, &filleted);
  vf::resample(filleted, false, 0.05, &resampled);
  EXPECT_TRUE(sameBits(resampled.front(), raw.front()));
  EXPECT_TRUE(sameBits(resampled.back(), raw.back()));

  PolylineCurve curve(SearchPolicy::hinted(1.0, 0.5, 0.1));
  ASSERT_EQ(curve.assign(resampled, false), AssignStatus::kOk);
  EXPECT_FALSE(curve.isClosed());
  EXPECT_TRUE(sameBits(curve.vertices()[0], raw.front()));
  EXPECT_TRUE(sameBits(curve.vertices()[curve.size() - 1], raw.back()));
  const double length = curve.length(0.0);
  EXPECT_TRUE(sameBits(curve.pointAt(length, 0.0), raw.back()));
  EXPECT_EQ(curve.closestPoint(raw.front(), context(1), CurveHint{}).s_star, 0.0);
  EXPECT_EQ(curve.closestPoint(raw.front(), context(-1), CurveHint{}).s_star, length);

  // Only an explicitly closed polyline drops the repeated end point.
  vf::dedupe(raw, true, &deduped);
  EXPECT_EQ(deduped.size(), raw.size() - 1);
}

// U13: upstream measured D to the nearest vertex. With two vertices 2 m apart and a query 0.3 m
// off the midpoint, D = 0.3 m and n^T T = 0 to 1e-12.
TEST(UpstreamRegression, U13_SegmentProjectionNotVertex)
{
  for (const double yaw : {0.0, 0.7, -2.1}) {
    const Vec2 a(1.0, -2.0);
    const Vec2 b = a + 2.0 * vf::heading(yaw);
    const Vec2 midpoint = 0.5 * (a + b);
    const Vec2 x = midpoint + 0.3 * vf::J(vf::heading(yaw));
    for (const SearchPolicy & policy :
         {SearchPolicy::hinted(1.0, 0.5, 0.1), SearchPolicy::windowScan(-0.5, 0.7)}) {
      PolylineCurve curve(policy);
      const std::vector<Vec2> points{a, b};
      ASSERT_EQ(curve.assign(points, false), AssignStatus::kOk);
      const ClosestPoint cp = curve.closestPoint(x, context(), CurveHint{});
      EXPECT_NEAR(cp.D, 0.3, 1e-12);
      EXPECT_LE(std::abs(cp.n.dot(cp.T)), 1e-12);
      EXPECT_LE((cp.x_star - midpoint).norm(), 1e-12);
      EXPECT_NEAR(cp.s_star, 1.0, 1e-12);
    }
  }
}

// U14: upstream never re-localised after a pose jump. On a closed ellipse with the Hinted
// search, a pose teleported 3 m along the curve (beyond search_window) has its s* re-acquired in
// one cycle; a jump that leaves the local search in a wrong local minimum is fixed by the global
// stage.
TEST(UpstreamRegression, U14_RelocalisesAfterPoseJump)
{
  std::vector<Vec2> ellipse;
  for (int i = 0; i < 400; ++i) {
    const double u = vf::kTwoPi * i / 400.0;
    ellipse.emplace_back(3.0 * std::cos(u), 2.0 * std::sin(u));
  }
  PolylineCurve curve(SearchPolicy::hinted(1.0, 0.5, 0.1));
  ASSERT_EQ(curve.assign(ellipse, true), AssignStatus::kOk);
  const double length = curve.length(0.0);
  ASSERT_GT(3.0, curve.searchPolicy().window);

  for (const double s0 : {0.0, 1.3, 7.9, length - 0.4}) {
    const ClosestPoint before = curve.closestPoint(curve.pointAt(s0, 0.0), context(), CurveHint{});
    ASSERT_NEAR(curve.wrapStationDifference(before.s_star - s0, 0.0), 0.0, 1e-9);
    for (const double jump : {3.0, -3.0, 0.5 * length}) {
      const ClosestPoint after =
        curve.closestPoint(curve.pointAt(s0 + jump, 0.0), context(), vf::makeHint(before));
      EXPECT_NEAR(curve.wrapStationDifference(after.s_star - (s0 + jump), 0.0), 0.0, 1e-9)
        << "s0 " << s0 << " jump " << jump;
      EXPECT_LT(after.D, 1e-9);
    }
  }

  // Tracking the bottom (0, -2), then teleported to (0, 0.5): the local search keeps a local
  // minimum at the bottom (D ~ 2.5), the global stage finds the top (D ~ 1.5; the inscribed
  // polyline's chords sit slightly inside the ellipse) and adopts it.
  const ClosestPoint bottom = curve.closestPoint(Vec2(0.0, -2.0), context(), CurveHint{});
  const ClosestPoint jumped = curve.closestPoint(Vec2(0.0, 0.5), context(), vf::makeHint(bottom));
  const ClosestPoint exact = curve.closestPoint(Vec2(0.0, 0.5), context(), CurveHint{});
  EXPECT_NEAR(jumped.x_star.y(), 2.0, 1e-3);
  EXPECT_NEAR(jumped.D, 1.5, 1e-3);
  EXPECT_NEAR(jumped.D, exact.D, vf::kTieTolerance);
  EXPECT_NE(jumped.status_bits & Status::kAmbiguousClosest, 0u);
}
