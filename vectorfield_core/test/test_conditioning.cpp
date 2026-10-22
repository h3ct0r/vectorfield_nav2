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

// Path conditioning (math_spec §2.8): dedupe, corner fillet, densify-only resampling, end
// tangent, tail conditioning, cusp decimation (oracle O14), the one-pose rule and kappa_path.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::Pose2;
using vf::Vec2;

namespace
{

Vec2 randomPoint(std::mt19937_64 & rng, std::uniform_real_distribution<double> & dist)
{
  const double a = dist(rng);
  const double b = dist(rng);
  return Vec2(a, b);
}

double segmentDistance(const Vec2 & x, const Vec2 & a, const Vec2 & b)
{
  const Vec2 ab = b - a;
  const double length_sq = ab.squaredNorm();
  const double tau = length_sq > 0.0 ? std::clamp((x - a).dot(ab) / length_sq, 0.0, 1.0) : 0.0;
  return (x - (a + tau * ab)).norm();
}

// Distance from x to a polyline (no rays).
double polylineDistance(const Vec2 & x, const std::vector<Vec2> & points, bool closed)
{
  if (points.size() == 1) {
    return (x - points[0]).norm();
  }
  double best = vf::kInf;
  const std::size_t segments = closed ? points.size() : points.size() - 1;
  for (std::size_t k = 0; k < segments; ++k) {
    best = std::min(best, segmentDistance(x, points[k], points[(k + 1) % points.size()]));
  }
  return best;
}

bool sameBits(const Vec2 & a, const Vec2 & b) { return a.x() == b.x() && a.y() == b.y(); }

// A NavFn-style tail: gradient steps of half a cell from `start` toward the goal cell centre
// `cell`, then `cell` itself, then nav2's smoothApproachToGoal: replace the last pose by the goal
// if the goal is closer to the second-to-last pose, else append the goal.
std::vector<Vec2> navfnTail(const Vec2 & start, const Vec2 & cell, const Vec2 & goal, double res)
{
  std::vector<Vec2> path;
  const double step = 0.5 * res;
  const Vec2 delta = cell - start;
  const double length = delta.norm();
  const Vec2 direction = delta / length;
  for (int j = 0; j * step < length - 1e-9; ++j) {
    path.push_back(start + (j * step) * direction);
  }
  path.push_back(cell);
  if (path.size() >= 2) {
    const Vec2 second_to_last = path[path.size() - 2];
    if ((path.back() - second_to_last).squaredNorm() > (goal - second_to_last).squaredNorm()) {
      path.back() = goal;
      return path;
    }
  }
  path.push_back(goal);
  return path;
}

// Poses with NavFn's identity orientation.
std::vector<Pose2> unoriented(const std::vector<Vec2> & points)
{
  std::vector<Pose2> plan;
  for (const Vec2 & point : points) {
    plan.push_back(Pose2{point, 0.0});
  }
  return plan;
}

// Smac's smoother orientations: every pose but the last along its step (unchanged over a
// zero-length step).
void orientAlongPath(std::vector<Pose2> * plan)
{
  for (std::size_t k = 0; k + 1 < plan->size(); ++k) {
    const Vec2 step = (*plan)[k + 1].p - (*plan)[k].p;
    if (step.norm() > 1e-4) {
      (*plan)[k].yaw = vf::angleOf(step);
    }
  }
}

// Identity-yaw poses every half cell from a toward b (b excluded): a NavFn leg.
void appendHalfCells(std::vector<Pose2> * plan, const Vec2 & a, const Vec2 & b, double res)
{
  const double step = 0.5 * res;
  const Vec2 delta = b - a;
  const double length = delta.norm();
  for (int j = 0; j * step < length - 1e-9; ++j) {
    plan->push_back(Pose2{a + (j * step / length) * delta, 0.0});
  }
}

// A NavigateThroughPoses plan of NavFn: the leg to the exact waypoint (with its yaw), then the
// next leg from NavFn's start, the grid vertex nearest the waypoint (a hook of up to 0.71 cell),
// to the goal.
std::vector<Pose2> navfnThroughWaypoint(
  const Vec2 & start, const Pose2 & waypoint, const Pose2 & goal, double res)
{
  std::vector<Pose2> plan;
  appendHalfCells(&plan, start, waypoint.p, res);
  plan.push_back(waypoint);
  const Vec2 vertex(std::round(waypoint.p.x() / res) * res, std::round(waypoint.p.y() / res) * res);
  appendHalfCells(&plan, vertex, goal.p, res);
  plan.push_back(goal);
  return plan;
}

// Drives a vehicle from the plan's last pose for `length` with drive sign q (+1 forward) and
// steering side s (+1 left, 0 straight, -1 right) on radius R; poses every `spacing` with the
// vehicle yaw, as Smac Hybrid-A* and Lattice emit them.
void appendDrive(std::vector<Pose2> * plan, double R, int q, int s, double length, double spacing)
{
  const Pose2 from = plan->back();
  const int n = std::max(1, static_cast<int>(std::ceil(length / spacing)));
  for (int k = 1; k <= n; ++k) {
    const double sigma = length * k / n;
    if (s == 0) {
      plan->push_back(Pose2{from.p + (q * sigma) * vf::heading(from.yaw), from.yaw});
      continue;
    }
    const Vec2 centre = from.p + (s * R) * vf::J(vf::heading(from.yaw));
    const double a = q * s * sigma / R;
    plan->push_back(Pose2{Vec2(centre + vf::rotate(a, from.p - centre)), from.yaw + a});
  }
}

std::vector<std::size_t> cuspsOf(const std::vector<Pose2> & plan, double chord)
{
  std::vector<std::size_t> cusps;
  vf::findCusps(plan, chord, &cusps);
  return cusps;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// dedupe
// ---------------------------------------------------------------------------------------------

TEST(Conditioning, DedupeDropsCloseVerticesAndKeepsEndpoints)
{
  const Vec2 a(0.0, 0.0);
  const Vec2 b(1.0, 0.0);
  const Vec2 c(1.0, 1.0);
  std::vector<Vec2> out;

  const std::vector<Vec2> interior{a, a + Vec2(5e-4, 0.0), b, b + Vec2(0.0, 2e-4), c};
  vf::dedupe(interior, false, &out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_TRUE(sameBits(out[0], a));
  EXPECT_TRUE(sameBits(out[1], b));
  EXPECT_TRUE(sameBits(out[2], c));

  // The last vertex (the goal) replaces a close previous vertex instead of being dropped.
  const Vec2 goal = c + Vec2(4e-4, 0.0);
  const std::vector<Vec2> close_goal{a, b, c, goal};
  vf::dedupe(close_goal, false, &out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_TRUE(sameBits(out.back(), goal));

  // The first vertex is never replaced.
  const std::vector<Vec2> close_start{a, a + Vec2(3e-4, 0.0), b};
  vf::dedupe(close_start, false, &out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_TRUE(sameBits(out[0], a));
  EXPECT_TRUE(sameBits(out[1], b));

  // Open out-and-back: the last point equals the first and both are kept (U12).
  const std::vector<Vec2> out_and_back{a, b, c, a};
  vf::dedupe(out_and_back, false, &out);
  EXPECT_EQ(out.size(), 4u);
  // Closed: the closing duplicates are dropped, including an interior one near the start.
  const std::vector<Vec2> closing{a, b, c, Vec2(6e-4, 3e-4), Vec2(2e-4, 0.0)};
  vf::dedupe(closing, true, &out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_TRUE(sameBits(out[2], c));
  vf::dedupe(out_and_back, true, &out);
  EXPECT_EQ(out.size(), 3u);

  // Degenerate inputs: both endpoints kept when distinct, a single vertex otherwise.
  const std::vector<Vec2> tiny{a, a + Vec2(4e-4, 0.0)};
  vf::dedupe(tiny, false, &out);
  EXPECT_EQ(out.size(), 2u);
  const std::vector<Vec2> same{a, a, a};
  vf::dedupe(same, false, &out);
  EXPECT_EQ(out.size(), 1u);
  const std::vector<Vec2> single{c};
  vf::dedupe(single, false, &out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(sameBits(out[0], c));
  const std::vector<Vec2> none;
  vf::dedupe(none, false, &out);
  EXPECT_TRUE(out.empty());

  // A custom spacing.
  const std::vector<Vec2> coarse{{0.0, 0.0}, {0.05, 0.0}, {0.2, 0.0}, {0.25, 0.0}};
  vf::dedupe(coarse, false, &out, 0.1);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_TRUE(sameBits(out[1], Vec2(0.25, 0.0)));
}

// ---------------------------------------------------------------------------------------------
// filletCorners
// ---------------------------------------------------------------------------------------------

TEST(Conditioning, FilletReplacesCornerByTangentArc)
{
  const double delta = 0.05;
  const double spacing = 0.01;
  const std::vector<Vec2> in{{-2.0, 0.0}, {0.0, 0.0}, {0.0, 2.0}};
  std::vector<Vec2> out;
  vf::filletCorners(in, false, delta, spacing, &out);

  const double theta = vf::kHalfPi;
  const double radius = delta / (1.0 / std::cos(0.5 * theta) - 1.0);  // R_delta < R_l = 1
  const double tangent = radius * std::tan(0.5 * theta);
  const Vec2 start(-tangent, 0.0);
  const Vec2 end(0.0, tangent);
  const Vec2 centre = Vec2(0.0, 0.0) + radius / std::cos(0.5 * theta) *
                                         (Vec2(0.0, 1.0) - Vec2(1.0, 0.0)).normalized();
  const std::size_t pieces = 20;  // ceil(R theta / 0.01) = 19, rounded up to even
  ASSERT_EQ(out.size(), pieces + 3);
  EXPECT_TRUE(sameBits(out.front(), in.front()));
  EXPECT_TRUE(sameBits(out.back(), in.back()));
  EXPECT_LE((out[1] - start).norm(), 1e-15);
  EXPECT_LE((out[pieces + 1] - end).norm(), 1e-15);
  EXPECT_LE((centre - Vec2(-radius, radius)).norm(), 1e-15);
  for (std::size_t i = 1; i <= pieces + 1; ++i) {
    EXPECT_NEAR((out[i] - centre).norm(), radius, 1e-12) << i;
    if (i > 1) {
      EXPECT_LE((out[i] - out[i - 1]).norm(), spacing);
    }
  }
  // The vertex is exactly delta from the filleted path; the path stays within delta.
  EXPECT_NEAR(polylineDistance(in[1], out, false), delta, 1e-12);
  for (const Vec2 & point : out) {
    EXPECT_LE(polylineDistance(point, in, false), delta + 1e-12);
  }
}

TEST(Conditioning, FilletRadiusIsBoundedBySegmentLengths)
{
  std::vector<Vec2> out;
  // R_l = min(l)/(2 tan(theta/2)) binds: the tangent points are the segment midpoints.
  const std::vector<Vec2> short_legs{{-0.1, 0.0}, {0.0, 0.0}, {0.0, 0.1}};
  vf::filletCorners(short_legs, false, 0.05, 0.01, &out);
  EXPECT_LE((out[1] - Vec2(-0.05, 0.0)).norm(), 1e-15);
  EXPECT_LE((out[out.size() - 2] - Vec2(0.0, 0.05)).norm(), 1e-15);
  EXPECT_LT(polylineDistance(short_legs[1], out, false), 0.05);

  // Adjacent fillets share a tangent point at the shared segment's midpoint: emitted once.
  const std::vector<Vec2> zigzag{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}};
  vf::filletCorners(zigzag, false, 1e9, 0.05, &out);
  std::size_t midpoints = 0;
  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_GE((out[i] - out[i - 1]).norm(), vf::kEpsD);
    midpoints += (out[i] - Vec2(1.0, 0.5)).norm() < 1e-12 ? 1 : 0;
  }
  EXPECT_EQ(midpoints, 1u);
  EXPECT_TRUE(sameBits(out.front(), zigzag.front()));
  EXPECT_TRUE(sameBits(out.back(), zigzag.back()));
}

TEST(Conditioning, FilletKeepsCollinearReversedAndDisabledCases)
{
  std::vector<Vec2> out;
  const std::vector<Vec2> collinear{{0.0, 0.0}, {1.0, 0.0}, {2.5, 0.0}};
  vf::filletCorners(collinear, false, 0.1, 0.01, &out);
  EXPECT_EQ(out, collinear);
  const std::vector<Vec2> reversal{{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.0}};
  vf::filletCorners(reversal, false, 0.1, 0.01, &out);
  EXPECT_EQ(out, reversal);
  const std::vector<Vec2> corner{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  vf::filletCorners(corner, false, 0.0, 0.01, &out);
  EXPECT_EQ(out, corner);
  vf::filletCorners(corner, false, -1.0, 0.01, &out);
  EXPECT_EQ(out, corner);
  const std::vector<Vec2> segment{{0.0, 0.0}, {1.0, 0.0}};
  vf::filletCorners(segment, false, 0.1, 0.01, &out);
  EXPECT_EQ(out, segment);
}

TEST(Conditioning, FilletClosedPolygon)
{
  const double delta = 0.05;
  const std::vector<Vec2> square{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  std::vector<Vec2> out;
  vf::filletCorners(square, true, delta, 0.02, &out);
  // Four arcs of ceil(R pi/2 / 0.02) = 10 pieces (even) each, 11 points per arc.
  EXPECT_EQ(out.size(), 44u);
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_GE((out[(i + 1) % out.size()] - out[i]).norm(), vf::kEpsD);
    EXPECT_LE(polylineDistance(out[i], square, true), delta + 1e-12);
  }
  for (const Vec2 & vertex : square) {
    EXPECT_NEAR(polylineDistance(vertex, out, true), delta, 1e-12);
  }
}

// Random polylines: endpoints bit-identical, the filleted path within delta of the input, every
// input vertex within delta of the output, no duplicates.
TEST(Conditioning, FilletDeviationBoundOnRandomPolylines)
{
  std::mt19937_64 rng(2028);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> coord(-4.0, 4.0);
  double worst_out = 0.0;
  double worst_in = 0.0;
  int duplicates = 0;
  int endpoint_moves = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const bool closed = trial % 3 == 0;
    std::vector<Vec2> raw;
    const int n = 3 + trial % 12;
    for (int i = 0; i < n; ++i) {
      raw.push_back(randomPoint(rng, coord));
    }
    std::vector<Vec2> in;
    vf::dedupe(raw, closed, &in);
    const double delta = 0.005 + 0.3 * unit(rng);
    const double spacing = 0.005 + 0.1 * unit(rng);
    std::vector<Vec2> out;
    vf::filletCorners(in, closed, delta, spacing, &out);
    if (!closed) {
      endpoint_moves +=
        sameBits(out.front(), in.front()) && sameBits(out.back(), in.back()) ? 0 : 1;
    }
    for (std::size_t i = 0; i < out.size(); ++i) {
      worst_out = std::max(worst_out, polylineDistance(out[i], in, closed) - delta);
      if (i > 0 || closed) {
        const Vec2 & previous = out[i == 0 ? out.size() - 1 : i - 1];
        duplicates += (out[i] - previous).norm() < vf::kEpsD ? 1 : 0;
      }
    }
    for (const Vec2 & vertex : in) {
      worst_in = std::max(worst_in, polylineDistance(vertex, out, closed) - delta);
    }
  }
  EXPECT_LE(worst_out, 1e-12);
  EXPECT_LE(worst_in, 1e-12);
  EXPECT_EQ(duplicates, 0);
  EXPECT_EQ(endpoint_moves, 0);
}

// ---------------------------------------------------------------------------------------------
// resample
// ---------------------------------------------------------------------------------------------

TEST(Conditioning, ResampleOnlyDensifies)
{
  const std::vector<Vec2> in{{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.12}};
  std::vector<Vec2> out;
  vf::resample(in, false, 0.05, &out);
  ASSERT_EQ(out.size(), 24u);  // 20 + 3 pieces + the last vertex
  EXPECT_TRUE(sameBits(out[0], in[0]));
  EXPECT_TRUE(sameBits(out[20], in[1]));
  EXPECT_TRUE(sameBits(out[23], in[2]));
  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_LE((out[i] - out[i - 1]).norm(), 0.05 + 1e-15);
    EXPECT_LE(polylineDistance(out[i], in, false), 1e-15);
  }
  EXPECT_NEAR(out[21].y(), 0.04, 1e-15);
  EXPECT_NEAR(out[22].y(), 0.08, 1e-15);

  const std::vector<Vec2> triangle{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  vf::resample(triangle, true, 0.3, &out);
  ASSERT_EQ(out.size(), 13u);  // 4 + 4 + 5 pieces, the first vertex not repeated
  EXPECT_TRUE(sameBits(out[0], triangle[0]));
  EXPECT_TRUE(sameBits(out[4], triangle[1]));
  EXPECT_TRUE(sameBits(out[8], triangle[2]));
  EXPECT_FALSE(sameBits(out.back(), out.front()));
  EXPECT_LE(polylineDistance(out.back(), triangle, true), 1e-15);
  EXPECT_LE((out.front() - out.back()).norm(), 0.3);

  vf::resample(in, false, 10.0, &out);
  EXPECT_EQ(out, in);
  for (const double bad : {0.0, -1.0, vf::kNaN, vf::kInf}) {
    vf::resample(in, false, bad, &out);
    EXPECT_EQ(out, in);
  }
}

// ---------------------------------------------------------------------------------------------
// Plugin window helpers
// ---------------------------------------------------------------------------------------------

TEST(Conditioning, EndTangentFallbacks)
{
  const double chord = 0.1;
  const std::vector<Vec2> plan{{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.05, 0.0}, {1.06, 0.01}};
  const vf::EndTangent found = vf::endTangent(plan, chord);
  EXPECT_TRUE(found.valid);
  EXPECT_TRUE(found.found_chord);
  EXPECT_TRUE(found.tangent.isApprox((plan[4] - plan[1]).normalized(), 1e-15));

  const std::vector<Vec2> short_plan{{0.0, 0.0}, {0.03, 0.0}, {0.05, 0.01}};
  const vf::EndTangent fallback = vf::endTangent(short_plan, chord);
  EXPECT_TRUE(fallback.valid);
  EXPECT_FALSE(fallback.found_chord);
  EXPECT_TRUE(fallback.tangent.isApprox((short_plan[2] - short_plan[0]).normalized(), 1e-15));

  const std::vector<Vec2> collapsed{{1.0, 1.0}, {1.0003, 1.0}, {1.0005, 1.0}};
  EXPECT_FALSE(vf::endTangent(collapsed, chord).valid);
  const std::vector<Vec2> one{{1.0, 1.0}};
  EXPECT_FALSE(vf::endTangent(one, chord).valid);
  const std::vector<Vec2> none;
  EXPECT_FALSE(vf::endTangent(none, chord).valid);
}

TEST(Conditioning, TailConditionFixesTheLastSegment)
{
  const double chord = 0.1;
  const Vec2 direction(1.0, 0.0);
  std::vector<Vec2> out;

  const std::vector<Vec2> window{{0.0, 0.0},  {0.5, 0.0},   {0.85, 0.0},
                                 {0.92, 0.0}, {0.95, 0.01}, {1.0, 0.0}};
  vf::tailCondition(window, direction, chord, &out);
  const Vec2 virtual_vertex = window.back() - chord * direction;
  ASSERT_EQ(out.size(), 5u);
  EXPECT_TRUE(sameBits(out[2], window[2]));
  EXPECT_TRUE(sameBits(out[out.size() - 2], virtual_vertex));
  EXPECT_TRUE(sameBits(out.back(), window.back()));
  EXPECT_NEAR((out.back() - out[out.size() - 2]).norm(), chord, 1e-15);

  // A remaining vertex within 1 mm of the virtual vertex is removed too.
  const std::vector<Vec2> near_virtual{{0.0, 0.0}, {0.5, 0.0}, {0.9, 5e-4}, {1.0, 0.0}};
  vf::tailCondition(near_virtual, direction, chord, &out);
  ASSERT_EQ(out.size(), 4u);
  EXPECT_TRUE(sameBits(out[2], virtual_vertex));

  const std::vector<Vec2> single{{1.0, 0.0}};
  vf::tailCondition(single, direction, chord, &out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_TRUE(sameBits(out[0], virtual_vertex));

  // The end direction comes from the raw plan, not from the window geometry.
  const std::vector<Vec2> sideways{{0.0, 0.0}, {1.0, 0.0}};
  vf::tailCondition(sideways, Vec2(0.0, 1.0), chord, &out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_TRUE(out[1].isApprox(Vec2(1.0, -0.1), 1e-15));

  const std::vector<Vec2> empty;
  vf::tailCondition(empty, direction, chord, &out);
  EXPECT_TRUE(out.empty());

  // Re-pruned windows (the server drops poses behind the robot) keep the same last segment.
  std::vector<Vec2> plan;
  for (int i = 0; i <= 40; ++i) {
    plan.emplace_back(0.05 * i, 0.02 * std::sin(0.3 * i));
  }
  plan.emplace_back(2.0 + 0.01, 0.03);  // the goal hook
  const Vec2 end_tangent = vf::endTangent(plan, chord).tangent;
  std::vector<Vec2> reference;
  vf::tailCondition(plan, end_tangent, chord, &reference);
  for (std::size_t first = 1; first + 1 < plan.size(); ++first) {
    const std::vector<Vec2> pruned(plan.begin() + static_cast<std::ptrdiff_t>(first), plan.end());
    vf::tailCondition(pruned, end_tangent, chord, &out);
    ASSERT_GE(out.size(), 2u);
    EXPECT_TRUE(sameBits(out[out.size() - 2], reference[reference.size() - 2]));
    EXPECT_TRUE(sameBits(out.back(), reference.back()));
  }
}

TEST(Conditioning, DecimateChordsPopsBeforeAppendingTheGoal)
{
  const double chord = 0.1;
  std::vector<Vec2> plan;
  for (int i = 0; i <= 25; ++i) {
    plan.emplace_back(i / 25.0, 0.0);
  }
  plan.emplace_back(1.0, 0.03);  // a sub-cell hook at the goal
  std::vector<Vec2> out;
  vf::decimateChords(plan, chord, &out);
  ASSERT_GE(out.size(), 3u);
  EXPECT_TRUE(sameBits(out.front(), plan.front()));
  EXPECT_TRUE(sameBits(out.back(), plan.back()));
  EXPECT_NEAR(out[out.size() - 2].x(), 0.84, 1e-12);
  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_GE((out[i] - out[i - 1]).norm(), chord);
  }
  EXPECT_FALSE(vf::hasCusp(out));

  // Random plans: every chord >= l_c except a final chord that starts at the first pose.
  std::mt19937_64 rng(14);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int violations = 0;
  for (int trial = 0; trial < 2000; ++trial) {
    std::vector<Vec2> walk{{0.0, 0.0}};
    double heading = vf::kTwoPi * unit(rng);
    const int count = 1 + trial % 60;
    for (int i = 0; i < count; ++i) {
      heading += (unit(rng) - 0.5) * 1.5;
      const double step = 0.06 * unit(rng);
      walk.push_back(walk.back() + step * vf::heading(heading));
    }
    vf::decimateChords(walk, chord, &out);
    if (!sameBits(out.front(), walk.front()) || !sameBits(out.back(), walk.back())) {
      ++violations;
    }
    for (std::size_t i = 1; i < out.size(); ++i) {
      const bool final_from_first = out.size() == 2;
      if ((out[i] - out[i - 1]).norm() < chord && !final_from_first) {
        ++violations;
      }
    }
  }
  EXPECT_EQ(violations, 0);

  const std::vector<Vec2> one{{1.0, 2.0}};
  vf::decimateChords(one, chord, &out);
  EXPECT_EQ(out.size(), 1u);
  const std::vector<Vec2> none;
  vf::decimateChords(none, chord, &out);
  EXPECT_TRUE(out.empty());
  const std::vector<Vec2> short_plan{{0.0, 0.0}, {0.03, 0.0}, {0.05, 0.0}};
  vf::decimateChords(short_plan, chord, &out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_TRUE(sameBits(out[1], short_plan[2]));
}

TEST(Conditioning, HasCuspDetectsReversingChords)
{
  const std::vector<Vec2> straight{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const std::vector<Vec2> right_angle{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const std::vector<Vec2> reversing{{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.1}};
  const std::vector<Vec2> two{{0.0, 0.0}, {1.0, 0.0}};
  const std::vector<Vec2> none;
  EXPECT_FALSE(vf::hasCusp(straight));
  EXPECT_FALSE(vf::hasCusp(right_angle));
  EXPECT_TRUE(vf::hasCusp(reversing));
  EXPECT_FALSE(vf::hasCusp(two));
  EXPECT_FALSE(vf::hasCusp(none));
}

// O14: NavFn-style tails (half-cell spacing toward the rounded goal cell, then
// smoothApproachToGoal's replace-or-append rule; 8 goal offsets of ±0.5 cell × 16 headings ×
// approach lengths 1.0, 1.0125, 1.025, 1.0375 m; res 0.05 and 0.10) give no cusp.
TEST(Conditioning, O14_NavFnTailsHaveNoCusp)
{
  const Vec2 cell(1.234, -0.567);
  std::vector<Vec2> decimated;
  int cases = 0;
  int cusps = 0;
  int hooks = 0;
  for (const double res : {0.05, 0.10}) {
    const double chord = vf::chordLength(res);
    for (int ox = -1; ox <= 1; ++ox) {
      for (int oy = -1; oy <= 1; ++oy) {
        if (ox == 0 && oy == 0) {
          continue;
        }
        const Vec2 goal = cell + 0.5 * res * Vec2(ox, oy);
        for (int k = 0; k < 16; ++k) {
          const Vec2 direction = vf::heading(vf::kTwoPi * k / 16.0);
          for (const double approach : {1.0, 1.0125, 1.025, 1.0375}) {
            const std::vector<Vec2> plan = navfnTail(cell - approach * direction, cell, goal, res);
            // The raw tail really reverses for some goals (the hook this rule must absorb).
            for (std::size_t i = 1; i + 1 < plan.size(); ++i) {
              hooks += (plan[i] - plan[i - 1]).dot(plan[i + 1] - plan[i]) < 0.0 ? 1 : 0;
            }
            vf::decimateChords(plan, chord, &decimated);
            cusps += vf::hasCusp(decimated) ? 1 : 0;
            cusps += cuspsOf(unoriented(plan), chord).empty() ? 0 : 1;
            ++cases;
          }
        }
      }
    }
  }
  EXPECT_EQ(cases, 2 * 8 * 16 * 4);
  EXPECT_GT(hooks, 0);
  EXPECT_EQ(cusps, 0);
}

// O14: a real reversal is flagged; NavFn's start-cell hook at the robot is not.
//
// The pop-before-append rule guarantees detection of an end-of-plan reversal longer than 2 l_c:
// the last forward vertex kept lies within l_c of the turn, so it stays more than l_c from the
// last pose and the final chord points backward. Between l_c and 2 l_c detection depends on where
// the chord grid falls. The 0.3 m reversal of O14 is therefore guaranteed at res 0.05
// (l_c = 0.10); at res 0.10 (l_c = 0.20) the guaranteed length is above 0.40 m.
TEST(Conditioning, O14_RealReversalIsFlaggedButStartHookIsNot)
{
  std::vector<Vec2> decimated;
  int cases = 0;
  for (const double res : {0.05, 0.10}) {
    const double chord = vf::chordLength(res);
    const double step = 0.5 * res;
    const double reversal_length = res < 0.075 ? 0.3 : 0.45;
    ASSERT_GT(reversal_length, 2.0 * chord);
    for (int k = 0; k < 16; ++k) {
      const Vec2 direction = vf::heading(vf::kTwoPi * k / 16.0 + 0.1);
      const Vec2 start(0.3, -0.2);
      for (const double approach : {1.0, 1.0125, 1.025, 1.0375}) {
        std::vector<Vec2> reversal;
        for (int j = 0; j * step <= approach + 1e-9; ++j) {
          reversal.push_back(start + (j * step) * direction);
        }
        const Vec2 turn = reversal.back();
        for (int j = 1; j * step <= reversal_length + 1e-9; ++j) {
          reversal.push_back(turn - (j * step) * direction);
        }
        vf::decimateChords(reversal, chord, &decimated);
        EXPECT_TRUE(vf::hasCusp(decimated)) << res << " " << k << " " << approach;
        EXPECT_EQ(cuspsOf(unoriented(reversal), chord).size(), 1U) << res << " " << k;

        // A reversal in the middle of a plan (it then continues backward) is flagged as well.
        for (int j = 1; j * step <= 1.0 + 1e-9; ++j) {
          reversal.push_back(reversal.back() - step * direction);
        }
        vf::decimateChords(reversal, chord, &decimated);
        EXPECT_TRUE(vf::hasCusp(decimated)) << res << " " << k << " " << approach;
        EXPECT_EQ(cuspsOf(unoriented(reversal), chord).size(), 1U) << res << " " << k;

        // The robot's cell centre lies half a cell behind the robot: the plan starts with a
        // sub-chord step backward, then runs forward.
        std::vector<Vec2> start_hook{start, start - 0.5 * res * direction};
        for (int j = 1; j * step <= approach + 1e-9; ++j) {
          start_hook.push_back(start + (j * step) * direction);
        }
        vf::decimateChords(start_hook, chord, &decimated);
        EXPECT_FALSE(vf::hasCusp(decimated)) << res << " " << k << " " << approach;
        EXPECT_TRUE(cuspsOf(unoriented(start_hook), chord).empty()) << res << " " << k;
        ++cases;
      }
    }
  }
  EXPECT_EQ(cases, 2 * 16 * 4);
}

// ---------------------------------------------------------------------------------------------
// findCusps (math_spec §2.8 [Design, 0.2.1])
// ---------------------------------------------------------------------------------------------

// NavigateThroughPoses corners of 100°..170° on NavFn, both ways round, 16 leg headings, the
// waypoint anywhere in its cell (NavFn's next leg starts at the nearest grid vertex: a hook of up
// to 0.71 cell), at res 0.05 and 0.10: no cusp, with NavFn's identity orientations (the waypoint
// along either leg), with Smac's along-path orientations, and with ±0.2 cell of noise.
TEST(Cusps, SharpForwardCornersAreNotCusps)
{
  std::mt19937_64 rng(21);
  std::uniform_real_distribution<double> noise(-0.2, 0.2);
  int cases = 0;
  int candidates = 0;
  int cusps = 0;
  int headings_wrong = 0;
  std::vector<Vec2> decimated;
  for (const double res : {0.05, 0.10}) {
    const double chord = vf::chordLength(res);
    for (int turn_deg = 100; turn_deg <= 170; turn_deg += 10) {
      for (const int side : {-1, 1}) {
        for (int k = 0; k < 16; ++k) {
          const double first = vf::kTwoPi * k / 16.0;
          const double second = first + side * turn_deg * vf::kPi / 180.0;
          for (int ox = -1; ox <= 1; ++ox) {
            for (int oy = -1; oy <= 1; ++oy) {
              const Vec2 corner = Vec2(1.0, -0.5) + res * Vec2(0.013 + 0.5 * ox, 0.007 + 0.5 * oy);
              const Vec2 start = corner - vf::heading(first);
              const Pose2 goal{Vec2(corner + vf::heading(second)), second};
              for (const double waypoint_yaw : {0.0, second}) {
                std::vector<Pose2> plan =
                  navfnThroughWaypoint(start, Pose2{corner, waypoint_yaw}, goal, res);
                std::vector<Vec2> points;
                for (const Pose2 & pose : plan) {
                  points.push_back(pose.p);
                }
                vf::decimateChords(points, chord, &decimated);
                candidates += vf::hasCusp(decimated) ? 1 : 0;
                cusps += cuspsOf(plan, chord).empty() ? 0 : 1;
                headings_wrong += vf::planHasHeadings(plan, chord) ? 1 : 0;

                std::vector<Pose2> noisy = plan;
                for (std::size_t i = 1; i + 1 < noisy.size(); ++i) {
                  if ((noisy[i].p - corner).norm() > res) {
                    noisy[i].p += res * Vec2(noise(rng), noise(rng));
                  }
                }
                cusps += cuspsOf(noisy, chord).empty() ? 0 : 1;

                orientAlongPath(&plan);
                cusps += cuspsOf(plan, chord).empty() ? 0 : 1;
                headings_wrong += vf::planHasHeadings(plan, chord) ? 0 : 1;
                cases += 3;
              }
            }
          }
        }
      }
    }
  }
  EXPECT_EQ(cases, 2 * 8 * 2 * 16 * 9 * 2 * 3);
  EXPECT_GT(candidates, cases / 6);  // the chord rule alone would have rejected most of them
  EXPECT_EQ(cusps, 0);
  EXPECT_EQ(headings_wrong, 0);
}

// Reeds-Shepp cusps as Smac Hybrid-A* and Lattice emit them (the vehicle yaw, opposite to the
// motion while reversing): forward 1 m on an arc of radius 0.4 or 1.0 m (or straight), then
// back 1 m or just over 2 l_c, any steering, 16 headings, the cusp moved by a cell either way:
// one cusp at the turning pose, and segmentPlan() drives forward then in reverse.
TEST(Cusps, ReedsSheppReversalsWithHeadingsAreCusps)
{
  int cases = 0;
  std::vector<std::size_t> work;
  std::vector<vf::PlanSegment> segments;
  for (const double res : {0.05, 0.10}) {
    const double chord = vf::chordLength(res);
    for (const double radius : {0.4, 1.0}) {
      for (int s1 = -1; s1 <= 1; ++s1) {
        for (int s2 = -1; s2 <= 1; ++s2) {
          for (int k = 0; k < 16; ++k) {
            for (int jitter = -1; jitter <= 1; ++jitter) {
              for (const double back : {1.0, 2.0 * chord + 0.5 * res}) {
                std::vector<Pose2> plan{Pose2{Vec2(0.3, -0.2), vf::kTwoPi * k / 16.0}};
                appendDrive(&plan, radius, 1, s1, 1.0 + jitter * res, 0.5 * res);
                const std::size_t turn = plan.size() - 1;
                appendDrive(&plan, radius, -1, s2, back, 0.5 * res);
                const std::vector<std::size_t> cusps = cuspsOf(plan, chord);
                ASSERT_EQ(cusps.size(), 1U) << res << " R " << radius << " " << s1 << s2 << " k "
                                            << k << " jitter " << jitter << " back " << back;
                EXPECT_LE(cusps[0] > turn ? cusps[0] - turn : turn - cusps[0], 1U);
                if (!(s1 == 0 && s2 == 0 && k == 0)) {  // all-identity: no headings, alternation
                  EXPECT_TRUE(vf::planHasHeadings(plan, chord)) << s1 << s2 << " k " << k;
                }
                vf::segmentPlan(plan, chord, 1, &work, &segments);
                ASSERT_EQ(segments.size(), 2U);
                EXPECT_EQ(segments[0].drive_sign, 1);
                EXPECT_EQ(segments[1].drive_sign, -1);
                ++cases;
              }
            }
          }
        }
      }
    }
  }
  EXPECT_EQ(cases, 2 * 2 * 3 * 3 * 16 * 3 * 2);
}

// Without headings a reversal is a cusp when the plan doubles back for more than 2 l_c (half a
// cell more, or 1 m): back along the same line or one cell beside it, at the end or in the middle
// of the plan. Shorter reversals and wider hairpins are followed forward.
TEST(Cusps, StraightReversalsWithoutHeadingsAreCusps)
{
  int cases = 0;
  for (const double res : {0.05, 0.10}) {
    const double chord = vf::chordLength(res);
    for (int k = 0; k < 16; ++k) {
      const Vec2 u = vf::heading(vf::kTwoPi * k / 16.0 + 0.1);
      const Vec2 side = vf::J(u);
      const Vec2 start(0.3, -0.2);
      const Vec2 turn = start + 1.0 * u;
      for (const double offset : {0.0, res}) {
        for (const double back : {2.0 * chord + 0.5 * res, 1.0}) {
          std::vector<Pose2> plan;
          appendHalfCells(&plan, start, turn, res);
          appendHalfCells(&plan, turn + offset * side, turn + offset * side - back * u, res);
          plan.push_back(Pose2{Vec2(turn + offset * side - back * u), 0.0});
          EXPECT_EQ(cuspsOf(plan, chord).size(), 1U) << res << " k " << k << " " << offset;
          EXPECT_FALSE(vf::planHasHeadings(plan, chord));
          // Then forward again (a three-point turn): two cusps.
          const Vec2 second_turn = plan.back().p;
          plan.pop_back();
          appendHalfCells(&plan, second_turn, second_turn + 1.0 * u, res);
          plan.push_back(Pose2{Vec2(second_turn + 1.0 * u), 0.0});
          EXPECT_EQ(cuspsOf(plan, chord).size(), 2U) << res << " k " << k << " " << offset;
          ++cases;
        }
      }
      // Shorter than 2 l_c: no cusp (followed forward with a deviation below 2 l_c).
      std::vector<Pose2> short_back;
      appendHalfCells(&short_back, start, turn, res);
      short_back.push_back(Pose2{turn, 0.0});
      short_back.push_back(Pose2{Vec2(turn - 1.5 * chord * u), 0.0});
      EXPECT_TRUE(cuspsOf(short_back, chord).empty()) << res << " k " << k;
      // A hairpin 1.5 l_c wide (three cells at res 0.05): two corners, not a reversal.
      std::vector<Pose2> hairpin;
      appendHalfCells(&hairpin, start, turn, res);
      appendHalfCells(&hairpin, turn, turn + 1.5 * chord * side, res);
      appendHalfCells(&hairpin, turn + 1.5 * chord * side, turn + 1.5 * chord * side - u, res);
      hairpin.push_back(Pose2{Vec2(turn + 1.5 * chord * side - u), 0.0});
      EXPECT_TRUE(cuspsOf(hairpin, chord).empty()) << res << " k " << k;
    }
  }
  EXPECT_EQ(cases, 2 * 16 * 2 * 2);
}

TEST(Conditioning, DespikeRemovesSubChordBacktracks)
{
  // A NavFn staircase (turns of 45° at most) is untouched; endpoints are kept.
  std::vector<Vec2> stairs{{0.0, 0.0}, {0.025, 0.0}, {0.05, 0.025}, {0.075, 0.025}, {0.1, 0.05}};
  std::vector<Vec2> out = stairs;
  vf::despike(&out, 0.05);
  EXPECT_EQ(out, stairs);
  // A NavigateThroughPoses hook (back 1.5 cm to the next leg's start) at a 90° corner goes; the
  // corner stays within the hook's length.
  std::vector<Vec2> hook{{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {0.99, -0.011}, {0.99, 0.5}};
  vf::despike(&hook, 0.05);
  ASSERT_EQ(hook.size(), 4U);
  EXPECT_TRUE(sameBits(hook[2], Vec2(0.99, -0.011)));
  EXPECT_TRUE(sameBits(hook.back(), Vec2(0.99, 0.5)));
  // A long reversal (a cusp, split into segments upstream) is kept.
  std::vector<Vec2> reversal{{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.0}};
  out = reversal;
  vf::despike(&out, 0.05);
  EXPECT_EQ(out, reversal);
  std::vector<Vec2> empty;
  vf::despike(&empty, 0.05);
  EXPECT_TRUE(empty.empty());
}

TEST(Cusps, HeadingsNeedAlignedOrientedRuns)
{
  const double chord = 0.1;
  std::vector<Pose2> plan;
  appendHalfCells(&plan, Vec2::Zero(), Vec2(1.0, 1.0), 0.05);
  plan.push_back(Pose2{Vec2(1.0, 1.0), 0.3});
  EXPECT_FALSE(vf::planHasHeadings(plan, chord));  // NavFn: identity, the goal yaw unused
  orientAlongPath(&plan);
  EXPECT_TRUE(vf::planHasHeadings(plan, chord));  // Smac 2D after its smoother
  for (Pose2 & pose : plan) {
    pose.yaw += vf::kPi;  // reversing all the way: still headings
  }
  EXPECT_TRUE(vf::planHasHeadings(plan, chord));
  for (Pose2 & pose : plan) {
    pose.yaw += 0.5 * vf::kPi;  // sideways (an omnidirectional lattice): not driving headings
  }
  EXPECT_FALSE(vf::planHasHeadings(plan, chord));
  // Straight along +x: aligned but all identity, so no headings (the configured sign decides).
  plan.clear();
  appendHalfCells(&plan, Vec2::Zero(), Vec2(1.0, 0.0), 0.05);
  plan.push_back(Pose2{Vec2(1.0, 0.0), 0.0});
  EXPECT_FALSE(vf::planHasHeadings(plan, chord));
  // An oriented run shorter than l_c (a waypoint pose) is not enough.
  plan[10].yaw = 1e-3;
  plan[11].yaw = 1e-3;
  EXPECT_FALSE(vf::planHasHeadings(plan, chord));
  for (std::size_t i = 10; i < 16; ++i) {
    plan[i].yaw = 1e-3;
  }
  EXPECT_TRUE(vf::planHasHeadings(plan, chord));
  EXPECT_FALSE(vf::planHasHeadings({}, chord));
  EXPECT_TRUE(cuspsOf({}, chord).empty());
  EXPECT_TRUE(cuspsOf({Pose2{}, Pose2{Vec2(1.0, 0.0), 0.0}}, chord).empty());
}

TEST(Conditioning, OnePoseWindow)
{
  const Pose2 last{Vec2(1.0, 0.0), vf::kHalfPi};
  const std::array<Vec2, 2> far = vf::onePoseWindow(Vec2(0.0, 0.0), last);
  EXPECT_TRUE(sameBits(far[0], Vec2(0.0, 0.0)));
  EXPECT_TRUE(sameBits(far[1], last.p));
  const std::array<Vec2, 2> near = vf::onePoseWindow(Vec2(1.0005, 0.0), last);
  EXPECT_TRUE(near[0].isApprox(Vec2(1.0, -0.01), 1e-15));
  EXPECT_TRUE(sameBits(near[1], last.p));
  EXPECT_NEAR((near[1] - near[0]).norm(), vf::kOnePoseOffset, 1e-15);
}

TEST(Conditioning, KappaPath)
{
  const std::vector<Vec2> square{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
  EXPECT_NEAR(vf::kappaPath(square, false), vf::kPi / 4.0, 1e-15);
  EXPECT_NEAR(vf::kappaPath(square, true), vf::kPi / 4.0, 1e-15);
  const std::vector<Vec2> uneven{{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.5}};
  EXPECT_NEAR(vf::kappaPath(uneven, false), vf::kHalfPi / 0.75, 1e-15);
  const std::vector<Vec2> reversal{{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.0}};
  EXPECT_NEAR(vf::kappaPath(reversal, false), vf::kPi / 0.75, 1e-15);
  const std::vector<Vec2> triangle{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
  EXPECT_NEAR(
    vf::kappaPath(triangle, false), 0.75 * vf::kPi / (0.5 * (1.0 + std::sqrt(2.0))), 1e-15);
  EXPECT_NEAR(
    vf::kappaPath(triangle, true), 0.75 * vf::kPi / (0.5 * (1.0 + std::sqrt(2.0))), 1e-15);
  const std::vector<Vec2> straight{{0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}};
  EXPECT_EQ(vf::kappaPath(straight, false), 0.0);
  const std::vector<Vec2> two{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_EQ(vf::kappaPath(two, true), 0.0);
  // Rays are excluded: an open polyline's end vertices do not count.
  const std::vector<Vec2> bend{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  EXPECT_NEAR(vf::kappaPath(bend, false), vf::kHalfPi, 1e-15);
  EXPECT_NEAR(vf::kappaPath(bend, true), 0.75 * vf::kPi / (0.5 * (1.0 + std::sqrt(2.0))), 1e-15);
}
