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

// PolylineCurve: pieces, ownership, wedge and hairpin tangents, and the WindowScan and Hinted
// searches (math_spec §2.2-§2.4; geometry oracles O1-O6, O8, O9 of §12.1).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

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
using vf::PieceKind;
using vf::PolylineCurve;
using vf::SearchPolicy;
using vf::Status;
using vf::Vec2;

namespace
{

PolylineCurve buildPolyline(
  const std::vector<Vec2> & points, bool closed, const SearchPolicy & policy)
{
  PolylineCurve curve(policy);
  EXPECT_EQ(curve.assign(points, closed), AssignStatus::kOk);
  return curve;
}

SearchPolicy hinted(double window = 1.0, double trigger = 0.5, double hysteresis = 0.10)
{
  return SearchPolicy::hinted(window, trigger, hysteresis);
}

SearchPolicy windowAll() { return SearchPolicy::windowScan(-1e9, 1e9); }

SearchPolicy pluginWindow() { return SearchPolicy::windowScan(-0.5, 0.7); }

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

EvalContext context(int direction = 1, const Vec2 & last_tangent = Vec2::Zero())
{
  EvalContext ctx;
  ctx.direction = direction;
  ctx.last_tangent = last_tangent;
  return ctx;
}

// Two draws in a fixed order (argument evaluation order is unspecified).
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

double rayDistance(const Vec2 & x, const Vec2 & anchor, const Vec2 & direction)
{
  const double tau = std::max(0.0, (x - anchor).dot(direction));
  return (x - (anchor + tau * direction)).norm();
}

// Independent reference: the distance from x to the borderless polyline.
double bruteForceDistance(const PolylineCurve & curve, const Vec2 & x)
{
  const auto points = curve.vertices();
  const std::size_t n = points.size();
  double best = vf::kInf;
  for (std::size_t k = 0; k < curve.segmentCount(); ++k) {
    best = std::min(best, segmentDistance(x, points[k], points[(k + 1) % n]));
  }
  if (!curve.isClosed()) {
    best = std::min(best, rayDistance(x, points[0], -curve.segmentTangent(0)));
    best = std::min(best, rayDistance(x, points[n - 1], curve.segmentTangent(n - 2)));
  }
  return best;
}

// Worst violations of the ClosestPoint guarantees (O1) over many results.
struct Violations
{
  double dot{0.0};
  double tangent_norm{0.0};
  double normal_norm{0.0};
  double residual{0.0};
  int invalid{0};
  int samples{0};
  int wedges{0};
  int rays{0};
  int hairpins{0};

  void add(const ClosestPoint & cp, const Vec2 & x)
  {
    ++samples;
    if (
      !cp.valid || !vf::isFinite(cp.x_star) || !std::isfinite(cp.s_star) || !std::isfinite(cp.D) ||
      !vf::isFinite(cp.T) || !vf::isFinite(cp.n)) {
      ++invalid;
      return;
    }
    dot = std::max(dot, std::abs(cp.n.dot(cp.T)));
    tangent_norm = std::max(tangent_norm, std::abs(cp.T.norm() - 1.0));
    if (cp.D >= vf::kEpsD) {
      normal_norm = std::max(normal_norm, std::abs(cp.n.norm() - 1.0));
      residual = std::max(residual, (x - cp.x_star - cp.D * cp.n).norm() / std::max(1.0, x.norm()));
    } else if (cp.n != Vec2::Zero()) {
      normal_norm = 1.0;
    }
    wedges += cp.in_wedge ? 1 : 0;
    rays += cp.on_ray ? 1 : 0;
    hairpins += cp.hairpin ? 1 : 0;
  }
};

// Leg 1 from (-leg, 0) to (0, 0), a semicircle of diameter `width` about (0, width/2) to
// (0, width), and leg 2 back to (-leg, width); legs sampled every `spacing` m.
std::vector<Vec2> uTurn(double width, double leg, int arc_segments, double spacing = 0.05)
{
  std::vector<Vec2> points;
  const int leg_pieces = static_cast<int>(std::lround(leg / spacing));
  for (int i = 0; i < leg_pieces; ++i) {
    points.emplace_back(-leg + leg * i / leg_pieces, 0.0);
  }
  const double r = 0.5 * width;
  for (int i = 0; i <= arc_segments; ++i) {
    const double phi = -vf::kHalfPi + vf::kPi * i / arc_segments;
    points.emplace_back(r * std::cos(phi), r + r * std::sin(phi));
  }
  for (int i = 1; i <= leg_pieces; ++i) {
    points.emplace_back(-leg * i / leg_pieces, width);
  }
  return points;
}

std::vector<Vec2> straightLine(double length, double spacing)
{
  std::vector<Vec2> points;
  const int pieces = static_cast<int>(std::lround(length / spacing));
  for (int i = 0; i <= pieces; ++i) {
    points.emplace_back(length * i / pieces, 0.0);
  }
  return points;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// assign() and accessors
// ---------------------------------------------------------------------------------------------

TEST(PolylineCurve, AssignBuildsPiecesAndStations)
{
  PolylineCurve open;
  EXPECT_TRUE(open.empty());
  EXPECT_FALSE(open.closestPoint(Vec2(0.0, 0.0), context(), CurveHint{}).valid);
  const std::vector<Vec2> points{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
  ASSERT_EQ(open.assign(points, false), AssignStatus::kOk);
  EXPECT_EQ(open.size(), 3u);
  EXPECT_EQ(open.segmentCount(), 2u);
  EXPECT_FALSE(open.isClosed());
  EXPECT_FALSE(open.isTimeVarying());
  EXPECT_DOUBLE_EQ(open.length(0.0), 7.0);
  EXPECT_DOUBLE_EQ(open.station(0), 0.0);
  EXPECT_DOUBLE_EQ(open.station(1), 3.0);
  EXPECT_DOUBLE_EQ(open.station(2), 7.0);
  EXPECT_EQ(open.segmentTangent(0), Vec2(1.0, 0.0));
  EXPECT_EQ(open.segmentTangent(1), Vec2(0.0, 1.0));
  EXPECT_DOUBLE_EQ(open.segmentLength(1), 4.0);
  EXPECT_EQ(open.spec(), nullptr);
  EXPECT_EQ(open.searchPolicy().kind, SearchPolicy::Kind::kHinted);
  EXPECT_DOUBLE_EQ(open.searchPolicy().window, vf::CurveOptions{}.search_window);
  const ClosestPoint cp = open.closestPoint(Vec2(1.0, 1.0), context(), CurveHint{});
  EXPECT_EQ(open.velocity(cp, 0.0), Vec2::Zero());

  PolylineCurve closed;
  ASSERT_EQ(closed.assign(points, true), AssignStatus::kOk);
  EXPECT_TRUE(closed.isClosed());
  EXPECT_EQ(closed.segmentCount(), 3u);
  EXPECT_DOUBLE_EQ(closed.length(0.0), 12.0);
  EXPECT_DOUBLE_EQ(closed.station(3), 12.0);
  EXPECT_TRUE(closed.segmentTangent(2).isApprox(Vec2(-0.6, -0.8), 1e-15));

  closed.clear();
  EXPECT_TRUE(closed.empty());
  EXPECT_EQ(closed.segmentCount(), 0u);
  EXPECT_DOUBLE_EQ(closed.length(0.0), 0.0);
}

TEST(PolylineCurve, AssignRejectsTooFewAndNonFinite)
{
  PolylineCurve curve;
  const std::vector<Vec2> none;
  const std::vector<Vec2> one{{1.0, 2.0}};
  const std::vector<Vec2> two{{0.0, 0.0}, {1.0, 0.0}};
  const std::vector<Vec2> duplicates{{1.0, 1.0}, {1.0, 1.0 + 1e-12}, {1.0, 1.0}};
  EXPECT_EQ(curve.assign(none, false), AssignStatus::kTooFew);
  EXPECT_EQ(curve.assign(one, false), AssignStatus::kTooFew);
  EXPECT_EQ(curve.assign(two, true), AssignStatus::kTooFew);
  EXPECT_EQ(curve.assign(duplicates, false), AssignStatus::kTooFew);
  EXPECT_TRUE(curve.empty());

  std::vector<Vec2> bad{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  ASSERT_EQ(curve.assign(bad, false), AssignStatus::kOk);
  bad[1].y() = vf::kNaN;
  EXPECT_EQ(curve.assign(bad, false), AssignStatus::kNonFinite);
  EXPECT_TRUE(curve.empty());
  bad[1].y() = vf::kInf;
  EXPECT_EQ(curve.assign(bad, true), AssignStatus::kNonFinite);
  EXPECT_TRUE(curve.empty());
  EXPECT_FALSE(curve.closestPoint(Vec2(0.0, 1.0), context(), CurveHint{}).valid);

  // Coordinates whose segment length overflows are reported as non-finite geometry.
  const std::vector<Vec2> huge{{-1e308, 0.0}, {1e308, 0.0}};
  EXPECT_EQ(curve.assign(huge, false), AssignStatus::kNonFinite);
  EXPECT_TRUE(curve.empty());
}

TEST(PolylineCurve, AssignDropsSubEpsilonVerticesAndClosingDuplicates)
{
  PolylineCurve curve;
  const std::vector<Vec2> points{{0.0, 0.0}, {0.0, 5e-10}, {1.0, 0.0}, {1.0, 1.0}, {1e-10, 1e-10}};
  ASSERT_EQ(curve.assign(points, true), AssignStatus::kOk);
  ASSERT_EQ(curve.size(), 3u);  // (0, 5e-10) and the closing duplicate are dropped
  EXPECT_EQ(curve.vertices()[0], Vec2(0.0, 0.0));
  EXPECT_EQ(curve.vertices()[2], Vec2(1.0, 1.0));
  ASSERT_EQ(curve.assign(points, false), AssignStatus::kOk);
  EXPECT_EQ(curve.size(), 4u);  // open: the last vertex is not a closing duplicate

  // A non-finite query point gives an invalid result, never a crash.
  EXPECT_FALSE(curve.closestPoint(Vec2(vf::kNaN, 0.0), context(), CurveHint{}).valid);
}

TEST(PolylineCurve, ReassignWithinReservedCapacityKeepsStorage)
{
  PolylineCurve curve(pluginWindow());
  curve.reserve(64);
  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> coord(-3.0, 3.0);
  std::vector<Vec2> points;
  points.reserve(64);
  const Vec2 * storage = nullptr;
  for (int cycle = 0; cycle < 50; ++cycle) {
    points.clear();
    const int count = 2 + cycle % 60;
    for (int i = 0; i < count; ++i) {
      points.push_back(randomPoint(rng, coord));
    }
    ASSERT_EQ(curve.assign(points, false), AssignStatus::kOk);
    if (storage == nullptr) {
      storage = curve.vertices().data();
    }
    EXPECT_EQ(curve.vertices().data(), storage) << "assign() reallocated in cycle " << cycle;
  }
  // assign() may view its own vertices.
  const std::size_t before = curve.size();
  ASSERT_EQ(curve.assign(curve.vertices(), false), AssignStatus::kOk);
  EXPECT_EQ(curve.size(), before);
}

// ---------------------------------------------------------------------------------------------
// Oracles of math_spec §12.1
// ---------------------------------------------------------------------------------------------

// O1: |n^T T| <= 1e-12 and |‖T‖ - 1| <= 1e-12 for random points around random polylines,
// including wedges, exact hairpins, rays and closed curves, under every search mode.
TEST(PolylineCurve, O1_OrthogonalityOnRandomPolylines)
{
  std::mt19937_64 rng(20261013);
  std::uniform_real_distribution<double> coord(-5.0, 5.0);
  std::uniform_real_distribution<double> query(-8.0, 8.0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_int_distribution<int> vertex_count(2, 12);
  Violations violations;
  double worst_global_error = 0.0;
  for (int trial = 0; trial < 700; ++trial) {
    const int n = vertex_count(rng);
    const bool closed = n >= 3 && unit(rng) < 0.4;
    std::vector<Vec2> points;
    for (int i = 0; i < n; ++i) {
      if (i >= 2 && unit(rng) < 0.25) {
        // An exact reversal: the next vertex lies back along the previous segment.
        const Vec2 back = points[i - 2] - points[i - 1];
        points.push_back(points[i - 1] + (0.3 + unit(rng)) * back);
      } else {
        points.push_back(randomPoint(rng, coord));
      }
    }
    PolylineCurve curve;
    if (curve.assign(points, closed) != AssignStatus::kOk) {
      continue;
    }
    const double length = curve.length(0.0);
    for (int q = 0; q < 20; ++q) {
      const Vec2 x = randomPoint(rng, query);
      const int direction = unit(rng) < 0.5 ? 1 : -1;
      Vec2 last_tangent = Vec2::Zero();
      if (unit(rng) < 0.5) {
        last_tangent = vf::heading(7.0 * unit(rng));
      }
      const EvalContext ctx = context(direction, last_tangent);

      curve.setSearchPolicy(hinted());
      const ClosestPoint global = curve.closestPoint(x, ctx, CurveHint{});
      violations.add(global, x);
      worst_global_error =
        std::max(worst_global_error, std::abs(global.D - bruteForceDistance(curve, x)));

      const ClosestPoint local =
        curve.closestPoint(x, ctx, stationHint(-1.0 + (length + 2.0) * unit(rng)));
      violations.add(local, x);

      const double s_min = -1.0 + (length + 2.0) * unit(rng);
      const double width = 1.5 * unit(rng);
      curve.setSearchPolicy(SearchPolicy::windowScan(s_min, s_min + width));
      CurveHint hint;
      if (unit(rng) < 0.5) {
        hint = positionHint(randomPoint(rng, query));
      }
      violations.add(curve.closestPoint(x, ctx, hint), x);
    }
  }
  EXPECT_GE(violations.samples, 10000);
  EXPECT_EQ(violations.invalid, 0);
  EXPECT_LE(violations.dot, 1e-12);
  EXPECT_LE(violations.tangent_norm, 1e-12);
  EXPECT_LE(violations.normal_norm, 1e-12);
  EXPECT_LE(violations.residual, 1e-12);
  EXPECT_LE(worst_global_error, vf::kTieTolerance + 1e-12);
  EXPECT_GE(violations.wedges, 1000);
  EXPECT_GE(violations.rays, 1000);
  EXPECT_GE(violations.hairpins, 20);
}

// O2: x exactly at P0, at an interior vertex and at W_back, and the one-pose polyline [p, g]
// evaluated at p: ‖T‖ = 1, n = 0, T the vertex tangent of math_spec §2.3.
TEST(PolylineCurve, O2_DegeneratePoints)
{
  const std::vector<Vec2> points{{0.0, 0.0}, {2.0, 0.0}, {3.0, 1.0}, {3.0, 3.0}};
  const Vec2 t0(1.0, 0.0);
  const Vec2 t1 = Vec2(1.0, 1.0).normalized();
  const Vec2 t2(0.0, 1.0);
  for (const SearchPolicy & policy : {hinted(), windowAll(), pluginWindow()}) {
    const PolylineCurve curve = buildPolyline(points, false, policy);
    const double length = curve.length(0.0);
    for (const CurveHint & hint :
         {CurveHint{}, stationHint(0.0), stationHint(length), positionHint(points[1])}) {
      for (const int direction : {1, -1}) {
        const ClosestPoint at_start = curve.closestPoint(points[0], context(direction), hint);
        ASSERT_TRUE(at_start.valid);
        EXPECT_EQ(at_start.n, Vec2::Zero());
        EXPECT_NEAR(at_start.T.norm(), 1.0, 1e-12);
        EXPECT_TRUE(at_start.T.isApprox(t0, 1e-15));
        EXPECT_EQ(at_start.s_star, 0.0);
        EXPECT_EQ(at_start.x_star, points[0]);
        EXPECT_EQ(at_start.D, 0.0);
        EXPECT_EQ(at_start.piece, PieceKind::kVertex);
        EXPECT_FALSE(at_start.on_ray);

        const ClosestPoint at_vertex = curve.closestPoint(points[1], context(direction), hint);
        ASSERT_TRUE(at_vertex.valid);
        EXPECT_EQ(at_vertex.n, Vec2::Zero());
        EXPECT_NEAR(at_vertex.T.norm(), 1.0, 1e-12);
        EXPECT_TRUE(at_vertex.T.isApprox((t0 + t1).normalized(), 1e-15));
        EXPECT_EQ(at_vertex.s_star, curve.station(1));
        EXPECT_EQ(at_vertex.piece, PieceKind::kVertex);
        EXPECT_EQ(at_vertex.segment, 1);
        EXPECT_FALSE(at_vertex.in_wedge);

        const ClosestPoint at_back = curve.closestPoint(points[3], context(direction), hint);
        ASSERT_TRUE(at_back.valid);
        EXPECT_EQ(at_back.n, Vec2::Zero());
        EXPECT_NEAR(at_back.T.norm(), 1.0, 1e-12);
        EXPECT_TRUE(at_back.T.isApprox(t2, 1e-15));
        EXPECT_EQ(at_back.s_star, length);
        EXPECT_EQ(at_back.x_star, points[3]);
        EXPECT_FALSE(at_back.on_ray);
      }
    }
  }

  // The one-pose polyline [p, g] evaluated at p.
  const Vec2 p(1.5, -0.5);
  const Vec2 g = p + 0.3 * vf::heading(0.7);
  for (const SearchPolicy & policy : {hinted(), pluginWindow()}) {
    const PolylineCurve one_pose = buildPolyline({p, g}, false, policy);
    const ClosestPoint cp = one_pose.closestPoint(p, context(), CurveHint{});
    ASSERT_TRUE(cp.valid);
    EXPECT_EQ(cp.n, Vec2::Zero());
    EXPECT_NEAR(cp.T.norm(), 1.0, 1e-12);
    EXPECT_TRUE(cp.T.isApprox(vf::heading(0.7), 1e-12));
    EXPECT_EQ(cp.s_star, 0.0);
  }

  // A closed polygon vertex and a hairpin apex (T = t_v there).
  const PolylineCurve square =
    buildPolyline({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, true, hinted());
  const ClosestPoint corner = square.closestPoint(Vec2(0.0, 0.0), context(), CurveHint{});
  EXPECT_EQ(corner.n, Vec2::Zero());
  EXPECT_TRUE(corner.T.isApprox(Vec2(1.0, -1.0).normalized(), 1e-15));
  EXPECT_EQ(corner.s_star, 0.0);
  const PolylineCurve hairpin =
    buildPolyline({{-1.0, 0.0}, {0.0, 0.0}, {-1.0, 0.0}}, false, hinted());
  const ClosestPoint apex = hairpin.closestPoint(Vec2(0.0, 0.0), context(), CurveHint{});
  EXPECT_EQ(apex.n, Vec2::Zero());
  EXPECT_EQ(apex.T, Vec2(-1.0, 0.0));
  EXPECT_FALSE(apex.hairpin);
  EXPECT_EQ(apex.status_bits & Status::kHairpin, 0u);
}

// O3: 179° and 180° turns. Everywhere in the wedge n^T T = 0 and ‖T‖ = 1 (the field's
// ‖Phi‖ = v_r follows and is tested with the field); STATUS_HAIRPIN only at 180°.
TEST(PolylineCurve, O3_Hairpins)
{
  std::mt19937_64 rng(179);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (const double turn_deg : {179.0, 179.99, 180.0}) {
    const double turn = turn_deg * vf::kPi / 180.0;
    const Vec2 t1(1.0, 0.0);
    const Vec2 t2 = turn_deg == 180.0 ? Vec2(-1.0, 0.0) : vf::heading(turn);
    const std::vector<Vec2> points{{-2.0, 0.0}, {0.0, 0.0}, Vec2(2.0 * t2)};
    const bool expect_hairpin = (t1 + t2).norm() < vf::kEpsHp;
    EXPECT_EQ(expect_hairpin, turn_deg != 179.0);
    const Vec2 bisector = (t1 - t2).normalized();
    for (const SearchPolicy & policy : {hinted(), windowAll()}) {
      const PolylineCurve curve = buildPolyline(points, false, policy);
      double worst_dot = 0.0;
      double worst_norm = 0.0;
      int wrong = 0;
      for (int i = 0; i < 4000; ++i) {
        const double alpha = (unit(rng) - 0.5) * 0.999 * turn;
        const double distance = 0.02 + 1.5 * unit(rng);
        const Vec2 x = distance * vf::rotate(alpha, bisector);
        Vec2 last = Vec2::Zero();
        if (i % 3 == 1) {
          last = t1;
        } else if (i % 3 == 2) {
          last = vf::heading(6.0 * unit(rng));
        }
        const ClosestPoint cp = curve.closestPoint(x, context(1, last), CurveHint{});
        worst_dot = std::max(worst_dot, std::abs(cp.n.dot(cp.T)));
        worst_norm = std::max(worst_norm, std::abs(cp.T.norm() - 1.0));
        const bool flagged = (cp.status_bits & Status::kHairpin) != 0u;
        if (
          !cp.valid || cp.piece != PieceKind::kVertex || !cp.in_wedge ||
          cp.hairpin != expect_hairpin || flagged != expect_hairpin) {
          ++wrong;
        }
        if (expect_hairpin && last == Vec2::Zero()) {
          EXPECT_EQ(cp.T, vf::J(cp.n));  // no T_last: +Jn
        }
      }
      EXPECT_EQ(wrong, 0) << turn_deg;
      EXPECT_LE(worst_dot, 1e-12) << turn_deg;
      EXPECT_LE(worst_norm, 1e-12) << turn_deg;
    }
  }
  // 180°: outside the wedge the segments own x, with no hairpin status.
  const PolylineCurve curve =
    buildPolyline({{-2.0, 0.0}, {0.0, 0.0}, {-2.0, 0.0}}, false, hinted());
  const ClosestPoint beside = curve.closestPoint(Vec2(-1.0, 0.3), context(), CurveHint{});
  EXPECT_EQ(beside.piece, PieceKind::kSegment);
  EXPECT_FALSE(beside.hairpin);
  EXPECT_EQ(beside.status_bits, Status::kNone);
  // With T_last the hairpin tangent is unit((I - nn^T) T_last).
  const ClosestPoint ahead =
    curve.closestPoint(Vec2(0.5, 0.2), context(1, Vec2(0.0, -1.0)), CurveHint{});
  const Vec2 projected = Vec2(0.0, -1.0) - ahead.n * ahead.n.dot(Vec2(0.0, -1.0));
  EXPECT_TRUE(ahead.T.isApprox(projected.normalized(), 1e-15));
}

// O4: on random strict wedges sgn((Jn)^T (t_{v-1} + t_v)) = sgn(t_{v-1} x t_v) exactly, and T is
// continuous across the wedge boundaries (1e-7 m either side: ‖ΔT‖ <= 1e-6).
TEST(PolylineCurve, O4_WedgeIdentityAndContinuity)
{
  std::mt19937_64 rng(4);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int mismatches = 0;
  int not_wedge = 0;
  double worst_jump = 0.0;
  int boundary_failures = 0;
  for (int trial = 0; trial < 20000; ++trial) {
    const double magnitude = 0.01 + (vf::kPi - 0.02) * unit(rng);
    const double turn = unit(rng) < 0.5 ? magnitude : -magnitude;
    const Vec2 dir_in = vf::heading(vf::kTwoPi * unit(rng));
    const Vec2 dir_out = vf::rotate(turn, dir_in);
    std::uniform_real_distribution<double> box(-3.0, 3.0);
    const Vec2 vertex = randomPoint(rng, box);
    const double length_in = 3.0 + 3.0 * unit(rng);
    const double length_out = 3.0 + 3.0 * unit(rng);
    const std::vector<Vec2> points{
      vertex - length_in * dir_in, vertex, vertex + length_out * dir_out};
    const PolylineCurve curve = buildPolyline(points, false, hinted());
    const Vec2 t1 = curve.segmentTangent(0);
    const Vec2 t2 = curve.segmentTangent(1);
    const double side = vf::sgn(vf::cross2(t1, t2));

    // Identity inside the strict wedge.
    const Vec2 bisector = (t1 - t2).normalized();
    const double alpha = (unit(rng) - 0.5) * 0.98 * std::abs(turn);
    const double radius = 0.05 + 2.0 * unit(rng);
    const Vec2 x = vertex + radius * vf::rotate(alpha, bisector);
    const ClosestPoint cp = curve.closestPoint(x, context(), CurveHint{});
    if (!cp.in_wedge || cp.piece != PieceKind::kVertex) {
      ++not_wedge;
    } else if (cp.T != side * vf::J(cp.n)) {
      ++mismatches;
    }

    // Continuity at both wedge boundaries.
    if (trial % 10 == 0) {
      const double distance = 0.2 + 1.8 * unit(rng);
      const Vec2 normal_in = -side * vf::J(t1);
      const Vec2 on_in = vertex + distance * normal_in;
      const ClosestPoint a_in = curve.closestPoint(on_in + 1e-7 * t1, context(), CurveHint{});
      const ClosestPoint a_out = curve.closestPoint(on_in - 1e-7 * t1, context(), CurveHint{});
      const Vec2 normal_out = -side * vf::J(t2);
      const Vec2 on_out = vertex + distance * normal_out;
      const ClosestPoint b_in = curve.closestPoint(on_out - 1e-7 * t2, context(), CurveHint{});
      const ClosestPoint b_out = curve.closestPoint(on_out + 1e-7 * t2, context(), CurveHint{});
      if (
        !a_in.in_wedge || a_out.piece != PieceKind::kSegment || a_out.segment != 0 ||
        !b_in.in_wedge || b_out.piece != PieceKind::kSegment || b_out.segment != 1) {
        ++boundary_failures;
      }
      worst_jump = std::max(worst_jump, (a_in.T - a_out.T).norm());
      worst_jump = std::max(worst_jump, (b_in.T - b_out.T).norm());
    }
  }
  EXPECT_EQ(not_wedge, 0);
  EXPECT_EQ(mismatches, 0);
  EXPECT_EQ(boundary_failures, 0);
  EXPECT_LE(worst_jump, 1e-6);
}

// O5: semi-infinite rays. 100 m beyond the goal and 100 m before P0 the foot is on the ray (not
// clamped); sigma = -100 m and the arc is -100 m (1e-9).
TEST(PolylineCurve, O5_SemiInfiniteRays)
{
  const std::vector<Vec2> points{{0.0, 0.0}, {1.0, 0.0}, {2.0, 1.0}, {3.0, 1.0}, {4.0, 1.5}};
  for (const SearchPolicy & policy : {hinted(), pluginWindow(), windowAll()}) {
    const PolylineCurve curve = buildPolyline(points, false, policy);
    const double length = curve.length(0.0);
    const Vec2 t_end = curve.segmentTangent(3);
    const Vec2 t_start = curve.segmentTangent(0);
    for (const CurveHint & hint : {CurveHint{}, stationHint(length), positionHint(points[4])}) {
      const Vec2 beyond = points[4] + 100.0 * t_end + 0.3 * vf::J(t_end);
      const ClosestPoint ahead = curve.closestPoint(beyond, context(), hint);
      ASSERT_TRUE(ahead.valid);
      EXPECT_TRUE(ahead.on_ray);
      EXPECT_EQ(ahead.piece, PieceKind::kForwardRay);
      EXPECT_EQ(ahead.segment, 4);
      EXPECT_NEAR(length - ahead.s_star, -100.0, 1e-9);  // sigma
      EXPECT_NEAR(vf::remainingToEnd(curve, ahead, 0.0), 0.0, 0.0);
      EXPECT_LE((ahead.x_star - (points[4] + 100.0 * t_end)).norm(), 1e-9);
      EXPECT_NEAR(ahead.D, 0.3, 1e-9);
      EXPECT_EQ(ahead.T, t_end);
      EXPECT_LE(std::abs(ahead.n.dot(ahead.T)), 1e-12);
    }
    for (const CurveHint & hint : {CurveHint{}, stationHint(0.0), positionHint(points[0])}) {
      const Vec2 before = points[0] - 100.0 * t_start - 0.3 * vf::J(t_start);
      const ClosestPoint behind = curve.closestPoint(before, context(), hint);
      ASSERT_TRUE(behind.valid);
      EXPECT_TRUE(behind.on_ray);
      EXPECT_EQ(behind.piece, PieceKind::kBackwardRay);
      EXPECT_EQ(behind.segment, -1);
      EXPECT_NEAR(behind.s_star, -100.0, 1e-9);
      EXPECT_LE((behind.x_star - (points[0] - 100.0 * t_start)).norm(), 1e-9);
      EXPECT_NEAR(behind.D, 0.3, 1e-9);
      EXPECT_EQ(behind.T, t_start);
    }
  }
}

// O6: inner corners of 60, 90, 120 and 150 degrees, D = 0.5 on the inner bisector: the two feet
// differ in arc length by 2 D tan(theta/2) (1e-12); the tie goes to the hint; 1e-6 m past the
// bisector the outgoing segment wins despite a hint on the incoming one.
TEST(PolylineCurve, O6_InnerCornerJump)
{
  const double D = 0.5;
  for (const double turn_deg : {60.0, 90.0, 120.0, 150.0}) {
    const double theta = turn_deg * vf::kPi / 180.0;
    const Vec2 t_out = vf::heading(theta);
    const std::vector<Vec2> points{{-5.0, 0.0}, {0.0, 0.0}, Vec2(5.0 * t_out)};
    const double half_tan = std::tan(0.5 * theta);
    const Vec2 x(-D * half_tan, D);  // distance D from both segments (inner side)
    const Vec2 foot_in(-D * half_tan, 0.0);
    const Vec2 foot_out = D * half_tan * t_out;
    const double s_in = 5.0 - D * half_tan;
    const double s_out = 5.0 + D * half_tan;
    const Vec2 past = x + 1e-6 * (Vec2(1.0, 0.0) + t_out).normalized();

    struct Mode
    {
      SearchPolicy policy;
      CurveHint hint_in;
      CurveHint hint_out;
    };
    const Mode modes[] = {
      {hinted(20.0), stationHint(s_in), stationHint(s_out)},
      {windowAll(), positionHint(foot_in), positionHint(foot_out)},
    };
    for (const Mode & mode : modes) {
      const PolylineCurve curve = buildPolyline(points, false, mode.policy);
      const ClosestPoint incoming = curve.closestPoint(x, context(), mode.hint_in);
      const ClosestPoint outgoing = curve.closestPoint(x, context(), mode.hint_out);
      EXPECT_EQ(incoming.segment, 0) << turn_deg;
      EXPECT_EQ(outgoing.segment, 1) << turn_deg;
      EXPECT_NEAR(outgoing.s_star - incoming.s_star, 2.0 * D * half_tan, 1e-12) << turn_deg;
      EXPECT_NEAR(incoming.s_star, s_in, 1e-12);
      EXPECT_NEAR(outgoing.s_star, s_out, 1e-12);
      EXPECT_NEAR(incoming.D, D, 1e-12);
      EXPECT_NEAR(outgoing.D, D, 1e-12);

      const ClosestPoint moved = curve.closestPoint(past, context(), mode.hint_in);
      EXPECT_EQ(moved.segment, 1) << turn_deg;
      EXPECT_NEAR(moved.s_star, s_out, 1e-5) << turn_deg;
    }
  }
}

// O8: open polyline whose last point equals its first, no hint, robot at P0: s* = 0 for
// dir = +1 and s* = L for dir = -1.
TEST(PolylineCurve, O8_TieAtStart)
{
  const std::vector<Vec2> points{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}};
  for (const SearchPolicy & policy : {hinted(), windowAll(), pluginWindow()}) {
    const PolylineCurve curve = buildPolyline(points, false, policy);
    ASSERT_EQ(curve.size(), 5u);
    const double length = curve.length(0.0);
    ASSERT_DOUBLE_EQ(length, 4.0);
    const ClosestPoint forward = curve.closestPoint(points[0], context(1), CurveHint{});
    EXPECT_EQ(forward.s_star, 0.0);
    EXPECT_EQ(forward.T, Vec2(1.0, 0.0));
    EXPECT_EQ(forward.n, Vec2::Zero());
    const ClosestPoint backward = curve.closestPoint(points[0], context(-1), CurveHint{});
    if (policy.kind == SearchPolicy::Kind::kWindowScan && policy.s_max < length) {
      // The plugin window [-0.5, 0.7] never reaches the end: no tie to resolve.
      EXPECT_EQ(backward.s_star, 0.0);
    } else {
      EXPECT_EQ(backward.s_star, length);
      EXPECT_EQ(backward.T, Vec2(0.0, -1.0));
      EXPECT_EQ(backward.n, Vec2::Zero());
    }
  }
}

// O9: the detour test on a U-turn with legs w_u = 0.8 m apart (follower Hinted search).
TEST(PolylineCurve, O9_DetourTestOnUTurn)
{
  const double w_u = 0.8;
  const double leg = 4.0;
  const int arc_segments = 200;
  const std::vector<Vec2> points = uTurn(w_u, leg, arc_segments);
  const PolylineCurve curve = buildPolyline(points, false, hinted());
  const std::size_t arc_start = static_cast<std::size_t>(std::lround(leg / 0.05));
  const double s_apex = curve.station(arc_start);
  const double arc_length = curve.station(arc_start + arc_segments) - s_apex;
  ASSERT_NEAR(s_apex, leg, 1e-12);
  ASSERT_NEAR(arc_length, vf::kPi * 0.5 * w_u, 2e-5);

  // 2.5 w_u before the apex, 0.45 m off leg 1 toward leg 2: s* stays on leg 1.
  {
    const ClosestPoint cp =
      curve.closestPoint(Vec2(-2.5 * w_u, 0.45), context(), stationHint(s_apex - 2.5 * w_u));
    EXPECT_EQ(cp.x_star.y(), 0.0);
    EXPECT_NEAR(cp.s_star, s_apex - 2.5 * w_u, 1e-12);
    EXPECT_EQ(cp.status_bits & Status::kAmbiguousClosest, 0u);
  }
  // Same station 0.6 m off leg 1: the global stage runs (D_loc > D_trig) and passes the
  // hysteresis, but the detour test rejects the jump (arc 5.26 > 4.0).
  {
    const ClosestPoint cp =
      curve.closestPoint(Vec2(-2.5 * w_u, 0.6), context(), stationHint(s_apex - 2.5 * w_u));
    EXPECT_EQ(cp.x_star.y(), 0.0);
    EXPECT_NEAR(cp.D, 0.6, 1e-12);
    EXPECT_EQ(cp.status_bits & Status::kAmbiguousClosest, 0u);
  }
  // 0.5 w_u before the apex, more than w_u/2 + h_g off leg 1: the jump is adopted and flagged.
  {
    const ClosestPoint cp =
      curve.closestPoint(Vec2(-0.5 * w_u, 0.55), context(), stationHint(s_apex - 0.5 * w_u));
    EXPECT_NEAR(cp.x_star.y(), w_u, 1e-12);
    EXPECT_NEAR(cp.s_star, s_apex + arc_length + 0.5 * w_u, 1e-9);
    EXPECT_NE(cp.status_bits & Status::kAmbiguousClosest, 0u);
  }
  // Feet at the same station x before the apex (D_loc + D_glob = w_u): the detour inequality
  // alone decides, and it holds iff x <= (5 w_u - arc)/2 = (5 - pi/2) w_u/2 for a true
  // semicircle.
  const double x_limit = 0.5 * (5.0 * w_u - arc_length);
  EXPECT_NEAR(x_limit, (5.0 - 0.5 * vf::kPi) * 0.5 * w_u, 1e-5);
  for (const double offset : {-1e-3, 1e-3}) {
    const double x_station = x_limit + offset;
    const ClosestPoint cp =
      curve.closestPoint(Vec2(-x_station, 0.6), context(), stationHint(s_apex - x_station));
    const bool adopted = (cp.status_bits & Status::kAmbiguousClosest) != 0u;
    EXPECT_EQ(adopted, offset < 0.0) << "x = " << x_station;
    EXPECT_NEAR(cp.x_star.y(), adopted ? w_u : 0.0, 1e-12);
  }
}

// ---------------------------------------------------------------------------------------------
// Search policies
// ---------------------------------------------------------------------------------------------

TEST(PolylineSearch, WindowScanExtendsAtWindowEdges)
{
  const PolylineCurve line = buildPolyline(straightLine(10.0, 0.05), false, pluginWindow());
  const ClosestPoint ahead = line.closestPoint(Vec2(3.0, 0.2), context(), CurveHint{});
  EXPECT_NEAR(ahead.s_star, 3.0, 1e-12);
  EXPECT_NEAR(ahead.D, 0.2, 1e-12);
  EXPECT_EQ(ahead.piece, PieceKind::kSegment);
  EXPECT_EQ(ahead.n, Vec2(0.0, 1.0));
  const ClosestPoint past_goal = line.closestPoint(Vec2(12.0, -0.5), context(), CurveHint{});
  EXPECT_EQ(past_goal.piece, PieceKind::kForwardRay);
  EXPECT_NEAR(past_goal.s_star, 12.0, 1e-12);
  const ClosestPoint behind = line.closestPoint(Vec2(-2.0, 0.3), context(), CurveHint{});
  EXPECT_EQ(behind.piece, PieceKind::kBackwardRay);
  EXPECT_NEAR(behind.s_star, -2.0, 1e-12);
  // A foot exactly at the far end of the window's last piece is not returned as such: the scan
  // extends and the result is a projection.
  const ClosestPoint edge = line.closestPoint(Vec2(0.75, 0.3), context(), CurveHint{});
  EXPECT_NEAR(edge.s_star, 0.75, 1e-12);
  EXPECT_NEAR(edge.D, 0.3, 1e-12);

  // A convex corner beyond the window: the result is the wedge vertex, never a clamped point of
  // the window's last piece.
  std::vector<Vec2> corner = straightLine(2.0, 0.05);
  for (int i = 1; i <= 40; ++i) {
    corner.emplace_back(2.0, 0.05 * i);
  }
  const PolylineCurve bend = buildPolyline(corner, false, pluginWindow());
  const ClosestPoint wedge = bend.closestPoint(Vec2(2.5, -0.5), context(), CurveHint{});
  EXPECT_TRUE(wedge.in_wedge);
  EXPECT_EQ(wedge.piece, PieceKind::kVertex);
  EXPECT_EQ(wedge.x_star, Vec2(2.0, 0.0));
  EXPECT_NEAR(wedge.s_star, 2.0, 1e-12);
  EXPECT_NEAR(wedge.D, std::sqrt(0.5), 1e-12);
}

// WindowScan over random windows returns a local minimum that is a projection: never a clamped
// edge point. With a window covering everything it is exact.
TEST(PolylineSearch, WindowScanResultIsAProjectionAndExactOverAll)
{
  std::mt19937_64 rng(31);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> jitter(-1.0, 1.0);
  int not_projection = 0;
  double worst_exact = 0.0;
  for (int trial = 0; trial < 300; ++trial) {
    std::vector<Vec2> points{{0.0, 0.0}};
    double heading = 0.0;
    for (int i = 1; i < 40; ++i) {
      heading += (unit(rng) - 0.5) * 2.0;
      points.push_back(points.back() + (0.05 + 0.3 * unit(rng)) * vf::heading(heading));
    }
    PolylineCurve curve(pluginWindow());
    if (curve.assign(points, false) != AssignStatus::kOk) {
      continue;
    }
    for (int q = 0; q < 20; ++q) {
      const Vec2 x = points[static_cast<std::size_t>(q % 40)] + randomPoint(rng, jitter);
      const double s_min = curve.length(0.0) * unit(rng) - 0.5;
      curve.setSearchPolicy(SearchPolicy::windowScan(s_min, s_min + 1.2));
      const ClosestPoint cp = curve.closestPoint(x, context(), CurveHint{});
      if (cp.D >= vf::kEpsD) {
        if (cp.in_wedge) {
          // Interior wedge: both adjacent pieces clamp at the vertex (a1 > 0 and a2 > 0).
          const std::size_t v = static_cast<std::size_t>(cp.segment);
          const std::size_t last_segment = curve.segmentCount() - 1;
          const Vec2 offset = x - curve.vertices()[v];
          const Vec2 t_in = curve.segmentTangent(v == 0 ? 0 : std::min(v - 1, last_segment));
          const Vec2 t_out = curve.segmentTangent(std::min(v, last_segment));
          const bool beyond_in = offset.dot(t_in) > -1e-12;
          const bool before_out = offset.dot(t_out) < 1e-12;
          not_projection += (beyond_in && before_out) ? 0 : 1;
        } else if (std::abs((x - cp.x_star).dot(cp.T)) > 1e-9) {
          ++not_projection;
        }
      }
      curve.setSearchPolicy(windowAll());
      const ClosestPoint exact = curve.closestPoint(x, context(), CurveHint{});
      worst_exact = std::max(worst_exact, std::abs(exact.D - bruteForceDistance(curve, x)));
    }
  }
  EXPECT_EQ(not_projection, 0);
  EXPECT_LE(worst_exact, vf::kTieTolerance + 1e-12);
}

TEST(PolylineSearch, HintedLocalWindowExtendsBothWays)
{
  const PolylineCurve line = buildPolyline(straightLine(10.0, 0.05), false, hinted());
  const CurveHint hint = stationHint(5.0);
  const ClosestPoint back = line.closestPoint(Vec2(2.0, 0.1), context(), hint);
  EXPECT_NEAR(back.s_star, 2.0, 1e-12);
  EXPECT_EQ(back.status_bits, Status::kNone);
  const ClosestPoint forward = line.closestPoint(Vec2(8.3, -0.2), context(), hint);
  EXPECT_NEAR(forward.s_star, 8.3, 1e-12);
  const ClosestPoint ray_back = line.closestPoint(Vec2(-3.0, 0.2), context(), hint);
  EXPECT_EQ(ray_back.piece, PieceKind::kBackwardRay);
  EXPECT_NEAR(ray_back.s_star, -3.0, 1e-12);
  const ClosestPoint ray_ahead = line.closestPoint(Vec2(13.0, 0.0), context(), hint);
  EXPECT_EQ(ray_ahead.piece, PieceKind::kForwardRay);
  EXPECT_NEAR(ray_ahead.s_star, 13.0, 1e-12);
  // A hint far out on a ray still finds the curve.
  const ClosestPoint from_ray = line.closestPoint(Vec2(4.0, 0.1), context(), stationHint(25.0));
  EXPECT_NEAR(from_ray.s_star, 4.0, 1e-12);
}

TEST(PolylineSearch, HintedGlobalStageTriggerHysteresisAndAdoption)
{
  const std::vector<Vec2> points = uTurn(0.8, 4.0, 200);
  const Vec2 x(-0.4, 0.52);  // D_loc = 0.52 on leg 1, D_glob = 0.28 on leg 2, near the apex
  const CurveHint on_leg_1 = stationHint(3.6);

  const PolylineCurve adopt = buildPolyline(points, false, hinted(1.0, 0.5, 0.10));
  const ClosestPoint jumped = adopt.closestPoint(x, context(), on_leg_1);
  EXPECT_NEAR(jumped.x_star.y(), 0.8, 1e-12);
  EXPECT_NEAR(jumped.D, 0.28, 1e-12);
  EXPECT_EQ(jumped.status_bits, Status::kAmbiguousClosest);
  // The next cycle tracks leg 2 with no flag: the status is per cycle.
  const ClosestPoint next = adopt.closestPoint(x, context(), vf::makeHint(jumped));
  EXPECT_NEAR(next.s_star, jumped.s_star, 1e-12);
  EXPECT_EQ(next.status_bits, Status::kNone);
  // Initialisation adopts the global result unconditionally, without the flag.
  const ClosestPoint initial = adopt.closestPoint(x, context(), CurveHint{});
  EXPECT_NEAR(initial.s_star, jumped.s_star, 1e-12);
  EXPECT_EQ(initial.status_bits, Status::kNone);

  // Hysteresis: D_glob < D_loc - h_g fails for h_g = 0.3.
  const PolylineCurve sticky = buildPolyline(points, false, hinted(1.0, 0.5, 0.3));
  const ClosestPoint kept = sticky.closestPoint(x, context(), on_leg_1);
  EXPECT_EQ(kept.x_star.y(), 0.0);
  EXPECT_NEAR(kept.D, 0.52, 1e-12);
  EXPECT_EQ(kept.status_bits, Status::kNone);

  // Trigger: with D_trig = 0.6 > D_loc the global stage does not run.
  const PolylineCurve lazy = buildPolyline(points, false, hinted(1.0, 0.6, 0.10));
  EXPECT_EQ(lazy.closestPoint(x, context(), on_leg_1).x_star.y(), 0.0);
}

TEST(PolylineSearch, HintedInitialisationIsExact)
{
  std::mt19937_64 rng(77);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  double worst = 0.0;
  int samples = 0;
  for (int trial = 0; trial < 400; ++trial) {
    std::vector<Vec2> points;
    const int n = 3 + trial % 20;
    std::uniform_real_distribution<double> coord(-5.0, 5.0);
    for (int i = 0; i < n; ++i) {
      points.push_back(randomPoint(rng, coord));
    }
    PolylineCurve curve(hinted());
    if (curve.assign(points, trial % 2 == 0) != AssignStatus::kOk) {
      continue;
    }
    std::uniform_real_distribution<double> query(-8.0, 8.0);
    for (int q = 0; q < 30; ++q) {
      const Vec2 x = randomPoint(rng, query);
      worst = std::max(
        worst,
        std::abs(curve.closestPoint(x, context(), CurveHint{}).D - bruteForceDistance(curve, x)));
      ++samples;
    }
  }
  EXPECT_GE(samples, 10000);
  EXPECT_LE(worst, vf::kTieTolerance + 1e-12);
}

// ---------------------------------------------------------------------------------------------
// Closed curves, stations and sampling
// ---------------------------------------------------------------------------------------------

TEST(PolylineCurve, ClosedCurveWrapsStations)
{
  const std::vector<Vec2> square{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  const PolylineCurve curve = buildPolyline(square, true, hinted());
  const double length = curve.length(0.0);
  ASSERT_DOUBLE_EQ(length, 4.0);
  // Stations are in [0, L): the closing segment approaches L, P0 is 0.
  EXPECT_NEAR(curve.closestPoint(Vec2(-0.1, 0.05), context(), CurveHint{}).s_star, 3.95, 1e-12);
  EXPECT_NEAR(curve.closestPoint(Vec2(0.05, -0.1), context(), CurveHint{}).s_star, 0.05, 1e-12);
  const ClosestPoint corner = curve.closestPoint(Vec2(-0.1, -0.1), context(), CurveHint{});
  EXPECT_TRUE(corner.in_wedge);
  EXPECT_EQ(corner.s_star, 0.0);
  const ClosestPoint on_p0 = curve.closestPoint(Vec2(0.0, 1e-12), context(), stationHint(3.99));
  EXPECT_GE(on_p0.s_star, 0.0);
  EXPECT_LT(on_p0.s_star, length);

  // The hinted window wraps across the seam: no global stage, no flag.
  const ClosestPoint across = curve.closestPoint(Vec2(0.05, -0.02), context(), stationHint(3.95));
  EXPECT_NEAR(across.s_star, 0.05, 1e-12);
  EXPECT_EQ(across.status_bits, Status::kNone);
  const ClosestPoint back = curve.closestPoint(Vec2(-0.02, 0.1), context(), stationHint(0.05));
  EXPECT_NEAR(back.s_star, 3.9, 1e-12);

  // Ties at the centre go to the smallest wrapped |s - s_prev|.
  PolylineCurve all = buildPolyline(square, true, hinted(10.0));
  EXPECT_NEAR(all.closestPoint(Vec2(0.5, 0.5), context(), stationHint(3.9)).s_star, 3.5, 1e-12);
  EXPECT_NEAR(all.closestPoint(Vec2(0.5, 0.5), context(), stationHint(0.1)).s_star, 0.5, 1e-12);
  EXPECT_NEAR(all.closestPoint(Vec2(0.5, 0.5), context(), stationHint(4.1)).s_star, 0.5, 1e-12);
  // Without a hint they go to the smallest dir * s.
  EXPECT_NEAR(all.closestPoint(Vec2(0.5, 0.5), context(1), CurveHint{}).s_star, 0.5, 1e-12);
  EXPECT_NEAR(all.closestPoint(Vec2(0.5, 0.5), context(-1), CurveHint{}).s_star, 3.5, 1e-12);

  // WindowScan on a closed curve wraps its window too.
  all.setSearchPolicy(SearchPolicy::windowScan(3.5, 4.3));
  EXPECT_NEAR(all.closestPoint(Vec2(0.2, -0.05), context(), CurveHint{}).s_star, 0.2, 1e-12);

  // pointAt/tangentAt wrap; wrapStationDifference maps into (-L/2, L/2].
  EXPECT_TRUE(curve.pointAt(4.5, 0.0).isApprox(curve.pointAt(0.5, 0.0), 1e-15));
  EXPECT_TRUE(curve.pointAt(-0.5, 0.0).isApprox(Vec2(0.0, 0.5), 1e-15));
  EXPECT_EQ(curve.tangentAt(-0.5, 0.0), Vec2(0.0, -1.0));
  EXPECT_EQ(curve.tangentAt(4.0, 0.0), Vec2(1.0, 0.0));
  EXPECT_NEAR(curve.wrapStationDifference(3.9, 0.0), -0.1, 1e-12);
  EXPECT_NEAR(curve.wrapStationDifference(-2.0, 0.0), 2.0, 1e-12);
  EXPECT_TRUE(std::isnan(vf::remainingToEnd(curve, corner, 0.0)));
}

TEST(PolylineCurve, PointAtTangentAtAndSampling)
{
  const std::vector<Vec2> points{{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.5}, {0.5, 1.5}};
  const PolylineCurve curve = buildPolyline(points, false, hinted());
  const double length = curve.length(0.0);
  for (std::size_t k = 0; k < points.size(); ++k) {
    EXPECT_EQ(curve.pointAt(curve.station(k), 0.0), points[k]) << k;
  }
  EXPECT_EQ(curve.pointAt(-1.0, 0.0), Vec2(-1.0, 0.0));
  EXPECT_TRUE(curve.pointAt(length + 2.0, 0.0).isApprox(Vec2(-1.5, 1.5), 1e-15));
  EXPECT_TRUE(curve.pointAt(2.75, 0.0).isApprox(Vec2(2.0, 0.75), 1e-15));
  EXPECT_EQ(curve.tangentAt(-1.0, 0.0), Vec2(1.0, 0.0));
  EXPECT_EQ(curve.tangentAt(2.0, 0.0), Vec2(0.0, 1.0));  // half-open [S_k, S_k+1)
  EXPECT_EQ(curve.tangentAt(1.999, 0.0), Vec2(1.0, 0.0));
  EXPECT_EQ(curve.tangentAt(length, 0.0), Vec2(-1.0, 0.0));
  EXPECT_EQ(curve.tangentAt(length + 5.0, 0.0), Vec2(-1.0, 0.0));
  EXPECT_TRUE(std::isnan(curve.pointAt(vf::kNaN, 0.0).x()));
  EXPECT_TRUE(std::isnan(PolylineCurve().pointAt(0.0, 0.0).x()));

  std::vector<Vec2> samples(100);
  const std::size_t count = curve.sampleStations(-1.0, length + 1.0, 0.4, 0.0, samples);
  ASSERT_EQ(count, static_cast<std::size_t>(std::floor(length / 0.4)) + 2u);  // rays clamped
  EXPECT_EQ(samples[0], points[0]);
  EXPECT_EQ(samples[count - 1], points[3]);
  for (std::size_t i = 0; i < count; ++i) {
    EXPECT_LE(bruteForceDistance(curve, samples[i]), 1e-12);
  }
  EXPECT_EQ(curve.sampleStations(0.0, 1.0, 0.0, 0.0, samples), 0u);
}

TEST(PolylineCurve, OnCurveInteriorPointsHaveZeroNormal)
{
  const PolylineCurve curve =
    buildPolyline({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}}, false, pluginWindow());
  const ClosestPoint mid = curve.closestPoint(Vec2(1.0, 0.0), context(), CurveHint{});
  EXPECT_EQ(mid.n, Vec2::Zero());
  EXPECT_EQ(mid.T, Vec2(1.0, 0.0));
  EXPECT_EQ(mid.piece, PieceKind::kSegment);
  EXPECT_EQ(mid.s_star, 1.0);
  const ClosestPoint near_mid = curve.closestPoint(Vec2(2.0, 0.7 + 1e-12), context(), CurveHint{});
  EXPECT_EQ(near_mid.n, Vec2::Zero());
  EXPECT_EQ(near_mid.T, Vec2(0.0, 1.0));
  EXPECT_NEAR(near_mid.s_star, 2.7, 1e-12);
  const ClosestPoint ray = curve.closestPoint(Vec2(2.0, 5.0), context(), CurveHint{});
  EXPECT_EQ(ray.n, Vec2::Zero());
  EXPECT_TRUE(ray.on_ray);
  EXPECT_NEAR(ray.s_star, 7.0, 1e-12);
}
