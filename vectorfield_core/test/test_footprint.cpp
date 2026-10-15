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

// Footprints, their radii and the footprint collision checker (math_spec §10.1; oracle O53).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::EuclideanDistanceTransform;
using vf::Footprint;
using vf::FootprintChecker;
using vf::FootprintDecision;
using vf::OccupancyGrid;
using vf::Pose2;
using vf::Vec2;

namespace
{

// TC footprint: a 16-gon of circumradius 0.22 with a vertex on +y (math_spec §12).
std::vector<Vec2> sixteenGon(double circumradius)
{
  std::vector<Vec2> polygon;
  for (int k = 0; k < 16; ++k) {
    polygon.push_back(circumradius * vf::heading(vf::kTwoPi * k / 16.0));
  }
  return polygon;
}

// Footprint::polygon() takes a Span, which never binds to a temporary vector.
Footprint polygonFootprint(const std::vector<Vec2> & vertices)
{
  return Footprint::polygon(vertices);
}

double polygonArea(const std::vector<Vec2> & polygon)
{
  double twice = 0.0;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    twice += vf::cross2(polygon[j], polygon[i]);
  }
  return 0.5 * twice;
}

// ---- Brute-force reference of the ground truth (math_spec §10.1) -------------------------------
// The polygon mapped into the grid frame, every site cell of the grid, and the textbook closed
// polygon/square test: a polygon vertex in the closed square, a square corner in the closed
// polygon (boundary included), or two intersecting closed edges (CLRS §33.1).

int orientation(const Vec2 & a, const Vec2 & b, const Vec2 & c)
{
  const double v = vf::cross2(b - a, c - a);
  return (v > 0.0) - (v < 0.0);
}

bool withinBox(const Vec2 & a, const Vec2 & b, const Vec2 & p)
{
  return std::min(a.x(), b.x()) <= p.x() && p.x() <= std::max(a.x(), b.x()) &&
         std::min(a.y(), b.y()) <= p.y() && p.y() <= std::max(a.y(), b.y());
}

bool segmentsIntersect(const Vec2 & p1, const Vec2 & p2, const Vec2 & q1, const Vec2 & q2)
{
  const int d1 = orientation(q1, q2, p1);
  const int d2 = orientation(q1, q2, p2);
  const int d3 = orientation(p1, p2, q1);
  const int d4 = orientation(p1, p2, q2);
  if (d1 * d2 < 0 && d3 * d4 < 0) {
    return true;
  }
  return (d1 == 0 && withinBox(q1, q2, p1)) || (d2 == 0 && withinBox(q1, q2, p2)) ||
         (d3 == 0 && withinBox(p1, p2, q1)) || (d4 == 0 && withinBox(p1, p2, q2));
}

bool inClosedPolygon(const std::vector<Vec2> & polygon, const Vec2 & p)
{
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Vec2 & a = polygon[j];
    const Vec2 & b = polygon[i];
    if (orientation(a, b, p) == 0 && withinBox(a, b, p)) {
      return true;
    }
    if ((a.y() > p.y()) != (b.y() > p.y())) {
      const double x_cross = a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
      if (x_cross > p.x()) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool polygonMeetsClosedSquare(const std::vector<Vec2> & polygon, const Vec2 & centre, double h)
{
  const std::array<Vec2, 4> corners = {
    centre + Vec2(-h, -h), centre + Vec2(h, -h), centre + Vec2(h, h), centre + Vec2(-h, h)};
  for (const Vec2 & v : polygon) {
    if (std::abs(v.x() - centre.x()) <= h && std::abs(v.y() - centre.y()) <= h) {
      return true;
    }
  }
  for (const Vec2 & corner : corners) {
    if (inClosedPolygon(polygon, corner)) {
      return true;
    }
  }
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    for (int k = 0; k < 4; ++k) {
      if (segmentsIntersect(polygon[j], polygon[i], corners[k], corners[(k + 1) % 4])) {
        return true;
      }
    }
  }
  return false;
}

bool referenceCollides(const Footprint & footprint, const Pose2 & pose, const OccupancyGrid & grid)
{
  const double h = 0.5 * grid.resolution();
  std::vector<Vec2> polygon;
  Vec2 lo = pose.p - Vec2(footprint.radius(), footprint.radius());
  Vec2 hi = pose.p + Vec2(footprint.radius(), footprint.radius());
  if (!footprint.isCircle()) {
    lo = Vec2(vf::kInf, vf::kInf);
    hi = -lo;
    for (const Vec2 & v : footprint.vertices()) {
      polygon.push_back(pose.p + vf::rotate(pose.yaw, v));
      lo = lo.cwiseMin(polygon.back());
      hi = hi.cwiseMax(polygon.back());
    }
  }
  for (int iy = 0; iy < grid.height(); ++iy) {
    for (int ix = 0; ix < grid.width(); ++ix) {
      if (!grid.isSite(ix, iy)) {
        continue;
      }
      const Vec2 c = grid.cellCenter(ix, iy);
      if (c.x() + h < lo.x() || c.x() - h > hi.x() || c.y() + h < lo.y() || c.y() - h > hi.y()) {
        continue;  // bounding boxes apart
      }
      if (footprint.isCircle()) {
        const Vec2 closest(
          std::clamp(pose.p.x(), c.x() - h, c.x() + h),
          std::clamp(pose.p.y(), c.y() - h, c.y() + h));
        if ((closest - pose.p).norm() <= footprint.radius()) {
          return true;
        }
      } else if (polygonMeetsClosedSquare(polygon, c, h)) {
        return true;
      }
    }
  }
  return false;
}

// ---- Random scenes ------------------------------------------------------------------------------

// Random angles in [0, 2 pi), sorted, at least `min_gap` apart (including across 2 pi).
std::vector<double> sortedAngles(std::mt19937_64 & rng, int n, double min_gap)
{
  std::uniform_real_distribution<double> angle(0.0, vf::kTwoPi);
  for (;;) {
    std::vector<double> a(n);
    for (auto & x : a) {
      x = angle(rng);
    }
    std::sort(a.begin(), a.end());
    bool ok = vf::kTwoPi - (a.back() - a.front()) >= min_gap;
    for (int i = 1; i < n && ok; ++i) {
      ok = a[i] - a[i - 1] >= min_gap;
    }
    if (ok) {
      return a;
    }
  }
}

enum class Shape
{
  kConvex,
  kStar,
  kCircle,
  kTc,
  kRectangle
};

Footprint randomFootprint(std::mt19937_64 & rng, Shape * shape)
{
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (;;) {
    const int pick = static_cast<int>(rng() % 10);
    std::vector<Vec2> polygon;
    const Vec2 offset(0.16 * unit(rng) - 0.08, 0.16 * unit(rng) - 0.08);  // base_link off-centre
    if (pick < 4) {  // convex: points on an ellipse in angular order
      *shape = Shape::kConvex;
      const int n = 3 + static_cast<int>(rng() % 10);
      const double a = 0.05 + 0.3 * unit(rng);
      const double b = 0.05 + 0.3 * unit(rng);
      const double tilt = vf::kTwoPi * unit(rng);
      for (const double t : sortedAngles(rng, n, 0.1)) {
        polygon.push_back(offset + vf::rotate(tilt, Vec2(a * std::cos(t), b * std::sin(t))));
      }
    } else if (pick < 8) {  // star-shaped, generally non-convex
      *shape = Shape::kStar;
      const int n = 5 + static_cast<int>(rng() % 12);
      for (const double t : sortedAngles(rng, n, 0.05)) {
        polygon.push_back(offset + (0.03 + 0.35 * unit(rng)) * vf::heading(t));
      }
    } else if (pick == 8) {
      *shape = Shape::kCircle;
      return Footprint::circle(rng() % 10 == 0 ? 0.0 : 0.4 * unit(rng));
    } else if (rng() % 2 == 0) {
      *shape = Shape::kTc;
      return polygonFootprint(sixteenGon(0.22));
    } else {
      *shape = Shape::kRectangle;
      const double front = 0.05 + 0.3 * unit(rng);
      const double back = 0.05 + 0.3 * unit(rng);
      const double half = 0.05 + 0.2 * unit(rng);
      polygon = {Vec2(-back, -half), Vec2(front, -half), Vec2(front, half), Vec2(-back, half)};
    }
    if (std::abs(polygonArea(polygon)) > 2e-3) {
      return Footprint::polygon(polygon);
    }
  }
}

struct Scene
{
  OccupancyGrid grid;
  EuclideanDistanceTransform edt;
};

void randomScene(std::mt19937_64 & rng, Scene * scene)
{
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const int width = 8 + static_cast<int>(rng() % 41);
  const int height = 8 + static_cast<int>(rng() % 41);
  const double res = 0.02 + 0.08 * unit(rng);
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(width) * height, 0);
  const int kind = static_cast<int>(rng() % 20);
  if (kind == 0) {
    // empty
  } else if (kind == 1) {
    std::fill(mask.begin(), mask.end(), 1);
  } else {
    const double density = 0.08 * unit(rng);
    for (auto & c : mask) {
      c = unit(rng) < density ? 1 : 0;
    }
    const int boxes = static_cast<int>(rng() % 6);
    for (int b = 0; b < boxes; ++b) {
      const int x0 = static_cast<int>(rng() % width);
      const int y0 = static_cast<int>(rng() % height);
      const int x1 = std::min(width - 1, x0 + static_cast<int>(rng() % 8));
      const int y1 = std::min(height - 1, y0 + static_cast<int>(rng() % 8));
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          mask[static_cast<std::size_t>(y) * width + x] = 1;
        }
      }
    }
  }
  const Vec2 origin(4.0 * unit(rng) - 2.0, 4.0 * unit(rng) - 2.0);
  ASSERT_TRUE(scene->grid.setFromMask(mask, width, height, res, origin));
  scene->edt.update(scene->grid);
}

Pose2 randomPose(std::mt19937_64 & rng, const OccupancyGrid & grid)
{
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const double res = grid.resolution();
  const Vec2 extent(grid.width() * res, grid.height() * res);
  // Mostly inside the grid; one pose in eight may lie up to 0.3 m outside it.
  const double margin = rng() % 8 == 0 ? 0.3 : 0.0;
  const Vec2 p = grid.origin() + Vec2(
                                   (extent.x() + 2.0 * margin) * unit(rng) - margin,
                                   (extent.y() + 2.0 * margin) * unit(rng) - margin);
  return Pose2{p, vf::kTwoPi * unit(rng) - vf::kPi};
}

// Minimum distance from p to a site centre (brute force), +inf without sites.
double nearestSiteCentre(const OccupancyGrid & grid, const Vec2 & p)
{
  double best = vf::kInf;
  for (int iy = 0; iy < grid.height(); ++iy) {
    for (int ix = 0; ix < grid.width(); ++ix) {
      if (grid.isSite(ix, iy)) {
        best = std::min(best, (grid.cellCenter(ix, iy) - p).norm());
      }
    }
  }
  return best;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Footprint

TEST(Footprint, PointRobotByDefault)
{
  const Footprint point;
  EXPECT_TRUE(point.isCircle());
  EXPECT_EQ(point.radius(), 0.0);
  EXPECT_TRUE(point.vertices().empty());
  EXPECT_EQ(point.wLat(), 0.0);
  EXPECT_EQ(point.rInsc(), 0.0);
  EXPECT_EQ(point.rCirc(), 0.0);
  const FootprintChecker checker;
  EXPECT_TRUE(checker.footprint().isCircle());
  EXPECT_EQ(checker.footprint().radius(), 0.0);
}

TEST(Footprint, RadiiOfCircleRectangleAndSixteenGon)
{
  const Footprint disc = Footprint::circle(0.3);
  EXPECT_TRUE(disc.isCircle());
  EXPECT_EQ(disc.radius(), 0.3);
  EXPECT_EQ(disc.wLat(), 0.3);
  EXPECT_EQ(disc.rInsc(), 0.3);
  EXPECT_EQ(disc.rCirc(), 0.3);

  // base_link off-centre, clockwise.
  const std::vector<Vec2> rectangle = {
    Vec2(-0.3, 0.15), Vec2(0.2, 0.15), Vec2(0.2, -0.15), Vec2(-0.3, -0.15)};
  const Footprint box = Footprint::polygon(rectangle);
  EXPECT_FALSE(box.isCircle());
  EXPECT_EQ(box.radius(), 0.0);
  ASSERT_EQ(box.vertices().size(), 4u);
  EXPECT_EQ(box.vertices()[1], Vec2(0.2, 0.15));
  EXPECT_NEAR(box.wLat(), 0.15, 1e-15);
  EXPECT_NEAR(box.rInsc(), 0.15, 1e-15);
  EXPECT_NEAR(box.rCirc(), std::hypot(0.3, 0.15), 1e-15);

  // TC: 16-gon of circumradius 0.22: r_insc = 0.2158, w_lat = r_circ = 0.22 (math_spec §12).
  const Footprint tc = polygonFootprint(sixteenGon(0.22));
  EXPECT_NEAR(tc.rInsc(), 0.2158, 1e-4);
  EXPECT_NEAR(tc.rInsc(), 0.22 * std::cos(vf::kPi / 16.0), 1e-12);
  EXPECT_NEAR(tc.wLat(), 0.22, 1e-4);
  EXPECT_NEAR(tc.rCirc(), 0.22, 1e-4);

  // The radii are config.hpp's computeFootprintRadii (one definition for the whole core).
  vf::FootprintSpec spec;
  spec.polygon = sixteenGon(0.22);
  const vf::FootprintRadii expected = vf::computeFootprintRadii(spec);
  const Footprint from_spec = Footprint::fromSpec(spec);
  EXPECT_EQ(from_spec.wLat(), expected.w_lat);
  EXPECT_EQ(from_spec.rInsc(), expected.r_insc);
  EXPECT_EQ(from_spec.rCirc(), expected.r_circ);
  vf::FootprintSpec round;
  round.radius = 0.17;
  EXPECT_TRUE(Footprint::fromSpec(round).isCircle());
  EXPECT_EQ(Footprint::fromSpec(round).rInsc(), 0.17);

  // base_link outside the polygon: no inscribed disc.
  const Footprint ahead = polygonFootprint(
    std::vector<Vec2>{Vec2(0.1, -0.1), Vec2(0.5, -0.1), Vec2(0.5, 0.1), Vec2(0.1, 0.1)});
  EXPECT_EQ(ahead.rInsc(), 0.0);
  EXPECT_NEAR(ahead.rCirc(), std::hypot(0.5, 0.1), 1e-15);
}

TEST(Footprint, InvalidInputThrowsConfigError)
{
  EXPECT_THROW(Footprint::circle(-0.1), vf::ConfigError);
  EXPECT_THROW(Footprint::circle(vf::kNaN), vf::ConfigError);
  EXPECT_THROW(Footprint::circle(vf::kInf), std::invalid_argument);
  EXPECT_NO_THROW(Footprint::circle(0.0));
  const std::vector<Vec2> two = {Vec2(0.0, 0.0), Vec2(1.0, 0.0)};
  EXPECT_THROW(Footprint::polygon(two), vf::ConfigError);
  EXPECT_THROW(Footprint::polygon(vf::Span<const Vec2>()), vf::ConfigError);
  const std::vector<Vec2> non_finite = {Vec2(0.0, 0.0), Vec2(1.0, vf::kNaN), Vec2(0.0, 1.0)};
  EXPECT_THROW(Footprint::polygon(non_finite), vf::ConfigError);
  const std::vector<Vec2> collinear = {Vec2(0.0, 0.0), Vec2(1.0, 0.0), Vec2(2.0, 0.0)};
  EXPECT_THROW(Footprint::polygon(collinear), vf::ConfigError);
  vf::FootprintSpec bad;
  bad.radius = -1.0;
  EXPECT_THROW(Footprint::fromSpec(bad), vf::ConfigError);
}

// -----------------------------------------------------------------------------------------------
// FootprintChecker

TEST(FootprintChecker, O53_ShortcutsAreSoundAndGroundTruthMatchesBruteForce)
{
  std::mt19937_64 rng(53);
  int tally[5] = {0, 0, 0, 0, 0};
  int shape_tally[5] = {0, 0, 0, 0, 0};
  int cases = 0;
  int ground_truth_collisions = 0;
  Scene scene;
  for (int s = 0; s < 250; ++s) {
    randomScene(rng, &scene);
    for (int f = 0; f < 8; ++f) {
      Shape shape = Shape::kCircle;
      const FootprintChecker checker(randomFootprint(rng, &shape));
      const Footprint & footprint = checker.footprint();
      for (int k = 0; k < 5; ++k) {
        const Pose2 pose = randomPose(rng, scene.grid);
        const FootprintDecision decision = checker.classify(pose, scene.grid, scene.edt);
        const bool truth = checker.collidesGroundTruth(pose, scene.grid);
        const bool reference = referenceCollides(footprint, pose, scene.grid);
        ++cases;
        ++tally[static_cast<int>(decision)];
        ++shape_tally[static_cast<int>(shape)];
        ground_truth_collisions += truth ? 1 : 0;
        ASSERT_EQ(truth, reference) << "scene " << s << " footprint " << f << " pose ("
                                    << pose.p.transpose() << ", " << pose.yaw << ")";
        switch (decision) {
          case FootprintDecision::kFreeShortcut:
            // O53: shortcut-free implies ground-truth free; and directly, no site centre lies
            // within r_circ + res/sqrt(2) of base_link.
            ASSERT_FALSE(truth) << "scene " << s << " footprint " << f << " pose " << k;
            EXPECT_GT(
              nearestSiteCentre(scene.grid, pose.p),
              footprint.rCirc() + scene.grid.resolution() / std::sqrt(2.0));
            break;
          case FootprintDecision::kCollisionShortcut:
            // O53: shortcut-collision implies ground-truth collision.
            ASSERT_TRUE(truth) << "scene " << s << " footprint " << f << " pose " << k;
            EXPECT_LT(nearestSiteCentre(scene.grid, pose.p), footprint.rInsc());
            break;
          case FootprintDecision::kFreeRasterized:
            ASSERT_FALSE(truth);
            break;
          case FootprintDecision::kCollisionRasterized:
            ASSERT_TRUE(truth);
            break;
          case FootprintDecision::kOffGrid:
            ASSERT_FALSE(scene.grid.cellAt(pose.p, nullptr, nullptr));
            break;
        }
        // collides() is the ground truth for every pose, off-grid ones included (in-grid cells).
        ASSERT_EQ(checker.collides(pose, scene.grid, scene.edt), truth);
      }
    }
  }
  std::printf(
    "[ O53 ] %d cases: free shortcut %d, collision shortcut %d, free rasterised %d, collision "
    "rasterised %d, off-grid %d; ground-truth collisions %d; convex %d, star %d, circle %d, TC %d, "
    "rectangle %d\n",
    cases, tally[0], tally[1], tally[2], tally[3], tally[4], ground_truth_collisions,
    shape_tally[0], shape_tally[1], shape_tally[2], shape_tally[3], shape_tally[4]);
  EXPECT_GE(cases, 10000);
  for (const int count : tally) {
    EXPECT_GT(count, 300);  // every rule was exercised
  }
}

TEST(FootprintChecker, TouchingCountsAsCollision)
{
  // Closed squares against the closed footprint, on dyadic values so every coordinate is exact:
  // res 0.25, one site cell (4, 4) = [1.0, 1.25]^2.
  const int n = 16;
  std::vector<std::uint8_t> mask(n * n, 0);
  mask[4 * n + 4] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, 0.25, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const double eps = std::ldexp(1.0, -30);

  const FootprintChecker box(polygonFootprint(
    std::vector<Vec2>{Vec2(-0.5, -0.25), Vec2(0.5, -0.25), Vec2(0.5, 0.25), Vec2(-0.5, 0.25)}));
  // Right edge on the square's left edge.
  EXPECT_TRUE(box.collidesGroundTruth(Pose2{Vec2(0.5, 1.125), 0.0}, grid));
  EXPECT_TRUE(box.collides(Pose2{Vec2(0.5, 1.125), 0.0}, grid, edt));
  EXPECT_FALSE(box.collidesGroundTruth(Pose2{Vec2(0.5 - eps, 1.125), 0.0}, grid));
  EXPECT_FALSE(box.collides(Pose2{Vec2(0.5 - eps, 1.125), 0.0}, grid, edt));
  // Corner on corner.
  EXPECT_TRUE(box.collidesGroundTruth(Pose2{Vec2(0.5, 0.75), 0.0}, grid));
  EXPECT_FALSE(box.collidesGroundTruth(Pose2{Vec2(0.5, 0.75 - eps), 0.0}, grid));
  EXPECT_FALSE(box.collidesGroundTruth(Pose2{Vec2(0.5 - eps, 0.75), 0.0}, grid));
  // Top edge on the square's bottom edge, overlapping in x.
  EXPECT_TRUE(box.collidesGroundTruth(Pose2{Vec2(1.0, 0.75), 0.0}, grid));
  EXPECT_FALSE(box.collidesGroundTruth(Pose2{Vec2(1.0, 0.75 - eps), 0.0}, grid));

  const FootprintChecker disc(Footprint::circle(0.5));
  EXPECT_TRUE(disc.collidesGroundTruth(Pose2{Vec2(0.5, 1.125), 0.0}, grid));
  EXPECT_FALSE(disc.collidesGroundTruth(Pose2{Vec2(0.5 - eps, 1.125), 0.0}, grid));

  const FootprintChecker point;
  EXPECT_TRUE(point.collidesGroundTruth(Pose2{Vec2(1.0, 1.1), 0.0}, grid));
  EXPECT_TRUE(point.collidesGroundTruth(Pose2{Vec2(1.25, 1.25), 0.0}, grid));
  EXPECT_TRUE(point.collides(Pose2{Vec2(1.1, 1.1), 0.0}, grid, edt));
  EXPECT_FALSE(point.collidesGroundTruth(Pose2{Vec2(1.0 - eps, 1.1), 0.0}, grid));
  EXPECT_FALSE(point.collides(Pose2{Vec2(0.5, 0.5), 0.0}, grid, edt));
}

TEST(FootprintChecker, NonConvexFootprintsUseTheFilledPolygon)
{
  // A U-shaped footprint (opening towards +x) around a site cell: the site sits in the notch, so
  // it is outside the filled polygon although inside its convex hull; a site under the U's base
  // collides.
  const std::vector<Vec2> u_shape = {Vec2(-0.3, -0.3),   Vec2(0.3, -0.3),   Vec2(0.3, -0.15),
                                     Vec2(-0.15, -0.15), Vec2(-0.15, 0.15), Vec2(0.3, 0.15),
                                     Vec2(0.3, 0.3),     Vec2(-0.3, 0.3)};
  const FootprintChecker checker(Footprint::polygon(u_shape));
  EXPECT_EQ(checker.footprint().rInsc(), 0.0);  // base_link is in the notch
  const int n = 40;
  const double res = 0.05;
  std::vector<std::uint8_t> mask(n * n, 0);
  mask[20 * n + 20] = 1;  // centre (1.025, 1.025)
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, res, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const Vec2 site = grid.cellCenter(20, 20);
  EXPECT_FALSE(checker.collides(Pose2{site, 0.0}, grid, edt));
  EXPECT_FALSE(checker.collides(Pose2{site - Vec2(0.05, 0.0), 0.0}, grid, edt));
  EXPECT_TRUE(checker.collides(Pose2{site + Vec2(0.25, 0.0), 0.0}, grid, edt));  // under the base
  EXPECT_TRUE(checker.collides(Pose2{site - Vec2(0.0, 0.2), 0.0}, grid, edt));   // in an arm
  EXPECT_FALSE(checker.collides(Pose2{site, vf::kPi}, grid, edt));               // notch to -x
  EXPECT_TRUE(
    checker.collides(Pose2{site, vf::kHalfPi}, grid, edt) ==
    referenceCollides(checker.footprint(), Pose2{site, vf::kHalfPi}, grid));
}

TEST(FootprintChecker, ClassifyReportsEachRule)
{
  // TC footprint, res 0.05, a 2 x 2 block of sites at cells (30..31, 30..31) = [1.5, 1.6]^2.
  const int n = 60;
  std::vector<std::uint8_t> mask(n * n, 0);
  for (int y = 30; y <= 31; ++y) {
    for (int x = 30; x <= 31; ++x) {
      mask[y * n + x] = 1;
    }
  }
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, 0.05, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const FootprintChecker checker(polygonFootprint(sixteenGon(0.22)));
  EXPECT_EQ(
    checker.classify(Pose2{Vec2(0.5, 0.5), 0.3}, grid, edt), FootprintDecision::kFreeShortcut);
  EXPECT_EQ(
    checker.classify(Pose2{Vec2(1.55, 1.55), 0.3}, grid, edt),
    FootprintDecision::kCollisionShortcut);
  // The 16-gon's vertex at angle 0 reaches 1.51 > 1.5: overlaps the block by 1 cm.
  EXPECT_EQ(
    checker.classify(Pose2{Vec2(1.29, 1.55), 0.0}, grid, edt),
    FootprintDecision::kCollisionRasterized);
  // ... and 1.485 < 1.5: misses it by 1.5 cm, inside both shortcut margins.
  EXPECT_EQ(
    checker.classify(Pose2{Vec2(1.265, 1.55), 0.0}, grid, edt), FootprintDecision::kFreeRasterized);
  EXPECT_EQ(checker.classify(Pose2{Vec2(-0.01, 1.0), 0.0}, grid, edt), FootprintDecision::kOffGrid);
  EXPECT_EQ(
    checker.classify(Pose2{Vec2(vf::kNaN, 1.0), 0.0}, grid, edt), FootprintDecision::kOffGrid);
  EXPECT_TRUE(checker.collides(Pose2{Vec2(vf::kNaN, 1.0), 0.0}, grid, edt));  // never free

  // No sites at all: free by the shortcut.
  const std::vector<std::uint8_t> empty(n * n, 0);
  OccupancyGrid clear;
  ASSERT_TRUE(clear.setFromMask(empty, n, n, 0.05, Vec2::Zero()));
  EuclideanDistanceTransform clear_edt;
  clear_edt.update(clear);
  EXPECT_EQ(
    checker.classify(Pose2{Vec2(1.5, 1.5), 0.0}, clear, clear_edt),
    FootprintDecision::kFreeShortcut);
  // The empty (0 x 0) grid: off-grid, nothing to collide with.
  const OccupancyGrid none;
  const EuclideanDistanceTransform none_edt;
  EXPECT_EQ(
    checker.classify(Pose2{Vec2(0.0, 0.0), 0.0}, none, none_edt), FootprintDecision::kOffGrid);
  EXPECT_FALSE(checker.collides(Pose2{Vec2(0.0, 0.0), 0.0}, none, none_edt));
}

TEST(FootprintChecker, StaleOrFilteredTransformFallsBackToGroundTruth)
{
  // The shortcuts need EDT_all of exactly this grid content; otherwise the checker rasterises.
  std::mt19937_64 rng(5);
  Scene scene;
  randomScene(rng, &scene);
  const int n = 40;
  std::vector<std::uint8_t> mask(n * n, 0);
  for (int y = 10; y < 30; ++y) {
    mask[y * n + 20] = 1;
  }
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, 0.05, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  // A new wall appears; the transform is not updated.
  for (int x = 0; x < n; ++x) {
    mask[35 * n + x] = 1;
  }
  ASSERT_TRUE(grid.setFromMask(mask, n, n, 0.05, Vec2::Zero()));
  ASSERT_NE(edt.builtVersion(), grid.version());
  const FootprintChecker checker(polygonFootprint(sixteenGon(0.22)));
  const Pose2 on_new_wall{grid.cellCenter(5, 35), 0.0};
  EXPECT_EQ(checker.classify(on_new_wall, grid, edt), FootprintDecision::kCollisionRasterized);
  EXPECT_TRUE(checker.collides(on_new_wall, grid, edt));
  const Pose2 far{grid.cellCenter(5, 5), 0.0};
  EXPECT_EQ(checker.classify(far, grid, edt), FootprintDecision::kFreeRasterized);
  // A filtered transform (EDT_K) has builtVersion 0: never used for shortcuts.
  std::vector<std::uint8_t> only_first(mask.size(), 0);
  only_first[10 * n + 20] = 1;
  edt.build(grid, only_first);
  EXPECT_EQ(checker.classify(on_new_wall, grid, edt), FootprintDecision::kCollisionRasterized);
  EXPECT_EQ(checker.classify(far, grid, edt), FootprintDecision::kFreeRasterized);
  // Updated: shortcuts again.
  edt.update(grid);
  EXPECT_EQ(checker.classify(on_new_wall, grid, edt), FootprintDecision::kCollisionShortcut);
  EXPECT_EQ(checker.classify(far, grid, edt), FootprintDecision::kFreeShortcut);
}

TEST(FootprintChecker, OffGridWhenTheInflatedDiscLeavesTheGrid)
{
  // [0, 3]^2 at 0.05; TC footprint: the disc of radius r_circ + res = 0.27 must stay inside.
  const int n = 60;
  const std::vector<std::uint8_t> mask(n * n, 0);
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, 0.05, Vec2::Zero()));
  const FootprintChecker tc(polygonFootprint(sixteenGon(0.22)));
  EXPECT_FALSE(tc.offGrid(Pose2{Vec2(1.5, 1.5), 0.0}, grid));
  EXPECT_FALSE(tc.offGrid(Pose2{Vec2(0.28, 0.28), 2.0}, grid));
  EXPECT_FALSE(tc.offGrid(Pose2{Vec2(2.72, 2.72), -1.0}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(0.26, 1.5), 0.0}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(2.74, 1.5), 0.0}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(1.5, 0.26), 0.0}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(1.5, 2.74), 0.0}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(-1.0, 1.5), 0.0}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(vf::kNaN, 1.5), 0.0}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(1.5, 1.5), vf::kInf}, grid));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(1.5, 1.5), 0.0}, OccupancyGrid()));
  // A point robot needs only res of margin; a shifted origin moves the bounds.
  const FootprintChecker point;
  EXPECT_FALSE(point.offGrid(Pose2{Vec2(0.06, 1.5), 0.0}, grid));
  EXPECT_TRUE(point.offGrid(Pose2{Vec2(0.04, 1.5), 0.0}, grid));
  OccupancyGrid shifted;
  ASSERT_TRUE(shifted.setFromMask(mask, n, n, 0.05, Vec2(-10.0, 5.0)));
  EXPECT_FALSE(tc.offGrid(Pose2{Vec2(-8.5, 6.5), 0.0}, shifted));
  EXPECT_TRUE(tc.offGrid(Pose2{Vec2(1.5, 1.5), 0.0}, shifted));
}

TEST(FootprintChecker, OffGridPosesTestTheInGridCells)
{
  // Sites along column 0 of a grid starting at x = 0: a base_link left of the grid still collides
  // when the footprint reaches into those cells.
  const int n = 40;
  std::vector<std::uint8_t> mask(n * n, 0);
  for (int y = 0; y < n; ++y) {
    mask[y * n] = 1;
  }
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, 0.05, Vec2::Zero()));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const FootprintChecker tc(polygonFootprint(sixteenGon(0.22)));
  const Pose2 reaching{Vec2(-0.1, 1.0), 0.0};  // reaches x = 0.12
  EXPECT_EQ(tc.classify(reaching, grid, edt), FootprintDecision::kOffGrid);
  EXPECT_TRUE(tc.offGrid(reaching, grid));
  EXPECT_TRUE(tc.collides(reaching, grid, edt));
  const Pose2 short_of{Vec2(-0.3, 1.0), 0.0};  // reaches x = -0.08
  EXPECT_FALSE(tc.collides(short_of, grid, edt));
  EXPECT_TRUE(tc.collides(Pose2{Vec2(-0.1, 1.0), vf::kNaN}, grid, edt));
}
