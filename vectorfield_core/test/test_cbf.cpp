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

// The CBF filter and its sites (math_spec §6.7 [Design, M7]).

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "test_support.hpp"
#include "vectorfield_core/cbf.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)
using vf::CbfConstraint;
using vf::CbfSite;

/// Brute-force QP: the feasible point of a fine polar grid around F closest to F.
Vec2 bruteForce(const Vec2 & F, const CbfConstraint * c, int count, double gain)
{
  Vec2 best = Vec2::Zero();
  double cost = vf::kInf;
  for (double r = 0.0; r <= 2.0; r += 0.002) {
    for (double a = 0.0; a < 2.0 * vf::kPi; a += 0.002) {
      const Vec2 x = F + r * vf::heading(a);
      bool ok = true;
      for (int j = 0; j < count && ok; ++j) {
        ok = c[j].n.dot(x) >= -gain * c[j].h;
      }
      if (ok && r * r < cost) {
        cost = r * r;
        best = x;
      }
    }
    if (cost < vf::kInf) {
      break;  // the first feasible ring is the closest
    }
  }
  return best;
}

}  // namespace

TEST(Cbf, SingleConstraintIsTheClosedForm)
{
  const Vec2 F(0.4, 0.0);
  const CbfConstraint c{vf::heading(vf::kPi - 0.3), 0.05};
  const vf::CbfSolution s = vf::solveCbf(F, &c, 1, 2.0);
  const double b = -2.0 * c.h;
  const Vec2 expected = F + std::max(0.0, b - c.n.dot(F)) * c.n;
  EXPECT_TRUE(s.active);
  EXPECT_FALSE(s.relaxed);
  EXPECT_EQ(s.binds, 1U);
  EXPECT_NEAR((s.F - expected).norm(), 0.0, 1e-12);
  EXPECT_NEAR(c.n.dot(s.F), b, 1e-12);
  EXPECT_LE(s.F.norm(), F.norm());
}

TEST(Cbf, InactiveConstraintLeavesTheFieldUnchanged)
{
  const Vec2 F(0.4, 0.1);
  const CbfConstraint c{Vec2(0.0, 1.0), 0.2};
  const vf::CbfSolution s = vf::solveCbf(F, &c, 1, 2.0);
  EXPECT_FALSE(s.active);
  EXPECT_EQ(s.binds, 0U);
  EXPECT_EQ(s.F, F);
}

// Two and three constraints with h >= 0: the enumeration equals a brute-force search, and the
// filtered speed never exceeds the field's.
TEST(Cbf, ActiveSetMatchesBruteForceAndNeverSpeedsUp)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * vf::kPi);
  std::uniform_real_distribution<double> barrier(0.0, 0.3);
  std::uniform_real_distribution<double> speed(0.05, 0.5);
  for (int trial = 0; trial < 60; ++trial) {
    const int count = 2 + trial % 2;
    CbfConstraint c[3];
    for (int j = 0; j < count; ++j) {
      c[j] = CbfConstraint{vf::heading(angle(rng)), barrier(rng)};
    }
    const Vec2 F = speed(rng) * vf::heading(angle(rng));
    const vf::CbfSolution s = vf::solveCbf(F, c, count, 2.0);
    const Vec2 brute = bruteForce(F, c, count, 2.0);
    SCOPED_TRACE(trial);
    EXPECT_NEAR((s.F - F).norm(), (brute - F).norm(), 3e-3);
    EXPECT_LE(s.F.norm(), F.norm() + 1e-12);
    for (int j = 0; j < count; ++j) {
      EXPECT_GE(c[j].n.dot(s.F), -2.0 * c[j].h - 1e-9);
    }
  }
}

// Inside the margin: the push-out along n is kept, but never faster than the field.
TEST(Cbf, PushOutIsBoundedByTheFieldSpeed)
{
  const Vec2 F(0.1, 0.0);
  const CbfConstraint c{Vec2(0.0, 1.0), -0.2};  // asks n^T F' >= 0.4
  const vf::CbfSolution s = vf::solveCbf(F, &c, 1, 2.0);
  EXPECT_TRUE(s.active);
  EXPECT_NEAR(s.F.norm(), F.norm(), 1e-12);
  EXPECT_GT(s.F.y(), 0.0);
}

// Opposite push-outs cannot both hold: the QP is solved again without them and holds.
TEST(Cbf, ConflictingPushOutsHold)
{
  const Vec2 F(0.4, 0.0);
  const CbfConstraint c[2] = {{Vec2(0.0, 1.0), -0.05}, {Vec2(0.0, -1.0), -0.05}};
  const vf::CbfSolution s = vf::solveCbf(F, c, 2, 2.0);
  EXPECT_TRUE(s.relaxed);
  EXPECT_NEAR(s.F.y(), 0.0, 1e-12);
  EXPECT_NEAR(s.F.x(), 0.4, 1e-12);  // moving along the slot is still allowed
}

// A head-on constraint at h = 0 stops the robot (the accepted deadlock).
TEST(Cbf, HeadOnAtTheBarrierStops)
{
  const Vec2 F(0.4, 0.0);
  const CbfConstraint c{Vec2(-1.0, 0.0), 0.0};
  const vf::CbfSolution s = vf::solveCbf(F, &c, 1, 2.0);
  EXPECT_NEAR(s.F.norm(), 0.0, 1e-12);
}

// Sites: a straight wall gives one site (the others are within 45°); two discs on either side
// give one site each; the count is clamped and respected.
TEST(Cbf, SitesSeparateObstaclesNotCellsOfOne)
{
  ObstacleWorld wall = centredWorld(-2.0, 2.0, -2.0, 2.0);
  wall.addBox(Vec2(-2.0, 0.4), Vec2(2.0, 0.6));
  wall.commit();
  CbfSite sites[vf::kCbfMaxSites];
  EXPECT_EQ(vf::nearestCbfSites(wall.edt(), Vec2(0.01, 0.0), 3, sites), 1);
  EXPECT_NEAR(sites[0].D, 0.4, 1e-9);
  EXPECT_NEAR(sites[0].n.y(), -1.0, 1e-9);

  ObstacleWorld gap = centredWorld(-2.0, 2.0, -2.0, 2.0);
  gap.addDisc(Vec2(0.0, 0.5), 0.15, 1);
  gap.addDisc(Vec2(0.0, -0.5), 0.15, 2);
  gap.commit();
  const Vec2 x(-0.05, 0.02);
  ASSERT_EQ(vf::nearestCbfSites(gap.edt(), x, 3, sites), 2);
  // One site per disc. Near the bisector, the continuous query (site 1) falls back to the
  // nearest cell's feature, which may be the other disc's (math_spec §6.1): within res.
  const int upper = sites[0].n.y() < 0.0 ? 0 : 1;
  EXPECT_LT(sites[upper].n.y(), 0.0);
  EXPECT_GT(sites[1 - upper].n.y(), 0.0);
  EXPECT_NEAR(sites[upper].D, gap.distanceToSites(x, 1), kRes);
  EXPECT_NEAR(sites[1 - upper].D, gap.distanceToSites(x, 2), kRes);
  EXPECT_EQ(vf::nearestCbfSites(gap.edt(), x, 1, sites), 1);
  EXPECT_EQ(vf::nearestCbfSites(gap.edt(), x, 0, sites), 0);
  EXPECT_EQ(vf::nearestCbfSites(gap.edt(), Vec2(9.0, 9.0), 3, sites), 0) << "off the grid";
}

TEST(Cbf, CoverRadiusContainsTheFootprint)
{
  const vf::Footprint disc = vf::Footprint::circle(0.22);
  EXPECT_DOUBLE_EQ(disc.coverRadius(Vec2::Zero()), 0.22);
  EXPECT_DOUBLE_EQ(disc.coverRadius(Vec2(0.2, 0.0)), 0.42);
  const std::vector<Vec2> box{Vec2(0.3, 0.2), Vec2(-0.3, 0.2), Vec2(-0.3, -0.2), Vec2(0.3, -0.2)};
  const vf::Footprint rect = vf::Footprint::polygon(vf::Span<const Vec2>(box.data(), box.size()));
  EXPECT_DOUBLE_EQ(rect.coverRadius(Vec2::Zero()), rect.rCirc());
  EXPECT_NEAR(rect.coverRadius(Vec2(0.1, 0.0)), std::hypot(0.4, 0.2), 1e-12);
}
