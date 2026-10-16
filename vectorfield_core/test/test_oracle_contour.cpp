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

// Contour oracles (math_spec §6, §12.5): O31 trap, O32 start predicate, O33 clearance,
// O34 identity K.
//
// Unless stated otherwise: TC with `obstacle_mode: contour` (FL, λ = 0.47, D_in = 0.62,
// D'_in = 0.82), path along +x on y = 0 ending at a free goal, follower-style inputs, a fixed
// grid whose cell centres include (2, 0) so that discs centred there are symmetric about the path.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "test_support.hpp"
#include "vectorfield_core/contour.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/gains.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

constexpr int kDiscTag = 1;
constexpr int kBoxTag = 2;
constexpr int kWallTag = 3;

// Every episode start satisfies the start predicate at the law's point (math_spec §6.4):
// n_oᵀΦ < 0 with the nominal Φ, and D_o <= D'_in.
void expectStartPredicate(const StraightRun & r)
{
  for (const Episode & e : r.eps) {
    const Cycle & c = r.cycles[e.start];
    EXPECT_LT(c.out.field.contour.n_o.dot(c.out.field.Phi), 0.0) << "start at t = " << c.rec.t;
    EXPECT_LE(c.out.field.contour.D_o, kDinPrime + 1e-9) << "start at t = " << c.rec.t;
  }
}

}  // namespace

// O32 Start predicate, closed loop (math_spec §6.4): in runs past a disc on the path and past a box
// beside it, every episode starts with n_oᵀΦ < 0 (nominal Φ) and D_o <= D'_in at the law's point.
TEST(OracleContour, O32_EpisodesStartOnlyAgainstTheField)
{
  {
    ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
    world.addDisc(Vec2(2.0, 0.0), 0.3, kDiscTag);
    world.commit();
    const StraightRun r =
      runStraight(tcContourConfig(), world, Pose2{Vec2(-1.0, 0.05), 0.0}, 6.0, 30.0);
    ASSERT_GE(r.eps.size(), 1U);
    expectStartPredicate(r);
  }
  {
    ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
    world.addBox(Vec2(2.0, -0.6), Vec2(2.6, 0.1), kBoxTag);
    world.commit();
    const StraightRun r =
      runStraight(tcContourConfig(), world, Pose2{Vec2(-1.0, 0.0), 0.0}, 6.0, 30.0);
    ASSERT_GE(r.eps.size(), 1U);
    expectStartPredicate(r);
  }
}

// O32 Start predicate, constructed (math_spec §6.4): one component K shaped like "⊐" around the
// robot: a bar 0.775 m behind the control point (nearest, n_oᵀΦ > 0, D_o <= D'_in) joined by a top
// bar to a bar ahead that blocks the path (gate open). No episode may start in this cycle.
TEST(OracleContour, O32_NoStartWhenFieldPointsAway)
{
  ObstacleWorld world = centredWorld(-3.0, 3.0, -2.0, 2.0);
  world.addBox(Vec2(-1.1, -1.0), Vec2(-0.95, 1.1), kBoxTag);  // rear bar
  world.addBox(Vec2(0.9, -1.0), Vec2(1.05, 1.1), kBoxTag);    // front bar
  world.addBox(Vec2(-1.1, 0.95), Vec2(1.05, 1.1), kBoxTag);   // top bar
  world.commit();
  const auto curve = makePolyline({Vec2(-3.0, 0.0), Vec2(5.0, 0.0)}, false);
  const vf::Tracker tracker(tcContourConfig());
  vf::StepScratch scratch;
  vf::TrackerInput input;
  input.pose = Pose2{Vec2(-0.4, 0.0), 0.0};
  input.measured = Twist2{0.3, 0.0, 0.0};
  input.dt = kPeriod;
  input.curve = curve.get();
  input.goal = goalAt(Vec2(5.0, 0.0));
  input.obstacles = world.map();
  vf::TrackerState state = vf::Tracker::initialState();
  const vf::TrackerOutput out = tracker.step(input, state, &state, scratch);
  const vf::ContourSample & cs = out.field.contour;
  ASSERT_TRUE(cs.gate_open) << "the front bar blocks the path within the lookahead";
  ASSERT_LE(cs.D_o, kDinPrime);
  ASSERT_GT(cs.n_o.dot(out.field.Phi), 0.0);
  EXPECT_FALSE(state.contour.active);
  EXPECT_FALSE(cs.active);
}

// O32 Band (math_spec §6.4): with an episode active and D_o in (D'_in, D'_in + h_c] at the
// control point, θ = 1 and the command equals the no-contour command to 1e-12. Constructed
// state: latched episode, σ_c = +1, k_seed on the disc; collision detection off in both trackers
// so that only the contour differs.
TEST(OracleContour, O32_BandCommandEqualsNoContourCommand)
{
  ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
  world.addDisc(Vec2(2.0, 0.0), 0.3, kDiscTag);
  world.commit();
  const auto curve = makePolyline({Vec2(-2.0, 0.0), Vec2(6.0, 0.0)}, false);
  vf::CoreConfig with_contour = tcContourConfig();
  with_contour.safety.use_collision_detection = false;
  vf::CoreConfig without_contour = tcConfig();
  without_contour.safety.use_collision_detection = false;
  const vf::Tracker a(with_contour);
  const vf::Tracker b(without_contour);
  for (const double D_target : {0.83, 0.845, 0.86}) {
    SCOPED_TRACE("D_o target " + std::to_string(D_target));
    // Head-on: the nearest site centre is (1.7, 0); p_c = base + 0.2.
    const double x_base = 1.7 - D_target - kD;
    vf::TrackerInput input;
    input.pose = Pose2{Vec2(x_base, 0.0), 0.0};
    input.measured = Twist2{0.4, 0.0, 0.0};
    input.dt = kPeriod;
    input.curve = curve.get();
    input.goal = goalAt(Vec2(6.0, 0.0));
    input.obstacles = world.map();
    vf::TrackerState in = vf::Tracker::initialState();
    in.limiter.u_prev = Twist2{0.4, 0.0, 0.0};
    in.limiter.valid = true;
    in.contour.gate_open = true;
    in.contour.active = true;
    in.contour.sigma_c = 1;
    in.contour.travelled_m = 0.5;
    in.contour.elapsed_s = 1.25;
    in.contour.k_seed = Vec2(1.7, 0.0);
    in.contour.k_valid = true;
    vf::StepScratch scratch_a;
    vf::StepScratch scratch_b;
    vf::TrackerState out_a;
    vf::TrackerState out_b;
    const vf::TrackerOutput oa = a.step(input, in, &out_a, scratch_a);
    const vf::TrackerOutput ob = b.step(input, in, &out_b, scratch_b);
    ASSERT_GT(oa.field.contour.D_o, kDinPrime);
    ASSERT_LE(oa.field.contour.D_o, kDinPrime + kHc);
    ASSERT_TRUE(out_a.contour.active) << "the episode stays latched in the band";
    EXPECT_EQ(oa.field.contour.theta, 1.0);
    EXPECT_NEAR(oa.cmd.vx, ob.cmd.vx, 1e-12);
    EXPECT_NEAR(oa.cmd.vy, ob.cmd.vy, 1e-12);
    EXPECT_NEAR(oa.cmd.wz, ob.cmd.wz, 1e-12);
    EXPECT_NEAR(oa.cmd_target.vx, ob.cmd_target.vx, 1e-12);
    EXPECT_NEAR(oa.cmd_target.wz, ob.cmd_target.wz, 1e-12);
    EXPECT_LE((oa.field.F - ob.field.F).norm(), 1e-12);
  }
}

// O33 Clearance (math_spec §6.2, §6.4): disc of radius 0.3 and 0.6 m centred on the path, head-on
// start. The distance from the evaluation point to the disc's cells (centres, brute force) stays
// >= λ − res, its minimum is <= λ + 2 res, and once the circulation (θ = 0) has reached
// R_o + λ ± res it stays within ±res while θ = 0. Timeout 35 s.
TEST(OracleContour, O33_ClearanceAndCirculationRadius)
{
  for (const double R_o : {0.3, 0.6}) {
    SCOPED_TRACE("R_o = " + std::to_string(R_o));
    ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
    world.addDisc(Vec2(2.0, 0.0), R_o, kDiscTag);
    world.commit();
    const StraightRun r =
      runStraight(tcContourConfig(), world, Pose2{Vec2(-1.0, 0.0), 0.0}, 6.0, 35.0);
    ASSERT_GE(r.eps.size(), 1U);
    double min_d = vf::kInf;
    for (const Cycle & c : r.cycles) {
      const double d = world.distanceToSites(c.out.field.x_eval, kDiscTag);
      min_d = std::min(min_d, d);
      EXPECT_GE(d, kLambda - kRes) << "t = " << c.rec.t;
    }
    EXPECT_LE(min_d, kLambda + 2.0 * kRes);
    bool converged = false;
    for (const Episode & e : r.eps) {
      for (std::size_t k = e.start; k < e.end; ++k) {
        const Cycle & c = r.cycles[k];
        if (c.out.field.contour.theta != 0.0) {
          continue;
        }
        const double err = (c.out.field.x_eval - Vec2(2.0, 0.0)).norm() - (R_o + kLambda);
        if (!converged) {
          converged = std::abs(err) <= kRes;
        } else {
          EXPECT_LE(std::abs(err), kRes) << "circulation left R_o + λ ± res at t = " << c.rec.t;
        }
      }
    }
    EXPECT_TRUE(converged) << "the circulation never reached R_o + λ ± res";
  }
}

// O34 Identity (math_spec §6.4): a box touching the path from below and a wall parallel to the
// path 0.45 m from it (not merged with the box in the dilated mask: 0.5 m between site centres).
// During every episode K is the box: each K cell lies within r_insc of a box site, at least one
// does, and no K cell lies in the wall. The wall clearance (base to wall sites) stays >= the path
// clearance − 0.05. Timeout 30 s.
TEST(OracleContour, O34_IdentityIsTheBlockingBox)
{
  // Grid lines (not centres) on multiples of 0.05 m: box site centres up to y = −0.025, wall
  // site centres from y = 0.475.
  ObstacleWorld world(Vec2(-2.5, -3.0), 10.5, 5.0);
  world.addBox(Vec2(2.5, -0.6), Vec2(3.3, 0.0), kBoxTag);
  world.addBox(Vec2(0.0, 0.45), Vec2(6.0, 0.6), kWallTag);
  world.commit();
  int k_checked = 0;
  int k_bad = 0;
  const auto per_step = [&](const Cycle & c, const vf::StepScratch & scratch) {
    if (!c.after.contour.active || !scratch.k_valid) {
      return;
    }
    ++k_checked;
    const vf::OccupancyGrid & grid = world.grid();
    bool has_box = false;
    bool bad = scratch.k_width != grid.width() || scratch.k_height != grid.height();
    for (int iy = 0; iy < scratch.k_height && !bad; ++iy) {
      for (int ix = 0; ix < scratch.k_width; ++ix) {
        if (scratch.k_mask[static_cast<std::size_t>(iy * scratch.k_width + ix)] == 0) {
          continue;
        }
        const Vec2 centre = grid.cellCenter(ix, iy);
        has_box = has_box || world.inShape(centre, kBoxTag);
        if (
          world.inShape(centre, kWallTag) ||
          world.distanceToSites(centre, kBoxTag) > kRInsc + 1e-9) {
          bad = true;
          break;
        }
      }
    }
    if (bad || !has_box) {
      ++k_bad;
    }
  };
  const StraightRun r =
    runStraight(tcContourConfig(), world, Pose2{Vec2(0.0, 0.0), 0.0}, 7.0, 30.0, per_step);
  ASSERT_GE(r.eps.size(), 1U);
  EXPECT_GT(k_checked, 0);
  EXPECT_EQ(k_bad, 0) << "K included the wall or missed the box";
  const double path_clearance = world.distanceToSites(Vec2(1.0, 0.0), kWallTag);
  for (const Cycle & c : r.cycles) {
    EXPECT_GE(world.distanceToSites(c.rec.pose.p, kWallTag), path_clearance - 0.05)
      << "t = " << c.rec.t;
  }
  EXPECT_GT(r.cycles.back().rec.pose.p.x(), 3.3) << "never passed the box";
}
