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

// Contour oracles (math_spec §6.4, §12.5): O36 K tracking under a rolling origin and removal,
// O37 goal veto. O38 is in test_oracle_contour_watchdog.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

constexpr int kBoxTag = 1;

struct KTrackResult
{
  std::vector<Cycle> cycles;
  std::vector<Episode> eps;
  int k_checked{0};
  int k_bad{0};
  std::size_t removed_at{0};  // first cycle stepped on a grid without the box (0: never)
};

// Box 0.6 × 0.6 m centred at (3, 0) on the path y = 0 → (6.5, 0); FL from (0, 0.02). With
// `rolling`, the grid origin moves by 1–10 cells per cycle (seeded, along x or y, always back
// toward its base, so the offset stays within ±10 cells); the box stays fixed in the evaluation
// frame. With `remove_after` > 0, the box is removed that many seconds after the episode starts.
// K (StepScratch::k_mask) is checked after every step with an active episode: every K cell lies
// within r_insc of a box site and at least one is a box site.
KTrackResult runKTracking(bool rolling, double remove_after, double timeout = 25.0)
{
  // The grid covers the robot and its rollouts (x in [−1, 6], |y| < 1.75) for every origin offset
  // (at most ±10 cells).
  const Vec2 base_origin(-1.5, -2.25);
  ObstacleWorld world(base_origin, 8.0, 4.5);
  const int box = world.addBox(Vec2(2.7, -0.3), Vec2(3.3, 0.3), kBoxTag);
  world.commit();
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(6.5, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.02), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(6.5, 0.0));
  runner.input().obstacles = world.map();
  std::mt19937_64 rng(3636);
  std::uniform_int_distribution<int> cells(1, 10);
  std::uniform_int_distribution<int> axis(0, 1);
  Vec2 offset(0.0, 0.0);
  KTrackResult r;
  double episode_start = -1.0;
  bool removed = false;
  const std::size_t steps = static_cast<std::size_t>(std::lround(timeout / kPeriod));
  for (std::size_t k = 0; k < steps; ++k) {
    bool changed = false;
    if (rolling) {
      const int a = axis(rng);
      const double shift = kRes * cells(rng);
      offset[a] += offset[a] > 0.0 ? -shift : shift;
      world.setOrigin(base_origin + offset);
      changed = true;
    }
    if (
      remove_after > 0.0 && !removed && episode_start >= 0.0 &&
      runner.time() >= episode_start + remove_after) {
      world.setEnabled(box, false);
      removed = true;
      r.removed_at = k;
      changed = true;
    }
    if (changed) {
      world.commit();
    }
    const Cycle & c = runner.step();
    if (c.after.contour.active && episode_start < 0.0) {
      episode_start = c.rec.t;
    }
    const vf::StepScratch & scratch = runner.sim().scratch();
    if (c.after.contour.active && scratch.k_valid) {
      ++r.k_checked;
      bool bad =
        scratch.k_width != world.grid().width() || scratch.k_height != world.grid().height();
      bool has_box = false;
      for (int iy = 0; iy < scratch.k_height && !bad; ++iy) {
        for (int ix = 0; ix < scratch.k_width; ++ix) {
          if (scratch.k_mask[static_cast<std::size_t>(iy * scratch.k_width + ix)] == 0) {
            continue;
          }
          const Vec2 centre = world.grid().cellCenter(ix, iy);
          has_box = has_box || world.inShape(centre, kBoxTag);
          if (world.distanceToSites(centre, kBoxTag) > kRInsc + 1e-9) {
            bad = true;
            break;
          }
        }
      }
      r.k_bad += (bad || !has_box) ? 1 : 0;
    }
    if (c.pose_after.p.x() > 4.6 && !c.after.contour.active && (remove_after <= 0.0 || removed)) {
      break;
    }
  }
  r.cycles = runner.cycles();
  r.eps = episodes(r.cycles);
  return r;
}

}  // namespace

// O36 K tracking (math_spec §6.4 "Identity K": k_seed in the grid frame, mapped through the current
// origin): rolling-origin shifts of 1–10 cells per cycle keep K (the box) and σ_c: one episode,
// constant σ_c equal to the fixed-grid run's, K checked on every active cycle, and the episode
// lasts until the base has passed the box centre.
TEST(OracleContour, O36_RollingOriginKeepsKAndSide)
{
  const KTrackResult fixed = runKTracking(false, 0.0);
  const KTrackResult rolling = runKTracking(true, 0.0);
  ASSERT_EQ(fixed.eps.size(), 1U);
  ASSERT_EQ(rolling.eps.size(), 1U);
  EXPECT_TRUE(rolling.eps[0].sigma_constant);
  EXPECT_EQ(rolling.eps[0].sigma_c, fixed.eps[0].sigma_c);
  EXPECT_GT(rolling.k_checked, 0);
  EXPECT_EQ(rolling.k_bad, 0);
  EXPECT_EQ(fixed.k_bad, 0);
  const std::size_t end = rolling.eps[0].end;
  ASSERT_LT(end, rolling.cycles.size()) << "episode never ended";
  EXPECT_GE(rolling.cycles[end].rec.pose.p.x(), 3.0) << "episode ended before passing the box";
}

// O36 removal (math_spec §6.4 "K is lost"; also UpstreamRegression.U24): a box removed mid-episode
// ends the episode on the first grid without it, on a fixed grid and under rolling shifts.
TEST(OracleContour, O36_RemovedBoxEndsEpisodeOnFirstGrid)
{
  for (const bool rolling : {false, true}) {
    SCOPED_TRACE(rolling ? "rolling origin" : "fixed grid");
    const KTrackResult r = runKTracking(rolling, 1.0);
    ASSERT_GT(r.removed_at, 0U) << "no episode to interrupt";
    ASSERT_LT(r.removed_at, r.cycles.size());
    EXPECT_TRUE(r.cycles[r.removed_at].before.contour.active);
    EXPECT_FALSE(r.cycles[r.removed_at].after.contour.active);
    EXPECT_FALSE(r.cycles[r.removed_at].out.field.contour.active);
  }
}

// O37 Goal veto, blocked goal (math_spec §6.4 "Goal veto"): a disc of 0.3 m diameter centred on an
// open curve's end: no episode ever starts and the robot stops >= m_s − res short of contact
// (ground-truth translation to contact along +x), never colliding. Timeout 20 s.
TEST(OracleContour, O37_BlockedGoalVetoesContour)
{
  ObstacleWorld world(Vec2(-2.525, -2.525), 7.0, 5.0);
  world.addDisc(Vec2(3.0, 0.0), 0.15);
  world.commit();
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(3.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(3.0, 0.0));
  runner.input().obstacles = world.map();
  runner.run(20.0);
  const vf::FootprintChecker checker(vf::Footprint::fromSpec(tcFootprint()));
  for (const Cycle & c : runner.cycles()) {
    EXPECT_FALSE(c.after.contour.active) << "episode at t = " << c.rec.t;
    EXPECT_FALSE(checker.collidesGroundTruth(c.pose_after, world.grid())) << "t = " << c.rec.t;
  }
  EXPECT_LT(std::abs(runner.cycles().back().rec.cmd.vx), 1e-3) << "did not stop";
  EXPECT_GE(travelToContact(checker, world, runner.pose(), Vec2(1.0, 0.0)), kStopMargin - kRes);
}

// O37 Goal veto, free goal behind a box (math_spec §6.4): a free goal 0.3 m behind a 0.2 m box
// centred on the path: exactly one episode, released once the box is behind the robot (no
// watchdog), and the goal is reached (settled, ‖p − g‖ <= xy_tol). Timeout 40 s.
TEST(OracleContour, O37_FreeGoalBehindBox)
{
  ObstacleWorld world(Vec2(-2.0, -2.5), 7.0, 5.0);
  world.addBox(Vec2(1.9, -0.1), Vec2(2.1, 0.1));
  world.commit();
  const Vec2 goal(2.4, 0.0);
  const auto curve = makePolyline({Vec2(-1.0, 0.0), goal}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(goal);
  runner.input().obstacles = world.map();
  runner.run(40.0, [](const Cycle & c) { return c.out.settled; });
  const auto & cs = runner.cycles();
  const std::vector<Episode> eps = episodes(cs);
  ASSERT_EQ(eps.size(), 1U);
  ASSERT_LT(eps[0].end, cs.size()) << "the episode never ended";
  // Released once the blocking set is behind the robot (math_spec §6.4, B(s*) empty), with the
  // base past the box's far face; the gate may still be within its release time then.
  const Cycle & end = cs[eps[0].end];
  EXPECT_GT(end.rec.pose.p.x(), 2.1) << "the episode ended before the box was passed";
  EXPECT_FALSE(anyStatus(cs, Status::kContourWatchdog));
  ASSERT_TRUE(cs.back().out.settled) << "goal not reached";
  EXPECT_LE((cs.back().rec.pose.p - goal).norm(), kXyTol);
}

// O37 Goal veto with the cap (math_spec §9.1 precedence, §10.4): contour on, a wall whose face is
// 0.08 m beyond the goal blocks r(s_end) (veto, no episode); the cap stops the robot inside
// xy_tol: settled through the cap clause (the first settled cycle has the cap binding), and
// kBlocked is never set. The face distance must lie in (0.07, 0.095) m so that the predicted
// contact precedes the nominal settle point (see O54). Timeout 20 s.
TEST(OracleContour, O37_CapInsideTolerancePrecedesBlocked)
{
  // Grid lines at x = 0.03 + 0.05 k so that the wall face lies exactly at x = 2.08.
  ObstacleWorld world(Vec2(-2.02, -2.5), 7.0, 5.0);
  world.addBox(Vec2(2.08, -1.0), Vec2(2.58, 1.0));
  world.commit();
  const Vec2 goal(2.0, 0.0);
  const auto curve = makePolyline({Vec2(-1.0, 0.0), goal}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(goal);
  runner.input().obstacles = world.map();
  runner.run(20.0);
  for (const Cycle & c : runner.cycles()) {
    EXPECT_FALSE(Status::has(c.out.status, Status::kBlocked)) << "t = " << c.rec.t;
    EXPECT_FALSE(c.after.contour.active) << "t = " << c.rec.t;
  }
  const auto & cs = runner.cycles();
  const std::size_t k_settled = firstIndex(cs, [](const Cycle & c) { return c.out.settled; });
  ASSERT_LT(k_settled, cs.size()) << "never settled";
  EXPECT_TRUE(cs[k_settled].out.shaping.capBinds()) << "settled, but not through the cap clause";
  EXPECT_GT((cs[k_settled].rec.pose.p - goal).norm(), kSigmaStop);
  EXPECT_LE((runner.pose().p - goal).norm(), kXyTol);
}
