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

// Closed-loop upstream regressions (docs/design/upstream_audit.md §3; named
// `UpstreamRegression.UNN_Name`): U06, U09 (phase part), U10, U15–U19, U24, U25 (closed-loop
// part), U28 (closed-loop part), U30. TC unless stated otherwise.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "test_support.hpp"
#include "vectorfield_core/contour.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/gains.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

// Base-link station on a curve by an independent hinted search chain (Curve::closestPoint is
// const and has no side effects).
class BaseStation
{
public:
  explicit BaseStation(const vf::Curve & curve) : curve_(curve) {}
  double operator()(const Vec2 & p)
  {
    vf::EvalContext ctx;
    const vf::ClosestPoint cp = curve_.closestPoint(p, ctx, hint_);
    hint_ = vf::makeHint(cp);
    return cp.s_star;
  }

private:
  const vf::Curve & curve_;
  vf::CurveHint hint_;
};

}  // namespace

// U06 (upstream: an open equation curve never terminates and wraps to its start): FL on an open
// EXPRESSION x = s, y = 0.5 sin s on [0, 6]: the base station s* is non-decreasing (1e-6), never
// returns toward s_min, no kNonFinite, and the run ends settled at r(s_max) (‖p − r(6)‖ <=
// 0.6 xy_tol). Timeout 40 s.
TEST(UpstreamRegression, U06_OpenExpressionCurveTerminates)
{
  const auto curve = makeExpression("s", "0.5*sin(s)", 0.0, 6.0, false);
  ASSERT_FALSE(curve->isClosed());
  const Vec2 end(6.0, 0.5 * std::sin(6.0));
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.0), std::atan(0.5)});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(end);
  runner.run(40.0, [](const Cycle & c) { return c.out.settled; });
  BaseStation station(*curve);
  double previous = -vf::kInf;
  for (const Cycle & c : runner.cycles()) {
    EXPECT_FALSE(Status::has(c.out.status, Status::kNonFinite)) << "t = " << c.rec.t;
    const double s = station(c.pose_after.p);
    EXPECT_GE(s, previous - 1e-6) << "s* decreased at t = " << c.rec.t;
    previous = std::max(previous, s);
  }
  ASSERT_TRUE(runner.cycles().back().out.settled) << "never settled";
  EXPECT_LE((runner.pose().p - end).norm(), 0.6 * kXyTol);
}

namespace
{

// U09: a C-shaped open polyline (circle of radius 1 centred at the origin, CCW) whose ends are
// 0.2 m apart, vertices every 2.5 cm.
std::vector<Vec2> cShape()
{
  const double gap = 2.0 * std::asin(0.1);
  return arcPoints(Vec2::Zero(), 1.0, -vf::kHalfPi + 0.5 * gap, 1.5 * vf::kPi - 0.5 * gap, 0.025);
}

struct U09Result
{
  std::vector<Cycle> cycles;
  double length{0.0};
  double s0{vf::kNaN};
  double s_at_final_approach{vf::kNaN};
  double s_max{0.0};
};

U09Result runCShape(bool windowed)
{
  const std::vector<Vec2> points = cShape();
  const auto fixed_curve = makePolyline(points, false);
  PrunedWindow window(points, 2.0);
  U09Result r;
  r.length = polylineLength(points, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcConfig(), options);
  const Vec2 t0 = (points[1] - points[0]).normalized();
  runner.reset(Pose2{points.front(), std::atan2(t0.y(), t0.x())});
  runner.input().goal = goalAt(points.back());
  if (windowed) {
    runner.input().cap_at_curve_end = true;
    runner.setPreStep(
      [&window](Runner & rr, vf::TrackerInput & in) { in.curve = &window.update(rr.pose().p); });
  } else {
    runner.input().curve = fixed_curve.get();
  }
  runner.run(40.0, [](const Cycle & c) { return c.out.settled; });
  r.cycles = runner.cycles();
  for (const Cycle & c : r.cycles) {
    double s = 0.0;
    polylineDistance(points, false, c.rec.pose.p, &s);
    if (std::isnan(r.s0)) {
      r.s0 = s;
    }
    if (std::isnan(r.s_at_final_approach) && c.out.phase == Phase::kFinalApproach) {
      r.s_at_final_approach = s;
    }
    r.s_max = std::max(r.s_max, s);
  }
  return r;
}

}  // namespace

// U09 phase part (upstream: open-path windows wrapped, so a robot at the start of a nearly closed
// path latched onto its end and stopped): on the C-shaped open polyline with ends 0.2 m apart
// (inside xy_tol), a robot at the start has s* ≈ 0 and σ_base ≈ L, never enters FINAL_APPROACH
// before the last final_approach_distance (0.45 m) + 0.1 m of the path, and completes the full
// length (settled at the end). Fixed follower curve and plugin-style re-pruned windows.
TEST(UpstreamRegression, U09_OpenPathWindowDoesNotWrap_PhasePart)
{
  for (const bool windowed : {false, true}) {
    SCOPED_TRACE(windowed ? "re-pruned windows" : "fixed curve");
    const U09Result r = runCShape(windowed);
    ASSERT_FALSE(r.cycles.empty());
    EXPECT_LT(r.s0, 0.01);
    if (!windowed) {
      EXPECT_GT(r.cycles.front().out.field.sigma, r.length - 0.1);
    }
    EXPECT_FALSE(r.cycles.front().out.settled);
    ASSERT_FALSE(std::isnan(r.s_at_final_approach)) << "never entered FINAL_APPROACH";
    EXPECT_GE(r.s_at_final_approach, r.length - 0.45 - 0.1);
    ASSERT_TRUE(r.cycles.back().out.settled) << "did not complete";
    EXPECT_GE(r.s_max, r.length - 0.15);
    EXPECT_LE((r.cycles.back().pose_after.p - cShape().back()).norm(), 0.6 * kXyTol);
  }
}

// U10 (upstream: the open-path stop fired at the last index, even far from the goal), four cases:
// (a) a start 1 m beside the last vertex does not stop before XY is reached (no 0.5 s stretch at
//     rest before ‖p − g‖ <= xy_tol) and ends settled within 0.6 xy_tol (with the settled rule
//     revised in review there is no σ clause for diff drive, so it no longer settles at the
//     tolerance edge);
// (b) speed is non-increasing over the last σ_b on a straight approach (one switch-lag cycle and
//     the v_min floor allowed, as O25);
// (c) direction −1 stops at the first vertex (settled, ‖p − P0‖ <= 0.6 xy_tol);
// (d) the straight run ends settled with ‖p − g‖ <= 0.6 xy_tol. Timeout 25 s each.
TEST(UpstreamRegression, U10_OpenPathStopsOnlyAtGoal)
{
  const auto curve = makePolyline({Vec2(0.0, 0.0), Vec2(4.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  {
    SCOPED_TRACE("(a) start beside the last vertex");
    Runner runner(tcConfig(), options);
    runner.reset(Pose2{Vec2(4.0, 1.0), -vf::kHalfPi});
    runner.input().curve = curve.get();
    runner.input().goal = goalAt(Vec2(4.0, 0.0));
    runner.run(25.0, [](const Cycle & c) { return c.out.settled; });
    const auto & cs = runner.cycles();
    const std::size_t k_xy = firstIndex(
      cs, [](const Cycle & c) { return (c.rec.pose.p - Vec2(4.0, 0.0)).norm() <= kXyTol; });
    ASSERT_LT(k_xy, cs.size()) << "XY never reached";
    int at_rest = 0;
    for (std::size_t k = 5; k < k_xy; ++k) {
      const bool rest = std::abs(cs[k].rec.cmd.vx) < 1e-3 && std::abs(cs[k].rec.cmd.wz) < 1e-3;
      at_rest = rest ? at_rest + 1 : 0;
      EXPECT_LT(at_rest, 10) << "stopped before XY at t = " << cs[k].rec.t;
      EXPECT_FALSE(cs[k].out.settled) << "settled before XY at t = " << cs[k].rec.t;
    }
    ASSERT_TRUE(cs.back().out.settled);
    EXPECT_LE((runner.pose().p - Vec2(4.0, 0.0)).norm(), 0.6 * kXyTol);
  }
  {
    SCOPED_TRACE("(b, d) straight approach");
    Runner runner(tcConfig(), options);
    runner.reset(Pose2{Vec2(2.0, 0.0), 0.0});
    runner.input().curve = curve.get();
    runner.input().goal = goalAt(Vec2(4.0, 0.0));
    runner.run(25.0, [](const Cycle & c) { return c.out.settled; });
    const auto & cs = runner.cycles();
    ASSERT_TRUE(cs.back().out.settled);
    EXPECT_LE((runner.pose().p - Vec2(4.0, 0.0)).norm(), 0.6 * kXyTol);
    const std::size_t k0 =
      firstIndex(cs, [](const Cycle & c) { return c.out.field.sigma <= kSigmaB; });
    const std::size_t k_fa =
      firstIndex(cs, [](const Cycle & c) { return c.out.phase == Phase::kFinalApproach; });
    int lag = 0;
    for (std::size_t k = k0 + 1; k < cs.size(); ++k) {
      const double prev = std::abs(cs[k - 1].rec.cmd.vx);
      const double now = std::abs(cs[k].rec.cmd.vx);
      if (now <= prev + 1e-9 || now <= kVmin + 1e-9) {
        continue;
      }
      if ((k == k_fa || k == k_fa + 1) && lag++ == 0) {
        continue;
      }
      ADD_FAILURE() << "|v| increased at t = " << cs[k].rec.t;
      break;
    }
  }
  {
    SCOPED_TRACE("(c) direction −1");
    vf::CoreConfig config = tcConfig();
    config.field.direction = -1;
    Runner runner(config, options);
    runner.reset(Pose2{Vec2(3.5, 0.0), vf::kPi});
    runner.input().curve = curve.get();
    runner.input().goal = goalAt(Vec2(0.0, 0.0));
    runner.run(25.0, [](const Cycle & c) { return c.out.settled; });
    ASSERT_TRUE(runner.cycles().back().out.settled);
    EXPECT_LE((runner.pose().p - Vec2(0.0, 0.0)).norm(), 0.6 * kXyTol);
  }
}

// U15 (upstream: equal switch distances gave a hard, unvalidated switch): D_in = D'_in = 0.7
// (λ = 0.47) is valid and gives a finite field with θ in {0, 1} on every cycle past a disc on the
// path; λ >= D_in is a validateConfig error and the Tracker refuses it.
TEST(UpstreamRegression, U15_EqualSwitchDistancesWellDefined)
{
  vf::CoreConfig config = tcContourConfig();
  config.obstacles.obstacle_full_switch_distance = 0.7;
  config.obstacles.obstacle_blend_start_distance = 0.7;
  ASSERT_TRUE(vf::validateConfig(config, std::nullopt).ok());
  ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
  world.addDisc(Vec2(2.0, 0.0), 0.3);
  world.commit();
  const auto curve = makePolyline({Vec2(-2.0, 0.0), Vec2(6.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(config, options);
  runner.reset(Pose2{Vec2(-1.0, 0.03), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(6.0, 0.0));
  runner.input().obstacles = world.map();
  runner.run(30.0, [](const Cycle & c) { return c.out.settled; });
  int active = 0;
  for (const Cycle & c : runner.cycles()) {
    EXPECT_TRUE(vf::isFinite(c.out.field.F)) << "t = " << c.rec.t;
    EXPECT_TRUE(vf::isFinite(c.rec.cmd)) << "t = " << c.rec.t;
    EXPECT_FALSE(Status::has(c.out.status, Status::kNonFinite)) << "t = " << c.rec.t;
    const double theta = c.out.field.contour.theta;
    EXPECT_TRUE(theta == 0.0 || theta == 1.0) << "θ = " << theta << " at t = " << c.rec.t;
    active += c.after.contour.active ? 1 : 0;
  }
  EXPECT_GT(active, 0);

  vf::CoreConfig bad = tcContourConfig();
  bad.obstacles.obstacle_contour_distance = 0.7;
  bad.obstacles.obstacle_full_switch_distance = 0.7;
  bad.obstacles.obstacle_blend_start_distance = 0.8;
  EXPECT_TRUE(vf::validateConfig(bad, std::nullopt).hasError(vf::issue::kContourOrder));
  EXPECT_THROW(vf::Tracker{bad}, vf::ConfigError);
}

// U16 (upstream: the contour tangent was undefined head-on and divided by zero at D_o = ε):
// an exact head-on approach (robot on y = 0, disc symmetric about the path) gives finite fields,
// no kNonFinite and exactly one episode with a definite, constant σ_c; and a probe exactly at
// D_o = λ (single nearest site, so the bilinear query is exact) is finite with the episode's side.
TEST(UpstreamRegression, U16_HeadOnObstacleFinite)
{
  ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
  world.addDisc(Vec2(2.0, 0.0), 0.3);
  world.commit();
  const auto curve = makePolyline({Vec2(-2.0, 0.0), Vec2(6.0, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(-1.0, 0.0), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(6.0, 0.0));
  runner.input().obstacles = world.map();
  bool probed = false;
  const double lambda = runner.tracker().config().obstacles.obstacle_contour_distance;
  for (int k = 0; k < 600; ++k) {
    const Cycle & c = runner.step();
    ASSERT_TRUE(vf::isFinite(c.out.field.F)) << "t = " << c.rec.t;
    ASSERT_TRUE(vf::isFinite(c.rec.cmd)) << "t = " << c.rec.t;
    ASSERT_FALSE(Status::has(c.out.status, Status::kNonFinite)) << "t = " << c.rec.t;
    if (!probed && c.after.contour.active) {
      probed = true;
      const Vec2 x(1.7 - lambda, 0.0);  // the nearest site centre is (1.7, 0)
      ASSERT_NEAR(world.distanceToSites(x), lambda, 1e-12);
      const vf::ProbeSample p =
        runner.tracker().probe(x, c.input, runner.sim().state(), runner.sim().scratch());
      EXPECT_TRUE(vf::isFinite(p.F));
      EXPECT_TRUE(std::isfinite(p.psi_d));
      EXPECT_TRUE(p.contour_active);
      EXPECT_EQ(p.sigma_c, c.after.contour.sigma_c);
    }
    if (c.out.settled) {
      break;
    }
  }
  EXPECT_TRUE(probed) << "no episode started";
  const std::vector<Episode> eps = episodes(runner.cycles());
  ASSERT_EQ(eps.size(), 1U);
  EXPECT_TRUE(eps[0].sigma_c == 1 || eps[0].sigma_c == -1);
  EXPECT_TRUE(eps[0].sigma_constant);
  EXPECT_GT(runner.pose().p.x(), 2.3) << "did not pass the disc";
}

// U17 (upstream: the hard Φ/Ψ switch chattered and the side flipped): a box centred on the path
// with a wall on one side, gaps 0.3, 0.45 and 0.8 m: exactly one episode, constant σ_c, no n_o
// reversal and no kBlocked (shares the O35 scenario).
TEST(UpstreamRegression, U17_SingleEpisodeNoChatter)
{
  for (const double gap : {0.3, 0.45, 0.8}) {
    SCOPED_TRACE("gap " + std::to_string(gap));
    const BoxWallResult r = runBoxWithWall(gap, 1, 0.01);
    ASSERT_EQ(r.eps.size(), 1U);
    EXPECT_TRUE(r.eps[0].sigma_constant);
    EXPECT_EQ(r.n_o_reversals, 0);
    EXPECT_EQ(r.blocked_cycles, 0);
  }
}

// U18 (upstream: the obstacle term used the robot position, not the evaluation point):
// (a) the FL start test uses D_o at p_c: the control point is 0.78 m from the disc (inside D'_in)
//     while base_link is 0.98 m away; one step from the initial state starts an episode, and the
//     reported D_o is the one at p_c;
// (b) probes at two points at different distances from the disc each use their own obstacle
//     term (ProbeSample has no D_o, see the report): far away the probe returns the shaped path
//     field (θ = 1); 0.51 m from the disc it returns the contour field Ψ computed here from the
//     frozen K (StepScratch::k_mask → EDT_K) at that point, with this cycle's σ_c and γ.
TEST(UpstreamRegression, U18_ObstacleTermUsesEvalPoint)
{
  ObstacleWorld world = centredWorld(-2.5, 7.0, -2.5, 2.5);
  world.addDisc(Vec2(2.0, 0.0), 0.3);
  world.commit();
  const auto curve = makePolyline({Vec2(-2.0, 0.0), Vec2(6.0, 0.0)}, false);
  const vf::Tracker tracker(tcContourConfig());
  vf::StepScratch scratch;
  vf::TrackerInput input;
  input.pose = Pose2{Vec2(1.7 - 0.78 - kD, 0.0), 0.0};
  input.measured = Twist2{0.3, 0.0, 0.0};
  input.dt = kPeriod;
  input.curve = curve.get();
  input.goal = goalAt(Vec2(6.0, 0.0));
  input.obstacles = world.map();
  vf::TrackerState state = vf::Tracker::initialState();
  const vf::TrackerOutput out = tracker.step(input, state, &state, scratch);
  ASSERT_LE((out.field.x_eval - Vec2(1.7 - 0.78, 0.0)).norm(), 1e-12);
  ASSERT_GT(world.distanceToSites(input.pose.p), kDinPrime + kHc);
  EXPECT_NEAR(out.field.contour.D_o, 0.78, 0.01);
  ASSERT_TRUE(state.contour.active) << "the start test must use D_o at p_c";
  ASSERT_TRUE(scratch.k_valid);

  // Expected fields at probe points, from the frozen K and this cycle's shaping.
  vf::EuclideanDistanceTransform edt_k;
  edt_k.build(
    world.grid(), vf::Span<const std::uint8_t>(scratch.k_mask.data(), scratch.k_mask.size()));
  const vf::DistanceField field(tracker.gain(), kVr, 1, 0.1);
  const vf::ObstacleConfig & oc = tracker.config().obstacles;
  const auto shaped_path_field = [&](const Vec2 & x) {
    vf::EvalContext ctx;
    const vf::ClosestPoint cp = curve->closestPoint(x, ctx, vf::CurveHint{});
    vf::FieldSample s;
    field.evaluate(x, cp, Vec2::Zero(), &s);
    return Vec2(out.field.gamma * s.Phi);
  };
  // Far point: D_o > D'_in + h_c, so F = Φ_γ.
  const Vec2 x1(-0.5, 1.5);
  ASSERT_GT(world.distanceToSites(x1), kDinPrime + kHc);
  const vf::ProbeSample p1 = tracker.probe(x1, input, state, scratch);
  EXPECT_LE((p1.F - shaped_path_field(x1)).norm(), 1e-12);
  EXPECT_NEAR(p1.D, 1.5, 1e-9);
  // Near point: D_o ≈ 0.51 < D_in, field against the obstacle, so F = Ψ(x2).
  const Vec2 x2(1.2, 0.1);
  const vf::ObstacleQuery q = edt_k.query(x2);
  ASSERT_TRUE(q.valid);
  ASSERT_LT(q.D_o, oc.obstacle_full_switch_distance);
  const Vec2 phi2 = shaped_path_field(x2);
  ASSERT_LT(q.n_o.dot(phi2), 0.0);
  const Vec2 psi2 = vf::contourField(
    q.n_o, q.D_o, oc.obstacle_contour_distance, state.contour.sigma_c, tracker.gain(), phi2.norm());
  const Vec2 expected2 = vf::blendContour(
    phi2, psi2,
    vf::contourBlendWeight(
      q.D_o, oc.obstacle_full_switch_distance, oc.obstacle_blend_start_distance));
  const vf::ProbeSample p2 = tracker.probe(x2, input, state, scratch);
  EXPECT_TRUE(p2.contour_active);
  EXPECT_EQ(p2.sigma_c, state.contour.sigma_c);
  EXPECT_LE((p2.F - expected2).norm(), 1e-9);
  EXPECT_NEAR(p2.D, 0.1, 1e-9);
}

// U24 (upstream: obstacle points never expired): a box on the path is removed 1 s into the
// episode: the episode ends on the first cycle without the box, no new episode starts, the robot
// rejoins the curve (|y| < 0.05 m) and settles at the goal. Timeout 30 s.
TEST(UpstreamRegression, U24_RemovedObstacleReleasesContour)
{
  ObstacleWorld world(Vec2(-2.0, -2.5), 10.0, 5.0);
  const int box = world.addBox(Vec2(2.7, -0.3), Vec2(3.3, 0.3));
  world.commit();
  const auto curve = makePolyline({Vec2(-1.0, 0.0), Vec2(6.5, 0.0)}, false);
  vf::sim::SimulationOptions options;
  options.stop_when_settled = false;
  Runner runner(tcContourConfig(), options);
  runner.reset(Pose2{Vec2(0.0, 0.02), 0.0});
  runner.input().curve = curve.get();
  runner.input().goal = goalAt(Vec2(6.5, 0.0));
  runner.input().obstacles = world.map();
  double episode_start = -1.0;
  std::size_t removed_at = 0;
  for (std::size_t k = 0; k < 600; ++k) {
    if (removed_at == 0 && episode_start >= 0.0 && runner.time() >= episode_start + 1.0) {
      world.setEnabled(box, false);
      world.commit();
      removed_at = k;
    }
    const Cycle & c = runner.step();
    if (episode_start < 0.0 && c.after.contour.active) {
      episode_start = c.rec.t;
    }
    if (c.out.settled) {
      break;
    }
  }
  const auto & cs = runner.cycles();
  ASSERT_GT(removed_at, 0U) << "no episode started";
  ASSERT_LT(removed_at, cs.size());
  EXPECT_TRUE(cs[removed_at].before.contour.active);
  EXPECT_FALSE(cs[removed_at].after.contour.active);
  bool rejoined = false;
  for (std::size_t k = removed_at; k < cs.size(); ++k) {
    EXPECT_FALSE(cs[k].after.contour.active) << "new episode at t = " << cs[k].rec.t;
    rejoined = rejoined || std::abs(cs[k].rec.pose.p.y()) < 0.05;
  }
  EXPECT_TRUE(rejoined);
  EXPECT_TRUE(cs.back().out.settled);
}

namespace
{

// U28: a U-turn, legs 0.6 m apart joined by a semicircle of radius 0.3 m (vertices every 2.5 cm).
std::vector<Vec2> uTurn()
{
  std::vector<Vec2> points = linePoints(Vec2(-2.0, 0.0), Vec2(0.0, 0.0), 0.025);
  appendPoints(&points, arcPoints(Vec2(0.0, 0.3), 0.3, -vf::kHalfPi, vf::kHalfPi, 0.025));
  appendPoints(&points, linePoints(Vec2(0.0, 0.6), Vec2(-2.0, 0.6), 0.025));
  return points;
}

}  // namespace

// U28 closed-loop part (upstream: the finite-difference feedforward spiked when the nearest sample
// switched): HT with a WindowScan over the whole U-turn (exact closest points, so probes see both
// legs). The jump test is re-derived with Tracker::probe at x_eval ± δh after the step, with δ of
// math_spec §7.4 as revised in review (max(0.02, |v|Δt, min(ℓ̄, 0.10)), ℓ̄ = 2.5 cm here):
// whenever |s*₊ − s*₋| > 2δ + 2 max(D₊, D₋) + 0.01 (a 1 cm margin), ψ̇_d = 0 that cycle, and
// |ω| <= ω_max always. A constructed cycle inside the U, crossing its bisector at
// 100° heading, guarantees one straddle; then a closed loop around the U-turn. Timeout 30 s.
TEST(UpstreamRegression, U28_FeedforwardIgnoresClosestPointJump_ClosedLoop)
{
  const std::vector<Vec2> points = uTurn();
  vf::PolylineCurve curve(vf::SearchPolicy::windowScan(-1.0, 10.0));
  ASSERT_EQ(
    curve.assign(vf::Span<const Vec2>(points.data(), points.size()), false), vf::AssignStatus::kOk);
  const vf::CoreConfig config =
    tcConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking);
  const vf::Tracker tracker(config);
  // Probe offset of math_spec §7.4 as revised in review: δ = max(0.02, |v|Δt, min(ℓ̄, 0.10)) with
  // ℓ̄ = L / segment count (Curve::vertexSpacing()); |v|Δt <= v_r Δt = 0.02 under TC.
  const double spacing = polylineLength(points, false) / static_cast<double>(points.size() - 1);
  const double delta = std::max(0.02, std::min(spacing, 0.10));
  const auto straddles = [&tracker, delta](
                           const vf::TrackerInput & input, const vf::TrackerState & state,
                           const vf::StepScratch & scratch, const vf::TrackerOutput & out) {
    const Vec2 h = vf::heading(input.pose.yaw);
    const vf::ProbeSample plus = tracker.probe(out.field.x_eval + delta * h, input, state, scratch);
    const vf::ProbeSample minus =
      tracker.probe(out.field.x_eval - delta * h, input, state, scratch);
    if (std::isnan(plus.psi_d) || std::isnan(minus.psi_d)) {
      return false;
    }
    return std::abs(plus.s_star - minus.s_star) >
           2.0 * delta + 2.0 * std::max(plus.D, minus.D) + 0.01;
  };
  {
    SCOPED_TRACE("constructed straddle");
    vf::StepScratch scratch;
    vf::TrackerInput input;
    input.pose = Pose2{Vec2(-0.05, 0.3001), 100.0 * vf::kPi / 180.0};
    input.dt = kPeriod;
    input.curve = &curve;
    input.goal = goalAt(points.back());
    vf::TrackerState state = vf::Tracker::initialState();
    const vf::TrackerOutput out = tracker.step(input, state, &state, scratch);
    ASSERT_EQ(out.phase, Phase::kFollow);
    ASSERT_TRUE(straddles(input, state, scratch, out)) << "the probes do not straddle the legs";
    EXPECT_EQ(out.field.psi_d_dot, 0.0);
    EXPECT_LE(std::abs(out.cmd.wz), kWmax + 1e-12);
    EXPECT_LE(std::abs(out.cmd_target.wz), kWmax + 1e-12);
  }
  {
    SCOPED_TRACE("closed loop around the U-turn");
    vf::sim::SimulationOptions options;
    options.stop_when_settled = false;
    Runner runner(config, options);
    runner.reset(Pose2{Vec2(-1.5, 0.0), 0.0});
    runner.input().curve = &curve;
    runner.input().goal = goalAt(points.back());
    for (int k = 0; k < 600; ++k) {
      const Cycle & c = runner.step();
      EXPECT_LE(std::abs(c.rec.cmd.wz), kWmax + 1e-12) << "t = " << c.rec.t;
      EXPECT_LE(std::abs(c.out.cmd_target.wz), kWmax + 1e-12) << "t = " << c.rec.t;
      if (
        vf::isTranslationPhase(c.out.phase) &&
        straddles(c.input, runner.sim().state(), runner.sim().scratch(), c.out)) {
        EXPECT_EQ(c.out.field.psi_d_dot, 0.0)
          << "feedforward kept across a jump at t = " << c.rec.t;
      }
      if (c.out.settled) {
        break;
      }
    }
  }
}
