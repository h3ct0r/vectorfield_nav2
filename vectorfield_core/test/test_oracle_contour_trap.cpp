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

// Contour oracle O31, the trap of the unlatched rule and its removal by the latch (math_spec §6.3,
// §6.4, §12.5).

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

// Unlatched rule (3.67) with the thesis fallback (keep the previous side where (Jn_o)ᵀΦ = 0)
// and activation (3.72), as a point integrator ẋ = F, Euler 1e-3 s, on the library's path field
// and contour building blocks, with an exact (analytic) disc. Returns the maximum x reached.
double unlatchedPointRun(double R_o, double y0, double duration)
{
  const auto path = makePolyline({Vec2(-10.0, 0.0), Vec2(10.0, 0.0)}, false);
  const vf::GainFunction gain(vf::GainType::kAtan, kKf);
  const vf::DistanceField field(gain, kVr, 1, 0.1);
  const double rho = R_o + kLambda;
  Vec2 x(-rho - 1.5, y0);
  vf::CurveHint hint;
  int side = 1;
  double max_x = x.x();
  constexpr double h = 1e-3;
  const int steps = static_cast<int>(std::lround(duration / h));
  for (int k = 0; k < steps; ++k) {
    vf::EvalContext ctx;
    const vf::ClosestPoint cp = path->closestPoint(x, ctx, hint);
    hint = vf::makeHint(cp);
    vf::FieldSample s;
    field.evaluate(x, cp, Vec2::Zero(), &s);
    const Vec2 phi = s.Phi;
    const double r = x.norm();
    const double D_o = r - R_o;
    const Vec2 n_o = x / r;
    Vec2 F = phi;
    if (n_o.dot(phi) < 0.0 && D_o <= kDinPrime) {
      const double jn = vf::J(n_o).dot(phi);
      if (jn != 0.0) {
        side = jn > 0.0 ? 1 : -1;
      }
      const Vec2 psi = vf::contourField(n_o, D_o, kLambda, side, gain, phi.norm());
      F = vf::blendContour(phi, psi, vf::contourBlendWeight(D_o, kDin, kDinPrime));
    }
    x += h * F;
    max_x = std::max(max_x, x.x());
  }
  return max_x;
}

}  // namespace

// O31 Trap, unlatched rule (math_spec §6.3, §12.5). The Tracker always latches σ_c, so the
// unlatched rule (3.67) is not reachable through its API (testability gap, see the report); it is
// reproduced here on the field and contour building blocks: a disc centred on a straight path
// with k_f(R_o + λ) in {1, 2.5, 4}, start 1 cm off the path: the unlatched rule passes the first
// and traps the last two (the point never passes the disc's far side within 20 s).
TEST(OracleContour, O31_UnlatchedRuleTraps_FieldLevel)
{
  const double rho_kf[] = {1.0, 2.5, 4.0};
  for (const double kf_rho : rho_kf) {
    const double R_o = kf_rho / kKf - kLambda;
    SCOPED_TRACE("k_f(R_o + λ) = " + std::to_string(kf_rho));
    const double max_x = unlatchedPointRun(R_o, 0.01, 20.0);
    if (kf_rho < vf::kHalfPi) {
      EXPECT_GT(max_x, R_o) << "no basin: the unlatched rule must pass";
    } else {
      EXPECT_LT(max_x, R_o) << "inside the trap basin the unlatched rule must not pass";
    }
  }
}

// O31 Trap, latched rule, closed loop (math_spec §6.4): FL, exact head-on start (y = 0) at a disc
// centred on the path with k_f(R_o + λ) in {1, 2.5, 4} (R_o = 0.03, 0.78, 1.53): the robot passes
// the disc's far side, rejoins the path (|y| < 0.05 afterwards), with at most one ROTATE_TO_FIELD
// entry per episode. obstacle_max_contour_length is raised to 10 m because the latched episode at
// k_f ρ = 4 is about 5.1 m long (point-dynamics reference), above TC's 5 m watchdog, which is not
// under test here. Each run stops once rejoined; timeout 45 s.
TEST(OracleContour, O31_LatchedRulePassesAndRejoins)
{
  for (const double R_o : {0.03, 0.78, 1.53}) {
    SCOPED_TRACE("R_o = " + std::to_string(R_o));
    vf::CoreConfig config = tcContourConfig();
    config.obstacles.obstacle_max_contour_length = 10.0;
    ObstacleWorld world = centredWorld(-2.5, 7.0, -3.2, 3.2);
    world.addDisc(Vec2(2.0, 0.0), R_o, kDiscTag);
    world.commit();
    ASSERT_GE(world.siteCount(), 1U);
    // Stop once the base has passed the far side and is back on the path with no episode.
    bool passed = false;
    const auto back_on_path = [&passed, R_o](const Cycle & c) {
      passed = passed || c.rec.pose.p.x() > 2.0 + R_o;
      return passed && std::abs(c.rec.pose.p.y()) < 0.05 && !c.after.contour.active;
    };
    const StraightRun r =
      runStraight(config, world, Pose2{Vec2(-1.0, 0.0), 0.0}, 6.5, 45.0, {}, back_on_path);
    ASSERT_GE(r.eps.size(), 1U) << "no contour episode";
    const std::size_t k_pass =
      firstIndex(r.cycles, [&](const Cycle & c) { return c.rec.pose.p.x() > 2.0 + R_o; });
    ASSERT_LT(k_pass, r.cycles.size()) << "never passed the far side";
    bool rejoined = false;
    for (std::size_t k = k_pass; k < r.cycles.size(); ++k) {
      rejoined = rejoined || std::abs(r.cycles[k].rec.pose.p.y()) < 0.05;
    }
    EXPECT_TRUE(rejoined);
    for (const Episode & e : r.eps) {
      int entries = 0;
      for (std::size_t k = e.start; k < e.end; ++k) {
        const Phase previous = k == 0 ? r.cycles[0].before.phase : r.cycles[k - 1].out.phase;
        entries +=
          r.cycles[k].out.phase == Phase::kRotateToField && previous != Phase::kRotateToField;
      }
      EXPECT_LE(entries, 1) << "episode starting at t = " << r.cycles[e.start].rec.t;
      EXPECT_TRUE(e.sigma_constant);
    }
    EXPECT_FALSE(anyStatus(r.cycles, Status::kContourWatchdog));
  }
}
