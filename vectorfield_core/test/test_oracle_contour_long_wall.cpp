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

// Contour oracle O35 with a long parallel wall (math_spec §6.4, §12.5):
// the wall runs alongside the whole approach instead of only beside the box.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

}  // namespace

// O35 with a long parallel wall (coordinator request): the wall runs from x = −1 to +5 m, i.e.
// alongside the whole approach, at gaps 0.3 (merged with the box in the dilated mask) and 0.45 m,
// on either side, ±1 cm initial error. Expected per plan intent: one episode, constant σ_c, no
// n_o reversal, no kBlocked, the open side. Every episode start and end is printed (time, base
// position, D_o, n_oᵀΦ/‖Φ‖ with the nominal Φ) for the design review.
TEST(OracleContour, O35_LongParallelWall)
{
  for (const double gap : {0.3, 0.45}) {
    for (const int wall_side : {1, -1}) {
      for (const double error : {0.01, -0.01}) {
        const std::string name = "gap " + std::to_string(gap) + ", wall side " +
                                 std::to_string(wall_side) + ", initial error " +
                                 std::to_string(error);
        SCOPED_TRACE(name);
        const BoxWallResult r = runBoxWithWall(gap, wall_side, error, 25.0, -1.0, 5.0);
        std::printf(
          "[long wall] %s: %zu episode(s), y at box %.3f, n_o reversals %d (max turn %.1f deg), "
          "kBlocked cycles %d\n",
          name.c_str(), r.eps.size(), r.y_at_box, r.n_o_reversals, r.max_n_o_turn_deg,
          r.blocked_cycles);
        for (const Episode & e : r.eps) {
          const Cycle & s0 = r.cycles[e.start];
          const std::size_t last = e.end > e.start ? e.end - 1 : e.start;
          const Cycle & s1 = r.cycles[last];
          const auto ratio = [](const Cycle & c) {
            const double norm = c.out.field.Phi.norm();
            return norm > 0.0 ? c.out.field.contour.n_o.dot(c.out.field.Phi) / norm : 0.0;
          };
          std::printf(
            "  episode σ_c %+d: start t %.2f base (%.3f, %.3f) D_o %.3f n_o·Φ/‖Φ‖ %+.3f | last "
            "active t %.2f base (%.3f, %.3f) D_o %.3f n_o·Φ/‖Φ‖ %+.3f%s\n",
            e.sigma_c, s0.rec.t, s0.rec.pose.p.x(), s0.rec.pose.p.y(), s0.out.field.contour.D_o,
            ratio(s0), s1.rec.t, s1.rec.pose.p.x(), s1.rec.pose.p.y(), s1.out.field.contour.D_o,
            ratio(s1), e.end < r.cycles.size() ? "" : " (still active)");
          if (e.end < r.cycles.size()) {
            const Cycle & x = r.cycles[e.end];
            std::printf(
              "    ended at t %.2f: gate_open %d, k_valid %d, D_o %.3f, n_o·Φ/‖Φ‖ %+.3f, phase %s, "
              "status 0x%x (kBlocked %d)\n",
              x.rec.t, x.after.contour.gate_open ? 1 : 0, x.after.contour.k_valid ? 1 : 0,
              x.out.field.contour.D_o, ratio(x), vf::phaseName(x.out.phase), x.out.status,
              Status::has(x.out.status, Status::kBlocked) ? 1 : 0);
          }
        }
        EXPECT_FALSE(std::isnan(r.y_at_box)) << "never passed the box";
        if (!std::isnan(r.y_at_box)) {
          EXPECT_LT(r.y_at_box * wall_side, -0.3) << "passed on the wall side: y = " << r.y_at_box;
        }
        EXPECT_EQ(r.eps.size(), 1U);
        for (const Episode & e : r.eps) {
          EXPECT_TRUE(e.sigma_constant);
        }
        EXPECT_EQ(r.blocked_cycles, 0);
        EXPECT_EQ(r.n_o_reversals, 0);
      }
    }
  }
}
