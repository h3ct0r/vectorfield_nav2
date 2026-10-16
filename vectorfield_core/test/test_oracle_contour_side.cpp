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

// Contour oracle O35, side probe (math_spec §6.4, §12.5).

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "test_support.hpp"

namespace
{

using namespace vf_test;  // NOLINT(build/namespaces)

}  // namespace

// Gaps 0.3 and 0.6 m here; 0.71 and 0.8 m in test_oracle_contour_side_wide.cpp.
// O35 Side probe (math_spec §6.4 "Side probe"): a 0.6 m box centred on the path with a parallel
// wall at gaps 0.3 (merged with the box in the dilated mask), 0.6 (wall side collides), 0.71
// (about 0.02–0.04 m wall-side clearance) and 0.8 m, wall on either side, ±1 cm initial lateral
// error: in every run the open side is chosen (the base crosses the box centre on the side
// opposite to the wall, outside the box), exactly one episode, no kBlocked, no n_o reversal during
// the episode, σ_c constant. Each run stops once past x = 4.6 with no episode; timeout 25 s.
TEST(OracleContour, O35_SideProbeChoosesOpenSide_Gaps030And060)
{
  for (const double gap : {0.3, 0.6}) {
    for (const int wall_side : {1, -1}) {
      for (const double error : {0.01, -0.01}) {
        SCOPED_TRACE(
          "gap " + std::to_string(gap) + ", wall side " + std::to_string(wall_side) +
          ", initial error " + std::to_string(error));
        const BoxWallResult r = runBoxWithWall(gap, wall_side, error);
        ASSERT_FALSE(std::isnan(r.y_at_box)) << "never passed the box";
        EXPECT_LT(r.y_at_box * wall_side, -0.3) << "passed on the wall side: y = " << r.y_at_box;
        ASSERT_EQ(r.eps.size(), 1U);
        EXPECT_TRUE(r.eps[0].sigma_constant);
        EXPECT_EQ(r.blocked_cycles, 0);
        EXPECT_EQ(r.n_o_reversals, 0);
      }
    }
  }
}
