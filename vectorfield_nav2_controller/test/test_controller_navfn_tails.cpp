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

// Layer 2 (plan §9), through the server harness: NavFn-style tails (nav2_api_matrix.md §9) driven
// to completion through the real path handler (LegacyPathHandler on Gen A/B, FeasiblePathHandler
// on Gen C). 8 sub-cell goal offsets × 16 approach headings, the robot starting 1 m before the goal,
// at xy_tol 0.05 and 0.25. Every goal: no InvalidPath, the window's end tangent within 30° of the
// approach every cycle (tail conditioning, math_spec §2.8), no kHeadingInfeasible; (a) a goal yaw
// 90° off the approach rotates only once settled (math_spec §9.1); (b) an aligned goal yaw ends
// within xy_tol, Ackermann with PositionGoalChecker included.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "harness_fixtures.hpp"
#include "vectorfield_core/status.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::modelOptions;
using vectorfield_nav2_controller::test::navfnTail;
using vectorfield_nav2_controller::test::ServerHarness;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

constexpr double kDeg = vf::kPi / 180.0;
constexpr int kHeadings = 16;
#ifdef VF_COVERAGE_BUILD
// The CI coverage build (Debug, --coverage) runs each goal about 100x slower: drive one offset
// per heading (k mod 8), so every heading and every offset is still covered (16 goals, not 128).
constexpr bool kSubset = true;
#else
constexpr bool kSubset = false;
#endif
constexpr std::array<std::array<double, 2>, 8> kGoalOffsets = {{
  {-0.5, -0.5},
  {-0.5, 0.0},
  {-0.5, 0.5},
  {0.0, -0.5},
  {0.0, 0.5},
  {0.5, -0.5},
  {0.5, 0.0},
  {0.5, 0.5},
}};
/// The goal cell's centre (odom = map), the costmap centre of modelOptions().
const vf::Vec2 kCellCentre(0.5, 0.0);
constexpr double kApproachLength = 1.0;
constexpr double kTangentLimit = 30.0 * kDeg;

/// (variant, xy_tol). Variants: "<law>_rotate" (goal yaw 90° off the approach, case a) and
/// "<law>_aligned" (case b), for the laws fl, ht and holonomic, plus "ackermann_aligned".
using TailCase = std::tuple<std::string, double>;

class NavfnTails : public ::testing::TestWithParam<TailCase>
{
};

HarnessOptions optionsFor(const std::string & variant, double xy_tol)
{
  HarnessOptions options;
  if (variant.rfind("fl_", 0) == 0) {
    options = modelOptions("differential", "feedback_linearization");
  } else if (variant.rfind("ht_", 0) == 0) {
    options = modelOptions("differential", "heading_tracking");
  } else if (variant.rfind("holonomic_", 0) == 0) {
    options = modelOptions("omnidirectional", "holonomic");
  } else {
    options = modelOptions("ackermann", "heading_tracking");
    options.goal_checker = "nav2_controller::PositionGoalChecker";
  }
  options.xy_goal_tolerance = xy_tol;
  return options;
}

TEST_P(NavfnTails, EveryOffsetAndHeadingEndsCleanly)
{
  const auto & [variant, xy_tol] = GetParam();
  const bool rotate = variant.find("_rotate") != std::string::npos;
  ServerHarness harness(optionsFor(variant, xy_tol));
  harness.configure();
  harness.activate();
  const double resolution = harness.options().costmap_resolution;

  std::size_t goals = 0;
  std::size_t failures = 0;
  for (int k = 0; k < kHeadings; ++k) {
    const double heading = vf::wrapAngle(k * 2.0 * vf::kPi / kHeadings);
    const vf::Vec2 direction = vf::heading(heading);
    for (std::size_t j = 0; j < kGoalOffsets.size(); ++j) {
      if (kSubset && j != static_cast<std::size_t>(k) % kGoalOffsets.size()) {
        continue;
      }
      const auto & offset_cells = kGoalOffsets[j];
      const vf::Vec2 offset(offset_cells[0], offset_cells[1]);
      const vf::Pose2 goal{
        kCellCentre + offset * resolution, vf::wrapAngle(heading + (rotate ? 0.5 * vf::kPi : 0.0))};
      std::ostringstream label;
      label << variant << " xy_tol " << xy_tol << " heading " << heading / kDeg << " offset ("
            << offset.x() << ", " << offset.y() << ")";

      // A pause of 4 periods (Humble's reset emulation), then the robot at the tail's start.
      for (int i = 0; i < 4; ++i) {
        harness.spinOnce();
      }
      harness.setRobotPose(vf::Pose2{kCellCentre - kApproachLength * direction, heading});
      const std::size_t begin = harness.history().size();
      harness.followPath(mapPlan(navfnTail(goal, offset, heading, kApproachLength, resolution)));
      const GoalStatus status = harness.runUntilDone(20.0);
      ++goals;

      bool ok = status == GoalStatus::kSucceeded;
      EXPECT_EQ(status, GoalStatus::kSucceeded)
        << label.str() << ": " << vectorfield_nav2_controller::test::describe(harness);
      const CycleRecord * first_rotation = nullptr;
      std::size_t computed = 0;
      for (std::size_t i = begin; i < harness.history().size(); ++i) {
        const CycleRecord & record = harness.history()[i];
        if (!record.computed) {
          continue;
        }
        ++computed;
        ok &= record.exception != "InvalidPath";
        EXPECT_NE(record.exception, "InvalidPath") << label.str() << ": " << record.message;
        const double tangent_error =
          std::abs(vf::wrapAngle(vf::angleOf(record.diagnostics.window_end_tangent) - heading));
        ok &= tangent_error <= kTangentLimit;
        EXPECT_LE(tangent_error, kTangentLimit) << label.str() << " at t = " << record.t;
        const bool infeasible =
          (record.diagnostics.output.status & vf::Status::kHeadingInfeasible) != 0U;
        ok &= !infeasible;
        EXPECT_FALSE(infeasible) << label.str() << " at t = " << record.t;
        if (
          first_rotation == nullptr &&
          record.diagnostics.output.phase == vf::Phase::kRotateToHeading) {
          first_rotation = &record;
        }
      }
      // Not vacuous: the 1 m − xy_tol to the tolerance edge take at least that long at
      // desired_speed 0.4 m/s.
      const auto min_cycles = static_cast<std::size_t>(
        (kApproachLength - xy_tol) / 0.4 * harness.options().controller_frequency);
      ok &= computed >= min_cycles;
      EXPECT_GE(computed, min_cycles) << label.str();
      if (rotate) {
        ok &= first_rotation != nullptr;
        EXPECT_NE(first_rotation, nullptr) << label.str() << ": ROTATE_TO_HEADING never started";
        if (first_rotation != nullptr) {
          const double distance = (first_rotation->pose_odom.p - goal.p).norm();
          ok &= distance <= 0.6 * xy_tol;
          EXPECT_LE(distance, 0.6 * xy_tol) << label.str();
        }
      } else {
        const double distance = (harness.history().back().pose_odom.p - goal.p).norm();
        ok &= distance <= xy_tol;
        EXPECT_LE(distance, xy_tol) << label.str();
      }
      if (!ok && ++failures >= 5) {
        FAIL() << "stopping after 5 failing goals of " << goals;
      }
    }
  }
  EXPECT_EQ(goals, static_cast<std::size_t>(kHeadings) * (kSubset ? 1U : kGoalOffsets.size()));
}

INSTANTIATE_TEST_SUITE_P(
  Layer2, NavfnTails,
  ::testing::Combine(
    ::testing::Values(
      "fl_rotate", "ht_rotate", "holonomic_rotate", "fl_aligned", "ht_aligned", "holonomic_aligned",
      "ackermann_aligned"),
    ::testing::Values(0.05, 0.25)),
  [](const ::testing::TestParamInfo<TailCase> & case_info) {
    return std::get<0>(case_info.param) + "_xy" +
           std::to_string(static_cast<int>(std::lround(100.0 * std::get<1>(case_info.param))));
  });

}  // namespace
