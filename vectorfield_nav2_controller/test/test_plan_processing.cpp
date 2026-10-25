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

// Pure unit test of the raw-plan intake: NavFn-style tails and sharp
// forward corners never read as a cusp while a real reversal longer than 2ℓ_c always does, and
// the end-tangent rules (walk back ℓ_c, keep the stored tangent for the same goal, none for a
// one-pose plan). No ROS.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "plan_generators.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_nav2_controller/plan_processing.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::intakePlan;
using vectorfield_nav2_controller::PlanIntake;
using vectorfield_nav2_controller::PlanIntakeState;
using vectorfield_nav2_controller::test::navfnTail;
using vectorfield_nav2_controller::test::reversingPlan;
using vectorfield_nav2_controller::test::straightPlan;

constexpr double kDeg = vf::kPi / 180.0;

/// The 8 sub-cell goal offsets of ±0.5 cell (the 3×3 neighbourhood without its centre).
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
constexpr std::array<double, 4> kApproachLengths = {1.0, 1.0125, 1.025, 1.0375};
constexpr std::array<double, 2> kResolutions = {0.05, 0.10};
constexpr int kHeadings = 16;

/// intakePlan() of one plan with its own work buffers.
PlanIntake intake(const std::vector<vf::Pose2> & plan, double chord_length, PlanIntakeState * state)
{
  std::vector<vf::Vec2> positions;
  std::vector<vf::Vec2> decimated;
  std::vector<std::size_t> cusps;
  return intakePlan(plan, chord_length, state, &positions, &decimated, &cusps);
}

double angleBetween(const vf::Vec2 & a, const vf::Vec2 & b)
{
  return std::abs(vf::wrapAngle(vf::angleOf(b) - vf::angleOf(a)));
}

TEST(NavfnTails, NoTailReadsAsACuspAndTheEndTangentFollowsTheApproach)
{
  std::size_t plans = 0;
  for (const double resolution : kResolutions) {
    const double chord = vf::chordLength(resolution);
    for (int k = 0; k < kHeadings; ++k) {
      const double heading = vf::wrapAngle(k * 2.0 * vf::kPi / kHeadings);
      for (const auto & offset : kGoalOffsets) {
        for (const double length : kApproachLengths) {
          const vf::Pose2 goal{vf::Vec2(1.3, -0.7), heading};
          const std::vector<vf::Pose2> plan =
            navfnTail(goal, vf::Vec2(offset[0], offset[1]), heading, length, resolution);
          PlanIntakeState state;
          const PlanIntake result = intake(plan, chord, &state);
          ++plans;
          EXPECT_FALSE(result.cusp)
            << "res " << resolution << " heading " << heading / kDeg << " offset (" << offset[0]
            << ", " << offset[1] << ") length " << length;
          ASSERT_TRUE(result.has_end_tangent);
          // The goal lies at most √2/2 cell off the approach line and the tangent is taken at
          // least ℓ_c = max(0.10, 2·res) back: asin(0.354) ≈ 21°.
          EXPECT_LE(angleBetween(state.end_tangent, vf::heading(heading)), 22.0 * kDeg)
            << "res " << resolution << " heading " << heading / kDeg;
          EXPECT_FALSE(result.sparse);
        }
      }
    }
  }
  EXPECT_EQ(plans, kResolutions.size() * kHeadings * kGoalOffsets.size() * kApproachLengths.size());
}

TEST(Reversals, AReversalLongerThanTwoChordsIsAlwaysACusp)
{
  // 2ℓ_c is 0.2 m at res 0.05 and 0.4 m at res 0.10; the reversals are 0.3 m and 0.45 m.
  const std::array<double, 2> reverse_lengths = {0.3, 0.45};
  for (std::size_t r = 0; r < kResolutions.size(); ++r) {
    const double resolution = kResolutions[r];
    const double chord = vf::chordLength(resolution);
    ASSERT_GT(reverse_lengths[r], 2.0 * chord);
    for (int k = 0; k < kHeadings; ++k) {
      const double heading = k * 2.0 * vf::kPi / kHeadings;
      for (const auto & offset : kGoalOffsets) {
        for (const double forward : kApproachLengths) {
          for (const double spacing : {0.5 * resolution, resolution}) {
            const vf::Pose2 start{
              vf::Vec2(-0.4 + offset[0] * resolution, 0.2 + offset[1] * resolution), heading};
            PlanIntakeState state;
            const PlanIntake result =
              intake(reversingPlan(start, forward, reverse_lengths[r], spacing), chord, &state);
            EXPECT_TRUE(result.cusp) << "res " << resolution << " heading " << heading / kDeg
                                     << " forward " << forward << " spacing " << spacing;
          }
        }
      }
    }
  }
}

TEST(Corners, ASharpForwardCornerIsNoCuspButUnsmoothed)
{
  // NavigateThroughPoses through a waypoint whose next leg turns by 100°..170°: NavFn's identity
  // orientations, or orientations along the legs (straightPlan). No cusp (math_spec §2.8).
  for (const double resolution : kResolutions) {
    const double chord = vf::chordLength(resolution);
    for (int turn = 100; turn <= 170; turn += 10) {
      for (int k = 0; k < kHeadings; ++k) {
        const double first = k * 2.0 * vf::kPi / kHeadings;
        const double second = first + turn * kDeg;
        const vf::Vec2 corner(0.31, -0.12);
        for (const bool oriented : {false, true}) {
          std::vector<vf::Pose2> plan =
            straightPlan(corner - vf::heading(first), corner, 0.5 * resolution, second);
          const std::vector<vf::Pose2> leg =
            straightPlan(corner, corner + vf::heading(second), 0.5 * resolution, second);
          plan.insert(plan.end(), leg.begin() + 1, leg.end());
          if (!oriented) {
            for (std::size_t i = 0; i + 1 < plan.size(); ++i) {
              plan[i].yaw = 0.0;
            }
          }
          PlanIntakeState state;
          const PlanIntake result = intake(plan, chord, &state);
          EXPECT_FALSE(result.cusp) << "res " << resolution << " turn " << turn << " heading "
                                    << first / kDeg << " oriented " << oriented;
          EXPECT_TRUE(result.unsmoothed);
        }
      }
    }
  }
}

TEST(EndTangent, WalksBackToTheFirstPoseAtLeastOneChordFromTheGoal)
{
  const double chord = vf::chordLength(0.05);  // 0.10 m
  // A diagonal last step: (0.10, 0) is 0.05 m and (0.05, 0) 0.089 m from the goal, so the walk
  // stops at (0, 0), 0.136 m away.
  const std::vector<vf::Pose2> diagonal = {
    vf::Pose2{vf::Vec2(-0.05, 0.0), 0.0}, vf::Pose2{vf::Vec2(0.0, 0.0), 0.0},
    vf::Pose2{vf::Vec2(0.05, 0.0), 0.0}, vf::Pose2{vf::Vec2(0.10, 0.0), 0.0},
    vf::Pose2{vf::Vec2(0.13, 0.04), 0.0}};
  PlanIntakeState state;
  PlanIntake result = intake(diagonal, chord, &state);
  ASSERT_TRUE(result.has_end_tangent);
  EXPECT_NEAR(state.end_tangent.x(), 0.13 / std::hypot(0.13, 0.04), 1e-12);
  EXPECT_NEAR(state.end_tangent.y(), 0.04 / std::hypot(0.13, 0.04), 1e-12);

  // An L-shaped plan: exactly ℓ_c back lies on the second leg.
  std::vector<vf::Pose2> corner = straightPlan(vf::Vec2::Zero(), vf::Vec2(1.0, 0.0), 0.05, 0.0);
  const std::vector<vf::Pose2> leg =
    straightPlan(vf::Vec2(1.0, 0.0), vf::Vec2(1.0, 0.3), 0.05, 0.5 * vf::kPi);
  corner.insert(corner.end(), leg.begin() + 1, leg.end());
  state = PlanIntakeState{};
  result = intake(corner, chord, &state);
  ASSERT_TRUE(result.has_end_tangent);
  EXPECT_NEAR(state.end_tangent.x(), 0.0, 1e-12);
  EXPECT_NEAR(state.end_tangent.y(), 1.0, 1e-12);
}

TEST(EndTangent, ANearGoalReplanOfTheSameGoalKeepsTheStoredTangent)
{
  const double chord = vf::chordLength(0.05);
  const vf::Pose2 goal{vf::Vec2(1.0, 0.0), 0.0};
  PlanIntakeState state;
  PlanIntake result = intake(straightPlan(vf::Vec2::Zero(), goal.p, 0.05, goal.yaw), chord, &state);
  ASSERT_TRUE(result.new_goal);
  ASSERT_TRUE(result.has_end_tangent);
  const std::uint32_t seq = state.goal_seq;
  EXPECT_EQ(seq, 1U);
  EXPECT_NEAR(state.end_tangent.x(), 1.0, 1e-12);

  // The planner restarts at the robot, 4 cm beside the goal: no pose ℓ_c back, same goal (within
  // 1 cm and 0.01 rad), so the stored tangent stays.
  const std::vector<vf::Pose2> near = {
    vf::Pose2{vf::Vec2(0.97, 0.03), 0.0}, vf::Pose2{vf::Vec2(1.005, 0.0), 0.005}};
  result = intake(near, chord, &state);
  EXPECT_FALSE(result.new_goal);
  EXPECT_EQ(state.goal_seq, seq);
  ASSERT_TRUE(result.has_end_tangent);
  EXPECT_NEAR(state.end_tangent.x(), 1.0, 1e-12);
  EXPECT_NEAR(state.end_tangent.y(), 0.0, 1e-12);

  // The goal moves 5 cm: a new goal, whose only direction is from the first pose.
  const std::vector<vf::Pose2> moved = {
    vf::Pose2{vf::Vec2(0.97, 0.03), 0.0}, vf::Pose2{vf::Vec2(1.0, 0.05), 0.0}};
  result = intake(moved, chord, &state);
  EXPECT_TRUE(result.new_goal);
  EXPECT_EQ(state.goal_seq, seq + 1);
  ASSERT_TRUE(result.has_end_tangent);
  EXPECT_NEAR(state.end_tangent.x(), 0.03 / std::hypot(0.03, 0.02), 1e-12);
  EXPECT_NEAR(state.end_tangent.y(), 0.02 / std::hypot(0.03, 0.02), 1e-12);

  // A new goal whose first pose is within 1 mm of it: no direction at all.
  const std::vector<vf::Pose2> degenerate = {
    vf::Pose2{vf::Vec2(2.0, 0.0), 0.0}, vf::Pose2{vf::Vec2(2.0005, 0.0), 0.0}};
  result = intake(degenerate, chord, &state);
  EXPECT_TRUE(result.new_goal);
  EXPECT_FALSE(result.has_end_tangent);
}

TEST(EndTangent, AOnePosePlanHasNoneEvenForTheSameGoal)
{
  const double chord = vf::chordLength(0.05);
  const vf::Pose2 goal{vf::Vec2(1.0, 0.0), 0.0};
  PlanIntakeState state;
  ASSERT_TRUE(
    intake(straightPlan(vf::Vec2::Zero(), goal.p, 0.05, goal.yaw), chord, &state).has_end_tangent);
  const std::uint32_t seq = state.goal_seq;

  const PlanIntake result = intake({goal}, chord, &state);
  EXPECT_FALSE(result.new_goal);
  EXPECT_EQ(state.goal_seq, seq);
  EXPECT_FALSE(result.has_end_tangent);
  EXPECT_FALSE(state.has_end_tangent);
  EXPECT_FALSE(result.cusp);

  // An empty plan changes nothing.
  const PlanIntakeState before = state;
  EXPECT_TRUE(intake({}, chord, &state).empty);
  EXPECT_EQ(state.goal_seq, before.goal_seq);
  EXPECT_EQ(state.has_goal, before.has_goal);
}

}  // namespace
