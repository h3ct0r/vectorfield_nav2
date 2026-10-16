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

// FieldState.msg constants equal the core Phase and Status values (plan §4.1, §7).
//
// The real check is compile time: including field_state.hpp evaluates its static_asserts. The
// tests below repeat the comparison at run time, so a failure is also visible in the test report,
// and check that the message constant names match the core's phase and status names.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "vectorfield_core/status.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_msgs/msg/field_state.hpp"
#include "vectorfield_ros/field_state.hpp"

namespace
{

using vectorfield_core::Phase;
using vectorfield_core::Status;
using FieldState = vectorfield_msgs::msg::FieldState;

struct PhaseCase
{
  std::uint8_t message_value;
  Phase phase;
  const char * name;
};

struct StatusCase
{
  std::uint32_t message_value;
  std::uint32_t status;
  const char * name;
};

TEST(FieldStateConstants, PhasesEqualCorePhases)
{
  const PhaseCase cases[] = {
    {FieldState::PHASE_ROTATE_TO_FIELD, Phase::kRotateToField, "ROTATE_TO_FIELD"},
    {FieldState::PHASE_FOLLOW, Phase::kFollow, "FOLLOW"},
    {FieldState::PHASE_FINAL_APPROACH, Phase::kFinalApproach, "FINAL_APPROACH"},
    {FieldState::PHASE_ROTATE_TO_HEADING, Phase::kRotateToHeading, "ROTATE_TO_HEADING"},
    {FieldState::PHASE_STOPPED, Phase::kStopped, "STOPPED"},
  };
  for (const PhaseCase & c : cases) {
    EXPECT_EQ(c.message_value, vectorfield_core::toUnderlying(c.phase)) << c.name;
    EXPECT_EQ(std::string(vectorfield_core::phaseName(c.phase)), c.name);
  }
}

TEST(FieldStateConstants, StatusBitsEqualCoreStatus)
{
  const StatusCase cases[] = {
    {FieldState::STATUS_NON_FINITE, Status::kNonFinite, "NON_FINITE"},
    {FieldState::STATUS_ASSUMPTION3_CLAMPED, Status::kAssumption3Clamped, "ASSUMPTION3_CLAMPED"},
    {FieldState::STATUS_AMBIGUOUS_CLOSEST, Status::kAmbiguousClosest, "AMBIGUOUS_CLOSEST"},
    {FieldState::STATUS_HAIRPIN, Status::kHairpin, "HAIRPIN"},
    {FieldState::STATUS_CURVATURE_SATURATED, Status::kCurvatureSaturated, "CURVATURE_SATURATED"},
    {FieldState::STATUS_CONTOUR_WATCHDOG, Status::kContourWatchdog, "CONTOUR_WATCHDOG"},
    {FieldState::STATUS_HEADING_INFEASIBLE, Status::kHeadingInfeasible, "HEADING_INFEASIBLE"},
    {FieldState::STATUS_COLLISION_AHEAD, Status::kCollisionAhead, "COLLISION_AHEAD"},
    {FieldState::STATUS_BLOCKED, Status::kBlocked, "BLOCKED"},
  };
  std::uint32_t all = 0;
  for (const StatusCase & c : cases) {
    EXPECT_EQ(c.message_value, c.status) << c.name;
    EXPECT_EQ(std::string(vectorfield_core::statusFlagName(c.status)), c.name);
    EXPECT_EQ(all & c.message_value, 0U) << c.name << " overlaps another bit";
    all |= c.message_value;
  }
  EXPECT_EQ(all, Status::kAll);
}

TEST(FieldStateConstants, FollowCurveFeedbackPhaseUsesTheSameType)
{
  // FollowCurve feedback carries `uint8 phase  # FieldState.PHASE_*`.
  vectorfield_msgs::action::FollowCurve::Feedback feedback;
  feedback.phase = FieldState::PHASE_FINAL_APPROACH;
  EXPECT_EQ(feedback.phase, vectorfield_core::toUnderlying(Phase::kFinalApproach));
}

}  // namespace
