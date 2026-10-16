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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_nav2_controller/plan_processing.hpp"

namespace vectorfield_nav2_controller
{

namespace vf = vectorfield_core;

bool sameGoal(const vf::Pose2 & a, const vf::Pose2 & b) noexcept
{
  return (a.p - b.p).norm() <= kGoalIdentityDistance &&
         std::abs(vf::wrapAngle(a.yaw - b.yaw)) <= kGoalIdentityYaw;
}

PlanIntake intakePlan(
  vf::Span<const vf::Pose2> plan, double chord_length, PlanIntakeState * state,
  std::vector<vf::Vec2> * positions, std::vector<vf::Vec2> * decimated) noexcept
{
  PlanIntake intake;
  if (plan.empty()) {
    intake.empty = true;
    return intake;
  }

  // 2. Goal identity on the last raw pose (plan frame).
  const vf::Pose2 & goal = plan[plan.size() - 1];
  if (!state->has_goal || !sameGoal(goal, state->goal)) {
    ++state->goal_seq;
    state->has_end_tangent = false;
    intake.new_goal = true;
  }
  state->has_goal = true;
  state->goal = goal;

  positions->clear();
  for (const vf::Pose2 & pose : plan) {
    positions->push_back(pose.p);
  }

  // 3. End tangent: a chord ℓ_c long gives a new one; otherwise a stored tangent of the same goal
  // is kept (near-goal replans start at the robot); otherwise the direction from the first pose.
  // A one-pose plan has none (the one-pose rule decides).
  if (plan.size() < 2) {
    state->has_end_tangent = false;
  } else {
    const vf::EndTangent tangent = vf::endTangent(*positions, chord_length);
    if (tangent.found_chord || (tangent.valid && !state->has_end_tangent)) {
      state->end_tangent = tangent.tangent;
      state->has_end_tangent = true;
    }
  }
  intake.has_end_tangent = state->has_end_tangent;

  // 4. Cusps on the chord-decimated plan (reversals shorter than ℓ_c vanish there).
  vf::decimateChords(*positions, chord_length, decimated);
  intake.cusp = vf::hasCusp(*decimated);

  // 5. Sparse plans.
  for (std::size_t i = 1; i < positions->size(); ++i) {
    const double length = ((*positions)[i] - (*positions)[i - 1]).norm();
    intake.max_segment = std::max(intake.max_segment, length);
  }
  intake.sparse = intake.max_segment > kSparseSegmentLength;

  // 6. Unsmoothed plans: the sharpest turn between consecutive decimated chords.
  for (std::size_t i = 1; i + 1 < decimated->size(); ++i) {
    const vf::Vec2 before = (*decimated)[i] - (*decimated)[i - 1];
    const vf::Vec2 after = (*decimated)[i + 1] - (*decimated)[i];
    const double turn = std::abs(vf::wrapAngle(vf::angleOf(after) - vf::angleOf(before)));
    intake.max_chord_turn = std::max(intake.max_chord_turn, turn);
  }
  intake.unsmoothed = intake.max_chord_turn > kUnsmoothedTurnAngle;

  // 7. The window must reach the goal of this plan again before the tail is conditioned.
  state->goal_in_window = false;
  return intake;
}

WindowBuild buildWindow(
  const WindowInput & input, PlanIntakeState * state, std::vector<vf::Vec2> * work,
  std::vector<vf::Vec2> * out) noexcept
{
  WindowBuild build;
  out->clear();
  if (input.window.empty()) {
    return build;
  }
  const vf::Pose2 & back = input.window[input.window.size() - 1];

  // 1. The "window ends at the goal" latch (until the next plan).
  if (!state->goal_in_window && (back.p - input.goal.p).norm() < vf::kWindowAtGoalDistance) {
    state->goal_in_window = true;
    build.goal_entered = true;
  }
  build.ends_at_goal = state->goal_in_window;

  work->clear();
  for (const vf::Pose2 & pose : input.window) {
    work->push_back(pose.p);
  }

  if (build.ends_at_goal && input.has_end_tangent) {
    // 2. Tail conditioning, then 3. dedupe. The swaps only exchange the caller's buffers.
    vf::tailCondition(*work, input.end_tangent, input.chord_length, out);
    out->swap(*work);
    vf::dedupe(*work, false, out);
    // tailCondition leaves no vertex within 1 mm of the virtual one, and the virtual vertex is
    // ℓ_c from W_back, so dedupe keeps it at size() − 2.
    if (out->size() >= 2) {
      build.kind = WindowKind::kTailConditioned;
      build.virtual_vertex = static_cast<int>(out->size()) - 2;
      return build;
    }
  } else {
    // 3. Dedupe at 1 mm.
    vf::dedupe(*work, false, out);
    if (out->size() >= 2) {
      return build;
    }
  }

  // 4. The one-pose rule.
  const std::array<vf::Vec2, 2> segment = vf::onePoseWindow(input.base, back);
  out->clear();
  out->push_back(segment[0]);
  out->push_back(segment[1]);
  build.kind = WindowKind::kOnePose;
  build.virtual_vertex = -1;
  return build;
}

double speedLimitScale(double speed_limit, bool percentage, double desired_speed) noexcept
{
  // NO_SPEED_LIMIT is 0; negative and non-finite limits are treated the same way.
  if (!std::isfinite(speed_limit) || speed_limit <= 0.0) {
    return 1.0;
  }
  double scale = 1.0;
  if (percentage) {
    scale = speed_limit / 100.0;
  } else if (desired_speed > 0.0 && std::isfinite(desired_speed)) {
    scale = speed_limit / desired_speed;
  }
  return std::clamp(scale, kMinSpeedLimitScale, 1.0);
}

bool resetDue(double seconds_since_last_compute, double control_period) noexcept
{
  if (!(seconds_since_last_compute >= 0.0)) {
    return true;  // negative or NaN: no compute yet for this goal
  }
  return seconds_since_last_compute > kResetIdlePeriods * control_period;
}

std::string noValidControlReason(std::uint32_t status, bool cancelling)
{
  if (cancelling || (status & kNoValidControlStatus) == 0U) {
    return std::string();
  }
  struct Reason
  {
    std::uint32_t flag;
    const char * explanation;
  };
  static constexpr std::array<Reason, 4> kReasons = {{
    {vf::Status::kBlocked, "no free path ahead"},
    {vf::Status::kNonFinite, "the field or the command is not finite"},
    {vf::Status::kContourWatchdog, "the obstacle contour exceeded its length or time limit"},
    {vf::Status::kHeadingInfeasible, "the goal heading cannot be reached"},
  }};
  std::string message = "vector field: ";
  bool first = true;
  for (const Reason & reason : kReasons) {
    if ((status & reason.flag) == 0U) {
      continue;
    }
    if (!first) {
      message += ", ";
    }
    first = false;
    message += vf::statusFlagName(reason.flag);
    message += " (";
    message += reason.explanation;
    message += ')';
  }
  return message;
}

}  // namespace vectorfield_nav2_controller
