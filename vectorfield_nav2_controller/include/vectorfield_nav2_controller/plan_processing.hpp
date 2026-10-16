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

/// @file
/// @brief Pure plan-processing rules of the controller plugin: raw-plan intake (goal identity,
/// end tangent, cusps, sparse and unsmoothed plans), the per-cycle window (tail conditioning,
/// dedupe, the one-pose rule) and the small server-input rules (plan §5.1–§5.3; math_spec §2.8,
/// §9.1).
///
/// Implemented in src/plan_processing.cpp (M2 wave 2, path). Everything here works on
/// vectorfield_core types only (no ROS), composes the core's conditioning helpers
/// (vectorfield_core/conditioning.hpp: endTangent, decimateChords, hasCusp, tailCondition, dedupe,
/// onePoseWindow) and is unit-testable without a node. Functions that fill caller-owned vectors
/// clear and refill them, so they do not allocate once the capacity suffices; like the core, the
/// only exception that can escape a `noexcept` function here is `std::bad_alloc` while such a
/// buffer grows, which terminates.

#ifndef VECTORFIELD_NAV2_CONTROLLER__PLAN_PROCESSING_HPP_
#define VECTORFIELD_NAV2_CONTROLLER__PLAN_PROCESSING_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

/// @brief The Nav2 controller plugin of vectorfield_nav2.
namespace vectorfield_nav2_controller
{

/// @name Fixed thresholds (not parameters)
/// @{
/// Goal identity: two raw-plan goals are the same goal within 1 cm ... (plan §3.3).
inline constexpr double kGoalIdentityDistance = 0.01;
/// ... and 0.01 rad (plan §3.3).
inline constexpr double kGoalIdentityYaw = 0.01;
/// Sparse-plan warning when a raw segment exceeds 0.25 m (plan §5.1).
inline constexpr double kSparseSegmentLength = 0.25;
/// "Unsmoothed plan" advisory (math_spec §11 "Runtime, once per plan"; criterion fixed in M2):
/// some interior vertex of the chord-decimated plan turns by more than 45°, i.e. a corner sharper
/// than a curvature of about \f$(\pi/4)/\ell_c\f$. Reversals (cusps, > 90°) are errors instead.
inline constexpr double kUnsmoothedTurnAngle = 0.25 * vectorfield_core::kPi;
/// Humble reset emulation: a full reset in setPlan() when more than this many control periods
/// passed since the last compute (plan §5.1).
inline constexpr double kResetIdlePeriods = 3.0;
/// Lower bound of the speed-limit scale (setSpeedLimit), so the field stays defined.
inline constexpr double kMinSpeedLimitScale = 0.01;
/// @}

/// @brief True iff two raw-plan goals (plan frame) are the same goal:
/// \f$\lVert p_a - p_b\rVert \le\f$ kGoalIdentityDistance and
/// \f$\lvert\operatorname{wrap}(\psi_a - \psi_b)\rvert \le\f$ kGoalIdentityYaw.
bool sameGoal(const vectorfield_core::Pose2 & a, const vectorfield_core::Pose2 & b) noexcept;

/// @brief Cross-plan state of the intake, owned by the plugin; kept across `setPlan()` /
/// `newPathReceived()` and cleared by `reset()` (default construction).
struct PlanIntakeState
{
  /// A goal was seen.
  bool has_goal{false};
  /// Last raw pose of the last plan (plan frame).
  vectorfield_core::Pose2 goal;
  /// Goal identity passed to `TrackerInput::goal_seq`; incremented on every new goal (the first
  /// plan gives 1).
  std::uint32_t goal_seq{0};
  /// `end_tangent` is defined.
  bool has_end_tangent{false};
  /// \f$\hat T_{\mathrm{end}}\f$ (unit, plan frame), kept across replans of the same goal when a
  /// near-goal replan has no pose \f$\ell_c\f$ away from the goal (math_spec §2.8).
  vectorfield_core::Vec2 end_tangent{vectorfield_core::Vec2::UnitX()};
  /// "The window ends at the goal" latch: set by buildWindow() once
  /// \f$\lVert W_{\mathrm{back}} - \mathrm{global\_goal}\rVert\f$ < kWindowAtGoalDistance, cleared
  /// by intakePlan() (latched until the next plan; nav2_api_matrix.md §4.1).
  bool goal_in_window{false};
};

/// @brief What intakePlan() found in one raw plan.
struct PlanIntake
{
  /// The plan has no pose (the caller throws `InvalidPath`; the state is unchanged).
  bool empty{false};
  /// The goal differs from the previous plan's (goal_seq was incremented).
  bool new_goal{false};
  /// `state.end_tangent` is usable this plan (false for one-pose or degenerate plans, which use
  /// the one-pose rule).
  bool has_end_tangent{false};
  /// A reversal longer than \f$\ell_c\f$ was found (the caller throws
  /// `InvalidPath("plan contains reversing segments; use a forward-only planner")` until M7).
  bool cusp{false};
  /// Some raw segment is longer than kSparseSegmentLength (warn once per goal).
  bool sparse{false};
  /// Longest raw segment (m).
  double max_segment{0.0};
  /// Some decimated vertex turns by more than kUnsmoothedTurnAngle (warn once per goal).
  bool unsmoothed{false};
  /// Largest turning angle between consecutive decimated chords (rad).
  double max_chord_turn{0.0};
};

/// @brief Raw-plan intake (plan §5.1), run in `setPlan()` / `newPathReceived()` on the raw plan
/// in its own (plan) frame.
///
/// In order:
/// 1. An empty plan → `empty`; the state is untouched.
/// 2. Goal identity: the last pose against `state->goal` with sameGoal(); a different goal (or
///    the first) increments `goal_seq` and drops the stored end tangent.
/// 3. End tangent \f$\hat T_{\mathrm{end}}\f$ (vectorfield_core::endTangent, math_spec §2.8): a
///    pose at least \f$\ell_c\f$ before the goal gives a new tangent; otherwise a stored tangent
///    of the same goal is kept (near-goal replans start at the robot); otherwise the direction
///    from the first pose, if it is at least 1 mm from the goal; otherwise none (one-pose rule).
/// 4. Cusps: vectorfield_core::decimateChords (pop-before-append rule), then hasCusp().
/// 5. Sparse: the longest raw segment against kSparseSegmentLength.
/// 6. Unsmoothed: the largest turn between consecutive decimated chords against
///    kUnsmoothedTurnAngle.
/// 7. Clears `goal_in_window`.
/// @param plan Raw plan poses (plan frame), in order.
/// @param chord_length \f$\ell_c\f$ = vectorfield_core::chordLength(costmap resolution).
/// @param state In/out cross-plan state.
/// @param positions Work buffer (the plan's positions).
/// @param decimated Work buffer (the chord-decimated plan).
PlanIntake intakePlan(
  vectorfield_core::Span<const vectorfield_core::Pose2> plan, double chord_length,
  PlanIntakeState * state, std::vector<vectorfield_core::Vec2> * positions,
  std::vector<vectorfield_core::Vec2> * decimated) noexcept;

/// @brief Inputs of buildWindow(), all in the costmap global frame.
struct WindowInput
{
  /// The window: Gen C the server's `transformed_global_plan`, Gen A/B LegacyPathHandler's.
  /// Must not be empty (the caller throws `InvalidPath`).
  vectorfield_core::Span<const vectorfield_core::Pose2> window;
  /// The goal: Gen C `global_goal`, Gen A/B LegacyPathHandler's goal.
  vectorfield_core::Pose2 goal;
  /// base_link position (the one-pose rule).
  vectorfield_core::Vec2 base{vectorfield_core::Vec2::Zero()};
  /// The stored end tangent is defined (PlanIntakeState::has_end_tangent).
  bool has_end_tangent{false};
  /// \f$R_{\mathrm{plan}\to\mathrm{costmap}}\,\hat T_{\mathrm{end}}\f$ (unit, costmap frame). Gen C:
  /// rotated by the yaw of `tf->lookupTransform(window frame, raw frame, window stamp)`; Gen A/B:
  /// by LegacyPathHandler's transform of this cycle.
  vectorfield_core::Vec2 end_tangent{vectorfield_core::Vec2::UnitX()};
  /// \f$\ell_c\f$.
  double chord_length{vectorfield_core::kChordMinLength};
};

/// @brief How buildWindow() shaped the vertices.
enum class WindowKind : std::uint8_t
{
  /// The deduplicated window as it is.
  kPolyline = 0,
  /// Tail-conditioned: the last segment is \f$[W_{\mathrm{back}} - \ell_c\hat T,
  /// W_{\mathrm{back}}]\f$ (virtual vertex at `size() − 2`).
  kTailConditioned = 1,
  /// The one-pose rule: \f$[p, p_{\mathrm{last}}]\f$ or \f$[p_{\mathrm{last}} -
  /// 0.01\,\mathbf h(\psi_{\mathrm{last}}), p_{\mathrm{last}}]\f$.
  kOnePose = 2,
};

/// @brief Result of buildWindow().
struct WindowBuild
{
  /// Shape of the output.
  WindowKind kind{WindowKind::kPolyline};
  /// The (latched) "window ends at the goal" test holds this cycle.
  bool ends_at_goal{false};
  /// The latch was set in this call (the first cycle whose window reaches the goal: the plugin
  /// then checks the "goal entered the window with \f$\sigma_{\mathrm{end}} < \sigma_b\f$"
  /// advisory once).
  bool goal_entered{false};
  /// Index of the virtual tail vertex in the output (kTailConditioned), else −1.
  int virtual_vertex{-1};
};

/// @brief Per-cycle window conditioning (plan §5.2 step 1, §5.3; math_spec §2.8).
///
/// 1. Updates `state->goal_in_window` (latched until the next plan) with
///    \f$\lVert W_{\mathrm{back}} - \mathrm{goal}\rVert\f$ < vectorfield_core::kWindowAtGoalDistance.
/// 2. If it holds and `has_end_tangent`: vectorfield_core::tailCondition (removes the vertices
///    within \f$\ell_c\f$ of \f$W_{\mathrm{back}}\f$ and inserts the virtual vertex). The terminal
///    point is always \f$W_{\mathrm{back}}\f$, never `goal`.
/// 3. vectorfield_core::dedupe at 1 mm (endpoints kept).
/// 4. Fewer than 2 vertices: vectorfield_core::onePoseWindow(base, last window pose).
///
/// No resampling or smoothing (neither is shift-invariant on a re-pruned window). The output goes
/// unchanged into `PolylineCurve::assign(out, false)`; a non-kOk status is `InvalidPath`.
/// @param input Window, goal, base and end tangent (costmap frame).
/// @param state In/out: the goal-in-window latch.
/// @param work Work buffer.
/// @param out The vertices (cleared first).
WindowBuild buildWindow(
  const WindowInput & input, PlanIntakeState * state, std::vector<vectorfield_core::Vec2> * work,
  std::vector<vectorfield_core::Vec2> * out) noexcept;

/// @brief `TrackerInput::speed_limit_scale` from `setSpeedLimit()` (plan §5.3).
///
/// `nav2_costmap_2d::NO_SPEED_LIMIT` (0), a negative or a non-finite limit → 1. Percentage:
/// limit/100. Absolute (m/s): limit / desired_speed. The result is clamped to
/// [kMinSpeedLimitScale, 1]. It scales \f$v_r\f$ and, inside the core, \f$\omega_{\max}\f$.
double speedLimitScale(double speed_limit, bool percentage, double desired_speed) noexcept;

/// @brief Humble reset emulation (plan §5.1): a full reset is due in `setPlan()` iff there was no
/// compute yet for this goal or more than kResetIdlePeriods control periods passed since the last
/// one. Replans and goal moves within a run never reset (goal_seq handles them).
/// @param seconds_since_last_compute Time since the last compute (s); negative or NaN: none yet.
/// @param control_period \f$1/f\f$ (s).
bool resetDue(double seconds_since_last_compute, double control_period) noexcept;

/// @brief Status bits that the plugin maps to `NoValidControl` (plan §5.2 step 6), after
/// committing the state: kBlocked, kNonFinite, kContourWatchdog and kHeadingInfeasible.
inline constexpr std::uint32_t kNoValidControlStatus =
  vectorfield_core::Status::kBlocked | vectorfield_core::Status::kNonFinite |
  vectorfield_core::Status::kContourWatchdog | vectorfield_core::Status::kHeadingInfeasible;

/// @brief The `NoValidControl` message for a cycle's status, or an empty string when the cycle
/// must not throw: none of kNoValidControlStatus is set, or a cancel is in progress (while
/// cancelling no status throws: with `failure_tolerance` 0 a throw would turn the cancel into an
/// abort). The message names every set bit, e.g. "vector field: BLOCKED (no free path ahead)".
std::string noValidControlReason(std::uint32_t status, bool cancelling);

}  // namespace vectorfield_nav2_controller

#endif  // VECTORFIELD_NAV2_CONTROLLER__PLAN_PROCESSING_HPP_
