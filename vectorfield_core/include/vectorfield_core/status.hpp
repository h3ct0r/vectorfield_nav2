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
/// @brief Tracker phases and status bits.
///
/// The numeric values are a stable surface: they equal the `PHASE_*` and `STATUS_*` constants of
/// `vectorfield_msgs/msg/FieldState.msg`, and vectorfield_ros `static_assert`s the equality
/// (plan §4.1). Never renumber them.

#ifndef VECTORFIELD_CORE__STATUS_HPP_
#define VECTORFIELD_CORE__STATUS_HPP_

#include <cstdint>

namespace vectorfield_core
{

/// @brief Tracker phase (math_spec §9.2, sec-phases-table). Values equal FieldState.msg
/// `PHASE_*`.
enum class Phase : std::uint8_t
{
  /// Rotate in place toward the tracked field (diff/skid only).
  kRotateToField = 0,
  /// Follow the field (the default phase).
  kFollow = 1,
  /// Terminal approach of an open curve; FL hands over to heading tracking at base_link.
  kFinalApproach = 2,
  /// Rotate in place to the goal yaw after settling (plugin only; not Ackermann).
  kRotateToHeading = 3,
  /// Decelerate to zero and stay there (cancel, heading infeasible, contour watchdog).
  kStopped = 4,
};

/// @brief Status bits of one control cycle. Values equal FieldState.msg `STATUS_*`.
///
/// Bits are OR-ed into `TrackerOutput::status`. They never throw; the ROS layer maps them to
/// Nav2 exceptions or action results after committing the state (plan §3.1, §5.2).
struct Status final
{
  Status() = delete;

  /// No status.
  static constexpr std::uint32_t kNone = 0U;
  /// A non-finite value reached the command; the command was zeroed (math_spec §8.4), or a
  /// parametric curve's speed fell below its regularity floor (math_spec §2.5).
  static constexpr std::uint32_t kNonFinite = 1U;
  /// The feedforward was clamped, or the curve cannot be tracked at the externally limited
  /// speed (math_spec §4.2, §8.2, §8.3).
  static constexpr std::uint32_t kAssumption3Clamped = 2U;
  /// A global closest-point candidate was adopted this cycle (math_spec §2.4).
  static constexpr std::uint32_t kAmbiguousClosest = 4U;
  /// The closest point lies in a hairpin wedge (exact reversal; math_spec §2.3).
  static constexpr std::uint32_t kHairpin = 8U;
  /// The Ackermann curvature clamp, or the \f$\omega_{\max}\f$ clamp of the skid slip
  /// compensation, acted (math_spec §8.4).
  static constexpr std::uint32_t kCurvatureSaturated = 16U;
  /// A contour episode exceeded its length or time limit; enters STOPPED (math_spec §6.4).
  static constexpr std::uint32_t kContourWatchdog = 32U;
  /// The Ackermann goal or heading is infeasible; enters STOPPED (math_spec §7.5).
  static constexpr std::uint32_t kHeadingInfeasible = 64U;
  /// The collision cap or the clearance slowdown binds the speed, or a cancel found a collision
  /// (math_spec §10.3).
  static constexpr std::uint32_t kCollisionAhead = 128U;
  /// The current pose collides, or the free distance ahead (or the swept rotation) is blocked;
  /// a per-cycle status, not a phase (math_spec §10.4).
  static constexpr std::uint32_t kBlocked = 256U;

  /// Every defined bit.
  static constexpr std::uint32_t kAll = 511U;
  /// Bits that make the Tracker enter STOPPED (math_spec §9.2).
  static constexpr std::uint32_t kStoppedEntry = kHeadingInfeasible | kContourWatchdog;

  /// @brief True iff every bit of `flag` is set in `bits` (false for an empty flag).
  static constexpr bool has(std::uint32_t bits, std::uint32_t flag) noexcept
  {
    return flag != 0U && (bits & flag) == flag;
  }
};

/// @brief Underlying value of a phase (the FieldState.msg constant).
constexpr std::uint8_t toUnderlying(Phase phase) noexcept
{
  return static_cast<std::uint8_t>(phase);
}

/// @brief True for the in-place rotation phases (ROTATE_TO_FIELD, ROTATE_TO_HEADING).
constexpr bool isRotationPhase(Phase phase) noexcept
{
  return phase == Phase::kRotateToField || phase == Phase::kRotateToHeading;
}

/// @brief True for the translation phases (FOLLOW, FINAL_APPROACH), where kBlocked uses the free
/// arc length (math_spec §10.4).
constexpr bool isTranslationPhase(Phase phase) noexcept
{
  return phase == Phase::kFollow || phase == Phase::kFinalApproach;
}

/// @brief Upper-case name of a phase as in FieldState.msg without the `PHASE_` prefix, e.g.
/// "FOLLOW"; "UNKNOWN" for an invalid value.
constexpr const char * phaseName(Phase phase) noexcept
{
  switch (phase) {
    case Phase::kRotateToField:
      return "ROTATE_TO_FIELD";
    case Phase::kFollow:
      return "FOLLOW";
    case Phase::kFinalApproach:
      return "FINAL_APPROACH";
    case Phase::kRotateToHeading:
      return "ROTATE_TO_HEADING";
    case Phase::kStopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}

/// @brief Upper-case name of a single status bit as in FieldState.msg without the `STATUS_`
/// prefix, e.g. "BLOCKED"; "NONE" for 0 and "UNKNOWN" for anything that is not exactly one
/// defined bit.
constexpr const char * statusFlagName(std::uint32_t flag) noexcept
{
  switch (flag) {
    case Status::kNone:
      return "NONE";
    case Status::kNonFinite:
      return "NON_FINITE";
    case Status::kAssumption3Clamped:
      return "ASSUMPTION3_CLAMPED";
    case Status::kAmbiguousClosest:
      return "AMBIGUOUS_CLOSEST";
    case Status::kHairpin:
      return "HAIRPIN";
    case Status::kCurvatureSaturated:
      return "CURVATURE_SATURATED";
    case Status::kContourWatchdog:
      return "CONTOUR_WATCHDOG";
    case Status::kHeadingInfeasible:
      return "HEADING_INFEASIBLE";
    case Status::kCollisionAhead:
      return "COLLISION_AHEAD";
    case Status::kBlocked:
      return "BLOCKED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__STATUS_HPP_
