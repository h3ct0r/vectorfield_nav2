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
/// @brief GoalSupervisor: the per-cycle verdict of a FollowCurve goal from its inputs' health and
/// the Tracker's status. Pure: times are passed in, no clock, no I/O.
///
/// **Precedence**, highest first:
/// 1. deactivation (abort UNKNOWN, zero now), then cancel (kCancel);
/// 2. a Nav2 action became active (abort NAV2_ACTIVE, one zero now); an invalid configuration
///    or kNonFinite (abort UNKNOWN, zero now); `max_duration` (abort TIMEOUT, decelerate);
/// 3. an untransformable curve frame (zero; TF_ERROR after `stale_abort_timeout`) and stale
///    inputs (decelerate; STALE_INPUT after `stale_abort_timeout`), checked before the Tracker
///    runs;
/// 4. kHeadingInfeasible (abort UNSUPPORTED, decelerate) and kContourWatchdog (abort BLOCKED,
///    decelerate);
/// 5. **kBlocked and the progress watchdog**: stop and wait; BLOCKED after `blocked_timeout` or
///    `progress_timeout`;
/// 6. **the Assumption-3 timer**: kAssumption3Clamped held for longer than
///    `assumption3_abort_time` aborts ASSUMPTION3_VIOLATED (decelerate). The timer is **paused**
///    while kBlocked is set (a block neither advances nor resets it).
///
/// The progress watchdog: the robot must move \f$r_p = \min(\texttt{progress\_radius},
/// \tfrac12 v_{\mathrm{nom}}\,\texttt{progress\_timeout})\f$ from an anchor within
/// `progress_timeout`; every such move re-anchors. It counts only cycles where the Tracker ran
/// (holds re-anchor it).

#ifndef VECTORFIELD_CURVE_FOLLOWER__GOAL_SUPERVISOR_HPP_
#define VECTORFIELD_CURVE_FOLLOWER__GOAL_SUPERVISOR_HPP_

#include <cstdint>

#include "vectorfield_core/types.hpp"

namespace vectorfield_curve_follower
{

/// @brief FollowCurve result codes (`vectorfield_msgs/action/FollowCurve` Result constants;
/// a test asserts they are equal).
namespace result_code
{
/// Succeeded (and canceled).
inline constexpr std::uint16_t kNone = 0;
/// Client side, never in a result: the follower rejected the goal (it is not active).
inline constexpr std::uint16_t kGoalRejected = 1;
/// Client side, never in a result: the client could not send the goal.
inline constexpr std::uint16_t kSendGoalFailure = 2;
/// Deactivation, kNonFinite, a rejected rebuild.
inline constexpr std::uint16_t kUnknown = 1000;
/// The curve was rejected.
inline constexpr std::uint16_t kInvalidCurve = 1001;
/// Open and time-varying, Odometry child ≠ curve frame, TF mode without a reference, the
/// interlock cannot see follow_path, kHeadingInfeasible.
inline constexpr std::uint16_t kUnsupported = 1002;
/// The curve frame stayed untransformable.
inline constexpr std::uint16_t kTfError = 1003;
/// An input stayed stale.
inline constexpr std::uint16_t kStaleInput = 1004;
/// kBlocked, no progress, kContourWatchdog.
inline constexpr std::uint16_t kBlocked = 1005;
/// `max_duration` reached.
inline constexpr std::uint16_t kTimeout = 1006;
/// The Assumption-3 clamp held too long.
inline constexpr std::uint16_t kAssumption3Violated = 1007;
/// A Nav2 action is (or became) active.
inline constexpr std::uint16_t kNav2Active = 1008;
/// The goal-scoped configuration was rejected.
inline constexpr std::uint16_t kInvalidGoal = 1009;
}  // namespace result_code

/// @brief Timeouts of the supervisor (the follower's parameters).
struct SupervisorConfig
{
  /// `stale_abort_timeout` (s).
  double stale_abort_timeout{2.0};
  /// `blocked_timeout` (s).
  double blocked_timeout{10.0};
  /// `progress_radius` (m).
  double progress_radius{0.5};
  /// `progress_timeout` (s).
  double progress_timeout{10.0};
  /// `assumption3_abort_time` (s).
  double assumption3_abort_time{1.0};
  /// The goal's `max_duration` (s); <= 0: unlimited.
  double max_duration{0.0};
};

/// @brief One cycle's inputs.
struct SupervisorInput
{
  /// Cycle time (s, the node's clock).
  double now{0.0};
  /// The client requested a cancel.
  bool cancel_requested{false};
  /// The node is deactivating.
  bool deactivating{false};
  /// An exclusive Nav2 action is active.
  bool nav2_active{false};
  /// The last parameter or footprint rebuild failed.
  bool config_invalid{false};
  /// The curve frame could be transformed into the control frame.
  bool curve_frame_ok{true};
  /// Robot pose, odometry, costmap or footprint is stale or missing.
  bool inputs_stale{false};
  /// The moving frame's input is stale (FRAME_MOVING only).
  bool frame_stale{false};
  /// The Tracker ran this cycle: `status` and `position` are valid.
  bool tracker_ran{false};
  /// Tracker status bits of this cycle.
  std::uint32_t status{0};
  /// Robot position (control frame), for the progress watchdog.
  vectorfield_core::Vec2 position{vectorfield_core::Vec2::Zero()};
  /// Robot height (control frame) for 3D callers (vectorfield_follower_3d): the watchdog then
  /// measures the 3D displacement. Planar callers leave it at 0.
  double position_z{0.0};
  /// Nominal speed \f$v_{\mathrm{nom}}\f$ (desired_speed × speed limit, before the cap and the
  /// clearance scaling) (m/s).
  double nominal_speed{0.0};
};

/// @brief What the node does this cycle.
enum class SupervisorAction : std::uint8_t
{
  /// Publish the Tracker's command.
  kContinue = 0,
  /// Do not publish the Tracker's command: ramp the last command down (stale inputs).
  kDecelerate = 1,
  /// Publish zero (curve frame not transformable).
  kZero = 2,
  /// A cancel is in progress: step with `TrackerInput::cancel`.
  kCancel = 3,
  /// Terminate the goal with `code` (`decelerate`: ramp down first, then one zero; otherwise one
  /// zero now).
  kAbort = 4,
};

/// @brief The verdict of one cycle.
struct SupervisorDecision
{
  /// Action.
  SupervisorAction action{SupervisorAction::kContinue};
  /// Result code (kAbort).
  std::uint16_t code{result_code::kNone};
  /// kAbort: decelerate before the final zero.
  bool decelerate{false};
  /// Static reason text (kAbort, and the holds), never null.
  const char * reason{""};
};

/// @brief Per-goal supervisor (see the file comment). Create one per goal, or reset() it.
class GoalSupervisor
{
public:
  /// @brief A supervisor for a goal starting at `start_time`.
  explicit GoalSupervisor(
    const SupervisorConfig & config = SupervisorConfig{}, double start_time = 0.0) noexcept;

  /// @brief Restarts every timer for a goal starting at `start_time`.
  void reset(const SupervisorConfig & config, double start_time) noexcept;
  /// @brief New timeouts (a parameter update during the goal); the timers keep running.
  void setConfig(const SupervisorConfig & config) noexcept { config_ = config; }
  /// @brief The settings.
  const SupervisorConfig & config() const noexcept { return config_; }

  /// @brief The verdict of one cycle; updates the timers.
  SupervisorDecision update(const SupervisorInput & input) noexcept;

  /// @brief Time the Assumption-3 clamp has been held, excluding blocked cycles (s).
  double assumption3Time() const noexcept { return a3_time_; }
  /// @brief Progress radius \f$r_p\f$ for a nominal speed.
  double progressRadius(double nominal_speed) const noexcept;

private:
  SupervisorConfig config_;
  double start_time_{0.0};
  double last_time_{0.0};
  bool has_last_time_{false};
  double stale_since_{-1.0};
  double tf_error_since_{-1.0};
  double blocked_since_{-1.0};
  double a3_time_{0.0};
  bool has_anchor_{false};
  vectorfield_core::Vec2 anchor_{vectorfield_core::Vec2::Zero()};
  double anchor_z_{0.0};
  double anchor_time_{0.0};
};

/// @brief Linear and angular stop thresholds of a cancel (0.01 m/s and 0.01 rad/s).
inline constexpr double kStopLinearThreshold = 0.01;
/// @copydoc kStopLinearThreshold
inline constexpr double kStopAngularThreshold = 0.01;

/// @brief True iff \f$\lVert(v_x, v_y)\rVert < 0.01\f$ m/s and \f$\lvert\omega\rvert < 0.01\f$
/// rad/s.
bool belowStopThresholds(const vectorfield_core::Twist2 & twist) noexcept;

/// @brief One step of the follower's deceleration ramp: each component moves toward zero by at
/// most its limit × dt (the linear part keeps its direction).
/// @param twist Last published command.
/// @param dt Step (s).
/// @param linear_decel Linear deceleration limit (m/s²), e.g. `max_linear_decel`.
/// @param angular_decel Angular deceleration limit (rad/s²), e.g. `max_angular_accel`.
/// @return The next command; exactly zero once below the stop thresholds.
vectorfield_core::Twist2 rampDown(
  const vectorfield_core::Twist2 & twist, double dt, double linear_decel,
  double angular_decel) noexcept;

}  // namespace vectorfield_curve_follower

#endif  // VECTORFIELD_CURVE_FOLLOWER__GOAL_SUPERVISOR_HPP_
