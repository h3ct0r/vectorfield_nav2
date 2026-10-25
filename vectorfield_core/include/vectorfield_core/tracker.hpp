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
/// @brief The Tracker: one control cycle of the vector-field controller, shared by the Nav2
/// plugin and the curve follower (math_spec §8–§10).
///
/// Implemented in src/tracker.cpp.

#ifndef VECTORFIELD_CORE__TRACKER_HPP_
#define VECTORFIELD_CORE__TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <vector>

#include "vectorfield_core/collision_predictor.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/limiter.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracking_law.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Cross-cycle state of the obstacle contour (math_spec §6.4).
struct ContourState
{
  /// The `path_blocked` gate is open.
  bool gate_open{false};
  /// Time the path has stayed clear while the gate is open (s).
  double gate_clear_s{0.0};
  /// A contour episode is active.
  bool active{false};
  /// Latched circulation side ±1 (0 without an episode).
  std::int8_t sigma_c{0};
  /// Episode base-link path length (m).
  double travelled_m{0.0};
  /// Episode time (s).
  double elapsed_s{0.0};
  /// Last \f$x_o^\star\f$ of the episode **in the grid frame** (never a cell index); mapped
  /// through the current grid origin each cycle to re-seed K.
  Vec2 k_seed{Vec2::Zero()};
  /// `k_seed` is set.
  bool k_valid{false};
  /// Ackermann, `path_blocked`: the nearest blocking site of the episode's latest cycle with
  /// \f$B(s^\star)\f$ non-empty, **in the grid frame**; once \f$B(s^\star)\f$ is empty the direction
  /// exit is measured to it (math_spec §6.6 [Design]).
  Vec2 b_site{Vec2::Zero()};
  /// `b_site` is set.
  bool b_valid{false};
};

/// @brief The only cross-cycle state of the controller.
///
/// Fixed-size Eigen types and PODs only; trivially destructible (asserted below). Never add a
/// vector, string, function, smart pointer or dynamic Eigen type. Copy it member-wise; never
/// `memcpy` it (Eigen types are not trivially copyable). No field is expressed in window arc
/// length or window vertex indices. Vectors are in the evaluation frame unless stated.
struct TrackerState
{
  /// Current phase.
  Phase phase{Phase::kFollow};
  /// Closest-point search state (geometry.hpp CurveHint; x_star in the evaluation frame).
  CurveHint hint;
  /// Last tangent \f$T\f$ (curve parameter direction, evaluation frame) for the hairpin rule;
  /// zero = none.
  Vec2 last_tangent{Vec2::Zero()};
  /// Last desired heading \f$\psi_d\f$ (held while \f$\lVert F\rVert < 0.05 v_r\f$); NaN = none.
  double last_psi_d{kNaN};
  /// Obstacle-contour state.
  ContourState contour;
  /// kBlocked hysteresis latch (math_spec §10.4).
  bool blocked{false};
  /// "Goal in curve end" hysteresis (math_spec §5.5).
  bool terminal_mode{false};
  /// Follower-only XY latch (the plugin passes `GoalInfo::xy_reached`).
  bool xy_latched{false};
  /// Goal-hold latch: settled with no rotate phase (math_spec §9.2; Appendix C item 13).
  bool holding{false};
  /// Goal identity of the last step; a change resets goal-scoped state (math_spec §9.1).
  std::uint32_t goal_seq{0};
  /// CommandLimiter state (\f$u_{\mathrm{prev}}\f$, cancel_hard, re-seed flag).
  LimiterState limiter;
  /// Target heading of an active rotation, kept while \f$\lVert F_{\mathrm{trk}}\rVert < 0.05
  /// v_r\f$ (math_spec §9.2 [Spec]); NaN = none.
  double rotate_target{kNaN};
  /// STOPPED-entry status bits (kHeadingInfeasible, kContourWatchdog) that caused the latched
  /// STOPPED; re-reported in TrackerOutput::status every cycle while STOPPED, so the ROS layer
  /// keeps mapping them.
  std::uint32_t stopped_status{Status::kNone};
  /// Ackermann: the \f$v_{\mathrm{ack}}\f$ band is lifted for this goal (latched once
  /// \f$\sigma_{\mathrm{base}} < \mathrm{xy\_tol} + v_{\mathrm{ack}}^2/(2a_{\mathrm{dec}})\f$;
  /// goal-scoped).
  bool ackermann_band_lifted{false};
  /// Drive sign \f$q\f$ of the last step (+1 body \f$+x\f$, −1 body \f$-x\f$; 0 before the
  /// first). A change resets the direction-scoped state: hint, last tangent, last_psi_d,
  /// rotate_target, the contour episode and the kBlocked latch (math_spec §9.4 [Design]).
  std::int8_t drive_sign{0};
};

static_assert(std::is_trivially_destructible_v<ContourState>, "TrackerState is POD");
static_assert(std::is_trivially_destructible_v<CurveHint>, "TrackerState is POD");
static_assert(std::is_trivially_destructible_v<LimiterState>, "TrackerState is POD");
static_assert(std::is_trivially_destructible_v<Twist2>, "TrackerState is POD");
static_assert(std::is_trivially_destructible_v<TrackerState>, "TrackerState is POD");

/// @brief The goal of an open curve for one cycle.
struct GoalInfo
{
  /// Goal pose (evaluation frame). Plugin: `global_goal` in the costmap frame; follower:
  /// \f$r(s_{\mathrm{goal}})\f$ with no yaw.
  Pose2 pose;
  /// The goal has a yaw to reach; the yaw is checked iff has_yaw and yaw_tol > 1e-6.
  bool has_yaw{false};
  /// XY tolerance (m): the goal checker's (plugin) or `goal_tolerance` (follower);
  /// \f$\sigma_{\mathrm{stop}} = \mathrm{xy\_tol}/2\f$.
  double xy_tol{0.25};
  /// Yaw tolerance (rad); <= 1e-6 means unchecked.
  double yaw_tol{0.25};
  /// XY reached as Nav2 sees it: Gen C the server's goal checker, Gen A/B the emulated latch
  /// (math_spec §9.1). nullopt: the core latch `xy_latched` decides (follower).
  std::optional<bool> xy_reached;
};

/// @brief The curve frame of one cycle (math_spec §4.2).
struct CurveFrame
{
  /// Curve frame → evaluation frame (\f$T_{\mathrm{eval}\leftarrow\mathcal F}\f$). Plugin:
  /// identity.
  Isometry2 T_eval_curve;
  /// Frame twist \f$(v_{\mathcal F}, \omega_{\mathcal F})\f$ expressed in \f$\mathcal F\f$
  /// (wz = \f$\omega_{\mathcal F}\f$).
  Twist2 twist_in_curve;
  /// The frame moves (FRAME_MOVING): \f$v_c\f$ includes the frame motion and rollouts
  /// extrapolate the frame with the twist.
  bool moving{false};
};

/// @brief Inputs of one control cycle.
struct TrackerInput
{
  /// base_link pose (evaluation frame).
  Pose2 pose;
  /// Measured body twist (re-seeds the limiter).
  Twist2 measured;
  /// Time \f$t\f$ (s), the curve's time base.
  double t{0.0};
  /// \f$\Delta t\f$ (s): measured time since the previous step.
  double dt{0.05};
  /// The curve, non-owning, valid for this call.
  const Curve * curve{nullptr};
  /// The curve frame.
  CurveFrame frame;
  /// Goal identity (ROS layer increments it on a new goal).
  std::uint32_t goal_seq{0};
  /// Goal of an open curve (plugin: `global_goal`; follower: \f$r(s_{\mathrm{goal}})\f$); nullopt
  /// for closed curves.
  std::optional<GoalInfo> goal;
  /// Obstacles; nullopt disables contour and collision prediction for the cycle.
  std::optional<ObstacleMap> obstacles;
  /// External speed limit \f$\gamma_{\mathrm{lim}} \in (0, 1]\f$ (plugin: setSpeedLimit;
  /// follower: speed_limit topic); also scales \f$\omega_{\max}\f$.
  double speed_limit_scale{1.0};
  /// A cancel is in progress (plugin: cancel() called; follower: cancel requested).
  bool cancel{false};
  /// Apply the window-end cap \f$\gamma_{\mathrm{ap}}\f$ (plugin: true).
  bool cap_at_curve_end{false};
  /// Drive sign \f$q\f$ of this cycle: 0 = the configuration's (−1 iff `drive_backward`;
  /// Ackermann +1), +1 = body \f$+x\f$, −1 = body \f$-x\f$ (plugin: the active plan segment's,
  /// math_spec §9.4 [Design]). Ignored by the holonomic law. A sign that differs from the
  /// configuration's turns the obstacle contour off for the cycle.
  std::int8_t drive_sign{0};
};

/// @brief Outputs of one control cycle.
struct TrackerOutput
{
  /// Final command: body twist at base_link (FieldState.cmd), after the skid slip compensation
  /// (Kinematics::slipCompensated(); the identity unless `skid_yaw_rate_efficiency` < 1).
  Twist2 cmd;
  /// Target after the kinematic box, before the CommandLimiter (FieldState.cmd_raw).
  Twist2 cmd_target;
  /// Field quantities at the law's evaluation point.
  FieldSample field;
  /// Phase after this cycle's transitions.
  Phase phase{Phase::kFollow};
  /// Status bits (status.hpp).
  std::uint32_t status{Status::kNone};
  /// Settled (math_spec §9.1): the only entry to ROTATE_TO_HEADING and the follower's open-curve
  /// success test.
  bool settled{false};
  /// Free arc length \f$s_{\mathrm{free}}\f$ (m); \f$+\infty\f$ if collision-free or not run.
  double s_free{kInf};
  /// Braking cap \f$v_{\mathrm{cap}}\f$ (m/s); \f$+\infty\f$ if none.
  double cap{kInf};
  /// Speed-shaping terms of the final step.
  SpeedShaping shaping;
  /// Uniform kinematic-box scale \f$c\f$ (1 when not binding or under the \f$\eta\f$-box).
  double box_scale{1.0};
  /// Acceleration-stage \f$\alpha\f$ (FieldState.limiter_scale).
  double limiter_alpha{1.0};
  /// Checked rollout poses: a view into StepScratch::rollout, valid until the scratch is reused.
  Span<const Pose2> predicted;
  /// Index of the first colliding pose in `predicted`, or −1.
  int first_collision{-1};
  /// During a cancel: \f$\lvert v_x\rvert, \lvert v_y\rvert\f$ < kStopLinearSpeed and
  /// \f$\lvert\omega\rvert\f$ < kStopAngularSpeed (the plugin's cancel() returns true).
  bool cancel_complete{false};
};

/// @brief One element of the blocking set B (math_spec §6.4, `path_blocked` activation): a blocked
/// gate sample whose feature site lies in K.
///
/// The contour's start test and its direction exit are measured to the part of K that blocks the
/// path, so a long wall merged into K (gaps narrower than \f$2r_{\mathrm{insc}}\f$) can neither
/// start an episode where the path runs beside it nor end one before the obstacle is passed.
struct BlockingSite
{
  /// Path station \f$s_j\f$ of the gate sample (curve arc length, not wrapped).
  double s{0.0};
  /// Feature site \f$b_j\f$ of the sample: the centre of the nearest site cell (grid frame).
  Vec2 site_grid{Vec2::Zero()};
};

/// @brief Caller-owned per-thread work buffers of Tracker::step.
///
/// Grows on demand and never shrinks: after warm-up a steady-state step performs no allocation
/// (oracle O55). The visualizer reads it after step() (K cells, gate samples, rollout poses).
/// One instance per thread; never shared between concurrent steps.
struct StepScratch
{
  /// Checked poses of the final rollout (TrackerOutput::predicted views into it).
  std::vector<Pose2> rollout;
  /// Checked poses of side-probe rollouts.
  std::vector<Pose2> side_rollout;
  /// K: 1 = cell in the blocking component; `k_width` × `k_height`, row-major.
  std::vector<std::uint8_t> k_mask;
  /// Flood-fill queue (cell indices).
  std::vector<int> flood_queue;
  /// \f$\mathrm{EDT}_K\f$ over the sites in K (holds its own FH work arrays).
  EuclideanDistanceTransform edt_k;
  /// Gate samples \f$r(s)\f$ of this cycle (evaluation frame).
  std::vector<Vec2> gate_samples;
  /// Gate samples in the grid frame.
  std::vector<Vec2> gate_samples_grid;
  /// 1 = gate sample blocked.
  std::vector<std::uint8_t> gate_blocked;
  /// Blocking set B of this cycle, in traversal order (`path_blocked` only; empty otherwise),
  /// frozen with K for the whole step.
  std::vector<BlockingSite> blocking;
  /// Grid width of `k_mask`.
  int k_width{0};
  /// Grid height of `k_mask`.
  int k_height{0};
  /// `k_mask` and `edt_k` hold this cycle's K.
  bool k_valid{false};

  /// @brief Reserves capacity so the first steps do not allocate either.
  /// @param grid_cells Cells of the largest expected grid.
  /// @param rollout_poses Checked poses of the longest expected rollout.
  /// @param gate_samples Samples of the longest expected gate.
  void reserve(std::size_t grid_cells, std::size_t rollout_poses, std::size_t gate_samples);
};

/// @brief One control cycle of the vector-field controller.
///
/// Owns only configuration (resolved CoreConfig, kinematics, gain, footprint checker, law).
/// step() is `const noexcept`: concurrent steps on one Tracker are safe with distinct states
/// and scratches. Inside step, in order: goal_seq reset; obstacle preparation (K, EDT_K, gate,
/// frozen for the call); rollout of the private stepNominal with \f$\gamma_{\mathrm{col}} = 1\f$
/// (s_free); cap; final command (re-run stepNominal once if the cap binds); kBlocked; commit.
/// `use_collision_detection: false` skips the rollout, the cap and kBlocked except the
/// current-pose test.
class Tracker
{
public:
  /// @brief Validates and resolves a configuration.
  /// @throws ConfigError if validateConfig(config, nullopt) reports an error (the message lists
  /// them); costmap rules are the caller's (they need CostmapInfo).
  explicit Tracker(const CoreConfig & config);

  /// @brief One control cycle.
  ///
  /// **Aliasing:** `out` may alias `&in`: step works on a local copy and writes `*out` once, at
  /// the end. **Commit before acting:** callers store `*out` and the returned command before
  /// mapping any status to an exception or action result.
  /// @param input Inputs of this cycle; `input.curve` must be non-null.
  /// @param in State after the previous cycle (initialState() at start).
  /// @param out Next state.
  /// @param scratch Caller-owned work buffers (read by the visualizer afterwards).
  TrackerOutput step(
    const TrackerInput & input, const TrackerState & in, TrackerState * out,
    StepScratch & scratch) const noexcept;

  /// @brief Field probe at a point with the state's hint and the scratch's frozen K and gate,
  /// at time \f$t\f$ + time_offset (math_spec §7.4; `field_grid`, Appendix A). Call after step()
  /// with the same input, the state step() wrote and the same scratch.
  ProbeSample probe(
    const Vec2 & x, const TrackerInput & input, const TrackerState & state,
    const StepScratch & scratch, double time_offset = 0.0) const noexcept;

  /// @brief The state of a fresh start or reset(): FOLLOW, no hint, limiter re-seed requested.
  static TrackerState initialState() noexcept;

  /// @brief The resolved configuration (resolveAutoValues(); `final_approach_distance` may stay
  /// auto).
  const CoreConfig & config() const noexcept;
  /// @brief Warnings of validateConfig() at construction.
  const ValidationReport & validation() const noexcept;
  /// @brief The kinematics.
  const Kinematics & kinematics() const noexcept;
  /// @brief The gain.
  const GainFunction & gain() const noexcept;
  /// @brief The footprint.
  const Footprint & footprint() const noexcept;
  /// @brief The law.
  const ControlLaw & law() const noexcept;

private:
  CoreConfig config_;
  ValidationReport validation_;
  Kinematics kinematics_;
  GainFunction gain_;
  ControlLaw law_;
  FootprintChecker checker_;
  /// The kinematics and the law with the drive sign opposite to the configuration's (a reverse
  /// plan segment, TrackerInput::drive_sign).
  Kinematics kinematics_flipped_;
  ControlLaw law_flipped_;
  /// @brief The cycle's drive sign differs from the configuration's (never for omni).
  bool flipped(const TrackerInput & input) const noexcept;
};

/// @brief Requests that the limiter re-seed \f$u_{\mathrm{prev}}\f$ from the measured twist at
/// the next step (after a throw, on a parameter rebuild, on cancel completion; math_spec §8.4).
inline void requestLimiterReseed(TrackerState * state) noexcept { state->limiter.valid = false; }

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__TRACKER_HPP_
