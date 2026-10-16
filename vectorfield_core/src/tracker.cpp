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

#include "vectorfield_core/tracker.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include "vectorfield_core/contour.hpp"

#include "collision_rollout.hpp"

// One control cycle (plan §3.1; math_spec §8–§10):
//
//   Tracker::step
//     1. goal_seq reset of goal-scoped state, cancel bookkeeping, terminal-mode hysteresis
//     2. obstacle preparation, once: gate, goal veto, K and EDT_K, episode end on the frozen
//        conditions, episode start with the side probe (two rollouts of stepNominal)
//     3. rollout of stepNominal with gamma_col = 1 (MainRolloutPolicy) -> s_free
//     4. braking cap and kBlocked; final command: the rollout's first step, or stepNominal re-run
//        once when the cap binds, kBlocked is set or a cancel just found a collision
//     5. commit
//
//   stepNominal: one cycle of the controller on a TrackerState, also inside every rollout
//     A. base_link geometry (sigma_base, sigma_end, goal distance) and XY reached
//     B. field-independent shaping terms (gamma_lim, gamma_cl, gamma_ap, gamma_col)
//     C. phase transitions 2-5 of math_spec §9.2 (STOPPED, settled, ROTATE_TO_HEADING exit,
//        FINAL_APPROACH)
//     D. nominal path field at the evaluation point of the tracking phase
//     E. contour episode (exit tests, blend weight, watchdog)
//     F. speed shaping;  G. contour blend -> F
//     H. transition 6 (ROTATE_TO_FIELD entry and exit)
//     I. law or phase target, Ackermann band, kinematic box, reversing backstop
//     J. STOPPED-entry statuses raised this cycle (transition 2's precedence)
//     K. CommandLimiter

namespace vectorfield_core
{

namespace
{

// -----------------------------------------------------------------------------------------------
// Constants

/// The limiter re-seeds u_prev when the measured period exceeds this many nominal periods
/// (math_spec §8.4: "when Delta t > 2/f").
constexpr double kReseedPeriodFactor = 2.0;

// -----------------------------------------------------------------------------------------------
// Per-step environment

/// @brief The Tracker's configuration objects and derived constants, fixed for its lifetime.
struct Model
{
  const CoreConfig & config;
  const Kinematics & kinematics;
  const GainFunction & gain;
  const ControlLaw & law;
  const FootprintChecker & checker;
  /// The path field (a cheap value type).
  DistanceField field;
  /// \f$v_r\f$.
  double v_r;
  /// Traversal direction \f$\pm 1\f$.
  int direction;
  /// Nominal control period \f$1/f\f$: the rollout's ZOH step.
  double period;
  /// \f$\lambda\f$, \f$D_{\mathrm{in}}\f$ and \f$D'_{\mathrm{in}}\f$ (resolved).
  double lambda;
  double d_in;
  double d_in_prime;
  /// Footprint radii \f$w_{\mathrm{lat}}\f$ and \f$r_{\mathrm{insc}}\f$.
  double w_lat;
  double r_insc;
  /// \f$v_{\mathrm{stop}}\f$ of the kBlocked set test (Ackermann: \f$v_{\mathrm{ack}}\f$).
  double v_stop;
};

/// @brief Builds the Model of a Tracker from its members.
Model makeModel(
  const CoreConfig & config, const Kinematics & kinematics, const GainFunction & gain,
  const ControlLaw & law, const FootprintChecker & checker) noexcept
{
  const double v_r = config.field.desired_speed;
  const int direction = config.field.direction < 0 ? -1 : 1;
  const FootprintRadii & radii = checker.footprint().radii();
  return Model{
    config,
    kinematics,
    gain,
    law,
    checker,
    DistanceField(gain, v_r, direction, config.field.assumption3_margin),
    v_r,
    direction,
    1.0 / config.node.controller_frequency,
    config.obstacles.obstacle_contour_distance,
    config.obstacles.obstacle_full_switch_distance,
    config.obstacles.obstacle_blend_start_distance,
    radii.w_lat,
    radii.r_insc,
    kinematics.isAckermann() ? kinematics.ackermann_min_speed : kVStop};
}

/// @brief The goal of the cycle, frozen for the step (math_spec §5, §9).
struct GoalEnv
{
  /// An open curve with a goal: settled, goal hold and the rotate-to-heading phase apply.
  bool active{false};
  /// The terminal field is used (terminal mode; math_spec §5.5).
  bool terminal{false};
  /// Goal position \f$g\f$ and yaw (evaluation frame).
  Vec2 g{Vec2::Zero()};
  double yaw{0.0};
  /// The yaw is checked: has_yaw and yaw_tol > 1e-6.
  bool yaw_checked{false};
  /// ROTATE_TO_HEADING applies: yaw checked, `use_rotate_to_heading`, not Ackermann.
  bool rth_applies{false};
  double xy_tol{0.25};
  double yaw_tol{0.25};
  /// \f$\sigma_{\mathrm{stop}} = \mathrm{xy\_tol}/2\f$.
  double sigma_stop{0.125};
  /// FINAL_APPROACH entry distance of this cycle (finalApproachDistance()).
  double final_approach{kInf};
  /// XY reached as the caller gives it (plugin); nullopt: the core latch decides (follower).
  std::optional<bool> xy_given;
  /// Goal station \f$s_{\mathrm{goal}}\f$ (end for dir +1, start for dir -1); NaN when closed.
  double s_goal{kNaN};
  /// Terminal speed profile \f$\varphi\f$.
  TerminalProfile profile{1.0, 0.0, 1.0, 0.0};
};

/// @brief The stations of this cycle's gate samples, as Curve::sampleStations() lays them out:
/// regular stations \f$s_0 + k\,\Delta s\f$, then \f$s_1\f$; reversed for dir -1, so that index i
/// is in traversal order.
struct GateStations
{
  double s0{0.0};
  double ds{0.0};
  std::size_t regular{0};
  double s1{0.0};
  std::size_t count{0};
  bool forward{true};

  /// @brief Station of sample i (traversal order). Precondition: i < count.
  double station(std::size_t i) const noexcept
  {
    const std::size_t o = forward ? i : count - 1U - i;
    return o < regular ? s0 + static_cast<double>(o) * ds : s1;
  }
};

/// @brief The obstacle preparation of the step, frozen for every stepNominal of the call
/// (plan §3.1 step 0; math_spec §6.4).
struct ObstacleEnv
{
  /// The grid and \f$\mathrm{EDT}_{\mathrm{all}}\f$ are usable this cycle.
  bool valid{false};
  const OccupancyGrid * grid{nullptr};
  const EuclideanDistanceTransform * edt_all{nullptr};
  Isometry2 T_grid_eval;
  double resolution{0.05};
  /// `obstacle_mode: contour` and a usable map.
  bool contour_on{false};
  /// Frozen gate decision.
  bool gate_open{false};
  /// The curve end is the goal and it is blocked.
  bool goal_veto{false};
  /// K and \f$\mathrm{EDT}_K\f$ hold this cycle's blocking component.
  bool k_valid{false};
  const EuclideanDistanceTransform * edt_k{nullptr};
  /// First blocked gate sample (evaluation frame).
  bool has_blocked_sample{false};
  Vec2 first_blocked_sample{Vec2::Zero()};
  /// Stations of the gate samples.
  GateStations gate;
  /// `path_blocked`: the start test and the direction exit use the blocking set B.
  bool use_blocking{false};
  /// B (StepScratch::blocking), frozen with K; its station extent \f$L_B\f$.
  const std::vector<BlockingSite> * blocking{nullptr};
  double blocking_extent{0.0};
  /// The side probe of this call started the episode (its watchdog counts from the next cycle).
  bool episode_started{false};
};

/// @brief Everything stepNominal reads besides the state it advances.
struct StepEnv
{
  const Model & model;
  const TrackerInput & input;
  const GoalEnv & goal;
  ObstacleEnv obstacles;
  /// The curve is closed.
  bool closed{false};
  /// The field is time-varying: a time-varying curve or a moving frame.
  bool time_varying{false};
  /// \f$L(t)\f$ at the cycle time (closed curves: the period of the jump test).
  double length{0.0};
  /// Mean vertex spacing \f$\bar\ell\f$ of the curve (heading tracking's probe offset).
  double vertex_spacing{0.0};
  /// Station of the curve end in the traversal direction (\f$\sigma_{\mathrm{end}}\f$).
  double s_end{0.0};
  /// \f$\gamma_{\mathrm{lim}}\f$ of speed shaping (Ackermann: floored at \f$v_{\mathrm{ack}}/v_r\f$).
  double gamma_lim{1.0};
  /// \f$\bar\omega = \omega_{\max}\cdot\f$ `speed_limit_scale`.
  double omega_bar{1.5};
  /// \f$\psi_d\f$ hold threshold \f$0.05\,v_r\gamma_{\mathrm{lim}}\f$ (also the probes' NaN rule).
  double hold_speed{0.02};
  /// Measured \f$\Delta t\f$ of this call, sanitised (non-finite or negative: 0).
  double input_dt{0.0};
  /// \f$\Delta t > 2/f\f$: the first step re-seeds the limiter.
  bool reseed_by_dt{false};
};

/// @brief Builds the per-step environment (the obstacles are prepared afterwards).
StepEnv makeStepEnv(const Model & model, const TrackerInput & input, const GoalEnv & goal) noexcept
{
  StepEnv env{model, input, goal, ObstacleEnv{}};
  const Curve & curve = *input.curve;
  env.closed = curve.isClosed();
  env.time_varying = curve.isTimeVarying() || input.frame.moving;
  env.length = curve.length(input.t);
  env.vertex_spacing = curve.vertexSpacing();
  env.s_end = model.direction > 0 ? env.length : 0.0;
  const double scale =
    std::isnan(input.speed_limit_scale) ? 1.0 : clamp(input.speed_limit_scale, 0.0, 1.0);
  env.gamma_lim = scale;
  if (model.kinematics.isAckermann() && model.v_r > 0.0) {
    // math_spec §7.5: v_r gamma_lim is floored at v_ack (never applies in STOPPED: target 0).
    env.gamma_lim = std::max(scale, model.kinematics.ackermann_min_speed / model.v_r);
  }
  env.omega_bar = model.kinematics.angularLimit(scale);
  env.hold_speed = kHoldSpeedFraction * model.v_r * env.gamma_lim;
  env.input_dt = (input.dt > 0.0 && std::isfinite(input.dt)) ? input.dt : 0.0;
  env.reseed_by_dt = env.input_dt > kReseedPeriodFactor * model.period;
  return env;
}

/// @brief Parameters of one stepNominal call.
struct StepParams
{
  /// Period of this step: the limiter's and the law's \f$\Delta t\f$.
  double dt{0.05};
  /// \f$\gamma_{\mathrm{col}}\f$ (1 inside rollouts).
  double gamma_col{1.0};
  /// Re-seed source of the limiter and the speed of the watchdog's path length.
  Twist2 measured;
  /// First step of the cycle: the caller's XY-reached value and the Delta t re-seed apply.
  bool first{true};
  /// kBlocked: the phase target becomes 0.
  bool zero_target{false};
  /// The contour episode started in this cycle: its watchdog counts from the next one.
  bool skip_contour_travel{false};
  /// Fill the visualization-only quantities (the all-sites obstacle query without an episode).
  bool visualize{false};
};

/// @brief The outcome of one stepNominal call.
struct NominalResult
{
  /// Limited command.
  Twist2 cmd;
  /// Target after the kinematic box, before the CommandLimiter.
  Twist2 target;
  Phase phase{Phase::kFollow};
  bool settled{false};
  bool xy_reached{false};
  /// \f$\gamma_{\mathrm{col}}\f$ binds (strictly the smallest term).
  bool cap_binds{false};
  std::uint32_t status{Status::kNone};
  SpeedShaping shaping;
  double box_scale{1.0};
  double alpha{1.0};
  FieldSample sample;
};

// -----------------------------------------------------------------------------------------------
// Small helpers

/// @brief The command is below the stop thresholds of a cancel (math_spec §9.2).
bool belowStopThresholds(const Twist2 & cmd) noexcept
{
  return std::abs(cmd.vx) < kStopLinearSpeed && std::abs(cmd.vy) < kStopLinearSpeed &&
         std::abs(cmd.wz) < kStopAngularSpeed;
}

/// @brief Ends the contour episode and clears its latched side, counters and seed.
void endEpisode(ContourState * contour) noexcept
{
  contour->active = false;
  contour->sigma_c = 0;
  contour->travelled_m = 0.0;
  contour->elapsed_s = 0.0;
  contour->k_valid = false;
}

/// @brief The curve frame at time t: `T_eval_curve` at the input time, extrapolated with the
/// frame's body twist for a moving frame (exact for a constant twist; math_spec §10.2).
Isometry2 curveFrameAt(const TrackerInput & input, double t) noexcept
{
  const CurveFrame & frame = input.frame;
  if (!frame.moving || t == input.t || !std::isfinite(t)) {
    return frame.T_eval_curve;
  }
  const double tau = t - input.t;
  const Pose2 start = frame.T_eval_curve.toPose();
  const Twist2 & twist = frame.twist_in_curve;
  const Pose2 end = tau > 0.0
                      ? sim::integrateTwist(start, twist, tau)
                      : sim::integrateTwist(start, Twist2{-twist.vx, -twist.vy, -twist.wz}, -tau);
  return Isometry2::fromPose(end);
}

/// @brief The phase whose law and evaluation point the field serves. FOLLOW and FINAL_APPROACH
/// serve themselves; the rotation phases and STOPPED use the phase they return to
/// (FINAL_APPROACH when its entry holds, else FOLLOW), so that ROTATE_TO_FIELD's entry and exit
/// measure the same e (math_spec §9.2: "same e").
Phase trackingPhase(Phase phase, bool final_approach_entry) noexcept
{
  if (phase == Phase::kFollow || phase == Phase::kFinalApproach) {
    return phase;
  }
  return final_approach_entry ? Phase::kFinalApproach : Phase::kFollow;
}

// -----------------------------------------------------------------------------------------------
// Curve geometry

/// @brief A closest point in the evaluation frame, with the curve-frame original that
/// Curve::velocity() needs.
struct CurveQuery
{
  ClosestPoint cp;
  ClosestPoint cp_curve;
};

/// @brief Closest point of x (evaluation frame) at time t. x, the hint's foot and
/// \f$T_{\mathrm{last}}\f$ are mapped into the curve frame and the result back
/// (plan §3.1 "Ownership and frames"). Read-only: the caller decides whether to keep the hint.
CurveQuery closestPointAt(
  const StepEnv & env, const Isometry2 & frame, const CurveHint & hint, const Vec2 & last_tangent,
  const Vec2 & x, double t) noexcept
{
  CurveHint hint_curve = hint;
  hint_curve.x_star = frame.applyInverse(hint.x_star);
  EvalContext context;
  context.t = t;
  context.direction = env.model.direction;
  context.last_tangent = frame.rotateInverse(last_tangent);
  CurveQuery query;
  query.cp_curve = env.input.curve->closestPoint(frame.applyInverse(x), context, hint_curve);
  query.cp = transformed(query.cp_curve, frame);
  return query;
}

/// @brief Curve-point velocity at the foot (math_spec §4.2):
/// \f$v_c = R_{\mathcal F}(v_{\mathcal F} + \omega_{\mathcal F} J x^\star_{\mathcal F} + \partial_t
/// r_{\mathcal F})\f$, the frame terms only for a moving frame.
Vec2 curvePointVelocity(
  const StepEnv & env, const Isometry2 & frame, const ClosestPoint & cp_curve, double t) noexcept
{
  Vec2 v_curve = Vec2::Zero();
  if (env.input.curve->isTimeVarying()) {
    v_curve += env.input.curve->velocity(cp_curve, t);
  }
  if (env.input.frame.moving) {
    const Twist2 & twist = env.input.frame.twist_in_curve;
    v_curve += Vec2(twist.vx, twist.vy) + twist.wz * J(cp_curve.x_star);
  }
  return frame.rotate(v_curve);
}

/// @brief Quantities measured at base_link against the unshifted goal station, for every law
/// (math_spec §5.2).
struct BaseGeometry
{
  /// The base search ran (open curves).
  bool has_base{false};
  CurveQuery base;
  /// \f$\sigma_{\mathrm{base}}\f$: NaN closed, \f$+\infty\f$ outside terminal mode.
  double sigma{kNaN};
  /// \f$\sigma_{\mathrm{end}}\f$ to the curve (window) end; NaN closed.
  double sigma_end{kNaN};
  /// \f$\lVert p - g\rVert\f$ (\f$+\infty\f$ without a goal).
  double goal_distance{kInf};
  /// FINAL_APPROACH entry holds: terminal mode and \f$\sigma_{\mathrm{base}}\f$ below the entry
  /// distance.
  bool final_approach_entry{false};
};

/// @brief The base_link search: open curves only, with the same policy and hint as the law's
/// (math_spec §5.2: "a second closest-point search at p with the same policy").
BaseGeometry evaluateBase(
  const StepEnv & env, const TrackerState & st, const Isometry2 & frame, const Pose2 & pose,
  double t) noexcept
{
  BaseGeometry base;
  const GoalEnv & goal = env.goal;
  if (!env.closed) {
    base.has_base = true;
    base.base = closestPointAt(env, frame, st.hint, st.last_tangent, pose.p, t);
    if (base.base.cp.valid) {
      const double dir = static_cast<double>(env.model.direction);
      const double s = base.base.cp.s_star;
      base.sigma_end = std::max(0.0, dir * (env.s_end - s));
      base.sigma = goal.terminal ? dir * (goal.s_goal - s) : kInf;
    }
  }
  if (goal.active) {
    base.goal_distance = (pose.p - goal.g).norm();
  }
  base.final_approach_entry = goal.terminal && base.sigma < goal.final_approach;
  return base;
}

/// @brief Nominal path field at the evaluation point. Terminal mode: the terminal field with
/// \f$\sigma\f$ against the shifted station \f$s_{\mathrm{goal}} + \mathrm{dir}\,x_{\mathrm{ref}}\f$
/// (math_spec §5.2); otherwise the static or time-varying distance field. Resets the sample
/// first and fills the goal quantities of Appendix A.
void evaluatePathField(
  const StepEnv & env, const Isometry2 & frame, const EvaluationPoint & point,
  const CurveQuery & query, double t, FieldSample * sample) noexcept
{
  const GoalEnv & goal = env.goal;
  const Model & model = env.model;
  const double dir = static_cast<double>(model.direction);
  *sample = FieldSample{};
  if (goal.terminal) {
    const double sigma_ref = dir * (goal.s_goal - query.cp.s_star) + point.x_ref;
    model.field.evaluateTerminal(point.x, query.cp, sigma_ref, goal.profile, sample);
  } else {
    const Vec2 v_c =
      env.time_varying ? curvePointVelocity(env, frame, query.cp_curve, t) : Vec2::Zero();
    model.field.evaluate(point.x, query.cp, v_c, sample);
  }
  sample->x_ref = point.x_ref;
  sample->has_goal = goal.active;
  sample->terminal = goal.terminal;
  if (goal.active) {
    sample->g = goal.g;
    sample->s_goal = goal.s_goal;
    sample->g_ref = frame.apply(env.input.curve->pointAt(goal.s_goal + dir * point.x_ref, t));
  }
}

// -----------------------------------------------------------------------------------------------
// Obstacle queries

/// @brief An obstacle query mapped into the evaluation frame, keeping the grid-frame foot.
struct ObstacleSample
{
  bool valid{false};
  double D_o{kInf};
  /// Normal and foot in the evaluation frame.
  Vec2 n_o{Vec2::Zero()};
  Vec2 x_o{Vec2::Zero()};
  /// Foot in the grid frame (the episode's k_seed).
  Vec2 x_o_grid{Vec2::Zero()};
};

/// @brief Continuous query of a distance transform at an evaluation-frame point.
ObstacleSample queryObstacle(
  const EuclideanDistanceTransform & edt, const ObstacleEnv & obstacles,
  const Vec2 & x_eval) noexcept
{
  ObstacleSample sample;
  const ObstacleQuery query = edt.query(obstacles.T_grid_eval.apply(x_eval));
  if (!query.valid) {
    return sample;
  }
  sample.valid = true;
  sample.D_o = query.D_o;
  sample.n_o = obstacles.T_grid_eval.rotateInverse(query.n_o);
  sample.x_o = obstacles.T_grid_eval.applyInverse(query.x_o);
  sample.x_o_grid = query.x_o;
  return sample;
}

/// @brief \f$\mathrm{EDT}_K\f$ at the evaluation point with the guard of math_spec §6.4: below
/// res/2 the normal is taken from the \f$\mathrm{EDT}_K\f$ query at base_link.
ObstacleSample queryEpisodeObstacle(
  const ObstacleEnv & obstacles, const Vec2 & x_eval, const Vec2 & base) noexcept
{
  ObstacleSample sample = queryObstacle(*obstacles.edt_k, obstacles, x_eval);
  if (sample.valid && sample.D_o < 0.5 * obstacles.resolution) {
    const ObstacleSample at_base = queryObstacle(*obstacles.edt_k, obstacles, base);
    if (at_base.valid && at_base.D_o > 0.0) {
      sample.n_o = at_base.n_o;
    }
  }
  return sample;
}

/// @brief Nearest site of the active blocking set \f$B(s^\star) = \{b_j : \mathrm{dir}\,(s_j -
/// s^\star) \ge 0\}\f$ (station difference wrapped on closed curves) to x, as \f$D_B\f$, \f$n_B\f$
/// and the site (math_spec §6.4). Below res/2 the normal is taken from base_link, as for K.
/// @return False when \f$B(s^\star)\f$ is empty: the blocking part of K is behind.
bool nearestBlockingSite(
  const StepEnv & env, double s_star, const Vec2 & x_eval, const Vec2 & base,
  ObstacleSample * out) noexcept
{
  const ObstacleEnv & obstacles = env.obstacles;
  if (obstacles.blocking == nullptr || !std::isfinite(s_star)) {
    return false;
  }
  const double dir = static_cast<double>(env.model.direction);
  const Vec2 x_grid = obstacles.T_grid_eval.apply(x_eval);
  double best = kInf;
  const BlockingSite * nearest = nullptr;
  for (const BlockingSite & site : *obstacles.blocking) {
    const double ds = env.closed ? wrapPeriodic(site.s - s_star, env.length) : site.s - s_star;
    if (dir * ds < 0.0) {
      continue;
    }
    const double d = (x_grid - site.site_grid).norm();
    if (d < best) {
      best = d;
      nearest = &site;
    }
  }
  if (nearest == nullptr) {
    return false;
  }
  Vec2 normal = best > 0.0 ? Vec2((x_grid - nearest->site_grid) / best) : Vec2(Vec2::Zero());
  if (best < 0.5 * obstacles.resolution) {
    const Vec2 from_base = obstacles.T_grid_eval.apply(base) - nearest->site_grid;
    const double norm = from_base.norm();
    if (norm > 0.0) {
      normal = from_base / norm;
    }
  }
  out->valid = true;
  out->D_o = best;
  out->n_o = obstacles.T_grid_eval.rotateInverse(normal);
  out->x_o = obstacles.T_grid_eval.applyInverse(nearest->site_grid);
  out->x_o_grid = nearest->site_grid;
  return true;
}

/// @brief The field exit tests of an episode at x (math_spec §6.4 "Exit"): the distance test
/// \f$D_o > D'_{\mathrm{in}} + h_c\f$ against K, and the direction test \f$n^\top\Phi \ge
/// 0.05\lVert\Phi\rVert\f$ with the nominal \f$\Phi\f$, measured to the blocking set \f$B(s^\star)\f$
/// in `path_blocked` mode (an empty set means the blocking part is behind: exited) and to K in
/// `always` mode. Gate closing, K lost and the goal veto are decided by the preparation.
bool episodeExits(
  const StepEnv & env, double s_star, const Vec2 & x, const Vec2 & base, const Vec2 & Phi,
  const ObstacleSample & k_obstacle) noexcept
{
  const double d_in_prime = env.model.d_in_prime;
  if (!env.obstacles.use_blocking) {
    return contourExitCondition(k_obstacle.D_o, k_obstacle.n_o, Phi, d_in_prime);
  }
  ObstacleSample blocking;
  if (!nearestBlockingSite(env, s_star, x, base, &blocking)) {
    return true;
  }
  return contourExitCondition(k_obstacle.D_o, blocking.n_o, Phi, d_in_prime);
}

/// @brief \f$\mathrm{EDT}_{\mathrm{all}}\f$ distance at a point (\f$+\infty\f$ off the grid,
/// without sites or without a map).
double allSitesDistance(const ObstacleEnv & obstacles, const Vec2 & x_eval) noexcept
{
  if (!obstacles.valid) {
    return kInf;
  }
  const ObstacleQuery query = obstacles.edt_all->query(obstacles.T_grid_eval.apply(x_eval));
  return query.valid ? query.D_o : kInf;
}

// -----------------------------------------------------------------------------------------------
// Probes (math_spec §7.4; Appendix A field_grid)

/// @brief Probe evaluation: closest point with the frozen hint (never kept), path field with
/// the law's goal shift, the latched episode where its exit test does not fire, speed shaping
/// with the step's \f$\gamma\f$ terms, blend. \f$\psi_d\f$ is NaN below the hold speed
/// (Appendix C item 5).
ProbeSample evaluateProbe(
  const StepEnv & env, const TrackerState & st, const Pose2 & pose, const Vec2 & x, double t,
  double x_ref, const SpeedShaping & shaping) noexcept
{
  ProbeSample probe;
  if (!isFinite(x) || !std::isfinite(t)) {
    return probe;
  }
  const Model & model = env.model;
  const Isometry2 frame = curveFrameAt(env.input, t);
  const CurveQuery query = closestPointAt(env, frame, st.hint, st.last_tangent, x, t);
  if (!query.cp.valid) {
    return probe;
  }
  probe.s_star = query.cp.s_star;
  probe.D = query.cp.D;

  EvaluationPoint point;
  point.x = x;
  point.x_ref = x_ref;
  FieldSample sample;
  evaluatePathField(env, frame, point, query, t, &sample);
  if (!sample.valid || !isFinite(sample.Phi)) {
    return probe;
  }

  bool active = false;
  double theta = 1.0;
  ObstacleSample obstacle;
  if (st.contour.active && env.obstacles.k_valid) {
    obstacle = queryEpisodeObstacle(env.obstacles, x, pose.p);
    active = obstacle.valid && !episodeExits(env, query.cp.s_star, x, pose.p, sample.Phi, obstacle);
    if (active) {
      theta = contourBlendWeight(obstacle.D_o, model.d_in, model.d_in_prime);
    }
  }
  model.field.shape(shaping.gamma(), shaping.gamma_lim, active && theta < 1.0, &sample);
  Vec2 F = sample.F;
  if (active) {
    const Vec2 psi = contourField(
      obstacle.n_o, obstacle.D_o, model.lambda, st.contour.sigma_c, model.gain,
      sample.Phi_gamma.norm());
    F = blendContour(sample.Phi_gamma, psi, theta);
  }
  probe.F = F;
  const double norm = F.norm();
  probe.psi_d = (norm > 0.0 && norm >= env.hold_speed) ? angleOf(F) : kNaN;
  probe.contour_active = active;
  probe.sigma_c = active ? st.contour.sigma_c : static_cast<std::int8_t>(0);
  return probe;
}

/// @brief The FieldProbe of one stepNominal: the field the law would track at a point with the
/// step's frozen hint, K, gate and episode, and its speed shaping.
struct ProbeContext
{
  const StepEnv * env;
  const TrackerState * state;
  Pose2 base;
  double time;
  double x_ref;
  SpeedShaping shaping;

  ProbeSample operator()(const Vec2 & x, double time_offset) const noexcept
  {
    return evaluateProbe(*env, *state, base, x, time + time_offset, x_ref, shaping);
  }
};

// -----------------------------------------------------------------------------------------------
// stepNominal stages

/// @brief A. XY reached: the caller's value on the first step (plugin), else the core latch
/// \f$\lVert p - g\rVert \le\f$ xy_tol (follower; rollouts emulate it for the plugin).
bool xyReached(
  const GoalEnv & goal, const BaseGeometry & base, bool first, TrackerState * st) noexcept
{
  if (!goal.active) {
    return false;
  }
  if (first && goal.xy_given.has_value()) {
    return *goal.xy_given;
  }
  if (base.goal_distance <= goal.xy_tol) {
    st->xy_latched = true;
  }
  return st->xy_latched;
}

/// @brief B. The shaping terms that do not depend on the field (math_spec §8.2).
SpeedShaping shapingTerms(
  const StepEnv & env, const BaseGeometry & base, const Pose2 & pose, double gamma_col) noexcept
{
  const Model & model = env.model;
  const ObstacleConfig & obstacles = model.config.obstacles;
  SpeedShaping shaping;
  shaping.gamma_lim = env.gamma_lim;
  shaping.gamma_col = gamma_col;
  if (env.obstacles.valid && obstacles.clearance_slowdown_distance > 0.0) {
    shaping.gamma_cl = clearanceScale(
      allSitesDistance(env.obstacles, pose.p), model.w_lat, obstacles.clearance_slowdown_distance,
      obstacles.clearance_slowdown_gain);
  }
  if (env.input.cap_at_curve_end) {
    shaping.gamma_ap = approachScale(
      base.sigma_end, model.config.phases.min_approach_speed, model.kinematics.max_linear_decel,
      model.v_r);
  }
  return shaping;
}

/// @brief C. Transitions 2-5 of math_spec §9.2 that need no field: STOPPED by cancel (or the
/// latch); settled -> ROTATE_TO_HEADING or goal hold; the ROTATE_TO_HEADING exit when XY is lost;
/// FINAL_APPROACH entry (and its exit when terminal mode is left). May request a zero target.
Phase transitionsBeforeField(
  const StepEnv & env, const BaseGeometry & base, bool xy, bool settled, bool cap_binds,
  TrackerState * st, bool * zero_target) noexcept
{
  Phase phase = st->phase;
  if (phase == Phase::kStopped) {
    return phase;  // latched until goal_seq changes or reset()
  }
  if (env.input.cancel) {
    return Phase::kStopped;
  }
  // Goal hold latch: hold_k = settled_k || (hold_{k-1} && XY_k) (Appendix C item 13).
  if (st->holding && !xy) {
    st->holding = false;
  }
  if (
    settled &&
    (phase == Phase::kFollow || phase == Phase::kFinalApproach || phase == Phase::kRotateToField)) {
    // Precedence: settled through a binding cap in a translation phase commands 0 this cycle;
    // the rotation or the hold starts next cycle, whose rollout checks the swept rotation.
    if (cap_binds && isTranslationPhase(phase)) {
      *zero_target = true;
    }
    if (env.goal.rth_applies) {
      phase = Phase::kRotateToHeading;
      st->rotate_target = kNaN;
      st->holding = false;
    } else {
      st->holding = true;
    }
  }
  if (phase == Phase::kRotateToHeading && !xy) {
    phase = base.final_approach_entry ? Phase::kFinalApproach : Phase::kFollow;
  }
  if (!st->holding) {
    if (phase == Phase::kFollow && base.final_approach_entry) {
      phase = Phase::kFinalApproach;
    } else if (phase == Phase::kFinalApproach && !env.goal.terminal) {
      phase = Phase::kFollow;  // FINAL_APPROACH exists only in terminal mode (math_spec §5.5)
    }
  }
  return phase;
}

/// @brief The contour of one step at the evaluation point (math_spec §6.4).
struct ContourStep
{
  bool active{false};
  double theta{1.0};
  ObstacleSample obstacle;
  /// The watchdog tripped this step: the failed episode ended, and its final counters are the
  /// ones this cycle reports.
  bool tripped{false};
  double tripped_travelled_m{0.0};
  double tripped_elapsed_s{0.0};
};

/// @brief E. Continues an episode that the preparation started or kept: the field exit tests
/// with the nominal \f$\Phi\f$, the blend weight, k_seed and the watchdog. Never starts one
/// (only the preparation's side probe does).
ContourStep continueEpisode(
  const StepEnv & env, const Vec2 & x, const Pose2 & pose, const FieldSample & sample,
  bool field_ok, const StepParams & params, TrackerState * st, std::uint32_t * status) noexcept
{
  ContourStep step;
  const Model & model = env.model;
  ContourState & contour = st->contour;
  if (!contour.active) {
    return step;
  }
  if (!env.obstacles.k_valid) {
    endEpisode(&contour);  // K lost (the preparation already ended it; defensive)
    return step;
  }
  if (!field_ok) {
    return step;  // no usable field: the episode is kept but not applied this step
  }
  step.obstacle = queryEpisodeObstacle(env.obstacles, x, pose.p);
  if (
    !step.obstacle.valid ||
    episodeExits(env, sample.s_star, x, pose.p, sample.Phi, step.obstacle)) {
    endEpisode(&contour);
    return step;
  }
  step.active = true;
  step.theta = contourBlendWeight(step.obstacle.D_o, model.d_in, model.d_in_prime);
  contour.k_seed = step.obstacle.x_o_grid;
  contour.k_valid = true;
  if (!params.skip_contour_travel) {
    const double dt = params.dt > 0.0 ? params.dt : 0.0;
    contour.travelled_m += std::hypot(params.measured.vx, params.measured.vy) * dt;
    contour.elapsed_s += dt;
    if (contourWatchdogTripped(contour.travelled_m, contour.elapsed_s, model.config.obstacles)) {
      *status |= Status::kContourWatchdog;  // -> STOPPED; the failed episode ends
      step.tripped = true;
      step.tripped_travelled_m = contour.travelled_m;
      step.tripped_elapsed_s = contour.elapsed_s;
      endEpisode(&contour);
      step.active = false;
    }
  }
  return step;
}

/// @brief Fills FieldSample::contour (Appendix A "contour"). Without an episode, x_o and D_o
/// come from all sites (visualization only).
void fillContourSample(
  const StepEnv & env, const TrackerState & st, const ContourStep & step, const Vec2 & x,
  bool visualize, FieldSample * sample) noexcept
{
  const ObstacleEnv & obstacles = env.obstacles;
  ContourSample & out = sample->contour;
  out.gate_open = obstacles.gate_open;
  out.active = step.active;
  out.sigma_c = step.active ? st.contour.sigma_c : static_cast<std::int8_t>(0);
  out.goal_veto = obstacles.goal_veto;
  out.theta = step.active ? step.theta : 1.0;
  out.travelled_m = step.tripped ? step.tripped_travelled_m : st.contour.travelled_m;
  out.elapsed_s = step.tripped ? step.tripped_elapsed_s : st.contour.elapsed_s;
  out.has_blocked_sample = obstacles.has_blocked_sample;
  out.first_blocked_sample = obstacles.first_blocked_sample;
  ObstacleSample obstacle = step.obstacle;
  if (!step.active && visualize && obstacles.valid) {
    obstacle = queryObstacle(*obstacles.edt_all, obstacles, x);
  }
  if (obstacle.valid) {
    out.x_o = obstacle.x_o;
    out.D_o = obstacle.D_o;
    out.n_o = obstacle.n_o;
  }
}

/// @brief The tracked field without the collision cap, for the ROTATE_TO_FIELD decision. Equal to
/// `sample.F` unless the cap binds; then a copy of the evaluated sample is re-shaped with
/// \f$\gamma_{\mathrm{col}} = 1\f$ (DistanceField::shape() is idempotent on an evaluated sample)
/// and blended again.
/// @param gamma Receives the \f$\gamma\f$ of the returned field.
Vec2 decisionField(
  const StepEnv & env, const NominalResult & result, const ContourStep & contour,
  const TrackerState & st, double * gamma) noexcept
{
  const Model & model = env.model;
  const FieldSample & sample = result.sample;
  *gamma = sample.gamma;
  if (!result.cap_binds || !sample.valid) {
    return sample.F;
  }
  SpeedShaping uncapped = result.shaping;
  uncapped.gamma_col = 1.0;
  FieldSample decision = sample;
  model.field.shape(
    uncapped.gamma(), uncapped.gamma_lim, contour.active && contour.theta < 1.0, &decision);
  *gamma = decision.gamma;
  if (!contour.active) {
    return decision.F;
  }
  const Vec2 psi = contourField(
    contour.obstacle.n_o, contour.obstacle.D_o, model.lambda, st.contour.sigma_c, model.gain,
    decision.Phi_gamma.norm());
  return blendContour(decision.Phi_gamma, psi, contour.theta);
}

/// @brief H. Transition 6: ROTATE_TO_FIELD entry and exit on the tracked field F at the law's
/// own point (math_spec §9.2). `F` is shaped without the collision cap (decisionField()) and its
/// direction is used only when \f$\lVert F\rVert \ge 0.05\,v_r\f$ before speed shaping, i.e.
/// \f$\lVert F\rVert \ge 0.05\,v_r\gamma\f$ after it; otherwise an active rotation keeps its
/// target. The decision therefore never depends on the collision cap: the capped re-run decides
/// the phase exactly as the rollout's first step did.
Phase rotateToFieldTransition(
  const StepEnv & env, Phase phase, Phase tracking, const Pose2 & pose, const Vec2 & F,
  double gamma, bool field_ok, TrackerState * st) noexcept
{
  if (
    st->holding || !(phase == Phase::kFollow || phase == Phase::kFinalApproach ||
                     phase == Phase::kRotateToField)) {
    return phase;
  }
  const Model & model = env.model;
  const PhaseConfig & phases = model.config.phases;
  if (!(model.kinematics.isUnicycle() && phases.use_rotate_to_field)) {
    if (phase == Phase::kRotateToField) {
      st->rotate_target = kNaN;  // not allowed for this robot (e.g. after a rebuild)
      return tracking;
    }
    return phase;
  }
  const double norm = F.norm();
  const bool usable =
    field_ok && gamma > 0.0 && norm > 0.0 && norm >= kHoldSpeedFraction * model.v_r * gamma;
  double target = kNaN;
  if (usable) {
    target = wrapAngle(angleOf(F) + (model.kinematics.drive_backward ? kPi : 0.0));
  }
  if (phase != Phase::kRotateToField) {
    if (usable && std::abs(wrapAngle(target - pose.yaw)) > phases.rotate_to_field_angle) {
      st->rotate_target = target;
      return Phase::kRotateToField;
    }
    return phase;
  }
  if (usable) {
    st->rotate_target = target;
  }
  if (
    !std::isfinite(st->rotate_target) ||
    std::abs(wrapAngle(st->rotate_target - pose.yaw)) < phases.rotate_to_field_release_angle) {
    st->rotate_target = kNaN;
    return tracking;
  }
  return phase;
}

/// @brief The kinematic box of a law command (math_spec §8.3): the \f$\eta\f$-box for a
/// time-varying field under FL or holonomic when shaping took the \f$\eta_1\f$ branch, the
/// uniform \f$c\f$ otherwise.
BoxResult kinematicBox(
  const StepEnv & env, Phase phase, const Pose2 & pose, const FieldSample & sample,
  const Twist2 & law_cmd) noexcept
{
  const Model & model = env.model;
  const Kinematics & kinematics = model.kinematics;
  const TrackingLaw active = model.law.activeLaw(phase);
  const bool linear_law =
    active == TrackingLaw::kFeedbackLinearization || active == TrackingLaw::kHolonomic;
  if (sample.uniform_shaping || !linear_law) {
    return uniformBox(law_cmd, kinematics, env.omega_bar);
  }
  Vec2 a;
  Vec2 b;
  if (active == TrackingLaw::kFeedbackLinearization) {
    const Mat2 M = flMatrix(pose.yaw, kinematics.control_point_offset, kinematics.driveSign());
    a = M * sample.Phi_T;
    b = M * sample.Phi_S;
  } else {
    a = rotate(-pose.yaw, sample.Phi_T);
    b = rotate(-pose.yaw, sample.Phi_S);
  }
  return etaBox(a, b, sample.eta, law_cmd.wz, law_cmd, kinematics, env.omega_bar);
}

/// @brief I (translation phases): the law, the FOLLOW-only Ackermann band, the kinematic box
/// and the reversing backstop (math_spec §7, §8.3, §9.2). Updates last_psi_d and the sample's
/// heading quantities.
Twist2 translationTarget(
  const StepEnv & env, Phase phase, const Pose2 & pose, double t, const EvaluationPoint & point,
  const StepParams & params, TrackerState * st, NominalResult * result) noexcept
{
  const Model & model = env.model;
  const Kinematics & kinematics = model.kinematics;
  FieldSample & sample = result->sample;

  LawInput law_input;
  law_input.pose = pose;
  law_input.x = point.x;
  law_input.F = sample.F;
  law_input.T_d = sample.T_d;
  law_input.dt = params.dt;
  law_input.omega_bar = env.omega_bar;
  law_input.hold_speed = env.hold_speed;
  law_input.last_psi_d = st->last_psi_d;
  law_input.curve_length = env.closed ? env.length : kInf;
  law_input.vertex_spacing = env.vertex_spacing;
  law_input.time_varying = env.time_varying;

  const ProbeContext probe{&env, st, pose, t, point.x_ref, result->shaping};
  const LawOutput law = model.law.track(phase, law_input, probe);
  result->status |= law.status;
  if (std::isfinite(law.psi_d)) {
    st->last_psi_d = law.psi_d;
    sample.target_heading = law.psi_d;
    sample.has_target_heading = true;
  }
  sample.psi_d = law.psi_d;
  sample.psi_d_dot = law.psi_d_dot;
  sample.heading_error = law.heading_error;

  Twist2 cmd = law.cmd;
  if (
    kinematics.isAckermann() && phase == Phase::kFollow && !st->ackermann_band_lifted &&
    !result->cap_binds) {
    // Positive speeds below v_ack are raised; only the cap, kBlocked and STOPPED command 0.
    cmd.vx = applyAckermannBand(cmd.vx, kinematics.ackermann_min_speed);
  }
  const BoxResult box = kinematicBox(env, phase, pose, sample, cmd);
  result->status |= box.status;
  result->box_scale = box.c;
  if (box.eta_box) {
    sample.eta = box.eta;
  }
  cmd = box.cmd;
  if (!kinematics.allow_reversing && !kinematics.isHolonomic()) {
    // Backstop: q v >= 0 in the drive direction (Ackermann drives forward only).
    cmd = applyReversingBackstop(cmd, kinematics.isAckermann() ? 1.0 : kinematics.driveSign());
  }
  return cmd;
}

/// @brief Ackermann's extra kHeadingInfeasible clauses (math_spec §7.5): \f$\sigma_{\mathrm{base}}
/// < 0\f$ while XY is not reached, or, at settled, a yaw outside the checked tolerance.
std::uint32_t ackermannInfeasible(
  const StepEnv & env, const BaseGeometry & base, const Pose2 & pose, bool xy,
  bool settled) noexcept
{
  const GoalEnv & goal = env.goal;
  if (!env.model.kinematics.isAckermann() || !goal.active) {
    return Status::kNone;
  }
  if (base.sigma < 0.0 && !xy) {
    return Status::kHeadingInfeasible;
  }
  if (settled && goal.yaw_checked && std::abs(wrapAngle(goal.yaw - pose.yaw)) > goal.yaw_tol) {
    return Status::kHeadingInfeasible;
  }
  return Status::kNone;
}

/// @brief One nominal control cycle on `st` at `pose` and time t (math_spec §8.1 with the
/// transitions of §9.2). The Tracker's step, its rollouts and its side probe all run it.
NominalResult stepNominal(
  const StepEnv & env, TrackerState * st, const Pose2 & pose, double t,
  const StepParams & params) noexcept
{
  const Model & model = env.model;
  const GoalEnv & goal = env.goal;
  const Kinematics & kinematics = model.kinematics;
  NominalResult result;
  FieldSample & sample = result.sample;
  const Isometry2 frame = curveFrameAt(env.input, t);

  // A. base_link geometry and XY reached.
  const BaseGeometry base = evaluateBase(env, *st, frame, pose, t);
  const bool xy = xyReached(goal, base, params.first, st);
  if (kinematics.isAckermann() && goal.terminal) {
    const double v_ack = kinematics.ackermann_min_speed;
    if (base.sigma < goal.xy_tol + v_ack * v_ack / (2.0 * kinematics.max_linear_decel)) {
      st->ackermann_band_lifted = true;  // goal-scoped latch (math_spec §7.5)
    }
  }

  // B. Field-independent shaping terms.
  result.shaping = shapingTerms(env, base, pose, params.gamma_col);
  result.cap_binds = result.shaping.capBinds();

  // C. Transitions 2-5.
  // math_spec §9.1: the along-track clause sigma_base <= sigma_stop only for Ackermann, which
  // cannot correct a lateral offset without reversing; the other laws converge laterally under
  // the terminal field, so they settle within sigma_stop of the goal.
  const bool along_track = kinematics.isAckermann() && base.sigma <= goal.sigma_stop;
  const bool settled =
    goal.active && xy && (base.goal_distance <= goal.sigma_stop || result.cap_binds || along_track);
  bool zero_target = params.zero_target;
  Phase phase = transitionsBeforeField(env, base, xy, settled, result.cap_binds, st, &zero_target);
  const Phase tracking = trackingPhase(phase, base.final_approach_entry);

  // D. Nominal path field at the tracking phase's evaluation point.
  const EvaluationPoint point = model.law.evaluationPoint(pose, tracking);
  const CurveQuery query = (base.has_base && point.x == pose.p)
                             ? base.base
                             : closestPointAt(env, frame, st->hint, st->last_tangent, point.x, t);
  if (query.cp.valid) {
    st->hint = makeHint(query.cp);
    st->last_tangent = query.cp.T;
  }
  evaluatePathField(env, frame, point, query, t, &sample);
  sample.sigma = base.sigma;
  sample.sigma_end = base.sigma_end;
  const bool field_ok = sample.valid && isFinite(sample.Phi);

  // E. Contour episode (decisions on the nominal Phi).
  const ContourStep contour =
    continueEpisode(env, point.x, pose, sample, field_ok, params, st, &result.status);

  // F. Speed shaping, then G. the contour blend with speed ||Phi_gamma||.
  model.field.shape(
    result.shaping.gamma(), result.shaping.gamma_lim, contour.active && contour.theta < 1.0,
    &sample);
  if (contour.active) {
    const Vec2 psi = contourField(
      contour.obstacle.n_o, contour.obstacle.D_o, model.lambda, st->contour.sigma_c, model.gain,
      sample.Phi_gamma.norm());
    sample.F = blendContour(sample.Phi_gamma, psi, contour.theta);
  }
  fillContourSample(env, *st, contour, point.x, params.visualize, &sample);

  // H. Transition 6, on the field without the collision cap.
  double decision_gamma = 1.0;
  const Vec2 decision_F = decisionField(env, result, contour, *st, &decision_gamma);
  phase =
    rotateToFieldTransition(env, phase, tracking, pose, decision_F, decision_gamma, field_ok, st);

  // I. Law or phase target.
  Twist2 target;
  if (phase == Phase::kRotateToField) {
    const double e_r =
      std::isfinite(st->rotate_target) ? wrapAngle(st->rotate_target - pose.yaw) : 0.0;
    target.wz = model.law.rotate(phase, e_r, env.omega_bar, goal.yaw_tol);
    sample.heading_error = e_r;
    sample.target_heading = st->rotate_target;
    sample.has_target_heading = std::isfinite(st->rotate_target);
  } else if (phase == Phase::kRotateToHeading) {
    const double e_r = wrapAngle(goal.yaw - pose.yaw);
    target.wz = model.law.rotate(phase, e_r, env.omega_bar, goal.yaw_tol);
    sample.heading_error = e_r;
    sample.target_heading = goal.yaw;
    sample.has_target_heading = true;
  } else if (isTranslationPhase(phase) && !st->holding) {
    if (field_ok) {
      target = translationTarget(env, phase, pose, t, point, params, st, &result);
    } else {
      target = Twist2{kNaN, kNaN, kNaN};  // the limiter's guard: u = 0 and kNonFinite
    }
  }
  result.status |= ackermannInfeasible(env, base, pose, xy, settled);

  // J. STOPPED-entry statuses raised this cycle enter STOPPED (transition 2's precedence).
  const std::uint32_t entry = result.status & Status::kStoppedEntry;
  if (entry != Status::kNone) {
    st->stopped_status |= entry;
    phase = Phase::kStopped;
  }
  if (phase == Phase::kStopped) {
    st->holding = false;
    st->rotate_target = kNaN;
    result.status |= st->stopped_status;
  }
  if (phase == Phase::kStopped || zero_target || st->holding) {
    target = Twist2{};  // bypasses the Ackermann band, the rotate floors and the law
  }

  // K. CommandLimiter (re-seed first: initial state, reset, rebuild, or Delta t > 2/f).
  if (!st->limiter.valid || (params.first && env.reseed_by_dt)) {
    st->limiter.u_prev = params.measured;
    st->limiter.valid = true;
  }
  const bool stopped_cancel = phase == Phase::kStopped && env.input.cancel;
  const CommandLimiter limiter;
  result.target = isFinite(target) ? target : Twist2{};  // a non-finite target reports as 0
  result.cmd = limiter.limit(
    target, st->limiter, &st->limiter, params.dt, kinematics, stopped_cancel, &result.status,
    &result.alpha);

  if (result.shaping.collisionAhead()) {
    result.status |= Status::kCollisionAhead;
  }
  result.status |= sample.status;
  st->phase = phase;
  result.phase = phase;
  result.settled = settled;
  result.xy_reached = xy;
  return result;
}

// -----------------------------------------------------------------------------------------------
// Rollout policies

/// @brief Parameters of rollout step k: the first step uses the measured period and twist (so
/// that it equals the final command when reused); later steps the nominal period and the
/// previous rollout command.
StepParams rolloutParams(const StepEnv & env, const TrackerState & work, std::size_t k) noexcept
{
  StepParams params;
  params.first = k == 0;
  params.dt = params.first ? env.input_dt : env.model.period;
  params.measured = params.first ? env.input.measured : work.limiter.u_prev;
  params.skip_contour_travel = params.first && env.obstacles.episode_started;
  params.visualize = params.first;
  params.gamma_col = 1.0;
  return params;
}

/// @brief The rollout step of a nominal result. A settled step ends the rollout (math_spec §10.2)
/// except in a rotation phase: ROTATE_TO_HEADING is entered at settled and stays settled, and its
/// rollout must sweep the remaining rotation (kBlocked's rotation rule, math_spec §9.2, §10.4).
RolloutStep rolloutStep(const NominalResult & result) noexcept
{
  return RolloutStep{result.cmd, result.phase, result.settled && !isRotationPhase(result.phase)};
}

/// @brief Seeds the rollout's emulated XY latch with the caller's value after the first step
/// (later poses have no goal-checker answer; the Gen A/B latch is emulated).
void seedRolloutXyLatch(const StepEnv & env, TrackerState * work) noexcept
{
  if (env.goal.xy_given.has_value() && *env.goal.xy_given) {
    work->xy_latched = true;
  }
}

/// @brief The main rollout (math_spec §10.2): stepNominal with \f$\gamma_{\mathrm{col}} = 1\f$
/// on a copy of the state; keeps the first step for reuse as the final command.
struct MainRolloutPolicy
{
  MainRolloutPolicy(const StepEnv & step_env, const TrackerState & start) noexcept
  : env(step_env), work(start)
  {
  }

  RolloutStep operator()(const Pose2 & pose, double t) noexcept
  {
    const StepParams params = rolloutParams(env, work, k);
    const NominalResult result = stepNominal(env, &work, pose, t, params);
    if (k == 0) {
      first = result;
      first_state = work;
      captured = true;
      seedRolloutXyLatch(env, &work);
    }
    ++k;
    return rolloutStep(result);
  }

  const StepEnv & env;
  TrackerState work;
  std::size_t k{0};
  bool captured{false};
  NominalResult first;
  TrackerState first_state;
};

/// @brief One side-probe rollout (math_spec §6.4): the episode forced on at one side; stops at
/// the episode-exit test (reported through `settled`), a collision or \f$T_{\mathrm{side}}\f$.
struct SideProbePolicy
{
  SideProbePolicy(
    const StepEnv & step_env, const TrackerState & start, int side, const Vec2 & seed) noexcept
  : env(step_env), work(start)
  {
    ContourState & contour = work.contour;
    contour.active = true;
    contour.sigma_c = static_cast<std::int8_t>(side);
    contour.travelled_m = 0.0;
    contour.elapsed_s = 0.0;
    contour.k_seed = seed;
    contour.k_valid = true;
  }

  RolloutStep operator()(const Pose2 & pose, double t) noexcept
  {
    if (k > 0) {
      arc += detail::baseSpeed(env.model.kinematics, previous_cmd) * env.model.period;
    }
    StepParams params = rolloutParams(env, work, k);
    params.skip_contour_travel = k == 0;
    params.visualize = false;
    const NominalResult result = stepNominal(env, &work, pose, t, params);
    if (k == 0) {
      seedRolloutXyLatch(env, &work);
    }
    ++k;
    previous_cmd = result.cmd;
    if (!work.contour.active) {
      // The exit test ended the episode here (a watchdog trip is a failure, not an exit).
      if ((result.status & Status::kContourWatchdog) == 0U) {
        exit_reached = true;
        path_to_exit = arc;
      }
      return RolloutStep{result.cmd, result.phase, true};
    }
    return rolloutStep(result);
  }

  const StepEnv & env;
  TrackerState work;
  std::size_t k{0};
  double arc{0.0};
  Twist2 previous_cmd;
  bool exit_reached{false};
  double path_to_exit{kInf};
};

// -----------------------------------------------------------------------------------------------
// Obstacle preparation (once per step)

/// @brief Validates the cycle's obstacle map: a grid and an \f$\mathrm{EDT}_{\mathrm{all}}\f$ of
/// the same size (built by the caller for this grid).
void bindObstacleMap(
  const TrackerInput & input, const EuclideanDistanceTransform * edt_k,
  ObstacleEnv * obstacles) noexcept
{
  obstacles->edt_k = edt_k;
  if (!input.obstacles.has_value()) {
    return;
  }
  const ObstacleMap & map = *input.obstacles;
  if (
    map.grid == nullptr || map.edt == nullptr || map.grid->width() <= 0 ||
    map.grid->height() <= 0 || map.edt->width() != map.grid->width() ||
    map.edt->height() != map.grid->height()) {
    return;
  }
  obstacles->valid = true;
  obstacles->grid = map.grid;
  obstacles->edt_all = map.edt;
  obstacles->T_grid_eval = map.T_grid_eval;
  obstacles->resolution = map.grid->resolution();
}

/// @brief Upper bound on the checked poses of a rollout of `horizon` seconds: one start pose plus,
/// per period, the sub-steps of math_spec §10.2 at the velocity limits (capped against absurd
/// configurations; a larger rollout still works, its buffer grows on demand).
std::size_t rolloutPoseBound(const Model & model, double horizon, double resolution) noexcept
{
  constexpr double kMaxPoses = 1e6;
  const Kinematics & kinematics = model.kinematics;
  const double period = model.period;
  double speed = kinematics.max_linear_vel;
  if (kinematics.isAckermann()) {
    speed = std::hypot(speed, kinematics.reference_point_offset * kinematics.max_angular_vel);
  }
  const double r_circ = model.checker.footprint().rCirc();
  const double yaw_step =
    2.0 * r_circ > resolution ? 2.0 * std::asin(resolution / (2.0 * r_circ)) : kInf;
  const double per_period = std::max(
    {1.0, std::ceil(speed * period / resolution),
     std::ceil(kinematics.max_angular_vel * period / yaw_step)});
  const double poses = (std::ceil(horizon / period) + 1.0) * per_period + 1.0;
  return static_cast<std::size_t>(std::min(poses, kMaxPoses));
}

/// @brief Grows the scratch buffers to the sizes this map and configuration can need, so that
/// they are allocated on the first step with a map rather than when the gate first opens or the
/// first side probe runs (oracle O55). A no-op once the capacity suffices; never shrinks.
void reserveScratch(const Model & model, const ObstacleEnv & obstacles, StepScratch * scratch)
{
  const OccupancyGrid & grid = *obstacles.grid;
  const double resolution = obstacles.resolution;
  if (!(resolution > 0.0)) {
    return;
  }
  const SafetyConfig & safety = model.config.safety;
  const Kinematics & kinematics = model.kinematics;
  // The main rollout may follow a cancel ramp past the horizon. One alpha scales every axis, so
  // the ramp lasts as long as its slowest axis: linear at cancel_deceleration, angular at
  // max_angular_accel (math_spec §8.4).
  const double linear_ramp = kinematics.cancel_deceleration > 0.0
                               ? kinematics.max_linear_vel / kinematics.cancel_deceleration
                               : 0.0;
  const double angular_ramp = kinematics.max_angular_accel > 0.0
                                ? kinematics.max_angular_vel / kinematics.max_angular_accel
                                : 0.0;
  const double ramp = std::max(linear_ramp, angular_ramp);
  scratch->rollout.reserve(
    rolloutPoseBound(model, safety.collision_check_horizon + ramp, resolution));
  if (model.config.obstacles.obstacle_mode != ObstacleMode::kContour) {
    return;
  }
  const double side_horizon = sideProbeHorizon(
    safety.collision_check_horizon, model.d_in_prime, model.lambda, model.v_r,
    std::max(0.0, model.config.obstacles.obstacle_lookahead));
  scratch->side_rollout.reserve(rolloutPoseBound(model, side_horizon, resolution));
  const std::size_t cells =
    static_cast<std::size_t>(grid.width()) * static_cast<std::size_t>(grid.height());
  scratch->k_mask.reserve(cells);
  scratch->flood_queue.reserve(cells);
  scratch->edt_k.reserve(grid.width(), grid.height());
  const double lookahead = std::max(0.0, model.config.obstacles.obstacle_lookahead);
  if (std::isfinite(lookahead)) {
    const auto samples = static_cast<std::size_t>(std::floor(lookahead / resolution)) + 3U;
    scratch->gate_samples.reserve(samples);
    scratch->gate_samples_grid.reserve(samples);
    scratch->gate_blocked.reserve(samples);
    scratch->blocking.reserve(samples);
  }
}

/// @brief Gate samples \f$r(s)\f$ one cell apart over the lookahead in the traversal direction,
/// rays excluded (math_spec §6.4), in traversal order; marks the blocked ones.
void sampleGate(
  const StepEnv & env, const Isometry2 & frame, double s_star, StepScratch * scratch,
  ObstacleEnv * obstacles) noexcept
{
  const Model & model = env.model;
  const ObstacleConfig & config = model.config.obstacles;
  const double resolution = obstacles->resolution;
  const double lookahead = std::max(0.0, config.obstacle_lookahead);
  if (!(resolution > 0.0) || !std::isfinite(s_star) || !std::isfinite(lookahead)) {
    return;
  }
  const auto capacity = static_cast<std::size_t>(std::floor(lookahead / resolution)) + 3U;
  std::vector<Vec2> & samples = scratch->gate_samples;
  if (samples.size() < capacity) {
    samples.resize(capacity);
  }
  const bool forward = model.direction > 0;
  const double s0 = forward ? s_star : s_star - lookahead;
  const double s1 = forward ? s_star + lookahead : s_star;
  const std::size_t count = env.input.curve->sampleStations(
    s0, s1, resolution, env.input.t, Span<Vec2>(samples.data(), capacity));
  samples.resize(count);
  // The stations of the samples, laid out as Curve::sampleStations() documents them.
  GateStations & gate = obstacles->gate;
  gate.s0 = env.closed ? s0 : std::max(s0, 0.0);
  gate.s1 = env.closed ? s1 : std::min(s1, env.length);
  gate.ds = resolution;
  gate.count = count;
  gate.forward = forward;
  gate.regular = 0;
  while (gate.regular < count &&
         gate.s0 + static_cast<double>(gate.regular) * resolution <= gate.s1) {
    ++gate.regular;
  }
  if (!forward) {
    std::reverse(samples.begin(), samples.end());  // traversal order: nearest ahead first
  }
  std::vector<Vec2> & grid_samples = scratch->gate_samples_grid;
  grid_samples.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    samples[i] = frame.apply(samples[i]);
    grid_samples[i] = obstacles->T_grid_eval.apply(samples[i]);
  }
  scratch->gate_blocked.resize(count);
  const int first = markBlockedSamples(
    Span<const Vec2>(grid_samples.data(), count), *obstacles->edt_all, model.w_lat,
    config.obstacle_gate_tolerance, Span<std::uint8_t>(scratch->gate_blocked.data(), count));
  if (first >= 0) {
    obstacles->has_blocked_sample = true;
    obstacles->first_blocked_sample = samples[static_cast<std::size_t>(first)];
  }
}

/// @brief The seed cell of K (math_spec §6.4): during an episode k_seed through the current
/// grid origin (or the nearest dilated cell within \f$r_{\mathrm{insc}} + 2\,\mathrm{res}\f$);
/// otherwise the feature cell of the first blocked gate sample (`always`: of the evaluation
/// point, when it is within \f$D'_{\mathrm{in}}\f$ of a site).
bool seedK(
  const StepEnv & env, const TrackerState & st, const Vec2 & x_eval, const ObstacleEnv & obstacles,
  int * seed_x, int * seed_y) noexcept
{
  const Model & model = env.model;
  const OccupancyGrid & grid = *obstacles.grid;
  const EuclideanDistanceTransform & edt_all = *obstacles.edt_all;
  if (!edt_all.hasSites()) {
    return false;
  }
  const ContourState & contour = st.contour;
  int cx = 0;
  int cy = 0;
  if (contour.active) {
    if (!contour.k_valid) {
      return false;
    }
    if (grid.cellAt(contour.k_seed, &cx, &cy) && edt_all.distance(cx, cy) <= model.r_insc) {
      *seed_x = cx;
      *seed_y = cy;
      return true;
    }
    return nearestDilatedCell(
      edt_all, model.r_insc, contour.k_seed, model.r_insc + 2.0 * obstacles.resolution, seed_x,
      seed_y);
  }
  if (!contour.gate_open) {
    return false;
  }
  if (model.config.obstacles.obstacle_activation == ObstacleActivation::kAlways) {
    // The paper's nearest obstacle; farther than D'_in no episode can start, so K is skipped.
    if (
      !grid.cellAt(obstacles.T_grid_eval.apply(x_eval), &cx, &cy) ||
      !(edt_all.distance(cx, cy) <= model.d_in_prime + obstacles.resolution)) {
      return false;
    }
  } else {
    if (
      !obstacles.has_blocked_sample ||
      !grid.cellAt(obstacles.T_grid_eval.apply(obstacles.first_blocked_sample), &cx, &cy)) {
      return false;
    }
  }
  edt_all.feature(cx, cy, seed_x, seed_y);
  return *seed_x >= 0 && *seed_y >= 0;
}

/// @brief K's flood fill and \f$\mathrm{EDT}_K\f$ into the scratch (math_spec §6.4 "Identity").
void buildK(
  const StepEnv & env, int seed_x, int seed_y, StepScratch * scratch,
  ObstacleEnv * obstacles) noexcept
{
  const OccupancyGrid & grid = *obstacles->grid;
  const std::size_t cells = floodFillDilatedComponent(
    *obstacles->edt_all, env.model.r_insc, seed_x, seed_y, &scratch->k_mask, &scratch->flood_queue);
  if (cells == 0) {
    return;
  }
  scratch->edt_k.build(grid, Span<const std::uint8_t>(scratch->k_mask));
  scratch->k_width = grid.width();
  scratch->k_height = grid.height();
  scratch->k_valid = scratch->edt_k.hasSites();
  obstacles->k_valid = scratch->k_valid;
}

/// @brief The blocking set B (math_spec §6.4, `path_blocked`): every blocked gate sample whose
/// feature site lies in K, as (station, site) in traversal order, and its station extent
/// \f$L_B\f$. Built after K; frozen with it for the call.
void buildBlockingSet(StepScratch * scratch, ObstacleEnv * obstacles) noexcept
{
  obstacles->blocking_extent = 0.0;
  if (!obstacles->use_blocking || !obstacles->k_valid) {
    return;
  }
  const OccupancyGrid & grid = *obstacles->grid;
  const EuclideanDistanceTransform & edt_all = *obstacles->edt_all;
  const std::size_t count =
    std::min(scratch->gate_samples_grid.size(), scratch->gate_blocked.size());
  for (std::size_t j = 0; j < count && j < obstacles->gate.count; ++j) {
    if (scratch->gate_blocked[j] == 0U) {
      continue;
    }
    int cx = 0;
    int cy = 0;
    if (!grid.cellAt(scratch->gate_samples_grid[j], &cx, &cy)) {
      continue;
    }
    int fx = -1;
    int fy = -1;
    edt_all.feature(cx, cy, &fx, &fy);
    if (fx < 0 || fy < 0) {
      continue;
    }
    const std::size_t index =
      static_cast<std::size_t>(fy) * static_cast<std::size_t>(scratch->k_width) +
      static_cast<std::size_t>(fx);
    if (index >= scratch->k_mask.size() || scratch->k_mask[index] == 0U) {
      continue;
    }
    scratch->blocking.push_back(BlockingSite{obstacles->gate.station(j), grid.cellCenter(fx, fy)});
  }
  if (scratch->blocking.size() >= 2U) {
    obstacles->blocking_extent = std::abs(scratch->blocking.back().s - scratch->blocking.front().s);
  }
}

/// @brief The side probe (math_spec §6.4): one rollout per side with the episode forced on,
/// the gate frozen and \f$\gamma_{\mathrm{col}} = 1\f$, against the full mask; ranks the sides.
/// @return +1, -1, or 0 (no episode).
int probeSides(
  const StepEnv & env, const TrackerState & st, const Vec2 & seed_grid, const Vec2 & n_start,
  const Vec2 & Phi, StepScratch * scratch) noexcept
{
  const Model & model = env.model;
  const SafetyConfig & safety = model.config.safety;
  RolloutOptions options;
  options.t0 = env.input.t;
  options.period = model.period;
  options.horizon = sideProbeHorizon(
    safety.collision_check_horizon, model.d_in_prime, model.lambda, model.v_r,
    env.obstacles.blocking_extent);
  options.through_cancel = false;

  SideProbeResult results[2];
  const int sides[2] = {1, -1};
  for (int i = 0; i < 2; ++i) {
    SideProbePolicy policy(env, st, sides[i], seed_grid);
    const RolloutResult rollout = detail::runRollout(
      model.kinematics, model.checker, env.input.pose, *env.input.obstacles, options, policy,
      &scratch->side_rollout);
    SideProbeResult & result = results[i];
    result.collision_free = !rollout.collided;
    result.exit_reached = policy.exit_reached;
    result.path_to_exit = policy.exit_reached ? policy.path_to_exit : kInf;
    result.s_free = rollout.s_free;
    double clearance = kInf;
    for (const Pose2 & checked : scratch->side_rollout) {
      clearance = std::min(clearance, allSitesDistance(env.obstacles, checked.p) - model.w_lat);
    }
    result.clearance = clearance;
  }
  const double a_dec = model.kinematics.max_linear_decel;
  const double min_free = model.v_r * model.v_r / (2.0 * a_dec) + safety.stop_margin;
  return chooseContourSide(
    results[0], results[1], env.obstacles.resolution, safety.stop_margin, min_free,
    J(n_start).dot(Phi), Phi.norm());
}

/// @brief Step 0 of plan §3.1: the obstacle preparation of the call, frozen for every
/// stepNominal (rollouts, side probe, final step, probes): the gate, the goal veto, K and
/// \f$\mathrm{EDT}_K\f$, the end of an episode on a frozen condition (gate closed, veto, K
/// lost) and the start of a new one (start test at the current evaluation point, side probe).
void prepareObstacles(StepEnv * env, TrackerState * st, StepScratch * scratch) noexcept
{
  ObstacleEnv & obstacles = env->obstacles;
  const TrackerInput & input = env->input;
  const Model & model = env->model;
  ContourState & contour = st->contour;
  scratch->k_valid = false;
  scratch->gate_samples.clear();
  scratch->gate_samples_grid.clear();
  scratch->gate_blocked.clear();
  bindObstacleMap(input, &scratch->edt_k, &obstacles);
  if (obstacles.valid) {
    reserveScratch(model, obstacles, scratch);
  }
  obstacles.contour_on =
    obstacles.valid && model.config.obstacles.obstacle_mode == ObstacleMode::kContour;
  obstacles.use_blocking = obstacles.contour_on && model.config.obstacles.obstacle_activation ==
                                                     ObstacleActivation::kPathBlocked;
  obstacles.blocking = &scratch->blocking;
  scratch->blocking.clear();
  if (!obstacles.contour_on) {
    if (contour.active) {
      endEpisode(&contour);  // no K without a map
    }
    contour.gate_open = false;
    contour.gate_clear_s = 0.0;
    return;
  }

  // Pre-evaluation at the current pose: the station of the gate samples and the start test's
  // nominal field, at the evaluation point the cycle will use.
  const Isometry2 frame = curveFrameAt(input, input.t);
  const BaseGeometry base = evaluateBase(*env, *st, frame, input.pose, input.t);
  Phase phase = st->phase;
  if (phase == Phase::kFollow && base.final_approach_entry && !st->holding) {
    phase = Phase::kFinalApproach;
  }
  const EvaluationPoint point =
    model.law.evaluationPoint(input.pose, trackingPhase(phase, base.final_approach_entry));
  const CurveQuery query =
    (base.has_base && point.x == input.pose.p)
      ? base.base
      : closestPointAt(*env, frame, st->hint, st->last_tangent, point.x, input.t);

  // Gate (path_blocked), sampled in both modes for the visualizer.
  if (query.cp.valid) {
    sampleGate(*env, frame, query.cp.s_star, scratch, &obstacles);
  }
  if (model.config.obstacles.obstacle_activation == ObstacleActivation::kAlways) {
    contour.gate_open = true;
    contour.gate_clear_s = 0.0;
  } else {
    const GateUpdate gate = updateGate(
      contour.gate_open, contour.gate_clear_s, obstacles.has_blocked_sample, env->input_dt,
      model.config.obstacles.obstacle_release_time);
    contour.gate_open = gate.open;
    contour.gate_clear_s = gate.clear_s;
  }
  obstacles.gate_open = contour.gate_open;

  // Goal veto: the curve end is the goal and r(s_goal) itself is blocked.
  if (env->goal.terminal) {
    const Vec2 g_c = frame.apply(input.curve->pointAt(env->goal.s_goal, input.t));
    const double D_goal = allSitesDistance(obstacles, g_c);
    obstacles.goal_veto = D_goal < model.w_lat - model.config.obstacles.obstacle_gate_tolerance;
  }
  if (contour.active && (!contour.gate_open || obstacles.goal_veto)) {
    endEpisode(&contour);
  }

  // K and EDT_K (when the gate is open or an episode is active).
  int seed_x = -1;
  int seed_y = -1;
  if (seedK(*env, *st, point.x, obstacles, &seed_x, &seed_y)) {
    buildK(*env, seed_x, seed_y, scratch, &obstacles);
  }
  if (contour.active && !obstacles.k_valid) {
    endEpisode(&contour);  // K lost
  }
  buildBlockingSet(scratch, &obstacles);

  // Episode start: gate open, no veto, D_o <= D'_in and n_o^T Phi < 0 measured to K at the
  // evaluation point with the nominal Phi, and a side from the probe. Never while stopping or
  // once arrived (goal hold, ROTATE_TO_HEADING), where the terminal field vanishes.
  if (
    contour.active || !contour.gate_open || obstacles.goal_veto || !obstacles.k_valid ||
    input.cancel || st->phase == Phase::kStopped || st->phase == Phase::kRotateToHeading ||
    st->holding) {
    return;
  }
  FieldSample nominal;
  evaluatePathField(*env, frame, point, query, input.t, &nominal);
  if (!nominal.valid || !isFinite(nominal.Phi)) {
    return;
  }
  // K's nearest point seeds the episode; the start test is measured to the blocking set B in
  // path_blocked mode (no start while B(s*) is empty) and to K with `always`.
  const ObstacleSample obstacle = queryEpisodeObstacle(obstacles, point.x, input.pose.p);
  if (!obstacle.valid) {
    return;
  }
  ObstacleSample start = obstacle;
  if (
    obstacles.use_blocking &&
    !nearestBlockingSite(*env, query.cp.s_star, point.x, input.pose.p, &start)) {
    return;
  }
  if (!contourStartCondition(start.D_o, start.n_o, nominal.Phi, model.d_in_prime)) {
    return;
  }
  const int side = probeSides(*env, *st, obstacle.x_o_grid, start.n_o, nominal.Phi, scratch);
  if (side == 0) {
    return;
  }
  contour.active = true;
  contour.sigma_c = static_cast<std::int8_t>(side);
  contour.travelled_m = 0.0;
  contour.elapsed_s = 0.0;
  contour.k_seed = obstacle.x_o_grid;
  contour.k_valid = true;
  obstacles.episode_started = true;
}

// -----------------------------------------------------------------------------------------------
// Cycle bookkeeping

/// @brief Transition 1 of math_spec §9.2: a goal_seq change resets only goal-scoped state
/// (terminal mode, the XY latch, goal hold, the Ackermann band latch, and FINAL_APPROACH,
/// ROTATE_TO_HEADING and STOPPED -> FOLLOW). The contour episode, the rotate hysteresis, the
/// hint and the limiter persist.
void resetGoalScoped(std::uint32_t goal_seq, TrackerState * st) noexcept
{
  st->goal_seq = goal_seq;
  st->terminal_mode = false;
  st->xy_latched = false;
  st->holding = false;
  st->ackermann_band_lifted = false;
  if (
    st->phase == Phase::kFinalApproach || st->phase == Phase::kRotateToHeading ||
    st->phase == Phase::kStopped) {
    st->phase = Phase::kFollow;
    st->stopped_status = Status::kNone;
    st->limiter.cancel_hard = false;
  }
}

/// @brief The goal of the cycle, and the terminal-mode hysteresis of math_spec §5.5: enter when
/// \f$\lVert g - r(s_{\mathrm{goal}})\rVert\f$ < 0.05 m, leave when it exceeds 0.25 m.
GoalEnv makeGoalEnv(const Model & model, const TrackerInput & input, TrackerState * st) noexcept
{
  GoalEnv goal;
  const Curve & curve = *input.curve;
  if (!input.goal.has_value() || curve.isClosed()) {
    st->terminal_mode = false;
    return goal;
  }
  const GoalInfo & info = *input.goal;
  const CoreConfig & config = model.config;
  goal.active = true;
  goal.g = info.pose.p;
  goal.yaw = info.pose.yaw;
  goal.xy_tol = info.xy_tol;
  goal.yaw_tol = info.yaw_tol;
  goal.sigma_stop = 0.5 * info.xy_tol;
  goal.yaw_checked = info.has_yaw && info.yaw_tol > kYawTolUnchecked;
  goal.rth_applies =
    goal.yaw_checked && config.phases.use_rotate_to_heading && !model.kinematics.isAckermann();
  goal.final_approach = finalApproachDistance(config, info.xy_tol);
  goal.xy_given = info.xy_reached;
  goal.s_goal = model.direction > 0 ? curve.endStation(input.t) : curve.startStation();
  goal.profile = TerminalProfile(
    model.v_r, config.phases.min_approach_speed, config.phases.terminal_braking_distance,
    goal.sigma_stop);

  const Vec2 g_c = input.frame.T_eval_curve.apply(curve.pointAt(goal.s_goal, input.t));
  const double distance = (goal.g - g_c).norm();
  if (st->terminal_mode) {
    if (!(distance <= kTerminalLeaveDistance)) {
      st->terminal_mode = false;
    }
  } else if (distance < kTerminalEnterDistance) {
    st->terminal_mode = true;
  }
  goal.terminal = st->terminal_mode;
  return goal;
}

/// @brief kBlocked of math_spec §10.4 (with the precedence of §9.2): set when the current pose
/// collides; in a translation phase when \f$s_{\mathrm{free}} < m_s + v_{\mathrm{stop}}^2/(2
/// a_{\mathrm{dec}})\f$, released only \f$h_b\f$ above that threshold, never when settled
/// (a binding cap with XY reached settles); in a rotation phase when the rollout collides before
/// leaving the rotation. Not evaluated in STOPPED.
bool evaluateBlocked(
  const Model & model, bool was_blocked, const NominalResult & first, const TrackerState & next,
  const RolloutResult & rollout, double s_free, bool cap_binds, double resolution) noexcept
{
  if (first.phase == Phase::kStopped) {
    return false;
  }
  if (rollout.collided && rollout.first_collision == 0) {
    return true;  // the current pose collides
  }
  if (isRotationPhase(first.phase)) {
    return rollout.collided && rollout.collided_in_rotation;
  }
  if (first.settled || next.holding || (first.xy_reached && cap_binds)) {
    return false;
  }
  // Set below m_s + v_stop^2/(2 a_dec); release h_b above that threshold, so the band stays real
  // when the Ackermann v_stop = v_ack makes the braking term exceed h_b (math_spec §10.4).
  const double a_dec = model.kinematics.max_linear_decel;
  const double set_below =
    model.config.safety.stop_margin + model.v_stop * model.v_stop / (2.0 * a_dec);
  if (was_blocked) {
    return !(s_free >= set_below + blockedReleaseBand(resolution));
  }
  return s_free < set_below;
}

/// @brief Validates a configuration for the constructor; throws ConfigError on any error.
ValidationReport validatedReport(const CoreConfig & configuration)
{
  ValidationReport report = validateConfig(configuration, std::nullopt);
  report.throwOnError();
  return report;
}

}  // namespace

// -------------------------------------------------------------------------------------------------
// StepScratch

void StepScratch::reserve(
  std::size_t grid_cells, std::size_t rollout_poses, std::size_t gate_sample_count)
{
  rollout.reserve(rollout_poses);
  side_rollout.reserve(rollout_poses);
  k_mask.reserve(grid_cells);
  flood_queue.reserve(grid_cells);
  // Any grid shape of `grid_cells` cells fits: rows and columns are at most that long.
  const int cells = static_cast<int>(std::min<std::size_t>(grid_cells, INT_MAX));
  edt_k.reserve(cells, 1);
  gate_samples.reserve(gate_sample_count);
  gate_samples_grid.reserve(gate_sample_count);
  gate_blocked.reserve(gate_sample_count);
  blocking.reserve(gate_sample_count);
}

// -------------------------------------------------------------------------------------------------
// Tracker

Tracker::Tracker(const CoreConfig & configuration)
: config_(resolveAutoValues(configuration)),
  validation_(validatedReport(configuration)),
  kinematics_(Kinematics::fromConfig(config_)),
  gain_(GainFunction::fromConfig(config_.field)),
  law_(config_),
  checker_(Footprint::fromSpec(config_.node.footprint))
{
}

TrackerOutput Tracker::step(
  const TrackerInput & input, const TrackerState & in, TrackerState * out,
  StepScratch & scratch) const noexcept
{
  TrackerOutput output;
  TrackerState state = in;  // `out` may alias `in`: work on a copy and write *out once
  scratch.rollout.clear();
  output.predicted = Span<const Pose2>(scratch.rollout.data(), 0);

  if (input.curve == nullptr || !isFinite(input.pose) || !std::isfinite(input.t)) {
    // Unusable input: stop at once, as the limiter's non-finite guard does.
    state.limiter.u_prev = Twist2{};
    state.limiter.valid = true;
    output.phase = state.phase;
    output.status = Status::kNonFinite;
    output.cancel_complete = input.cancel;
    if (out != nullptr) {
      *out = state;
    }
    return output;
  }

  // 1. Goal-scoped reset, cancel bookkeeping, terminal mode.
  if (input.goal_seq != state.goal_seq) {
    resetGoalScoped(input.goal_seq, &state);
  }
  if (!input.cancel) {
    state.limiter.cancel_hard = false;  // the hard bound lasts for the rest of one cancel only
  }
  const Model model = makeModel(config_, kinematics_, gain_, law_, checker_);
  const GoalEnv goal = makeGoalEnv(model, input, &state);
  StepEnv env = makeStepEnv(model, input, goal);

  // 2. Obstacle preparation (frozen for the whole call).
  prepareObstacles(&env, &state, &scratch);

  StepParams final_params;
  final_params.first = true;
  final_params.dt = env.input_dt;
  final_params.measured = input.measured;
  final_params.skip_contour_travel = env.obstacles.episode_started;
  final_params.visualize = true;

  NominalResult final_result;
  TrackerState final_state = state;
  bool blocked = false;
  double s_free = kInf;
  double cap = kInf;
  std::uint32_t extra_status = Status::kNone;

  if (env.obstacles.valid && config_.safety.use_collision_detection) {
    // 3. Rollout with gamma_col = 1 on a copy of the state.
    RolloutOptions options;
    options.t0 = input.t;
    options.period = model.period;
    options.horizon = config_.safety.collision_check_horizon;
    options.through_cancel = input.cancel;
    MainRolloutPolicy policy(env, state);
    const RolloutResult rollout = detail::runRollout(
      kinematics_, checker_, input.pose, *input.obstacles, options, policy, &scratch.rollout);
    output.first_collision = rollout.first_collision;
    if (!policy.captured) {
      policy.first_state = state;
      policy.first = stepNominal(env, &policy.first_state, input.pose, input.t, final_params);
    }

    // 4. Cap, cancel collision, kBlocked, final command.
    s_free = rollout.s_free;
    cap = brakingCap(s_free, config_.safety.stop_margin, kinematics_.max_linear_decel);
    const double gamma_col = collisionScale(cap, model.v_r);
    SpeedShaping shaping = policy.first.shaping;
    shaping.gamma_col = gamma_col;
    const bool cap_binds = shaping.capBinds();
    bool rerun = cap_binds;
    if (input.cancel && rollout.collided) {
      extra_status |= Status::kCollisionAhead;
      if (!state.limiter.cancel_hard) {
        state.limiter.cancel_hard = true;  // max(a_can, a_dec) for the rest of the cancel
        rerun = true;
      }
    }
    blocked = evaluateBlocked(
      model, state.blocked, policy.first, policy.first_state, rollout, s_free, cap_binds,
      env.obstacles.resolution);
    rerun = rerun || blocked;
    if (rerun) {
      final_state = state;
      final_params.gamma_col = gamma_col;
      final_params.zero_target = blocked;
      final_result = stepNominal(env, &final_state, input.pose, input.t, final_params);
    } else {
      final_result = policy.first;
      final_state = policy.first_state;
      final_result.shaping.gamma_col = gamma_col;  // not binding: the command is unchanged
    }
  } else {
    final_result = stepNominal(env, &final_state, input.pose, input.t, final_params);
    if (env.obstacles.valid && final_result.phase != Phase::kStopped) {
      // use_collision_detection: false keeps only kBlocked's current-pose test.
      const Pose2 pose_grid = env.obstacles.T_grid_eval.apply(input.pose);
      if (checker_.collides(pose_grid, *env.obstacles.grid, *env.obstacles.edt_all)) {
        blocked = true;
        final_state = state;
        final_params.zero_target = true;
        final_result = stepNominal(env, &final_state, input.pose, input.t, final_params);
      }
    }
  }

  // 5. Commit.
  final_state.blocked = blocked;
  if (input.cancel && final_state.limiter.cancel_hard) {
    extra_status |= Status::kCollisionAhead;
  }
  output.cmd = final_result.cmd;
  output.cmd_target = final_result.target;
  output.field = final_result.sample;
  output.phase = final_result.phase;
  output.status = final_result.status | extra_status | (blocked ? Status::kBlocked : 0U);
  output.settled = final_result.settled;
  output.s_free = s_free;
  output.cap = cap;
  output.shaping = final_result.shaping;
  output.box_scale = final_result.box_scale;
  output.limiter_alpha = final_result.alpha;
  output.predicted = Span<const Pose2>(scratch.rollout.data(), scratch.rollout.size());
  output.cancel_complete = input.cancel && belowStopThresholds(output.cmd);
  if (out != nullptr) {
    *out = final_state;
  }
  return output;
}

ProbeSample Tracker::probe(
  const Vec2 & x, const TrackerInput & input, const TrackerState & state,
  const StepScratch & scratch, double time_offset) const noexcept
{
  if (input.curve == nullptr || !isFinite(input.pose) || !std::isfinite(input.t)) {
    return ProbeSample{};
  }
  // The state step() wrote: its hint, terminal mode and episode; the scratch's K. The goal is
  // rebuilt on a copy (the terminal-mode hysteresis is idempotent for the same input). The
  // shaping is the external speed limit only: the cycle's collision cap is not stored.
  TrackerState frozen = state;
  const Model model = makeModel(config_, kinematics_, gain_, law_, checker_);
  const GoalEnv goal = makeGoalEnv(model, input, &frozen);
  StepEnv env = makeStepEnv(model, input, goal);
  bindObstacleMap(input, &scratch.edt_k, &env.obstacles);
  env.obstacles.k_valid = env.obstacles.valid && scratch.k_valid && scratch.edt_k.hasSites();
  env.obstacles.gate_open = state.contour.gate_open;
  env.obstacles.use_blocking =
    env.obstacles.valid && config_.obstacles.obstacle_mode == ObstacleMode::kContour &&
    config_.obstacles.obstacle_activation == ObstacleActivation::kPathBlocked;
  env.obstacles.blocking = &scratch.blocking;

  const Isometry2 frame = curveFrameAt(input, input.t);
  const BaseGeometry base = evaluateBase(env, state, frame, input.pose, input.t);
  const EvaluationPoint point =
    law_.evaluationPoint(input.pose, trackingPhase(state.phase, base.final_approach_entry));
  SpeedShaping shaping;
  shaping.gamma_lim = env.gamma_lim;
  return evaluateProbe(env, state, input.pose, x, input.t + time_offset, point.x_ref, shaping);
}

TrackerState Tracker::initialState() noexcept { return TrackerState{}; }

const CoreConfig & Tracker::config() const noexcept { return config_; }

const ValidationReport & Tracker::validation() const noexcept { return validation_; }

const Kinematics & Tracker::kinematics() const noexcept { return kinematics_; }

const GainFunction & Tracker::gain() const noexcept { return gain_; }

const Footprint & Tracker::footprint() const noexcept { return checker_.footprint(); }

const ControlLaw & Tracker::law() const noexcept { return law_; }

}  // namespace vectorfield_core
