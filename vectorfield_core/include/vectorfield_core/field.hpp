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
/// @brief The path field: static, time-varying and terminal (math_spec §4–§5), speed shaping
/// (§8.2) and `FieldSample`, the per-cycle record of every field quantity (Appendix A).
///
/// Implemented in src/field.cpp (M1 wave 2, field/laws/limiter). The public API is binding;
/// private members are the implementer's choice.

#ifndef VECTORFIELD_CORE__FIELD_HPP_
#define VECTORFIELD_CORE__FIELD_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Obstacle-contour quantities of one cycle (math_spec §6.4; Appendix A "contour").
///
/// All points are in the evaluation frame. K's cells and the blocked gate samples are in
/// StepScratch (tracker.hpp).
struct ContourSample
{
  /// The `path_blocked` gate is open (always true with `obstacle_activation: always`).
  bool gate_open{false};
  /// A contour episode is active.
  bool active{false};
  /// Latched circulation side \f$\sigma_c \in \{-1, 0, +1\}\f$ (0 without an episode).
  std::int8_t sigma_c{0};
  /// The goal veto applies this cycle (the curve end is the goal and is blocked).
  bool goal_veto{false};
  /// Nearest point \f$x_o^\star\f$ of K (or of all sites without an episode), if any.
  Vec2 x_o{Vec2::Zero()};
  /// \f$D_o\f$ (m); \f$+\infty\f$ when there is no obstacle (FieldState.obstacle_distance).
  double D_o{kInf};
  /// Obstacle normal \f$n_o = (x - x_o^\star)/D_o\f$; zero when there is no obstacle.
  Vec2 n_o{Vec2::Zero()};
  /// Blend weight \f$\theta \in [0, 1]\f$ (1 = path field only; FieldState.blend).
  double theta{1.0};
  /// Episode base-link path length (m), watchdog.
  double travelled_m{0.0};
  /// Episode time (s), watchdog.
  double elapsed_s{0.0};
  /// First blocked gate sample \f$r(s)\f$, meaningful when `has_blocked_sample`.
  Vec2 first_blocked_sample{Vec2::Zero()};
  /// At least one gate sample was blocked this cycle.
  bool has_blocked_sample{false};
};

/// @brief Most obstacle sites the CBF filter constrains (`cbf_sites`, math_spec §6.7).
inline constexpr int kCbfMaxSites = 3;

/// @brief CBF-filter quantities of one evaluation (math_spec §6.7 [Design, M7]; Appendix A
/// "cbf"). Points and vectors are in the evaluation frame.
struct CbfSample
{
  /// The filter ran (`obstacle_mode: cbf`, a usable map, XY not reached).
  bool on{false};
  /// The filter changed the field: at least one constraint binds.
  bool active{false};
  /// CBF hold: active with \f$\lVert F^\star\rVert < 0.05\lVert F\rVert\f$ (kBlocked in a
  /// translation phase, math_spec §10.4).
  bool hold{false};
  /// The push-out requests conflicted; the QP was solved again without them.
  bool relaxed{false};
  /// Constraints used, in [0, kCbfMaxSites].
  int count{0};
  /// \f$\delta_{\mathrm{safe}} = r_e + m\f$ (m).
  double delta{0.0};
  /// \f$\min_j h_j\f$ (m); \f$+\infty\f$ without constraints.
  double h{kInf};
  /// \f$\lVert F^\star\rVert / \lVert F\rVert\f$ (1 when inactive or \f$F = 0\f$).
  double speed_ratio{1.0};
  /// The field before the filter (\f$\Phi_\gamma\f$).
  Vec2 F_in{Vec2::Zero()};
  /// Per constraint j < count: nearest point of the site, unit normal \f$n_j\f$ (away from
  /// the obstacle), \f$h_j\f$, and whether it binds.
  std::array<Vec2, kCbfMaxSites> x_o{Vec2::Zero(), Vec2::Zero(), Vec2::Zero()};
  std::array<Vec2, kCbfMaxSites> n{Vec2::Zero(), Vec2::Zero(), Vec2::Zero()};
  std::array<double, kCbfMaxSites> h_j{kInf, kInf, kInf};
  std::array<bool, kCbfMaxSites> binds{false, false, false};
  /// Ackermann: the turning margin of each constraint (m; 0 for the other motion models); the
  /// filter constrains \f$\max(0, h_j - \text{margin}_j)\f$ (math_spec §6.7 [Design, 0.3.0]).
  std::array<double, kCbfMaxSites> turn_margin{0.0, 0.0, 0.0};
};

/// @brief Every field quantity of one evaluation (math_spec Appendix A, sec-math-viz).
///
/// Filled step by step: DistanceField::evaluate() or evaluateTerminal() (closest point, gains,
/// nominal field), DistanceField::shape() (speed shaping), the contour (F, contour), and the
/// Tracker (goal, phase target, status). Vectors are in the evaluation frame. Fixed size and
/// trivially destructible, so TrackerOutput can carry it without allocation.
struct FieldSample
{
  /// @name Closest point (math_spec §2)
  /// @{
  /// Evaluation point \f$x\f$ of the active law (math_spec §7.1; the control_point marker).
  Vec2 x_eval{Vec2::Zero()};
  /// Closest point \f$x^\star\f$.
  Vec2 x_star{Vec2::Zero()};
  /// Unit tangent \f$T\f$ in the curve's parameter direction.
  Vec2 T{Vec2::UnitX()};
  /// Field tangent \f$T_d = \mathrm{dir}\,T\f$ (FieldState.tangent).
  Vec2 T_d{Vec2::UnitX()};
  /// Unit normal \f$n\f$ (zero on the curve).
  Vec2 n{Vec2::Zero()};
  /// Distance \f$D\f$ (m).
  double D{0.0};
  /// Arc-length station \f$s^\star\f$ (FieldState.arc_length).
  double s_star{0.0};
  /// Remaining arc length \f$\sigma_{\mathrm{base}}\f$ to the goal station at base_link against
  /// the unshifted \f$s_{\mathrm{goal}}\f$ (math_spec §5.2); NaN for closed curves, \f$+\infty\f$
  /// outside terminal mode (math_spec §5.5). FieldState.distance_to_end.
  double sigma{kNaN};
  /// \f$\sigma_{\mathrm{end}}\f$ to the window's last real vertex (window-end cap); NaN closed.
  double sigma_end{kNaN};
  /// @}

  /// @name Gains and scale (math_spec §3, §4.3)
  /// @{
  /// \f$G(D)\f$.
  double G{0.0};
  /// \f$H(D)\f$.
  double H{1.0};
  /// Scale actually used: \f$\gamma v_r\f$ for static and terminal fields, \f$\eta_1\f$ or
  /// \f$\eta^\ast\f$ for time-varying fields (Appendix A).
  double eta{0.0};
  /// Terminal speed profile \f$\varphi(\sigma)\f$; 1 outside the terminal field.
  double phi{1.0};
  /// Speed-shaping scale \f$\gamma\f$ (math_spec §8.2).
  double gamma{1.0};
  /// DistanceField::shape() took the uniform branch (static or terminal field, contour episode
  /// with \f$\theta < 1\f$, or the safety fallback): the kinematic box then uses the uniform
  /// \f$c\f$. False only for the time-varying \f$\eta_1\f$ branch, where the \f$\eta\f$-box
  /// applies under FL and holonomic (math_spec §8.3).
  bool uniform_shaping{true};
  /// @}

  /// @name Field vectors
  /// @{
  /// Unit field \f$\Phi_S = -G n + H T_d\f$; for the terminal field \f$-G n + H\varphi T_d\f$
  /// (norm <= 1).
  Vec2 Phi_S{Vec2::Zero()};
  /// Feedforward \f$\Phi_T = \Pi_T v_c\f$ after the Assumption-3 clamp (FieldState.feedforward).
  Vec2 Phi_T{Vec2::Zero()};
  /// Curve-point velocity \f$v_c = \partial r/\partial t\f$ at the foot (math_spec §4.2).
  Vec2 v_c{Vec2::Zero()};
  /// Nominal (unshaped) path field \f$\Phi = \eta\Phi_S + \Phi_T\f$ (static: \f$v_r\Phi_S\f$).
  /// Contour start and exit tests use it (Appendix C item 8).
  Vec2 Phi{Vec2::Zero()};
  /// Shaped path field \f$\Phi_\gamma\f$ (math_spec §8.2).
  Vec2 Phi_gamma{Vec2::Zero()};
  /// Field the law tracks: post-contour, before the kinematic box (FieldState.field).
  Vec2 F{Vec2::Zero()};
  /// Convergence arrow \f$-\eta G n\f$ (shaped).
  Vec2 conv{Vec2::Zero()};
  /// Circulation arrow \f$\eta H T_d\f$ (terminal: \f$\gamma v_r H\varphi T_d\f$). `conv`,
  /// `circ` and `Phi_T` sum to `Phi_gamma`.
  Vec2 circ{Vec2::Zero()};
  /// @}

  /// @name Goal (math_spec §5.1–§5.2)
  /// @{
  /// A goal is known (open curves; plugin terminal mode).
  bool has_goal{false};
  /// The terminal field is in use this cycle.
  bool terminal{false};
  /// Goal \f$g\f$ given to the core.
  Vec2 g{Vec2::Zero()};
  /// Shifted goal \f$g_{\mathrm{ref}} = r(s_{\mathrm{goal}} + \mathrm{dir}\,x_{\mathrm{ref}})\f$.
  Vec2 g_ref{Vec2::Zero()};
  /// Goal station \f$s_{\mathrm{goal}}\f$.
  double s_goal{kNaN};
  /// Reference-point offset \f$x_{\mathrm{ref}}\f$ of the active law (math_spec §1).
  double x_ref{0.0};
  /// @}

  /// @name Obstacle contour (math_spec §6)
  /// @{
  /// Contour quantities.
  ContourSample contour;
  /// CBF filter (`obstacle_mode: cbf`).
  CbfSample cbf;
  /// @}

  /// @name Heading (math_spec §7; Appendix A "heading target")
  /// @{
  /// Desired heading \f$\psi_d = \angle F\f$ used by heading tracking (held value when held).
  double psi_d{kNaN};
  /// Feedforward \f$\dot\psi_d\f$ used by heading tracking.
  double psi_d_dot{0.0};
  /// Heading error \f$e\f$ of the active law (rad).
  double heading_error{0.0};
  /// Phase target heading: ROTATE_TO_FIELD \f$\angle F_{\mathrm{trk}}\f$ (+π with
  /// drive_backward), ROTATE_TO_HEADING the goal yaw, heading tracking \f$\psi_d\f$.
  double target_heading{kNaN};
  /// `target_heading` is meaningful.
  bool has_target_heading{false};
  /// @}

  /// Status bits raised while evaluating (kAssumption3Clamped, kHairpin, kAmbiguousClosest,
  /// kNonFinite).
  std::uint32_t status{Status::kNone};
  /// The closest point was valid.
  bool valid{false};
};

/// @brief Terminal speed profile \f$\varphi\f$ (math_spec §5.1, sec-terminal-def):
/// \f$\varphi(\sigma) = \operatorname{sgn}(\sigma)\min\big(1, \max\big(\lvert\sigma\rvert/\sigma_b,
/// \frac{v_{\min}}{v_r}\min(1, \lvert\sigma\rvert/\sigma_{\mathrm{stop}})\big)\big)\f$.
///
/// Odd, \f$\lvert\varphi\rvert \le 1\f$, Lipschitz with \f$L_\varphi = \max(1/\sigma_b,
/// v_{\min}/(v_r\sigma_{\mathrm{stop}}))\f$ (oracle O26). With \f$\sigma_{\mathrm{stop}} \le 0\f$
/// the floor term is \f$v_{\min}/v_r\f$ for every \f$\sigma \ne 0\f$.
class TerminalProfile
{
public:
  /// @brief Profile for the given speeds and distances.
  /// @param v_r Reference speed \f$v_r\f$ (> 0).
  /// @param v_min `min_approach_speed` \f$v_{\min}\f$.
  /// @param sigma_b `terminal_braking_distance` \f$\sigma_b\f$ (> 0).
  /// @param sigma_stop \f$\sigma_{\mathrm{stop}} = \mathrm{xy\_tol}/2\f$.
  TerminalProfile(double v_r, double v_min, double sigma_b, double sigma_stop) noexcept;

  /// @brief \f$\varphi(\sigma)\f$.
  double operator()(double sigma) const noexcept;
  /// @brief \f$L_\varphi\f$.
  double lipschitz() const noexcept;
  /// @brief \f$\sigma_{\mathrm{stop}}\f$.
  double sigmaStop() const noexcept { return sigma_stop_; }
  /// @brief \f$\sigma_b\f$.
  double sigmaBraking() const noexcept { return sigma_b_; }

private:
  double v_r_;
  double v_min_;
  double sigma_b_;
  double sigma_stop_;
};

/// @brief Feedforward \f$\Phi_T = \Pi_T v_c = v_c - (T^\top v_c)T\f$ (math_spec §4.2 [Derived]).
Vec2 feedforward(const Vec2 & T, const Vec2 & v_c) noexcept;

/// @brief Result of clampFeedforward().
struct ClampedFeedforward
{
  /// \f$\Phi_T\f$ after the clamp.
  Vec2 Phi_T{Vec2::Zero()};
  /// The clamp acted (Status::kAssumption3Clamped).
  bool clamped{false};
};

/// @brief Assumption-3 clamp (math_spec §4.2 [Design]): if \f$\lVert\Phi_T\rVert >
/// (1 - \varepsilon_v)v_r\f$, scale \f$\Phi_T\f$ to that norm (oracle O19).
ClampedFeedforward clampFeedforward(
  const Vec2 & Phi_T, double v_r, double assumption3_margin) noexcept;

/// @brief \f$\eta\f$, the larger root of \f$\eta^2 + 2b\eta + \lVert\Phi_T\rVert^2 - v^2 = 0\f$
/// with \f$b = \Phi_S^\top\Phi_T\f$ [Thesis 3.28–3.29] (math_spec §4.3, sec-eta), in the
/// cancellation-free form for \f$b > 0\f$.
/// @param Phi_S Unit field.
/// @param Phi_T Feedforward with \f$\lVert\Phi_T\rVert < v\f$.
/// @param speed Target speed \f$v\f$ (\f$v_r\f$, or \f$\gamma v_r\f$ when shaping).
/// @return \f$\eta > 0\f$; NaN when \f$c_\eta = v^2 - \lVert\Phi_T\rVert^2 \le 0\f$.
double solveEta(const Vec2 & Phi_S, const Vec2 & Phi_T, double speed) noexcept;

/// @brief The path field of math_spec §4 and §5 for one curve configuration.
///
/// A cheap value type holding the gain, \f$v_r\f$, dir and \f$\varepsilon_v\f$. All methods are
/// `noexcept` and allocation-free; non-finite inputs produce `valid == false` or
/// Status::kNonFinite in the sample, never an exception.
class DistanceField
{
public:
  /// @brief A field with the given gain, reference speed, direction and Assumption-3 margin.
  DistanceField(
    const GainFunction & gain, double v_r, int direction, double assumption3_margin) noexcept;

  /// @brief Static or time-varying field at a closest point (math_spec §4.1–§4.3).
  ///
  /// Fills x_eval, x_star, T, T_d, n, D, s_star, G, H, v_c, Phi_S, Phi_T (after
  /// clampFeedforward()), eta (\f$v_r\f$ when \f$\Phi_T = 0\f$, else solveEta() at \f$v_r\f$),
  /// Phi, and resets the shaping outputs to the unshaped values (gamma 1, Phi_gamma = Phi,
  /// F = Phi, conv, circ). Sets Status::kAssumption3Clamped when the clamp acts and ORs
  /// `cp.status_bits` into `sample->status`. Other fields are left untouched.
  /// @param x_eval Evaluation point (evaluation frame).
  /// @param cp Closest point in the evaluation frame (transformed()).
  /// @param v_c Curve-point velocity at the foot (evaluation frame); zero for static curves.
  /// @param sample Output.
  void evaluate(
    const Vec2 & x_eval, const ClosestPoint & cp, const Vec2 & v_c,
    FieldSample * sample) const noexcept;

  /// @brief Terminal field on a static open curve (math_spec §5.1):
  /// \f$\Phi = v_r(-G n + H\varphi(\sigma)T_d)\f$.
  ///
  /// Fills the same fields as evaluate() with \f$\Phi_T = 0\f$, \f$\eta = v_r\f$,
  /// \f$\Phi_S = -Gn + H\varphi T_d\f$, plus `phi` and `terminal = true`.
  /// @param x_eval Evaluation point (evaluation frame).
  /// @param cp Closest point in the evaluation frame.
  /// @param sigma \f$\sigma = \mathrm{dir}(s_{\mathrm{goal,ref}} - s^\star)\f$ at this point
  /// (math_spec §5.2: the shifted goal station for a shifted law).
  /// @param profile \f$\varphi\f$.
  /// @param sample Output.
  void evaluateTerminal(
    const Vec2 & x_eval, const ClosestPoint & cp, double sigma, const TerminalProfile & profile,
    FieldSample * sample) const noexcept;

  /// @brief Speed shaping (math_spec §8.2, sec-limiter-shaping) of an evaluated sample.
  ///
  /// - uniform (static or terminal field, `force_uniform`, or the safety fallback):
  ///   \f$\Phi_\gamma = \gamma\Phi\f$, eta = \f$\gamma\,\eta\f$, and a time-varying
  ///   \f$\Phi_T\f$ is scaled by \f$\gamma\f$ too ("scale the whole field");
  /// - time-varying otherwise: \f$\eta_1\f$ = solveEta() at \f$\gamma v_r\f$, \f$\Phi_\gamma =
  ///   \eta_1\Phi_S + \Phi_T\f$, valid when \f$\lVert\Phi_T\rVert < (1 - \varepsilon_v)\gamma
  ///   v_r\f$, else the uniform fallback;
  /// - Status::kAssumption3Clamped iff \f$\lVert\Phi_T\rVert \ge (1 - \varepsilon_v)v_r\min(1,
  ///   \gamma_{\mathrm{lim}})\f$.
  ///
  /// Sets gamma, Phi_gamma, eta, conv, circ, Phi_T (the feedforward actually contained in
  /// Phi_gamma), F = Phi_gamma (the contour may replace F) and `uniform_shaping`. The η-box of
  /// the kinematic box applies only when the non-uniform branch was taken.
  /// @param gamma \f$\gamma\f$ (SpeedShaping::gamma()).
  /// @param gamma_lim \f$\gamma_{\mathrm{lim}}\f$ (the external speed limit term).
  /// @param force_uniform True during a contour episode with \f$\theta < 1\f$.
  /// @param sample In/out.
  void shape(
    double gamma, double gamma_lim, bool force_uniform, FieldSample * sample) const noexcept;

  /// @brief The gain.
  const GainFunction & gain() const noexcept { return gain_; }
  /// @brief \f$v_r\f$.
  double referenceSpeed() const noexcept { return v_r_; }
  /// @brief dir.
  int direction() const noexcept { return direction_; }
  /// @brief \f$\varepsilon_v\f$.
  double assumption3Margin() const noexcept { return eps_v_; }

private:
  GainFunction gain_;
  double v_r_;
  int direction_;
  double eps_v_;
};

/// @brief RK4 streamline of a field for the `field_grid` streamlines (math_spec Appendix A).
///
/// Integrates \f$dx/d\ell = F(x)/\lVert F(x)\rVert\f$ with arc-length step `step` from `start`,
/// writing `start` and then each new point into `out`, and stops at `out.size()` points or when
/// \f$\lVert F\rVert\f$ is zero or non-finite at a stage. Allocation-free.
/// @param field \f$F(x)\f$, e.g. the Tracker's probe at the frozen t (evaluation frame).
/// @param start First point.
/// @param step Arc-length step (m), `field_grid_resolution`/2.
/// @param out Output points.
/// @return The number of points written.
std::size_t streamline(
  FunctionRef<Vec2(const Vec2 &)> field, const Vec2 & start, double step, Span<Vec2> out) noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__FIELD_HPP_
