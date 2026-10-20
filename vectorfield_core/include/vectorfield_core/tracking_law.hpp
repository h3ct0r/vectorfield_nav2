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
/// @brief Tracking laws: holonomic, feedback linearisation, heading tracking (with the
/// \f$\dot\psi_d\f$ probes), Ackermann rules and the rotate law (plan §3.2; math_spec §7).
///
/// Implemented in src/tracking_law.cpp (M1 wave 2, field/laws/limiter). The public API is
/// binding; private members are the implementer's choice. The enum `TrackingLaw` (config.hpp)
/// names the law; the class that runs it is ControlLaw.

#ifndef VECTORFIELD_CORE__TRACKING_LAW_HPP_
#define VECTORFIELD_CORE__TRACKING_LAW_HPP_

#include <cstdint>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief What a field probe returns at a point (plan §3.2 TrackingLaw; math_spec §7.4).
///
/// Computed by the Tracker with this cycle's frozen hint, K and gate, so a probe sees exactly the
/// field the controller uses (also for the `field_grid` visualisation).
struct ProbeSample
{
  /// Field the law would track at the point: post-contour, after this cycle's speed shaping.
  Vec2 F{Vec2::Zero()};
  /// \f$\psi_d = \angle F\f$; NaN when \f$\lVert F\rVert\f$ < `LawInput::hold_speed` (a NaN zeroes
  /// \f$\dot\psi_d\f$, Appendix C item 5).
  double psi_d{kNaN};
  /// Arc-length station of the closest point. Only differences between probes of one cycle are
  /// meaningful (closed curves: wrap the difference).
  double s_star{kNaN};
  /// Distance to the curve.
  double D{kNaN};
  /// A contour episode would be active at the point.
  bool contour_active{false};
  /// Its circulation side (0 without an episode).
  std::int8_t sigma_c{0};
};

/// @brief Non-owning field probe: `probe(x, time_offset)` evaluates the field at point x
/// (evaluation frame) at time \f$t + \f$ time_offset.
///
/// The spatial probes of \f$\dot\psi_d\f$ use time_offset 0; \f$\partial_t\psi_d\f$ of a
/// time-varying field uses \f$\pm\Delta t/2\f$ (math_spec §7.4). A FunctionRef: never allocates;
/// the referenced callable must outlive the call that uses it.
using FieldProbe = FunctionRef<ProbeSample(const Vec2 & x, double time_offset)>;

/// @brief The law's evaluation point and reference offset (math_spec §7.1, sec-laws-points).
struct EvaluationPoint
{
  /// Evaluation point \f$x\f$ (evaluation frame).
  Vec2 x{Vec2::Zero()};
  /// Offset \f$x_{\mathrm{ref}}\f$ of the point from base_link along the drive direction
  /// \f$q\,\mathbf h\f$ (signed; FL: \f$d\f$, with \f$x = p + q\,d\,\mathbf h\f$; Ackermann:
  /// \f$q\,x_r\f$; otherwise 0). The terminal-field shift uses it (math_spec §5.2).
  double x_ref{0.0};
  /// The law that runs in this phase (FL becomes heading tracking in FINAL_APPROACH).
  TrackingLaw law{TrackingLaw::kFeedbackLinearization};
};

/// @brief Evaluation point per (law, phase), math_spec §7.1: FL in FOLLOW and ROTATE_TO_FIELD
/// \f$p_c = p + q\,d\,\mathbf h(\psi)\f$; FL in FINAL_APPROACH, heading tracking (diff/skid) and
/// holonomic: \f$p\f$; Ackermann: \f$p_r = p + x_r\mathbf h(\psi)\f$ in every phase. In
/// ROTATE_TO_HEADING and STOPPED the point of FOLLOW is used.
/// @param base base_link pose.
/// @param phase Current phase.
/// @param law Resolved law (never kAuto).
/// @param kinematics Offsets and drive sign.
EvaluationPoint evaluationPoint(
  const Pose2 & base, Phase phase, TrackingLaw law, const Kinematics & kinematics) noexcept;

/// @brief \f$M(\psi) = \begin{bmatrix}\mathbf h^\top \\ q(J\mathbf h)^\top/d\end{bmatrix}\f$:
/// \f$(v, \omega) = M F\f$ (math_spec §7.3; thesis eq. 2.1).
Mat2 flMatrix(double psi, double d, double q) noexcept;
/// @brief \f$M^{-1} = [\mathbf h,\ q\,d\,J\mathbf h]\f$: control-point velocity of
/// \f$(v, \omega)\f$ (oracle O39).
Mat2 flInverse(double psi, double d, double q) noexcept;

/// @brief Per-cycle inputs of a translation law.
struct LawInput
{
  /// base_link pose (evaluation frame).
  Pose2 pose;
  /// The law's evaluation point (evaluationPoint()).
  Vec2 x{Vec2::Zero()};
  /// Field to track at x: FieldSample::F (post-contour, shaped).
  Vec2 F{Vec2::Zero()};
  /// Field tangent \f$T_d\f$ at the closest point (holonomic `path_tangent`).
  Vec2 T_d{Vec2::UnitX()};
  /// \f$\Delta t\f$ (s), the measured time since the previous step.
  double dt{0.05};
  /// \f$\bar\omega = \omega_{\max}\gamma_{\mathrm{lim}}\f$ (rad/s).
  double omega_bar{1.5};
  /// Field norm below which \f$\psi_d\f$ is held: \f$0.05\,v_r\f$ (math_spec §7.2, §7.4).
  double hold_speed{0.02};
  /// `TrackerState::last_psi_d` (NaN: none).
  double last_psi_d{kNaN};
  /// Closed curves: \f$L(t)\f$ for the wrapped jump test; open curves: \f$+\infty\f$.
  double curve_length{kInf};
  /// Mean vertex spacing \f$\bar\ell\f$ of the curve (Curve::vertexSpacing(); 0 for parametric
  /// curves): heading tracking's probe offset is \f$\delta = \max(0.02, \lvert v\rvert\Delta t,
  /// \min(\bar\ell, 0.10))\f$ (math_spec §7.4).
  double vertex_spacing{0.0};
  /// The field is time-varying: \f$\partial_t\psi_d\f$ is probed (math_spec §7.4).
  bool time_varying{false};
  /// A contour episode is active (math_spec §6.6 [Design, M7]). Ackermann only: the speed factor
  /// \f$\cos e\f$ is floored at kContourTurnSpeedFactor and \f$\lvert e\rvert > \pi/2\f$ does not
  /// set kHeadingInfeasible, so the car keeps turning at \f$R_{\min}\f$ toward the field.
  bool contour_episode{false};
};

/// @brief Floor of heading tracking's speed factor \f$\cos e\f$ for Ackermann during a contour
/// episode (math_spec §6.6 [Design, M7]).
inline constexpr double kContourTurnSpeedFactor = 0.5;

/// @brief Output of a translation law, before the kinematic box.
struct LawOutput
{
  /// Body twist at base_link (Ackermann: \f$(v, 0, \omega)\f$).
  Twist2 cmd;
  /// Desired heading used this cycle (heading tracking: \f$\psi_d\f$; holonomic `field` policy:
  /// its target); store it as `TrackerState::last_psi_d`. NaN when not applicable.
  double psi_d{kNaN};
  /// Heading error of the law: heading tracking \f$e = \operatorname{wrap}(\psi_d -
  /// \psi_{\mathrm{dr}})\f$; FL \f$e_c\f$ from \f$q\mathbf h\f$ to F; holonomic
  /// \f$\operatorname{wrap}(\psi_{\mathrm{tgt}} - \psi)\f$.
  double heading_error{0.0};
  /// Heading tracking: \f$\dot\psi_d\f$ after jump zeroing, before the \f$\pm\omega_c\f$ clamp.
  double psi_d_dot{0.0};
  /// \f$\psi_d\f$ was held (\f$\lVert F\rVert\f$ < hold_speed).
  bool held{false};
  /// Jump-only zeroing fired (math_spec §7.4).
  bool jump{false};
  /// Status bits: Status::kHeadingInfeasible when Ackermann sees \f$\lvert e\rvert > \pi/2\f$ at
  /// \f$p_r\f$ outside a contour episode (the Tracker adds the \f$\sigma\f$ and yaw clauses of
  /// math_spec §7.5).
  std::uint32_t status{Status::kNone};
};

/// @brief Holonomic law (math_spec §7.2): \f$(v_x, v_y) = R(\psi)^\top F\f$;
/// \f$\omega = \operatorname{clamp}(k_\psi\operatorname{wrap}(\psi_{\mathrm{tgt}} - \psi),
/// \pm\bar\omega)\f$ with \f$\psi_{\mathrm{tgt}} = \angle T_d\f$ (path_tangent) or \f$\angle F\f$
/// (field; held at `last_psi_d` while \f$\lVert F\rVert\f$ < hold_speed); `fixed` gives
/// \f$\omega = 0\f$.
LawOutput holonomicLaw(const LawInput & input, OmniHeadingPolicy policy, double k_psi) noexcept;

/// @brief Feedback linearisation (math_spec §7.3): \f$(v, \omega) = M(\psi)F\f$ with offset d and
/// drive sign q (d > 0).
LawOutput feedbackLinearizationLaw(const LawInput & input, double d, double q) noexcept;

/// @brief Heading tracking (math_spec §7.4, sec-laws-ht), also the Ackermann law (§7.5).
///
/// \f$\psi_{\mathrm{dr}} = \psi + \pi[q = -1]\f$, \f$\psi_d = \angle F\f$ (held at `last_psi_d`
/// while \f$\lVert F\rVert\f$ < hold_speed, then \f$\dot\psi_d = 0\f$), \f$e =
/// \operatorname{wrap}(\psi_d - \psi_{\mathrm{dr}})\f$, \f$v = q\lVert F\rVert\max(0, \cos e)\f$,
/// \f$\omega = \operatorname{clamp}(\dot\psi_d, \pm\omega_c) + k_\psi e\f$ with \f$\omega_c =
/// \bar\omega\f$ (Ackermann: \f$\lvert v\rvert/R_{\min}\f$).
/// \f$\dot\psi_d = v\,\operatorname{wrap}(\psi_d(x + \delta\mathbf h) - \psi_d(x - \delta\mathbf
/// h))/(2\delta) + \partial_t\psi_d\f$, \f$\delta = \max(0.02, \lvert v\rvert\Delta t,
/// \min(\bar\ell, 0.10))\f$ with \f$\bar\ell\f$ = LawInput::vertex_spacing (Curve::vertexSpacing()),
/// \f$\partial_t\psi_d = \operatorname{wrap}(\psi_d(x, t + \Delta t/2) - \psi_d(x, t -
/// \Delta t/2))/\Delta t\f$ for time-varying fields. \f$\dot\psi_d := 0\f$ only on a probe NaN
/// or a jump: \f$\lvert s^\star_+ - s^\star_-\rvert > 2\delta + 2\max(D_+, D_-)\f$ (wrapped when
/// closed), or different `contour_active` / `sigma_c`.
/// @param input Cycle inputs; `input.x` is the probe centre.
/// @param probe The Tracker's field probe.
/// @param k_psi \f$k_\psi\f$.
/// @param q Drive sign (Ackermann: +1).
/// @param min_turning_radius Ackermann \f$R_{\min}\f$ (> 0); 0 for diff/skid.
LawOutput headingTrackingLaw(
  const LawInput & input, FieldProbe probe, double k_psi, double q,
  double min_turning_radius) noexcept;

/// @brief Parameters of the rotate law.
struct RotateLawParams
{
  /// \f$\omega_{\mathrm{floor}}\f$ (rad/s): `min_in_place_angular_vel` (ROTATE_TO_FIELD and,
  /// outside the dead band, ROTATE_TO_HEADING).
  double omega_floor{0.0};
  /// \f$\omega_{\mathrm{rot}} = \min(\omega_{\mathrm{rth}}, \bar\omega)\f$ (rad/s).
  double omega_rot{1.5};
  /// \f$\alpha_{\max}\f$ (rad/s²).
  double alpha_max{3.2};
  /// ROTATE_TO_HEADING: \f$\mathrm{yaw\_tol}/2\f$, inside which \f$\omega^\ast = 0\f$;
  /// ROTATE_TO_FIELD: 0.
  double dead_band{0.0};
};

/// @brief Rotate law (math_spec §7.6 [Spec]): \f$\omega^\ast = \operatorname{sgn}(e_r)
/// \max(\omega_{\mathrm{floor}}, \min(\omega_{\mathrm{rot}}, \sqrt{2\alpha_{\max}\lvert e_r\rvert}))\f$,
/// and 0 when \f$\lvert e_r\rvert \le\f$ dead_band.
double rotateLawOmega(double e_r, const RotateLawParams & params) noexcept;

/// @brief Allow-reversing backstop (math_spec §9.2): clamps the drive-direction speed to
/// \f$q\,v \ge 0\f$, keeping \f$\omega\f$.
Twist2 applyReversingBackstop(const Twist2 & cmd, double q) noexcept;

/// @brief Ackermann FOLLOW speed band (math_spec §7.5): a positive speed below
/// \f$v_{\mathrm{ack}}\f$ is raised to \f$v_{\mathrm{ack}}\f$; zero and negative speeds are kept.
double applyAckermannBand(double v, double v_ack) noexcept;

/// @brief Runs the configured law for a phase (math_spec §7).
///
/// Built from a resolved configuration; holds its own Kinematics, \f$k_\psi\f$ and heading
/// policy. track() runs the translation law of FOLLOW and FINAL_APPROACH (FL hands over to
/// heading tracking at base_link in FINAL_APPROACH, honouring drive_backward); rotate() runs the
/// rotate law. Speed shaping, the Ackermann band, the backstop and the kinematic box are the
/// Tracker's steps (math_spec §8.1).
class ControlLaw
{
public:
  /// @brief A law for the given configuration (tracking_law may be auto).
  /// @throws ConfigError on an invalid (motion_model, tracking_law) pair, or FL with d <= 0.
  explicit ControlLaw(const CoreConfig & config);

  /// @brief The resolved law of FOLLOW.
  TrackingLaw law() const noexcept;
  /// @brief The law that runs in a phase.
  TrackingLaw activeLaw(Phase phase) const noexcept;
  /// @brief evaluationPoint() for this law.
  EvaluationPoint evaluationPoint(const Pose2 & base, Phase phase) const noexcept;
  /// @brief The kinematics.
  const Kinematics & kinematics() const noexcept;

  /// @brief This law with the drive sign \f$q\f$ (−1: body \f$-x\f$) in place of the
  /// configured one: a reverse plan segment (math_spec §9.4 [Design, M7]); for any model.
  ControlLaw withDriveSign(double q) const noexcept;

  /// @brief Translation law of a phase (FOLLOW or FINAL_APPROACH).
  LawOutput track(Phase phase, const LawInput & input, FieldProbe probe) const noexcept;

  /// @brief Rotate law (math_spec §7.6) in ROTATE_TO_FIELD or ROTATE_TO_HEADING.
  /// @param phase The rotation phase.
  /// @param e_r Angle to go (rad).
  /// @param omega_bar \f$\bar\omega\f$.
  /// @param yaw_tol Goal yaw tolerance (ROTATE_TO_HEADING dead band yaw_tol/2).
  double rotate(Phase phase, double e_r, double omega_bar, double yaw_tol) const noexcept;

private:
  Kinematics kinematics_;
  TrackingLaw law_{TrackingLaw::kFeedbackLinearization};
  OmniHeadingPolicy heading_policy_{OmniHeadingPolicy::kPathTangent};
  double k_psi_{2.0};
  double rotate_to_heading_vel_{1.5};
  double min_in_place_vel_{0.0};
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__TRACKING_LAW_HPP_
