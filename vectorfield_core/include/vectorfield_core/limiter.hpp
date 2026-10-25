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
/// @brief Speed shaping terms, the kinematic box (uniform \f$c\f$ and the \f$\eta\f$-box) and the
/// CommandLimiter (math_spec §8).
///
/// Implemented in src/limiter.cpp.

#ifndef VECTORFIELD_CORE__LIMITER_HPP_
#define VECTORFIELD_CORE__LIMITER_HPP_

#include <cstdint>
#include <type_traits>

#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief The term that binds \f$\gamma\f$ (math_spec §8.2).
enum class ShapingTerm : std::uint8_t
{
  /// \f$\gamma = 1\f$.
  kNone = 0,
  /// External speed limit \f$\gamma_{\mathrm{lim}}\f$.
  kSpeedLimit = 1,
  /// Window-end approach cap \f$\gamma_{\mathrm{ap}}\f$.
  kApproach = 2,
  /// Clearance slowdown \f$\gamma_{\mathrm{cl}}\f$.
  kClearance = 3,
  /// Collision cap \f$\gamma_{\mathrm{col}}\f$.
  kCollision = 4,
};

/// @brief Speed-shaping terms \f$\gamma = \min(1, \gamma_{\mathrm{lim}}, \gamma_{\mathrm{cl}},
/// \gamma_{\mathrm{ap}}, \gamma_{\mathrm{col}})\f$ (math_spec §8.2, sec-limiter-shaping).
struct SpeedShaping
{
  /// \f$\gamma_{\mathrm{lim}}\f$: `speed_limit_scale` (Ackermann: \f$\max(\gamma_{\mathrm{lim}},
  /// v_{\mathrm{ack}}/v_r)\f$).
  double gamma_lim{1.0};
  /// \f$\gamma_{\mathrm{cl}}\f$: clearanceScale().
  double gamma_cl{1.0};
  /// \f$\gamma_{\mathrm{ap}}\f$: approachScale(); 1 when `cap_at_curve_end` is false.
  double gamma_ap{1.0};
  /// \f$\gamma_{\mathrm{col}}\f$: collisionScale(); 1 inside rollouts.
  double gamma_col{1.0};

  /// @brief \f$\gamma\f$.
  double gamma() const noexcept;
  /// @brief The binding term: the first of kSpeedLimit, kApproach, kClearance, kCollision whose
  /// value equals gamma() (kNone when gamma() == 1). Ties therefore never blame the collision
  /// cap, which binds only when strictly smallest.
  ShapingTerm binding() const noexcept;
  /// @brief "The cap binds" (math_spec §9.1 [Spec]): \f$\gamma_{\mathrm{col}} < \min(1,
  /// \gamma_{\mathrm{lim}}, \gamma_{\mathrm{cl}}, \gamma_{\mathrm{ap}})\f$.
  bool capBinds() const noexcept;
  /// @brief STATUS_COLLISION_AHEAD condition (math_spec §10.3 [Spec]): the binding term is
  /// \f$\gamma_{\mathrm{col}}\f$ or \f$\gamma_{\mathrm{cl}}\f$.
  bool collisionAhead() const noexcept;
};

/// @brief \f$\gamma_{\mathrm{cl}}\f$ (math_spec §8.2 [Spec]): 1 if \f$d_{\mathrm{cl}} = 0\f$, else
/// \f$\min(1, k_{\mathrm{cl}}\max(0, \mathrm{EDT}_{\mathrm{all}}(p) - w_{\mathrm{lat}})/d_{\mathrm{cl}})\f$.
double clearanceScale(
  double edt_at_base, double w_lat, double slowdown_distance, double slowdown_gain) noexcept;

/// @brief \f$\gamma_{\mathrm{ap}} = \min(1, \sqrt{v_{\min}^2 + 2a_{\mathrm{dec}}\sigma_{\mathrm{end}}}
/// /v_r)\f$ (math_spec §5.5); 1 for a NaN \f$\sigma_{\mathrm{end}}\f$ (closed curve).
double approachScale(double sigma_end, double v_min, double a_dec, double v_r) noexcept;

/// @brief \f$\gamma_{\mathrm{col}} = \min(1, v_{\mathrm{cap}}/v_r)\f$ (math_spec §8.2, §10.3).
double collisionScale(double v_cap, double v_r) noexcept;

/// @brief Closed interval \f$[lo, hi]\f$ of \f$\eta\f$ values; empty when lo > hi.
struct Interval
{
  /// Lower bound.
  double lo{-kInf};
  /// Upper bound.
  double hi{kInf};
  /// @brief True iff empty.
  bool empty() const noexcept { return !(lo <= hi); }
  /// @brief Intersection.
  Interval intersect(const Interval & other) const noexcept
  {
    return Interval{lo > other.lo ? lo : other.lo, hi < other.hi ? hi : other.hi};
  }
};

/// @brief \f$\{\eta : \lvert a + \eta b\rvert \le L\}\f$ (math_spec §8.3): for \f$b \ne 0\f$
/// \f$[\min(e_1, e_2), \max(e_1, e_2)]\f$, \f$e_{1,2} = (\pm L - a)/b\f$; for \f$b = 0\f$ all of
/// \f$\mathbb R\f$ when \f$\lvert a\rvert \le L\f$, else empty.
Interval boxAxisInterval(double a, double b, double limit) noexcept;

/// @brief \f$\{\eta : \lVert a + \eta b\rVert \le L\}\f$ (math_spec §8.3, omni): with
/// \f$B = \lVert b\rVert^2\f$, \f$A = a^\top b\f$, \f$C = \lVert a\rVert^2 - L^2\f$,
/// \f$\Delta = A^2 - BC\f$: empty if \f$\Delta < 0\f$, else \f$[(-A \mp \sqrt\Delta)/B]\f$;
/// \f$B = 0\f$: all of \f$\mathbb R\f$ iff \f$C \le 0\f$.
Interval boxNormInterval(const Vec2 & a, const Vec2 & b, double limit) noexcept;

/// @brief Result of the kinematic box (math_spec §8.3, sec-limiter-box).
struct BoxResult
{
  /// Command after the box.
  Twist2 cmd;
  /// Uniform scale \f$c \in [0, 1]\f$ (1 when the \f$\eta\f$-box was used).
  double c{1.0};
  /// \f$\eta^\ast\f$ when the \f$\eta\f$-box was used, else NaN.
  double eta{kNaN};
  /// The \f$\eta\f$-box produced the command.
  bool eta_box{false};
  /// The \f$\eta\f$-box set was empty and the uniform fallback was used (sets
  /// Status::kAssumption3Clamped in `status`).
  bool fallback{false};
  /// Status bits (kAssumption3Clamped).
  std::uint32_t status{Status::kNone};
};

/// @brief Uniform kinematic box (static fields, heading tracking, contour with \f$\theta < 1\f$,
/// or the shaping fallback): diff, skid, Ackermann \f$c = \min(1, v_{\max}/\lvert v\rvert,
/// \bar\omega/\lvert\omega\rvert)\f$ applied to \f$(v, \omega)\f$; omni \f$c = \min(1,
/// v_{\max}/\lVert v_{xy}\rVert, v_{y,\max}/\lvert v_y\rvert)\f$ applied to the translation only,
/// with \f$\omega\f$ clamped to \f$\pm\bar\omega\f$ on its own. Terms with a zero denominator are
/// dropped. No Assumption-3 flag.
/// @param cmd Law command.
/// @param kinematics Limits and model.
/// @param omega_bar \f$\bar\omega\f$.
BoxResult uniformBox(const Twist2 & cmd, const Kinematics & kinematics, double omega_bar) noexcept;

/// @brief \f$\eta\f$-box (time-varying field under FL or holonomic, no contour with
/// \f$\theta < 1\f$; math_spec §8.3).
///
/// With \f$u(\eta) = a + \eta b\f$ (FL: \f$a = M\Phi_T\f$, \f$b = M\Phi_S\f$, \f$u = (v, \omega)\f$,
/// limits \f$(v_{\max}, \bar\omega)\f$; holonomic: \f$M = R(\psi)^\top\f$, \f$u = (v_x, v_y)\f$,
/// limits \f$(v_{\max}, v_{y,\max})\f$ plus \f$\lVert u\rVert \le v_{\max}\f$), intersect the
/// intervals with \f$[0, \eta_1]\f$ and take the maximum \f$\eta^\ast\f$; the command is
/// \f$a + \eta^\ast b\f$ (holonomic \f$\omega\f$ clamped on its own). If the set is empty, fall back
/// to uniformBox() on `fallback_cmd` and set Status::kAssumption3Clamped. A negative \f$\eta\f$ is
/// never commanded (oracle O44).
/// @param a \f$M\Phi_T\f$.
/// @param b \f$M\Phi_S\f$.
/// @param eta1 \f$\eta_1\f$ of speed shaping.
/// @param holonomic_omega Holonomic \f$\omega\f$ (ignored for FL).
/// @param fallback_cmd The step-3 law command for the uniform fallback.
/// @param kinematics Limits and model (FL or holonomic).
/// @param omega_bar \f$\bar\omega\f$.
BoxResult etaBox(
  const Vec2 & a, const Vec2 & b, double eta1, double holonomic_omega, const Twist2 & fallback_cmd,
  const Kinematics & kinematics, double omega_bar) noexcept;

/// @brief Cross-cycle state of the CommandLimiter (lives in TrackerState).
struct LimiterState
{
  /// Previous output \f$u_{\mathrm{prev}}\f$.
  Twist2 u_prev;
  /// A rollout found a collision during this cancel: the linear bound is
  /// \f$\max(a_{\mathrm{can}}, a_{\mathrm{dec}})\f$ for the rest of the cancel (math_spec §8.4).
  bool cancel_hard{false};
  /// False requests a re-seed: the Tracker sets \f$u_{\mathrm{prev}}\f$ to the measured twist at
  /// the next step (initial state, after a throw, on reset or cancel completion, on a parameter
  /// rebuild; the Tracker also re-seeds when \f$\Delta t > 2/f\f$). math_spec §8.4.
  bool valid{false};
};

/// @brief Acceleration stage, Ackermann curvature clamp and non-finite guard (math_spec §8.4,
/// sec-limiter-accel).
///
/// Stateless: the state is passed in and out, so `limit` is `const noexcept` and thread-safe.
class CommandLimiter
{
public:
  /// @brief Limits a target twist.
  ///
  /// 1. Acceleration: \f$u = u_{\mathrm{prev}} + \alpha(u^\ast - u_{\mathrm{prev}})\f$ with the
  ///    largest \f$\alpha \in [0, 1]\f$ meeting every axis bound \f$A_i\Delta t\f$: linear axes
  ///    (x and omni y) \f$a_{\mathrm{dec}}\f$ when \f$u_{\mathrm{prev},i}\Delta_i < 0\f$, else
  ///    \f$a_{\mathrm{acc}}\f$ (Appendix C item 11); \f$\alpha_{\max}\f$ for \f$\omega\f$. All axes
  ///    move along one straight line (oracle O48). For a cancel (`stopped_cancel`) the linear
  ///    bound is \f$a_{\mathrm{can}}\f$, or \f$\max(a_{\mathrm{can}}, a_{\mathrm{dec}})\f$ when
  ///    `in.cancel_hard`. With `brake_translation` the translational axes (x, omni y) and
  ///    \f$\omega\f$ each take their own \f$\alpha\f$, so the translation brakes at its own bound
  ///    while \f$\omega\f$ ramps (math_spec §8.4 [Design, 0.1.1]).
  /// 2. Ackermann curvature clamp: \f$\lvert\omega\rvert \le \lvert v\rvert/R_{\min}\f$, else
  ///    Status::kCurvatureSaturated.
  /// 3. Non-finite guard: any non-finite component gives \f$u = 0\f$ and Status::kNonFinite.
  ///
  /// Re-seeding is the caller's job (LimiterState::valid). `*out` may alias `in`.
  /// @param target Target \f$u^\ast\f$ after the kinematic box.
  /// @param in Previous state.
  /// @param out Next state: u_prev = the result; cancel_hard copied; valid = true.
  /// @param dt \f$\Delta t\f$ (s); a non-positive dt keeps \f$u_{\mathrm{prev}}\f$.
  /// @param kinematics Limits and model.
  /// @param stopped_cancel STOPPED because of a cancel.
  /// @param status Status bits are OR-ed into it (may be null).
  /// @param alpha If not null, receives \f$\alpha\f$ (FieldState.limiter_scale); with
  /// `brake_translation`, the smaller of the translational and the angular \f$\alpha\f$.
  /// @param brake_translation The phase's translational target is zero (ROTATE_TO_FIELD,
  /// ROTATE_TO_HEADING, STOPPED): the translational axes decelerate at their own bound,
  /// independently of \f$\omega\f$, instead of sharing \f$\omega\f$'s \f$\alpha\f$.
  /// @return The limited command.
  Twist2 limit(
    const Twist2 & target, const LimiterState & in, LimiterState * out, double dt,
    const Kinematics & kinematics, bool stopped_cancel, std::uint32_t * status,
    double * alpha = nullptr, bool brake_translation = false) const noexcept;
};

static_assert(std::is_trivially_destructible_v<LimiterState>, "LimiterState is TrackerState data");

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__LIMITER_HPP_
