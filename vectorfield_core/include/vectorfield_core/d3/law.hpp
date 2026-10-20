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
/// @brief 3D core (experimental): the holonomic single-integrator law with its kinematic box and
/// yaw policy, and the uniform-\f$\alpha\f$ CommandLimiter3 (math_spec §8.3–§8.4, §14.4).

#ifndef VECTORFIELD_CORE__D3__LAW_HPP_
#define VECTORFIELD_CORE__D3__LAW_HPP_

#include <cstdint>
#include <type_traits>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/field.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/limiter.hpp"
#include "vectorfield_core/status.hpp"

namespace vectorfield_core::d3
{

/// @brief Velocity, acceleration and yaw limits of a holonomic 3D body.
struct Limits3
{
  /// \f$\lVert v\rVert \le v_{\max}\f$ (m/s).
  double max_speed{1.0};
  /// Per-axis bounds \f$\lvert v_i\rvert \le v_{i,\max}\f$ (m/s), curve frame.
  Vec3 max_axis_speed{1.0, 1.0, 0.5};
  /// Acceleration bound \f$a_{\mathrm{acc}}\f$ per axis (m/s²).
  double max_accel{1.0};
  /// Deceleration bound \f$a_{\mathrm{dec}}\f$ per axis (m/s²).
  double max_decel{1.0};
  /// Deceleration of a cancel \f$a_{\mathrm{can}}\f$ (m/s²).
  double cancel_decel{1.0};
  /// \f$\lvert\omega\rvert \le \omega_{\max}\f$ (rad/s).
  double max_yaw_rate{1.5};
  /// Yaw acceleration \f$\alpha_{\max}\f$ (rad/s²).
  double max_yaw_accel{3.2};
};

/// @brief Yaw policy of the 3D law (the planar omni heading policy enum).
struct YawConfig3
{
  /// kPathTangent: \f$\psi_d = \angle T_d\f$; kField: \f$\angle F\f$; kFixed: `fixed_yaw`.
  OmniHeadingPolicy policy{OmniHeadingPolicy::kPathTangent};
  /// \f$k_\psi\f$ (1/s).
  double gain{2.0};
  /// \f$\psi_d\f$ for kFixed (rad).
  double fixed_yaw{0.0};
};

/// @brief Horizontal norm below which \f$\psi_d\f$ is held (math_spec §14.4).
inline constexpr double kYawHoldNorm = 1e-3;

/// @brief \f$\{\eta : \lVert a + \eta b\rVert \le L\}\f$ in \f$\mathbb R^3\f$ (math_spec §8.3).
Interval boxNormInterval(const Vec3 & a, const Vec3 & b, double limit) noexcept;

/// @brief Uniform box scale \f$c = \min(1, v_{\max}/\lVert v\rVert, v_{i,\max}/\lvert v_i\rvert)\f$
/// (terms with a zero denominator dropped).
double uniformBoxScale(const Vec3 & v, const Limits3 & limits) noexcept;

/// @brief Output of VelocityLaw3::command().
struct LawOutput3
{
  /// Command after the kinematic box.
  Twist3 cmd;
  /// The shaped field before the box (\f$\gamma\Phi\f$, or \f$\eta_1\Phi_S + \Phi_T\f$).
  Vec3 F{Vec3::Zero()};
  /// Uniform scale c (1 when the \f$\eta\f$-box was used).
  double c{1.0};
  /// \f$\eta^\ast\f$ of the \f$\eta\f$-box, else NaN.
  double eta{kNaN};
  /// The \f$\eta\f$-box produced the command.
  bool eta_box{false};
  /// Desired yaw \f$\psi_d\f$ (rad), held or the current yaw when undefined.
  double psi_d{kNaN};
  /// Status bits (kAssumption3Clamped).
  std::uint32_t status{Status::kNone};
};

/// @brief Holonomic single-integrator law \f$v = F\f$ with a kinematic box and a yaw policy
/// (math_spec §14.4).
class VelocityLaw3
{
public:
  /// @brief Stores the limits and the yaw policy (validated by validateTracker3Config()).
  VelocityLaw3(const Limits3 & limits, const YawConfig3 & yaw) noexcept;

  /// @brief The command for one valid field sample.
  /// @param sample The evaluated field (FieldSample3::valid).
  /// @param gamma Speed-shaping factor \f$\gamma \in [0, 1]\f$.
  /// @param v_r Reference speed.
  /// @param assumption3_margin \f$\varepsilon_v\f$.
  /// @param yaw Current yaw \f$\psi\f$.
  /// @param last_psi_d Previous \f$\psi_d\f$ (NaN for none).
  LawOutput3 command(
    const FieldSample3 & sample, double gamma, double v_r, double assumption3_margin, double yaw,
    double last_psi_d) const noexcept;

  /// @brief \f$\psi_d\f$ of the policy for direction source `w` (T_d or F); held when the
  /// horizontal part of `w` is below kYawHoldNorm.
  double desiredYaw(const Vec3 & w, double yaw, double last_psi_d) const noexcept;

  /// @brief The limits.
  const Limits3 & limits() const noexcept { return limits_; }
  /// @brief The yaw policy.
  const YawConfig3 & yaw() const noexcept { return yaw_; }

private:
  Limits3 limits_;
  YawConfig3 yaw_;
};

/// @brief Cross-cycle state of CommandLimiter3.
struct LimiterState3
{
  /// Previous output.
  Twist3 u_prev;
  /// False requests a re-seed from the measured twist (initial state, reset, \f$\Delta t > 2/f\f$).
  bool valid{false};
};

/// @brief Uniform-\f$\alpha\f$ acceleration limiter over \f$(v_x, v_y, v_z, \omega)\f$ and the
/// non-finite guard (math_spec §8.4, §14.4). Stateless; `*out` may alias `in`.
class CommandLimiter3
{
public:
  /// @brief \f$u = u_{\mathrm{prev}} + \alpha(u^\ast - u_{\mathrm{prev}})\f$ with the largest
  /// \f$\alpha \le 1\f$ meeting every axis bound (\f$a_{\mathrm{dec}}\f$ when
  /// \f$u_{\mathrm{prev},i}\Delta_i < 0\f$, else \f$a_{\mathrm{acc}}\f$; \f$\alpha_{\max}\f$ for
  /// \f$\omega\f$). With `brake_translation` the translation and \f$\omega\f$ take separate
  /// scales; `stopped_cancel` uses \f$a_{\mathrm{can}}\f$ for the translation. Any non-finite
  /// component gives \f$u = 0\f$ and Status::kNonFinite.
  /// @param target Target \f$u^\ast\f$.
  /// @param in Previous state.
  /// @param out Next state (u_prev = result, valid = true).
  /// @param dt \f$\Delta t\f$; a non-positive dt keeps \f$u_{\mathrm{prev}}\f$.
  /// @param limits Bounds.
  /// @param stopped_cancel STOPPED because of a cancel.
  /// @param brake_translation The translational target is zero.
  /// @param status OR-ed status bits (may be null).
  /// @param alpha Receives \f$\alpha\f$ (the smaller of the two when braking; may be null).
  Twist3 limit(
    const Twist3 & target, const LimiterState3 & in, LimiterState3 * out, double dt,
    const Limits3 & limits, bool stopped_cancel, bool brake_translation, std::uint32_t * status,
    double * alpha = nullptr) const noexcept;
};

static_assert(std::is_trivially_destructible_v<LimiterState3>, "Tracker3State data");

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__LAW_HPP_
