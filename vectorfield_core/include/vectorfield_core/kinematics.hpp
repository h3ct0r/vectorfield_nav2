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
/// @brief Kinematic model and limits of the robot, and exact planar integration
/// (plan §3.2; math_spec §7.5, §10.2).

#ifndef VECTORFIELD_CORE__KINEMATICS_HPP_
#define VECTORFIELD_CORE__KINEMATICS_HPP_

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Motion model and limits, shared by the tracking laws, the kinematic box, the
/// CommandLimiter, the CollisionPredictor and sim::integrate (plan §3.2).
///
/// A plain value type; fromConfig() copies the Motion and Phases parameters it needs without
/// validating them (validateConfig() does that).
struct Kinematics
{
  /// Motion model.
  MotionModel type{MotionModel::kDifferential};
  /// \f$v_{\max}\f$ (m/s).
  double max_linear_vel{0.5};
  /// \f$v_{y,\max}\f$ (m/s), omnidirectional only.
  double max_lateral_vel{0.5};
  /// \f$\omega_{\max}\f$ (rad/s).
  double max_angular_vel{1.5};
  /// \f$a_{\mathrm{acc}}\f$ (m/s²).
  double max_linear_accel{1.0};
  /// \f$a_{\mathrm{dec}}\f$ (m/s², positive).
  double max_linear_decel{1.0};
  /// \f$\alpha_{\max}\f$ (rad/s²).
  double max_angular_accel{3.2};
  /// \f$a_{\mathrm{can}}\f$ (m/s²), the linear bound during a cancel.
  double cancel_deceleration{1.0};
  /// FL control-point offset \f$d\f$ (m).
  double control_point_offset{0.2};
  /// Ackermann rear-axle offset \f$x_r\f$ along body x from base_link (m; negative = behind).
  double reference_point_offset{0.0};
  /// Ackermann \f$R_{\min}\f$ (m); <= 0 means no curvature limit.
  double min_turning_radius{0.0};
  /// Ackermann \f$v_{\mathrm{ack}}\f$ (m/s).
  double ackermann_min_speed{0.05};
  /// Drive backward: \f$q = -1\f$.
  bool drive_backward{false};
  /// Allow \f$q\,v < 0\f$.
  bool allow_reversing{false};

  /// @brief The kinematics of a configuration (Motion group, plus `cancel_deceleration`).
  static Kinematics fromConfig(const CoreConfig & config) noexcept;

  /// @brief Drive sign \f$q\f$: −1 iff drive_backward.
  double driveSign() const noexcept { return drive_backward ? -1.0 : 1.0; }
  /// @brief True for the omnidirectional model (\f$v_y\f$ is commandable).
  bool isHolonomic() const noexcept { return type == MotionModel::kOmnidirectional; }
  /// @brief True for the Ackermann model.
  bool isAckermann() const noexcept { return type == MotionModel::kAckermann; }
  /// @brief True for differential and skid-steer (unicycle) models.
  bool isUnicycle() const noexcept
  {
    return type == MotionModel::kDifferential || type == MotionModel::kSkidSteer;
  }
  /// @brief Effective angular limit \f$\bar\omega = \omega_{\max}\gamma_{\mathrm{lim}}\f$
  /// (math_spec §1).
  double angularLimit(double speed_limit_scale) const noexcept
  {
    return max_angular_vel * speed_limit_scale;
  }
};

/// @brief Integration helpers used by the CollisionPredictor, the ClosedLoopSimulator and
/// vectorfield_sim.
namespace sim
{

/// @brief Simulated slip of the plant (simulator and vectorfield_sim only; never in the
/// predictor's nominal plant).
///
/// Applied to the feasible body twist before integration: \f$\omega' = \eta_\psi\,\omega\f$ and
/// \f$v_y' = v_y - x_{\mathrm{ICR}}\,\omega'\f$ (the instantaneous centre of rotation sits at body
/// \f$x = x_{\mathrm{ICR}}\f$). The defaults are the identity; vectorfield_sim uses
/// \f$\eta_\psi = 0.75\f$, \f$x_{\mathrm{ICR}} = 0.05\f$ for skid steer.
struct SlipParams
{
  /// Yaw-rate efficiency \f$\eta_\psi\f$.
  double eta_psi{1.0};
  /// ICR longitudinal offset \f$x_{\mathrm{ICR}}\f$ (m).
  double x_icr{0.0};
};

/// @brief The body twist the model can execute: unicycle models drop \f$v_y\f$; Ackermann drops
/// \f$v_y\f$ and saturates \f$\lvert\omega\rvert \le \lvert v\rvert/R_{\min}\f$ when
/// \f$R_{\min} > 0\f$ (a car cannot turn in place, so \f$(v, \omega) = (0, \omega)\f$ does not
/// rotate it); omnidirectional is unchanged. This is the rear-axle twist for Ackermann.
Twist2 feasibleTwist(const Kinematics & kinematics, const Twist2 & command) noexcept;

/// @brief Exact SE(2) integration of a constant body twist (math_spec §10.2 [Derived]):
/// \f$p(\tau) = p + R(\psi)\big(\frac{\sin\omega\tau}{\omega}I + \frac{1 - \cos\omega\tau}{\omega}
/// J\big)(v_x, v_y)^\top\f$, \f$\psi(\tau) = \psi + \omega\tau\f$, with the limit \f$\tau I\f$ as
/// \f$\omega \to 0\f$ (no cancellation at small \f$\omega\tau\f$).
/// @param pose Start pose of the integrated point.
/// @param twist Body twist at that point.
/// @param dt Duration \f$\tau\f$ (s).
/// @return The end pose (yaw not wrapped); `pose` when dt <= 0 or dt is not finite.
Pose2 integrateTwist(const Pose2 & pose, const Twist2 & twist, double dt) noexcept;

/// @brief One zero-order-hold step of the plant (math_spec §10.2): exact-arc unicycle,
/// omnidirectional and bicycle.
///
/// The command (a body twist at base_link) is first made feasible (feasibleTwist()), then slip
/// is applied to the twist of the integrated point, then integrateTwist(). The bicycle
/// (Ackermann) is integrated at the rear axle
/// \f$p_r = p + x_r\mathbf h(\psi)\f$, which has no lateral velocity, and mapped back to
/// base_link by the fixed offset; base_link's lateral velocity \f$-x_r\omega\f$ follows from the
/// rigid body (math_spec §7.5 "Command semantics").
/// @param kinematics Motion model and limits.
/// @param pose base_link pose.
/// @param command Commanded body twist \f$(v, v_y, \omega)\f$.
/// @param dt Duration (s).
/// @param slip Plant slip (identity by default).
/// @return The base_link pose after dt (yaw not wrapped); `pose` when dt <= 0 or dt is not
/// finite. Non-finite twists give non-finite poses.
Pose2 integrate(
  const Kinematics & kinematics, const Pose2 & pose, const Twist2 & command, double dt,
  const SlipParams & slip = {}) noexcept;

}  // namespace sim

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__KINEMATICS_HPP_
