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
/// @brief 3D core (experimental): the minimal Tracker3 (math_spec §14.5).

#ifndef VECTORFIELD_CORE__D3__TRACKER_HPP_
#define VECTORFIELD_CORE__D3__TRACKER_HPP_

#include <cstdint>
#include <type_traits>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/d3/field.hpp"
#include "vectorfield_core/d3/law.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/status.hpp"

namespace vectorfield_core::d3
{

/// @brief Configuration of Tracker3 (planar parameter names where the meaning is the same).
struct Tracker3Config
{
  /// Reference speed \f$v_r\f$ (m/s).
  double desired_speed{0.5};
  /// \f$k_f\f$ (1/m).
  double convergence_gain{2.0};
  /// G-type function.
  GainType gain_function{GainType::kAtan};
  /// Traversal direction ±1.
  int direction{1};
  /// \f$\varepsilon_v \in (0, 1)\f$.
  double assumption3_margin{0.1};
  /// Goal tolerance of open curves (m): the XY latch radius; settled within half of it.
  double goal_tolerance{0.10};
  /// \f$\sigma_b\f$ of the terminal profile (m).
  double terminal_braking_distance{0.5};
  /// \f$v_{\min}\f$ of the terminal profile (m/s).
  double min_approach_speed{0.05};
  /// FINAL_APPROACH entry distance; kAutoValue: \f$\max(\sigma_b, \mathrm{tol})\f$.
  double final_approach_distance{kAutoValue};
  /// Control frequency f (Hz): \f$\Delta t > 2/f\f$ re-seeds the limiter.
  double control_frequency{20.0};
  /// Velocity, acceleration and yaw limits.
  Limits3 limits;
  /// Yaw policy.
  YawConfig3 yaw;
};

/// @brief Throws ConfigError naming the first invalid field of `config`.
void validateTracker3Config(const Tracker3Config & config);

/// @brief Cross-cycle state of Tracker3 (trivially destructible, copyable).
struct Tracker3State
{
  /// Current phase (kFollow, kFinalApproach or kStopped).
  Phase phase{Phase::kFollow};
  /// Closest-point hint.
  CurveHint3 hint;
  /// \f$T_{\mathrm{last}}\f$ for the hairpin rule.
  Vec3 last_tangent{Vec3::Zero()};
  /// Previous \f$\psi_d\f$.
  double last_psi_d{kNaN};
  /// XY latch \f$\lVert p - g\rVert \le\f$ tol (cleared by a goal_seq change).
  bool xy_latched{false};
  /// Goal hold latch.
  bool holding{false};
  /// goal_seq of the last step.
  std::uint32_t goal_seq{0};
  /// Limiter state.
  LimiterState3 limiter;
};

/// @brief Inputs of one Tracker3 cycle.
struct Tracker3Input
{
  /// Body pose (curve frame).
  Pose3 pose;
  /// Measured twist (seeds the limiter).
  Twist3 measured;
  /// Time t (s).
  double t{0.0};
  /// \f$\Delta t\f$ (s).
  double dt{0.05};
  /// The curve (not owned; must outlive the call).
  const Curve3 * curve{nullptr};
  /// Goal identity; a change resets the latch, the hold and FINAL_APPROACH / STOPPED.
  std::uint32_t goal_seq{0};
  /// \f$\gamma_{\mathrm{lim}} \in [0, 1]\f$.
  double speed_limit_scale{1.0};
  /// Cancel: STOPPED, decelerating at \f$a_{\mathrm{can}}\f$.
  bool cancel{false};
};

/// @brief Outputs of one Tracker3 cycle.
struct Tracker3Output
{
  /// Limited command.
  Twist3 cmd;
  /// Target before the limiter.
  Twist3 cmd_target;
  /// The field sample at the body position.
  FieldSample3 field;
  /// Phase after this cycle.
  Phase phase{Phase::kFollow};
  /// Status bits.
  std::uint32_t status{Status::kNone};
  /// Open curve: latched and within tol/2 of the goal.
  bool settled{false};
  /// STOPPED by a cancel and below 0.01 m/s and 0.01 rad/s.
  bool cancel_complete{false};
  /// The goal of an open curve.
  Vec3 goal{Vec3::Zero()};
  /// True for an open curve.
  bool has_goal{false};
  /// Desired yaw.
  double psi_d{kNaN};
  /// Uniform box scale c.
  double box_scale{1.0};
  /// Limiter \f$\alpha\f$.
  double limiter_alpha{1.0};
};

/// @brief A minimal 3D tracker: FOLLOW, FINAL_APPROACH (open curves), settled and goal hold,
/// cancel → STOPPED (math_spec §14.5). No obstacles. `step` is `const noexcept`, allocation-free
/// and thread-safe (the state is passed in and out).
class Tracker3
{
public:
  /// @throws ConfigError (validateTracker3Config()).
  explicit Tracker3(const Tracker3Config & config);

  /// @brief One control cycle. `*out` may alias `in`.
  Tracker3Output step(
    const Tracker3Input & input, const Tracker3State & in, Tracker3State * out) const noexcept;

  /// @brief The initial state.
  static Tracker3State initialState() noexcept { return Tracker3State{}; }

  /// @brief The configuration.
  const Tracker3Config & config() const noexcept { return config_; }
  /// @brief The field.
  const DistanceField3 & field() const noexcept { return field_; }
  /// @brief The law.
  const VelocityLaw3 & law() const noexcept { return law_; }
  /// @brief The terminal profile.
  const TerminalProfile & profile() const noexcept { return profile_; }
  /// @brief The effective FINAL_APPROACH distance.
  double finalApproachDistance() const noexcept { return final_approach_distance_; }

private:
  Tracker3Config config_;
  DistanceField3 field_;
  VelocityLaw3 law_;
  TerminalProfile profile_;
  double final_approach_distance_;
};

static_assert(std::is_trivially_destructible_v<Tracker3State>, "Tracker3State is plain data");

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__TRACKER_HPP_
