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
/// @brief Collision prediction: zero-order-hold rollout of the nominal step, free arc length and
/// braking cap (plan §3.1 steps 1–3, §3.2; math_spec §10.2–§10.3).
///
/// Implemented in src/collision_predictor.cpp (M1 wave 3, tracker). The predictor is generic:
/// the Tracker supplies its private `stepNominal` as a RolloutPolicy that owns the copied
/// TrackerState.

#ifndef VECTORFIELD_CORE__COLLISION_PREDICTOR_HPP_
#define VECTORFIELD_CORE__COLLISION_PREDICTOR_HPP_

#include <cstddef>
#include <vector>

#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief One nominal step proposed by the policy during a rollout.
struct RolloutStep
{
  /// Command held for the next control period (body twist at base_link).
  Twist2 cmd;
  /// Phase of the step.
  Phase phase{Phase::kFollow};
  /// The step is settled: the rollout stops after it (math_spec §10.2).
  bool settled{false};
};

/// @brief The rollout policy: given the pose at the start of a period and the rollout time,
/// returns the nominal step. It advances its own copy of the TrackerState (with
/// \f$\gamma_{\mathrm{col}} = 1\f$ and no slip).
using RolloutPolicy = FunctionRef<RolloutStep(const Pose2 & pose, double t)>;

/// @brief Options of one rollout.
struct RolloutOptions
{
  /// Start time \f$t\f$ (s).
  double t0{0.0};
  /// Control period: the ZOH step (s).
  double period{0.05};
  /// Horizon \f$T_h\f$ (s); side probes use \f$T_{\mathrm{side}}\f$.
  double horizon{1.5};
  /// While a cancel is in progress, continue through the STOPPED ramp until the command is below
  /// kStopLinearSpeed and kStopAngularSpeed (math_spec §10.2).
  bool through_cancel{false};
};

/// @brief Outcome of one rollout (math_spec §10.2).
struct RolloutResult
{
  /// \f$s_{\mathrm{free}}\f$: \f$+\infty\f$ if no checked pose collides; else the base arc length
  /// to the last collision-free checked pose before the first colliding one (0 if the start pose
  /// collides or the collision happens during rotation only); the arc to the last on-grid pose if
  /// the footprint leaves the grid.
  double s_free{kInf};
  /// Index into the pose buffer of the first colliding pose, or −1.
  int first_collision{-1};
  /// A checked pose collided.
  bool collided{false};
  /// The first collision happened in a rotation phase (kBlocked rotation rule, math_spec §10.4).
  bool collided_in_rotation{false};
  /// The footprint left the grid.
  bool off_grid{false};
  /// The horizon was reached.
  bool reached_horizon{false};
  /// A settled step ended the rollout.
  bool settled{false};
  /// The rollout reached STOPPED (and, through a cancel, the stop thresholds).
  bool stopped{false};
  /// Number of control periods simulated.
  std::size_t steps{0};
};

/// @brief ZOH rollout with exact sub-stepped integration on the nominal plant (math_spec §10.2).
///
/// Each period integrates the policy's command with sim::integrate (no slip); checked poses are
/// sub-sampled so consecutive ones differ by at most one cell in translation and by at most
/// \f$2\arcsin(\mathrm{res}/(2r_{\mathrm{circ}}))\f$ in yaw. Stops at the first collision, the
/// horizon, settled, off-grid or STOPPED (except through a cancel). Poses are in the evaluation
/// frame; obstacle checks map them through `ObstacleMap::T_grid_eval`. Const and allocation-free
/// once the pose buffer has capacity.
class CollisionPredictor
{
public:
  /// @brief A predictor for a plant and a footprint checker (both copied).
  CollisionPredictor(const Kinematics & plant, const FootprintChecker & checker);

  /// @brief Runs one rollout from `start`.
  /// @param start Start pose (evaluation frame).
  /// @param map Grid and \f$\mathrm{EDT}_{\mathrm{all}}\f$.
  /// @param options Times and cancel behaviour.
  /// @param policy Nominal step.
  /// @param poses Checked poses (cleared first; StepScratch::rollout).
  RolloutResult rollout(
    const Pose2 & start, const ObstacleMap & map, const RolloutOptions & options,
    RolloutPolicy policy, std::vector<Pose2> * poses) const noexcept;

  /// @brief The plant.
  const Kinematics & plant() const noexcept;
  /// @brief The checker.
  const FootprintChecker & checker() const noexcept;

private:
  Kinematics plant_;
  FootprintChecker checker_;
};

/// @brief Braking cap \f$v_{\mathrm{cap}} = \sqrt{2a_{\mathrm{dec}}\max(0, s_{\mathrm{free}} -
/// m_s)}\f$, \f$+\infty\f$ for \f$s_{\mathrm{free}} = +\infty\f$ (math_spec §10.3).
double brakingCap(double s_free, double stop_margin, double a_dec) noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__COLLISION_PREDICTOR_HPP_
