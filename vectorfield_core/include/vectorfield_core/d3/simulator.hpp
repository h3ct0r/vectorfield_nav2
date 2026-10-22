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
/// @brief 3D core (experimental): point / multirotor kinematic plant and the closed-loop
/// simulator of a Tracker3 (math_spec §14).

#ifndef VECTORFIELD_CORE__D3__SIMULATOR_HPP_
#define VECTORFIELD_CORE__D3__SIMULATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "vectorfield_core/d3/frame.hpp"
#include "vectorfield_core/d3/tracker.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/status.hpp"

namespace vectorfield_core::d3::sim
{

/// @brief Plant parameters.
struct PlantParams3
{
  /// First-order velocity lag \f$\tau\f$ (s) of the velocity loop (multirotor approximation);
  /// 0 is an ideal point (the command is the velocity).
  double velocity_time_constant{0.0};
};

/// @brief Plant state: pose and realised twist.
struct PlantState3
{
  /// Pose.
  Pose3 pose;
  /// Realised twist (curve frame linear velocity, yaw rate).
  Twist3 twist;
};

/// @brief Integrates the plant over \f$\Delta t\f$ under a zero-order-hold command, exactly:
/// \f$\tau = 0\f$: \f$p \mathrel{+}= v\Delta t\f$; \f$\tau > 0\f$: \f$\dot v = (u - v)/\tau\f$
/// (same for \f$\omega\f$), integrated in closed form. A non-finite command or state is
/// returned unchanged.
PlantState3 integrate(
  const PlantState3 & state, const Twist3 & cmd, double dt, const PlantParams3 & params) noexcept;

/// @brief An additive velocity disturbance \f$\delta_v(t, \mathrm{pose})\f$ (m/s).
using Disturbance3 = std::function<Vec3(double t, const Pose3 & pose)>;

/// @brief The curve frame at time t (pose and twist), for a frame with a prescribed motion.
using FrameMotion3 = std::function<CurveFrame3(double t)>;

/// @brief Options of ClosedLoopSimulator3.
struct SimulationOptions3
{
  /// Control period (s).
  double period{0.05};
  /// Plant.
  PlantParams3 plant;
  /// Optional disturbance, added to the realised velocity.
  Disturbance3 disturbance;
  /// run() stops at the first settled step.
  bool stop_when_settled{true};
  /// run() stops when a cancel completes.
  bool stop_when_stopped{true};
  /// The curve frame at the reset time \f$t_0\f$. A moving frame moves with its constant twist,
  /// exactly: \f$T(t) = T(t_0)\exp((t - t_0)\hat\xi)\f$ (math_spec §14.7). Default: identity.
  CurveFrame3 frame;
  /// Optional frame motion; when set it gives the frame (and the twist the tracker sees) at
  /// every step and `frame` is ignored.
  FrameMotion3 frame_motion;
};

/// @brief One recorded step.
struct SimulationRecord3
{
  /// Time at the start of the step.
  double t{0.0};
  /// Pose at the start of the step.
  Pose3 pose;
  /// Command.
  Twist3 cmd;
  /// Phase.
  Phase phase{Phase::kFollow};
  /// Status bits.
  std::uint32_t status{Status::kNone};
  /// Settled.
  bool settled{false};
  /// Distance D.
  double D{0.0};
  /// Terminal \f$\sigma\f$ (NaN on closed curves).
  double sigma{kNaN};
  /// Station of the foot.
  double s_star{0.0};
  /// Field at the body.
  Vec3 F{Vec3::Zero()};
  /// The curve frame \f$T_{\mathrm{eval}\leftarrow\mathcal F}\f$ at the start of the step.
  Isometry3 T_eval_curve{Isometry3::Identity()};
};

/// @brief Closed loop of a Tracker3 and the plant at a fixed period (ZOH). The simulator owns
/// the curve frame: step() replaces Tracker3Input::frame by frameAt() of the step's time.
class ClosedLoopSimulator3
{
public:
  /// @throws ConfigError on a non-positive period, a negative time constant, or a non-rigid
  /// `options.frame`.
  explicit ClosedLoopSimulator3(Tracker3 tracker, SimulationOptions3 options = {});

  /// @brief Resets pose, time, twist, tracker state and records.
  void reset(const Pose3 & pose, double t0 = 0.0, const Twist3 & twist = {});
  /// @brief One step: Tracker3::step at the current pose and time, then the plant.
  const SimulationRecord3 & step(const Tracker3Input & input);
  /// @brief Steps until `duration` elapses (or a stop condition); `update` may edit the input
  /// before each step.
  /// @return The number of steps taken.
  std::size_t run(
    const Tracker3Input & input, double duration,
    const std::function<void(double t, Tracker3Input * input)> & update = {});

  /// @brief The curve frame at time t: `frame_motion(t)`, or `frame` extrapolated from the reset
  /// time.
  CurveFrame3 frameAt(double t) const;
  /// @brief Current pose.
  const Pose3 & pose() const noexcept { return plant_.pose; }
  /// @brief Current time.
  double time() const noexcept { return t_; }
  /// @brief Tracker state.
  const Tracker3State & state() const noexcept { return state_; }
  /// @brief Output of the last step.
  const Tracker3Output & lastOutput() const noexcept { return last_output_; }
  /// @brief Every recorded step.
  const std::vector<SimulationRecord3> & records() const noexcept { return records_; }
  /// @brief The tracker.
  const Tracker3 & tracker() const noexcept { return tracker_; }

private:
  Tracker3 tracker_;
  SimulationOptions3 options_;
  Tracker3State state_;
  Tracker3Output last_output_;
  PlantState3 plant_;
  double t_{0.0};
  double t0_{0.0};
  std::vector<SimulationRecord3> records_;
};

}  // namespace vectorfield_core::d3::sim

#endif  // VECTORFIELD_CORE__D3__SIMULATOR_HPP_
