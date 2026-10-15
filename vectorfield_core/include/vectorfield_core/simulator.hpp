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
/// @brief Closed-loop kinematic simulation of a Tracker: zero-order hold, exact integration,
/// optional slip and bounded disturbance (plan §3.2, §8; used by tests and the Python tools).
///
/// Implemented in src/simulator.cpp (M1 wave 3, tracker). Not a hot path: it records every step
/// and may allocate.

#ifndef VECTORFIELD_CORE__SIMULATOR_HPP_
#define VECTORFIELD_CORE__SIMULATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{
namespace sim
{

/// @brief A world-frame velocity disturbance \f$\delta(t, \text{pose})\f$ (m/s), held constant
/// over each control period and added to the base velocity (exact: a constant world-frame
/// velocity adds \f$\delta\,\Delta t\f$ to the position).
using Disturbance = std::function<Vec2(double t, const Pose2 & pose)>;

/// @brief A seeded random disturbance, uniform in the disc of radius `bound` (m/s), redrawn each
/// period; reproducible for a given seed.
Disturbance boundedRandomDisturbance(double bound, std::uint64_t seed);

/// @brief Options of a closed-loop simulation.
struct SimulationOptions
{
  /// Control period: the ZOH step and TrackerInput::dt (s).
  double period{0.05};
  /// Plant slip (identity by default).
  SlipParams slip;
  /// Optional disturbance (empty: none).
  Disturbance disturbance;
  /// Stop run() when the output is settled.
  bool stop_when_settled{true};
  /// Stop run() when the phase is STOPPED and the robot has stopped.
  bool stop_when_stopped{true};
};

/// @brief One recorded control period.
struct SimulationRecord
{
  /// Time at the start of the period (s).
  double t{0.0};
  /// base_link pose at the start of the period.
  Pose2 pose;
  /// Measured twist given to the Tracker.
  Twist2 measured;
  /// Command held over the period.
  Twist2 cmd;
  /// Phase.
  Phase phase{Phase::kFollow};
  /// Status bits.
  std::uint32_t status{Status::kNone};
  /// Settled.
  bool settled{false};
  /// Distance to the curve at the law's point.
  double D{0.0};
  /// \f$\sigma_{\mathrm{base}}\f$ (NaN closed).
  double sigma{kNaN};
  /// Arc-length station of the closest point.
  double s_star{0.0};
  /// Law's evaluation point.
  Vec2 x_eval{Vec2::Zero()};
  /// Tracked field F.
  Vec2 F{Vec2::Zero()};
  /// Free arc length.
  double s_free{kInf};
};

/// @brief ZOH loop: TrackerInput → Tracker::step → sim::integrate over one period, repeated.
///
/// The simulator owns a copy of the Tracker, the TrackerState, a StepScratch and the plant pose.
/// Each step builds the input from a caller template, overwriting pose, measured (the last
/// command, as an ideal odometry), t and dt.
class ClosedLoopSimulator
{
public:
  /// @brief A simulator whose plant uses the Tracker's kinematics.
  explicit ClosedLoopSimulator(Tracker tracker, SimulationOptions options = {});
  /// @brief A simulator with a distinct plant (e.g. other limits).
  ClosedLoopSimulator(Tracker tracker, const Kinematics & plant, SimulationOptions options);

  /// @brief Resets the time, pose, twist, TrackerState (initialState()) and records.
  void reset(const Pose2 & pose, double t0 = 0.0, const Twist2 & twist = {});

  /// @brief One control period.
  /// @param input Template: curve, frame, goal, obstacles, speed limit, cancel, cap flag.
  /// @return The record of this period.
  const SimulationRecord & step(const TrackerInput & input);

  /// @brief Runs until `duration` elapses or a stop condition of the options holds.
  /// @param input Template, as for step().
  /// @param duration Simulated time (s).
  /// @param update Optional per-step hook to edit the input (moving goals, obstacles, cancel).
  /// @return Number of periods simulated.
  std::size_t run(
    const TrackerInput & input, double duration,
    const std::function<void(double t, TrackerInput * input)> & update = {});

  /// @brief Current base_link pose.
  const Pose2 & pose() const noexcept;
  /// @brief Current time (s).
  double time() const noexcept;
  /// @brief Current TrackerState.
  const TrackerState & state() const noexcept;
  /// @brief The scratch of the last step (rollout poses, K).
  const StepScratch & scratch() const noexcept;
  /// @brief The last output.
  const TrackerOutput & lastOutput() const noexcept;
  /// @brief Every recorded period.
  const std::vector<SimulationRecord> & records() const noexcept;
  /// @brief The Tracker.
  const Tracker & tracker() const noexcept;

private:
  Tracker tracker_;
  Kinematics plant_;
  SimulationOptions options_;
  TrackerState state_;
  StepScratch scratch_;
  TrackerOutput last_output_;
  Pose2 pose_;
  Twist2 twist_;
  double t_{0.0};
  std::vector<SimulationRecord> records_;
};

}  // namespace sim
}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__SIMULATOR_HPP_
