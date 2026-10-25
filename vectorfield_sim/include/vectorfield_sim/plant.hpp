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
/// @brief The simulated robot: fixed-step integration with `vectorfield_core::sim::integrate`,
/// the command timeout, the Ackermann infeasible-command count, simulated time and the
/// map→odom bookkeeping. ROS-free.

#ifndef VECTORFIELD_SIM__PLANT_HPP_
#define VECTORFIELD_SIM__PLANT_HPP_

#include <cstdint>

#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_sim
{

/// @brief Slip of the skid-steer plant: \f$\eta_\psi = 0.75\f$, \f$x_{\mathrm{ICR}} = 0.05\f$ m
///. The other models use the identity.
inline constexpr vectorfield_core::sim::SlipParams kSkidSteerSlip{0.75, 0.05};

/// @brief Velocities at or below this magnitude count as zero in the Ackermann infeasibility
/// test (m/s, rad/s).
inline constexpr double kZeroVelocity = 1e-6;

/// @brief Configuration of a KinematicPlant.
struct PlantConfig
{
  /// Motion model. sim::integrate reads only `type`, `min_turning_radius` and
  /// `reference_point_offset`; the plant never saturates speeds.
  vectorfield_core::Kinematics kinematics;
  /// Plant slip, applied to every model as given.
  vectorfield_core::sim::SlipParams slip;
  /// Fixed step (ns), > 0.
  std::int64_t step_ns{10'000'000};
  /// A command older than this (simulated ns) is replaced by zero; <= 0 disables the timeout.
  std::int64_t command_timeout_ns{500'000'000};
};

/// @brief The body twist that one step of sim::integrate executes at base_link: the feasible
/// twist with slip applied, plus, for Ackermann, the lateral velocity \f$-x_r\omega\f$ of
/// base_link relative to the rear axle.
vectorfield_core::Twist2 executedTwist(
  const vectorfield_core::Kinematics & kinematics, const vectorfield_core::Twist2 & command,
  const vectorfield_core::sim::SlipParams & slip) noexcept;

/// @brief True for an Ackermann command a car cannot execute: \f$v = 0\f$ with
/// \f$\omega \ne 0\f$ (both against kZeroVelocity).
bool isInfeasibleAckermann(const vectorfield_core::Twist2 & command) noexcept;

/// @brief Seconds to integer nanoseconds (rounded); non-finite or negative inputs give 0.
std::int64_t toNanoseconds(double seconds) noexcept;

/// @brief Number of fixed steps per period, at least 1 (e.g. the 10 Hz scan at a 10 ms step
/// is every 10 steps).
std::int64_t stepsPerPeriod(std::int64_t period_ns, std::int64_t step_ns) noexcept;

/// @brief The simulated robot.
///
/// **Frames.** The plant integrates base_footprint in the odom frame, starting at the identity.
/// Two map→odom transforms are kept:
/// - the *published* one (the localization estimate, sent on `/tf_static`), and
/// - the *true* one, which places the robot in the world for the laser (groundTruth()).
///
/// setInitialPose() moves both (it teleports the robot and localizes it perfectly);
/// stepMapToOdom() moves only the published one, which is what a localization correction looks
/// like to the rest of the stack.
///
/// **Time** is an integer count of fixed steps; a command is stamped with the simulated time at
/// which setCommand() is called. Not thread-safe.
class KinematicPlant
{
public:
  /// @brief A plant at rest at the odom origin, with both map→odom transforms at `map_to_odom`.
  explicit KinematicPlant(
    const PlantConfig & config, const vectorfield_core::Pose2 & map_to_odom = {}) noexcept;

  /// @brief Holds `command` from now on (until the next command or the timeout).
  /// @return True when it is an infeasible Ackermann command (only counted for Ackermann).
  bool setCommand(const vectorfield_core::Twist2 & command) noexcept;

  /// @brief Advances one fixed step with the held command, or zero once it is older than the
  /// timeout.
  void step() noexcept;

  /// @brief Places base_footprint at `map_pose` (map frame) by moving both map→odom transforms;
  /// the odom pose is unchanged.
  void setInitialPose(const vectorfield_core::Pose2 & map_pose) noexcept;

  /// @brief Adds (dx, dy, dyaw) to the published map→odom, component by component (the odom
  /// origin moves by (dx, dy) in the map frame and its yaw by dyaw). The true map→odom and the
  /// odom pose are unchanged.
  void stepMapToOdom(double dx, double dy, double dyaw) noexcept;

  /// @brief Simulated time (ns): steps() × step.
  std::int64_t timeNs() const noexcept { return static_cast<std::int64_t>(steps_) * step_ns_; }
  /// @brief Number of steps taken.
  std::uint64_t steps() const noexcept { return steps_; }
  /// @brief The fixed step (ns).
  std::int64_t stepNs() const noexcept { return step_ns_; }
  /// @brief Pose of base_footprint in the odom frame (yaw wrapped).
  const vectorfield_core::Pose2 & odomPose() const noexcept { return odom_pose_; }
  /// @brief Body twist executed during the last step (zero before the first step).
  const vectorfield_core::Twist2 & twist() const noexcept { return twist_; }
  /// @brief Whether the last step used a held command (false: no command yet, or timed out).
  bool commandActive() const noexcept { return command_active_; }
  /// @brief Infeasible Ackermann commands received so far (always 0 for the other models).
  std::uint64_t infeasibleCount() const noexcept { return infeasible_count_; }
  /// @brief The published map→odom (pose of odom in map, yaw wrapped).
  const vectorfield_core::Pose2 & mapToOdom() const noexcept { return map_to_odom_; }
  /// @brief The true map→odom.
  const vectorfield_core::Pose2 & trueMapToOdom() const noexcept { return true_map_to_odom_; }
  /// @brief The true pose of base_footprint in the map frame.
  vectorfield_core::Pose2 groundTruth() const noexcept;
  /// @brief The configuration.
  const PlantConfig & config() const noexcept { return config_; }

private:
  PlantConfig config_;
  std::int64_t step_ns_;
  std::uint64_t steps_{0};
  vectorfield_core::Pose2 odom_pose_;
  vectorfield_core::Twist2 twist_;
  vectorfield_core::Twist2 command_;
  std::int64_t command_time_ns_{0};
  bool has_command_{false};
  bool command_active_{false};
  std::uint64_t infeasible_count_{0};
  vectorfield_core::Pose2 map_to_odom_;
  vectorfield_core::Pose2 true_map_to_odom_;
};

}  // namespace vectorfield_sim

#endif  // VECTORFIELD_SIM__PLANT_HPP_
