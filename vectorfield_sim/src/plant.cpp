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

#include "vectorfield_sim/plant.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vectorfield_sim
{

namespace vf = vectorfield_core;

vf::Twist2 executedTwist(
  const vf::Kinematics & kinematics, const vf::Twist2 & command,
  const vf::sim::SlipParams & slip) noexcept
{
  // The same steps as sim::integrate(): feasibility, then slip on the integrated point.
  vf::Twist2 twist = vf::sim::feasibleTwist(kinematics, command);
  twist.wz *= slip.eta_psi;
  twist.vy -= slip.x_icr * twist.wz;
  if (kinematics.isAckermann()) {
    twist.vy -= kinematics.reference_point_offset * twist.wz;
  }
  return twist;
}

bool isInfeasibleAckermann(const vf::Twist2 & command) noexcept
{
  return std::abs(command.vx) <= kZeroVelocity && std::abs(command.wz) > kZeroVelocity;
}

std::int64_t toNanoseconds(double seconds) noexcept
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return 0;
  }
  return std::llround(seconds * 1e9);
}

std::int64_t stepsPerPeriod(std::int64_t period_ns, std::int64_t step_ns) noexcept
{
  if (step_ns <= 0 || period_ns <= step_ns) {
    return 1;
  }
  // Rounded to the nearest whole number of steps.
  return std::max<std::int64_t>(1, (period_ns + step_ns / 2) / step_ns);
}

KinematicPlant::KinematicPlant(const PlantConfig & config, const vf::Pose2 & map_to_odom) noexcept
: config_(config),
  step_ns_(std::max<std::int64_t>(config.step_ns, 1)),
  map_to_odom_(map_to_odom),
  true_map_to_odom_(map_to_odom)
{
  map_to_odom_.yaw = vf::wrapAngle(map_to_odom_.yaw);
  true_map_to_odom_.yaw = map_to_odom_.yaw;
}

bool KinematicPlant::setCommand(const vf::Twist2 & command) noexcept
{
  command_ = command;
  command_time_ns_ = timeNs();
  has_command_ = true;
  const bool infeasible = config_.kinematics.isAckermann() && isInfeasibleAckermann(command);
  if (infeasible) {
    ++infeasible_count_;
  }
  return infeasible;
}

void KinematicPlant::step() noexcept
{
  const std::int64_t age = timeNs() - command_time_ns_;
  command_active_ =
    has_command_ && (config_.command_timeout_ns <= 0 || age <= config_.command_timeout_ns);
  const vf::Twist2 command = command_active_ ? command_ : vf::Twist2{};
  const double dt = static_cast<double>(step_ns_) * 1e-9;
  odom_pose_ = vf::sim::integrate(config_.kinematics, odom_pose_, command, dt, config_.slip);
  odom_pose_.yaw = vf::wrapAngle(odom_pose_.yaw);
  twist_ = executedTwist(config_.kinematics, command, config_.slip);
  ++steps_;
}

void KinematicPlant::setInitialPose(const vf::Pose2 & map_pose) noexcept
{
  const vf::Isometry2 map_base = vf::Isometry2::fromPose(map_pose);
  const vf::Isometry2 odom_base = vf::Isometry2::fromPose(odom_pose_);
  map_to_odom_ = (map_base * odom_base.inverse()).toPose();
  true_map_to_odom_ = map_to_odom_;
}

void KinematicPlant::stepMapToOdom(double dx, double dy, double dyaw) noexcept
{
  map_to_odom_.p += vf::Vec2(dx, dy);
  map_to_odom_.yaw = vf::wrapAngle(map_to_odom_.yaw + dyaw);
}

vf::Pose2 KinematicPlant::groundTruth() const noexcept
{
  return vf::Isometry2::fromPose(true_map_to_odom_).apply(odom_pose_);
}

}  // namespace vectorfield_sim
