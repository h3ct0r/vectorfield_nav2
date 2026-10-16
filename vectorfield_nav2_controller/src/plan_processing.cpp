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

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "vectorfield_nav2_controller/plan_processing.hpp"

namespace vectorfield_nav2_controller
{

bool sameGoal(const vectorfield_core::Pose2 & /*a*/, const vectorfield_core::Pose2 & /*b*/) noexcept
{
  // M2 wave 2: path
  return false;
}

PlanIntake intakePlan(
  vectorfield_core::Span<const vectorfield_core::Pose2> /*plan*/, double /*chord_length*/,
  PlanIntakeState * /*state*/, std::vector<vectorfield_core::Vec2> * /*positions*/,
  std::vector<vectorfield_core::Vec2> * /*decimated*/) noexcept
{
  // M2 wave 2: path
  return PlanIntake{};
}

WindowBuild buildWindow(
  const WindowInput & /*input*/, PlanIntakeState * /*state*/,
  std::vector<vectorfield_core::Vec2> * /*work*/,
  std::vector<vectorfield_core::Vec2> * /*out*/) noexcept
{
  // M2 wave 2: path
  return WindowBuild{};
}

double speedLimitScale(
  double /*speed_limit*/, bool /*percentage*/, double /*desired_speed*/) noexcept
{
  // M2 wave 2: path
  return 1.0;
}

bool resetDue(double /*seconds_since_last_compute*/, double /*control_period*/) noexcept
{
  // M2 wave 2: path
  return false;
}

std::string noValidControlReason(std::uint32_t /*status*/, bool /*cancelling*/)
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: vectorfield_nav2_controller::noValidControlReason");
}

}  // namespace vectorfield_nav2_controller
