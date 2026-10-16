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

#include "plan_generators.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace vectorfield_nav2_controller
{
namespace test
{

std::vector<vectorfield_core::Pose2> straightPlan(
  const vectorfield_core::Vec2 & /*start*/, const vectorfield_core::Vec2 & /*end*/,
  double /*spacing*/, double /*end_yaw*/)
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: test::straightPlan");
}

std::vector<vectorfield_core::Pose2> navfnTail(
  const vectorfield_core::Pose2 & /*goal*/, const vectorfield_core::Vec2 & /*goal_offset*/,
  double /*heading*/, double /*approach_length*/, double /*resolution*/)
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: test::navfnTail");
}

std::vector<vectorfield_core::Pose2> reversingPlan(
  const vectorfield_core::Pose2 & /*start*/, double /*forward_length*/, double /*reverse_length*/,
  double /*spacing*/)
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: test::reversingPlan");
}

nav_msgs::msg::Path toPathMsg(
  vectorfield_core::Span<const vectorfield_core::Pose2> /*poses*/, const std::string & /*frame_id*/)
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: test::toPathMsg");
}

}  // namespace test
}  // namespace vectorfield_nav2_controller
