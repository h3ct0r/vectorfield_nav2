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
/// @brief Pure generators of test plans, shared by the plan-processing unit tests and the
/// layer-2 harness tests.
///
/// Implemented in test/support/plan_generators.cpp. No ROS runtime needed;
/// toPathMsg() only builds a message.

#ifndef VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__PLAN_GENERATORS_HPP_
#define VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__PLAN_GENERATORS_HPP_

#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_nav2_controller
{
namespace test
{

/// @brief Poses every `spacing` metres from `start` to `end` (both included), each oriented
/// along the segment; `end`'s yaw is `end_yaw`.
std::vector<vectorfield_core::Pose2> straightPlan(
  const vectorfield_core::Vec2 & start, const vectorfield_core::Vec2 & end, double spacing,
  double end_yaw);

/// @brief A NavFn-style tail (nav2_api_matrix.md §9): a straight approach of length
/// `approach_length` toward the goal cell along `heading`, sampled every half cell
/// (`pathStep = 0.5`) on the cell grid of `resolution`, ending with NavFn's
/// `smoothApproachToGoal` rule: the exact goal replaces the last pose if it is closer to the
/// second-to-last pose, otherwise it is appended after the goal-cell pose.
/// @param goal Exact goal pose (plan frame).
/// @param goal_offset Sub-cell offset of `goal` from its cell centre, in cells (each component
/// in [−0.5, 0.5]).
/// @param heading Approach direction (rad).
/// @param approach_length Length of the approach (m), e.g. 1.0 to 1.0375.
/// @param resolution Grid resolution (m), e.g. 0.05 or 0.10.
std::vector<vectorfield_core::Pose2> navfnTail(
  const vectorfield_core::Pose2 & goal, const vectorfield_core::Vec2 & goal_offset, double heading,
  double approach_length, double resolution);

/// @brief A plan that runs forward for `forward_length`, then reverses for `reverse_length`
/// (a cusp), sampled every `spacing`.
std::vector<vectorfield_core::Pose2> reversingPlan(
  const vectorfield_core::Pose2 & start, double forward_length, double reverse_length,
  double spacing);

/// @brief A `nav_msgs/Path` of poses in `frame_id` (z = 0, pure-yaw orientations, zero stamps).
nav_msgs::msg::Path toPathMsg(
  vectorfield_core::Span<const vectorfield_core::Pose2> poses, const std::string & frame_id);

}  // namespace test
}  // namespace vectorfield_nav2_controller

#endif  // VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__PLAN_GENERATORS_HPP_
