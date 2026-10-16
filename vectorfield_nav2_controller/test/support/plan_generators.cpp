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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "vectorfield_ros/conversions.hpp"

namespace vectorfield_nav2_controller
{
namespace test
{

namespace vf = vectorfield_core;

std::vector<vf::Pose2> straightPlan(
  const vf::Vec2 & start, const vf::Vec2 & end, double spacing, double end_yaw)
{
  const vf::Vec2 delta = end - start;
  const double length = delta.norm();
  if (!(length > 0.0)) {
    return {vf::Pose2{end, end_yaw}};
  }
  const double yaw = vf::angleOf(delta);
  const auto count =
    static_cast<std::size_t>(std::max(1.0, std::ceil(length / std::max(spacing, 1e-6))));
  std::vector<vf::Pose2> poses;
  poses.reserve(count + 1);
  for (std::size_t i = 0; i <= count; ++i) {
    const double f = static_cast<double>(i) / static_cast<double>(count);
    poses.push_back(vf::Pose2{start + f * delta, yaw});
  }
  poses.back().p = end;
  poses.back().yaw = end_yaw;
  return poses;
}

std::vector<vf::Pose2> navfnTail(
  const vf::Pose2 & goal, const vf::Vec2 & goal_offset, double heading, double approach_length,
  double resolution)
{
  // NavFn walks the gradient in steps of half a cell (pathStep = 0.5) and ends at the centre of
  // the goal cell; the exact goal then replaces the last pose or is appended after it
  // (smoothApproachToGoal).
  const vf::Vec2 cell_centre = goal.p - goal_offset * resolution;
  const vf::Vec2 direction = vf::heading(heading);
  const double step = 0.5 * resolution;
  const auto steps = static_cast<std::size_t>(std::floor(approach_length / step + 1e-9));
  const vf::Vec2 start = cell_centre - approach_length * direction;
  std::vector<vf::Pose2> poses;
  poses.reserve(steps + 3);
  for (std::size_t i = 0; i <= steps; ++i) {
    poses.push_back(vf::Pose2{start + static_cast<double>(i) * step * direction, heading});
  }
  if ((poses.back().p - cell_centre).norm() > 1e-9) {
    poses.push_back(vf::Pose2{cell_centre, heading});
  }
  if (poses.size() >= 2) {
    const vf::Vec2 & second_to_last = poses[poses.size() - 2].p;
    if ((poses.back().p - second_to_last).squaredNorm() > (goal.p - second_to_last).squaredNorm()) {
      poses.back() = goal;
      return poses;
    }
  }
  poses.push_back(goal);
  return poses;
}

std::vector<vf::Pose2> reversingPlan(
  const vf::Pose2 & start, double forward_length, double reverse_length, double spacing)
{
  // The robot keeps its heading while it backs up (a reversing segment, as Smac emits it).
  const vf::Vec2 direction = vf::heading(start.yaw);
  const double ds = std::max(spacing, 1e-6);
  std::vector<vf::Pose2> poses;
  const auto forward = static_cast<std::size_t>(std::max(1.0, std::ceil(forward_length / ds)));
  for (std::size_t i = 0; i <= forward; ++i) {
    const double s = forward_length * static_cast<double>(i) / static_cast<double>(forward);
    poses.push_back(vf::Pose2{start.p + s * direction, start.yaw});
  }
  const vf::Vec2 turn = poses.back().p;
  const auto reverse = static_cast<std::size_t>(std::max(1.0, std::ceil(reverse_length / ds)));
  for (std::size_t i = 1; i <= reverse; ++i) {
    const double s = reverse_length * static_cast<double>(i) / static_cast<double>(reverse);
    poses.push_back(vf::Pose2{turn - s * direction, start.yaw});
  }
  return poses;
}

nav_msgs::msg::Path toPathMsg(vf::Span<const vf::Pose2> poses, const std::string & frame_id)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.poses.reserve(poses.size());
  for (const vf::Pose2 & pose : poses) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = frame_id;
    stamped.pose = vectorfield_ros::toPoseMsg(pose);
    path.poses.push_back(stamped);
  }
  return path;
}

}  // namespace test
}  // namespace vectorfield_nav2_controller
