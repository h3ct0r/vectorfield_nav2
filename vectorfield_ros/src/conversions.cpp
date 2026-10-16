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
#include <optional>
#include <stdexcept>
#include <vector>

#include "vectorfield_ros/conversions.hpp"

namespace vectorfield_ros
{

vectorfield_core::Vec2 toVec2(const geometry_msgs::msg::Point & /*point*/) noexcept
{
  // M2 wave 2: ros-io
  return vectorfield_core::Vec2::Zero();
}

vectorfield_core::Vec2 toVec2(const geometry_msgs::msg::Point32 & /*point*/) noexcept
{
  // M2 wave 2: ros-io
  return vectorfield_core::Vec2::Zero();
}

vectorfield_core::Vec2 toVec2(const geometry_msgs::msg::Vector3 & /*vector*/) noexcept
{
  // M2 wave 2: ros-io
  return vectorfield_core::Vec2::Zero();
}

geometry_msgs::msg::Point toPointMsg(
  const vectorfield_core::Vec2 & /*point*/, double /*z*/) noexcept
{
  // M2 wave 2: ros-io
  return geometry_msgs::msg::Point{};
}

geometry_msgs::msg::Vector3 toVector3Msg(const vectorfield_core::Vec2 & /*vector*/) noexcept
{
  // M2 wave 2: ros-io
  return geometry_msgs::msg::Vector3{};
}

double yawOf(const geometry_msgs::msg::Quaternion & /*orientation*/) noexcept
{
  // M2 wave 2: ros-io
  return vectorfield_core::kNaN;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double /*yaw*/) noexcept
{
  // M2 wave 2: ros-io
  return geometry_msgs::msg::Quaternion{};
}

vectorfield_core::Pose2 toPose2(const geometry_msgs::msg::Pose & /*pose*/) noexcept
{
  // M2 wave 2: ros-io
  return vectorfield_core::Pose2{};
}

geometry_msgs::msg::Pose toPoseMsg(const vectorfield_core::Pose2 & /*pose*/) noexcept
{
  // M2 wave 2: ros-io
  return geometry_msgs::msg::Pose{};
}

vectorfield_core::Twist2 toTwist2(const geometry_msgs::msg::Twist & /*twist*/) noexcept
{
  // M2 wave 2: ros-io
  return vectorfield_core::Twist2{};
}

geometry_msgs::msg::Twist toTwistMsg(const vectorfield_core::Twist2 & /*twist*/) noexcept
{
  // M2 wave 2: ros-io
  return geometry_msgs::msg::Twist{};
}

vectorfield_core::Isometry2 toIsometry2(
  const geometry_msgs::msg::Transform & /*transform*/) noexcept
{
  // M2 wave 2: ros-io
  return vectorfield_core::Isometry2{};
}

void toPoses(const nav_msgs::msg::Path & /*path*/, std::vector<vectorfield_core::Pose2> * /*out*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toPoses");
}

void toPositions(
  const nav_msgs::msg::Path & /*path*/, std::vector<vectorfield_core::Vec2> * /*out*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toPositions");
}

vectorfield_core::CurveSpec toCurveSpec(const vectorfield_msgs::msg::Curve & /*msg*/, double /*t0*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toCurveSpec");
}

vectorfield_msgs::msg::Curve toCurveMsg(
  const vectorfield_core::CurveSpec & /*spec*/, const std_msgs::msg::Header & /*header*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toCurveMsg");
}

bool toOccupancyGrid(
  nav2_costmap_2d::Costmap2D & /*costmap*/, bool /*unknown_is_obstacle*/,
  std::vector<std::uint8_t> * /*mask*/, vectorfield_core::OccupancyGrid * /*grid*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toOccupancyGrid(Costmap2D)");
}

bool toOccupancyGrid(
  const nav2_msgs::msg::Costmap & /*costmap*/, bool /*unknown_is_obstacle*/,
  std::vector<std::uint8_t> * /*mask*/, vectorfield_core::OccupancyGrid * /*grid*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toOccupancyGrid(Costmap msg)");
}

vectorfield_core::CostmapInfo toCostmapInfo(
  const nav2_costmap_2d::Costmap2D & /*costmap*/, std::optional<double> /*window_limit*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toCostmapInfo(Costmap2D)");
}

vectorfield_core::CostmapInfo toCostmapInfo(
  const nav2_msgs::msg::CostmapMetaData & /*metadata*/, std::optional<double> /*window_limit*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toCostmapInfo(CostmapMetaData)");
}

vectorfield_core::FootprintSpec toFootprintSpec(
  const std::vector<geometry_msgs::msg::Point> & /*polygon*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toFootprintSpec");
}

void toVertices(
  const std::vector<geometry_msgs::msg::Point> & /*polygon*/,
  std::vector<vectorfield_core::Vec2> * /*out*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::toVertices");
}

bool footprintChanged(
  vectorfield_core::Span<const vectorfield_core::Vec2> /*previous*/,
  vectorfield_core::Span<const vectorfield_core::Vec2> /*current*/, double /*tolerance*/) noexcept
{
  // M2 wave 2: ros-io
  return false;
}

GoalTolerances toGoalTolerances(const geometry_msgs::msg::Pose & /*pose_tolerance*/) noexcept
{
  // M2 wave 2: ros-io
  return GoalTolerances{};
}

}  // namespace vectorfield_ros
