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

#include <stdexcept>
#include <string_view>

#include "vectorfield_ros/marker_builder.hpp"

namespace vectorfield_ros
{

MarkerBuilder::MarkerBuilder()
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::MarkerBuilder");
}

const visualization_msgs::msg::MarkerArray & MarkerBuilder::markers(
  const MarkerContext & /*context*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::MarkerBuilder::markers");
}

const visualization_msgs::msg::MarkerArray & MarkerBuilder::fieldGrid(
  const MarkerContext & /*context*/, const VizConfig & /*config*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::MarkerBuilder::fieldGrid");
}

const nav_msgs::msg::Path & MarkerBuilder::window(const MarkerContext & /*context*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::MarkerBuilder::window");
}

const nav_msgs::msg::Path & MarkerBuilder::predictedPath(const MarkerContext & /*context*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::MarkerBuilder::predictedPath");
}

const nav_msgs::msg::OccupancyGrid & MarkerBuilder::obstacleDistance(
  const MarkerContext & /*context*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::MarkerBuilder::obstacleDistance");
}

const visualization_msgs::msg::MarkerArray & MarkerBuilder::deleteAll(
  const builtin_interfaces::msg::Time & /*stamp*/, std::string_view /*frame_id*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::MarkerBuilder::deleteAll");
}

}  // namespace vectorfield_ros
