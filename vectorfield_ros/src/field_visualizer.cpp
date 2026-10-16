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
#include <string>
#include <string_view>

#include "vectorfield_ros/field_visualizer.hpp"

namespace vectorfield_ros
{

FieldVisualizer::FieldVisualizer(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & /*ns*/,
  const VizConfig & config)
: logger_(node->get_logger()), clock_(node->get_clock()), config_(config)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::FieldVisualizer");
}

FieldVisualizer::~FieldVisualizer() = default;

void FieldVisualizer::on_activate()
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::FieldVisualizer::on_activate");
}

void FieldVisualizer::on_deactivate(
  const builtin_interfaces::msg::Time & /*stamp*/, std::string_view /*frame_id*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::FieldVisualizer::on_deactivate");
}

void FieldVisualizer::setConfig(const VizConfig & /*config*/)
{
  // M2 wave 2: ros-viz
  throw std::logic_error("not implemented: vectorfield_ros::FieldVisualizer::setConfig");
}

const VizConfig & FieldVisualizer::config() const noexcept
{
  // M2 wave 2: ros-viz
  return config_;
}

void FieldVisualizer::publish(const MarkerContext & /*context*/) noexcept
{
  // M2 wave 2: ros-viz
}

void FieldVisualizer::clear(
  const builtin_interfaces::msg::Time & /*stamp*/, std::string_view /*frame_id*/) noexcept
{
  // M2 wave 2: ros-viz
}

std::size_t FieldVisualizer::builderCalls() const noexcept
{
  // M2 wave 2: ros-viz
  return builder_calls_;
}

}  // namespace vectorfield_ros
