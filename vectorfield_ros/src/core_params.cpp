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

// Node-level parameter handling: node-derived fields, validateParams(), validateFromNode() and
// the CrossValidator (plan §4). The binding table itself is in core_params_table.cpp.

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "vectorfield_ros/core_params.hpp"

namespace vectorfield_ros
{

void setNodeDerivedFields(
  double /*controller_frequency*/,
  const std::vector<geometry_msgs::msg::Point> & /*robot_footprint*/,
  vectorfield_core::CoreConfig * /*config*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::setNodeDerivedFields");
}

double controllerFrequencyOf(
  const rclcpp::node_interfaces::NodeParametersInterface & /*parameters*/, double /*fallback*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::controllerFrequencyOf");
}

vectorfield_core::ValidationReport validateParams(
  const CommonParams & /*params*/, double /*controller_frequency*/,
  const std::optional<vectorfield_core::CostmapInfo> & /*costmap*/,
  const std::vector<geometry_msgs::msg::Point> & /*robot_footprint*/,
  vectorfield_core::CoreConfig * /*config*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::validateParams");
}

vectorfield_core::ValidationReport validateFromNode(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & /*parameters*/,
  const rclcpp::Logger & /*logger*/, const std::string & /*plugin_name*/,
  const vectorfield_core::CostmapInfo & /*costmap*/,
  const std::vector<geometry_msgs::msg::Point> & /*robot_footprint*/,
  vectorfield_core::CoreConfig * /*config*/)
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::validateFromNode");
}

CrossValidator::CrossValidator(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
  const std::string & prefix, std::vector<ParamSetter> setters,
  std::optional<vectorfield_core::CostmapInfo> costmap, BaseProvider base, rclcpp::Logger logger,
  ExtraCheck extra)
: parameters_(std::move(parameters)),
  prefix_(prefix),
  setters_(std::move(setters)),
  costmap_(std::move(costmap)),
  base_(std::move(base)),
  extra_(std::move(extra)),
  logger_(std::move(logger))
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::CrossValidator");
}

CrossValidator::~CrossValidator() = default;

rcl_interfaces::msg::SetParametersResult CrossValidator::validate(
  const std::vector<rclcpp::Parameter> & /*parameters*/) const
{
  // M2 wave 2: ros-io
  throw std::logic_error("not implemented: vectorfield_ros::CrossValidator::validate");
}

const std::string & CrossValidator::prefix() const noexcept
{
  // M2 wave 2: ros-io
  return prefix_;
}

}  // namespace vectorfield_ros
