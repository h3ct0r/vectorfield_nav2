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

#include <memory>
#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "vectorfield_nav2_controller/vectorfield_controller.hpp"

namespace vectorfield_nav2_controller
{

// M2 wave 2: plugin -- the private state of the plugin (see the header's Impl comment).
struct VectorFieldController::Impl
{
};

VectorFieldController::VectorFieldController() : impl_(std::make_unique<Impl>()) {}

VectorFieldController::~VectorFieldController() = default;

void VectorFieldController::configure(
  const vectorfield_ros::compat::NodeWeakPtr & /*parent*/, std::string /*name*/,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::configure");
}

void VectorFieldController::cleanup()
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::cleanup");
}

void VectorFieldController::activate()
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::activate");
}

void VectorFieldController::deactivate()
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::deactivate");
}

#if VF_HAS_PATH_HANDLER  // Gen C

void VectorFieldController::newPathReceived(const nav_msgs::msg::Path & /*raw_global_path*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::newPathReceived");
}

geometry_msgs::msg::TwistStamped VectorFieldController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*pose*/, const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/,
  const nav_msgs::msg::Path & /*transformed_global_plan*/,
  const geometry_msgs::msg::PoseStamped & /*global_goal*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::computeVelocityCommands");
}

#else  // Gen A/B

void VectorFieldController::setPlan(const nav_msgs::msg::Path & /*path*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::setPlan");
}

geometry_msgs::msg::TwistStamped VectorFieldController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*pose*/, const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::computeVelocityCommands");
}

#endif

void VectorFieldController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::setSpeedLimit");
}

bool VectorFieldController::cancel()
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::cancel");
}

void VectorFieldController::reset()
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::reset");
}

CycleDiagnostics VectorFieldController::lastCycle() const
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::lastCycle");
}

vectorfield_core::CoreConfig VectorFieldController::appliedConfig() const
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::appliedConfig");
}

geometry_msgs::msg::TwistStamped VectorFieldController::computeCommandImpl(
  const geometry_msgs::msg::PoseStamped & /*pose*/, const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/, const nav_msgs::msg::Path & /*window*/,
  const geometry_msgs::msg::PoseStamped & /*goal*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::computeCommandImpl");
}

void VectorFieldController::processRawPlan(const nav_msgs::msg::Path & /*raw_plan*/)
{
  // M2 wave 2: plugin
  throw std::logic_error("not implemented: VectorFieldController::processRawPlan");
}

}  // namespace vectorfield_nav2_controller

PLUGINLIB_EXPORT_CLASS(vectorfield_nav2_controller::VectorFieldController, nav2_core::Controller)
