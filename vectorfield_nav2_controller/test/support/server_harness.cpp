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

#include "server_harness.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace vectorfield_nav2_controller
{
namespace test
{

// M2 wave 2: harness -- the private state of the harness (nodes, costmap, loaders, plant).
struct ServerHarness::Impl
{
};

ServerHarness::ServerHarness(HarnessOptions /*options*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness");
}

ServerHarness::~ServerHarness() = default;

void ServerHarness::configure()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::configure");
}

void ServerHarness::activate()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::activate");
}

void ServerHarness::deactivate()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::deactivate");
}

void ServerHarness::cleanup()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::cleanup");
}

void ServerHarness::followPath(const nav_msgs::msg::Path & /*plan*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::followPath");
}

void ServerHarness::replan(const nav_msgs::msg::Path & /*plan*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::replan");
}

void ServerHarness::requestCancel()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::requestCancel");
}

GoalStatus ServerHarness::status() const noexcept
{
  // M2 wave 2: harness
  return GoalStatus::kNone;
}

const GoalResult & ServerHarness::result() const noexcept
{
  // M2 wave 2: harness
  static const GoalResult kNoResult;
  return kNoResult;
}

const CycleRecord & ServerHarness::spinOnce()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::spinOnce");
}

GoalStatus ServerHarness::runUntilDone(
  double /*timeout*/, const std::function<void(const CycleRecord &)> & /*on_cycle*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::runUntilDone");
}

const std::vector<CycleRecord> & ServerHarness::history() const noexcept
{
  // M2 wave 2: harness
  static const std::vector<CycleRecord> kNoHistory;
  return kNoHistory;
}

double ServerHarness::now() const noexcept
{
  // M2 wave 2: harness
  return 0.0;
}

void ServerHarness::spinSome()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::spinSome");
}

void ServerHarness::setMapToOdom(const vectorfield_core::Pose2 & /*map_to_odom*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setMapToOdom");
}

void ServerHarness::setGoalSkew(const vectorfield_core::Pose2 & /*delta*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setGoalSkew");
}

void ServerHarness::setRobotPose(const vectorfield_core::Pose2 & /*pose_odom*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setRobotPose");
}

void ServerHarness::setLethalBox(
  const vectorfield_core::Vec2 & /*min_corner*/, const vectorfield_core::Vec2 & /*max_corner*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setLethalBox");
}

void ServerHarness::setLethalDisc(const vectorfield_core::Vec2 & /*center*/, double /*radius*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setLethalDisc");
}

void ServerHarness::clearObstacles()
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::clearObstacles");
}

void ServerHarness::setFootprint(const std::vector<geometry_msgs::msg::Point> & /*polygon*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setFootprint");
}

void ServerHarness::setSpeedLimit(double /*speed_limit*/, bool /*percentage*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setSpeedLimit");
}

rcl_interfaces::msg::SetParametersResult ServerHarness::setParameters(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::setParameters");
}

VectorFieldController * ServerHarness::controller() noexcept
{
  // M2 wave 2: harness
  return nullptr;
}

nav2_core::Controller * ServerHarness::activeController() noexcept
{
  // M2 wave 2: harness
  return nullptr;
}

vectorfield_ros::compat::NodeSharedPtr ServerHarness::node() const
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::node");
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ServerHarness::costmap() const
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::costmap");
}

std::shared_ptr<tf2_ros::Buffer> ServerHarness::tf() const
{
  // M2 wave 2: harness
  throw std::logic_error("not implemented: ServerHarness::tf");
}

nav2_core::GoalChecker * ServerHarness::goalChecker() noexcept
{
  // M2 wave 2: harness
  return nullptr;
}

const HarnessOptions & ServerHarness::options() const noexcept
{
  // M2 wave 2: harness
  static const HarnessOptions kDefaults;
  return kDefaults;
}

}  // namespace test
}  // namespace vectorfield_nav2_controller
