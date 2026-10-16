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

// Adapted from navigation2 nav2_controller/plugins/feasible_path_handler.cpp (branch main),
// Apache-2.0: Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov;
// Copyright (c) 2023 Dexory; Copyright (c) 2023 Open Navigation LLC. See NOTICE.

#include <memory>
#include <stdexcept>
#include <utility>

#include "vectorfield_nav2_controller/legacy_path_handler.hpp"

namespace vectorfield_nav2_controller
{

LegacyPathHandler::LegacyPathHandler(
  const LegacyPathConfig & config, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: config_(config), tf_(std::move(tf)), costmap_ros_(std::move(costmap_ros))
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: LegacyPathHandler");
}

void LegacyPathHandler::setPlan(const nav_msgs::msg::Path & /*path*/)
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: LegacyPathHandler::setPlan");
}

void LegacyPathHandler::computeWindow(
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/, LegacyPathOutput * /*out*/)
{
  // M2 wave 2: path
  throw std::logic_error("not implemented: LegacyPathHandler::computeWindow");
}

const nav_msgs::msg::Path & LegacyPathHandler::plan() const noexcept
{
  // M2 wave 2: path
  return plan_;
}

const LegacyPathConfig & LegacyPathHandler::config() const noexcept
{
  // M2 wave 2: path
  return config_;
}

}  // namespace vectorfield_nav2_controller
