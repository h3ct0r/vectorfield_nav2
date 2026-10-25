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
/// @brief Shared pieces of the layer-2 gtests: the ROS environment,
/// the default harness options per motion model and small queries over a run. Header-only; it
/// needs gtest, so it is not part of the support library. Distro-free.

#ifndef VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__HARNESS_FIXTURES_HPP_
#define VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__HARNESS_FIXTURES_HPP_

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "plan_generators.hpp"
#include "rclcpp/rclcpp.hpp"
#include "server_harness.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_nav2_controller
{
namespace test
{

/// @brief rclcpp::init() / shutdown() once per test binary (register with
/// `::testing::AddGlobalTestEnvironment`).
class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

/// @brief A plugin parameter (`<plugin_name>.<key>`).
template <typename ValueT>
rclcpp::Parameter pluginParameter(
  const HarnessOptions & options, const std::string & key, ValueT value)
{
  return rclcpp::Parameter(options.plugin_name + "." + key, std::move(value));
}

/// @brief Default options for a motion model ("differential", "skid_steer", "omnidirectional",
/// "ackermann") and a tracking law ("auto" or a law name): a 4 m costmap centred on (0.5, 0), the
/// robot at the odom origin facing +x, map = odom, and a plant of the same model.
inline HarnessOptions modelOptions(
  const std::string & motion_model = "differential", const std::string & tracking_law = "auto")
{
  HarnessOptions options;
  options.costmap_size = 4.0;
  options.costmap_center = vectorfield_core::Vec2(0.5, 0.0);
  options.parameters.push_back(pluginParameter(options, "motion_model", motion_model));
  options.parameters.push_back(pluginParameter(options, "tracking_law", tracking_law));
  vectorfield_core::MotionModel model = vectorfield_core::MotionModel::kDifferential;
  vectorfield_core::fromString(motion_model, &model);
  options.plant.type = model;
  if (model == vectorfield_core::MotionModel::kAckermann) {
    options.parameters.push_back(pluginParameter(options, "min_turning_radius", 0.5));
    options.plant.min_turning_radius = 0.5;
  }
  return options;
}

/// @brief A plan in the map frame from poses.
inline nav_msgs::msg::Path mapPlan(const std::vector<vectorfield_core::Pose2> & poses)
{
  return toPathMsg(poses, "map");
}

/// @brief Status, reason and message of the goal, for failure output.
inline std::string describe(const ServerHarness & harness)
{
  std::ostringstream text;
  const GoalResult & result = harness.result();
  text << "status " << static_cast<int>(result.status) << " reason '" << result.reason
       << "' message '" << result.message << "' after " << harness.history().size() << " cycles";
  return text.str();
}

/// @brief Number of cycles whose compute threw `name`.
inline std::size_t countExceptions(const ServerHarness & harness, const std::string & name)
{
  const std::vector<CycleRecord> & history = harness.history();
  return static_cast<std::size_t>(std::count_if(
    history.begin(), history.end(), [&](const CycleRecord & r) { return r.exception == name; }));
}

/// @brief Planar distance of the robot (odom frame) from a point at the end of the run.
inline double finalDistance(const ServerHarness & harness, const vectorfield_core::Vec2 & point)
{
  return (harness.history().back().pose_odom.p - point).norm();
}

}  // namespace test
}  // namespace vectorfield_nav2_controller

#endif  // VECTORFIELD_NAV2_CONTROLLER__TEST__SUPPORT__HARNESS_FIXTURES_HPP_
