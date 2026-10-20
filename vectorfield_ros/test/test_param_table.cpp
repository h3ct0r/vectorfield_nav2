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

// The binding table kCommonCoreParams against vectorfield_common_params.yaml and CoreConfig
// (plan §4 "Binding tables"): integrity, default parity, round trip of a non-default value, and
// coverage of every key the gpl listener declares.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_ros/core_params.hpp"

namespace
{

using vectorfield_ros::CommonParamListener;
using vectorfield_ros::CommonParams;
using vectorfield_ros::CoreParamTarget;
using vectorfield_ros::kCommonCoreParams;

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

const ::testing::Environment * const kRclcpp =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

// Every enum string of CoreConfig, to find a valid non-default value of a string row.
const std::vector<std::string> kEnumStrings = {
  "atan",
  "rational",
  "differential",
  "skid_steer",
  "omnidirectional",
  "ackermann",
  "auto",
  "feedback_linearization",
  "heading_tracking",
  "holonomic",
  "path_tangent",
  "field",
  "fixed",
  "none",
  "contour",
  "cbf",
  "path_blocked",
  "always",
};

TEST(CommonParamTable, RowsAreCompleteAndUnique)
{
  std::set<std::string> names;
  for (const auto & row : kCommonCoreParams) {
    ASSERT_NE(row.name, nullptr);
    EXPECT_NE(row.from_params, nullptr) << row.name;
    EXPECT_NE(row.set, nullptr) << row.name;
    EXPECT_NE(row.get, nullptr) << row.name;
    EXPECT_TRUE(names.insert(row.name).second) << "duplicate row " << row.name;
  }
  EXPECT_EQ(names.size(), vectorfield_ros::kCommonCoreParamCount);
  for (const auto & ros_only : vectorfield_ros::kCommonRosOnlyParams) {
    EXPECT_EQ(names.count(std::string(ros_only)), 0U) << ros_only;
  }
  EXPECT_NE(vectorfield_ros::findRow(kCommonCoreParams, "desired_speed"), nullptr);
  EXPECT_EQ(vectorfield_ros::findRow(kCommonCoreParams, "visualize"), nullptr);
}

// The gpl defaults (YAML) equal the CoreConfig defaults (config.hpp), row by row.
TEST(CommonParamTable, DefaultParity)
{
  const CommonParams defaults;
  const CoreParamTarget target;
  for (const auto & row : kCommonCoreParams) {
    EXPECT_EQ(row.from_params(defaults), row.get(target)) << row.name;
  }
}

// Every row round-trips a non-default value, and rejects a value of the wrong type.
TEST(CommonParamTable, EveryRowRoundTripsANonDefaultValue)
{
  for (const auto & row : kCommonCoreParams) {
    const CoreParamTarget defaults;
    const rclcpp::ParameterValue default_value = row.get(defaults);
    std::vector<rclcpp::ParameterValue> candidates;
    switch (default_value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        candidates.emplace_back(default_value.get<double>() + 0.125);
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        candidates.emplace_back(!default_value.get<bool>());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        candidates.emplace_back(default_value.get<std::int64_t>() - 1);
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        for (const std::string & text : kEnumStrings) {
          if (text != default_value.get<std::string>()) {
            candidates.emplace_back(text);
          }
        }
        break;
      default:
        ADD_FAILURE() << row.name << " has an unexpected type " << default_value.get_type();
    }
    bool round_tripped = false;
    for (const rclcpp::ParameterValue & candidate : candidates) {
      CoreParamTarget target;
      if (row.set(target, candidate)) {
        EXPECT_EQ(row.get(target), candidate) << row.name;
        round_tripped = true;
        break;
      }
    }
    EXPECT_TRUE(round_tripped) << row.name << " accepted no non-default value";

    CoreParamTarget untouched;
    EXPECT_FALSE(row.set(untouched, rclcpp::ParameterValue(std::vector<std::uint8_t>{1})))
      << row.name;
    EXPECT_EQ(row.get(untouched), default_value) << row.name << " changed on a rejected set";
  }
}

TEST(CommonParamTable, EnumRowsRejectUnknownStrings)
{
  for (const auto & row : kCommonCoreParams) {
    const CoreParamTarget defaults;
    if (row.get(defaults).get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      continue;
    }
    CoreParamTarget target;
    EXPECT_FALSE(row.set(target, rclcpp::ParameterValue(std::string("no_such_value")))) << row.name;
    EXPECT_EQ(row.get(target), row.get(defaults)) << row.name;
  }
}

// The gpl defaults with the stock 3 x 3 m / 0.05 m costmap and the nav2_bringup robot_radius 0.22
// footprint pass validateConfig with no error and no warning, in both obstacle modes.
TEST(CommonParamTable, DefaultsValidateCleanly)
{
  vectorfield_core::CoreConfig config = vectorfield_ros::toCoreConfig(CommonParams());
  vectorfield_core::CostmapInfo costmap;
  costmap.width_m = 3.0;
  costmap.height_m = 3.0;
  costmap.resolution = 0.05;
  for (const auto mode :
       {vectorfield_core::ObstacleMode::kNone, vectorfield_core::ObstacleMode::kContour}) {
    config.obstacles.obstacle_mode = mode;
    const vectorfield_core::ValidationReport report =
      vectorfield_core::validateConfig(config, costmap);
    EXPECT_TRUE(report.errors.empty()) << report.summary();
    EXPECT_TRUE(report.warnings.empty()) << report.summary();
  }
}

TEST(CommonParamTable, VizConfigDefaults)
{
  const vectorfield_ros::VizConfig config = vectorfield_ros::toVizConfig(CommonParams());
  const vectorfield_ros::VizConfig expected;
  EXPECT_EQ(config.visualize, expected.visualize);
  EXPECT_EQ(config.field_grid_size, expected.field_grid_size);
  EXPECT_EQ(config.field_grid_resolution, expected.field_grid_resolution);
  EXPECT_EQ(config.field_grid_rate, expected.field_grid_rate);
  EXPECT_EQ(config.field_grid_style, expected.field_grid_style);
  EXPECT_EQ(config.publish_obstacle_distance, expected.publish_obstacle_distance);
}

// The keys the gpl listener declares under a dotted plugin prefix are exactly the table plus the
// ROS-only allow-list, and none is declared outside the prefix.
TEST(CommonParamTable, ListenerDeclaresExactlyTheTableAndTheRosOnlyKeys)
{
  auto node = std::make_shared<rclcpp::Node>("vf_param_table_test");
  const std::string prefix = "FollowPath.primary_controller";
  auto listener = std::make_shared<CommonParamListener>(node, prefix);

  std::set<std::string> expected;
  for (const auto & row : kCommonCoreParams) {
    expected.insert(prefix + "." + row.name);
  }
  for (const auto & name : vectorfield_ros::kCommonRosOnlyParams) {
    expected.insert(prefix + "." + std::string(name));
  }

  std::set<std::string> declared;
  for (const std::string & name : node->list_parameters({}, 0).names) {
    if (name.rfind(prefix + ".", 0) == 0) {
      declared.insert(name);
    }
  }
  EXPECT_EQ(declared, expected);

  // The listener's committed values equal the defaults, and motion_model is read-only.
  const vectorfield_core::CoreConfig config = vectorfield_ros::toCoreConfig(listener->get_params());
  EXPECT_EQ(config.field.desired_speed, vectorfield_core::CoreConfig{}.field.desired_speed);
  EXPECT_TRUE(node->describe_parameter(prefix + ".motion_model").read_only);
  EXPECT_FALSE(node->describe_parameter(prefix + ".desired_speed").read_only);
}

}  // namespace
