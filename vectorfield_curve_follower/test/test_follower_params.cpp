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

// The follower's parameter tables (plan §4 "Binding tables", §9 layer 2b): default parity and
// coverage over the union of the tables the follower owns (kCommonCoreParams and
// kFollowerCoreParams) and its ROS-only allow-list, on the gpl structs and on a configured
// CurveFollower. Parameters that libraries declare on the node (use_sim_time, the TwistPublisher's
// enable_stamped_cmd_vel, diagnostic_updater.*, the Nav2 lifecycle node's autostart, bond and
// parameter-loading options) are not the follower's.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_curve_follower/curve_follower.hpp"
#include "vectorfield_curve_follower/follower_params.hpp"
#include "vectorfield_ros/core_params.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;
using vectorfield_curve_follower::CurveFollower;
using vectorfield_curve_follower::FollowerParams;
using vectorfield_curve_follower::kFollowerCoreParams;
using vectorfield_curve_follower::kFollowerRosOnlyParams;
using vr::kCommonCoreParams;
using vr::kCommonRosOnlyParams;

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

const ::testing::Environment * const kRclcpp =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

/// Every name the follower owns: both tables and both ROS-only lists.
std::set<std::string> followerKeys()
{
  std::set<std::string> keys;
  for (const auto & row : kCommonCoreParams) {
    keys.insert(row.name);
  }
  for (const auto & row : kFollowerCoreParams) {
    keys.insert(row.name);
  }
  for (const std::string_view name : kCommonRosOnlyParams) {
    keys.emplace(name);
  }
  for (const std::string_view name : kFollowerRosOnlyParams) {
    keys.emplace(name);
  }
  return keys;
}

/// Declared on the node by rclcpp, the Nav2 lifecycle node (autostart, the bond, and on Lyrical
/// nav2::LifecycleNode's parameter-loading options), the TwistPublisher or diagnostic_updater.
bool isLibraryParameter(const std::string & name)
{
  static const std::set<std::string> kLibrary = {
    "use_sim_time",         "start_type_description_service",
    "autostart_node",       "enable_stamped_cmd_vel",
    "introspection_mode",   "allow_parameter_qos_overrides",
    "strict_param_loading", "warn_on_missing_params",
  };
  return kLibrary.count(name) > 0 || name.rfind("diagnostic_updater.", 0) == 0 ||
         name.rfind("qos_overrides.", 0) == 0 || name.find("bond_") != std::string::npos;
}

TEST(FollowerParamTables, RowsAreCompleteUniqueAndDisjointFromTheRosOnlyKeys)
{
  std::set<std::string> names;
  for (const auto & row : kCommonCoreParams) {
    EXPECT_TRUE(names.insert(row.name).second) << "duplicate row " << row.name;
  }
  for (const auto & row : kFollowerCoreParams) {
    ASSERT_NE(row.name, nullptr);
    EXPECT_NE(row.from_params, nullptr) << row.name;
    EXPECT_NE(row.set, nullptr) << row.name;
    EXPECT_NE(row.get, nullptr) << row.name;
    EXPECT_TRUE(names.insert(row.name).second) << "duplicate row " << row.name;
  }
  std::set<std::string> ros_only;
  for (const std::string_view name : kFollowerRosOnlyParams) {
    EXPECT_TRUE(ros_only.emplace(name).second) << "duplicate ROS-only key " << name;
  }
  for (const std::string_view name : kCommonRosOnlyParams) {
    EXPECT_TRUE(ros_only.emplace(name).second) << "duplicate ROS-only key " << name;
  }
  for (const std::string & name : ros_only) {
    EXPECT_EQ(names.count(name), 0U) << name << " is both a row and ROS-only";
  }
  EXPECT_EQ(
    names.size(), vr::kCommonCoreParamCount + vectorfield_curve_follower::kFollowerCoreParamCount);
}

// The gpl defaults (both YAMLs) equal the core defaults (CoreConfig, CurveOptions), row by row.
TEST(FollowerParamTables, DefaultParity)
{
  const vr::CommonParams common;
  const FollowerParams follower;
  const vr::CoreParamTarget target;
  for (const auto & row : kCommonCoreParams) {
    EXPECT_EQ(row.from_params(common), row.get(target)) << row.name;
  }
  for (const auto & row : kFollowerCoreParams) {
    EXPECT_EQ(row.from_params(follower), row.get(target)) << row.name;
  }
}

// Every follower row round-trips a non-default value and rejects a value of the wrong type
// (integer rows also reject a value that does not fit an int).
TEST(FollowerParamTables, EveryFollowerRowRoundTripsANonDefaultValue)
{
  const FollowerParams defaults;
  for (const auto & row : kFollowerCoreParams) {
    SCOPED_TRACE(row.name);
    const auto target = std::make_unique<vr::CoreParamTarget>();
    const rclcpp::ParameterValue value = row.from_params(defaults);
    rclcpp::ParameterValue changed;
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      changed = rclcpp::ParameterValue(value.get<double>() + 0.25);
    } else {
      ASSERT_EQ(value.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
      changed = rclcpp::ParameterValue(value.get<std::int64_t>() + 1);
      EXPECT_FALSE(row.set(*target, rclcpp::ParameterValue(std::int64_t{1} << 40)));
      EXPECT_FALSE(row.set(*target, rclcpp::ParameterValue(1.5)));
    }
    ASSERT_NE(changed, row.get(*target));
    EXPECT_TRUE(row.set(*target, changed));
    EXPECT_EQ(row.get(*target), changed);
    EXPECT_FALSE(row.set(*target, rclcpp::ParameterValue(std::string("x"))));
    EXPECT_EQ(row.get(*target), changed) << "a rejected value changed the target";
  }
}

TEST(FollowerParamTables, DefaultsValidateCleanly)
{
  const vr::CommonParams common;
  const FollowerParams follower;
  vr::CoreParamTarget target;
  std::string failed;
  ASSERT_TRUE(vectorfield_curve_follower::toFollowerTarget(common, follower, {}, &target, &failed))
    << failed;
  EXPECT_TRUE(vf::validateConfig(target.config, std::nullopt).ok());
  EXPECT_TRUE(vf::validateCurveOptions(target.curve_options).ok());
}

// A configured CurveFollower declares exactly the follower's keys (plus the libraries'), each
// with the core default.
TEST(FollowerParamTables, ConfiguredNodeDeclaresExactlyTheFollowerKeysWithCoreDefaults)
{
  auto follower = std::make_shared<CurveFollower>();
  ASSERT_EQ(follower->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const std::set<std::string> expected = followerKeys();
  std::set<std::string> declared;
  for (const std::string & name : follower->list_parameters({}, 0).names) {
    if (!isLibraryParameter(name)) {
      declared.insert(name);
      EXPECT_EQ(expected.count(name), 1U) << "declared but not the follower's: " << name;
    }
  }
  for (const std::string & name : expected) {
    EXPECT_EQ(declared.count(name), 1U) << "not declared: " << name;
  }

  const vr::CoreParamTarget defaults;
  for (const auto & row : kCommonCoreParams) {
    EXPECT_EQ(follower->get_parameter(row.name).get_parameter_value(), row.get(defaults))
      << row.name;
  }
  for (const auto & row : kFollowerCoreParams) {
    EXPECT_EQ(follower->get_parameter(row.name).get_parameter_value(), row.get(defaults))
      << row.name;
  }
  EXPECT_EQ(follower->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

}  // namespace
