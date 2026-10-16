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

// Layer 2 (plan §9), through the server harness: the plugin loaded by pluginlib through its
// lifecycle, speed limits, dynamic parameters (gpl validators and the cross-validator), a dotted
// plugin name, and the visualization gating (plan §4.1).

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "harness_fixtures.hpp"
#include "vectorfield_ros/compat.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::test::countExceptions;
using vectorfield_nav2_controller::test::describe;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::modelOptions;
using vectorfield_nav2_controller::test::pluginParameter;
using vectorfield_nav2_controller::test::ServerHarness;
using vectorfield_nav2_controller::test::straightPlan;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

nav_msgs::msg::Path straightAhead(double length)
{
  return mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(length, 0.0), 0.05, 0.0));
}

/// Spins the harness node until `done` or a wall-clock timeout (DDS discovery and delivery only;
/// control time is never waited for).
template <typename Done>
bool spinUntil(ServerHarness & harness, Done && done, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    harness.spinSome();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}

TEST(Lifecycle, ConfigureActivateDeactivateCleanupAndAgain)
{
  ServerHarness harness(modelOptions());
  EXPECT_EQ(harness.controller(), nullptr);
  harness.configure();
  ASSERT_NE(harness.controller(), nullptr) << "pluginlib did not create our plugin";
  EXPECT_TRUE(harness.node()->has_parameter("FollowPath.desired_speed"));
  EXPECT_DOUBLE_EQ(harness.controller()->appliedConfig().field.desired_speed, 0.4);
  harness.activate();
  harness.deactivate();
  harness.activate();
  harness.deactivate();
  harness.cleanup();
  EXPECT_EQ(harness.controller(), nullptr);

  // A second configuration on the same node reuses the declared parameters.
  harness.configure();
  harness.activate();
  harness.followPath(straightAhead(0.8));
  EXPECT_EQ(harness.runUntilDone(10.0), GoalStatus::kSucceeded) << describe(harness);
  harness.deactivate();
  harness.cleanup();
}

TEST(Lifecycle, ConfigureRejectsAnInconsistentConfiguration)
{
  auto options = modelOptions();
  options.parameters.push_back(pluginParameter(options, "desired_speed", 0.8));
  ServerHarness harness(options);
  EXPECT_THROW(harness.configure(), std::invalid_argument);
  EXPECT_EQ(harness.controller(), nullptr);
}

TEST(SpeedLimit, AbsoluteAndPercentageLimitsScaleTheReferenceSpeed)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  harness.followPath(straightAhead(2.0));
  for (int i = 0; i < 20; ++i) {
    harness.spinOnce();
  }
  ASSERT_EQ(harness.status(), GoalStatus::kRunning) << describe(harness);

  // Absolute: 0.2 m/s against desired_speed 0.4.
  harness.setSpeedLimit(0.2, false);
  double peak = 0.0;
  for (int i = 0; i < 30; ++i) {
    const auto & record = harness.spinOnce();
    ASSERT_TRUE(record.computed) << describe(harness);
    EXPECT_DOUBLE_EQ(record.diagnostics.speed_limit_scale, 0.5);
    if (i >= 20) {
      peak = std::max(peak, std::abs(record.published.linear.x));
    }
  }
  EXPECT_LE(peak, 0.2 + 1e-6);
  EXPECT_GT(peak, 0.1);

  // Percentage: 25 %.
  harness.setSpeedLimit(25.0, true);
  peak = 0.0;
  for (int i = 0; i < 20; ++i) {
    const auto & record = harness.spinOnce();
    EXPECT_DOUBLE_EQ(record.diagnostics.speed_limit_scale, 0.25);
    if (i >= 10) {
      peak = std::max(peak, std::abs(record.published.linear.x));
    }
  }
  EXPECT_LE(peak, 0.1 + 1e-6);

  // NO_SPEED_LIMIT restores the full speed.
  harness.setSpeedLimit(0.0, false);
  EXPECT_DOUBLE_EQ(harness.spinOnce().diagnostics.speed_limit_scale, 1.0);
}

TEST(DynamicParameters, AcceptedChangesApplyAndRejectedOnesDoNot)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  harness.followPath(straightAhead(2.0));
  harness.spinOnce();

  // Accepted: a consistent change reaches the Tracker on the next cycle.
  auto result = harness.setParameters(
    {rclcpp::Parameter("FollowPath.desired_speed", 0.3),
     rclcpp::Parameter("FollowPath.convergence_gain", 3.0)});
  ASSERT_TRUE(result.successful) << result.reason;
  harness.spinOnce();
  EXPECT_DOUBLE_EQ(harness.controller()->appliedConfig().field.desired_speed, 0.3);
  EXPECT_DOUBLE_EQ(harness.controller()->appliedConfig().field.convergence_gain, 3.0);

  // Rejected by the cross-validator: desired_speed above max_linear_vel.
  result = harness.setParameters({rclcpp::Parameter("FollowPath.desired_speed", 0.9)});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("desired_speed"), std::string::npos) << result.reason;
  EXPECT_DOUBLE_EQ(harness.node()->get_parameter("FollowPath.desired_speed").as_double(), 0.3);

  // Rejected as a batch: one bad value rejects the whole request.
  result = harness.setParameters(
    {rclcpp::Parameter("FollowPath.max_linear_vel", 0.25),
     rclcpp::Parameter("FollowPath.heading_gain", 3.0)});
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(harness.node()->get_parameter("FollowPath.heading_gain").as_double(), 2.0);

  // Rejected by gpl: motion_model is read-only.
  result = harness.setParameters({rclcpp::Parameter("FollowPath.motion_model", "ackermann")});
  EXPECT_FALSE(result.successful);

  // Foreign keys are not the cross-validator's business.
  result = harness.setParameters({rclcpp::Parameter("FollowPath.plugin", "anything")});
  EXPECT_TRUE(result.successful) << result.reason;

  for (int i = 0; i < 5; ++i) {
    const auto & record = harness.spinOnce();
    EXPECT_FALSE(record.diagnostics.config_invalid);
  }
  EXPECT_DOUBLE_EQ(harness.controller()->appliedConfig().field.desired_speed, 0.3);
  EXPECT_DOUBLE_EQ(harness.controller()->appliedConfig().motion.max_linear_vel, 0.5);
  EXPECT_EQ(harness.runUntilDone(15.0), GoalStatus::kSucceeded) << describe(harness);
}

TEST(PluginName, ADottedNameIsAParameterPrefixAndATopicNamespace)
{
  auto options = modelOptions();
  options.plugin_name = "vf.follow_path";
  options.parameters.clear();
  options.parameters.push_back(pluginParameter(options, "desired_speed", 0.3));
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  EXPECT_TRUE(harness.node()->has_parameter("vf.follow_path.convergence_gain"));
  EXPECT_DOUBLE_EQ(harness.controller()->appliedConfig().field.desired_speed, 0.3);
  const auto node = harness.node();
  EXPECT_TRUE(spinUntil(
    harness, [&] { return node->count_publishers("/vf/follow_path/markers") > 0; },
    std::chrono::seconds(5)));
  harness.followPath(straightAhead(0.8));
  EXPECT_EQ(harness.runUntilDone(10.0), GoalStatus::kSucceeded) << describe(harness);
  EXPECT_EQ(countExceptions(harness, "NoValidControl"), 0U);
}

TEST(Visualization, WithoutSubscribersTheBuilderIsNeverCalled)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  harness.followPath(straightAhead(0.8));
  EXPECT_EQ(harness.runUntilDone(10.0), GoalStatus::kSucceeded) << describe(harness);
  std::size_t computed = 0;
  for (const auto & record : harness.history()) {
    if (record.computed) {
      ++computed;
      EXPECT_EQ(record.diagnostics.visualizer_builder_calls, 0U);
    }
  }
  EXPECT_GT(computed, 10U);
}

TEST(Visualization, WithASubscriberMarkersArePublishedEveryCycle)
{
  ServerHarness harness(modelOptions());
  harness.configure();
  harness.activate();
  std::size_t received = 0;
  const auto subscription =
    vectorfield_ros::compat::subscribe<visualization_msgs::msg::MarkerArray>(
      harness.node(), "FollowPath/markers", rclcpp::QoS(10).reliable(),
      [&received](visualization_msgs::msg::MarkerArray::ConstSharedPtr) { ++received; });
  const auto node = harness.node();
  ASSERT_TRUE(spinUntil(
    harness, [&] { return node->count_subscribers("/FollowPath/markers") > 0; },
    std::chrono::seconds(5)));

  harness.followPath(straightAhead(1.5));
  // Warm-up until the publisher has matched the subscription (discovery is asynchronous).
  std::size_t calls = 0;
  for (int i = 0; i < 40 && calls == 0; ++i) {
    calls = harness.spinOnce().diagnostics.visualizer_builder_calls;
  }
  ASSERT_GT(calls, 0U) << "the markers publisher never matched the subscription";
  ASSERT_TRUE(spinUntil(harness, [&] { return received >= 1; }, std::chrono::seconds(5)));

  // From now on: a build and a delivered markers message every cycle.
  for (int i = 0; i < 20; ++i) {
    const std::size_t before = received;
    const auto & record = harness.spinOnce();
    ASSERT_TRUE(record.computed) << describe(harness);
    EXPECT_GT(record.diagnostics.visualizer_builder_calls, calls) << "cycle " << i;
    calls = record.diagnostics.visualizer_builder_calls;
    EXPECT_TRUE(spinUntil(
      harness, [&] { return received > before; }, std::chrono::seconds(2)))
      << "no markers message for cycle " << i;
  }
}

}  // namespace
