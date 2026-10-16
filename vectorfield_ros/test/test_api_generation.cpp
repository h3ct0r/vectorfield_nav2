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

// API-generation test (plan §9 CI, nav2_api_matrix.md §2.3 and §7.4).
//
// Compile time: the probe macros match the distro this package was configured for, and the
// CMake-side VF_CMAKE_HAS_ROS_COMMON equals the header probe VF_HAS_ROS_COMMON. Run time: logs the
// detected generation in every CI job, and smoke-tests the compat helpers on a real node.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_ros/compat.hpp"

#ifndef VF_CMAKE_HAS_ROS_COMMON
#error "VF_CMAKE_HAS_ROS_COMMON must be defined (0 or 1) by the vectorfield_ros target"
#endif
#ifndef VF_TEST_EXPECTED_GENERATION
#error "VF_TEST_EXPECTED_GENERATION must be defined by CMakeLists.txt"
#endif

namespace compat = vectorfield_ros::compat;

static_assert(
  VF_CMAKE_HAS_ROS_COMMON == VF_HAS_ROS_COMMON,
  "find_package(nav2_ros_common) and __has_include(<nav2_ros_common/lifecycle_node.hpp>) disagree");

// Generation consistency (any distro).
static_assert(VF_HAS_PATH_HANDLER == VF_HAS_ROS_COMMON, "Gen C has both, Gen A/B neither");
static_assert(
  VF_HAS_PATH_HANDLER <= VF_HAS_CONTROLLER_EXCEPTIONS,
  "the path handler implies Jazzy+ exceptions");
static_assert(
  compat::kServerCallsReset == (VF_HAS_CONTROLLER_EXCEPTIONS == 1) &&
    compat::kServerPollsCancel == (VF_HAS_CONTROLLER_EXCEPTIONS == 1),
  "reset() and cancel() are server-driven exactly when the controller exceptions exist");
static_assert(
  std::is_base_of_v<compat::ControllerException, compat::NoValidControl> &&
    std::is_base_of_v<compat::ControllerException, compat::InvalidPath> &&
    std::is_base_of_v<compat::ControllerException, compat::ControllerTFError> &&
    std::is_base_of_v<std::runtime_error, compat::ControllerException>,
  "the plugin's exceptions derive from the server's base class on every distro");

// Generation of the configured distro (1 = A Humble, 2 = B Jazzy/Kilted, 3 = C Lyrical/Rolling).
#if VF_TEST_EXPECTED_GENERATION != 0
static_assert(
  compat::kApiGeneration - 'A' + 1 == VF_TEST_EXPECTED_GENERATION,
  "the API generation detected by compat.hpp does not match ROS_DISTRO");
static_assert(
  VF_HAS_CONTROLLER_EXCEPTIONS == (VF_TEST_EXPECTED_GENERATION >= 2 ? 1 : 0),
  "VF_HAS_CONTROLLER_EXCEPTIONS does not match ROS_DISTRO");
static_assert(
  VF_HAS_PATH_HANDLER == (VF_TEST_EXPECTED_GENERATION == 3 ? 1 : 0),
  "VF_HAS_PATH_HANDLER does not match ROS_DISTRO");
static_assert(
  VF_HAS_ROS_COMMON == (VF_TEST_EXPECTED_GENERATION == 3 ? 1 : 0),
  "VF_HAS_ROS_COMMON does not match ROS_DISTRO");
#endif
#if VF_TEST_EXPECTED_STAMPED >= 0
static_assert(
  compat::kStampedCmdVelDefault == (VF_TEST_EXPECTED_STAMPED == 1),
  "kStampedCmdVelDefault does not match the distro's enable_stamped_cmd_vel default");
#endif

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

const ::testing::Environment * const kRclcpp =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

TEST(ApiGeneration, LogsDetectedGeneration)
{
  const std::string generation(1, compat::kApiGeneration);
  std::cout << "[vectorfield_ros] ROS_DISTRO=" << VF_TEST_ROS_DISTRO
            << " API generation: " << compat::kApiGenerationName
            << " | VF_HAS_ROS_COMMON=" << VF_HAS_ROS_COMMON
            << " VF_HAS_PATH_HANDLER=" << VF_HAS_PATH_HANDLER
            << " VF_HAS_CONTROLLER_EXCEPTIONS=" << VF_HAS_CONTROLLER_EXCEPTIONS
            << " VF_CMAKE_HAS_ROS_COMMON=" << VF_CMAKE_HAS_ROS_COMMON
            << " kStampedCmdVelDefault=" << compat::kStampedCmdVelDefault << std::endl;
  RecordProperty("api_generation", generation);
  RecordProperty("ros_distro", VF_TEST_ROS_DISTRO);
  EXPECT_TRUE(generation == "A" || generation == "B" || generation == "C");
}

TEST(Compat, HelpersWorkOnALifecycleNode)
{
  auto node = std::make_shared<compat::LifecycleNode>("vf_compat_test");

  auto buffer = compat::makeTfBuffer(node);
  ASSERT_NE(buffer, nullptr);
  auto listener = compat::makeTfListener(*buffer, node, false);
  EXPECT_NE(listener, nullptr);
  EXPECT_NE(compat::makeTfBroadcaster(node), nullptr);
  EXPECT_NE(compat::makeStaticTfBroadcaster(node), nullptr);

  auto subscription = compat::subscribe<std_msgs::msg::Header>(
    node, "vf_compat_test_topic", rclcpp::QoS(1), [](std_msgs::msg::Header::ConstSharedPtr) {});
  EXPECT_NE(subscription, nullptr);

  auto server = compat::makeActionServer<vectorfield_msgs::action::FollowCurve>(
    node, "vf_compat_test_action", []() {}, nullptr, std::chrono::milliseconds(500), false);
  EXPECT_NE(server, nullptr);

  compat::TwistPublisher publisher(node, "vf_compat_test_cmd_vel", rclcpp::QoS(1));
  // Jazzy and later declare enable_stamped_cmd_vel with Nav2's default; Humble publishes Twist.
  if (node->has_parameter("enable_stamped_cmd_vel")) {
    EXPECT_EQ(
      node->get_parameter("enable_stamped_cmd_vel").as_bool(), compat::kStampedCmdVelDefault);
  } else {
    EXPECT_FALSE(compat::kStampedCmdVelDefault);
  }
  EXPECT_FALSE(publisher.is_activated());
  publisher.on_activate();
  EXPECT_TRUE(publisher.is_activated());
  EXPECT_EQ(publisher.get_subscription_count(), 0U);
  publisher.publish(std::make_unique<geometry_msgs::msg::TwistStamped>());
  publisher.on_deactivate();
  EXPECT_FALSE(publisher.is_activated());
}

TEST(Compat, EmulatedLatchAndNullChecker)
{
  compat::EmulatedXYLatch latch;
  EXPECT_FALSE(latch.latched());
  EXPECT_FALSE(latch.update(0.30, 0.25));
  EXPECT_TRUE(latch.update(0.25, 0.25));
  EXPECT_TRUE(latch.update(1.00, 0.25)) << "the latch stays set until rearm()";
  latch.rearm();
  EXPECT_FALSE(latch.latched());
  EXPECT_FALSE(latch.update(std::nan(""), 0.25));

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.2;
  const geometry_msgs::msg::Twist velocity;
  const nav_msgs::msg::Path plan;
  // A null checker falls back to the latch on every distro.
  EXPECT_TRUE(compat::goal_xy_reached(nullptr, latch, pose, goal, velocity, plan, 0.25));
  goal.pose.position.x = 5.0;
  EXPECT_TRUE(compat::goal_xy_reached(nullptr, latch, pose, goal, velocity, plan, 0.25));
  latch.rearm();
  EXPECT_FALSE(compat::goal_xy_reached(nullptr, latch, pose, goal, velocity, plan, 0.25));

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  EXPECT_FALSE(compat::get_goal_tolerances(nullptr, pose_tolerance, velocity_tolerance));
}

}  // namespace
