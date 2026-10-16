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
/// @brief Fixture of the in-process node tests: a paused KinematicSimNode (real_time_factor 0,
/// stepped by the test) and a probe node that records everything the simulator publishes.

#ifndef SIM_NODE_TEST_SUPPORT_HPP_
#define SIM_NODE_TEST_SUPPORT_HPP_

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_sim/kinematic_sim_node.hpp"

namespace vf_sim_test
{

namespace vf = vectorfield_core;
using namespace std::chrono_literals;

/// @brief Simulated time of a stamp (ns).
inline std::int64_t toNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
}

/// @brief Planar pose of a transform.
inline vf::Pose2 toPose2(const geometry_msgs::msg::Transform & transform)
{
  const auto & q = transform.rotation;
  return vf::Pose2{
    vf::Vec2(transform.translation.x, transform.translation.y),
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

/// @brief Planar pose of an odometry message.
inline vf::Pose2 toPose2(const nav_msgs::msg::Odometry & odometry)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = odometry.pose.pose.position.x;
  transform.translation.y = odometry.pose.pose.position.y;
  transform.rotation = odometry.pose.pose.orientation;
  return toPose2(transform);
}

/// @brief Expects two poses to agree within `tol` (yaw compared modulo 2π).
inline void expectPoseNear(const vf::Pose2 & actual, const vf::Pose2 & expected, double tol)
{
  EXPECT_NEAR(actual.p.x(), expected.p.x(), tol);
  EXPECT_NEAR(actual.p.y(), expected.p.y(), tol);
  EXPECT_NEAR(vf::wrapAngle(actual.yaw - expected.yaw), 0.0, tol);
}

class SimNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void TearDown() override
  {
    executor_.reset();
    twist_pub_.reset();
    twist_stamped_pub_.reset();
    subscriptions_.clear();
    probe_.reset();
    sim_.reset();
  }

  /// @brief Starts the simulator with `overrides` and the probe; `paused` adds
  /// `real_time_factor: 0` (the test steps the simulator).
  void start(std::vector<rclcpp::Parameter> overrides = {}, bool paused = true)
  {
    if (paused) {
      overrides.emplace_back("real_time_factor", 0.0);
    }
    sim_ = std::make_shared<vectorfield_sim::KinematicSimNode>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
    step_ns_ =
      static_cast<std::int64_t>(std::llround(sim_->get_parameter("step_size").as_double() * 1e9));
    stamped_ = sim_->get_parameter("enable_stamped_cmd_vel").as_bool();
    const auto cmd_vel_topic = sim_->get_parameter("cmd_vel_topic").as_string();

    probe_ = std::make_shared<rclcpp::Node>("vf_sim_probe");
    if (stamped_) {
      twist_stamped_pub_ =
        probe_->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);
    } else {
      twist_pub_ = probe_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    }
    record<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10), &odom_);
    record<nav_msgs::msg::Odometry>("/vectorfield_sim/ground_truth", rclcpp::QoS(10), &truth_);
    record<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), &scan_);
    subscriptions_.push_back(probe_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(100), [this](const rosgraph_msgs::msg::Clock & message) {
        clocks_.push_back(toNs(message.clock));
      }));
    subscriptions_.push_back(probe_->create_subscription<std_msgs::msg::UInt64>(
      "/vectorfield_sim/infeasible_command_count", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::UInt64 & message) {
        infeasible_ = message.data;
        has_infeasible_ = true;
      }));
    subscriptions_.push_back(probe_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100), [this](const tf2_msgs::msg::TFMessage & message) {
        for (const auto & transform : message.transforms) {
          if (transform.child_frame_id == "base_footprint") {
            odom_tf_ = transform;
          }
        }
      }));
    subscriptions_.push_back(probe_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local(),
      [this](const tf2_msgs::msg::TFMessage & message) {
        for (const auto & transform : message.transforms) {
          if (transform.child_frame_id == "odom") {
            map_tf_ = transform;
            ++map_tf_count_;
          }
        }
      }));

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(sim_);
    executor_->add_node(probe_);
    ASSERT_TRUE(spinUntil([this]() { return map_tf_count_ > 0 && has_infeasible_; }));
  }

  /// @brief Spins both nodes until `done` or `timeout`.
  bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout = 10s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      executor_->spin_some(5ms);
    }
    return true;
  }

  /// @brief Simulated time after the steps taken so far (ns).
  std::int64_t timeNs() const { return static_cast<std::int64_t>(steps_) * step_ns_; }

  /// @brief Steps the simulator once and waits for that step's odometry and clock.
  void stepAndWait()
  {
    sim_->step();
    ++steps_;
    const std::int64_t expected = timeNs();
    ASSERT_TRUE(spinUntil([this, expected]() {
      return toNs(odom_.header.stamp) == expected && !clocks_.empty() && clocks_.back() == expected;
    }))
      << "no odometry or clock for t = " << expected << " ns";
  }

  /// @brief Publishes `command` with the type the simulator subscribes to.
  void publish(const vf::Twist2 & command)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.vx;
    twist.linear.y = command.vy;
    twist.angular.z = command.wz;
    if (stamped_) {
      geometry_msgs::msg::TwistStamped message;
      message.header.frame_id = "base_footprint";
      message.twist = twist;
      twist_stamped_pub_->publish(message);
    } else {
      twist_pub_->publish(twist);
    }
  }

  /// @brief Publishes `command` until a step executes it; the steps before it had no command and
  /// left the robot where it was.
  void activate(const vf::Twist2 & command)
  {
    for (int attempt = 0; attempt < 200; ++attempt) {
      publish(command);
      executor_->spin_some(10ms);
      stepAndWait();
      const auto & twist = odom_.twist.twist;
      if (twist.linear.x != 0.0 || twist.linear.y != 0.0 || twist.angular.z != 0.0) {
        return;
      }
    }
    FAIL() << "the simulator never executed the command";
  }

  template <typename MessageT>
  void record(const std::string & topic, const rclcpp::QoS & qos, MessageT * out)
  {
    subscriptions_.push_back(probe_->create_subscription<MessageT>(
      topic, qos, [out](const MessageT & message) { *out = message; }));
  }

  std::shared_ptr<vectorfield_sim::KinematicSimNode> sim_;
  rclcpp::Node::SharedPtr probe_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
  bool stamped_{false};
  std::int64_t step_ns_{0};
  std::uint64_t steps_{0};

  // The latest messages; the stamps stay 0 until the first step.
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Odometry truth_;
  sensor_msgs::msg::LaserScan scan_;
  geometry_msgs::msg::TransformStamped odom_tf_;
  geometry_msgs::msg::TransformStamped map_tf_;
  std::size_t map_tf_count_{0};
  std::uint64_t infeasible_{0};
  bool has_infeasible_{false};
  std::vector<std::int64_t> clocks_;
};

}  // namespace vf_sim_test

#endif  // SIM_NODE_TEST_SUPPORT_HPP_
