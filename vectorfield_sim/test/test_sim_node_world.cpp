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
/// @brief KinematicSimNode in process, frames and world: `initialpose`, the map→odom step
/// service, the laser against `/map` and the unmapped obstacles, and parameter validation
/// (upstream_audit U37).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sim_node_test_support.hpp"

namespace
{

using vf_sim_test::expectPoseNear;
using vf_sim_test::SimNodeTest;
using vf_sim_test::toNs;
using vf_sim_test::toPose2;
namespace vf = vectorfield_core;
using namespace std::chrono_literals;

geometry_msgs::msg::Pose toPose(const vf::Pose2 & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.p.x();
  message.position.y = pose.p.y();
  message.orientation.z = std::sin(0.5 * pose.yaw);
  message.orientation.w = std::cos(0.5 * pose.yaw);
  return message;
}

bool poseNear(const vf::Pose2 & a, const vf::Pose2 & b, double tol)
{
  return (a.p - b.p).norm() <= tol && std::abs(vf::wrapAngle(a.yaw - b.yaw)) <= tol;
}

/// Constructs and drops a paused simulator (for the constructor's validation).
void construct(std::vector<rclcpp::Parameter> overrides)
{
  overrides.emplace_back("real_time_factor", 0.0);
  const auto node = std::make_shared<vectorfield_sim::KinematicSimNode>(
    rclcpp::NodeOptions().parameter_overrides(overrides));
  EXPECT_NE(node, nullptr);
}

class SimNodeWorld : public SimNodeTest
{
protected:
  /// Steps until the robot is at rest after a zero command.
  void stop()
  {
    publish({});
    for (int i = 0; i < 200; ++i) {
      executor_->spin_some(10ms);
      ASSERT_NO_FATAL_FAILURE(stepAndWait());
      if (odom_.twist.twist.linear.x == 0.0 && odom_.twist.twist.angular.z == 0.0) {
        return;
      }
    }
    FAIL() << "the robot did not stop";
  }

  /// Steps until the scan of the current step arrives and satisfies `done`.
  bool scanUntil(const std::function<bool(const sensor_msgs::msg::LaserScan &)> & done)
  {
    for (int attempt = 0; attempt < 100; ++attempt) {
      executor_->spin_some(10ms);
      for (int i = 0; i < 10; ++i) {
        stepAndWait();
      }
      const std::int64_t now = timeNs();
      if (spinUntil([this, now]() { return toNs(scan_.header.stamp) == now; }) && done(scan_)) {
        return true;
      }
    }
    return false;
  }
};

TEST_F(SimNodeWorld, InitialPoseResetsTheMapPoseThroughMapToOdom)
{
  ASSERT_NO_FATAL_FAILURE(start({{"cmd_vel_timeout", 0.0}}));
  ASSERT_NO_FATAL_FAILURE(activate({0.5, 0.0, 0.4}));
  for (int i = 0; i < 20; ++i) {
    ASSERT_NO_FATAL_FAILURE(stepAndWait());
  }
  ASSERT_NO_FATAL_FAILURE(stop());
  const vf::Pose2 odom = toPose2(odom_);
  ASSERT_GT(odom.p.norm(), 0.1);

  auto initial_pose_pub =
    probe_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  // A pose in another frame is ignored.
  message.header.frame_id = "odom";
  message.pose.pose = toPose(vf::Pose2{vf::Vec2(5.0, 5.0), 0.0});
  initial_pose_pub->publish(message);
  spinUntil([]() { return false; }, 200ms);
  expectPoseNear(toPose2(map_tf_.transform), vf::Pose2{}, 0.0);

  const vf::Pose2 target{vf::Vec2(2.0, -1.0), 1.0};
  message.header.frame_id = "map";
  message.pose.pose = toPose(target);
  initial_pose_pub->publish(message);
  ASSERT_TRUE(spinUntil([this, &odom, &target]() {
    const auto map_to_odom = vf::Isometry2::fromPose(toPose2(map_tf_.transform));
    return poseNear(map_to_odom.apply(odom), target, 1e-9);
  }));
  // The odom pose is unchanged; the robot is at the target in the map.
  ASSERT_NO_FATAL_FAILURE(stepAndWait());
  expectPoseNear(toPose2(odom_), odom, 1e-12);
  ASSERT_TRUE(spinUntil([this]() { return toNs(truth_.header.stamp) == timeNs(); }));
  expectPoseNear(toPose2(truth_), target, 1e-9);
}

TEST_F(SimNodeWorld, StepServiceMovesOnlyThePublishedMapToOdom)
{
  ASSERT_NO_FATAL_FAILURE(start({{"initial_pose_x", 1.0}, {"initial_pose_y", 2.0}}));
  ASSERT_NO_FATAL_FAILURE(stepAndWait());
  ASSERT_TRUE(spinUntil([this]() { return toNs(truth_.header.stamp) == timeNs(); }));
  const vf::Pose2 truth = toPose2(truth_);
  expectPoseNear(truth, vf::Pose2{vf::Vec2(1.0, 2.0), 0.0}, 1e-12);

  auto client =
    probe_->create_client<nav2_msgs::srv::SetInitialPose>("/vectorfield_sim/step_map_to_odom");
  ASSERT_TRUE(client->wait_for_service(10s));
  const vf::Pose2 expected{vf::Vec2(1.05, 1.98), 0.035};
  for (int call = 0; call < 2; ++call) {
    auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
    request->pose.pose.pose = toPose(vf::Pose2{vf::Vec2(0.05, -0.02), 0.035});
    if (call == 1) {
      // A non-finite step is ignored.
      request->pose.pose.pose.position.x = std::nan("");
    }
    auto future = client->async_send_request(request);
    ASSERT_EQ(
      executor_->spin_until_future_complete(future, 10s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(spinUntil(
      [this, &expected]() { return poseNear(toPose2(map_tf_.transform), expected, 1e-9); }));
  }

  // The true pose, and hence the laser, did not move.
  ASSERT_NO_FATAL_FAILURE(stepAndWait());
  ASSERT_TRUE(spinUntil([this]() { return toNs(truth_.header.stamp) == timeNs(); }));
  expectPoseNear(toPose2(truth_), truth, 1e-12);
  expectPoseNear(toPose2(map_tf_.transform), expected, 1e-9);
}

TEST_F(SimNodeWorld, ScanSeesTheMapAndTheUnmappedObstacles)
{
  ASSERT_NO_FATAL_FAILURE(start({{"circle_obstacles", std::vector<double>{0.0, 1.0, 0.25}}}));
  // 5 m x 5 m centred on the robot; the wall's face is at x = -2.5 + 80 * 0.05 = 1.5.
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  constexpr std::size_t kSide = 100;
  map.info.width = static_cast<std::uint32_t>(kSide);
  map.info.height = static_cast<std::uint32_t>(kSide);
  map.info.resolution = 0.05F;
  map.info.origin = toPose(vf::Pose2{vf::Vec2(-2.5, -2.5), 0.0});
  map.data.assign(kSide * kSide, 0);
  for (std::size_t row = 0; row < kSide; ++row) {
    map.data[row * kSide + 80] = 100;
  }
  auto map_pub = probe_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable());
  map_pub->publish(map);
  ASSERT_TRUE(sim_->set_parameter({"box_obstacles", std::vector<double>{-1.0, 0.0, 0.4, 0.4, 0.0}})
                .successful);

  ASSERT_TRUE(scanUntil([](const auto & scan) { return std::isfinite(scan.ranges.at(180)); }));
  EXPECT_EQ(scan_.header.frame_id, "base_scan");
  EXPECT_EQ(toNs(scan_.header.stamp) % 100'000'000, 0);
  ASSERT_EQ(scan_.ranges.size(), 360U);
  EXPECT_FLOAT_EQ(scan_.angle_min, static_cast<float>(-vf::kPi));
  EXPECT_FLOAT_EQ(scan_.angle_increment, static_cast<float>(vf::kTwoPi / 360.0));
  EXPECT_FLOAT_EQ(scan_.angle_max, static_cast<float>(vf::kPi - vf::kTwoPi / 360.0));
  EXPECT_FLOAT_EQ(scan_.range_max, 30.0F);
  EXPECT_FLOAT_EQ(scan_.range_min, 0.05F);
  EXPECT_FLOAT_EQ(scan_.scan_time, 0.1F);
  EXPECT_NEAR(scan_.ranges[180], 1.5, 1e-5);   // the wall, ahead
  EXPECT_NEAR(scan_.ranges[270], 0.75, 1e-5);  // the circle, to the left
  EXPECT_NEAR(scan_.ranges[0], 0.8, 1e-5);     // the box, behind
  EXPECT_TRUE(std::isinf(scan_.ranges[90]));   // nothing to the right

  // Obstacles change at run time.
  ASSERT_TRUE(sim_->set_parameter({"circle_obstacles", std::vector<double>{}}).successful);
  ASSERT_TRUE(scanUntil([](const auto & scan) { return std::isinf(scan.ranges.at(270)); }));
  EXPECT_NEAR(scan_.ranges[180], 1.5, 1e-5);
}

TEST_F(SimNodeWorld, NoReturnValueFollowsScanUseInf)
{
  ASSERT_NO_FATAL_FAILURE(start({{"scan_use_inf", false}, {"scan_range_max", 8.0}}));
  ASSERT_TRUE(scanUntil([](const auto & scan) { return !scan.ranges.empty(); }));
  for (const float range : scan_.ranges) {
    ASSERT_FLOAT_EQ(range, 7.9F);
  }
}

TEST_F(SimNodeWorld, InvalidConfigurationsAreRejected)
{
  EXPECT_THROW(construct({{"motion_model", std::string("tank")}}), std::invalid_argument);
  EXPECT_THROW(
    construct({{"motion_model", std::string("ackermann")}, {"min_turning_radius", 0.0}}),
    std::invalid_argument);
  EXPECT_THROW(construct({{"step_size", 0.0}}), rclcpp::exceptions::InvalidParameterValueException);
  EXPECT_THROW(
    construct({{"scan_beams", static_cast<std::int64_t>(0)}}),
    rclcpp::exceptions::InvalidParameterValueException);
  EXPECT_NO_THROW(construct({{"motion_model", std::string("skid_steer")}}));
}

class UpstreamRegression : public SimNodeWorld
{
};

/// U37: the upstream simulators failed with no obstacles and silently dropped unmatched ones.
TEST_F(UpstreamRegression, U37_SimWithoutObstacles)
{
  ASSERT_NO_FATAL_FAILURE(start());
  EXPECT_TRUE(sim_->get_parameter("circle_obstacles").as_double_array().empty());
  EXPECT_TRUE(sim_->get_parameter("box_obstacles").as_double_array().empty());
  ASSERT_NO_FATAL_FAILURE(activate({0.3, 0.0, 0.0}));
  for (int i = 0; i < 10; ++i) {
    ASSERT_NO_FATAL_FAILURE(stepAndWait());
  }
  EXPECT_NEAR(odom_.pose.pose.position.x, 11 * 0.003, 1e-12);

  // Malformed lists are rejected with an error and leave the obstacles unchanged.
  auto result = sim_->set_parameter({"circle_obstacles", std::vector<double>{1.0, 2.0}});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("multiple of 3"), std::string::npos) << result.reason;
  result = sim_->set_parameter({"box_obstacles", std::vector<double>{0.0, 0.0, 1.0, -1.0, 0.0}});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("size_y"), std::string::npos) << result.reason;
  result = sim_->set_parameter({"box_obstacles", std::string("a box")});
  EXPECT_FALSE(result.successful);
  EXPECT_TRUE(sim_->get_parameter("circle_obstacles").as_double_array().empty());
  // Integer lists and an untyped empty list (what `ros2 param set ... "[]"` sends) are accepted.
  EXPECT_TRUE(
    sim_->set_parameter({"circle_obstacles", std::vector<std::int64_t>{1, 2, 1}}).successful);
  EXPECT_TRUE(sim_->set_parameter({"circle_obstacles", std::vector<bool>{}}).successful);

  // At construction, a malformed list fails the node.
  EXPECT_THROW(
    construct({{"circle_obstacles", std::vector<double>{1.0, 2.0, -0.5}}}),
    rclcpp::exceptions::InvalidParameterValueException);
  EXPECT_THROW(
    construct({{"box_obstacles", std::vector<double>{1.0, 2.0, 0.5}}}),
    rclcpp::exceptions::InvalidParameterValueException);
}

}  // namespace
