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

// Smoke test of the CurveFollower node in process (plan §6): configure and activate with a fake
// robot (odometry, TF, an empty local costmap and a footprint), one short closed-curve goal
// SUCCEEDS with laps_completed = 1, then the follower is silent. An invalid goal ends with
// INVALID_GOAL and no command.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_curve_follower/curve_follower.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_ros/compat.hpp"
#include "vectorfield_ros/conversions.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;
using vectorfield_curve_follower::CurveFollower;
using FollowCurve = vectorfield_msgs::action::FollowCurve;
using namespace std::chrono_literals;

/// A unicycle that executes `cmd_vel_nav` and publishes what the follower needs.
class FakeRobot
{
public:
  explicit FakeRobot(vf::Pose2 start)
  : node_(std::make_shared<rclcpp::Node>("fake_robot")), pose_(std::move(start))
  {
    tf_ = vr::compat::makeTfBroadcaster(node_);
    static_tf_ = vr::compat::makeStaticTfBroadcaster(node_);
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    costmap_pub_ = node_->create_publisher<nav2_msgs::msg::Costmap>(
      "local_costmap/costmap_raw", rclcpp::QoS(1).reliable());
    footprint_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "local_costmap/published_footprint", rclcpp::QoS(1).reliable());
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = vr::toTwist2(*msg);
        ++commands_;
      });
    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.frame_id = "map";
    map_odom.header.stamp = node_->now();
    map_odom.child_frame_id = "odom";
    map_odom.transform.rotation.w = 1.0;
    static_tf_->sendTransform(map_odom);
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

  std::size_t commands() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }

  vf::Twist2 lastCommand() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return cmd_;
  }

  /// Integrates the last command over `dt` and publishes TF, odometry and, every 5th step, the
  /// costmap and the footprint.
  void step(double dt)
  {
    vf::Pose2 pose;
    vf::Twist2 cmd;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      pose_ = vf::sim::integrateTwist(pose_, cmd_, dt);
      pose = pose_;
      cmd = cmd_;
    }
    const rclcpp::Time stamp = node_->now();
    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.frame_id = "odom";
    odom_base.header.stamp = stamp;
    odom_base.child_frame_id = "base_link";
    odom_base.transform.translation.x = pose.p.x();
    odom_base.transform.translation.y = pose.p.y();
    odom_base.transform.rotation = vr::yawToQuaternion(pose.yaw);
    tf_->sendTransform(odom_base);

    nav_msgs::msg::Odometry odom;
    odom.header = odom_base.header;
    odom.child_frame_id = "base_link";
    odom.pose.pose = vr::toPoseMsg(pose);
    odom.twist.twist = vr::toTwistMsg(cmd);
    odom_pub_->publish(odom);

    if (ticks_++ % 5 == 0) {
      nav2_msgs::msg::Costmap costmap;
      costmap.header.frame_id = "odom";
      costmap.header.stamp = stamp;
      costmap.metadata.resolution = 0.05F;
      costmap.metadata.size_x = 80;
      costmap.metadata.size_y = 80;
      costmap.metadata.origin.position.x = -2.0;
      costmap.metadata.origin.position.y = -2.0;
      costmap.metadata.origin.orientation.w = 1.0;
      costmap.data.assign(std::size_t{80} * std::size_t{80}, 0U);
      costmap_pub_->publish(costmap);

      geometry_msgs::msg::PolygonStamped footprint;
      footprint.header.frame_id = "base_link";
      footprint.header.stamp = stamp;
      constexpr float kCorners[4][2] = {
        {0.15F, 0.15F}, {-0.15F, 0.15F}, {-0.15F, -0.15F}, {0.15F, -0.15F}};
      for (const auto & corner : kCorners) {
        geometry_msgs::msg::Point32 point;
        point.x = corner[0];
        point.y = corner[1];
        footprint.polygon.points.push_back(point);
      }
      footprint_pub_->publish(footprint);
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  mutable std::mutex mutex_;
  vf::Pose2 pose_;
  vf::Twist2 cmd_;
  std::size_t commands_{0};
  std::size_t ticks_{0};
};

class FollowerSmoke : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      // No Nav2 in this test: nothing publishes follow_path/_action/status.
      rclcpp::Parameter("require_nav2_interlock", false),
      rclcpp::Parameter("enable_stamped_cmd_vel", false),
    });
    follower_ = std::make_shared<CurveFollower>(options);
    // On the circle x² + y² = 0.5², heading along it (counter-clockwise).
    robot_ = std::make_unique<FakeRobot>(vf::Pose2{vf::Vec2(0.5, 0.0), 0.5 * vf::kPi});
    executor_.add_node(follower_->get_node_base_interface());
    executor_.add_node(robot_->node());
    spinner_ = std::thread([this]() { executor_.spin(); });
    simulator_ = std::thread([this]() {
      auto last = std::chrono::steady_clock::now();
      while (!stop_) {
        std::this_thread::sleep_for(20ms);
        const auto now = std::chrono::steady_clock::now();
        robot_->step(std::chrono::duration<double>(now - last).count());
        last = now;
      }
    });
    client_ = rclcpp_action::create_client<FollowCurve>(robot_->node(), "follow_curve");
  }

  void TearDown() override
  {
    stop_ = true;
    simulator_.join();
    executor_.cancel();
    spinner_.join();
    client_.reset();
    executor_.remove_node(robot_->node());
    executor_.remove_node(follower_->get_node_base_interface());
    robot_.reset();
    follower_.reset();
  }

  void bringUp()
  {
    ASSERT_EQ(follower_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(follower_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(client_->wait_for_action_server(10s));
  }

  rclcpp_action::ClientGoalHandle<FollowCurve>::WrappedResult run(
    const FollowCurve::Goal & goal, std::chrono::seconds timeout)
  {
    auto goal_future = client_->async_send_goal(goal);
    if (goal_future.wait_for(10s) != std::future_status::ready) {
      ADD_FAILURE() << "goal not answered";
      return {};
    }
    const auto & handle = goal_future.get();
    if (!handle) {
      ADD_FAILURE() << "goal rejected (the follower never rejects goals)";
      return {};
    }
    auto result_future = client_->async_get_result(handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      ADD_FAILURE() << "no result within " << timeout.count() << " s";
      return {};
    }
    return result_future.get();
  }

  static FollowCurve::Goal circleGoal()
  {
    FollowCurve::Goal goal;
    goal.curve.type = vectorfield_msgs::msg::Curve::BUILTIN;
    goal.curve.builtin_shape = "ellipse";
    goal.curve.parameter_names = {"a", "b"};
    goal.curve.parameter_values = {0.5, 0.5};
    goal.frame_motion = FollowCurve::Goal::FRAME_STATIC;
    goal.laps = 1;
    return goal;
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::shared_ptr<CurveFollower> follower_;
  std::unique_ptr<FakeRobot> robot_;
  rclcpp_action::Client<FollowCurve>::SharedPtr client_;
  std::thread spinner_;
  std::thread simulator_;
  std::atomic<bool> stop_{false};
};

TEST_F(FollowerSmoke, ClosedCurveGoalSucceedsWithOneLapThenSilence)
{
  bringUp();
  const auto wrapped = run(circleGoal(), 90s);
  ASSERT_TRUE(wrapped.result) << "no result";
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED) << wrapped.result->error_msg;
  EXPECT_EQ(wrapped.result->error_code, FollowCurve::Result::NONE);
  EXPECT_EQ(wrapped.result->laps_completed, 1U);
  EXPECT_GT(wrapped.result->distance_travelled, 2.5);
  EXPECT_GT(follower_->lastCycle().commands_published, 20U);

  // One zero, then silence.
  std::this_thread::sleep_for(200ms);
  const std::size_t after = robot_->commands();
  const vf::Twist2 last = robot_->lastCommand();
  EXPECT_EQ(last.vx, 0.0);
  EXPECT_EQ(last.wz, 0.0);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(robot_->commands(), after) << "the follower must be silent between goals";

  EXPECT_EQ(follower_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(follower_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(FollowerSmoke, InvalidGoalsEndBeforeAnyCommand)
{
  bringUp();
  // desired_speed above max_linear_vel (0.5): the goal-scoped configuration is rejected.
  FollowCurve::Goal fast = circleGoal();
  fast.desired_speed = 0.6;
  const auto rejected = run(fast, 10s);
  ASSERT_TRUE(rejected.result);
  EXPECT_EQ(rejected.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(rejected.result->error_code, FollowCurve::Result::INVALID_GOAL);

  // laps > 0 on an open curve.
  FollowCurve::Goal open;
  open.curve.type = vectorfield_msgs::msg::Curve::POLYLINE;
  open.curve.points.resize(2);
  open.curve.points[1].x = 1.0;
  open.laps = 2;
  const auto invalid = run(open, 10s);
  ASSERT_TRUE(invalid.result);
  EXPECT_EQ(invalid.result->error_code, FollowCurve::Result::INVALID_CURVE);

  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(robot_->commands(), 0U);
  EXPECT_EQ(follower_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

}  // namespace
