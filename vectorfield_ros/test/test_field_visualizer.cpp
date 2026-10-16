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

// FieldVisualizer (plan §4.1; §9 layer 2 "visualization"): with a subscriber, `markers` is
// published every cycle and `field_grid` at `field_grid_rate`; with no subscriber the builder is
// never called; `visualize: false` publishes nothing; clear() and on_deactivate() send DELETEALL.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_msgs/msg/field_state.hpp"
#include "vectorfield_ros/field_visualizer.hpp"
#include "vectorfield_ros/legend.hpp"
#include "vectorfield_ros/viz_config.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "viz_test_support.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_ros::FieldVisualizer;
using vectorfield_ros::MarkerContext;
using vectorfield_ros::VizConfig;
using vf::Pose2;
using vf::Vec2;
using vf_viz_test::Scenario;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class FieldVisualizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("vf_field_visualizer_test");
    listener_ = std::make_shared<rclcpp::Node>("vf_field_visualizer_listener");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(listener_);
    world_ = std::make_unique<vf_viz_test::World>(Vec2(-2.0, -2.0), 8.0, 4.0);
    world_->addBox(Vec2(1.5, 1.2), Vec2(2.0, 1.6));
    world_->commit();
    scenario_ = std::make_unique<Scenario>(
      vf::CoreConfig{}, std::vector<Vec2>{Vec2(-1.0, 0.0), Vec2(4.0, 0.0)},
      Pose2{Vec2(0.0, 0.1), 0.0});
    scenario_->input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
    scenario_->input().obstacles = world_->map();
    scenario_->run(3);
  }

  void TearDown() override
  {
    executor_.reset();
    listener_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  /// @brief The scenario's last cycle with the stamp of cycle k.
  MarkerContext contextAt(std::int64_t cycle) const
  {
    MarkerContext c = scenario_->context();
    c.stamp = vf_viz_test::stampOf(cycle);
    return c;
  }

  /// @brief Spins the listener until `done` or the timeout; returns `done()`.
  bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(5));
    }
    return done();
  }

  /// @brief Spins the listener for a fixed time (to observe that nothing arrives).
  void spinFor(std::chrono::milliseconds duration)
  {
    spinUntil([] { return false; }, duration);
  }

  template <typename MessageT>
  typename rclcpp::Subscription<MessageT>::SharedPtr subscribe(
    const std::string & topic, std::size_t * count, MessageT * last)
  {
    return listener_->create_subscription<MessageT>(
      topic, rclcpp::QoS(10).reliable(), [count, last](const MessageT & message) {
        ++*count;
        *last = message;
      });
  }

  /// @brief Publishes cycles until `ready` holds (discovery done); returns the next cycle.
  std::int64_t warmUp(
    FieldVisualizer & viz, const std::function<bool()> & ready, std::int64_t cycle)
  {
    const bool ok = spinUntil(
      [&] {
        viz.publish(contextAt(cycle++));
        return ready();
      },
      std::chrono::seconds(20));
    EXPECT_TRUE(ok) << "no subscriber matched";
    return cycle;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr listener_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<vf_viz_test::World> world_;
  std::unique_ptr<Scenario> scenario_;
};

TEST_F(FieldVisualizerTest, BuildsNothingWithoutSubscribers)
{
  FieldVisualizer viz(node_, "FollowPath");
  viz.on_activate();
  for (std::int64_t k = 0; k < 20; ++k) {
    viz.publish(contextAt(k));
  }
  EXPECT_EQ(viz.builderCalls(), 0U);
  viz.clear(vf_viz_test::stampOf(20), vf_viz_test::kFrame);
  viz.on_deactivate(vf_viz_test::stampOf(21), vf_viz_test::kFrame);
  EXPECT_EQ(viz.builderCalls(), 0U);
}

TEST_F(FieldVisualizerTest, MarkersEveryCycleAndTheGridAtItsRate)
{
  VizConfig config;
  config.field_grid_rate = 2.0;
  FieldVisualizer viz(node_, "FollowPath", config);
  viz.on_activate();

  std::size_t markers = 0;
  std::size_t grids = 0;
  MarkerArray last_markers;
  MarkerArray last_grid;
  const auto markers_sub = subscribe("FollowPath/markers", &markers, &last_markers);
  const auto grid_sub = subscribe("FollowPath/field_grid", &grids, &last_grid);
  std::int64_t cycle = warmUp(viz, [&] { return markers > 0 && grids > 0; }, 0);

  spinFor(std::chrono::milliseconds(300));  // drain the warm-up messages

  // 40 cycles at 20 Hz (2 s) from a fresh schedule: 40 markers, 4 grids (t0 + 0, 0.5, 1, 1.5 s).
  viz.setConfig(config);
  markers = 0;
  grids = 0;
  const std::size_t calls = viz.builderCalls();
  for (int k = 0; k < 40; ++k) {
    viz.publish(contextAt(cycle++));
    const std::size_t expected = static_cast<std::size_t>(k) + 1U;
    ASSERT_TRUE(spinUntil([&] { return markers >= expected; }, std::chrono::seconds(10)));
  }
  ASSERT_TRUE(spinUntil([&] { return grids >= 4U; }, std::chrono::seconds(10)));
  spinFor(std::chrono::milliseconds(200));
  EXPECT_EQ(markers, 40U);
  EXPECT_EQ(grids, 4U);
  EXPECT_EQ(viz.builderCalls() - calls, 44U);  // only the subscribed topics are built
  ASSERT_EQ(last_markers.markers.size(), vectorfield_ros::legend::kMarkerCount);
  EXPECT_EQ(last_markers.markers[0].header.frame_id, vf_viz_test::kFrame);
  EXPECT_EQ(last_markers.markers[0].header.stamp, vf_viz_test::stampOf(cycle - 1));
  ASSERT_FALSE(last_grid.markers.empty());
  EXPECT_EQ(last_grid.markers[0].action, Marker::DELETEALL);
  EXPECT_GT(last_grid.markers.size(), 1U);

  // Goal end: one DELETEALL on each topic.
  viz.clear(vf_viz_test::stampOf(cycle), vf_viz_test::kFrame);
  ASSERT_TRUE(spinUntil([&] { return markers == 41U && grids == 5U; }, std::chrono::seconds(10)));
  ASSERT_EQ(last_markers.markers.size(), 1U);
  EXPECT_EQ(last_markers.markers[0].action, Marker::DELETEALL);
  ASSERT_EQ(last_grid.markers.size(), 1U);
  EXPECT_EQ(last_grid.markers[0].action, Marker::DELETEALL);

  // visualize: false publishes and builds nothing (clear() counted one deleteAll()).
  config.visualize = false;
  viz.setConfig(config);
  const std::size_t frozen = viz.builderCalls();
  for (int k = 0; k < 20; ++k) {
    viz.publish(contextAt(cycle++));
  }
  spinFor(std::chrono::milliseconds(300));
  EXPECT_EQ(markers, 41U);
  EXPECT_EQ(grids, 5U);
  EXPECT_EQ(viz.builderCalls(), frozen);

  // Deactivation clears, then nothing is published.
  config.visualize = true;
  viz.setConfig(config);
  viz.on_deactivate(vf_viz_test::stampOf(cycle), vf_viz_test::kFrame);
  ASSERT_TRUE(spinUntil([&] { return markers == 42U; }, std::chrono::seconds(10)));
  EXPECT_EQ(last_markers.markers.size(), 1U);
  EXPECT_EQ(viz.builderCalls(), frozen + 1U);
  viz.publish(contextAt(cycle++));
  spinFor(std::chrono::milliseconds(300));
  EXPECT_EQ(markers, 42U);
  EXPECT_EQ(viz.builderCalls(), frozen + 1U);
}

TEST_F(FieldVisualizerTest, FieldStatePathsAndObstacleDistance)
{
  VizConfig config;
  FieldVisualizer viz(node_, "FollowPath", config);
  viz.on_activate();
  std::size_t markers = 0;
  std::size_t states = 0;
  std::size_t windows = 0;
  std::size_t predicted = 0;
  std::size_t distances = 0;
  MarkerArray last_markers;
  vectorfield_msgs::msg::FieldState last_state;
  nav_msgs::msg::Path last_window;
  nav_msgs::msg::Path last_predicted;
  nav_msgs::msg::OccupancyGrid last_distance;
  const auto s1 = subscribe("FollowPath/markers", &markers, &last_markers);
  const auto s2 = subscribe("FollowPath/field_state", &states, &last_state);
  const auto s3 = subscribe("FollowPath/window", &windows, &last_window);
  const auto s4 = subscribe("FollowPath/predicted_path", &predicted, &last_predicted);
  const auto s5 = subscribe("FollowPath/obstacle_distance", &distances, &last_distance);
  std::int64_t cycle =
    warmUp(viz, [&] { return markers > 0 && states > 0 && windows > 0 && predicted > 0; }, 0);
  spinFor(std::chrono::milliseconds(200));
  EXPECT_EQ(distances, 0U);  // publish_obstacle_distance is false by default

  const vf::TrackerOutput & out = scenario_->output();
  EXPECT_EQ(last_state.header.frame_id, vf_viz_test::kFrame);
  EXPECT_EQ(last_state.phase, vf::toUnderlying(out.phase));
  EXPECT_EQ(last_state.status, out.status);
  EXPECT_EQ(last_state.distance, out.field.D);
  EXPECT_EQ(last_state.cmd.linear.x, out.cmd.vx);
  EXPECT_EQ(last_state.field.x, out.field.F.x());
  EXPECT_EQ(last_state.limiter_scale, out.limiter_alpha);
  EXPECT_EQ(last_state.compute_time_ms, 0.1);
  EXPECT_EQ(last_window.poses.size(), 2U);
  EXPECT_EQ(last_predicted.poses.size(), out.predicted.size());

  config.publish_obstacle_distance = true;
  viz.setConfig(config);
  cycle = warmUp(viz, [&] { return distances > 0; }, cycle);
  EXPECT_EQ(last_distance.info.width, 160U);
  EXPECT_EQ(last_distance.info.height, 80U);
  EXPECT_EQ(last_distance.header.frame_id, vf_viz_test::kFrame);
}

}  // namespace
