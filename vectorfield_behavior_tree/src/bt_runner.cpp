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
/// @brief bt_runner: ticks a behaviour tree from a file until it finishes, without
/// bt_navigator (docs/behavior_tree.md, "bt_runner").
///
/// It loads `plugin_lib_names`, puts what nav2_behavior_tree's nodes read on the blackboard
/// (`node`, `bt_loop_duration`, `server_timeout`, `cancel_timeout`, `wait_for_service_timeout`)
/// and, when `goal_pose` is set, a `goal` (geometry_msgs/PoseStamped), then ticks the root every
/// `bt_loop_duration`. Exit status: 0 SUCCESS, 1 FAILURE, 2 an error, or SIGINT/SIGTERM (the
/// tree is halted first, so running goals are canceled).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vectorfield_behavior_tree/bt_compat.hpp"

namespace
{

namespace compat = vectorfield_behavior_tree::compat;

volatile std::sig_atomic_t g_stop = 0;

extern "C" void onSignal(int /*signal*/) { g_stop = 1; }

constexpr int kExitSuccess = 0;
constexpr int kExitFailure = 1;
constexpr int kExitError = 2;

/// The plugins of the example trees in vectorfield_bringup/behavior_trees.
const std::vector<std::string> kDefaultPlugins = {
  "vectorfield_follow_curve_action_bt_node",
  "nav2_compute_path_to_pose_action_bt_node",
  "nav2_follow_path_action_bt_node",
  "nav2_clear_costmap_service_bt_node",
  "nav2_wait_action_bt_node",
  "nav2_recovery_node_bt_node",
  "nav2_pipeline_sequence_bt_node",
  "nav2_rate_controller_bt_node",
};

std::chrono::milliseconds milliseconds(
  const compat::BtNode::SharedPtr & node, const std::string & name, std::int64_t default_ms)
{
  const auto value = node->declare_parameter<std::int64_t>(name, default_ms);
  if (value <= 0) {
    throw std::invalid_argument(name + " must be positive (ms), not " + std::to_string(value));
  }
  return std::chrono::milliseconds(value);
}

int run()
{
  auto node = std::make_shared<compat::BtNode>("bt_runner");
  const auto tree_file = node->declare_parameter<std::string>("tree_file", "");
  const auto plugins =
    node->declare_parameter<std::vector<std::string>>("plugin_lib_names", kDefaultPlugins);
  const auto goal_pose =
    node->declare_parameter<std::vector<double>>("goal_pose", std::vector<double>{});
  const auto goal_frame = node->declare_parameter<std::string>("goal_frame", "map");
  const bool dry_run = node->declare_parameter<bool>("dry_run", false);
  const bool log_transitions = node->declare_parameter<bool>("log_transitions", true);
  const auto loop = milliseconds(node, "bt_loop_duration", 10);
  const auto server_timeout = milliseconds(node, "default_server_timeout", 1000);
  const auto cancel_timeout = milliseconds(node, "default_cancel_timeout", 1000);
  const auto wait_for_service = milliseconds(node, "wait_for_service_timeout", 5000);
  if (tree_file.empty()) {
    RCLCPP_FATAL(node->get_logger(), "set the tree_file parameter");
    return kExitError;
  }

  BT::BehaviorTreeFactory factory;
  for (const auto & plugin : plugins) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set<compat::BtNode::SharedPtr>("node", node);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", loop);
  blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout);
  blackboard->set<std::chrono::milliseconds>("cancel_timeout", cancel_timeout);
  blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service);
  if (!goal_pose.empty()) {
    if (goal_pose.size() != 3) {
      RCLCPP_FATAL(node->get_logger(), "goal_pose must be [x, y, yaw] (m, m, rad)");
      return kExitError;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = goal_frame;
    goal.header.stamp = node->now();
    goal.pose.position.x = goal_pose[0];
    goal.pose.position.y = goal_pose[1];
    goal.pose.orientation.z = std::sin(0.5 * goal_pose[2]);
    goal.pose.orientation.w = std::cos(0.5 * goal_pose[2]);
    blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal);
  }

  auto tree = factory.createTreeFromFile(tree_file, blackboard);
  RCLCPP_INFO(node->get_logger(), "created the tree of %s", tree_file.c_str());
  if (dry_run) {
    return kExitSuccess;
  }
  std::unique_ptr<BT::StdCoutLogger> logger;
  if (log_transitions) {
    logger = std::make_unique<BT::StdCoutLogger>(tree);
  }

  auto next = std::chrono::steady_clock::now();
  while (true) {
    if (g_stop != 0 || !rclcpp::ok()) {
      tree.haltTree();
      RCLCPP_WARN(node->get_logger(), "interrupted: halted the tree");
      return kExitError;
    }
    const auto status = compat::tickOnce(tree);
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      const bool success = status == BT::NodeStatus::SUCCESS;
      RCLCPP_INFO(node->get_logger(), "the tree finished: %s", success ? "SUCCESS" : "FAILURE");
      return success ? kExitSuccess : kExitFailure;
    }
    next = std::max(next + loop, std::chrono::steady_clock::now());
    std::this_thread::sleep_until(next);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  // The tree is halted on a signal before rclcpp shuts down, so cancellations still go out.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  int code = kExitError;
  try {
    code = run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bt_runner"), "%s", error.what());
  }
  rclcpp::shutdown();
  return code;
}
