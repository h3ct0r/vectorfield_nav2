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
/// @brief FieldVisualizer: owns the visualization publishers and publishes one cycle at the end
/// of each control step (plan §4.1).
///
/// Implemented in src/field_visualizer.cpp (M2 wave 2, ros-viz).

#ifndef VECTORFIELD_ROS__FIELD_VISUALIZER_HPP_
#define VECTORFIELD_ROS__FIELD_VISUALIZER_HPP_

#include <cstddef>
#include <string>
#include <string_view>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "vectorfield_msgs/msg/field_state.hpp"
#include "vectorfield_ros/marker_builder.hpp"
#include "vectorfield_ros/viz_config.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vectorfield_ros
{

/// @brief Publishes the six visualization topics of plan §4.1 under a namespace.
///
/// - **Topics** (topics.hpp): `<ns>/markers`, `<ns>/field_grid`, `<ns>/predicted_path`,
///   `<ns>/window`, `<ns>/field_state` and `<ns>/obstacle_distance`; `<ns>` is
///   `topics::pluginNamespace(plugin_name)` for the controller and `~` for the follower.
/// - **Gating:** nothing is built or published unless `visualize` is true, and each topic is
///   built only while it has at least one subscriber (`get_subscription_count()`), so the cost is
///   zero without a viewer. `field_grid` is additionally decimated to `field_grid_rate` using the
///   cycle stamps, and `obstacle_distance` needs `publish_obstacle_distance`.
/// - **No timers:** publish() runs in the control thread at the end of each cycle.
/// - **DELETEALL** on goal end, reset and deactivation: clear().
/// - **Lifecycle:** the publishers are rclcpp_lifecycle publishers created through the
///   `rclcpp_lifecycle::LifecycleNode` base (no auto-activation on Lyrical); the owner calls
///   on_activate() in activate() and on_deactivate() in deactivate(), and destroys the visualizer
///   in cleanup().
///
/// QoS: reliable, volatile, depth 1 for the marker arrays, paths and the distance grid; depth 10
/// for field_state.
///
/// **Threading:** every method except the constructor and builderCalls() is called from the
/// control thread (or a lifecycle transition, which never overlaps a control cycle).
class FieldVisualizer
{
public:
  /// @brief Creates the publishers (inactive).
  /// @param node The owning node (the controller_server, or the follower).
  /// @param ns Topic namespace without a trailing '/': `topics::pluginNamespace(name)` or `~`.
  /// @param config Initial settings (setConfig() updates them).
  FieldVisualizer(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & ns,
    const VizConfig & config = VizConfig{});

  /// @brief Destroys the publishers.
  ~FieldVisualizer();

  FieldVisualizer(const FieldVisualizer &) = delete;
  FieldVisualizer & operator=(const FieldVisualizer &) = delete;
  FieldVisualizer(FieldVisualizer &&) = delete;
  FieldVisualizer & operator=(FieldVisualizer &&) = delete;

  /// @brief Activates every publisher.
  void on_activate();

  /// @brief Publishes a DELETEALL on `markers` and `field_grid` (when activated and subscribed),
  /// then deactivates every publisher.
  /// @param stamp Stamp of the DELETEALL markers.
  /// @param frame_id Frame of the DELETEALL markers (the control frame).
  void on_deactivate(const builtin_interfaces::msg::Time & stamp, std::string_view frame_id);

  /// @brief New settings (after a parameter update; control thread).
  void setConfig(const VizConfig & config);

  /// @brief The current settings.
  const VizConfig & config() const noexcept;

  /// @brief Publishes one cycle (end of the control step).
  ///
  /// For each topic with subscribers (and `visualize` true): `markers` ← MarkerBuilder::markers;
  /// `field_grid` ← fieldGrid (at most `field_grid_rate`); `predicted_path`, `window`;
  /// `field_state` ← toFieldState() with the context's header; `obstacle_distance` (only with
  /// `publish_obstacle_distance`). Never throws: errors are logged, throttled to 1 s.
  void publish(const MarkerContext & context) noexcept;

  /// @brief DELETEALL on `markers` and `field_grid` (goal end, reset); no-op when inactive or
  /// unsubscribed. Never throws.
  void clear(const builtin_interfaces::msg::Time & stamp, std::string_view frame_id) noexcept;

  /// @brief Number of MarkerBuilder / toFieldState() invocations since construction (tests: it
  /// stays 0 while no topic has a subscriber).
  std::size_t builderCalls() const noexcept;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  VizConfig config_;
  MarkerBuilder builder_;
  vectorfield_msgs::msg::FieldState field_state_;
  double last_grid_time_{-1.0};
  std::size_t builder_calls_{0};
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr window_pub_;
  rclcpp_lifecycle::LifecyclePublisher<vectorfield_msgs::msg::FieldState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr distance_pub_;
};

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__FIELD_VISUALIZER_HPP_
