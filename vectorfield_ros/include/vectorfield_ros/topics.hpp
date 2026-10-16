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
/// @brief Topic names of the visualization and diagnostics outputs, and the `<ns>` rule
/// (plan §4 "Topics", §4.1; nav2_api_matrix.md §4.10). A stable surface (plan §9): frozen once
/// released.
///
/// - **Plugin:** `<ns>/markers`, `<ns>/field_grid`, `<ns>/predicted_path`, `<ns>/window`,
///   `<ns>/field_state`, `<ns>/obstacle_distance`, where `<ns>` is the plugin name with every
///   '.' replaced by '/' (ROS names cannot contain '.', Nav2 plugin names may, e.g.
///   `FollowPath.primary_controller` under the Lyrical rotation shim). The names are relative,
///   so they resolve in the controller_server's namespace: `FollowPath/markers`.
/// - **Follower:** the same names under `~/` (its private namespace), plus `~/curve`.

#ifndef VECTORFIELD_ROS__TOPICS_HPP_
#define VECTORFIELD_ROS__TOPICS_HPP_

#include <string>
#include <string_view>

namespace vectorfield_ros
{

/// @brief Topic names relative to the node's visualization namespace.
namespace topics
{

/// `visualization_msgs/MarkerArray`: the marker namespaces of the legend (legend.hpp).
inline constexpr std::string_view kMarkers = "markers";
/// `visualization_msgs/MarkerArray`: the field on a grid around the robot (opt-in cost).
inline constexpr std::string_view kFieldGrid = "field_grid";
/// `nav_msgs/Path`: the collision-prediction rollout.
inline constexpr std::string_view kPredictedPath = "predicted_path";
/// `nav_msgs/Path`: the curve the field uses (plugin: the conditioned window).
inline constexpr std::string_view kWindow = "window";
/// `vectorfield_msgs/FieldState`: every per-cycle value, for plots.
inline constexpr std::string_view kFieldState = "field_state";
/// `nav_msgs/OccupancyGrid`: the obstacle distance transform (only with
/// `publish_obstacle_distance`).
inline constexpr std::string_view kObstacleDistance = "obstacle_distance";
/// `vectorfield_msgs/Curve` (follower only, transient local): the active curve.
inline constexpr std::string_view kCurve = "curve";

/// @brief The follower's namespace: its private namespace.
inline constexpr std::string_view kPrivateNamespace = "~";

/// @brief `<ns>` of a plugin: the plugin name with every '.' replaced by '/'.
/// @param plugin_name Nav2 plugin name, e.g. "FollowPath" or "FollowPath.primary_controller".
inline std::string pluginNamespace(std::string_view plugin_name)
{
  std::string ns(plugin_name);
  for (char & c : ns) {
    if (c == '.') {
      c = '/';
    }
  }
  return ns;
}

/// @brief `<ns>/<topic>`; just `<topic>` for an empty namespace.
/// @param ns pluginNamespace() of the plugin, or kPrivateNamespace for the follower.
/// @param topic One of the names above.
inline std::string topicName(std::string_view ns, std::string_view topic)
{
  std::string name;
  name.reserve(ns.size() + 1 + topic.size());
  name.append(ns);
  if (!ns.empty() && ns.back() != '/') {
    name.push_back('/');
  }
  name.append(topic);
  return name;
}

}  // namespace topics
}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__TOPICS_HPP_
