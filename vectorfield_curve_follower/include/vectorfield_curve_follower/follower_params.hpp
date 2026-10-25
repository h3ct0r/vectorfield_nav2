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
/// @brief The follower's parameter binding: `kFollowerCoreParams`, the ROS-only allow-list and
/// the follower's configuration builder.
///
/// The follower owns two tables, both unprefixed: vectorfield_ros' `kCommonCoreParams` and
/// `kFollowerCoreParams` (the CurveOptions keys and `assumption3_margin`). Every other key of
/// vectorfield_follower_params.yaml is ROS-only (kFollowerRosOnlyParams). The node-derived fields
/// (`CoreConfig::node`: `controller_frequency` and the footprint from `published_footprint`) are
/// set after the tables by vectorfield_ros::setNodeDerivedFields().

#ifndef VECTORFIELD_CURVE_FOLLOWER__FOLLOWER_PARAMS_HPP_
#define VECTORFIELD_CURVE_FOLLOWER__FOLLOWER_PARAMS_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_curve_follower/vectorfield_follower_params.hpp"
#include "vectorfield_ros/core_params.hpp"

namespace vectorfield_curve_follower
{

/// @brief The gpl-generated parameters of vectorfield_follower_params.yaml.
using FollowerParams = vectorfield_follower_params::Params;
/// @brief The gpl-generated listener of vectorfield_follower_params.yaml.
using FollowerParamListener = vectorfield_follower_params::ParamListener;
/// @brief A row of kFollowerCoreParams.
using FollowerParamRow = vectorfield_ros::CoreParamRow<FollowerParams>;

/// @brief Number of rows of kFollowerCoreParams: the six CurveOptions keys and
/// `assumption3_margin`.
inline constexpr std::size_t kFollowerCoreParamCount = 7;

/// @brief The binding table of vectorfield_follower_params, in YAML order: the
/// curve-intake keys (`CoreParamTarget::curve_options`) and `assumption3_margin`
/// (`CoreParamTarget::config.field.assumption3_margin`). Integer keys accept only integer values
/// that fit an `int`.
extern const std::array<FollowerParamRow, kFollowerCoreParamCount> kFollowerCoreParams;

/// @brief The keys of vectorfield_follower_params with no core field: the follower's part of the
/// ROS-only allow-list of the coverage test (the common part is
/// vectorfield_ros::kCommonRosOnlyParams). Parameters that libraries declare on the same node
/// (`use_sim_time`, `enable_stamped_cmd_vel` of the TwistPublisher on Jazzy and later,
/// `diagnostic_updater.*`, the bond parameters on Lyrical) are not listed.
inline constexpr std::array<std::string_view, 18> kFollowerRosOnlyParams = {
  "controller_frequency",   "global_frame",
  "robot_base_frame",       "odom_topic",
  "transform_tolerance",    "input_timeout",
  "costmap_timeout",        "stale_abort_timeout",
  "blocked_timeout",        "goal_tolerance",
  "assumption3_abort_time", "frame_twist_cutoff_hz",
  "frame_twist_max_speed",  "moving_frame_reference",
  "progress_radius",        "progress_timeout",
  "exclusive_actions",      "require_nav2_interlock",
};

/// @brief The setters of both tables the follower owns (common, then follower), for the
/// vectorfield_ros::CrossValidator.
std::vector<vectorfield_ros::ParamSetter> followerSetters();

/// @brief The node-level configuration: both tables applied to a default target, then the
/// node-derived fields (`controller_frequency`, and the robot-frame footprint when known).
///
/// The goal-scoped overrides (desired_speed, direction) are not applied here (goalScopedConfig()
/// in goal_setup.hpp).
/// @param common Values of the common listener.
/// @param follower Values of the follower listener.
/// @param footprint Robot-frame footprint polygon; empty: the core's default footprint is kept
/// (no published_footprint received yet).
/// @param target Output.
/// @param error If not null, receives the name of the first row that failed (a gpl value the
/// core cannot use: a bug).
/// @return False if a row failed; the other rows are still applied.
bool toFollowerTarget(
  const vectorfield_ros::CommonParams & common, const FollowerParams & follower,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  vectorfield_ros::CoreParamTarget * target, std::string * error = nullptr);

}  // namespace vectorfield_curve_follower

#endif  // VECTORFIELD_CURVE_FOLLOWER__FOLLOWER_PARAMS_HPP_
