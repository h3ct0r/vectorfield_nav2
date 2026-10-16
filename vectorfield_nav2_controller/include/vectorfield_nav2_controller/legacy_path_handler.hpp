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
/// @brief LegacyPathHandler: the plan window of the Gen A/B controller API (Humble to Kilted),
/// a port of Nav2's `nav2_controller::FeasiblePathHandler` from navigation2 `main` (plan §5.1;
/// nav2_api_matrix.md §3.3).
///
/// Adapted from navigation2 (https://github.com/ros-navigation/navigation2),
/// `nav2_controller/plugins/feasible_path_handler.{hpp,cpp}`, Apache-2.0:
/// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov;
/// Copyright (c) 2023 Dexory; Copyright (c) 2023 Open Navigation LLC. See NOTICE.
///
/// Implemented in src/legacy_path_handler.cpp (M2 wave 2, path). A plain class, compiled on every
/// distro (so the parity test can run it on Lyrical against the real FeasiblePathHandler); the
/// plugin constructs it, and the gpl listener of its parameters, only under
/// `#if !VF_HAS_PATH_HANDLER`. On Gen C the server's own path handler builds the window.

#ifndef VECTORFIELD_NAV2_CONTROLLER__LEGACY_PATH_HANDLER_HPP_
#define VECTORFIELD_NAV2_CONTROLLER__LEGACY_PATH_HANDLER_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "vectorfield_ros/compat.hpp"

namespace vectorfield_nav2_controller
{

/// @brief Configuration of LegacyPathHandler: the Legacy group of plan §5.4, declared by
/// `vectorfield_legacy_path_params` under the plugin prefix (Gen A/B only, read-only).
struct LegacyPathConfig
{
  /// `prune_distance` (m): length of plan kept ahead of the closest pose.
  double prune_distance{2.0};
  /// `max_robot_pose_search_dist` (m): how far along the plan the closest pose is searched.
  /// 0 = auto, \f$\max(W, H)/2\f$ of the costmap at construction (FeasiblePathHandler's
  /// default); negative = unbounded.
  double max_robot_pose_search_dist{0.0};
  /// `transform_tolerance` (s): TF lookup timeout.
  double transform_tolerance{0.1};
  /// `reject_unit_path`: a one-pose plan is `InvalidPath`.
  bool reject_unit_path{false};
};

/// @brief One cycle of LegacyPathHandler.
struct LegacyPathOutput
{
  /// The window: at least one pose, in the costmap global frame, stamped with the robot-pose
  /// stamp; clipped at `prune_distance` and at the first pose outside the costmap.
  nav_msgs::msg::Path window;
  /// The last pose of the full plan, transformed with the same transform (so `W.back()` and the
  /// goal never skew, unlike Lyrical's server; nav2_api_matrix.md §4.1).
  geometry_msgs::msg::PoseStamped goal;
  /// The single transform of the cycle: costmap global frame ← plan frame at the robot-pose
  /// stamp. Its yaw is \f$R_{\mathrm{plan}\to\mathrm{costmap}}\f$ for tail conditioning.
  geometry_msgs::msg::TransformStamped costmap_from_plan;
};

/// @brief The Gen A/B plan window (plan §5.1).
///
/// Each cycle (computeWindow()), exactly like `main`'s FeasiblePathHandler:
/// 1. an empty plan, or a one-pose plan with `reject_unit_path`, throws `InvalidPath`;
/// 2. **one** `tf->lookupTransform(costmap frame, plan frame, robot-pose stamp,
///    transform_tolerance)`; failure throws `ControllerTFError`; the robot pose is mapped into the
///    plan frame with its inverse;
/// 3. the closest pose is the minimum distance over the poses within `max_robot_pose_search_dist`
///    of integrated distance from the plan start, ties going to the **last** minimum (the `<=`
///    of `nav2_util::geometry_utils::min_by` on Jazzy and later; Humble's own uses `<`, so the
///    port carries its own); if it is the last pose of the searched range (and not the first), it
///    steps back one so at least 2 poses remain;
/// 4. the segment ends at the first pose more than `prune_distance` of integrated distance after
///    the closest pose (`first_after_integrated_distance`);
/// 5. the segment is transformed into the costmap frame and cut at the first pose outside the
///    costmap; an empty result throws `InvalidPath`;
/// 6. the stored plan is erased up to the closest pose, so the next search starts there
///    (monotonic progress).
///
/// **Threading:** control thread only (setPlan() and computeWindow() are called by the server's
/// control loop). Not copyable.
class LegacyPathHandler
{
public:
  /// @brief A handler for a costmap and a TF buffer.
  /// @param config Parameters; `max_robot_pose_search_dist` 0 is resolved here from the
  /// costmap's size.
  /// @param tf The TF buffer (the plugin passes the server's, which is `costmap_ros->getTfBuffer()`).
  /// @param costmap_ros The local costmap (its global frame is the output frame).
  LegacyPathHandler(
    const LegacyPathConfig & config, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  LegacyPathHandler(const LegacyPathHandler &) = delete;
  LegacyPathHandler & operator=(const LegacyPathHandler &) = delete;

  /// @brief Stores a new raw plan (any frame; its header frame is the plan frame).
  void setPlan(const nav_msgs::msg::Path & path);

  /// @brief Builds this cycle's window and goal (see the class comment).
  /// @param robot_pose Robot pose as passed by the server (costmap global frame).
  /// @param out Output; its vectors are refilled in place.
  /// @throws vectorfield_ros::compat::InvalidPath (empty plan, rejected unit plan, empty window).
  /// @throws vectorfield_ros::compat::ControllerTFError (transform unavailable).
  void computeWindow(const geometry_msgs::msg::PoseStamped & robot_pose, LegacyPathOutput * out);

  /// @brief The stored (pruned) plan.
  const nav_msgs::msg::Path & plan() const noexcept;

  /// @brief The configuration with `max_robot_pose_search_dist` resolved.
  const LegacyPathConfig & config() const noexcept;

private:
  LegacyPathConfig config_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path plan_;
};

}  // namespace vectorfield_nav2_controller

#endif  // VECTORFIELD_NAV2_CONTROLLER__LEGACY_PATH_HANDLER_HPP_
