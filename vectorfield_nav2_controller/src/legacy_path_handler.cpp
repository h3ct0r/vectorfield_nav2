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

// Adapted from navigation2 nav2_controller/plugins/feasible_path_handler.cpp (branch main),
// Apache-2.0: Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov;
// Copyright (c) 2023 Dexory; Copyright (c) 2023 Open Navigation LLC. See NOTICE.

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "vectorfield_nav2_controller/legacy_path_handler.hpp"
#include "vectorfield_ros/conversions.hpp"

namespace vectorfield_nav2_controller
{

namespace
{

using PoseIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;

/// Planar distance between two poses (nav2_util::geometry_utils::euclidean_distance, 2D).
double planarDistance(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b) noexcept
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

/// nav2_util::geometry_utils::first_after_integrated_distance: the first pose after `distance`
/// of integrated path length from `begin`, or `end`.
PoseIterator firstAfterIntegratedDistance(PoseIterator begin, PoseIterator end, double distance)
{
  if (begin == end) {
    return end;
  }
  double integrated = 0.0;
  for (PoseIterator it = begin; it != std::prev(end); ++it) {
    integrated += planarDistance(*it, *std::next(it));
    if (integrated > distance) {
      return std::next(it);
    }
  }
  return end;
}

/// The pose of [begin, end) closest to `point`, ties going to the last minimum (the `<=` of
/// nav2_util::geometry_utils::min_by on Jazzy and later).
PoseIterator closestPose(PoseIterator begin, PoseIterator end, const vectorfield_core::Vec2 & point)
{
  if (begin == end) {
    return end;
  }
  PoseIterator best = begin;
  double best_distance = std::numeric_limits<double>::infinity();
  for (PoseIterator it = begin; it != end; ++it) {
    const double distance =
      std::hypot(it->pose.position.x - point.x(), it->pose.position.y - point.y());
    if (distance <= best_distance) {
      best_distance = distance;
      best = it;
    }
  }
  return best;
}

/// Half the larger side of the costmap (FeasiblePathHandler's getCostmapMaxExtent()).
double costmapMaxExtent(nav2_costmap_2d::Costmap2DROS & costmap_ros)
{
  const nav2_costmap_2d::Costmap2D & costmap = *costmap_ros.getCostmap();
  return 0.5 * std::max(costmap.getSizeInMetersX(), costmap.getSizeInMetersY());
}

}  // namespace

LegacyPathHandler::LegacyPathHandler(
  const LegacyPathConfig & config, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: config_(config), tf_(std::move(tf)), costmap_ros_(std::move(costmap_ros))
{
  // FeasiblePathHandler's default: half the larger side of the costmap, at construction.
  if (config_.max_robot_pose_search_dist == 0.0) {
    config_.max_robot_pose_search_dist = costmapMaxExtent(*costmap_ros_);
  }
}

void LegacyPathHandler::setPlan(const nav_msgs::msg::Path & path) { plan_ = path; }

void LegacyPathHandler::computeWindow(
  const geometry_msgs::msg::PoseStamped & robot_pose, LegacyPathOutput * out)
{
  // 1. Empty and (optionally) unit plans.
  if (plan_.poses.empty()) {
    throw vectorfield_ros::compat::InvalidPath("Received plan with zero length");
  }
  if (config_.reject_unit_path && plan_.poses.size() == 1) {
    throw vectorfield_ros::compat::InvalidPath("Plan is too short");
  }

  // 2. One transform per cycle: costmap frame <- plan frame at the robot-pose stamp.
  const std::string & costmap_frame = costmap_ros_->getGlobalFrameID();
  try {
    out->costmap_from_plan = tf_->lookupTransform(
      costmap_frame, plan_.header.frame_id, rclcpp::Time(robot_pose.header.stamp),
      rclcpp::Duration::from_seconds(config_.transform_tolerance));
  } catch (const tf2::TransformException & e) {
    throw vectorfield_ros::compat::ControllerTFError(
      std::string("Unable to transform the global plan into the costmap frame: ") + e.what());
  }
  const vectorfield_core::Isometry2 costmap_from_plan =
    vectorfield_ros::toIsometry2(out->costmap_from_plan.transform);
  const vectorfield_core::Vec2 robot_in_plan = costmap_from_plan.applyInverse(
    vectorfield_core::Vec2(robot_pose.pose.position.x, robot_pose.pose.position.y));

  // 3. The closest pose within max_robot_pose_search_dist of integrated distance; keep >= 2
  // poses when the closest is the last one.
  std::vector<geometry_msgs::msg::PoseStamped> & poses = plan_.poses;
  const double search_distance = config_.max_robot_pose_search_dist < 0.0
                                   ? std::numeric_limits<double>::max()
                                   : config_.max_robot_pose_search_dist;
  const PoseIterator search_end =
    firstAfterIntegratedDistance(poses.begin(), poses.end(), search_distance);
  PoseIterator closest = closestPose(poses.begin(), search_end, robot_in_plan);
  if (
    poses.size() > 1 && search_end != poses.begin() && closest == std::prev(search_end) &&
    closest != poses.begin()) {
    closest = std::prev(closest);
  }

  // 4. The segment ends prune_distance of integrated distance after the closest pose.
  const PoseIterator segment_end =
    firstAfterIntegratedDistance(closest, poses.end(), config_.prune_distance);

  // 5. Transform the segment, cut at the first pose outside the costmap.
  nav2_costmap_2d::Costmap2D & costmap = *costmap_ros_->getCostmap();
  out->window.header.frame_id = costmap_frame;
  out->window.header.stamp = robot_pose.header.stamp;
  out->window.poses.clear();
  for (PoseIterator it = closest; it != segment_end; ++it) {
    geometry_msgs::msg::PoseStamped pose;
    tf2::doTransform(*it, pose, out->costmap_from_plan);
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      break;
    }
    pose.header.frame_id = costmap_frame;
    pose.header.stamp = robot_pose.header.stamp;
    pose.pose.position.z = 0.0;
    out->window.poses.push_back(pose);
  }

  // The goal: the last pose of the full plan, with the same transform.
  tf2::doTransform(poses.back(), out->goal, out->costmap_from_plan);
  out->goal.header.frame_id = costmap_frame;
  out->goal.header.stamp = robot_pose.header.stamp;
  out->goal.pose.position.z = 0.0;

  // 6. Prune the stored plan up to the closest pose (monotonic progress).
  poses.erase(poses.begin(), closest);

  if (out->window.poses.empty()) {
    throw vectorfield_ros::compat::InvalidPath("Resulting plan has 0 poses in it.");
  }
}

const nav_msgs::msg::Path & LegacyPathHandler::plan() const noexcept { return plan_; }

const LegacyPathConfig & LegacyPathHandler::config() const noexcept { return config_; }

}  // namespace vectorfield_nav2_controller
