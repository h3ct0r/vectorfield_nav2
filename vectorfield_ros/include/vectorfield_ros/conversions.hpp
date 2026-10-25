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
/// @brief Conversions between ROS messages and vectorfield_core types: poses, twists, paths,
/// `Curve.msg`, costmaps, footprints and goal-checker tolerances.
///
/// Implemented in src/conversions.cpp. Every function is free of distro
/// conditionals. Conversions that fill caller-owned buffers clear and refill them, so once their
/// capacity suffices they never allocate (the plugin calls them every cycle).
///
/// **Frames.** None of these functions transforms between frames: inputs and outputs are in the
/// same frame, which is the caller's business (the plugin works in the costmap global frame).

#ifndef VECTORFIELD_ROS__CONVERSIONS_HPP_
#define VECTORFIELD_ROS__CONVERSIONS_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_msgs/msg/curve.hpp"

namespace vectorfield_ros
{

/// @name Points, vectors, poses and twists
/// z components are dropped on the way in and set to 0 on the way out (everything is planar).
/// @{

/// @brief (x, y) of a point.
vectorfield_core::Vec2 toVec2(const geometry_msgs::msg::Point & point) noexcept;
/// @brief (x, y) of a float32 point (polygons, `published_footprint`).
vectorfield_core::Vec2 toVec2(const geometry_msgs::msg::Point32 & point) noexcept;
/// @brief (x, y) of a vector.
vectorfield_core::Vec2 toVec2(const geometry_msgs::msg::Vector3 & vector) noexcept;
/// @brief A point at height `z`.
geometry_msgs::msg::Point toPointMsg(const vectorfield_core::Vec2 & point, double z = 0.0) noexcept;
/// @brief A vector with z = 0.
geometry_msgs::msg::Vector3 toVector3Msg(const vectorfield_core::Vec2 & vector) noexcept;

/// @brief Yaw of a quaternion, in \f$(-\pi, \pi]\f$ (tf2::getYaw semantics on the normalised
/// quaternion).
/// @return NaN for a non-finite or zero-norm quaternion.
double yawOf(const geometry_msgs::msg::Quaternion & orientation) noexcept;
/// @brief The quaternion of a rotation by `yaw` about +z.
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) noexcept;

/// @brief Planar pose (x, y, yaw) of a pose message.
vectorfield_core::Pose2 toPose2(const geometry_msgs::msg::Pose & pose) noexcept;
/// @brief A pose message with z = 0 and a pure-yaw orientation.
geometry_msgs::msg::Pose toPoseMsg(const vectorfield_core::Pose2 & pose) noexcept;
/// @brief Body twist (linear.x, linear.y, angular.z).
vectorfield_core::Twist2 toTwist2(const geometry_msgs::msg::Twist & twist) noexcept;
/// @brief A twist message with only linear.x, linear.y and angular.z set.
geometry_msgs::msg::Twist toTwistMsg(const vectorfield_core::Twist2 & twist) noexcept;
/// @brief The planar part (x, y, yaw) of a transform, as `T_parent_child` (core naming:
/// `Isometry2::apply()` maps child coordinates into the parent frame).
vectorfield_core::Isometry2 toIsometry2(const geometry_msgs::msg::Transform & transform) noexcept;
/// @}

/// @name Paths
/// @{

/// @brief Planar poses of a path (frame and stamps ignored).
/// @param path Input path.
/// @param out Cleared and refilled, one entry per pose.
void toPoses(const nav_msgs::msg::Path & path, std::vector<vectorfield_core::Pose2> * out);

/// @brief Positions of a path (frame and stamps ignored).
/// @param path Input path.
/// @param out Cleared and refilled, one entry per pose.
void toPositions(const nav_msgs::msg::Path & path, std::vector<vectorfield_core::Vec2> * out);
/// @}

/// @name Curve.msg ↔ CurveSpec
/// @{

/// @brief The core specification of a `Curve.msg` (field for field; `points` lose z).
///
/// Performs no validation beyond the type code: `makeCurve` is the single validation point
///. The header is not used: the follower resolves the frame and the time origin.
/// @param msg The curve message.
/// @param t0 Time origin of \f$r(s, t)\f$ (s), resolved by the caller (the header stamp, or the
/// goal acceptance time when the stamp is zero).
/// @throws vectorfield_core::CurveError with `kInvalidCurve` for an unknown `type`.
vectorfield_core::CurveSpec toCurveSpec(const vectorfield_msgs::msg::Curve & msg, double t0);

/// @brief The `Curve.msg` of a specification (the follower's `~/curve` topic); inverse of
/// toCurveSpec() for every field except `t0`, which the caller encodes in `header.stamp`.
/// @param spec The specification.
/// @param header Header to copy into the message.
vectorfield_msgs::msg::Curve toCurveMsg(
  const vectorfield_core::CurveSpec & spec, const std_msgs::msg::Header & header);
/// @}

/// @name Costmaps → OccupancyGrid
/// The obstacle sites are the lethal cells (`LETHAL_OBSTACLE`, 254), plus the unknown cells
/// (`NO_INFORMATION`, 255) when `unknown_is_obstacle` (math_spec §6.1). Inflated costs are never
/// sites: the core works on its own EDT, never on inflation costs (nav2_api_matrix.md §6.3).
/// @{

/// @brief Whether a Nav2 cost value is an obstacle site.
constexpr bool isObstacleSite(std::uint8_t cost, bool unknown_is_obstacle) noexcept
{
  return cost == 254U || (unknown_is_obstacle && cost == 255U);
}

/// @brief Plugin: snapshot of a `Costmap2D` into an OccupancyGrid.
///
/// Locks `costmap.getMutex()` only while copying the cell values into `mask` (no allocation
/// once its capacity suffices), then builds the site mask and calls
/// `OccupancyGrid::setFromMask` outside the lock. The grid's version changes only when the
/// geometry or a site flag changed, so `EuclideanDistanceTransform::update` rebuilds only then.
/// @param costmap The costmap (non-const only because Nav2's getMutex() is non-const).
/// @param unknown_is_obstacle `unknown_is_obstacle`.
/// @param mask Caller-owned work buffer (resized to the cell count).
/// @param grid Output; unchanged on failure.
/// @return False for an empty costmap or a rejected geometry (grid unchanged).
bool toOccupancyGrid(
  nav2_costmap_2d::Costmap2D & costmap, bool unknown_is_obstacle, std::vector<std::uint8_t> * mask,
  vectorfield_core::OccupancyGrid * grid);

/// @brief Follower: a `nav2_msgs/Costmap` (`costmap_raw`) into an OccupancyGrid.
///
/// Same site rule. The metadata origin must have an identity orientation (Nav2 costmaps are
/// axis-aligned); a rotated origin is rejected.
/// @param costmap The costmap message.
/// @param unknown_is_obstacle `unknown_is_obstacle`.
/// @param mask Caller-owned work buffer.
/// @param grid Output; unchanged on failure.
/// @return False for inconsistent sizes, an empty map or a rotated origin (grid unchanged).
bool toOccupancyGrid(
  const nav2_msgs::msg::Costmap & costmap, bool unknown_is_obstacle,
  std::vector<std::uint8_t> * mask, vectorfield_core::OccupancyGrid * grid);

/// @brief Costmap geometry for the costmap-dependent validation rules.
/// @param costmap The costmap (size in metres and resolution).
/// @param window_limit Gen A/B: LegacyPathHandler's `prune_distance`; Gen C and the follower:
/// nullopt.
vectorfield_core::CostmapInfo toCostmapInfo(
  const nav2_costmap_2d::Costmap2D & costmap, std::optional<double> window_limit);

/// @brief Costmap geometry of a `nav2_msgs/Costmap`'s metadata (follower).
vectorfield_core::CostmapInfo toCostmapInfo(
  const nav2_msgs::msg::CostmapMetaData & metadata, std::optional<double> window_limit);
/// @}

/// @name Footprints
/// The robot-frame footprint is node-derived Tracker configuration: the plugin uses
/// `Costmap2DROS::getRobotFootprint()` (padded, robot frame; a polygon even for a circular
/// robot); the follower transforms each `published_footprint` into the base frame first.
/// @{

/// @brief The core footprint of a robot-frame polygon.
///
/// A polygon with at least 3 vertices becomes `FootprintSpec::polygon`; an empty input becomes a
/// point robot (`radius` 0). One or two vertices are kept as given, so validateConfig() reports
/// `invalid_footprint`.
vectorfield_core::FootprintSpec toFootprintSpec(
  const std::vector<geometry_msgs::msg::Point> & polygon);

/// @brief Polygon vertices as Vec2 (z dropped).
/// @param polygon Input polygon.
/// @param out Cleared and refilled.
void toVertices(
  const std::vector<geometry_msgs::msg::Point> & polygon,
  std::vector<vectorfield_core::Vec2> * out);

/// @brief Whether a footprint changed: a different vertex count, or any vertex moved by
/// more than `tolerance` (plugin: 1 mm; follower: 1 cm).
bool footprintChanged(
  vectorfield_core::Span<const vectorfield_core::Vec2> previous,
  vectorfield_core::Span<const vectorfield_core::Vec2> current, double tolerance) noexcept;
/// @}

/// @name Goal-checker tolerances
/// @{

/// @brief Goal tolerances decoded from `GoalChecker::getTolerances()` (nav2_api_matrix.md §3.2).
struct GoalTolerances
{
  /// XY tolerance (m): the smaller of `position.x` and `position.y` when they are measured;
  /// otherwise 0.25 (Nav2's stock `xy_goal_tolerance`) with `xy_valid` false.
  double xy{0.25};
  /// Yaw tolerance (rad): the magnitude of the yaw of `orientation`; 0 means unchecked
  /// (PositionGoalChecker reports identity), as does an unmeasured or non-finite orientation.
  double yaw{0.0};
  /// The checker measured the XY tolerance.
  bool xy_valid{false};
};

/// @brief Decodes the pose tolerance of `compat::get_goal_tolerances()`. Fields equal to
/// `std::numeric_limits<double>::lowest()` or non-finite are "not measured" (nav2_core contract).
GoalTolerances toGoalTolerances(const geometry_msgs::msg::Pose & pose_tolerance) noexcept;
/// @}

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__CONVERSIONS_HPP_
