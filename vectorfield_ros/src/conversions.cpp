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

// Conversions between ROS messages and vectorfield_core types.
//
// Costmap sizes in metres are cells × resolution in both toCostmapInfo() overloads (the extent
// the grid covers), not Costmap2D::getSizeInMetersX(), which is half a cell short; so the plugin
// and the follower compute the same CostmapInfo for the same costmap.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "vectorfield_core/errors.hpp"
#include "vectorfield_ros/conversions.hpp"

namespace vectorfield_ros
{

namespace vf = vectorfield_core;

namespace
{

/// Largest |x|, |y|, |z| of a normalised quaternion still taken as the identity: a rotation of
/// at most 2e-6 rad (costmap origins).
constexpr double kIdentityTolerance = 1e-6;

/// Whether a goal-checker tolerance field was measured (nav2_core: lowest() means "not measured";
/// a negative or non-finite tolerance is meaningless, so it is treated the same way).
bool isMeasured(double value) noexcept { return std::isfinite(value) && value >= 0.0; }

/// Whether a quaternion is finite with a non-zero norm.
bool isUsable(const geometry_msgs::msg::Quaternion & q, double * norm) noexcept
{
  if (!(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w))) {
    return false;
  }
  *norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(*norm) && *norm > 0.0;
}

/// Grid dimensions as ints for OccupancyGrid; false when empty or too large.
bool toGridSize(std::uint64_t size_x, std::uint64_t size_y, int * width, int * height) noexcept
{
  constexpr auto kMax = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
  if (size_x == 0U || size_y == 0U || size_x > kMax || size_y > kMax || size_x * size_y > kMax) {
    return false;
  }
  *width = static_cast<int>(size_x);
  *height = static_cast<int>(size_y);
  return true;
}

}  // namespace

vf::Vec2 toVec2(const geometry_msgs::msg::Point & point) noexcept
{
  return vf::Vec2(point.x, point.y);
}

vf::Vec2 toVec2(const geometry_msgs::msg::Point32 & point) noexcept
{
  return vf::Vec2(static_cast<double>(point.x), static_cast<double>(point.y));
}

vf::Vec2 toVec2(const geometry_msgs::msg::Vector3 & vector) noexcept
{
  return vf::Vec2(vector.x, vector.y);
}

geometry_msgs::msg::Point toPointMsg(const vf::Vec2 & point, double z) noexcept
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = z;
  return msg;
}

geometry_msgs::msg::Vector3 toVector3Msg(const vf::Vec2 & vector) noexcept
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = 0.0;
  return msg;
}

double yawOf(const geometry_msgs::msg::Quaternion & orientation) noexcept
{
  double norm = 0.0;
  if (!isUsable(orientation, &norm)) {
    return vf::kNaN;
  }
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  // tf2::getYaw: the yaw of the ZYX Euler angles, with its gimbal-lock branches.
  const double sin_pitch = -2.0 * (x * z - w * y);
  double yaw = 0.0;
  if (sin_pitch <= -0.99999) {
    yaw = -2.0 * std::atan2(y, x);
  } else if (sin_pitch >= 0.99999) {
    yaw = 2.0 * std::atan2(y, x);
  } else {
    yaw = std::atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
  }
  return vf::wrapAngle(yaw);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) noexcept
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = 0.0;
  msg.y = 0.0;
  msg.z = std::sin(0.5 * yaw);
  msg.w = std::cos(0.5 * yaw);
  return msg;
}

vf::Pose2 toPose2(const geometry_msgs::msg::Pose & pose) noexcept
{
  return vf::Pose2{toVec2(pose.position), yawOf(pose.orientation)};
}

geometry_msgs::msg::Pose toPoseMsg(const vf::Pose2 & pose) noexcept
{
  geometry_msgs::msg::Pose msg;
  msg.position = toPointMsg(pose.p);
  msg.orientation = yawToQuaternion(pose.yaw);
  return msg;
}

vf::Twist2 toTwist2(const geometry_msgs::msg::Twist & twist) noexcept
{
  return vf::Twist2{twist.linear.x, twist.linear.y, twist.angular.z};
}

geometry_msgs::msg::Twist toTwistMsg(const vf::Twist2 & twist) noexcept
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = twist.vx;
  msg.linear.y = twist.vy;
  msg.angular.z = twist.wz;
  return msg;
}

vf::Isometry2 toIsometry2(const geometry_msgs::msg::Transform & transform) noexcept
{
  return vf::Isometry2(yawOf(transform.rotation), toVec2(transform.translation));
}

void toPoses(const nav_msgs::msg::Path & path, std::vector<vf::Pose2> * out)
{
  out->clear();
  out->reserve(path.poses.size());
  for (const auto & stamped : path.poses) {
    out->push_back(toPose2(stamped.pose));
  }
}

void toPositions(const nav_msgs::msg::Path & path, std::vector<vf::Vec2> * out)
{
  out->clear();
  out->reserve(path.poses.size());
  for (const auto & stamped : path.poses) {
    out->push_back(toVec2(stamped.pose.position));
  }
}

vf::CurveSpec toCurveSpec(const vectorfield_msgs::msg::Curve & msg, double t0)
{
  using Curve = vectorfield_msgs::msg::Curve;
  vf::CurveSpec spec;
  switch (msg.type) {
    case Curve::POLYLINE:
      spec.type = vf::CurveSpec::Type::kPolyline;
      break;
    case Curve::EXPRESSION:
      spec.type = vf::CurveSpec::Type::kExpression;
      break;
    case Curve::BUILTIN:
      spec.type = vf::CurveSpec::Type::kBuiltin;
      break;
    default:
      throw vf::CurveError(
        vf::CurveError::Code::kInvalidCurve, "unknown curve type " +
                                               std::to_string(static_cast<unsigned int>(msg.type)) +
                                               " (POLYLINE = 0, EXPRESSION = 1, BUILTIN = 2)");
  }
  spec.closed = msg.closed;
  spec.points.reserve(msg.points.size());
  for (const auto & point : msg.points) {
    spec.points.push_back(toVec2(point));
  }
  spec.x_expression = msg.x_expression;
  spec.y_expression = msg.y_expression;
  spec.s_min = msg.s_min;
  spec.s_max = msg.s_max;
  spec.builtin_shape = msg.builtin_shape;
  spec.parameter_names = msg.parameter_names;
  spec.parameter_values = msg.parameter_values;
  spec.sample_spacing = msg.sample_spacing;
  spec.max_corner_deviation = msg.max_corner_deviation;
  spec.t0 = t0;
  return spec;
}

vectorfield_msgs::msg::Curve toCurveMsg(
  const vf::CurveSpec & spec, const std_msgs::msg::Header & header)
{
  vectorfield_msgs::msg::Curve msg;
  msg.header = header;
  msg.type = static_cast<std::uint8_t>(spec.type);
  msg.closed = spec.closed;
  msg.points.reserve(spec.points.size());
  for (const auto & point : spec.points) {
    msg.points.push_back(toPointMsg(point));
  }
  msg.x_expression = spec.x_expression;
  msg.y_expression = spec.y_expression;
  msg.s_min = spec.s_min;
  msg.s_max = spec.s_max;
  msg.builtin_shape = spec.builtin_shape;
  msg.parameter_names = spec.parameter_names;
  msg.parameter_values = spec.parameter_values;
  msg.sample_spacing = spec.sample_spacing;
  msg.max_corner_deviation = spec.max_corner_deviation;
  return msg;
}

bool toOccupancyGrid(
  nav2_costmap_2d::Costmap2D & costmap, bool unknown_is_obstacle, std::vector<std::uint8_t> * mask,
  vf::OccupancyGrid * grid)
{
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  vf::Vec2 origin = vf::Vec2::Zero();
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
    const unsigned char * costs = costmap.getCharMap();
    if (
      costs == nullptr ||
      !toGridSize(costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), &width, &height)) {
      return false;
    }
    const auto count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    mask->resize(count);
    std::copy(costs, costs + count, mask->begin());
    resolution = costmap.getResolution();
    origin = vf::Vec2(costmap.getOriginX(), costmap.getOriginY());
  }
  for (std::uint8_t & cell : *mask) {
    cell = isObstacleSite(cell, unknown_is_obstacle) ? 1U : 0U;
  }
  return grid->setFromMask(
    vf::Span<const std::uint8_t>(mask->data(), mask->size()), width, height, resolution, origin);
}

bool toOccupancyGrid(
  const nav2_msgs::msg::Costmap & costmap, bool unknown_is_obstacle,
  std::vector<std::uint8_t> * mask, vf::OccupancyGrid * grid)
{
  const nav2_msgs::msg::CostmapMetaData & metadata = costmap.metadata;
  int width = 0;
  int height = 0;
  if (!toGridSize(metadata.size_x, metadata.size_y, &width, &height)) {
    return false;
  }
  const auto count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (costmap.data.size() != count) {
    return false;
  }
  double norm = 0.0;
  const geometry_msgs::msg::Quaternion & q = metadata.origin.orientation;
  if (
    !isUsable(q, &norm) || std::abs(q.x) > kIdentityTolerance * norm ||
    std::abs(q.y) > kIdentityTolerance * norm || std::abs(q.z) > kIdentityTolerance * norm) {
    return false;
  }
  mask->resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    (*mask)[i] = isObstacleSite(costmap.data[i], unknown_is_obstacle) ? 1U : 0U;
  }
  return grid->setFromMask(
    vf::Span<const std::uint8_t>(mask->data(), mask->size()), width, height,
    static_cast<double>(metadata.resolution), toVec2(metadata.origin.position));
}

vf::CostmapInfo toCostmapInfo(
  const nav2_costmap_2d::Costmap2D & costmap, std::optional<double> window_limit)
{
  vf::CostmapInfo info;
  info.resolution = costmap.getResolution();
  info.width_m = static_cast<double>(costmap.getSizeInCellsX()) * info.resolution;
  info.height_m = static_cast<double>(costmap.getSizeInCellsY()) * info.resolution;
  info.window_limit = window_limit;
  return info;
}

vf::CostmapInfo toCostmapInfo(
  const nav2_msgs::msg::CostmapMetaData & metadata, std::optional<double> window_limit)
{
  vf::CostmapInfo info;
  info.resolution = static_cast<double>(metadata.resolution);
  info.width_m = static_cast<double>(metadata.size_x) * info.resolution;
  info.height_m = static_cast<double>(metadata.size_y) * info.resolution;
  info.window_limit = window_limit;
  return info;
}

vf::FootprintSpec toFootprintSpec(const std::vector<geometry_msgs::msg::Point> & polygon)
{
  vf::FootprintSpec spec;
  // An empty polygon is a point robot; otherwise the radius is unused.
  spec.radius = 0.0;
  toVertices(polygon, &spec.polygon);
  return spec;
}

void toVertices(const std::vector<geometry_msgs::msg::Point> & polygon, std::vector<vf::Vec2> * out)
{
  out->clear();
  out->reserve(polygon.size());
  for (const auto & point : polygon) {
    out->push_back(toVec2(point));
  }
}

bool footprintChanged(
  vf::Span<const vf::Vec2> previous, vf::Span<const vf::Vec2> current, double tolerance) noexcept
{
  if (previous.size() != current.size()) {
    return true;
  }
  for (std::size_t i = 0; i < current.size(); ++i) {
    // Negated so that a non-finite vertex counts as a change (and is then validated).
    if (!((current[i] - previous[i]).norm() <= tolerance)) {
      return true;
    }
  }
  return false;
}

GoalTolerances toGoalTolerances(const geometry_msgs::msg::Pose & pose_tolerance) noexcept
{
  GoalTolerances tolerances;
  const double x = pose_tolerance.position.x;
  const double y = pose_tolerance.position.y;
  if (isMeasured(x) && isMeasured(y)) {
    tolerances.xy = std::min(x, y);
    tolerances.xy_valid = true;
  } else if (isMeasured(x) || isMeasured(y)) {
    tolerances.xy = isMeasured(x) ? x : y;
    tolerances.xy_valid = true;
  }
  const geometry_msgs::msg::Quaternion & q = pose_tolerance.orientation;
  const double lowest = std::numeric_limits<double>::lowest();
  if (q.x != lowest && q.y != lowest && q.z != lowest && q.w != lowest) {
    const double yaw = yawOf(q);
    tolerances.yaw = std::isfinite(yaw) ? std::abs(yaw) : 0.0;
  }
  return tolerances;
}

}  // namespace vectorfield_ros
