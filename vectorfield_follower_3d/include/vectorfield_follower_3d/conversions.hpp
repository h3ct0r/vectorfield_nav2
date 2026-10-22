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
/// @brief Conversions between ROS messages and the 3D core types (experimental). Header-only and
/// pure.

#ifndef VECTORFIELD_FOLLOWER_3D__CONVERSIONS_HPP_
#define VECTORFIELD_FOLLOWER_3D__CONVERSIONS_HPP_

#include <cmath>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "vectorfield_core/d3/frame.hpp"
#include "vectorfield_core/d3/types.hpp"

namespace vectorfield_follower_3d
{

/// @brief The 3D core namespace.
namespace d3 = vectorfield_core::d3;

/// @brief A point message as a Vec3.
inline d3::Vec3 toVec3(const geometry_msgs::msg::Point & p) { return d3::Vec3(p.x, p.y, p.z); }

/// @brief A vector message as a Vec3.
inline d3::Vec3 toVec3(const geometry_msgs::msg::Vector3 & v) { return d3::Vec3(v.x, v.y, v.z); }

/// @brief A Vec3 as a point message.
inline geometry_msgs::msg::Point toPointMsg(const d3::Vec3 & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

/// @brief A Vec3 as a vector message.
inline geometry_msgs::msg::Vector3 toVector3Msg(const d3::Vec3 & v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
  return out;
}

/// @brief A quaternion message as an Eigen quaternion; an all-zero message is the identity
/// (the default-constructed message). Not normalised.
inline Eigen::Quaterniond toQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

/// @brief An Eigen quaternion as a message.
inline geometry_msgs::msg::Quaternion toQuaternionMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

/// @brief The yaw (rotation about +z of the ZYX Euler angles) of a quaternion message.
inline double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  const Eigen::Quaterniond u = toQuaternion(q).normalized();
  return std::atan2(
    2.0 * (u.w() * u.z() + u.x() * u.y()), 1.0 - 2.0 * (u.y() * u.y() + u.z() * u.z()));
}

/// @brief ZYX Euler angles (yaw, pitch, roll) of a rotation, the inverse of
/// d3::rotationZYX().
struct EulerZYX
{
  /// Rotation about z (rad).
  double yaw{0.0};
  /// Rotation about y (rad).
  double pitch{0.0};
  /// Rotation about x (rad).
  double roll{0.0};
};

/// @brief The ZYX Euler angles of a rotation matrix (pitch clamped to [-pi/2, pi/2]).
inline EulerZYX eulerZYX(const d3::Mat3 & r)
{
  EulerZYX out;
  const double s = std::fmax(-1.0, std::fmin(1.0, -r(2, 0)));
  out.pitch = std::asin(s);
  out.yaw = std::atan2(r(1, 0), r(0, 0));
  out.roll = std::atan2(r(2, 1), r(2, 2));
  return out;
}

/// @brief A pose message as an isometry (the quaternion is normalised; zero = identity).
inline d3::Isometry3 toIsometry3(const geometry_msgs::msg::Pose & pose)
{
  d3::Isometry3 out = d3::Isometry3::Identity();
  out.linear() = toQuaternion(pose.orientation).normalized().toRotationMatrix();
  out.translation() = toVec3(pose.position);
  return out;
}

/// @brief A transform message as an isometry.
inline d3::Isometry3 toIsometry3(const geometry_msgs::msg::Transform & transform)
{
  d3::Isometry3 out = d3::Isometry3::Identity();
  out.linear() = toQuaternion(transform.rotation).normalized().toRotationMatrix();
  out.translation() = toVec3(transform.translation);
  return out;
}

/// @brief An isometry as a pose message.
inline geometry_msgs::msg::Pose toPoseMsg(const d3::Isometry3 & T)
{
  geometry_msgs::msg::Pose out;
  out.position = toPointMsg(T.translation());
  out.orientation = toQuaternionMsg(Eigen::Quaterniond(T.rotation()));
  return out;
}

/// @brief The rotation by `yaw` about +z.
inline d3::Mat3 yawRotation(double yaw)
{
  return Eigen::AngleAxisd(yaw, d3::Vec3::UnitZ()).toRotationMatrix();
}

/// @brief The velocity setpoint of a core command: the world (ENU) velocity, or with `body` the
/// heading-frame (FLU) velocity \f$R_z(-\psi)\,v\f$. `angular.z` is the yaw rate in both.
inline geometry_msgs::msg::Twist toCommandMsg(const d3::Twist3 & cmd, bool body, double yaw)
{
  geometry_msgs::msg::Twist out;
  const d3::Vec3 v = body ? d3::Vec3(yawRotation(-yaw) * cmd.v) : cmd.v;
  out.linear = toVector3Msg(v);
  out.angular.z = cmd.wz;
  return out;
}

/// @brief A core twist as a message in the world frame (FieldState3).
inline geometry_msgs::msg::Twist toTwistMsg(const d3::Twist3 & twist)
{
  return toCommandMsg(twist, false, 0.0);
}

}  // namespace vectorfield_follower_3d

#endif  // VECTORFIELD_FOLLOWER_3D__CONVERSIONS_HPP_
