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
/// @brief 3D core (experimental): vectors, poses, twists and small helpers (math_spec §14).
///
/// The 3D core lives in namespace `vectorfield_core::d3`, compiled into the same library as the
/// planar core. It reuses the scalar pieces of the planar core (GainFunction, TerminalProfile,
/// Status, Phase, CurveOptions, the numeric constants of types.hpp). Every 3D type carries a `3`
/// suffix, so `using namespace` on both namespaces is never ambiguous. **Experimental in 0.x**:
/// the API may change in a minor release.

#ifndef VECTORFIELD_CORE__D3__TYPES_HPP_
#define VECTORFIELD_CORE__D3__TYPES_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "vectorfield_core/types.hpp"

namespace vectorfield_core::d3
{

/// @brief A point or free vector in \f$\mathbb R^3\f$ (m, m/s).
using Vec3 = Eigen::Vector3d;
/// @brief A 3x3 matrix (rotations, projectors).
using Mat3 = Eigen::Matrix3d;

/// @brief Position and yaw of a multirotor-like body (roll and pitch are the attitude loop's).
struct Pose3
{
  /// Position \f$p\f$ (m).
  Vec3 p{Vec3::Zero()};
  /// Yaw \f$\psi\f$ (rad) about +z. Not required to be wrapped.
  double yaw{0.0};
};

/// @brief A holonomic command: linear velocity in the curve frame and yaw rate.
struct Twist3
{
  /// Linear velocity \f$v\f$ (m/s), in the curve (world) frame, not the body frame.
  Vec3 v{Vec3::Zero()};
  /// Yaw rate \f$\omega\f$ (rad/s).
  double wz{0.0};
};

/// @brief True iff every component is finite.
inline bool isFinite(const Vec3 & v) noexcept
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

/// @brief True iff position and yaw are finite.
inline bool isFinite(const Pose3 & pose) noexcept
{
  return isFinite(pose.p) && std::isfinite(pose.yaw);
}

/// @brief True iff every component is finite.
inline bool isFinite(const Twist3 & u) noexcept { return isFinite(u.v) && std::isfinite(u.wz); }

/// @brief `v / ‖v‖`, or `fallback` when the norm is zero or non-finite.
inline Vec3 unitOr(const Vec3 & v, const Vec3 & fallback) noexcept
{
  const double norm = v.norm();
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return fallback;
  }
  return v / norm;
}

/// @brief \f$\Pi_n v = v - (n^\top v)\,n\f$ for a unit `n` (the component of v orthogonal to n).
inline Vec3 rejectFrom(const Vec3 & v, const Vec3 & n) noexcept { return v - n.dot(v) * n; }

/// @brief Rotation \f$R_z(\psi)\,R_y(\theta)\,R_x(\phi)\f$ (intrinsic Z-Y'-X'', i.e. yaw, then
/// pitch, then roll; the ROS RPY convention).
/// @param yaw \f$\psi\f$ (rad).
/// @param pitch \f$\theta\f$ (rad).
/// @param roll \f$\phi\f$ (rad).
inline Mat3 rotationZYX(double yaw, double pitch, double roll) noexcept
{
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  Mat3 r;
  r << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,  //
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,     //
    -sp, cp * sr, cp * cr;
  return r;
}

/// @brief A rigid placement \f$x \mapsto R\,x + c\f$ of a curve (built-in pose parameters).
struct Placement3
{
  /// Rotation \f$R\f$.
  Mat3 R{Mat3::Identity()};
  /// Translation \f$c\f$ (m).
  Vec3 c{Vec3::Zero()};
  /// @brief \f$R\,x + c\f$.
  Vec3 apply(const Vec3 & x) const noexcept { return R * x + c; }
  /// @brief \f$R\,v\f$ (free vectors).
  Vec3 rotate(const Vec3 & v) const noexcept { return R * v; }
};

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__TYPES_HPP_
