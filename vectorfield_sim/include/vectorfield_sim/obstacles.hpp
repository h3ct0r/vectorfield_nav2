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
/// @brief Unmapped obstacles of the simulator and the parsers of their parameters
/// (`circle_obstacles`, `box_obstacles`; plan §8, upstream_audit U37).
///
/// **Format (fixed at M3).** Both parameters are flat lists of numbers in the map frame:
/// - `circle_obstacles`: `[x, y, radius]` per circle, radius > 0;
/// - `box_obstacles`: `[x, y, size_x, size_y, yaw]` per box (centre, full side lengths along the
///   box axes, rotation about the centre), sizes > 0.
///
/// An empty list means no obstacles. A malformed list (a length that is not a multiple of the
/// stride, a non-finite value, a non-positive radius or size) is rejected as a whole, never
/// truncated. ROS-free.

#ifndef VECTORFIELD_SIM__OBSTACLES_HPP_
#define VECTORFIELD_SIM__OBSTACLES_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "vectorfield_core/types.hpp"

/// @brief Kinematic simulator of the vectorfield_nav2 demos and launch tests.
namespace vectorfield_sim
{

/// @brief A disc in the map frame.
struct Circle
{
  /// Centre (m).
  vectorfield_core::Vec2 center{vectorfield_core::Vec2::Zero()};
  /// Radius (m), > 0.
  double radius{0.0};
};

/// @brief An oriented rectangle in the map frame.
struct Box
{
  /// Centre (m).
  vectorfield_core::Vec2 center{vectorfield_core::Vec2::Zero()};
  /// Full side lengths along the box's own x and y axes (m), > 0.
  vectorfield_core::Vec2 size{vectorfield_core::Vec2::Zero()};
  /// Rotation of the box axes about the centre (rad).
  double yaw{0.0};
};

/// @brief Numbers per circle in `circle_obstacles`: x, y, radius.
inline constexpr std::size_t kCircleStride = 3;
/// @brief Numbers per box in `box_obstacles`: x, y, size_x, size_y, yaw.
inline constexpr std::size_t kBoxStride = 5;

/// @brief Parses a `circle_obstacles` list.
/// @param values Flat `[x, y, radius]` triples; empty means no circles.
/// @param out Receives the circles on success; unchanged on failure. Must not be null.
/// @param error Receives a one-line description on failure (may be null).
/// @return True when the whole list is valid.
bool parseCircleObstacles(
  const std::vector<double> & values, std::vector<Circle> * out, std::string * error);

/// @brief Parses a `box_obstacles` list.
/// @param values Flat `[x, y, size_x, size_y, yaw]` quintuples; empty means no boxes.
/// @param out Receives the boxes on success; unchanged on failure. Must not be null.
/// @param error Receives a one-line description on failure (may be null).
/// @return True when the whole list is valid.
bool parseBoxObstacles(
  const std::vector<double> & values, std::vector<Box> * out, std::string * error);

}  // namespace vectorfield_sim

#endif  // VECTORFIELD_SIM__OBSTACLES_HPP_
