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
/// @brief The simulated laser: exact ray casting against an occupancy grid (cell-boundary DDA)
/// plus the unmapped circles and boxes (plan §8). ROS-free.

#ifndef VECTORFIELD_SIM__RAY_CASTER_HPP_
#define VECTORFIELD_SIM__RAY_CASTER_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "vectorfield_core/types.hpp"
#include "vectorfield_sim/obstacles.hpp"

namespace vectorfield_sim
{

/// @brief A ROS-free copy of a `nav_msgs/OccupancyGrid` in the map frame.
struct GridMap
{
  /// Number of columns (cells along the grid's x axis).
  int width{0};
  /// Number of rows.
  int height{0};
  /// Cell side (m), > 0.
  double resolution{0.05};
  /// Pose of the outer corner of cell (0, 0) in the map frame (`info.origin`).
  vectorfield_core::Pose2 origin;
  /// Row-major cell values: −1 unknown, 0 (free) to 100 (occupied).
  std::vector<std::int8_t> data;
  /// A cell is occupied when its value is at least this (map_server's trinary maps use 100).
  std::int8_t occupied_threshold{65};

  /// @brief True when the dimensions are positive and `data` has width × height cells.
  bool valid() const noexcept;
  /// @brief True for an in-bounds cell whose value is >= occupied_threshold (unknown is free).
  bool occupied(int ix, int iy) const noexcept;
};

/// @brief Beam layout of a scan: `beams` rays from `angle_min` in steps of `angle_increment`,
/// in the sensor frame.
struct ScanGeometry
{
  /// Number of beams.
  int beams{360};
  /// Angle of beam 0 (rad).
  double angle_min{-vectorfield_core::kPi};
  /// Angle between consecutive beams (rad).
  double angle_increment{vectorfield_core::kTwoPi / 360.0};
  /// Maximum range (m); nothing beyond it is reported.
  double range_max{30.0};

  /// @brief A full turn of `beams` evenly spaced beams starting at −π (no duplicate at +π).
  static ScanGeometry fullCircle(int beams, double range_max) noexcept;
  /// @brief Angle of beam i (rad, sensor frame).
  double angle(int i) const noexcept { return angle_min + angle_increment * i; }
  /// @brief Angle of the last beam (`LaserScan::angle_max`).
  double angleMax() const noexcept { return angle(beams - 1); }
};

/// @brief Exact ray casting against a grid and analytic shapes.
///
/// The grid is traversed cell by cell along the ray (Amanatides–Woo), so the distance to a wall
/// is the exact distance to the boundary of its first occupied cell; circles and boxes are
/// intersected analytically. A ray that starts inside an occupied cell or a shape returns 0.
/// Not thread-safe; a plain value type.
class RayCaster
{
public:
  /// @brief Replaces the grid (an invalid grid is treated as no grid).
  void setMap(GridMap map);
  /// @brief Removes the grid.
  void clearMap() noexcept { map_ = GridMap{}; }
  /// @brief True when a valid grid is set.
  bool hasMap() const noexcept { return map_.valid(); }
  /// @brief Replaces the unmapped circles.
  void setCircles(std::vector<Circle> circles) { circles_ = std::move(circles); }
  /// @brief Replaces the unmapped boxes.
  void setBoxes(std::vector<Box> boxes) { boxes_ = std::move(boxes); }
  /// @brief The unmapped circles.
  const std::vector<Circle> & circles() const noexcept { return circles_; }
  /// @brief The unmapped boxes.
  const std::vector<Box> & boxes() const noexcept { return boxes_; }

  /// @brief Distance from `origin` along the map-frame direction `angle` to the nearest hit.
  /// @param origin Ray origin (map frame, m).
  /// @param angle Ray direction (rad, map frame).
  /// @param max_range Maximum range (m).
  /// @return The distance in [0, max_range]; +infinity when nothing is hit within max_range.
  double cast(const vectorfield_core::Vec2 & origin, double angle, double max_range) const;

  /// @brief Casts every beam of `geometry` from a sensor at `sensor_pose` (map frame).
  /// @param sensor_pose Sensor pose in the map frame.
  /// @param geometry Beam layout (sensor frame).
  /// @return One range per beam, as cast() returns them.
  std::vector<double> castScan(
    const vectorfield_core::Pose2 & sensor_pose, const ScanGeometry & geometry) const;

private:
  double castGrid(const vectorfield_core::Vec2 & origin, double angle, double max_range) const;

  GridMap map_;
  std::vector<Circle> circles_;
  std::vector<Box> boxes_;
};

/// @brief Ray–disc intersection.
/// @param origin Ray origin.
/// @param direction Unit direction.
/// @param circle The disc.
/// @return Entry distance (0 when the origin is inside); +infinity when the ray misses.
double intersectCircle(
  const vectorfield_core::Vec2 & origin, const vectorfield_core::Vec2 & direction,
  const Circle & circle) noexcept;

/// @brief Ray–oriented-rectangle intersection (slab test in the box frame).
/// @param origin Ray origin.
/// @param direction Unit direction.
/// @param box The rectangle.
/// @return Entry distance (0 when the origin is inside); +infinity when the ray misses.
double intersectBox(
  const vectorfield_core::Vec2 & origin, const vectorfield_core::Vec2 & direction,
  const Box & box) noexcept;

}  // namespace vectorfield_sim

#endif  // VECTORFIELD_SIM__RAY_CASTER_HPP_
