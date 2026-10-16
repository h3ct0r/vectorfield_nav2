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
/// @brief RayCaster against analytic distances: grid walls (including an offset or rotated
/// origin and a ray that starts outside the grid), circles, boxes, max range and the beam layout.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "vectorfield_core/types.hpp"
#include "vectorfield_sim/obstacles.hpp"
#include "vectorfield_sim/ray_caster.hpp"

namespace
{

using vectorfield_core::kPi;
using vectorfield_core::kTwoPi;
using vectorfield_core::Pose2;
using vectorfield_core::Vec2;
using vectorfield_sim::Box;
using vectorfield_sim::Circle;
using vectorfield_sim::GridMap;
using vectorfield_sim::RayCaster;
using vectorfield_sim::ScanGeometry;

constexpr double kTol = 1e-9;

/// A free width × height grid at `resolution` whose cell (0, 0) corner is at `origin`.
GridMap freeGrid(int width, int height, double resolution, const Pose2 & origin = {})
{
  GridMap map;
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.origin = origin;
  map.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
  return map;
}

void setCell(GridMap * map, int ix, int iy, std::int8_t value)
{
  map->data
    [static_cast<std::size_t>(iy) * static_cast<std::size_t>(map->width) +
     static_cast<std::size_t>(ix)] = value;
}

void fillColumn(GridMap * map, int ix, std::int8_t value = 100)
{
  for (int iy = 0; iy < map->height; ++iy) {
    setCell(map, ix, iy, value);
  }
}

TEST(RayCaster, NothingToHitIsInfinite)
{
  const RayCaster caster;
  EXPECT_FALSE(caster.hasMap());
  EXPECT_TRUE(std::isinf(caster.cast(Vec2(0.0, 0.0), 0.3, 10.0)));
}

TEST(RayCaster, WallDistanceIsExact)
{
  // 5 m x 5 m at 0.05 m; the wall's face is the left boundary of column 60, x = 3.0.
  GridMap map = freeGrid(100, 100, 0.05);
  fillColumn(&map, 60);
  RayCaster caster;
  caster.setMap(map);
  ASSERT_TRUE(caster.hasMap());
  const Vec2 origin(1.0, 2.5);
  for (double angle = -0.5; angle <= 0.5; angle += 0.05) {
    EXPECT_NEAR(caster.cast(origin, angle, 10.0), 2.0 / std::cos(angle), kTol) << angle;
  }
  // Away from the wall the ray leaves the grid.
  EXPECT_TRUE(std::isinf(caster.cast(origin, kPi, 10.0)));
  // A ray that starts on the boundary x = 1.0 of occupied column 20 is blocked only when it
  // moves into it; inside the column it is blocked at once.
  fillColumn(&map, 20);
  caster.setMap(map);
  EXPECT_DOUBLE_EQ(caster.cast(Vec2(1.0, 2.5), 0.0, 10.0), 0.0);
  EXPECT_TRUE(std::isinf(caster.cast(Vec2(1.0, 2.5), kPi, 10.0)));
  EXPECT_DOUBLE_EQ(caster.cast(Vec2(1.02, 2.5), kPi, 10.0), 0.0);
  EXPECT_NEAR(caster.cast(Vec2(0.5, 2.5), 0.0, 10.0), 0.5, kTol);
}

TEST(RayCaster, OccupancyThresholdAndUnknownCells)
{
  GridMap map = freeGrid(100, 20, 0.05);
  fillColumn(&map, 40, -1);  // unknown: free
  fillColumn(&map, 50, 64);  // below the default threshold of 65
  fillColumn(&map, 60, 65);
  RayCaster caster;
  caster.setMap(map);
  EXPECT_NEAR(caster.cast(Vec2(0.5, 0.5), 0.0, 10.0), 2.5, kTol);
  map.occupied_threshold = 50;
  caster.setMap(map);
  EXPECT_NEAR(caster.cast(Vec2(0.5, 0.5), 0.0, 10.0), 2.0, kTol);
}

TEST(RayCaster, OffsetOriginAndRayFromOutsideTheGrid)
{
  // The grid covers [-2, 3] x [-2, 3]; the wall's face is at x = -2 + 3.0 = 1.0.
  GridMap map = freeGrid(100, 100, 0.05, Pose2{Vec2(-2.0, -2.0), 0.0});
  fillColumn(&map, 60);
  RayCaster caster;
  caster.setMap(map);
  EXPECT_NEAR(caster.cast(Vec2(-3.0, 0.5), 0.0, 10.0), 4.0, kTol);
  EXPECT_NEAR(caster.cast(Vec2(-3.0, 0.5), 0.2, 10.0), 4.0 / std::cos(0.2), kTol);
  // Rays that miss the grid entirely.
  EXPECT_TRUE(std::isinf(caster.cast(Vec2(-3.0, 0.5), kPi, 10.0)));
  EXPECT_TRUE(std::isinf(caster.cast(Vec2(-3.0, 5.0), 0.0, 10.0)));
}

TEST(RayCaster, RotatedOrigin)
{
  // Origin yaw pi/2: grid x runs along map y, so column 40 is the line map y = 2.0.
  GridMap map = freeGrid(100, 100, 0.05, Pose2{Vec2(0.0, 0.0), 0.5 * kPi});
  fillColumn(&map, 40);
  RayCaster caster;
  caster.setMap(map);
  EXPECT_NEAR(caster.cast(Vec2(-2.5, 0.5), 0.5 * kPi, 10.0), 1.5, kTol);
  EXPECT_NEAR(caster.cast(Vec2(-2.5, 0.5), 0.5 * kPi + 0.3, 10.0), 1.5 / std::cos(0.3), kTol);
}

TEST(RayCaster, CircleDistanceIsExact)
{
  RayCaster caster;
  caster.setCircles({Circle{Vec2(3.0, 0.0), 0.5}});
  for (double angle = -0.15; angle <= 0.15; angle += 0.01) {
    const double offset = 3.0 * std::sin(angle);
    const double expected = 3.0 * std::cos(angle) - std::sqrt(0.25 - offset * offset);
    EXPECT_NEAR(caster.cast(Vec2::Zero(), angle, 10.0), expected, kTol) << angle;
  }
  EXPECT_TRUE(std::isinf(caster.cast(Vec2::Zero(), 0.2, 10.0)));
  EXPECT_TRUE(std::isinf(caster.cast(Vec2::Zero(), kPi, 10.0)));
  // Inside the disc.
  EXPECT_DOUBLE_EQ(caster.cast(Vec2(3.1, 0.0), 1.0, 10.0), 0.0);
}

TEST(RayCaster, BoxDistanceIsExact)
{
  RayCaster caster;
  caster.setBoxes({Box{Vec2(2.0, 0.0), Vec2(1.0, 0.6), 0.0}});
  EXPECT_NEAR(caster.cast(Vec2::Zero(), 0.0, 10.0), 1.5, kTol);
  EXPECT_NEAR(caster.cast(Vec2::Zero(), 0.1, 10.0), 1.5 / std::cos(0.1), kTol);
  EXPECT_NEAR(caster.cast(Vec2(2.0, -2.0), 0.5 * kPi, 10.0), 1.7, kTol);
  EXPECT_TRUE(std::isinf(caster.cast(Vec2(0.0, 0.5), 0.0, 10.0)));
  EXPECT_DOUBLE_EQ(caster.cast(Vec2(2.2, 0.1), 2.0, 10.0), 0.0);

  // Rotated by 45 degrees: the nearest corner is at x = 3 - sqrt(2)/2.
  caster.setBoxes({Box{Vec2(3.0, 0.0), Vec2(1.0, 1.0), 0.25 * kPi}});
  EXPECT_NEAR(caster.cast(Vec2::Zero(), 0.0, 10.0), 3.0 - 0.5 * std::sqrt(2.0), kTol);
}

TEST(RayCaster, NearestHitWinsAndMaxRangeCuts)
{
  GridMap map = freeGrid(100, 100, 0.05, Pose2{Vec2(-2.5, -2.5), 0.0});
  fillColumn(&map, 80);  // x = 1.5
  RayCaster caster;
  caster.setMap(map);
  caster.setCircles({Circle{Vec2(1.0, 0.0), 0.25}});
  caster.setBoxes({Box{Vec2(1.2, 0.0), Vec2(0.2, 0.2), 0.0}});
  EXPECT_NEAR(caster.cast(Vec2::Zero(), 0.0, 10.0), 0.75, kTol);
  caster.setCircles({});
  EXPECT_NEAR(caster.cast(Vec2::Zero(), 0.0, 10.0), 1.1, kTol);
  caster.setBoxes({});
  EXPECT_NEAR(caster.cast(Vec2::Zero(), 0.0, 10.0), 1.5, kTol);
  // Max range: inclusive, then nothing.
  EXPECT_NEAR(caster.cast(Vec2::Zero(), 0.0, 1.5), 1.5, kTol);
  EXPECT_TRUE(std::isinf(caster.cast(Vec2::Zero(), 0.0, 1.4)));
  caster.setCircles({Circle{Vec2(1.0, 0.0), 0.25}});
  EXPECT_TRUE(std::isinf(caster.cast(Vec2::Zero(), 0.0, 0.7)));
  caster.clearMap();
  EXPECT_FALSE(caster.hasMap());
}

TEST(RayCaster, MalformedGridIsIgnored)
{
  GridMap map = freeGrid(10, 10, 0.05);
  map.data.pop_back();
  RayCaster caster;
  caster.setMap(map);
  EXPECT_FALSE(caster.hasMap());
  map = freeGrid(10, 10, 0.0);
  caster.setMap(map);
  EXPECT_FALSE(caster.hasMap());
}

TEST(RayCaster, ScanBeamLayout)
{
  const ScanGeometry geometry = ScanGeometry::fullCircle(360, 10.0);
  EXPECT_EQ(geometry.beams, 360);
  EXPECT_DOUBLE_EQ(geometry.angle_min, -kPi);
  EXPECT_DOUBLE_EQ(geometry.angle_increment, kTwoPi / 360.0);
  EXPECT_NEAR(geometry.angleMax(), kPi - kTwoPi / 360.0, kTol);
  EXPECT_NEAR(geometry.angle(180), 0.0, kTol);

  // A disc straight ahead of beam 200 for a sensor at yaw 0.3.
  const Pose2 sensor{Vec2(1.0, -1.0), 0.3};
  const double beam_angle = sensor.yaw + geometry.angle(200);
  RayCaster caster;
  caster.setCircles({Circle{sensor.p + 3.0 * vectorfield_core::heading(beam_angle), 0.1}});
  const std::vector<double> ranges = caster.castScan(sensor, geometry);
  ASSERT_EQ(ranges.size(), 360U);
  EXPECT_NEAR(ranges[200], 2.9, kTol);
  std::size_t hits = 0;
  for (const double range : ranges) {
    hits += std::isfinite(range) ? 1U : 0U;
  }
  // The disc subtends about 2 * asin(0.1 / 3) = 3.8 degrees: beams 199 to 201 at most.
  EXPECT_GE(hits, 1U);
  EXPECT_LE(hits, 4U);
  EXPECT_TRUE(std::isinf(ranges[20]));
}

}  // namespace
