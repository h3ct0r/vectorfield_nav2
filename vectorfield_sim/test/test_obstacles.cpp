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
/// @brief The `circle_obstacles` / `box_obstacles` parsers, including malformed input
/// (upstream_audit U37: rejected as a whole, never truncated).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "vectorfield_sim/obstacles.hpp"

namespace
{

using vectorfield_sim::Box;
using vectorfield_sim::Circle;
using vectorfield_sim::parseBoxObstacles;
using vectorfield_sim::parseCircleObstacles;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

TEST(Obstacles, EmptyListsMeanNoObstacles)
{
  std::vector<Circle> circles{Circle{}};
  std::vector<Box> boxes{Box{}};
  std::string error;
  EXPECT_TRUE(parseCircleObstacles({}, &circles, &error));
  EXPECT_TRUE(parseBoxObstacles({}, &boxes, &error));
  EXPECT_TRUE(circles.empty());
  EXPECT_TRUE(boxes.empty());
  EXPECT_TRUE(error.empty());
}

TEST(Obstacles, ParsesCircles)
{
  std::vector<Circle> circles;
  ASSERT_TRUE(parseCircleObstacles({1.0, 2.0, 0.5, -1.0, 0.0, 1.25}, &circles, nullptr));
  ASSERT_EQ(circles.size(), 2U);
  EXPECT_DOUBLE_EQ(circles[0].center.x(), 1.0);
  EXPECT_DOUBLE_EQ(circles[0].center.y(), 2.0);
  EXPECT_DOUBLE_EQ(circles[0].radius, 0.5);
  EXPECT_DOUBLE_EQ(circles[1].center.x(), -1.0);
  EXPECT_DOUBLE_EQ(circles[1].radius, 1.25);
}

TEST(Obstacles, ParsesBoxes)
{
  std::vector<Box> boxes;
  ASSERT_TRUE(parseBoxObstacles({1.0, 2.0, 0.4, 0.6, 0.3}, &boxes, nullptr));
  ASSERT_EQ(boxes.size(), 1U);
  EXPECT_DOUBLE_EQ(boxes[0].center.x(), 1.0);
  EXPECT_DOUBLE_EQ(boxes[0].center.y(), 2.0);
  EXPECT_DOUBLE_EQ(boxes[0].size.x(), 0.4);
  EXPECT_DOUBLE_EQ(boxes[0].size.y(), 0.6);
  EXPECT_DOUBLE_EQ(boxes[0].yaw, 0.3);
}

TEST(Obstacles, RejectsALengthThatIsNotAMultipleOfTheStride)
{
  const std::vector<Circle> kept{Circle{{0.0, 0.0}, 1.0}};
  std::vector<Circle> circles = kept;
  std::string error;
  // Upstream silently dropped the unmatched obstacle (U37); here the list is rejected.
  EXPECT_FALSE(parseCircleObstacles({1.0, 2.0, 0.5, 3.0}, &circles, &error));
  EXPECT_NE(error.find("not a multiple of 3"), std::string::npos) << error;
  ASSERT_EQ(circles.size(), 1U);
  EXPECT_DOUBLE_EQ(circles[0].radius, 1.0);

  std::vector<Box> boxes;
  error.clear();
  EXPECT_FALSE(parseBoxObstacles({1.0, 2.0, 0.4, 0.6}, &boxes, &error));
  EXPECT_NE(error.find("not a multiple of 5"), std::string::npos) << error;
  EXPECT_TRUE(boxes.empty());
}

TEST(Obstacles, RejectsNonFiniteValues)
{
  std::vector<Circle> circles;
  std::string error;
  EXPECT_FALSE(parseCircleObstacles({0.0, 0.0, 1.0, kNan, 0.0, 1.0}, &circles, &error));
  EXPECT_NE(error.find("circle_obstacles[1]"), std::string::npos) << error;
  EXPECT_NE(error.find("not finite"), std::string::npos) << error;
  EXPECT_TRUE(circles.empty());

  std::vector<Box> boxes;
  EXPECT_FALSE(parseBoxObstacles({0.0, 0.0, 1.0, 1.0, kInf}, &boxes, &error));
  EXPECT_NE(error.find("box_obstacles[0]"), std::string::npos) << error;
}

TEST(Obstacles, RejectsNonPositiveRadiiAndSizes)
{
  std::vector<Circle> circles;
  std::string error;
  EXPECT_FALSE(parseCircleObstacles({0.0, 0.0, 0.0}, &circles, &error));
  EXPECT_NE(error.find("radius"), std::string::npos) << error;
  EXPECT_FALSE(parseCircleObstacles({0.0, 0.0, 1.0, 0.0, 0.0, -0.5}, &circles, &error));
  EXPECT_NE(error.find("circle_obstacles[1]"), std::string::npos) << error;
  EXPECT_TRUE(circles.empty());

  std::vector<Box> boxes;
  EXPECT_FALSE(parseBoxObstacles({0.0, 0.0, 0.0, 1.0, 0.0}, &boxes, &error));
  EXPECT_NE(error.find("size_x"), std::string::npos) << error;
  EXPECT_FALSE(parseBoxObstacles({0.0, 0.0, 1.0, -1.0, 0.0}, &boxes, &error));
  EXPECT_NE(error.find("size_y"), std::string::npos) << error;
  EXPECT_FALSE(parseBoxObstacles({0.0, 0.0, 1.0, 1.0, 0.0, 0.0}, &boxes, nullptr));
  EXPECT_TRUE(boxes.empty());
}

}  // namespace
