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

// The ROS ↔ core conversions (plan §4 "Conversions"): poses, twists, quaternions, paths,
// Curve.msg ↔ CurveSpec, costmaps → OccupancyGrid, CostmapInfo, footprints and goal tolerances.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_ros/conversions.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;
using vectorfield_msgs::msg::Curve;

constexpr double kTol = 1e-12;

geometry_msgs::msg::Point point(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Quaternion quaternion(double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

// ---------------------------------------------------------------------------------------------
// Points, poses, twists

TEST(Conversions, PointsAndVectorsArePlanar)
{
  EXPECT_EQ(vr::toVec2(point(1.5, -2.0, 7.0)), vf::Vec2(1.5, -2.0));
  geometry_msgs::msg::Point32 p32;
  p32.x = 0.25F;
  p32.y = -0.5F;
  p32.z = 3.0F;
  EXPECT_EQ(vr::toVec2(p32), vf::Vec2(0.25, -0.5));
  geometry_msgs::msg::Vector3 v3;
  v3.x = 3.0;
  v3.y = 4.0;
  v3.z = 5.0;
  EXPECT_EQ(vr::toVec2(v3), vf::Vec2(3.0, 4.0));

  const geometry_msgs::msg::Point p = vr::toPointMsg(vf::Vec2(1.0, 2.0), 0.3);
  EXPECT_EQ(p.x, 1.0);
  EXPECT_EQ(p.y, 2.0);
  EXPECT_EQ(p.z, 0.3);
  EXPECT_EQ(vr::toPointMsg(vf::Vec2(1.0, 2.0)).z, 0.0);
  const geometry_msgs::msg::Vector3 v = vr::toVector3Msg(vf::Vec2(-1.0, 0.5));
  EXPECT_EQ(v.x, -1.0);
  EXPECT_EQ(v.y, 0.5);
  EXPECT_EQ(v.z, 0.0);
}

TEST(Conversions, QuaternionRoundTrip)
{
  for (const double yaw : {-3.0, -vf::kHalfPi, -0.1, 0.0, 0.7, vf::kHalfPi, 3.0, vf::kPi}) {
    const geometry_msgs::msg::Quaternion q = vr::yawToQuaternion(yaw);
    EXPECT_EQ(q.x, 0.0);
    EXPECT_EQ(q.y, 0.0);
    EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, kTol);
    EXPECT_NEAR(vr::yawOf(q), yaw, 1e-12) << yaw;
  }
  // Wrapped into (-pi, pi].
  EXPECT_NEAR(vr::yawOf(vr::yawToQuaternion(4.0)), 4.0 - vf::kTwoPi, 1e-12);
  // Normalised first: a scaled quaternion has the same yaw.
  const geometry_msgs::msg::Quaternion q = vr::yawToQuaternion(0.4);
  EXPECT_NEAR(vr::yawOf(quaternion(0.0, 0.0, 3.0 * q.z, 3.0 * q.w)), 0.4, 1e-12);
  // The yaw part of a general rotation (roll 0.3 about x after yaw 0.5 about z).
  const double cy = std::cos(0.25);
  const double sy = std::sin(0.25);
  const double cr = std::cos(0.15);
  const double sr = std::sin(0.15);
  EXPECT_NEAR(vr::yawOf(quaternion(cy * sr, sy * sr, sy * cr, cy * cr)), 0.5, 1e-12);
}

TEST(Conversions, YawOfAnUnusableQuaternionIsNaN)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(std::isnan(vr::yawOf(quaternion(nan, nan, nan, nan))));
  EXPECT_TRUE(std::isnan(vr::yawOf(quaternion(0.0, 0.0, 0.0, nan))));
  EXPECT_TRUE(std::isnan(vr::yawOf(quaternion(0.0, 0.0, inf, 1.0))));
  EXPECT_TRUE(std::isnan(vr::yawOf(quaternion(0.0, 0.0, 0.0, 0.0))));
}

TEST(Conversions, PoseRoundTrip)
{
  const vf::Pose2 pose{vf::Vec2(1.25, -3.5), 2.1};
  const geometry_msgs::msg::Pose msg = vr::toPoseMsg(pose);
  EXPECT_EQ(msg.position.z, 0.0);
  EXPECT_EQ(msg.orientation.x, 0.0);
  EXPECT_EQ(msg.orientation.y, 0.0);
  const vf::Pose2 back = vr::toPose2(msg);
  EXPECT_EQ(back.p, pose.p);
  EXPECT_NEAR(back.yaw, pose.yaw, 1e-12);

  geometry_msgs::msg::Pose nan_orientation = msg;
  nan_orientation.orientation.w = std::numeric_limits<double>::quiet_NaN();
  const vf::Pose2 nan_pose = vr::toPose2(nan_orientation);
  EXPECT_EQ(nan_pose.p, pose.p);
  EXPECT_TRUE(std::isnan(nan_pose.yaw));
  EXPECT_FALSE(vf::isFinite(nan_pose));
}

TEST(Conversions, TwistRoundTrip)
{
  const vf::Twist2 twist{0.3, -0.1, 0.8};
  const geometry_msgs::msg::Twist msg = vr::toTwistMsg(twist);
  EXPECT_EQ(msg.linear.z, 0.0);
  EXPECT_EQ(msg.angular.x, 0.0);
  EXPECT_EQ(msg.angular.y, 0.0);
  const vf::Twist2 back = vr::toTwist2(msg);
  EXPECT_EQ(back.vx, twist.vx);
  EXPECT_EQ(back.vy, twist.vy);
  EXPECT_EQ(back.wz, twist.wz);

  geometry_msgs::msg::Twist extra = msg;
  extra.linear.z = 5.0;
  extra.angular.x = 5.0;
  EXPECT_EQ(vr::toTwist2(extra).wz, twist.wz);
}

TEST(Conversions, TransformIsParentFromChild)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.0;
  transform.translation.y = 2.0;
  transform.translation.z = 9.0;
  transform.rotation = vr::yawToQuaternion(vf::kHalfPi);
  const vf::Isometry2 T = vr::toIsometry2(transform);
  EXPECT_NEAR(T.yaw(), vf::kHalfPi, 1e-12);
  const vf::Vec2 x = T.apply(vf::Vec2(1.0, 0.0));
  EXPECT_NEAR(x.x(), 1.0, 1e-12);
  EXPECT_NEAR(x.y(), 3.0, 1e-12);
}

// ---------------------------------------------------------------------------------------------
// Paths

TEST(Conversions, PathsClearAndRefillWithoutReallocation)
{
  nav_msgs::msg::Path path;
  for (const double i : {0.0, 1.0, 2.0, 3.0, 4.0}) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.pose = vr::toPoseMsg(vf::Pose2{vf::Vec2(i, 2.0 * i), 0.1 * i});
    path.poses.push_back(stamped);
  }
  std::vector<vf::Pose2> poses(3);
  vr::toPoses(path, &poses);
  ASSERT_EQ(poses.size(), 5U);
  EXPECT_EQ(poses[4].p, vf::Vec2(4.0, 8.0));
  EXPECT_NEAR(poses[4].yaw, 0.4, 1e-12);

  std::vector<vf::Vec2> positions;
  vr::toPositions(path, &positions);
  ASSERT_EQ(positions.size(), 5U);
  EXPECT_EQ(positions[3], vf::Vec2(3.0, 6.0));

  // A shorter path reuses the storage.
  const vf::Pose2 * pose_data = poses.data();
  const vf::Vec2 * position_data = positions.data();
  path.poses.resize(2);
  vr::toPoses(path, &poses);
  vr::toPositions(path, &positions);
  EXPECT_EQ(poses.size(), 2U);
  EXPECT_EQ(positions.size(), 2U);
  EXPECT_EQ(poses.data(), pose_data);
  EXPECT_EQ(positions.data(), position_data);

  path.poses.clear();
  vr::toPoses(path, &poses);
  EXPECT_TRUE(poses.empty());
}

// ---------------------------------------------------------------------------------------------
// Curve.msg <-> CurveSpec

Curve polylineMsg()
{
  Curve msg;
  msg.header.frame_id = "map";
  msg.type = Curve::POLYLINE;
  msg.closed = true;
  msg.points = {point(0.0, 0.0), point(2.0, 0.0), point(2.0, 1.0), point(0.0, 1.0)};
  msg.sample_spacing = 0.05;
  msg.max_corner_deviation = 0.1;
  return msg;
}

Curve expressionMsg()
{
  Curve msg;
  msg.header.frame_id = "odom";
  msg.header.stamp.sec = 12;
  msg.type = Curve::EXPRESSION;
  msg.closed = false;
  msg.x_expression = "a * cos(s) + v * t";
  msg.y_expression = "b * sin(s)";
  msg.s_min = -1.0;
  msg.s_max = 2.5;
  msg.parameter_names = {"a", "b", "v"};
  msg.parameter_values = {2.0, 1.0, 0.1};
  return msg;
}

Curve builtinMsg()
{
  Curve msg;
  msg.header.frame_id = "map";
  msg.type = Curve::BUILTIN;
  msg.closed = true;
  msg.builtin_shape = "lemniscate_gerono";
  msg.parameter_names = {"a", "cx"};
  msg.parameter_values = {1.5, -0.5};
  return msg;
}

TEST(Conversions, CurveMsgRoundTripPolyline)
{
  const Curve msg = polylineMsg();
  const vf::CurveSpec spec = vr::toCurveSpec(msg, 3.5);
  EXPECT_EQ(spec.type, vf::CurveSpec::Type::kPolyline);
  EXPECT_TRUE(spec.closed);
  ASSERT_EQ(spec.points.size(), 4U);
  EXPECT_EQ(spec.points[2], vf::Vec2(2.0, 1.0));
  EXPECT_EQ(spec.sample_spacing, 0.05);
  EXPECT_EQ(spec.max_corner_deviation, 0.1);
  EXPECT_EQ(spec.t0, 3.5);
  EXPECT_EQ(vr::toCurveMsg(spec, msg.header), msg);
}

TEST(Conversions, CurveMsgRoundTripExpression)
{
  const Curve msg = expressionMsg();
  const vf::CurveSpec spec = vr::toCurveSpec(msg, 12.0);
  EXPECT_EQ(spec.type, vf::CurveSpec::Type::kExpression);
  EXPECT_FALSE(spec.closed);
  EXPECT_EQ(spec.x_expression, msg.x_expression);
  EXPECT_EQ(spec.y_expression, msg.y_expression);
  EXPECT_EQ(spec.s_min, -1.0);
  EXPECT_EQ(spec.s_max, 2.5);
  EXPECT_EQ(spec.parameter_names, msg.parameter_names);
  EXPECT_EQ(spec.parameter_values, msg.parameter_values);
  EXPECT_EQ(vr::toCurveMsg(spec, msg.header), msg);
}

TEST(Conversions, CurveMsgRoundTripBuiltin)
{
  const Curve msg = builtinMsg();
  const vf::CurveSpec spec = vr::toCurveSpec(msg, 0.0);
  EXPECT_EQ(spec.type, vf::CurveSpec::Type::kBuiltin);
  EXPECT_EQ(spec.builtin_shape, "lemniscate_gerono");
  EXPECT_EQ(spec.s_min, 0.0);
  EXPECT_EQ(spec.s_max, 0.0);
  EXPECT_EQ(vr::toCurveMsg(spec, msg.header), msg);
  // The spec round trip is exact too (t0 is the caller's).
  EXPECT_EQ(vr::toCurveSpec(vr::toCurveMsg(spec, msg.header), 0.0), spec);
}

TEST(Conversions, CurvePointsLoseZ)
{
  Curve msg = polylineMsg();
  msg.points[1].z = 4.0;
  const vf::CurveSpec spec = vr::toCurveSpec(msg, 0.0);
  EXPECT_EQ(spec.points[1], vf::Vec2(2.0, 0.0));
  EXPECT_EQ(vr::toCurveMsg(spec, msg.header).points[1].z, 0.0);
}

TEST(Conversions, UnknownCurveTypeIsInvalidCurve)
{
  Curve msg = polylineMsg();
  msg.type = 7;
  try {
    (void)vr::toCurveSpec(msg, 0.0);
    FAIL() << "no CurveError for type 7";
  } catch (const vf::CurveError & e) {
    EXPECT_EQ(e.code(), vf::CurveError::Code::kInvalidCurve);
    EXPECT_NE(std::string(e.what()).find('7'), std::string::npos) << e.what();
  }
}

// ---------------------------------------------------------------------------------------------
// Costmaps

// A 4 x 3 costmap at 0.1 m with origin (-1, 2): lethal (1, 1), unknown (2, 1), inscribed (3, 2)
// and a free-space cost (0, 2).
void fillCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap->setCost(1, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->setCost(2, 1, nav2_costmap_2d::NO_INFORMATION);
  costmap->setCost(3, 2, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  costmap->setCost(0, 2, 100);
}

nav2_msgs::msg::Costmap costmapMsg()
{
  nav2_msgs::msg::Costmap msg;
  msg.metadata.size_x = 4;
  msg.metadata.size_y = 3;
  msg.metadata.resolution = 0.1F;
  msg.metadata.origin.position.x = -1.0;
  msg.metadata.origin.position.y = 2.0;
  msg.data.assign(12, nav2_costmap_2d::FREE_SPACE);
  msg.data[1 * 4 + 1] = nav2_costmap_2d::LETHAL_OBSTACLE;
  msg.data[1 * 4 + 2] = nav2_costmap_2d::NO_INFORMATION;
  msg.data[2 * 4 + 3] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  msg.data[2 * 4 + 0] = 100;
  return msg;
}

void expectSites(const vf::OccupancyGrid & grid, bool unknown_is_obstacle)
{
  ASSERT_EQ(grid.width(), 4);
  ASSERT_EQ(grid.height(), 3);
  EXPECT_NEAR(grid.resolution(), 0.1, 1e-7);
  EXPECT_NEAR(grid.origin().x(), -1.0, 1e-12);
  EXPECT_NEAR(grid.origin().y(), 2.0, 1e-12);
  for (int iy = 0; iy < 3; ++iy) {
    for (int ix = 0; ix < 4; ++ix) {
      const bool site = (ix == 1 && iy == 1) || (unknown_is_obstacle && ix == 2 && iy == 1);
      EXPECT_EQ(grid.isSite(ix, iy), site)
        << ix << ", " << iy << " unknown " << unknown_is_obstacle;
    }
  }
}

TEST(Conversions, Costmap2DLethalAndUnknownSites)
{
  nav2_costmap_2d::Costmap2D costmap(4, 3, 0.1, -1.0, 2.0);
  fillCostmap(&costmap);
  std::vector<std::uint8_t> mask;
  for (const bool unknown_is_obstacle : {false, true}) {
    vf::OccupancyGrid grid;
    ASSERT_TRUE(vr::toOccupancyGrid(costmap, unknown_is_obstacle, &mask, &grid));
    expectSites(grid, unknown_is_obstacle);
    EXPECT_EQ(mask.size(), 12U);
  }
}

TEST(Conversions, Costmap2DVersionChangesOnlyWithTheSites)
{
  nav2_costmap_2d::Costmap2D costmap(4, 3, 0.1, -1.0, 2.0);
  fillCostmap(&costmap);
  std::vector<std::uint8_t> mask;
  vf::OccupancyGrid grid;
  ASSERT_TRUE(vr::toOccupancyGrid(costmap, false, &mask, &grid));
  const std::uint64_t version = grid.version();
  EXPECT_NE(version, 0U);
  const std::uint8_t * mask_data = mask.data();

  // Identical costmap: same version, no reallocation of the work buffer.
  ASSERT_TRUE(vr::toOccupancyGrid(costmap, false, &mask, &grid));
  EXPECT_EQ(grid.version(), version);
  EXPECT_EQ(mask.data(), mask_data);

  // A non-site cost change keeps the version; a site change does not.
  costmap.setCost(0, 0, 200);
  ASSERT_TRUE(vr::toOccupancyGrid(costmap, false, &mask, &grid));
  EXPECT_EQ(grid.version(), version);
  costmap.setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(vr::toOccupancyGrid(costmap, false, &mask, &grid));
  EXPECT_NE(grid.version(), version);
  EXPECT_TRUE(grid.isSite(0, 0));
}

TEST(Conversions, EmptyCostmap2DIsRejected)
{
  nav2_costmap_2d::Costmap2D costmap(4, 3, 0.1, -1.0, 2.0);
  fillCostmap(&costmap);
  std::vector<std::uint8_t> mask;
  vf::OccupancyGrid grid;
  ASSERT_TRUE(vr::toOccupancyGrid(costmap, false, &mask, &grid));
  const std::uint64_t version = grid.version();

  nav2_costmap_2d::Costmap2D empty;
  EXPECT_FALSE(vr::toOccupancyGrid(empty, false, &mask, &grid));
  EXPECT_EQ(grid.version(), version);
  expectSites(grid, false);
}

TEST(Conversions, CostmapMsgLethalAndUnknownSites)
{
  const nav2_msgs::msg::Costmap msg = costmapMsg();
  std::vector<std::uint8_t> mask;
  for (const bool unknown_is_obstacle : {false, true}) {
    vf::OccupancyGrid grid;
    ASSERT_TRUE(vr::toOccupancyGrid(msg, unknown_is_obstacle, &mask, &grid));
    expectSites(grid, unknown_is_obstacle);
  }

  // Identical message: same version.
  vf::OccupancyGrid grid;
  ASSERT_TRUE(vr::toOccupancyGrid(msg, true, &mask, &grid));
  const std::uint64_t version = grid.version();
  ASSERT_TRUE(vr::toOccupancyGrid(costmapMsg(), true, &mask, &grid));
  EXPECT_EQ(grid.version(), version);

  // q and -q are the same identity orientation.
  nav2_msgs::msg::Costmap negated = costmapMsg();
  negated.metadata.origin.orientation.w = -1.0;
  EXPECT_TRUE(vr::toOccupancyGrid(negated, true, &mask, &grid));
  EXPECT_EQ(grid.version(), version);
}

TEST(Conversions, CostmapMsgRejectsRotatedOrInconsistentMaps)
{
  std::vector<std::uint8_t> mask;
  vf::OccupancyGrid grid;
  ASSERT_TRUE(vr::toOccupancyGrid(costmapMsg(), false, &mask, &grid));
  const std::uint64_t version = grid.version();

  nav2_msgs::msg::Costmap rotated = costmapMsg();
  rotated.metadata.origin.orientation = vr::yawToQuaternion(0.3);
  EXPECT_FALSE(vr::toOccupancyGrid(rotated, false, &mask, &grid));

  nav2_msgs::msg::Costmap zero_quaternion = costmapMsg();
  zero_quaternion.metadata.origin.orientation = quaternion(0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(vr::toOccupancyGrid(zero_quaternion, false, &mask, &grid));

  nav2_msgs::msg::Costmap short_data = costmapMsg();
  short_data.data.pop_back();
  EXPECT_FALSE(vr::toOccupancyGrid(short_data, false, &mask, &grid));

  nav2_msgs::msg::Costmap empty = costmapMsg();
  empty.metadata.size_x = 0;
  empty.data.clear();
  EXPECT_FALSE(vr::toOccupancyGrid(empty, false, &mask, &grid));

  nav2_msgs::msg::Costmap bad_resolution = costmapMsg();
  bad_resolution.metadata.resolution = 0.0F;
  EXPECT_FALSE(vr::toOccupancyGrid(bad_resolution, false, &mask, &grid));

  EXPECT_EQ(grid.version(), version);
  expectSites(grid, false);
}

TEST(Conversions, CostmapInfo)
{
  const nav2_costmap_2d::Costmap2D costmap(60, 40, 0.05, -1.5, -1.0);
  const vf::CostmapInfo info = vr::toCostmapInfo(costmap, std::nullopt);
  EXPECT_NEAR(info.width_m, 3.0, 1e-12);
  EXPECT_NEAR(info.height_m, 2.0, 1e-12);
  EXPECT_EQ(info.resolution, 0.05);
  EXPECT_FALSE(info.window_limit.has_value());
  EXPECT_EQ(vr::toCostmapInfo(costmap, 2.0).window_limit, std::optional<double>(2.0));

  nav2_msgs::msg::CostmapMetaData metadata;
  metadata.size_x = 60;
  metadata.size_y = 40;
  metadata.resolution = 0.05F;
  const vf::CostmapInfo from_msg = vr::toCostmapInfo(metadata, 1.5);
  EXPECT_NEAR(from_msg.width_m, 3.0, 1e-6);
  EXPECT_NEAR(from_msg.height_m, 2.0, 1e-6);
  EXPECT_NEAR(from_msg.resolution, 0.05, 1e-8);
  EXPECT_EQ(from_msg.window_limit, std::optional<double>(1.5));
}

// ---------------------------------------------------------------------------------------------
// Footprints

TEST(Conversions, FootprintSpec)
{
  const vf::FootprintSpec point_robot = vr::toFootprintSpec({});
  EXPECT_TRUE(point_robot.polygon.empty());
  EXPECT_EQ(point_robot.radius, 0.0);
  EXPECT_TRUE(vf::footprintIsValid(point_robot));

  const vf::FootprintSpec segment = vr::toFootprintSpec({point(0.1, 0.1), point(-0.1, 0.1)});
  ASSERT_EQ(segment.polygon.size(), 2U);
  EXPECT_FALSE(vf::footprintIsValid(segment));
  vf::CoreConfig config;
  config.node.footprint = segment;
  EXPECT_TRUE(vf::validateConfig(config, std::nullopt).hasError(vf::issue::kInvalidFootprint));

  const std::vector<geometry_msgs::msg::Point> square = {
    point(0.2, 0.15, 0.3), point(-0.2, 0.15), point(-0.2, -0.15), point(0.2, -0.15)};
  const vf::FootprintSpec polygon = vr::toFootprintSpec(square);
  ASSERT_EQ(polygon.polygon.size(), 4U);
  EXPECT_EQ(polygon.polygon[0], vf::Vec2(0.2, 0.15));
  EXPECT_EQ(polygon.polygon[2], vf::Vec2(-0.2, -0.15));
  EXPECT_TRUE(vf::footprintIsValid(polygon));
  const vf::FootprintRadii radii = vf::computeFootprintRadii(polygon);
  EXPECT_NEAR(radii.w_lat, 0.15, 1e-12);
  EXPECT_NEAR(radii.r_insc, 0.15, 1e-12);
  EXPECT_NEAR(radii.r_circ, 0.25, 1e-12);

  std::vector<vf::Vec2> vertices(10);
  const vf::Vec2 * data = vertices.data();
  vr::toVertices(square, &vertices);
  EXPECT_EQ(vertices, polygon.polygon);
  EXPECT_EQ(vertices.data(), data);
}

TEST(Conversions, FootprintChanged)
{
  const std::vector<vf::Vec2> previous = {
    vf::Vec2(0.2, 0.15), vf::Vec2(-0.2, 0.15), vf::Vec2(-0.2, -0.15)};
  std::vector<vf::Vec2> current = previous;
  EXPECT_FALSE(vr::footprintChanged(previous, current, 1e-3));

  current[1].x() += 0.5e-3;
  EXPECT_FALSE(vr::footprintChanged(previous, current, 1e-3));
  current[1].x() += 1.0e-3;
  EXPECT_TRUE(vr::footprintChanged(previous, current, 1e-3));
  EXPECT_FALSE(vr::footprintChanged(previous, current, 1e-2));

  current = previous;
  current.push_back(vf::Vec2(0.2, -0.15));
  EXPECT_TRUE(vr::footprintChanged(previous, current, 1e-2));
  EXPECT_FALSE(vr::footprintChanged({}, {}, 1e-3));

  current = previous;
  current[0].y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(vr::footprintChanged(previous, current, 1e-2));
}

// ---------------------------------------------------------------------------------------------
// Goal-checker tolerances

TEST(Conversions, GoalTolerancesOfSimpleGoalChecker)
{
  geometry_msgs::msg::Pose tolerance;
  tolerance.position.x = 0.25;
  tolerance.position.y = 0.25;
  tolerance.orientation = vr::yawToQuaternion(0.25);
  vr::GoalTolerances decoded = vr::toGoalTolerances(tolerance);
  EXPECT_TRUE(decoded.xy_valid);
  EXPECT_EQ(decoded.xy, 0.25);
  EXPECT_NEAR(decoded.yaw, 0.25, 1e-12);

  tolerance.position.x = 0.3;
  tolerance.position.y = 0.2;
  tolerance.orientation = vr::yawToQuaternion(-0.1);
  decoded = vr::toGoalTolerances(tolerance);
  EXPECT_TRUE(decoded.xy_valid);
  EXPECT_EQ(decoded.xy, 0.2);
  EXPECT_NEAR(decoded.yaw, 0.1, 1e-12);
}

TEST(Conversions, GoalTolerancesPositionGoalCheckerHasNoYaw)
{
  geometry_msgs::msg::Pose tolerance;
  tolerance.position.x = 0.1;
  tolerance.position.y = 0.1;
  tolerance.orientation = quaternion(0.0, 0.0, 0.0, 1.0);
  const vr::GoalTolerances decoded = vr::toGoalTolerances(tolerance);
  EXPECT_TRUE(decoded.xy_valid);
  EXPECT_EQ(decoded.xy, 0.1);
  EXPECT_EQ(decoded.yaw, 0.0);
}

TEST(Conversions, GoalTolerancesUnmeasuredFields)
{
  const double lowest = std::numeric_limits<double>::lowest();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  geometry_msgs::msg::Pose tolerance;
  tolerance.position.x = lowest;
  tolerance.position.y = lowest;
  tolerance.orientation = quaternion(lowest, lowest, lowest, lowest);
  vr::GoalTolerances decoded = vr::toGoalTolerances(tolerance);
  EXPECT_FALSE(decoded.xy_valid);
  EXPECT_EQ(decoded.xy, 0.25);
  EXPECT_EQ(decoded.yaw, 0.0);

  tolerance.position.x = nan;
  tolerance.position.y = std::numeric_limits<double>::infinity();
  tolerance.orientation = quaternion(nan, nan, nan, nan);
  decoded = vr::toGoalTolerances(tolerance);
  EXPECT_FALSE(decoded.xy_valid);
  EXPECT_EQ(decoded.xy, 0.25);
  EXPECT_EQ(decoded.yaw, 0.0);

  // One measured axis is used alone.
  tolerance.position.x = lowest;
  tolerance.position.y = 0.4;
  tolerance.orientation = vr::yawToQuaternion(0.2);
  tolerance.orientation.z = lowest;
  decoded = vr::toGoalTolerances(tolerance);
  EXPECT_TRUE(decoded.xy_valid);
  EXPECT_EQ(decoded.xy, 0.4);
  EXPECT_EQ(decoded.yaw, 0.0);
}

}  // namespace
