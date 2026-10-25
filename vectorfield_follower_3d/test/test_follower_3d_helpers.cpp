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

// The 3D follower's pure helpers: goal validation, the curve conversion, the obstacle entries,
// the voxeliser, the output frame and the markers.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"
#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"
#include "vectorfield_follower_3d/conversions.hpp"
#include "vectorfield_follower_3d/goal_setup.hpp"
#include "vectorfield_follower_3d/markers.hpp"
#include "vectorfield_follower_3d/params.hpp"
#include "vectorfield_follower_3d/voxelizer.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace d3 = vectorfield_core::d3;
namespace rc = vectorfield_curve_follower::result_code;
using vectorfield_follower_3d::GoalSetupContext3;
using vectorfield_follower_3d::PreparedGoal3;
using FollowCurve3 = vectorfield_msgs::action::FollowCurve3;
using Curve3 = vectorfield_msgs::msg::Curve3;

FollowCurve3::Goal builtinGoal(const std::string & shape, std::uint32_t laps = 0)
{
  FollowCurve3::Goal goal;
  goal.curve.type = Curve3::BUILTIN;
  goal.curve.builtin_shape = shape;
  goal.laps = laps;
  return goal;
}

GoalSetupContext3 context()
{
  GoalSetupContext3 c;
  c.global_frame = "map";
  c.now = 10.0;
  return c;
}

std::uint16_t codeOf(const FollowCurve3::Goal & goal, const GoalSetupContext3 & c = context())
{
  PreparedGoal3 prepared;
  const auto rejection = vectorfield_follower_3d::prepareGoal3(goal, c, &prepared);
  return rejection ? rejection->code : rc::kNone;
}

TEST(PrepareGoal3, AcceptsBuiltinsAndResolvesFrameAndTime)
{
  PreparedGoal3 prepared;
  FollowCurve3::Goal goal = builtinGoal("viviani", 2);
  goal.desired_speed = 0.7;
  goal.reverse_direction = true;
  ASSERT_FALSE(vectorfield_follower_3d::prepareGoal3(goal, context(), &prepared));
  EXPECT_TRUE(prepared.curve->isClosed());
  EXPECT_EQ(prepared.frame_id, "map");
  EXPECT_DOUBLE_EQ(prepared.spec.t0, 10.0);
  EXPECT_DOUBLE_EQ(prepared.config.desired_speed, 0.7);
  EXPECT_EQ(prepared.config.direction, -1);
  EXPECT_EQ(prepared.laps, 2U);
  EXPECT_EQ(codeOf(builtinGoal("helix")), rc::kNone);
  EXPECT_EQ(codeOf(builtinGoal("torus_knot", 1)), rc::kNone);
}

TEST(PrepareGoal3, InvalidCurves)
{
  EXPECT_EQ(codeOf(builtinGoal("no_such_shape")), rc::kInvalidCurve);
  EXPECT_EQ(codeOf(builtinGoal("helix", 1)), rc::kInvalidCurve) << "laps on an open curve";
  FollowCurve3::Goal goal;
  goal.curve.type = Curve3::POLYLINE;
  goal.curve.points.resize(1);
  EXPECT_EQ(codeOf(goal), rc::kInvalidCurve) << "a single vertex";
  goal.curve.type = 7;
  EXPECT_EQ(codeOf(goal), rc::kInvalidCurve) << "an unknown type";
  goal = FollowCurve3::Goal{};
  goal.curve.type = Curve3::EXPRESSION;
  goal.curve.x_expression = "cos(s";
  goal.curve.y_expression = "sin(s)";
  goal.curve.z_expression = "0";
  goal.curve.s_max = 6.0;
  goal.curve.closed = true;
  EXPECT_EQ(codeOf(goal), rc::kInvalidCurve) << "a syntax error";
  goal.curve.x_expression = "cos(s)";
  goal.curve.s_max = 2.0 * vf::kPi;
  EXPECT_EQ(codeOf(goal), rc::kNone);
  goal.curve.sample_spacing = std::nan("");
  EXPECT_EQ(codeOf(goal), rc::kInvalidCurve) << "a non-finite sample_spacing";
}

TEST(PrepareGoal3, PoseOfBuiltins)
{
  FollowCurve3::Goal goal = builtinGoal("circle", 1);
  goal.curve.pose.position.z = 2.0;
  goal.curve.pose.orientation.z = std::sin(0.25);
  goal.curve.pose.orientation.w = std::cos(0.25);
  PreparedGoal3 prepared;
  ASSERT_FALSE(vectorfield_follower_3d::prepareGoal3(goal, context(), &prepared));
  // The pose became cx cy cz yaw pitch roll: the circle sits at z = 2.
  EXPECT_NEAR(prepared.curve->pointAt(0.0, 10.0).z(), 2.0, 1e-9);
  const auto & names = prepared.spec.parameter_names;
  ASSERT_EQ(names.size(), 6U);
  EXPECT_EQ(names[3], "yaw");
  EXPECT_NEAR(prepared.spec.parameter_values[3], 0.5, 1e-12);
  // A pose together with a pose parameter, a non-unit quaternion, a pose on a polyline.
  FollowCurve3::Goal both = goal;
  both.curve.parameter_names = {"cz"};
  both.curve.parameter_values = {1.0};
  EXPECT_EQ(codeOf(both), rc::kInvalidCurve);
  FollowCurve3::Goal scaled = goal;
  scaled.curve.pose.orientation.w = 2.0;
  EXPECT_EQ(codeOf(scaled), rc::kInvalidCurve);
  FollowCurve3::Goal polyline;
  polyline.curve.type = Curve3::POLYLINE;
  polyline.curve.points.resize(2);
  polyline.curve.points[1].x = 1.0;
  EXPECT_EQ(codeOf(polyline), rc::kNone);
  polyline.curve.pose.position.x = 1.0;
  EXPECT_EQ(codeOf(polyline), rc::kInvalidCurve);
  // The ZYX angles invert rotationZYX.
  const auto angles = vectorfield_follower_3d::eulerZYX(d3::rotationZYX(0.3, -0.4, 1.1));
  EXPECT_NEAR(angles.yaw, 0.3, 1e-12);
  EXPECT_NEAR(angles.pitch, -0.4, 1e-12);
  EXPECT_NEAR(angles.roll, 1.1, 1e-12);
}

TEST(PrepareGoal3, UnsupportedAndInvalidGoals)
{
  FollowCurve3::Goal open_moving;
  open_moving.curve.type = Curve3::EXPRESSION;
  open_moving.curve.x_expression = "s";
  open_moving.curve.y_expression = "0.1*t";
  open_moving.curve.z_expression = "0";
  open_moving.curve.s_max = 2.0;
  EXPECT_EQ(codeOf(open_moving), rc::kUnsupported) << "open and time-varying";
  FollowCurve3::Goal moving = builtinGoal("circle");
  moving.frame_motion = FollowCurve3::Goal::FRAME_MOVING;
  EXPECT_EQ(codeOf(moving), rc::kUnsupported) << "no frame source";
  GoalSetupContext3 with_reference = context();
  with_reference.moving_frame_reference = "odom";
  moving.curve.header.frame_id = "platform";
  EXPECT_EQ(codeOf(moving, with_reference), rc::kNone);
  moving.frame_odom_topic = "platform/odom";
  EXPECT_EQ(codeOf(moving), rc::kNone);
  FollowCurve3::Goal bad = builtinGoal("circle");
  bad.frame_motion = 5;
  EXPECT_EQ(codeOf(bad), rc::kInvalidGoal);
  bad = builtinGoal("circle");
  bad.max_duration.sec = -1;
  EXPECT_EQ(codeOf(bad), rc::kInvalidGoal);
  bad = builtinGoal("circle");
  bad.desired_speed = std::nan("");
  EXPECT_EQ(codeOf(bad), rc::kInvalidGoal);
  // desired_speed below min_approach_speed fails validateTracker3Config.
  bad = builtinGoal("circle");
  bad.desired_speed = 0.01;
  EXPECT_EQ(codeOf(bad), rc::kInvalidGoal);
}

TEST(PrepareGoal3, KeepsTrackerStateOnlyForTheSameGoal)
{
  PreparedGoal3 a;
  PreparedGoal3 b;
  ASSERT_FALSE(vectorfield_follower_3d::prepareGoal3(builtinGoal("circle", 1), context(), &a));
  GoalSetupContext3 later = context();
  later.now = 20.0;
  ASSERT_FALSE(vectorfield_follower_3d::prepareGoal3(builtinGoal("circle", 3), later, &b));
  EXPECT_TRUE(vectorfield_follower_3d::keepsTrackerState3(a, b)) << "t0 and laps do not matter";
  FollowCurve3::Goal faster = builtinGoal("circle", 1);
  faster.desired_speed = 0.8;
  ASSERT_FALSE(vectorfield_follower_3d::prepareGoal3(faster, later, &b));
  EXPECT_FALSE(vectorfield_follower_3d::keepsTrackerState3(a, b));
}

TEST(Primitives, ParsesEveryKindAndRejectsTypos)
{
  using vectorfield_follower_3d::parsePrimitive;
  const auto sphere = parsePrimitive("sphere 1 2 3 0.5");
  ASSERT_TRUE(sphere);
  EXPECT_EQ(sphere->type, d3::PrimitiveType3::kSphere);
  EXPECT_DOUBLE_EQ(sphere->radius, 0.5);
  EXPECT_TRUE(parsePrimitive("box 0 0 1 0.5 0.5 1"));
  const auto rotated = parsePrimitive("box 0 0 1 0.5 0.5 1 0.3 0 0");
  ASSERT_TRUE(rotated);
  EXPECT_NEAR(rotated->rotation(1, 0), std::sin(0.3), 1e-12);
  const auto pole = parsePrimitive("cylinder 1 1 0 0.1");
  ASSERT_TRUE(pole);
  EXPECT_TRUE(std::isinf(pole->half_height));
  EXPECT_TRUE(parsePrimitive("cylinder 1 1 0 0.1 2"));
  EXPECT_TRUE(parsePrimitive("plane 0 0 0 0 0 1"));
  std::string error;
  EXPECT_FALSE(parsePrimitive("sphere 1 2 3", &error));
  EXPECT_NE(error.find("x y z r"), std::string::npos) << error;
  EXPECT_FALSE(parsePrimitive("sphere 1 2 3 -1", &error));
  EXPECT_FALSE(parsePrimitive("sphere 1 2 3 0.5x", &error));
  EXPECT_FALSE(parsePrimitive("box 0 0 1 0.5 0.5 1 0.3", &error));
  EXPECT_FALSE(parsePrimitive("plane 0 0 0 0 0 0", &error));
  EXPECT_FALSE(parsePrimitive("cone 0 0 0 1", &error));
  std::vector<d3::Primitive3> primitives;
  EXPECT_TRUE(
    vectorfield_follower_3d::parsePrimitives({"sphere 0 0 0 1", "plane 0 0 0 0 0 1"}, &primitives));
  EXPECT_EQ(primitives.size(), 2U);
  EXPECT_FALSE(
    vectorfield_follower_3d::parsePrimitives({"sphere 0 0 0 1", "sphere"}, &primitives, &error));
  EXPECT_NE(error.find("obstacles[1]"), std::string::npos) << error;
}

TEST(Voxelizer, WindowAndMask)
{
  using vectorfield_follower_3d::VoxelWindow;
  const VoxelWindow window =
    vectorfield_follower_3d::windowAround(d3::Vec3(1.03, -0.02, 2.0), 0.1, 10);
  // Snapped to the lattice, centred within half a voxel.
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(std::remainder(window.origin[i], 0.1), 0.0, 1e-9);
  }
  EXPECT_NEAR(window.origin.x(), 0.5, 1e-9);
  EXPECT_NEAR(window.origin.z(), 1.5, 1e-9);
  std::vector<std::uint8_t> mask;
  const std::vector<d3::Vec3> points = {
    d3::Vec3(0.55, 0.0, 1.55), d3::Vec3(0.56, 0.01, 1.56),  // the same voxel
    d3::Vec3(1.45, 0.45, 2.45),                             // the last voxel
    d3::Vec3(1.5, 0.0, 2.0),                                // outside (upper bound)
    d3::Vec3(std::nan(""), 0.0, 2.0)};
  EXPECT_EQ(vectorfield_follower_3d::voxelize(points, window, &mask), 2U);
  ASSERT_EQ(mask.size(), 1000U);
  // (ix, iy, iz) = (0, 5, 0) and (9, 9, 9).
  EXPECT_EQ(mask[(0 * 10 + 5) * 10 + 0], 1U);
  EXPECT_EQ(mask[(9 * 10 + 9) * 10 + 9], 1U);
}

TEST(Voxelizer, CloudPoints)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 2;
  const char * names[3] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = 4U * i;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.point_step = 16;  // padding
  cloud.row_step = 32;
  cloud.data.assign(32, 0U);
  const float a[3] = {1.0F, 2.0F, 3.0F};
  const float b[3] = {std::nanf(""), 0.0F, 0.0F};
  std::memcpy(cloud.data.data(), a, sizeof(a));
  std::memcpy(cloud.data.data() + 16, b, sizeof(b));
  d3::Isometry3 shift = d3::Isometry3::Identity();
  shift.translation() = d3::Vec3(0.0, 0.0, 1.0);
  std::vector<d3::Vec3> points;
  ASSERT_TRUE(vectorfield_follower_3d::cloudPoints(cloud, shift, &points));
  ASSERT_EQ(points.size(), 1U);
  EXPECT_TRUE(points[0].isApprox(d3::Vec3(1.0, 2.0, 4.0)));
  cloud.fields.pop_back();
  EXPECT_FALSE(vectorfield_follower_3d::cloudPoints(cloud, shift, &points));
}

TEST(Conversions, BodyCommandIsTheHeadingFrame)
{
  d3::Twist3 cmd;
  cmd.v = d3::Vec3(1.0, 0.0, 0.3);
  cmd.wz = 0.2;
  const auto world = vectorfield_follower_3d::toCommandMsg(cmd, false, 0.5 * vf::kPi);
  EXPECT_DOUBLE_EQ(world.linear.x, 1.0);
  const auto body = vectorfield_follower_3d::toCommandMsg(cmd, true, 0.5 * vf::kPi);
  // Facing +y, a world +x velocity is to the right (−y in FLU); z and the yaw rate are kept.
  EXPECT_NEAR(body.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(body.linear.y, -1.0, 1e-12);
  EXPECT_DOUBLE_EQ(body.linear.z, 0.3);
  EXPECT_DOUBLE_EQ(body.angular.z, 0.2);
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.4);
  q.w = std::cos(0.4);
  EXPECT_NEAR(vectorfield_follower_3d::yawOf(q), 0.8, 1e-12);
  EXPECT_DOUBLE_EQ(vectorfield_follower_3d::yawOf(geometry_msgs::msg::Quaternion{}), 0.0);
}

TEST(Markers, DrawsTheCurveTheObstaclesAndTheStatus)
{
  PreparedGoal3 prepared;
  ASSERT_FALSE(vectorfield_follower_3d::prepareGoal3(builtinGoal("helix"), context(), &prepared));
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::sphere(d3::Vec3(1.0, 0.0, 0.5), 0.3));
  obstacles.add(d3::Primitive3::plane(d3::Vec3::Zero(), d3::Vec3::UnitZ()));
  d3::Tracker3 tracker(prepared.config);
  vectorfield_follower_3d::MarkerInput3 input;
  input.frame_id = "map";
  input.curve = prepared.curve.get();
  input.t = 10.0;
  input.field = &tracker.field();
  input.field_extent = 0.5;
  input.field_step = 0.5;
  input.obstacles = &obstacles;
  input.body_radius = 0.2;
  input.status_text = "FOLLOW";
  const auto array = vectorfield_follower_3d::buildMarkers3(input);
  ASSERT_FALSE(array.markers.empty());
  EXPECT_EQ(array.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  int curve = 0;
  int obstacle = 0;
  const visualization_msgs::msg::Marker * field = nullptr;
  for (const auto & m : array.markers) {
    curve += m.ns == "curve" ? 1 : 0;
    obstacle += m.ns == "obstacles" ? 1 : 0;
    field = m.ns == "field" ? &m : field;
  }
  EXPECT_EQ(curve, 1);
  EXPECT_EQ(obstacle, 2);
  ASSERT_NE(field, nullptr);
  // 27 arrows: a shaft and three head strokes each, one colour per point.
  EXPECT_EQ(field->points.size(), 8U * 27U);
  ASSERT_EQ(field->colors.size(), field->points.size());
  // Coloured by distance: the lattice spans the curve's neighbourhood, so not all one colour.
  bool varied = false;
  for (const auto & c : field->colors) {
    varied = varied || c.r != field->colors.front().r || c.b != field->colors.front().b;
  }
  EXPECT_TRUE(varied);

  // A finer lattice is capped at 7 arrows per axis, spread over the same extent.
  input.field_extent = 1.5;
  input.field_step = 0.1;
  for (const auto & m : vectorfield_follower_3d::buildMarkers3(input).markers) {
    if (m.ns == "field") {
      EXPECT_EQ(m.points.size(), 8U * 343U);
    }
  }
}

}  // namespace
