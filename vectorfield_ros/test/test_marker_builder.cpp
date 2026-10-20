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

// MarkerBuilder (plan §4.1; §9 layer 2 "visualization"): for every phase × status combination
// exactly the legend's namespaces are present and the others are DELETEd, ids are stable, every
// marker is in the control frame with the cycle stamp and finite coordinates, the colours match
// the legend, and the other messages (field_grid, window, predicted_path, obstacle_distance,
// DELETEALL) have the documented shape. Inputs come from a real Tracker run (viz_test_support).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "vectorfield_core/status.hpp"
#include "vectorfield_ros/legend.hpp"
#include "vectorfield_ros/marker_builder.hpp"
#include "vectorfield_ros/viz_config.hpp"
#include "viz_test_support.hpp"

namespace
{

namespace legend = vectorfield_ros::legend;
namespace vf = vectorfield_core;
using legend::Ns;
using vectorfield_ros::MarkerBuilder;
using vectorfield_ros::MarkerContext;
using vf::Phase;
using vf::Pose2;
using vf::Status;
using vf::Vec2;
using vf_viz_test::Scenario;
using vf_viz_test::World;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/// Vectors shorter than this are not drawn (marker_builder.cpp kMinVector).
constexpr double kMinVector = 1.0e-4;

std::string nsName(Ns ns) { return std::string(legend::spec(ns).name); }

bool sameColor(const std_msgs::msg::ColorRGBA & c, const legend::Rgba & e)
{
  return c.r == e.r && c.g == e.g && c.b == e.b && c.a == e.a;
}

legend::Rgba ring(const legend::Rgba & c)
{
  return legend::Rgba{c.r, c.g, c.b, legend::kRingAlpha};
}

bool drawable(const Vec2 & v) { return vf::isFinite(v) && v.norm() > kMinVector; }

/// @brief The marker of (ns, id) in a `markers` message (legend order).
const Marker & find(const MarkerArray & a, Ns ns, std::int32_t id)
{
  std::size_t index = 0;
  for (std::size_t k = 0; k < static_cast<std::size_t>(ns); ++k) {
    index += static_cast<std::size_t>(legend::kNamespaces[k].id_count);
  }
  return a.markers.at(index + static_cast<std::size_t>(id));
}

bool added(const MarkerArray & a, Ns ns, std::int32_t id)
{
  return find(a, ns, id).action == Marker::ADD;
}

/// @brief Namespaces with at least one ADD.
std::set<std::string> present(const MarkerArray & a)
{
  std::set<std::string> out;
  for (const Marker & m : a.markers) {
    if (m.action == Marker::ADD) {
      out.insert(m.ns);
    }
  }
  return out;
}

/// @brief The legend's applicability rules (plan §4.1 table), evaluated on the cycle.
std::set<std::string> expectedNamespaces(const MarkerContext & c)
{
  const vf::TrackerOutput & out = *c.output;
  const vf::FieldSample & f = out.field;
  std::set<std::string> s{
    nsName(Ns::kStatus), nsName(Ns::kFootprint), nsName(Ns::kCurve), nsName(Ns::kControlPoint)};
  if (f.valid) {
    s.insert(nsName(Ns::kClosestPoint));
    if (drawable(f.conv) || drawable(f.circ) || drawable(f.Phi_T) || drawable(f.F)) {
      s.insert(nsName(Ns::kField));
    }
  }
  if (std::hypot(out.cmd.vx, out.cmd.vy) > kMinVector) {
    s.insert(nsName(Ns::kCommand));
  }
  if (
    (out.phase == Phase::kRotateToField || out.phase == Phase::kRotateToHeading) &&
    f.has_target_heading) {
    s.insert(nsName(Ns::kHeading));
  }
  if (!out.predicted.empty()) {
    s.insert(nsName(Ns::kPrediction));
  }
  if (c.input->goal.has_value()) {
    s.insert(nsName(Ns::kGoal));
  }
  const vf::CbfSample & cbf = f.cbf;
  const bool cbf_drawn = cbf.on && cbf.count > 0 && (cbf.active || cbf.h < cbf.delta);
  if (f.contour.gate_open || f.contour.active || cbf_drawn) {
    s.insert(nsName(Ns::kContour));
  }
  if (c.curve_info != nullptr) {
    s.insert(nsName(Ns::kCurveInfo));
  }
  return s;
}

/// @brief The legend colour of an ADDed marker.
legend::Rgba expectedColor(Ns ns, std::int32_t id, const MarkerContext & c)
{
  namespace id_ = legend::id;
  const vf::TrackerOutput & out = *c.output;
  switch (ns) {
    case Ns::kStatus:
      return legend::phaseColor(out.phase);
    case Ns::kFootprint:
      if (Status::has(out.status, Status::kBlocked)) {
        return legend::kFootprintBlocked;
      }
      return out.shaping.collisionAhead() ? legend::kFootprintCapped : legend::kFootprintNormal;
    case Ns::kCurve:
      return id == id_::kCurveLine ? legend::kCurveColor : legend::kCurveRays;
    case Ns::kClosestPoint:
      return legend::kClosestPointColor;
    case Ns::kControlPoint:
      return legend::kControlPointColor;
    case Ns::kField:
      switch (id) {
        case id_::kFieldConvergence:
          return legend::kFieldConvergence;
        case id_::kFieldCirculation:
          return legend::kFieldCirculation;
        case id_::kFieldFeedforward:
          return legend::kFieldFeedforward;
        default:
          return legend::kFieldResult;
      }
    case Ns::kCommand:
      return legend::kCommandColor;
    case Ns::kHeading:
      return out.phase == Phase::kRotateToField ? legend::kHeadingToField : legend::kHeadingToGoal;
    case Ns::kPrediction:
      return id == id_::kPredictionCollision ? legend::kPredictionCollision
                                             : legend::kPredictionFree;
    case Ns::kGoal:
      switch (id) {
        case id_::kGoalPoint:
        case id_::kGoalReference:
          return legend::kGoalColor;
        case id_::kGoalToleranceRing:
        case id_::kGoalStopRing:
          return ring(legend::kGoalTolerance);
        case id_::kGoalYaw:
          return legend::kGoalTolerance;
        default:
          return legend::kGoalBrakingZone;
      }
    case Ns::kContour:
      switch (id) {
        case id_::kContourCells:
          return legend::kContourCells;
        case id_::kContourLambdaRing:
          return ring(legend::kContourLambda);
        case id_::kContourFullSwitchRing:
        case id_::kContourBlendStartRing:
          return ring(legend::kContourBlend);
        case id_::kContourSide:
          return legend::kContourLambda;
        default:
          return legend::kContourObstacle;
      }
    case Ns::kCurveInfo:
      return id == id_::kCurveInfoFrameVelocity ? legend::kCurveInfoFrame : legend::kCurveInfoColor;
  }
  return legend::kWhite;
}

bool finitePoint(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/// @brief The class contract of markers(), for any cycle.
void expectContract(const MarkerArray & a, const MarkerContext & c)
{
  ASSERT_EQ(a.markers.size(), legend::kMarkerCount);
  EXPECT_LE(a.markers.size(), 80U);
  std::size_t index = 0;
  for (std::size_t n = 0; n < legend::kNamespaceCount; ++n) {
    const legend::NamespaceSpec & spec = legend::kNamespaces[n];
    for (std::int32_t id = 0; id < spec.id_count; ++id, ++index) {
      const Marker & m = a.markers[index];
      SCOPED_TRACE(m.ns + "/" + std::to_string(m.id));
      EXPECT_EQ(m.ns, spec.name);
      EXPECT_EQ(m.id, id);
      EXPECT_EQ(m.header.frame_id, c.frame_id);
      EXPECT_EQ(m.header.stamp, c.stamp);
      EXPECT_FALSE(m.frame_locked);
      EXPECT_EQ(m.lifetime.sec, 0);
      EXPECT_EQ(m.lifetime.nanosec, 0U);
      ASSERT_TRUE(m.action == Marker::ADD || m.action == Marker::DELETE);
      EXPECT_TRUE(finitePoint(m.pose.position));
      EXPECT_TRUE(
        std::isfinite(m.pose.orientation.z) && std::isfinite(m.pose.orientation.w) &&
        std::isfinite(m.scale.x) && std::isfinite(m.scale.y) && std::isfinite(m.scale.z));
      for (const auto & p : m.points) {
        EXPECT_TRUE(finitePoint(p));
      }
      if (m.action != Marker::ADD) {
        EXPECT_TRUE(m.points.empty());
        continue;
      }
      EXPECT_TRUE(sameColor(m.color, expectedColor(static_cast<Ns>(n), id, c)));
      EXPECT_TRUE(m.colors.empty() || m.colors.size() == m.points.size());
      if (m.type == Marker::LINE_STRIP) {
        EXPECT_GE(m.points.size(), 2U);
      }
      if (m.type == Marker::ARROW) {
        EXPECT_EQ(m.points.size(), 2U);
      }
      if (m.type == Marker::LINE_LIST) {
        EXPECT_EQ(m.points.size() % 2, 0U);
        EXPECT_GE(m.points.size(), 2U);
      }
      if (m.type == Marker::TEXT_VIEW_FACING) {
        EXPECT_FALSE(m.text.empty());
        EXPECT_GT(m.scale.z, 0.0);
      }
      if (m.type == Marker::SPHERE || m.type == Marker::CUBE || m.type == Marker::CYLINDER) {
        EXPECT_GT(m.scale.x, 0.0);
      }
    }
  }
}

/// @brief Contract + exactly the expected namespaces.
void expectCycle(MarkerBuilder & builder, const MarkerContext & c)
{
  const MarkerArray & a = builder.markers(c);
  expectContract(a, c);
  EXPECT_EQ(present(a), expectedNamespaces(c));
}

const std::string & statusText(const MarkerArray & a)
{
  return find(a, Ns::kStatus, legend::id::kStatusText).text;
}

std::vector<Vec2> line(const Vec2 & a, const Vec2 & b) { return {a, b}; }

// ------------------------------------------------------------------------------------------------
// Phases
// ------------------------------------------------------------------------------------------------

TEST(MarkerBuilder, FollowWithoutObstacles)
{
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.15), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
  s.run(10);
  ASSERT_EQ(s.output().phase, Phase::kFollow);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_FALSE(added(a, Ns::kPrediction, legend::id::kPredictionPath));
  EXPECT_FALSE(added(a, Ns::kHeading, legend::id::kHeadingTarget));
  EXPECT_FALSE(added(a, Ns::kContour, legend::id::kContourText));
  EXPECT_FALSE(added(a, Ns::kCurveInfo, legend::id::kCurveInfoText));
  EXPECT_TRUE(added(a, Ns::kCurve, legend::id::kCurveRays));
  EXPECT_FALSE(added(a, Ns::kCurve, legend::id::kCurveVirtualVertex));
  EXPECT_TRUE(added(a, Ns::kField, legend::id::kFieldResult));
  EXPECT_FALSE(added(a, Ns::kField, legend::id::kFieldFeedforward));  // Phi_T = 0
  EXPECT_TRUE(added(a, Ns::kGoal, legend::id::kGoalReference));       // FL: x_ref = d != 0
  EXPECT_TRUE(added(a, Ns::kGoal, legend::id::kGoalBrakingZone));     // the window ends at g
  EXPECT_FALSE(added(a, Ns::kGoal, legend::id::kGoalYaw));
  EXPECT_EQ(statusText(a).rfind("FOLLOW", 0), 0U);
  EXPECT_TRUE(sameColor(find(a, Ns::kFootprint, 0).color, legend::kFootprintNormal));
  // The curve is the window, exactly.
  const Marker & curve = find(a, Ns::kCurve, legend::id::kCurveLine);
  ASSERT_EQ(curve.points.size(), 2U);
  EXPECT_EQ(curve.points[0].x, -1.0);
  EXPECT_EQ(curve.points[1].x, 4.0);
}

TEST(MarkerBuilder, WindowShortOfTheGoalShowsNoTerminalField)
{
  // The window ends at (2, 0), the goal is (4, 0): no terminal mode, so neither g_ref nor the
  // braking zone is drawn at the window end, and g, its rings and its yaw stay at the goal.
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(2.0, 0.0)), Pose2{Vec2(0.0, 0.15), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0), 0.5 * vf::kPi, true);
  s.run(10);
  ASSERT_TRUE(s.output().field.has_goal);
  ASSERT_FALSE(s.output().field.terminal);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_FALSE(added(a, Ns::kGoal, legend::id::kGoalReference));
  EXPECT_FALSE(added(a, Ns::kGoal, legend::id::kGoalBrakingZone));
  for (const std::int32_t id :
       {legend::id::kGoalPoint, legend::id::kGoalToleranceRing, legend::id::kGoalStopRing,
        legend::id::kGoalYaw}) {
    const Marker & m = find(a, Ns::kGoal, id);
    ASSERT_EQ(m.action, Marker::ADD) << id;
    const double x = id == legend::id::kGoalYaw ? m.points.at(0).x : m.pose.position.x;
    EXPECT_EQ(x, 4.0) << id;
  }
}

TEST(MarkerBuilder, RotateToFieldShowsTheTargetHeading)
{
  Scenario s(
    vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.0), vf::kPi});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
  s.step();
  ASSERT_EQ(s.output().phase, Phase::kRotateToField);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_TRUE(added(a, Ns::kHeading, legend::id::kHeadingTarget));
  EXPECT_TRUE(added(a, Ns::kHeading, legend::id::kHeadingArc));
  EXPECT_TRUE(sameColor(find(a, Ns::kHeading, 0).color, legend::kHeadingToField));
  EXPECT_TRUE(sameColor(find(a, Ns::kStatus, 0).color, legend::phaseColor(Phase::kRotateToField)));
}

TEST(MarkerBuilder, FinalApproach)
{
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(3.0, 0.0)), Pose2{Vec2(2.6, 0.0), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(3.0, 0.0));
  s.step();
  ASSERT_EQ(s.output().phase, Phase::kFinalApproach);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_FALSE(added(a, Ns::kHeading, legend::id::kHeadingTarget));
  EXPECT_FALSE(added(a, Ns::kGoal, legend::id::kGoalReference));  // x_ref = 0 at base_link
  EXPECT_TRUE(added(a, Ns::kGoal, legend::id::kGoalBrakingZone));
  EXPECT_TRUE(added(a, Ns::kGoal, legend::id::kGoalToleranceRing));
  EXPECT_TRUE(added(a, Ns::kGoal, legend::id::kGoalStopRing));
  EXPECT_NE(statusText(a).find("sigma"), std::string::npos);
}

TEST(MarkerBuilder, RotateToHeadingShowsTheGoalYaw)
{
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(3.0, 0.0)), Pose2{Vec2(2.97, 0.0), 0.0});
  vf::GoalInfo goal = vf_viz_test::goalAt(Vec2(3.0, 0.0), 0.5 * vf::kPi, true);
  goal.xy_reached = true;
  s.input().goal = goal;
  s.step();
  ASSERT_EQ(s.output().phase, Phase::kRotateToHeading);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_TRUE(added(a, Ns::kHeading, legend::id::kHeadingTarget));
  EXPECT_TRUE(sameColor(find(a, Ns::kHeading, 0).color, legend::kHeadingToGoal));
  EXPECT_TRUE(added(a, Ns::kGoal, legend::id::kGoalYaw));
  EXPECT_FALSE(added(a, Ns::kCommand, legend::id::kCommandArrow));  // rotation in place
}

TEST(MarkerBuilder, AReverseSegmentFlagsTheStatus)
{
  vf::CoreConfig config;
  config.motion.allow_reversing = true;
  Scenario s(config, line(Vec2(1.0, 0.0), Vec2(-3.0, 0.0)), Pose2{Vec2(0.0, 0.0), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(-3.0, 0.0));
  s.input().drive_sign = -1;
  s.run(20);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_NE(statusText(a).find(legend::kFlagReverse), std::string::npos);
  EXPECT_LT(s.output().cmd.vx, 0.0);  // backs up instead of turning around
}

TEST(MarkerBuilder, CancelStopsAndFlagsTheStatus)
{
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.0), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
  s.run(20);
  s.input().cancel = true;
  s.step();
  ASSERT_EQ(s.output().phase, Phase::kStopped);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_NE(statusText(a).find(legend::kFlagCancel), std::string::npos);
  EXPECT_EQ(statusText(a).rfind("STOPPED", 0), 0U);
  EXPECT_TRUE(sameColor(find(a, Ns::kStatus, 0).color, legend::kRed));
  EXPECT_FALSE(added(a, Ns::kHeading, legend::id::kHeadingTarget));
  // Decelerate to zero, then nothing to command.
  ASSERT_TRUE(s.runUntil(
    [](const vf::TrackerOutput & out) {
      return std::abs(out.cmd.vx) <= kMinVector && std::abs(out.cmd.vy) <= kMinVector;
    },
    100));
  const MarkerContext stopped = s.context();
  expectCycle(builder, stopped);
  EXPECT_FALSE(added(builder.markers(stopped), Ns::kCommand, legend::id::kCommandArrow));
}

// ------------------------------------------------------------------------------------------------
// Status combinations
// ------------------------------------------------------------------------------------------------

TEST(MarkerBuilder, ClearPredictionHasNoCollisionCube)
{
  World world(Vec2(-2.0, -2.0), 8.0, 4.0);
  world.addBox(Vec2(1.5, 1.2), Vec2(2.0, 1.6));
  world.commit();
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.0), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
  s.input().obstacles = world.map();
  s.run(10);
  ASSERT_EQ(s.output().phase, Phase::kFollow);
  ASSERT_FALSE(s.output().predicted.empty());
  ASSERT_LT(s.output().first_collision, 0);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_TRUE(added(a, Ns::kPrediction, legend::id::kPredictionPath));
  EXPECT_FALSE(added(a, Ns::kPrediction, legend::id::kPredictionCollision));
  EXPECT_FALSE(added(a, Ns::kPrediction, legend::id::kPredictionText));  // s_free = +inf
  const Marker & path = find(a, Ns::kPrediction, legend::id::kPredictionPath);
  ASSERT_EQ(path.points.size(), s.output().predicted.size());
  for (const auto & color : path.colors) {
    EXPECT_TRUE(sameColor(color, legend::kPredictionFree));
  }
}

TEST(MarkerBuilder, CollisionAheadThenBlocked)
{
  World world(Vec2(-2.0, -2.0), 8.0, 4.0);
  world.addBox(Vec2(1.4, -1.0), Vec2(1.6, 1.0));  // a wall across the path
  world.commit();
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.0), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
  s.input().obstacles = world.map();
  MarkerBuilder builder;

  ASSERT_TRUE(s.runUntil(
    [](const vf::TrackerOutput & out) {
      return out.shaping.collisionAhead() && !Status::has(out.status, Status::kBlocked);
    },
    200));
  {
    const MarkerContext c = s.context();
    expectCycle(builder, c);
    const MarkerArray & a = builder.markers(c);
    EXPECT_TRUE(sameColor(find(a, Ns::kFootprint, 0).color, legend::kFootprintCapped));
    EXPECT_TRUE(added(a, Ns::kPrediction, legend::id::kPredictionCollision));
    EXPECT_TRUE(added(a, Ns::kPrediction, legend::id::kPredictionText));
    EXPECT_EQ(find(a, Ns::kPrediction, legend::id::kPredictionText).text.rfind("s_free", 0), 0U);
    const Marker & path = find(a, Ns::kPrediction, legend::id::kPredictionPath);
    ASSERT_FALSE(path.colors.empty());
    EXPECT_TRUE(sameColor(path.colors.back(), legend::kPredictionCollision));
  }

  ASSERT_TRUE(s.runUntil(
    [](const vf::TrackerOutput & out) { return Status::has(out.status, Status::kBlocked); }, 400));
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_TRUE(sameColor(find(a, Ns::kFootprint, 0).color, legend::kFootprintBlocked));
  EXPECT_TRUE(added(a, Ns::kPrediction, legend::id::kPredictionCollision));
  EXPECT_NE(statusText(a).find(legend::kFlagBlocked), std::string::npos);
}

TEST(MarkerBuilder, ContourEpisodeShowsKLambdaAndSide)
{
  World world(Vec2(-2.5, -2.5), 9.5, 5.0);
  world.addDisc(Vec2(2.0, 0.0), 0.3);
  world.commit();
  Scenario s(
    vf_viz_test::contourConfig(), line(Vec2(-3.0, 0.0), Vec2(6.0, 0.0)),
    Pose2{Vec2(-1.0, 0.05), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(6.0, 0.0));
  s.input().obstacles = world.map();
  MarkerBuilder builder;
  bool gate_seen = false;
  ASSERT_TRUE(s.runUntil(
    [&](const vf::TrackerOutput & out) {
      if (out.field.contour.gate_open && !out.field.contour.active && !gate_seen) {
        gate_seen = true;
        const MarkerContext gate = s.context();
        expectCycle(builder, gate);
        EXPECT_TRUE(added(builder.markers(gate), Ns::kContour, legend::id::kContourText));
      }
      return out.field.contour.active;
    },
    400));
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourCells));
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourLambdaRing));
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourFullSwitchRing));
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourBlendStartRing));
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourSide));
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourObstaclePoint));
  // One cube per cell of K.
  std::size_t k_cells = 0;
  for (const std::uint8_t cell : s.scratch().k_mask) {
    k_cells += cell != 0U ? 1U : 0U;
  }
  EXPECT_EQ(find(a, Ns::kContour, legend::id::kContourCells).points.size(), k_cells);
  // The lambda ring is centred on x*_o with radius lambda.
  const Marker & lambda = find(a, Ns::kContour, legend::id::kContourLambdaRing);
  EXPECT_NEAR(
    lambda.scale.x, 2.0 * s.tracker().config().obstacles.obstacle_contour_distance, 1e-12);
  EXPECT_NEAR(lambda.pose.position.x, s.output().field.contour.x_o.x(), 1e-12);
  // The side arrow is sigma_c J n_o.
  const Marker & side = find(a, Ns::kContour, legend::id::kContourSide);
  const vf::ContourSample & cs = s.output().field.contour;
  const Vec2 expected = static_cast<double>(cs.sigma_c) * vf::J(cs.n_o.normalized());
  const Vec2 drawn(side.points[1].x - side.points[0].x, side.points[1].y - side.points[0].y);
  EXPECT_NEAR(drawn.normalized().dot(expected), 1.0, 1e-9);
  const std::string sign = cs.sigma_c > 0 ? "+1" : "-1";
  EXPECT_NE(statusText(a).find(std::string(legend::kFlagContour) + " " + sign), std::string::npos);
  EXPECT_EQ(find(a, Ns::kContour, legend::id::kContourText).text.rfind("contour " + sign, 0), 0U);
}

// obstacle_mode cbf (math_spec §6.7): the contour namespace shows the δ_safe ring about the
// evaluation point, the binding normals and the cbf text; the status carries the CBF flag.
TEST(MarkerBuilder, CbfShowsTheBarrierAndBindingNormals)
{
  World world(Vec2(-2.5, -2.5), 9.5, 5.0);
  world.addDisc(Vec2(2.0, 0.2), 0.15);
  world.commit();
  vf::CoreConfig config = vf_viz_test::contourConfig();
  config.obstacles.obstacle_mode = vf::ObstacleMode::kCbf;
  Scenario s(config, line(Vec2(-3.0, 0.0), Vec2(6.0, 0.0)), Pose2{Vec2(-1.0, 0.0), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(6.0, 0.0));
  s.input().obstacles = world.map();
  MarkerBuilder builder;
  ASSERT_TRUE(s.runUntil([](const vf::TrackerOutput & out) { return out.field.cbf.active; }, 400));
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  const vf::CbfSample & cbf = s.output().field.cbf;
  EXPECT_FALSE(added(a, Ns::kContour, legend::id::kContourCells)) << "no K in cbf mode";
  const Marker & disc = find(a, Ns::kContour, legend::id::kContourLambdaRing);
  EXPECT_NEAR(disc.scale.x, 2.0 * cbf.delta, 1e-12);
  EXPECT_NEAR(disc.pose.position.x, s.output().field.x_eval.x(), 1e-12);
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourObstacleLine));
  EXPECT_TRUE(added(a, Ns::kContour, legend::id::kContourSide));
  EXPECT_EQ(
    find(a, Ns::kContour, legend::id::kContourGateSamples).points.size(),
    static_cast<std::size_t>(cbf.count));
  EXPECT_EQ(find(a, Ns::kContour, legend::id::kContourText).text.rfind("cbf h ", 0), 0U);
  EXPECT_NE(statusText(a).find(legend::kFlagCbf), std::string::npos);
}

TEST(MarkerBuilder, Assumption3ClampAndFeedforward)
{
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.1), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
  s.run(5);
  // A moving curve whose feedforward was clamped, filled in on a real cycle.
  vf::TrackerOutput out = s.output();
  out.status |= Status::kAssumption3Clamped | Status::kCurvatureSaturated;
  out.field.Phi_T = Vec2(0.1, 0.05);
  MarkerContext c = s.context();
  c.output = &out;
  MarkerBuilder builder;
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_TRUE(added(a, Ns::kField, legend::id::kFieldFeedforward));
  EXPECT_NE(statusText(a).find(legend::kFlagAssumption3), std::string::npos);
  EXPECT_NE(statusText(a).find(legend::kFlagCurvature), std::string::npos);
  EXPECT_EQ(statusText(a).find(legend::kFlagBlocked), std::string::npos);
}

TEST(MarkerBuilder, NonFiniteInputsDeleteTheMarker)
{
  Scenario s(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.1), 0.0});
  s.run(3);
  vf::TrackerOutput out = s.output();
  out.field.x_star = Vec2(vf::kNaN, 0.0);
  out.field.F = Vec2(vf::kInf, 0.0);
  MarkerContext c = s.context();
  c.output = &out;
  MarkerBuilder builder;
  const MarkerArray & a = builder.markers(c);
  expectContract(a, c);
  EXPECT_FALSE(added(a, Ns::kClosestPoint, legend::id::kClosestPoint));
  EXPECT_FALSE(added(a, Ns::kField, legend::id::kFieldResult));
}

// ------------------------------------------------------------------------------------------------
// Curve, follower data, stability
// ------------------------------------------------------------------------------------------------

TEST(MarkerBuilder, VirtualTailVertex)
{
  Scenario s(
    vf::CoreConfig{}, {Vec2(-1.0, 0.0), Vec2(1.0, 0.0), Vec2(2.0, 0.3)},
    Pose2{Vec2(0.0, 0.1), 0.0});
  s.setVirtualVertex(2);
  s.run(2);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const Marker & vertex = find(builder.markers(c), Ns::kCurve, legend::id::kCurveVirtualVertex);
  ASSERT_EQ(vertex.action, Marker::ADD);
  EXPECT_EQ(vertex.pose.position.x, 2.0);
  EXPECT_EQ(vertex.pose.position.y, 0.3);
}

TEST(MarkerBuilder, FollowerCurveInfo)
{
  vectorfield_ros::CurveInfo info;
  info.laps = 3;
  info.laps_completed = 1;
  info.remaining_arc_length = 7.4;
  info.frame_moving = true;
  info.frame_velocity = Vec2(0.1, 0.0);
  Scenario s(
    vf::CoreConfig{}, vf_viz_test::circlePoints(Vec2::Zero(), 2.0, 64),
    Pose2{Vec2(2.1, 0.0), 0.5 * vf::kPi}, info);
  s.run(5);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  expectCycle(builder, c);
  const MarkerArray & a = builder.markers(c);
  EXPECT_FALSE(added(a, Ns::kGoal, legend::id::kGoalPoint));   // closed curve: no goal
  EXPECT_FALSE(added(a, Ns::kCurve, legend::id::kCurveRays));  // closed curve: no rays
  EXPECT_TRUE(added(a, Ns::kCurveInfo, legend::id::kCurveInfoDirection));
  EXPECT_TRUE(added(a, Ns::kCurveInfo, legend::id::kCurveInfoStart));
  EXPECT_TRUE(added(a, Ns::kCurveInfo, legend::id::kCurveInfoFrameVelocity));
  EXPECT_EQ(find(a, Ns::kCurveInfo, legend::id::kCurveInfoText).text, "lap 1/3 | rem 7.4 m");
  // A chevron (two segments) every metre, at s = 0.5, 1.5, ...
  const double length = c.input->curve->length(c.input->t);
  const auto chevrons = static_cast<std::size_t>(std::floor(length - 0.5)) + 1U;
  EXPECT_EQ(find(a, Ns::kCurveInfo, legend::id::kCurveInfoDirection).points.size(), 4U * chevrons);
  // The sampled curve closes on itself.
  const Marker & curve = find(a, Ns::kCurve, legend::id::kCurveLine);
  ASSERT_GE(curve.points.size(), 64U);
  EXPECT_NEAR(curve.points.front().x, curve.points.back().x, 1e-9);
  EXPECT_NEAR(curve.points.front().y, curve.points.back().y, 1e-9);
}

TEST(MarkerBuilder, IdsAreStableAcrossCalls)
{
  Scenario a(vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.1), 0.0});
  a.input().goal = vf_viz_test::goalAt(Vec2(4.0, 0.0));
  a.run(3);
  Scenario b(
    vf::CoreConfig{}, line(Vec2(-1.0, 0.0), Vec2(4.0, 0.0)), Pose2{Vec2(0.0, 0.0), vf::kPi});
  b.step();
  MarkerBuilder builder;
  std::vector<std::pair<std::string, std::int32_t>> first;
  for (const Marker & m : builder.markers(a.context()).markers) {
    first.emplace_back(m.ns, m.id);
  }
  std::vector<std::pair<std::string, std::int32_t>> second;
  for (const Marker & m : builder.markers(b.context()).markers) {
    second.emplace_back(m.ns, m.id);
  }
  EXPECT_EQ(first, second);
  std::set<std::pair<std::string, std::int32_t>> unique(first.begin(), first.end());
  EXPECT_EQ(unique.size(), first.size());
  // The heading namespace of the second call is deleted by the third.
  EXPECT_TRUE(added(builder.markers(b.context()), Ns::kHeading, 0));
  EXPECT_FALSE(added(builder.markers(a.context()), Ns::kHeading, 0));
}

TEST(MarkerBuilder, IncompleteContextDeletesEverything)
{
  MarkerBuilder builder;
  MarkerContext c;
  c.frame_id = "map";
  c.stamp = vf_viz_test::stampOf(3);
  const MarkerArray & a = builder.markers(c);
  ASSERT_EQ(a.markers.size(), legend::kMarkerCount);
  for (const Marker & m : a.markers) {
    EXPECT_EQ(m.action, Marker::DELETE);
    EXPECT_EQ(m.header.frame_id, "map");
  }
}

TEST(MarkerBuilder, DeleteAll)
{
  MarkerBuilder builder;
  const auto stamp = vf_viz_test::stampOf(7);
  const MarkerArray & a = builder.deleteAll(stamp, "map");
  ASSERT_EQ(a.markers.size(), 1U);
  EXPECT_EQ(a.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(a.markers[0].header.frame_id, "map");
  EXPECT_EQ(a.markers[0].header.stamp, stamp);
}

// ------------------------------------------------------------------------------------------------
// Other messages
// ------------------------------------------------------------------------------------------------

TEST(MarkerBuilder, FieldGridArrowsAndStreamlines)
{
  World world(Vec2(-2.5, -2.5), 9.5, 5.0);
  world.addDisc(Vec2(2.0, 0.0), 0.3);
  world.commit();
  Scenario s(
    vf_viz_test::contourConfig(), line(Vec2(-3.0, 0.0), Vec2(6.0, 0.0)),
    Pose2{Vec2(-1.0, 0.05), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(6.0, 0.0));
  s.input().obstacles = world.map();
  s.run(5);
  MarkerBuilder builder;
  const MarkerContext c = s.context();
  vectorfield_ros::VizConfig config;  // 3.0 m at 0.25 m: 12 x 12 cells
  const MarkerArray & arrows = builder.fieldGrid(c, config);
  ASSERT_GE(arrows.markers.size(), 2U);
  EXPECT_LE(arrows.markers.size(), 1U + 12U * 12U);
  EXPECT_GE(arrows.markers.size(), 1U + 12U * 12U - 4U);  // a node may sit on the goal
  EXPECT_EQ(arrows.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(arrows.markers[0].ns, "field_grid");
  std::set<std::int32_t> ids;
  for (std::size_t k = 1; k < arrows.markers.size(); ++k) {
    const Marker & m = arrows.markers[k];
    EXPECT_EQ(m.ns, "field_grid");
    EXPECT_EQ(m.type, Marker::ARROW);
    EXPECT_EQ(m.action, Marker::ADD);
    EXPECT_EQ(m.header.frame_id, c.frame_id);
    EXPECT_EQ(m.header.stamp, c.stamp);
    ASSERT_EQ(m.points.size(), 2U);
    const double length = std::hypot(m.points[1].x - m.points[0].x, m.points[1].y - m.points[0].y);
    EXPECT_LE(length, 0.9 * config.field_grid_resolution + 1e-12);
    // Inside the square around base_link.
    EXPECT_LE(std::abs(0.5 * (m.points[0].x + m.points[1].x) - c.input->pose.p.x()), 1.5);
    EXPECT_TRUE(m.color.r >= 0.0F && m.color.r <= 1.0F && m.color.a == 1.0F);
    ids.insert(m.id);
  }
  EXPECT_EQ(ids.size(), arrows.markers.size() - 1U);
  // The arrow at a node points along the field the controller would track there.
  const Marker & first = arrows.markers[1];
  const Vec2 node(
    0.5 * (first.points[0].x + first.points[1].x), 0.5 * (first.points[0].y + first.points[1].y));
  const Vec2 F = s.tracker().probe(node, *c.input, *c.state, *c.scratch).F;
  const Vec2 drawn(first.points[1].x - first.points[0].x, first.points[1].y - first.points[0].y);
  EXPECT_NEAR(drawn.normalized().dot(F.normalized()), 1.0, 1e-9);

  config.field_grid_style = vectorfield_ros::FieldGridStyle::kStreamlines;
  config.field_grid_size = 1.0;
  config.field_grid_resolution = 0.25;
  const MarkerArray & lines = builder.fieldGrid(c, config);
  EXPECT_EQ(lines.markers[0].action, Marker::DELETEALL);
  EXPECT_LE(lines.markers.size(), 1U + 4U * 4U);
  EXPECT_GE(lines.markers.size(), 2U);
  for (std::size_t k = 1; k < lines.markers.size(); ++k) {
    EXPECT_EQ(lines.markers[k].type, Marker::LINE_STRIP);
    EXPECT_GE(lines.markers[k].points.size(), 2U);
  }
}

TEST(MarkerBuilder, PathsAndObstacleDistance)
{
  World world(Vec2(-2.0, -2.0), 8.0, 4.0);
  world.addBox(Vec2(1.5, 1.2), Vec2(2.0, 1.6));
  world.commit();
  Scenario s(
    vf::CoreConfig{}, {Vec2(-1.0, 0.0), Vec2(1.0, 0.0), Vec2(4.0, 1.0)},
    Pose2{Vec2(0.0, 0.0), 0.0});
  s.input().goal = vf_viz_test::goalAt(Vec2(4.0, 1.0));
  s.input().obstacles = world.map();
  s.run(4);
  MarkerBuilder builder;
  const MarkerContext c = s.context();

  const nav_msgs::msg::Path & window = builder.window(c);
  EXPECT_EQ(window.header.frame_id, c.frame_id);
  ASSERT_EQ(window.poses.size(), 3U);
  EXPECT_EQ(window.poses[2].pose.position.x, 4.0);
  EXPECT_NEAR(window.poses[0].pose.orientation.w, 1.0, 1e-12);  // along +x
  EXPECT_EQ(window.poses[1].header.stamp, c.stamp);

  const nav_msgs::msg::Path & predicted = builder.predictedPath(c);
  ASSERT_EQ(predicted.poses.size(), s.output().predicted.size());
  ASSERT_FALSE(predicted.poses.empty());
  EXPECT_EQ(predicted.poses.back().pose.position.x, s.output().predicted.back().p.x());

  const nav_msgs::msg::OccupancyGrid & distance = builder.obstacleDistance(c);
  EXPECT_EQ(distance.header.frame_id, c.frame_id);
  ASSERT_EQ(distance.info.width, 160U);
  ASSERT_EQ(distance.info.height, 80U);
  EXPECT_NEAR(distance.info.resolution, 0.05, 1e-6);
  EXPECT_NEAR(distance.info.origin.position.x, -2.0, 1e-12);
  ASSERT_EQ(distance.data.size(), 160U * 80U);
  // A site cell is 100, a far cell 0.
  const auto at = [&](int ix, int iy) {
    return distance.data[static_cast<std::size_t>(iy) * 160U + static_cast<std::size_t>(ix)];
  };
  EXPECT_EQ(at(75, 70), 100);  // (1.775, 1.525) is inside the box
  EXPECT_EQ(at(5, 5), 0);
  for (const std::int8_t v : distance.data) {
    EXPECT_TRUE(v >= 0 && v <= 100);
  }

  // Without obstacles: an empty grid.
  s.input().obstacles.reset();
  s.step();
  const nav_msgs::msg::OccupancyGrid & empty = builder.obstacleDistance(s.context());
  EXPECT_EQ(empty.info.width, 0U);
  EXPECT_TRUE(empty.data.empty());
  EXPECT_TRUE(builder.predictedPath(s.context()).poses.empty());
}

}  // namespace
