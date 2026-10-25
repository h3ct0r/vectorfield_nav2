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

// Topic names and the <ns> rule, and the consistency of the visualization
// legend.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <string>

#include "vectorfield_core/status.hpp"
#include "vectorfield_ros/legend.hpp"
#include "vectorfield_ros/topics.hpp"
#include "vectorfield_ros/viz_config.hpp"

namespace
{

namespace legend = vectorfield_ros::legend;
namespace topics = vectorfield_ros::topics;

TEST(Topics, PluginNamespaceReplacesDots)
{
  EXPECT_EQ(topics::pluginNamespace("FollowPath"), "FollowPath");
  EXPECT_EQ(
    topics::pluginNamespace("FollowPath.primary_controller"), "FollowPath/primary_controller");
  EXPECT_EQ(topics::pluginNamespace("a.b.c"), "a/b/c");
  EXPECT_EQ(topics::pluginNamespace(""), "");
}

TEST(Topics, TopicNames)
{
  EXPECT_EQ(topics::topicName("FollowPath", topics::kMarkers), "FollowPath/markers");
  EXPECT_EQ(
    topics::topicName(topics::pluginNamespace("FollowPath.primary_controller"), topics::kWindow),
    "FollowPath/primary_controller/window");
  EXPECT_EQ(topics::topicName(topics::kPrivateNamespace, topics::kFieldState), "~/field_state");
  EXPECT_EQ(topics::topicName("", topics::kCurve), "curve");
  EXPECT_EQ(topics::topicName("ns/", topics::kFieldGrid), "ns/field_grid");
}

TEST(Legend, NamespacesAreUniqueAndBounded)
{
  std::set<std::string> names;
  std::size_t total = 0;
  for (const legend::NamespaceSpec & ns : legend::kNamespaces) {
    EXPECT_TRUE(names.insert(std::string(ns.name)).second) << ns.name;
    EXPECT_GT(ns.id_count, 0) << ns.name;
    total += static_cast<std::size_t>(ns.id_count);
  }
  EXPECT_EQ(total, legend::kMarkerCount);
  EXPECT_LE(legend::kMarkerCount, 80U);
  EXPECT_EQ(legend::spec(legend::Ns::kStatus).name, "status");
  EXPECT_EQ(legend::spec(legend::Ns::kContour).name, "contour");
  EXPECT_EQ(legend::spec(legend::Ns::kCurveInfo).name, "curve_info");
}

TEST(Legend, LastIdOfEachNamespaceIsInRange)
{
  EXPECT_EQ(legend::spec(legend::Ns::kStatus).id_count, legend::id::kStatusText + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kFootprint).id_count, legend::id::kFootprintOutline + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kCurve).id_count, legend::id::kCurveVirtualVertex + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kClosestPoint).id_count, legend::id::kClosestDistance + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kControlPoint).id_count, legend::id::kControlLine + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kField).id_count, legend::id::kFieldResult + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kCommand).id_count, legend::id::kCommandArc + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kHeading).id_count, legend::id::kHeadingArc + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kPrediction).id_count, legend::id::kPredictionText + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kGoal).id_count, legend::id::kGoalBrakingZone + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kContour).id_count, legend::id::kContourText + 1);
  EXPECT_EQ(legend::spec(legend::Ns::kCurveInfo).id_count, legend::id::kCurveInfoFrameVelocity + 1);
}

TEST(Legend, PhaseColoursAreDistinct)
{
  using vectorfield_core::Phase;
  const Phase phases[] = {
    Phase::kRotateToField, Phase::kFollow, Phase::kFinalApproach, Phase::kRotateToHeading,
    Phase::kStopped};
  for (const Phase a : phases) {
    for (const Phase b : phases) {
      if (a != b) {
        EXPECT_FALSE(legend::phaseColor(a) == legend::phaseColor(b))
          << vectorfield_core::phaseName(a) << " vs " << vectorfield_core::phaseName(b);
      }
    }
  }
  EXPECT_TRUE(legend::phaseColor(Phase::kFollow) == legend::kGreen);
  EXPECT_TRUE(legend::phaseColor(Phase::kStopped) == legend::kRed);
}

/// WCAG relative luminance of an sRGB colour.
double luminance(const legend::Rgba & c)
{
  const auto linear = [](double v) {
    return v <= 0.04045 ? v / 12.92 : std::pow((v + 0.055) / 1.055, 2.4);
  };
  return 0.2126 * linear(c.r) + 0.7152 * linear(c.g) + 0.0722 * linear(c.b);
}

/// WCAG contrast ratio of two luminances.
double contrast(double a, double b) { return (std::max(a, b) + 0.05) / (std::min(a, b) + 0.05); }

TEST(Legend, NoColourVanishesOnAWhiteMapOrADarkBackground)
{
  const double white = 1.0;
  const double dark = luminance(legend::Rgba{48.0F / 255.0F, 48.0F / 255.0F, 48.0F / 255.0F, 1.0F});
  // Every colour of every namespace shows on both; the curve, the closest point, the resulting
  // field, the goal and curve_info (white until 0.2.0) clear 2.5:1 on both.
  const legend::Rgba all[] = {
    legend::kFootprintNormal,     legend::kFootprintCapped,  legend::kFootprintBlocked,
    legend::kCurveColor,          legend::kCurveRays,        legend::kClosestPointColor,
    legend::kControlPointColor,   legend::kFieldConvergence, legend::kFieldCirculation,
    legend::kFieldFeedforward,    legend::kFieldResult,      legend::kCommandColor,
    legend::kHeadingToField,      legend::kHeadingToGoal,    legend::kPredictionFree,
    legend::kPredictionCollision, legend::kGoalColor,        legend::kGoalBrakingZone,
    legend::kGoalTolerance,       legend::kContourObstacle,  legend::kContourLambda,
    legend::kContourBlend,        legend::kCurveInfoColor,   legend::kCurveInfoFrame};
  for (const legend::Rgba & c : all) {
    EXPECT_GE(contrast(luminance(c), white), 1.7) << c.r << " " << c.g << " " << c.b;
    EXPECT_GE(contrast(luminance(c), dark), 2.5) << c.r << " " << c.g << " " << c.b;
  }
  const legend::Rgba both[] = {
    legend::kCurveColor, legend::kClosestPointColor, legend::kFieldResult, legend::kGoalColor,
    legend::kCurveInfoColor};
  for (const legend::Rgba & c : both) {
    EXPECT_GE(contrast(luminance(c), white), 2.5) << c.r << " " << c.g << " " << c.b;
    EXPECT_GE(contrast(luminance(c), dark), 2.5) << c.r << " " << c.g << " " << c.b;
  }
}

TEST(VizConfig, FieldGridStyleStrings)
{
  using vectorfield_ros::FieldGridStyle;
  FieldGridStyle style = FieldGridStyle::kArrows;
  EXPECT_TRUE(vectorfield_ros::fromString("streamlines", &style));
  EXPECT_EQ(style, FieldGridStyle::kStreamlines);
  EXPECT_FALSE(vectorfield_ros::fromString("dots", &style));
  EXPECT_EQ(style, FieldGridStyle::kStreamlines);
  EXPECT_EQ(std::string(vectorfield_ros::toString(FieldGridStyle::kArrows)), "arrows");
}

}  // namespace
