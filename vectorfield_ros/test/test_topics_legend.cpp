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

// Topic names and the <ns> rule (plan §4 "Topics"), and the consistency of the visualization
// legend (plan §4.1).

#include <gtest/gtest.h>

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
