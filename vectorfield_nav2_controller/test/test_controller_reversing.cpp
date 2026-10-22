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

// Layer 2, M7 reversing (math_spec §9.4, docs/design/m7_reversing.md): plans with a cusp under
// allow_reversing for each motion model, a cusp that jitters across replans, sharp forward
// corners that are no cusps (math_spec §2.8, 0.2.1), and (Lyrical) the path handler's
// enforce_path_inversion and enforce_path_rotation.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <tuple>
#include <vector>

#include "harness_fixtures.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::test::countExceptions;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::describe;
using vectorfield_nav2_controller::test::finalDistance;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::modelOptions;
using vectorfield_nav2_controller::test::pluginParameter;
using vectorfield_nav2_controller::test::ServerHarness;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

constexpr double kHeading = 0.3;
constexpr double kCuspTol = 0.05;

/// Poses every `spacing` from a to b (both included), all with the vehicle yaw `yaw`.
std::vector<vf::Pose2> line(const vf::Vec2 & a, const vf::Vec2 & b, double spacing, double yaw)
{
  std::vector<vf::Pose2> out;
  const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / spacing)));
  for (int k = 0; k <= n; ++k) {
    out.push_back(vf::Pose2{vf::Vec2(a + (b - a) * (static_cast<double>(k) / n)), yaw});
  }
  return out;
}

/// An arc driven from `start` with drive sign q, steering side s (+1 left), through `angle`.
std::vector<vf::Pose2> arc(
  const vf::Pose2 & start, double R, int q, int s, double angle, double spacing)
{
  const vf::Vec2 centre = start.p + (s * R) * vf::J(vf::heading(start.yaw));
  const int n = static_cast<int>(std::ceil(R * angle / spacing));
  std::vector<vf::Pose2> out;
  for (int k = 0; k <= n; ++k) {
    const double a = q * s * angle * k / n;
    out.push_back(vf::Pose2{vf::Vec2(centre + vf::rotate(a, start.p - centre)), start.yaw + a});
  }
  return out;
}

/// From `from` forward along kHeading to the cusp `cusp_s` (arc length from the origin), then
/// in reverse on a 90° right arc of 0.6 m: the goal ends 0.6 m to the side, away from the
/// forward leg (Nav2 checks the goal on every cycle).
std::vector<vf::Pose2> cuspPlan(const vf::Vec2 & from, double cusp_s)
{
  const vf::Vec2 u = vf::heading(kHeading);
  std::vector<vf::Pose2> plan = line(from, cusp_s * u, 0.05, kHeading);
  const std::vector<vf::Pose2> back = arc(plan.back(), 0.6, -1, -1, 0.5 * vf::kPi, 0.05);
  plan.insert(plan.end(), back.begin() + 1, back.end());
  return plan;
}

HarnessOptions reversingOptions(const std::string & model, const std::string & law)
{
  HarnessOptions options = modelOptions(model, law);
  options.initial_pose = vf::Pose2{vf::Vec2::Zero(), kHeading};
  options.parameters.push_back(pluginParameter(options, "allow_reversing", true));
  options.parameters.push_back(pluginParameter(options, "cusp_tolerance", kCuspTol));
  return options;
}

struct Summary
{
  int flips{0};
  bool reversed{false};
  bool rotated{false};
  double switch_distance{vf::kInf};
  std::size_t segments{0};
};

Summary summarize(const ServerHarness & harness, const vf::Vec2 & cusp)
{
  Summary summary;
  int last_sign = 0;
  std::size_t last_segment = 0;
  for (const CycleRecord & r : harness.history()) {
    if (!r.computed) {
      continue;
    }
    const int sign = r.diagnostics.drive_sign;
    if (last_sign != 0 && sign != last_sign) {
      ++summary.flips;
    }
    if (r.diagnostics.segment != last_segment) {
      summary.switch_distance = std::min(summary.switch_distance, (r.pose_map.p - cusp).norm());
    }
    last_sign = sign;
    last_segment = r.diagnostics.segment;
    summary.segments = std::max(summary.segments, r.diagnostics.segment_count);
    summary.reversed = summary.reversed || (sign < 0 && r.published.linear.x < -0.05);
    summary.rotated = summary.rotated || r.diagnostics.output.phase == vf::Phase::kRotateToField;
  }
  return summary;
}

class ReversingModels : public ::testing::TestWithParam<std::tuple<std::string, std::string>>
{
};

TEST_P(ReversingModels, StopsAtTheCuspThenReversesToTheGoal)
{
  const auto & [model, law] = GetParam();
  ServerHarness harness(reversingOptions(model, law));
  harness.configure();
  harness.activate();
  const std::vector<vf::Pose2> plan = cuspPlan(vf::Vec2::Zero(), 1.0);
  const vf::Vec2 cusp = 1.0 * vf::heading(kHeading);
  harness.followPath(mapPlan(plan));
  ASSERT_EQ(harness.runUntilDone(60.0), GoalStatus::kSucceeded) << describe(harness);
  const Summary summary = summarize(harness, cusp);
  EXPECT_EQ(summary.segments, 2U);
  EXPECT_LE(summary.switch_distance, 2.0 * kCuspTol);
  EXPECT_LT(finalDistance(harness, plan.back().p), 0.25);
  if (model != "omnidirectional") {
    EXPECT_EQ(summary.flips, 1);
    EXPECT_TRUE(summary.reversed);
    EXPECT_FALSE(summary.rotated);  // backs up, never turns around
  }
}

INSTANTIATE_TEST_SUITE_P(
  M7, ReversingModels,
  ::testing::Values(
    std::make_tuple("differential", "feedback_linearization"),
    std::make_tuple("differential", "heading_tracking"), std::make_tuple("ackermann", "auto"),
    std::make_tuple("omnidirectional", "auto")),
  [](const ::testing::TestParamInfo<std::tuple<std::string, std::string>> & case_info) {
    return std::get<0>(case_info.param) + "_" + std::get<1>(case_info.param);
  });

class HeadinglessReverse : public ::testing::TestWithParam<std::string>
{
};

TEST_P(HeadinglessReverse, APlanBehindTheRobotIsBackedUp)
{
  // Identity orientations (Smac along -x, NavFn): the plan starts behind the robot, so it
  // backs up (math_spec §9.4, RPP's allow_reversing rule).
  HarnessOptions options = reversingOptions(GetParam(), "auto");
  options.initial_pose = vf::Pose2{};
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  const std::vector<vf::Pose2> plan = line(vf::Vec2::Zero(), vf::Vec2(-1.0, 0.0), 0.05, 0.0);
  harness.followPath(mapPlan(plan));
  ASSERT_EQ(harness.runUntilDone(60.0), GoalStatus::kSucceeded) << describe(harness);
  const Summary summary = summarize(harness, plan.back().p);
  EXPECT_TRUE(summary.reversed);
  EXPECT_FALSE(summary.rotated);
  EXPECT_EQ(summary.flips, 0);
  EXPECT_LT(std::abs(harness.history().back().pose_map.yaw), 0.2);
}

INSTANTIATE_TEST_SUITE_P(
  M7, HeadinglessReverse, ::testing::Values("differential", "ackermann"),
  [](const ::testing::TestParamInfo<std::string> & case_info) { return case_info.param; });

TEST(Reversing, ACuspThatJittersAcrossReplansDoesNotFlipTwice)
{
  ServerHarness harness(reversingOptions("differential", "auto"));
  harness.configure();
  harness.activate();
  harness.followPath(mapPlan(cuspPlan(vf::Vec2::Zero(), 1.0)));
  const vf::Vec2 u = vf::heading(kHeading);
  int replans = 0;
  std::size_t since_switch = 0;
  const GoalStatus status = harness.runUntilDone(60.0, [&](const CycleRecord & r) {
    if (!r.computed) {
      return;
    }
    const double s = r.pose_map.p.dot(u);
    if (r.diagnostics.segment == 0 && harness.history().size() % 20 == 0) {
      // While driving forward: the cusp one cell ahead or behind, the plan from the robot.
      harness.replan(mapPlan(cuspPlan(s * u, 1.0 + ((++replans % 2) != 0 ? 0.05 : -0.05))));
    } else if (r.diagnostics.drive_sign < 0 && ++since_switch == 2) {
      // Just after the switch: a plan whose cusp is 3 cells further (a forward piece between
      // l_c and 2 l_c), which the hysteresis skips.
      harness.replan(mapPlan(cuspPlan(s * u, s + 0.15)));
    }
  });
  ASSERT_EQ(status, GoalStatus::kSucceeded) << describe(harness);
  EXPECT_GT(replans, 2);
  EXPECT_EQ(summarize(harness, 1.0 * u).flips, 1);
}

TEST(Reversing, WithoutAllowReversingTheCuspIsStillInvalid)
{
  HarnessOptions options = modelOptions("differential", "auto");
  options.initial_pose = vf::Pose2{vf::Vec2::Zero(), kHeading};
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  harness.followPath(mapPlan(cuspPlan(vf::Vec2::Zero(), 1.0)));
  EXPECT_EQ(harness.status(), GoalStatus::kAborted);
  EXPECT_EQ(harness.result().reason, "INVALID_PATH") << describe(harness);
}

/// A NavigateThroughPoses plan of NavFn with a forward corner of `turn` (rad, to the left) at a
/// waypoint near (1.5, 0): identity orientations except the waypoint's and the goal's (along the
/// second leg), and the second leg from the grid vertex nearest the waypoint, 1.5 m long.
std::vector<vf::Pose2> throughPosesCorner(double turn)
{
  const vf::Vec2 waypoint(1.513, 0.007);
  const vf::Vec2 vertex(1.5, 0.0);
  const vf::Vec2 goal = vertex + 1.5 * vf::heading(turn);
  std::vector<vf::Pose2> plan = line(vf::Vec2::Zero(), waypoint, 0.025, 0.0);
  plan.back().yaw = turn;
  const std::vector<vf::Pose2> second = line(vertex, goal, 0.025, 0.0);
  plan.insert(plan.end(), second.begin(), second.end());
  plan.back().yaw = turn;
  return plan;
}

class SharpCorner : public ::testing::TestWithParam<std::tuple<std::string, bool>>
{
};

TEST_P(SharpCorner, A150DegreeWaypointCornerIsFollowedForward)
{
  // The chord rule of 0.2.0 read the corner as a cusp: INVALID_PATH, or with allow_reversing a
  // second leg driven backward. The window's despike() drops the junction's step back to the
  // next leg's start cell, which held ROTATE_TO_FIELD at the corner.
  const auto & [model, allow_reversing] = GetParam();
  HarnessOptions options = modelOptions(model, "auto");
  options.parameters.push_back(pluginParameter(options, "allow_reversing", allow_reversing));
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  const std::vector<vf::Pose2> plan = throughPosesCorner(150.0 * vf::kPi / 180.0);
  harness.followPath(mapPlan(plan));
  const GoalStatus status = harness.runUntilDone(90.0);
  const Summary summary = summarize(harness, plan.back().p);
  EXPECT_EQ(summary.segments, 1U);
  EXPECT_EQ(summary.flips, 0);
  EXPECT_FALSE(summary.reversed);
  if (model == "ackermann") {
    // A car cannot turn 150° at a point (that takes a Smac Hybrid-A* plan): the plan is accepted
    // and driven forward, and the controller reports the turn infeasible instead.
    EXPECT_EQ(status, GoalStatus::kAborted);
    EXPECT_EQ(harness.result().reason, "NO_VALID_CONTROL") << describe(harness);
    EXPECT_NE(harness.result().message.find("HEADING_INFEASIBLE"), std::string::npos);
    return;
  }
  ASSERT_EQ(status, GoalStatus::kSucceeded) << describe(harness);
  EXPECT_EQ(countExceptions(harness, "NoValidControl"), 0U);
  EXPECT_LT(finalDistance(harness, plan.back().p), 0.25);
}

INSTANTIATE_TEST_SUITE_P(
  NavigateThroughPoses, SharpCorner,
  ::testing::Values(
    std::make_tuple("differential", false), std::make_tuple("differential", true),
    std::make_tuple("omnidirectional", false), std::make_tuple("ackermann", false),
    std::make_tuple("ackermann", true)),
  [](const ::testing::TestParamInfo<std::tuple<std::string, bool>> & case_info) {
    return std::get<0>(case_info.param) +
           (std::get<1>(case_info.param) ? "_allow_reversing" : "_forward_only");
  });

/// Lyrical and later: the server's FeasiblePathHandler truncates its window at the constraint.
class PathConstraints : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (vectorfield_ros::compat::kApiGeneration != 'C') {
      GTEST_SKIP() << "path handlers exist on Lyrical and later only";
    }
  }
};

TEST_F(PathConstraints, EnforcePathInversion)
{
  HarnessOptions options = reversingOptions("ackermann", "auto");
  options.path_handler_parameters.emplace_back("enforce_path_inversion", true);
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  const std::vector<vf::Pose2> plan = cuspPlan(vf::Vec2::Zero(), 1.0);
  harness.followPath(mapPlan(plan));
  ASSERT_EQ(harness.runUntilDone(60.0), GoalStatus::kSucceeded) << describe(harness);
  const Summary summary = summarize(harness, 1.0 * vf::heading(kHeading));
  EXPECT_EQ(summary.flips, 1);
  EXPECT_LE(summary.switch_distance, 2.0 * kCuspTol);
  EXPECT_TRUE(harness.history().back().diagnostics.own_window);
  EXPECT_LT(finalDistance(harness, plan.back().p), 0.25);
}

TEST_F(PathConstraints, EnforcePathRotation)
{
  // Forward 1 m, an in-place turn of 90° in 10° poses, then 0.8 m along the new heading;
  // allow_reversing is not needed.
  HarnessOptions options = modelOptions("differential", "auto");
  options.path_handler_parameters.emplace_back("enforce_path_rotation", true);
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  std::vector<vf::Pose2> plan = line(vf::Vec2::Zero(), vf::Vec2(1.0, 0.0), 0.05, 0.0);
  for (int k = 1; k <= 9; ++k) {
    plan.push_back(vf::Pose2{vf::Vec2(1.0, 0.0), k * 10.0 * vf::kPi / 180.0});
  }
  const std::vector<vf::Pose2> up =
    line(vf::Vec2(1.0, 0.0), vf::Vec2(1.0, 0.8), 0.05, 0.5 * vf::kPi);
  plan.insert(plan.end(), up.begin() + 1, up.end());
  harness.followPath(mapPlan(plan));
  ASSERT_EQ(harness.runUntilDone(60.0), GoalStatus::kSucceeded) << describe(harness);
  EXPECT_TRUE(harness.history().back().diagnostics.own_window);
  EXPECT_EQ(harness.history().back().diagnostics.segment_count, 1U);
  EXPECT_LT(finalDistance(harness, plan.back().p), 0.25);
}

}  // namespace
