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

// Layer 2 (plan §9), through the server harness: a blocked path maps to NoValidControl after the
// state is committed, so the server's patience resumes a short block and aborts a persistent one
// (plan §5.2 step 6); and a cancel decelerates through the core's STOPPED phase until cancel()
// returns true (plan §5.3; Humble stops at once without telling the plugin).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include "harness_fixtures.hpp"
#include "vectorfield_ros/compat.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_nav2_controller::test::countExceptions;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::describe;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::modelOptions;
using vectorfield_nav2_controller::test::pluginParameter;
using vectorfield_nav2_controller::test::ServerHarness;
using vectorfield_nav2_controller::test::straightPlan;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

/// A box across the straight path, 7 cm in front of the padded 0.23 m footprint: inside
/// stop_margin from the first cycle.
const vf::Vec2 kBoxMin(0.30, -0.4);
const vf::Vec2 kBoxMax(0.50, 0.4);

TEST(Blocked, AShortBlockWithinFailureToleranceResumesAndSucceeds)
{
  HarnessOptions options = modelOptions();
  options.failure_tolerance = 1.0;
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  harness.setLethalBox(kBoxMin, kBoxMax);
  const double removal = harness.now() + 0.5;
  bool present = true;
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(1.5, 0.0), 0.05, 0.0)));
  const GoalStatus status = harness.runUntilDone(20.0, [&](const CycleRecord &) {
    if (present && harness.now() >= removal - 1e-9) {
      harness.clearObstacles();
      present = false;
    }
  });
  ASSERT_EQ(status, GoalStatus::kSucceeded) << describe(harness);
  EXPECT_GE(countExceptions(harness, "NoValidControl"), 5U);
  for (const CycleRecord & record : harness.history()) {
    if (record.t >= removal + 0.05) {
      EXPECT_TRUE(record.exception.empty()) << "t = " << record.t << ": " << record.message;
    }
    if (!record.exception.empty()) {
      EXPECT_EQ(record.published.linear.x, 0.0);
      EXPECT_NE(record.message.find("BLOCKED"), std::string::npos) << record.message;
    }
  }
}

TEST(Blocked, APersistentBlockExhaustsPatienceWithoutMotion)
{
  HarnessOptions options = modelOptions();
  options.failure_tolerance = 0.5;
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  harness.setLethalBox(kBoxMin, kBoxMax);
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(1.5, 0.0), 0.05, 0.0)));
  ASSERT_EQ(harness.runUntilDone(5.0), GoalStatus::kAborted) << describe(harness);
  EXPECT_EQ(harness.result().reason, "PATIENCE_EXCEEDED") << describe(harness);
  std::size_t computed = 0;
  for (const CycleRecord & record : harness.history()) {
    if (!record.computed) {
      continue;
    }
    ++computed;
    EXPECT_EQ(record.exception, "NoValidControl") << "t = " << record.t;
    EXPECT_EQ(record.published.linear.x, 0.0);
    EXPECT_EQ(record.published.angular.z, 0.0);
    EXPECT_LT(record.pose_odom.p.norm(), 1e-12);
    EXPECT_EQ(record.pose_odom.yaw, 0.0);
  }
  // 0.5 s of patience at 20 Hz.
  EXPECT_GE(computed, 10U);
}

/// (motion model, extra parameter of the case).
using CancelCase = std::tuple<std::string, std::string>;

class CancelMidMotion : public ::testing::TestWithParam<CancelCase>
{
};

TEST_P(CancelMidMotion, DeceleratesAndReportsTheCancelWithinTheStoppingTime)
{
  const auto & [model, scenario] = GetParam();
  HarnessOptions options = modelOptions(model);
  vf::Vec2 goal(2.0, 0.0);
  if (scenario == "rotate_to_field") {
    // Skid steer turning in place toward a goal behind, with a 0.3 rad/s in-place floor.
    options.parameters.push_back(pluginParameter(options, "min_in_place_angular_vel", 0.3));
    goal = vf::Vec2(-1.0, 0.0);
  }
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), goal, 0.05, vf::angleOf(goal))));

  // Run until the robot moves in the phase of the case.
  const vf::Phase phase =
    scenario == "rotate_to_field" ? vf::Phase::kRotateToField : vf::Phase::kFollow;
  bool moving = false;
  for (int i = 0; i < 60 && !moving; ++i) {
    const CycleRecord & record = harness.spinOnce();
    ASSERT_EQ(harness.status(), GoalStatus::kRunning) << describe(harness);
    moving =
      record.computed && record.diagnostics.output.phase == phase &&
      (std::abs(record.published.linear.x) > 0.2 || std::abs(record.published.angular.z) > 0.6);
  }
  ASSERT_TRUE(moving) << "never moving in phase " << vf::phaseName(phase);
  const double speed = std::hypot(
    harness.history().back().published.linear.x, harness.history().back().published.linear.y);
  const double requested = harness.now();
  harness.requestCancel();

  if (!vectorfield_ros::compat::kServerPollsCancel) {
    // Humble: the server stops at once and the plugin is not told.
    const CycleRecord & record = harness.spinOnce();
    EXPECT_EQ(harness.status(), GoalStatus::kCanceled);
    EXPECT_FALSE(record.computed);
    EXPECT_EQ(record.published.linear.x, 0.0);
    return;
  }

  ASSERT_EQ(harness.runUntilDone(5.0), GoalStatus::kCanceled) << describe(harness);
  const double deceleration = harness.controller()->appliedConfig().phases.cancel_deceleration;
  const CycleRecord & last = harness.history().back();
  EXPECT_TRUE(last.cancel_called && last.cancel_result);
  EXPECT_LE(last.t - requested, speed / deceleration + 0.5 + 1e-9)
    << "speed at the request " << speed << " m/s";
  for (const CycleRecord & record : harness.history()) {
    if (record.t < requested) {
      continue;
    }
    EXPECT_TRUE(record.cancel_called);
    EXPECT_TRUE(record.exception.empty()) << record.message;
    if (record.computed) {
      EXPECT_TRUE(record.diagnostics.cancelling);
      EXPECT_EQ(record.diagnostics.output.phase, vf::Phase::kStopped);
    }
  }
  EXPECT_EQ(last.published.linear.x, 0.0);
  EXPECT_EQ(last.published.angular.z, 0.0);
}

INSTANTIATE_TEST_SUITE_P(
  Layer2, CancelMidMotion,
  ::testing::Values(
    CancelCase{"differential", "follow"}, CancelCase{"skid_steer", "rotate_to_field"},
    CancelCase{"omnidirectional", "follow"}, CancelCase{"ackermann", "follow"}),
  [](const ::testing::TestParamInfo<CancelCase> & case_info) {
    return std::get<0>(case_info.param) + "_" + std::get<1>(case_info.param);
  });

}  // namespace

namespace
{

/// Distance from a point to the axis-aligned box [lo, hi].
double boxDistance(const vf::Vec2 & p, const vf::Vec2 & lo, const vf::Vec2 & hi)
{
  const double dx = std::max({lo.x() - p.x(), 0.0, p.x() - hi.x()});
  const double dy = std::max({lo.y() - p.y(), 0.0, p.y() - hi.y()});
  return std::hypot(dx, dy);
}

}  // namespace

// obstacle_mode cbf (math_spec §6.7), layer 2: a box whose near side is 0.12 m beside the plan
// (inside the footprint's lateral reach) is passed without NoValidControl, the base keeps the
// footprint at least cbf_margin − res from the box, and the goal succeeds.
TEST(Cbf, ABoxBesideThePlanIsPassedWithTheMargin)
{
  for (const std::string & law :
       {std::string("feedback_linearization"), std::string("holonomic")}) {
    SCOPED_TRACE(law);
    const bool omni = law == "holonomic";
    HarnessOptions options = modelOptions(omni ? "omnidirectional" : "differential", law);
    options.parameters.push_back(pluginParameter(options, "obstacle_mode", std::string("cbf")));
    ServerHarness harness(options);
    harness.configure();
    harness.activate();
    const vf::Vec2 lo(0.8, 0.12);
    const vf::Vec2 hi(1.2, 0.5);
    harness.setLethalBox(lo, hi);
    harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(2.2, 0.0), 0.05, 0.0)));
    ASSERT_EQ(harness.runUntilDone(30.0), GoalStatus::kSucceeded) << describe(harness);
    EXPECT_EQ(countExceptions(harness, "NoValidControl"), 0U) << describe(harness);
    double clearance = vf::kInf;
    bool filtered = false;
    for (const CycleRecord & record : harness.history()) {
      clearance =
        std::min(clearance, boxDistance(record.pose_map.p, lo, hi) - options.robot_radius);
      filtered = filtered || (record.computed && record.diagnostics.output.field.cbf.active);
    }
    EXPECT_TRUE(filtered) << "the CBF filter never bound";
    const double margin = harness.controller()->appliedConfig().obstacles.cbf_margin;
    EXPECT_GE(clearance, margin - 0.05) << "closest footprint clearance to the box";
  }
}

// A box across the plan: the filter holds the robot in front of it (h ≈ 0), the CBF hold raises
// kBlocked (NoValidControl "BLOCKED"), and the goal ends on patience without a collision.
TEST(Cbf, ABoxAcrossThePlanHoldsAndExhaustsPatience)
{
  HarnessOptions options = modelOptions();
  options.failure_tolerance = 1.0;
  options.parameters.push_back(pluginParameter(options, "obstacle_mode", std::string("cbf")));
  ServerHarness harness(options);
  harness.configure();
  harness.activate();
  const vf::Vec2 lo(1.2, -0.4);
  const vf::Vec2 hi(1.4, 0.4);
  harness.setLethalBox(lo, hi);
  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(2.2, 0.0), 0.05, 0.0)));
  ASSERT_EQ(harness.runUntilDone(20.0), GoalStatus::kAborted) << describe(harness);
  EXPECT_EQ(harness.result().reason, "PATIENCE_EXCEEDED") << describe(harness);
  bool held = false;
  double clearance = vf::kInf;
  for (const CycleRecord & record : harness.history()) {
    clearance = std::min(clearance, boxDistance(record.pose_map.p, lo, hi) - options.robot_radius);
    if (!record.exception.empty()) {
      EXPECT_NE(record.message.find("BLOCKED"), std::string::npos) << record.message;
      held = held || (record.computed && record.diagnostics.output.field.cbf.hold);
    }
  }
  EXPECT_TRUE(held) << "kBlocked without the CBF hold";
  EXPECT_GT(clearance, 0.0);
}
