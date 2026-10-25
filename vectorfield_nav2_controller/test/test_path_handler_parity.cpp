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

// Parity of LegacyPathHandler with Nav2's FeasiblePathHandler (gating on
// Gen C, compiled out elsewhere). Both handlers get the same explicit parameters, the harness's
// costmap (configured, fixed origin, not rolling) and its TF buffer (the only one), and are driven
// in lockstep over multi-cycle robot-pose sequences: straight (with a jump past the search range),
// U-turn, edge clip, unit and empty plans, robot past the end. Windows and goals must agree to
// 1e-9 and both must throw the same exception type in the same cycles. Lyrical's silent per-pose
// TF failure is a documented difference (nav2_api_matrix.md §3.3) and is not exercised: every
// transform here exists.

#include <gtest/gtest.h>

#include "vectorfield_ros/compat.hpp"

#if VF_HAS_PATH_HANDLER

#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "harness_fixtures.hpp"
#include "nav2_core/path_handler.hpp"
#include "pluginlib/class_loader.hpp"
#include "vectorfield_nav2_controller/legacy_path_handler.hpp"
#include "vectorfield_ros/conversions.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace compat = vectorfield_ros::compat;
using vectorfield_nav2_controller::LegacyPathConfig;
using vectorfield_nav2_controller::LegacyPathHandler;
using vectorfield_nav2_controller::LegacyPathOutput;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::ServerHarness;
using vectorfield_nav2_controller::test::straightPlan;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

constexpr double kDeg = vf::kPi / 180.0;
constexpr double kTolerance = 1e-9;
/// The explicit parameters both handlers get.
constexpr double kPruneDistance = 1.5;
constexpr double kSearchDistance = 0.6;
/// The map→odom transform of every scenario (pose of odom in map).
const vf::Pose2 kMapToOdom{vf::Vec2(0.3, -0.2), 10.0 * kDeg};

/// The exception class of a handler call, "" for none.
std::string exceptionName(const std::exception & e)
{
  if (dynamic_cast<const compat::InvalidPath *>(&e) != nullptr) {
    return "InvalidPath";
  }
  if (dynamic_cast<const compat::ControllerTFError *>(&e) != nullptr) {
    return "ControllerTFError";
  }
  return std::string("other: ") + e.what();
}

/// One cycle of a handler.
struct HandlerCycle
{
  std::string exception;
  nav_msgs::msg::Path window;
  geometry_msgs::msg::PoseStamped goal;
};

void expectPoseNear(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b,
  const std::string & what)
{
  EXPECT_EQ(a.header.frame_id, b.header.frame_id) << what;
  EXPECT_NEAR(a.pose.position.x, b.pose.position.x, kTolerance) << what;
  EXPECT_NEAR(a.pose.position.y, b.pose.position.y, kTolerance) << what;
  EXPECT_NEAR(a.pose.position.z, b.pose.position.z, kTolerance) << what;
  EXPECT_NEAR(a.pose.orientation.x, b.pose.orientation.x, kTolerance) << what;
  EXPECT_NEAR(a.pose.orientation.y, b.pose.orientation.y, kTolerance) << what;
  EXPECT_NEAR(a.pose.orientation.z, b.pose.orientation.z, kTolerance) << what;
  EXPECT_NEAR(a.pose.orientation.w, b.pose.orientation.w, kTolerance) << what;
}

/// FeasiblePathHandler (pluginlib) and LegacyPathHandler on one harness's costmap and TF buffer.
class HandlerPair
{
public:
  /// @param id FeasiblePathHandler's plugin name (its parameter prefix).
  HandlerPair(ServerHarness & harness, const std::string & id, bool reject_unit_path)
  : harness_(harness), loader_("nav2_core", "nav2_core::PathHandler")
  {
    feasible_ = loader_.createSharedInstance("nav2_controller::FeasiblePathHandler");
    feasible_->initialize(
      compat::NodeWeakPtr(harness.node()), harness.node()->get_logger(), id, harness.costmap(),
      harness.tf());
    LegacyPathConfig config;
    config.prune_distance = kPruneDistance;
    config.max_robot_pose_search_dist = kSearchDistance;
    config.reject_unit_path = reject_unit_path;
    legacy_ = std::make_unique<LegacyPathHandler>(config, harness.tf(), harness.costmap());
  }

  void setPlan(const nav_msgs::msg::Path & plan)
  {
    feasible_->setPlan(plan);
    legacy_->setPlan(plan);
  }

  /// One lockstep cycle with the robot at `robot_map` (map frame), compared.
  void cycle(const vf::Pose2 & robot_map, const std::string & what)
  {
    const vf::Isometry2 map_from_odom = vf::Isometry2::fromPose(kMapToOdom);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = harness_.options().odom_frame;
    pose.header.stamp = harness_.node()->now();
    pose.pose = vectorfield_ros::toPoseMsg(
      vf::Pose2{map_from_odom.applyInverse(robot_map.p), robot_map.yaw - kMapToOdom.yaw});

    HandlerCycle nav2;
    try {
      const nav2_core::PathSegment segment = feasible_->findPlanSegment(pose);
      nav2.window = feasible_->transformLocalPlan(segment.first, segment.second);
      nav2.goal = feasible_->getTransformedGoal(pose.header.stamp);
    } catch (const std::exception & e) {
      nav2.exception = exceptionName(e);
    }
    HandlerCycle ours;
    try {
      LegacyPathOutput out;
      legacy_->computeWindow(pose, &out);
      ours.window = out.window;
      ours.goal = out.goal;
    } catch (const std::exception & e) {
      ours.exception = exceptionName(e);
    }

    ++cycles_;
    ASSERT_EQ(ours.exception, nav2.exception) << what;
    if (!nav2.exception.empty()) {
      return;
    }
    EXPECT_EQ(ours.window.header.frame_id, nav2.window.header.frame_id) << what;
    EXPECT_EQ(ours.window.header.stamp.sec, nav2.window.header.stamp.sec) << what;
    EXPECT_EQ(ours.window.header.stamp.nanosec, nav2.window.header.stamp.nanosec) << what;
    ASSERT_EQ(ours.window.poses.size(), nav2.window.poses.size()) << what;
    for (std::size_t i = 0; i < nav2.window.poses.size(); ++i) {
      expectPoseNear(
        ours.window.poses[i], nav2.window.poses[i], what + ", pose " + std::to_string(i));
    }
    expectPoseNear(ours.goal, nav2.goal, what + ", goal");
    windows_ += 1;
    last_window_ = nav2.window;
  }

  /// Cycles run and cycles that produced a window.
  std::size_t cycles() const noexcept { return cycles_; }
  std::size_t windows() const noexcept { return windows_; }
  /// FeasiblePathHandler's last window.
  const nav_msgs::msg::Path & lastWindow() const noexcept { return last_window_; }

private:
  ServerHarness & harness_;
  pluginlib::ClassLoader<nav2_core::PathHandler> loader_;
  std::shared_ptr<nav2_core::PathHandler> feasible_;
  std::unique_ptr<LegacyPathHandler> legacy_;
  std::size_t cycles_{0};
  std::size_t windows_{0};
  nav_msgs::msg::Path last_window_;
};

HarnessOptions parityOptions()
{
  HarnessOptions options;
  options.costmap_size = 4.0;
  options.costmap_center = vf::Vec2(0.5, 0.0);
  options.map_to_odom = kMapToOdom;
  options.path_handler_parameters = {
    rclcpp::Parameter("prune_distance", kPruneDistance),
    rclcpp::Parameter("max_robot_pose_search_dist", kSearchDistance),
    rclcpp::Parameter("reject_unit_path", false)};
  options.parameters = {
    rclcpp::Parameter("StrictPathHandler.prune_distance", kPruneDistance),
    rclcpp::Parameter("StrictPathHandler.max_robot_pose_search_dist", kSearchDistance),
    rclcpp::Parameter("StrictPathHandler.reject_unit_path", true)};
  return options;
}

/// Drives `pair` along `robot` poses (map frame).
void drive(HandlerPair & pair, const std::vector<vf::Pose2> & robot, const std::string & scenario)
{
  for (std::size_t i = 0; i < robot.size(); ++i) {
    pair.cycle(robot[i], scenario + ", cycle " + std::to_string(i));
    if (::testing::Test::HasFatalFailure()) {
      return;
    }
  }
}

/// Poses every `step` along `plan`'s positions from index `from`, offset `lateral` to the left.
std::vector<vf::Pose2> alongPlan(
  const std::vector<vf::Pose2> & plan, std::size_t from, std::size_t step, double lateral)
{
  std::vector<vf::Pose2> robot;
  for (std::size_t i = from; i < plan.size(); i += step) {
    const vf::Vec2 left = vf::heading(plan[i].yaw + 0.5 * vf::kPi);
    robot.push_back(vf::Pose2{plan[i].p + lateral * left, plan[i].yaw});
  }
  return robot;
}

class PathHandlerParity : public ::testing::Test
{
protected:
  PathHandlerParity() : harness_(parityOptions()), pair_(harness_, "PathHandler", false) {}

  ServerHarness harness_;
  HandlerPair pair_;
};

TEST_F(PathHandlerParity, StraightPlanIncludingAJumpPastTheSearchRange)
{
  const std::vector<vf::Pose2> plan =
    straightPlan(vf::Vec2(-0.5, 0.0), vf::Vec2(1.6, 0.3), 0.05, 0.0);
  pair_.setPlan(mapPlan(plan));
  std::vector<vf::Pose2> robot = alongPlan(plan, 0, 3, 0.04);
  robot.resize(8);
  // A localization jump 0.9 m ahead: farther than max_robot_pose_search_dist.
  const std::vector<vf::Pose2> rest = alongPlan(plan, 7 * 3 + 18, 2, -0.03);
  robot.insert(robot.end(), rest.begin(), rest.end());
  drive(pair_, robot, "straight");
  EXPECT_EQ(pair_.windows(), robot.size());
}

TEST_F(PathHandlerParity, UTurnKeepsToTheLegItIsOn)
{
  // 1 m east, a half circle of radius 0.25 m to the left, 1 m west: the legs are 0.5 m apart, the
  // search range 0.6 m.
  std::vector<vf::Pose2> plan = straightPlan(vf::Vec2(-0.5, -0.5), vf::Vec2(0.5, -0.5), 0.05, 0.0);
  const vf::Vec2 centre(0.5, -0.25);
  for (int i = 1; i <= 16; ++i) {
    const double angle = -0.5 * vf::kPi + i * vf::kPi / 16.0;
    plan.push_back(vf::Pose2{centre + 0.25 * vf::heading(angle), angle + 0.5 * vf::kPi});
  }
  const std::vector<vf::Pose2> back =
    straightPlan(vf::Vec2(0.5, 0.0), vf::Vec2(-0.5, 0.0), 0.05, vf::kPi);
  plan.insert(plan.end(), back.begin() + 1, back.end());
  pair_.setPlan(mapPlan(plan));
  drive(pair_, alongPlan(plan, 0, 2, 0.06), "U-turn");
  EXPECT_GT(pair_.windows(), 20U);
}

TEST_F(PathHandlerParity, AWindowLeavingTheCostmapIsClippedAtItsEdge)
{
  // The costmap ends 2.0 m from its centre (odom frame); the plan runs to 3.2 m in the map frame.
  const std::vector<vf::Pose2> plan =
    straightPlan(vf::Vec2(0.0, 0.4), vf::Vec2(3.2, -0.2), 0.05, 0.0);
  pair_.setPlan(mapPlan(plan));
  const std::vector<vf::Pose2> robot = alongPlan(plan, 0, 4, 0.0);
  std::vector<vf::Pose2> inside;
  for (const vf::Pose2 & pose : robot) {
    if (pose.p.x() < 1.8) {
      inside.push_back(pose);
    }
  }
  drive(pair_, inside, "edge clip");
  EXPECT_GT(pair_.windows(), 5U);
  // The last window stops at the costmap's edge (odom x = 2.5), short of prune_distance.
  ASSERT_FALSE(pair_.lastWindow().poses.empty());
  const double end_x = pair_.lastWindow().poses.back().pose.position.x;
  EXPECT_GT(end_x, 2.4);
  EXPECT_LT(end_x, 2.5);
}

TEST_F(PathHandlerParity, UnitAndEmptyPlans)
{
  const vf::Pose2 target{vf::Vec2(0.8, 0.1), 30.0 * kDeg};
  pair_.setPlan(mapPlan({target}));
  drive(
    pair_, {vf::Pose2{vf::Vec2(0.2, 0.0), 0.0}, vf::Pose2{vf::Vec2(0.79, 0.1), 0.0}}, "unit plan");
  EXPECT_EQ(pair_.windows(), 2U);

  // reject_unit_path: both throw InvalidPath.
  HandlerPair strict(harness_, "StrictPathHandler", true);
  strict.setPlan(mapPlan({target}));
  drive(strict, {vf::Pose2{vf::Vec2(0.2, 0.0), 0.0}}, "rejected unit plan");
  EXPECT_EQ(strict.windows(), 0U);
  EXPECT_EQ(strict.cycles(), 1U);

  // An empty plan: both throw InvalidPath.
  nav_msgs::msg::Path empty;
  empty.header.frame_id = "map";
  pair_.setPlan(empty);
  drive(pair_, {vf::Pose2{vf::Vec2(0.2, 0.0), 0.0}}, "empty plan");
  EXPECT_EQ(pair_.windows(), 2U);
}

TEST_F(PathHandlerParity, RobotPastTheEndKeepsTwoPoses)
{
  const std::vector<vf::Pose2> plan =
    straightPlan(vf::Vec2(0.0, 0.0), vf::Vec2(1.0, 0.0), 0.05, 0.0);
  pair_.setPlan(mapPlan(plan));
  std::vector<vf::Pose2> robot = alongPlan(plan, 0, 5, 0.02);
  for (const double x : {1.05, 1.2, 1.4, 1.3}) {
    robot.push_back(vf::Pose2{vf::Vec2(x, 0.1), 0.0});
  }
  drive(pair_, robot, "past the end");
  EXPECT_EQ(pair_.windows(), robot.size());
}

}  // namespace

#else  // Gen A/B: no nav2_core::PathHandler to compare with

TEST(PathHandlerParity, GatesOnGenCOnly)
{
  GTEST_SKIP() << "FeasiblePathHandler exists on Lyrical and later";
}

#endif
