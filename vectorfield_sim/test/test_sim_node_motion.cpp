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
/// @brief KinematicSimNode in process, motion and time: a constant twist moves odom and TF as
/// sim::integrate predicts (every model, Twist and TwistStamped input), the command timeout, the
/// Ackermann infeasible-command count and `/clock`.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "sim_node_test_support.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_ros/compat.hpp"
#include "vectorfield_sim/plant.hpp"

namespace
{

using vf_sim_test::expectPoseNear;
using vf_sim_test::SimNodeTest;
using vf_sim_test::toNs;
using vf_sim_test::toPose2;
namespace vf = vectorfield_core;

struct MotionCase
{
  const char * model;
  bool stamped;
};

class SimNodeMotion : public SimNodeTest, public ::testing::WithParamInterface<MotionCase>
{
};

TEST_P(SimNodeMotion, ConstantTwistMovesOdomAndTfAsSimIntegrate)
{
  const MotionCase motion = GetParam();
  const vf::Pose2 initial{vf::Vec2(1.0, -2.0), 0.3};
  ASSERT_NO_FATAL_FAILURE(start({
    {"motion_model", std::string(motion.model)},
    {"enable_stamped_cmd_vel", motion.stamped},
    {"cmd_vel_timeout", 0.0},
    {"reference_point_offset", -0.2},
    {"initial_pose_x", initial.p.x()},
    {"initial_pose_y", initial.p.y()},
    {"initial_pose_yaw", initial.yaw},
  }));
  EXPECT_EQ(map_tf_.header.frame_id, "map");
  expectPoseNear(toPose2(map_tf_.transform), initial, 1e-12);

  vf::Kinematics kinematics;
  ASSERT_TRUE(vf::fromString(motion.model, &kinematics.type));
  kinematics.min_turning_radius = 0.5;
  kinematics.reference_point_offset = -0.2;
  const vf::sim::SlipParams slip = kinematics.type == vf::MotionModel::kSkidSteer
                                     ? vectorfield_sim::kSkidSteerSlip
                                     : vf::sim::SlipParams{};
  const double dt = 0.01;
  const vf::Twist2 command{0.4, 0.1, 0.6};

  ASSERT_NO_FATAL_FAILURE(activate(command));
  vf::Pose2 expected = vf::sim::integrate(kinematics, vf::Pose2{}, command, dt, slip);
  for (int i = 0; i < 50; ++i) {
    ASSERT_NO_FATAL_FAILURE(stepAndWait());
    expected = vf::sim::integrate(kinematics, expected, command, dt, slip);
  }

  EXPECT_EQ(odom_.header.frame_id, "odom");
  EXPECT_EQ(odom_.child_frame_id, "base_footprint");
  expectPoseNear(toPose2(odom_), expected, 1e-9);
  const vf::Twist2 twist = vectorfield_sim::executedTwist(kinematics, command, slip);
  EXPECT_NEAR(odom_.twist.twist.linear.x, twist.vx, 1e-12);
  EXPECT_NEAR(odom_.twist.twist.linear.y, twist.vy, 1e-12);
  EXPECT_NEAR(odom_.twist.twist.angular.z, twist.wz, 1e-12);

  ASSERT_TRUE(spinUntil([this]() { return toNs(odom_tf_.header.stamp) == timeNs(); }));
  EXPECT_EQ(odom_tf_.header.frame_id, "odom");
  expectPoseNear(toPose2(odom_tf_.transform), expected, 1e-9);

  ASSERT_TRUE(spinUntil([this]() { return toNs(truth_.header.stamp) == timeNs(); }));
  EXPECT_EQ(truth_.header.frame_id, "map");
  expectPoseNear(toPose2(truth_), vf::Isometry2::fromPose(initial).apply(expected), 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
  ModelsAndInputTypes, SimNodeMotion,
  ::testing::Values(
    MotionCase{"differential", false}, MotionCase{"differential", true},
    MotionCase{"skid_steer", false}, MotionCase{"skid_steer", true},
    MotionCase{"omnidirectional", false}, MotionCase{"omnidirectional", true},
    MotionCase{"ackermann", false}, MotionCase{"ackermann", true}),
  [](const ::testing::TestParamInfo<MotionCase> & param_info) {
    return std::string(param_info.param.model) +
           (param_info.param.stamped ? "_stamped" : "_unstamped");
  });

TEST_F(SimNodeTest, DefaultInputTypeIsTheDistroDefault)
{
  ASSERT_NO_FATAL_FAILURE(start());
  EXPECT_EQ(stamped_, vectorfield_ros::compat::kStampedCmdVelDefault);
}

TEST_F(SimNodeTest, CommandTimeoutZeroesMotion)
{
  ASSERT_NO_FATAL_FAILURE(start({{"cmd_vel_timeout", 0.05}}));
  ASSERT_NO_FATAL_FAILURE(activate({0.5, 0.0, 0.0}));
  for (int i = 0; i < 6; ++i) {
    ASSERT_NO_FATAL_FAILURE(stepAndWait());
  }
  // The command is now more than 50 ms old.
  EXPECT_DOUBLE_EQ(odom_.twist.twist.linear.x, 0.0);
  const double x = odom_.pose.pose.position.x;
  EXPECT_GT(x, 0.0);
  for (int i = 0; i < 5; ++i) {
    ASSERT_NO_FATAL_FAILURE(stepAndWait());
  }
  EXPECT_DOUBLE_EQ(odom_.pose.pose.position.x, x);
  EXPECT_DOUBLE_EQ(odom_.twist.twist.linear.x, 0.0);
}

TEST_F(SimNodeTest, AckermannInfeasibleCommandsAreCounted)
{
  ASSERT_NO_FATAL_FAILURE(start({{"motion_model", std::string("ackermann")}}));
  EXPECT_EQ(infeasible_, 0U);
  publish({0.0, 0.0, 0.5});
  ASSERT_TRUE(spinUntil([this]() { return infeasible_ == 1U; }));
  publish({0.3, 0.0, 0.2});
  publish({0.0, 0.0, -0.4});
  ASSERT_TRUE(spinUntil([this]() { return infeasible_ == 2U; }));
  // The feasible command in between was not counted.
  spinUntil([]() { return false; }, std::chrono::milliseconds(100));
  EXPECT_EQ(infeasible_, 2U);

  // A car does not turn in place.
  for (int i = 0; i < 20; ++i) {
    ASSERT_NO_FATAL_FAILURE(stepAndWait());
  }
  expectPoseNear(toPose2(odom_), vf::Pose2{}, 1e-12);
}

TEST_F(SimNodeTest, ClockAdvancesByTheFixedStep)
{
  ASSERT_NO_FATAL_FAILURE(start({{"step_size", 0.02}}));
  EXPECT_TRUE(clocks_.empty());
  for (int i = 0; i < 25; ++i) {
    ASSERT_NO_FATAL_FAILURE(stepAndWait());
  }
  ASSERT_EQ(clocks_.size(), 25U);
  for (std::size_t i = 0; i < clocks_.size(); ++i) {
    EXPECT_EQ(clocks_[i], static_cast<std::int64_t>(i + 1) * 20'000'000) << i;
  }
  EXPECT_EQ(toNs(odom_.header.stamp), 500'000'000);
}

TEST_F(SimNodeTest, StepTimerRunsWhenNotPaused)
{
  ASSERT_NO_FATAL_FAILURE(start({{"real_time_factor", 5.0}}, false));
  ASSERT_TRUE(spinUntil([this]() { return clocks_.size() >= 20; }));
  for (std::size_t i = 1; i < clocks_.size(); ++i) {
    const std::int64_t advance = clocks_[i] - clocks_[i - 1];
    EXPECT_GT(advance, 0) << i;
    EXPECT_EQ(advance % 10'000'000, 0) << i;
  }
}

}  // namespace
