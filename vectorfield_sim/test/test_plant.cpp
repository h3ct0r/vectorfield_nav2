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
/// @brief KinematicPlant: integration against sim::integrate for every model, the command
/// timeout, the Ackermann infeasible count, simulated time and the map→odom bookkeeping.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_sim/plant.hpp"

namespace
{

namespace vf = vectorfield_core;
using vectorfield_sim::KinematicPlant;
using vectorfield_sim::PlantConfig;
using vf::MotionModel;
using vf::Pose2;
using vf::Twist2;
using vf::Vec2;

constexpr double kTol = 1e-12;

PlantConfig configFor(MotionModel model)
{
  PlantConfig config;
  config.kinematics.type = model;
  config.kinematics.min_turning_radius = 0.5;
  config.kinematics.reference_point_offset = -0.2;
  if (model == MotionModel::kSkidSteer) {
    config.slip = vectorfield_sim::kSkidSteerSlip;
  }
  config.step_ns = 10'000'000;
  config.command_timeout_ns = 0;
  return config;
}

void expectPoseNear(const Pose2 & actual, const Pose2 & expected, double tol)
{
  EXPECT_NEAR(actual.p.x(), expected.p.x(), tol);
  EXPECT_NEAR(actual.p.y(), expected.p.y(), tol);
  EXPECT_NEAR(vf::wrapAngle(actual.yaw - expected.yaw), 0.0, tol);
}

class PlantModels : public ::testing::TestWithParam<MotionModel>
{
};

TEST_P(PlantModels, MatchesSimIntegrate)
{
  const PlantConfig config = configFor(GetParam());
  KinematicPlant plant(config);
  const Twist2 command{0.4, 0.1, 0.6};
  plant.setCommand(command);
  Pose2 expected;
  for (int i = 0; i < 200; ++i) {
    plant.step();
    expected = vf::sim::integrate(config.kinematics, expected, command, 0.01, config.slip);
  }
  expectPoseNear(plant.odomPose(), expected, 1e-9);
  EXPECT_TRUE(plant.commandActive());
  EXPECT_EQ(plant.steps(), 200U);
  EXPECT_EQ(plant.timeNs(), 2'000'000'000);

  // The executed twist reproduces one step of sim::integrate to first order.
  const Twist2 twist = plant.twist();
  const double dt = 1e-6;
  const Pose2 start{Vec2(0.3, -0.2), 0.7};
  const Pose2 end = vf::sim::integrate(config.kinematics, start, command, dt, config.slip);
  const Vec2 body = vf::rotate(-start.yaw, end.p - start.p) / dt;
  EXPECT_NEAR(body.x(), twist.vx, 1e-5);
  EXPECT_NEAR(body.y(), twist.vy, 1e-5);
  EXPECT_NEAR((end.yaw - start.yaw) / dt, twist.wz, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
  AllModels, PlantModels,
  ::testing::Values(
    MotionModel::kDifferential, MotionModel::kSkidSteer, MotionModel::kOmnidirectional,
    MotionModel::kAckermann),
  [](const ::testing::TestParamInfo<MotionModel> & param_info) {
    return std::string(vf::toString(param_info.param));
  });

TEST(Plant, SkidSteerSlipDefaults)
{
  EXPECT_DOUBLE_EQ(vectorfield_sim::kSkidSteerSlip.eta_psi, 0.75);
  EXPECT_DOUBLE_EQ(vectorfield_sim::kSkidSteerSlip.x_icr, 0.05);
  const PlantConfig config = configFor(MotionModel::kSkidSteer);
  const Twist2 twist =
    vectorfield_sim::executedTwist(config.kinematics, {0.3, 0.2, 1.0}, config.slip);
  EXPECT_DOUBLE_EQ(twist.vx, 0.3);
  EXPECT_DOUBLE_EQ(twist.wz, 0.75);
  EXPECT_DOUBLE_EQ(twist.vy, -0.05 * 0.75);
}

TEST(Plant, NoCommandNoMotion)
{
  KinematicPlant plant(configFor(MotionModel::kDifferential));
  for (int i = 0; i < 10; ++i) {
    plant.step();
  }
  EXPECT_FALSE(plant.commandActive());
  expectPoseNear(plant.odomPose(), Pose2{}, 0.0);
  EXPECT_DOUBLE_EQ(plant.twist().vx, 0.0);
}

TEST(Plant, CommandTimeoutZeroesMotion)
{
  PlantConfig config = configFor(MotionModel::kDifferential);
  config.command_timeout_ns = 100'000'000;
  KinematicPlant plant(config);
  plant.setCommand({0.5, 0.0, 0.0});
  // Ages 0, 10, ..., 100 ms are within the timeout: 11 steps of 5 mm.
  for (int i = 0; i <= 10; ++i) {
    plant.step();
    EXPECT_TRUE(plant.commandActive()) << i;
  }
  plant.step();
  EXPECT_FALSE(plant.commandActive());
  EXPECT_DOUBLE_EQ(plant.twist().vx, 0.0);
  EXPECT_NEAR(plant.odomPose().p.x(), 11 * 0.005, kTol);
  plant.step();
  EXPECT_NEAR(plant.odomPose().p.x(), 11 * 0.005, kTol);

  // A fresh command restarts motion.
  plant.setCommand({0.5, 0.0, 0.0});
  plant.step();
  EXPECT_TRUE(plant.commandActive());
  EXPECT_NEAR(plant.odomPose().p.x(), 12 * 0.005, kTol);
}

TEST(Plant, AckermannInfeasibleCommandsAreCounted)
{
  KinematicPlant plant(configFor(MotionModel::kAckermann));
  EXPECT_TRUE(plant.setCommand({0.0, 0.0, 0.5}));
  EXPECT_FALSE(plant.setCommand({0.2, 0.0, 0.5}));
  EXPECT_FALSE(plant.setCommand({0.0, 0.0, 0.0}));
  EXPECT_TRUE(plant.setCommand({1e-7, 0.0, -0.3}));
  EXPECT_EQ(plant.infeasibleCount(), 2U);

  // v = 0 does not turn a car (R_min > 0).
  plant.setCommand({0.0, 0.0, 0.5});
  for (int i = 0; i < 50; ++i) {
    plant.step();
  }
  EXPECT_NEAR(plant.odomPose().yaw, 0.0, kTol);
  EXPECT_EQ(plant.infeasibleCount(), 3U);

  KinematicPlant differential(configFor(MotionModel::kDifferential));
  EXPECT_FALSE(differential.setCommand({0.0, 0.0, 0.5}));
  EXPECT_EQ(differential.infeasibleCount(), 0U);
  EXPECT_TRUE(vectorfield_sim::isInfeasibleAckermann({0.0, 0.0, 0.5}));
  EXPECT_FALSE(vectorfield_sim::isInfeasibleAckermann({0.0, 0.0, 1e-7}));
}

TEST(Plant, InitialPoseMovesBothMapToOdom)
{
  KinematicPlant plant(configFor(MotionModel::kDifferential), Pose2{Vec2(1.0, 2.0), 0.5});
  expectPoseNear(plant.groundTruth(), Pose2{Vec2(1.0, 2.0), 0.5}, kTol);
  plant.setCommand({0.5, 0.0, 0.4});
  for (int i = 0; i < 100; ++i) {
    plant.step();
  }
  const Pose2 odom = plant.odomPose();
  const Pose2 target{Vec2(-3.0, 4.0), -2.0};
  plant.setInitialPose(target);
  expectPoseNear(plant.groundTruth(), target, 1e-12);
  expectPoseNear(vf::Isometry2::fromPose(plant.mapToOdom()).apply(odom), target, 1e-12);
  expectPoseNear(plant.trueMapToOdom(), plant.mapToOdom(), 0.0);
  expectPoseNear(plant.odomPose(), odom, 0.0);
}

TEST(Plant, MapToOdomStepMovesOnlyThePublishedTransform)
{
  KinematicPlant plant(configFor(MotionModel::kDifferential), Pose2{Vec2(1.0, 2.0), 3.1});
  const Pose2 truth = plant.groundTruth();
  plant.stepMapToOdom(0.05, -0.02, 0.1);
  EXPECT_NEAR(plant.mapToOdom().p.x(), 1.05, kTol);
  EXPECT_NEAR(plant.mapToOdom().p.y(), 1.98, kTol);
  EXPECT_NEAR(plant.mapToOdom().yaw, vf::wrapAngle(3.2), kTol);
  expectPoseNear(plant.trueMapToOdom(), Pose2{Vec2(1.0, 2.0), 3.1}, 0.0);
  expectPoseNear(plant.groundTruth(), truth, 0.0);
}

TEST(Plant, TimeHelpers)
{
  EXPECT_EQ(vectorfield_sim::toNanoseconds(0.01), 10'000'000);
  EXPECT_EQ(vectorfield_sim::toNanoseconds(1.5), 1'500'000'000);
  EXPECT_EQ(vectorfield_sim::toNanoseconds(-1.0), 0);
  EXPECT_EQ(vectorfield_sim::toNanoseconds(std::numeric_limits<double>::quiet_NaN()), 0);
  EXPECT_EQ(vectorfield_sim::stepsPerPeriod(100'000'000, 10'000'000), 10);
  EXPECT_EQ(vectorfield_sim::stepsPerPeriod(100'000'000, 30'000'000), 3);
  EXPECT_EQ(vectorfield_sim::stepsPerPeriod(5'000'000, 10'000'000), 1);
  EXPECT_EQ(vectorfield_sim::stepsPerPeriod(100'000'000, 0), 1);
}

}  // namespace
