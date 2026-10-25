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

// Configuration, auto values and the validator of math_spec §11, one test per rule. Values are
// the default test configuration (TC) of math_spec §12 and the TB3 auto values of §11.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
namespace issue = vectorfield_core::issue;
using vf::CoreConfig;
using vf::CostmapInfo;
using vf::MotionModel;
using vf::ObstacleMode;
using vf::TrackingLaw;
using vf::ValidationReport;
using vf::Vec2;

namespace
{

// TC footprint: a 16-gon of circumradius 0.22 with a vertex on +y (r_insc = 0.2158,
// w_lat = r_circ = 0.22).
std::vector<Vec2> sixteenGon(double circumradius)
{
  std::vector<Vec2> polygon;
  for (int k = 0; k < 16; ++k) {
    polygon.push_back(circumradius * vf::heading(vf::kTwoPi * k / 16.0));
  }
  return polygon;
}

// TC of math_spec §12 (the defaults with the 16-gon), contour on so every rule is active.
CoreConfig tc()
{
  CoreConfig config;
  config.node.footprint.polygon = sixteenGon(0.22);
  config.obstacles.obstacle_mode = ObstacleMode::kContour;
  return config;
}

// The stock 3 x 3 m costmap at 0.05 m.
CostmapInfo stockCostmap() { return CostmapInfo{3.0, 3.0, 0.05, std::nullopt}; }

CoreConfig ackermann()
{
  CoreConfig config = tc();
  config.obstacles.obstacle_mode = ObstacleMode::kNone;
  config.motion.motion_model = MotionModel::kAckermann;
  config.motion.min_turning_radius = 0.5;
  return config;
}

std::string describe(const ValidationReport & report) { return report.summary(); }

// Exactly the expected codes, in any order, and nothing else.
void expectOnly(
  const ValidationReport & report, const std::vector<std::string_view> & errors,
  const std::vector<std::string_view> & warnings)
{
  EXPECT_EQ(report.errors.size(), errors.size()) << describe(report);
  for (std::string_view code : errors) {
    EXPECT_TRUE(report.hasError(code)) << "missing error " << code << "\n" << describe(report);
  }
  EXPECT_EQ(report.warnings.size(), warnings.size()) << describe(report);
  for (std::string_view code : warnings) {
    EXPECT_TRUE(report.hasWarning(code)) << "missing warning " << code << "\n" << describe(report);
  }
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Defaults, TC and auto values.

TEST(Config, DefaultsAreThePlanDefaults)
{
  const CoreConfig config;
  EXPECT_DOUBLE_EQ(config.field.desired_speed, 0.4);
  EXPECT_DOUBLE_EQ(config.field.convergence_gain, 2.0);
  EXPECT_EQ(config.field.gain_function, vf::GainType::kAtan);
  EXPECT_EQ(config.field.direction, 1);
  EXPECT_DOUBLE_EQ(config.field.assumption3_margin, 0.1);
  EXPECT_EQ(config.motion.motion_model, MotionModel::kDifferential);
  EXPECT_EQ(config.motion.tracking_law, TrackingLaw::kAuto);
  EXPECT_DOUBLE_EQ(config.motion.control_point_offset, 0.2);
  EXPECT_FALSE(config.motion.drive_backward);
  EXPECT_FALSE(config.motion.allow_reversing);
  EXPECT_DOUBLE_EQ(config.motion.heading_gain, 2.0);
  EXPECT_EQ(config.motion.omni_heading_policy, vf::OmniHeadingPolicy::kPathTangent);
  EXPECT_DOUBLE_EQ(config.motion.max_linear_vel, 0.5);
  EXPECT_DOUBLE_EQ(config.motion.max_lateral_vel, 0.5);
  EXPECT_DOUBLE_EQ(config.motion.max_angular_vel, 1.5);
  EXPECT_DOUBLE_EQ(config.motion.max_linear_accel, 1.0);
  EXPECT_DOUBLE_EQ(config.motion.max_linear_decel, 1.0);
  EXPECT_DOUBLE_EQ(config.motion.max_angular_accel, 3.2);
  EXPECT_DOUBLE_EQ(config.motion.min_in_place_angular_vel, 0.0);
  EXPECT_DOUBLE_EQ(config.motion.min_turning_radius, 0.0);
  EXPECT_DOUBLE_EQ(config.motion.ackermann_min_speed, 0.05);
  EXPECT_DOUBLE_EQ(config.motion.reference_point_offset, 0.0);
  EXPECT_DOUBLE_EQ(config.motion.skid_yaw_rate_efficiency, 1.0);
  EXPECT_DOUBLE_EQ(config.motion.skid_icr_offset, 0.0);
  EXPECT_TRUE(config.phases.use_rotate_to_field);
  EXPECT_DOUBLE_EQ(config.phases.rotate_to_field_angle, 1.57);
  EXPECT_DOUBLE_EQ(config.phases.rotate_to_field_release_angle, 0.785);
  EXPECT_TRUE(config.phases.use_rotate_to_heading);
  EXPECT_DOUBLE_EQ(config.phases.rotate_to_heading_angular_vel, 1.5);
  EXPECT_TRUE(vf::isAuto(config.phases.final_approach_distance));
  EXPECT_DOUBLE_EQ(config.phases.terminal_braking_distance, 0.5);
  EXPECT_DOUBLE_EQ(config.phases.min_approach_speed, 0.05);
  EXPECT_DOUBLE_EQ(config.phases.cancel_deceleration, 1.0);
  EXPECT_EQ(config.obstacles.obstacle_mode, ObstacleMode::kNone);
  EXPECT_EQ(config.obstacles.obstacle_activation, vf::ObstacleActivation::kPathBlocked);
  EXPECT_TRUE(vf::isAuto(config.obstacles.obstacle_contour_distance));
  EXPECT_TRUE(vf::isAuto(config.obstacles.obstacle_full_switch_distance));
  EXPECT_TRUE(vf::isAuto(config.obstacles.obstacle_blend_start_distance));
  EXPECT_TRUE(vf::isAuto(config.obstacles.obstacle_lookahead));
  EXPECT_DOUBLE_EQ(config.obstacles.obstacle_gate_tolerance, 0.05);
  EXPECT_DOUBLE_EQ(config.obstacles.obstacle_release_time, 0.5);
  EXPECT_DOUBLE_EQ(config.obstacles.obstacle_max_contour_length, 5.0);
  EXPECT_DOUBLE_EQ(config.obstacles.obstacle_max_contour_time, 20.0);
  EXPECT_FALSE(config.obstacles.unknown_is_obstacle);
  EXPECT_DOUBLE_EQ(config.obstacles.clearance_slowdown_distance, 0.0);
  EXPECT_DOUBLE_EQ(config.obstacles.clearance_slowdown_gain, 1.0);
  EXPECT_DOUBLE_EQ(config.obstacles.cbf_gain, 2.0);
  EXPECT_DOUBLE_EQ(config.obstacles.cbf_margin, 0.05);
  EXPECT_EQ(config.obstacles.cbf_sites, 3);
  EXPECT_TRUE(config.safety.use_collision_detection);
  EXPECT_DOUBLE_EQ(config.safety.collision_check_horizon, 1.5);
  EXPECT_DOUBLE_EQ(config.safety.stop_margin, 0.10);
  EXPECT_DOUBLE_EQ(config.node.controller_frequency, 20.0);
  EXPECT_TRUE(config.node.footprint.polygon.empty());
  EXPECT_DOUBLE_EQ(config.node.footprint.radius, 0.22);

  const vf::CurveOptions options;
  EXPECT_EQ(options.max_curve_points, 20000);
  EXPECT_DOUBLE_EQ(options.default_sample_spacing, 0.05);
  EXPECT_DOUBLE_EQ(options.search_window, 1.0);
  EXPECT_DOUBLE_EQ(options.global_search_trigger, 0.5);
  EXPECT_DOUBLE_EQ(options.global_search_hysteresis, 0.10);
  EXPECT_EQ(options.global_search_samples, 400);
}

TEST(Config, DefaultsPassWithoutIssues)
{
  for (ObstacleMode mode : {ObstacleMode::kNone, ObstacleMode::kContour, ObstacleMode::kCbf}) {
    CoreConfig config;
    config.obstacles.obstacle_mode = mode;
    SCOPED_TRACE(vf::toString(mode));
    expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
    expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
  }
  expectOnly(vf::validateConfig(tc(), stockCostmap()), {}, {});
}

TEST(Config, Tb3AutoValues)
{
  for (const CoreConfig & config : {CoreConfig{}, tc()}) {
    const CoreConfig resolved = vf::resolveAutoValues(config);
    EXPECT_EQ(resolved.motion.tracking_law, TrackingLaw::kFeedbackLinearization);
    EXPECT_NEAR(resolved.obstacles.obstacle_contour_distance, 0.47, 1e-12);
    EXPECT_NEAR(resolved.obstacles.obstacle_full_switch_distance, 0.62, 1e-12);
    EXPECT_NEAR(resolved.obstacles.obstacle_blend_start_distance, 0.82, 1e-12);
    EXPECT_NEAR(resolved.obstacles.obstacle_lookahead, 1.0, 1e-12);
    EXPECT_TRUE(vf::isAuto(resolved.phases.final_approach_distance));
    EXPECT_NEAR(vf::finalApproachDistance(resolved, 0.25), 0.45, 1e-12);
  }
  // The lateral branch of the auto lambda: sqrt(0.32^2 + 0.2^2) = 0.3774 < 0.47.
  const vf::FootprintRadii tb3{0.22, 0.22, 0.22};
  EXPECT_NEAR(vf::autoContourDistance(tb3, 0.0), 0.32, 1e-12);
  EXPECT_NEAR(std::sqrt(0.32 * 0.32 + 0.04), 0.3774, 1e-4);
}

TEST(Config, AutoValuesPassEveryRule)
{
  // math_spec §11: lambda - d >= r_circ + 0.05 > r_circ >= r_insc for every footprint, offset and
  // law, so no contour rule ever fires on auto values.
  std::vector<vf::FootprintSpec> footprints;
  for (double radius : {0.0, 0.05, 0.22, 0.6}) {
    vf::FootprintSpec circle;
    circle.radius = radius;
    footprints.push_back(circle);
  }
  vf::FootprintSpec rectangle;
  rectangle.polygon = {Vec2(-0.3, -0.25), Vec2(0.45, -0.25), Vec2(0.45, 0.25), Vec2(-0.3, 0.25)};
  footprints.push_back(rectangle);
  vf::FootprintSpec gon;
  gon.polygon = sixteenGon(0.22);
  footprints.push_back(gon);

  struct Pair
  {
    MotionModel model;
    TrackingLaw law;
  };
  const std::vector<Pair> pairs{
    {MotionModel::kDifferential, TrackingLaw::kFeedbackLinearization},
    {MotionModel::kDifferential, TrackingLaw::kHeadingTracking},
    {MotionModel::kSkidSteer, TrackingLaw::kAuto},
    {MotionModel::kOmnidirectional, TrackingLaw::kHolonomic},
  };
  for (const vf::FootprintSpec & footprint : footprints) {
    for (const Pair & pair : pairs) {
      for (double d : {0.05, 0.1, 0.2, 0.3, 0.5}) {
        CoreConfig config = tc();
        config.node.footprint = footprint;
        config.motion.motion_model = pair.model;
        config.motion.tracking_law = pair.law;
        config.motion.control_point_offset = d;
        const ValidationReport report = vf::validateConfig(config, std::nullopt);
        const CoreConfig resolved = vf::resolveAutoValues(config);
        const vf::FootprintRadii radii = vf::computeFootprintRadii(footprint);
        const double lambda = resolved.obstacles.obstacle_contour_distance;
        const double offset = vf::evaluationOffset(resolved);
        SCOPED_TRACE(describe(report));
        EXPECT_GE(lambda - offset, radii.r_circ + 0.05 - 1e-12);
        EXPECT_FALSE(report.hasError(issue::kContourOrder));
        EXPECT_FALSE(report.hasError(issue::kContourBelowInscribed));
        EXPECT_FALSE(report.hasWarning(issue::kContourBelowCircumscribed));
      }
    }
  }
}

TEST(Config, FinalApproachDistance)
{
  CoreConfig config = tc();
  EXPECT_NEAR(vf::finalApproachDistance(config, 0.05), 0.40, 1e-12);  // max(2d, xy_tol + d)
  EXPECT_NEAR(vf::finalApproachDistance(config, 0.25), 0.45, 1e-12);
  config.motion.tracking_law = TrackingLaw::kHeadingTracking;
  EXPECT_NEAR(vf::finalApproachDistance(config, 0.25), 0.25, 1e-12);
  config.phases.final_approach_distance = 0.7;
  EXPECT_NEAR(vf::finalApproachDistance(config, 0.25), 0.7, 1e-12);
}

TEST(Config, EvaluationOffset)
{
  CoreConfig config = tc();
  EXPECT_DOUBLE_EQ(vf::evaluationOffset(config), 0.2);  // FL (auto): d
  config.motion.tracking_law = TrackingLaw::kHeadingTracking;
  EXPECT_DOUBLE_EQ(vf::evaluationOffset(config), 0.0);
  config.motion.motion_model = MotionModel::kOmnidirectional;
  config.motion.tracking_law = TrackingLaw::kAuto;
  EXPECT_DOUBLE_EQ(vf::evaluationOffset(config), 0.0);
  config.motion.motion_model = MotionModel::kAckermann;
  config.motion.reference_point_offset = -0.3;
  EXPECT_DOUBLE_EQ(vf::evaluationOffset(config), 0.3);
}

// ------------------------------------------------------------------------------------------------
// Reject rules of math_spec §11, one test each.

TEST(ConfigReject, DesiredSpeedAboveMaxLinearVel)
{
  CoreConfig config = tc();
  config.field.desired_speed = 0.6;
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kSpeedAboveMax}, {});
  config.field.desired_speed = 0.5;  // equality is allowed
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigReject, ReverseSpeedAboveMax)
{
  // max_reverse_vel (math_spec §9.4 [Design, 0.3.0]): auto = max_linear_vel; set: (0, v_max].
  CoreConfig config = tc();
  EXPECT_DOUBLE_EQ(vf::reverseSpeedLimit(config), config.motion.max_linear_vel);
  EXPECT_DOUBLE_EQ(vf::resolveAutoValues(config).motion.max_reverse_vel, 0.5);
  EXPECT_DOUBLE_EQ(vf::referenceSpeed(config, -1), config.field.desired_speed);
  config.motion.max_reverse_vel = 0.6;
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kReverseAboveMax}, {});
  config.motion.max_reverse_vel = 0.5;  // equality is allowed
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
  config.motion.max_reverse_vel = 0.2;  // below desired_speed: the reverse field runs at 0.2
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
  EXPECT_DOUBLE_EQ(vf::referenceSpeed(config, -1), 0.2);
  EXPECT_DOUBLE_EQ(vf::referenceSpeed(config, 1), config.field.desired_speed);
  EXPECT_DOUBLE_EQ(vf::Kinematics::fromConfig(config).max_reverse_vel, 0.2);
  config.motion.max_reverse_vel = 0.0;
  EXPECT_TRUE(vf::validateConfig(config, stockCostmap()).hasError(issue::kInvalidValue));
  config.motion.max_reverse_vel = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(vf::validateConfig(config, stockCostmap()).hasError(issue::kInvalidValue));

  // Omni has no drive direction; Ackermann needs v_ack <= max_reverse_vel when it is set.
  CoreConfig omni = tc();
  omni.motion.motion_model = vf::MotionModel::kOmnidirectional;
  omni.motion.max_reverse_vel = 0.2;
  EXPECT_DOUBLE_EQ(vf::referenceSpeed(omni, -1), omni.field.desired_speed);
  CoreConfig car = ackermann();
  car.motion.max_reverse_vel = 0.5 * car.motion.ackermann_min_speed;
  EXPECT_TRUE(vf::validateConfig(car, std::nullopt).hasError(issue::kAckermannReverseBelowBand));
  car.motion.max_reverse_vel = car.motion.ackermann_min_speed;
  EXPECT_FALSE(vf::validateConfig(car, std::nullopt).hasError(issue::kAckermannReverseBelowBand));
}

TEST(ConfigReject, InPlaceFloorAboveRotateSpeed)
{
  CoreConfig config = tc();
  config.motion.min_in_place_angular_vel = 1.6;
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kInPlaceAboveRotate}, {});
  config.motion.min_in_place_angular_vel = 1.5;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigReject, ContourDistanceOrder)
{
  CoreConfig config = tc();
  config.obstacles.obstacle_contour_distance = 0.5;
  config.obstacles.obstacle_full_switch_distance = 0.5;  // not lambda < D_in
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kContourOrder}, {});

  config.obstacles.obstacle_full_switch_distance = 0.9;
  config.obstacles.obstacle_blend_start_distance = 0.8;  // not D_in <= D'_in
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kContourOrder}, {});

  config.obstacles.obstacle_blend_start_distance = 0.9;  // D_in = D'_in: the hard switch
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});

  config.obstacles.obstacle_blend_start_distance = 0.8;
  config.obstacles.obstacle_mode = ObstacleMode::kNone;  // contour-only rule
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigReject, ContourDistanceBelowInscribedRadius)
{
  CoreConfig config = tc();
  config.obstacles.obstacle_contour_distance = 0.40;  // 0.40 - 0.2 < r_insc = 0.2158
  expectOnly(
    vf::validateConfig(config, stockCostmap()), {issue::kContourBelowInscribed},
    {issue::kContourBelowCircumscribed});
  config.obstacles.obstacle_mode = ObstacleMode::kNone;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigReject, InvalidModelLawPair)
{
  const struct
  {
    MotionModel model;
    TrackingLaw law;
  } invalid[] = {
    {MotionModel::kDifferential, TrackingLaw::kHolonomic},
    {MotionModel::kSkidSteer, TrackingLaw::kHolonomic},
    {MotionModel::kOmnidirectional, TrackingLaw::kFeedbackLinearization},
    {MotionModel::kOmnidirectional, TrackingLaw::kHeadingTracking},
  };
  for (const auto & pair : invalid) {
    CoreConfig config = tc();
    config.motion.motion_model = pair.model;
    config.motion.tracking_law = pair.law;
    SCOPED_TRACE(std::string(vf::toString(pair.model)) + " + " + vf::toString(pair.law));
    EXPECT_TRUE(vf::validateConfig(config, stockCostmap()).hasError(issue::kInvalidLaw));
    EXPECT_FALSE(vf::isValidCombination(pair.model, pair.law));
  }
  CoreConfig config = ackermann();
  config.motion.tracking_law = TrackingLaw::kFeedbackLinearization;
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kInvalidLaw}, {});
  config.motion.tracking_law = TrackingLaw::kHolonomic;
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kInvalidLaw}, {});
}

TEST(Config, AckermannContourAutoValues)
{
  // math_spec §6.6 [Design]: lambda = max(auto, R_min + w_lat + 0.05), D_in = lambda + R_min,
  // D'_in = D_in + 0.2, lookahead = max(1.0, v_r T_h, D'_in + 0.1). TC footprint: w_lat = 0.22.
  CoreConfig config = ackermann();
  config.obstacles.obstacle_mode = ObstacleMode::kContour;
  const CoreConfig half = vf::resolveAutoValues(config);
  EXPECT_NEAR(half.obstacles.obstacle_contour_distance, 0.77, 1e-12);
  EXPECT_NEAR(half.obstacles.obstacle_full_switch_distance, 1.27, 1e-12);
  EXPECT_NEAR(half.obstacles.obstacle_blend_start_distance, 1.47, 1e-12);
  EXPECT_NEAR(half.obstacles.obstacle_lookahead, 1.57, 1e-12);
  config.motion.min_turning_radius = 1.0;
  const CoreConfig one = vf::resolveAutoValues(config);
  EXPECT_NEAR(one.obstacles.obstacle_contour_distance, 1.27, 1e-12);
  EXPECT_NEAR(one.obstacles.obstacle_full_switch_distance, 2.27, 1e-12);
  EXPECT_NEAR(one.obstacles.obstacle_blend_start_distance, 2.47, 1e-12);
  EXPECT_NEAR(one.obstacles.obstacle_lookahead, 2.57, 1e-12);
  // The footprint branch wins for a long car evaluated at its rear axle: |x_r| = 0.3,
  // r_circ = |(0.45, 0.25)| = 0.5148, so lambda = r_circ + 0.3 + 0.05 > 0.5 + 0.25 + 0.05.
  config.motion.min_turning_radius = 0.5;
  config.motion.reference_point_offset = -0.3;
  config.node.footprint.polygon = {
    Vec2(-0.3, -0.25), Vec2(0.45, -0.25), Vec2(0.45, 0.25), Vec2(-0.3, 0.25)};
  const vf::FootprintRadii radii = vf::computeFootprintRadii(config.node.footprint);
  const double lambda = vf::ackermannAutoContourDistance(radii, 0.3, 0.5);
  EXPECT_NEAR(lambda, std::hypot(0.45, 0.25) + 0.35, 1e-12);
  EXPECT_NEAR(vf::resolveAutoValues(config).obstacles.obstacle_contour_distance, lambda, 1e-12);
  // An explicit lambda keeps the Ackermann turn-in band.
  config.obstacles.obstacle_contour_distance = 0.9;
  const CoreConfig explicit_lambda = vf::resolveAutoValues(config);
  EXPECT_NEAR(explicit_lambda.obstacles.obstacle_full_switch_distance, 1.4, 1e-12);
  EXPECT_NEAR(explicit_lambda.obstacles.obstacle_blend_start_distance, 1.6, 1e-12);
  // Other models keep their auto values.
  EXPECT_NEAR(vf::resolveAutoValues(tc()).obstacles.obstacle_lookahead, 1.0, 1e-12);
}

TEST(ConfigReject, AckermannWithContour)
{
  // Like the shipped ackermann.yaml with contour: valid. The stock 3 m window is too small for the
  // car's blend band (D'_in = 1.47 >= 1.3); a 4 m window fits it.
  CoreConfig config = ackermann();
  config.obstacles.obstacle_mode = ObstacleMode::kContour;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kBlendExceedsWindow});
  expectOnly(vf::validateConfig(config, CostmapInfo{4.0, 4.0, 0.05, std::nullopt}), {}, {});
  config.obstacles.obstacle_mode = ObstacleMode::kNone;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

// math_spec §6.7 [Design, 0.3.0]: cbf is valid for every model, Ackermann included. A car warns
// when a face straight ahead would enter the costmap inside the turning margin
// (x_r + δ_safe + R_min + v_r/γ_b > ½ min(W, H_c)), and when the stop from v_ack at the hold can
// use more than cbf_margin.
TEST(ConfigReject, AckermannWithCbf)
{
  CoreConfig config = ackermann();
  config.obstacles.obstacle_mode = ObstacleMode::kCbf;
  // 0 + (0.22 + 0.05) + 0.5 + 0.4 / 2 = 0.97 m <= 1.5 m.
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
  // R_min = 1.2: 1.67 m > 1.5 m; the warning needs the costmap.
  config.motion.min_turning_radius = 1.2;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kAckermannCbfWindow});
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
  // A 4 m window fits it.
  expectOnly(vf::validateConfig(config, CostmapInfo{4.0, 4.0, 0.05, std::nullopt}), {}, {});
  config = ackermann();
  config.obstacles.obstacle_mode = ObstacleMode::kCbf;
  config.obstacles.cbf_margin = 0.001;  // < 0.05^2 / 2
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kAckermannCbfMargin});
  config.obstacles.obstacle_mode = ObstacleMode::kContour;  // CBF warnings only with cbf
  expectOnly(vf::validateConfig(config, CostmapInfo{4.0, 4.0, 0.05, std::nullopt}), {}, {});
  CoreConfig diff = tc();
  diff.obstacles.obstacle_mode = ObstacleMode::kCbf;
  diff.obstacles.cbf_margin = 0.001;
  expectOnly(vf::validateConfig(diff, stockCostmap()), {}, {});
}

TEST(ConfigReject, AckermannContourBelowTurningRadius)
{
  // lambda < R_min + 0.05 is an error; up to R_min + w_lat + 0.05 a warning.
  CoreConfig config = ackermann();
  config.obstacles.obstacle_mode = ObstacleMode::kContour;
  for (const double lambda : {0.3, 0.5, 0.549}) {
    SCOPED_TRACE("lambda " + std::to_string(lambda));
    config.obstacles.obstacle_contour_distance = lambda;
    expectOnly(
      vf::validateConfig(config, std::nullopt), {issue::kAckermannContourRadius},
      {issue::kAckermannContourCurvature});
  }
  for (const double lambda : {0.55, 0.769}) {
    SCOPED_TRACE("lambda " + std::to_string(lambda));
    config.obstacles.obstacle_contour_distance = lambda;
    expectOnly(vf::validateConfig(config, std::nullopt), {}, {issue::kAckermannContourCurvature});
  }
  config.obstacles.obstacle_contour_distance = 0.77;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
  // Not for the other models, nor without contour.
  config.obstacles.obstacle_mode = ObstacleMode::kNone;
  config.obstacles.obstacle_contour_distance = 0.3;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
  CoreConfig diff = tc();
  diff.motion.min_turning_radius = 0.5;
  diff.obstacles.obstacle_contour_distance = 0.5;
  expectOnly(vf::validateConfig(diff, std::nullopt), {}, {});
}

TEST(ConfigReject, AckermannWithoutTurningRadius)
{
  CoreConfig config = ackermann();
  config.motion.min_turning_radius = 0.0;
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kAckermannRadius}, {});
  config.motion.min_turning_radius = -1.0;
  expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kAckermannRadius}, {});
  config.motion.motion_model = MotionModel::kDifferential;  // only Ackermann needs R_min
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigReject, BrakingDistanceExceedsWindow)
{
  CoreConfig config = tc();
  config.obstacles.obstacle_mode = ObstacleMode::kNone;
  // sigma_b + d + 0.1 = 0.8 > 1.5/2.
  const CostmapInfo small{1.5, 1.5, 0.05, std::nullopt};
  EXPECT_TRUE(vf::validateConfig(config, small).hasError(issue::kBrakingExceedsWindow));
  // ... or > window_limit (Gen A/B prune_distance).
  CostmapInfo pruned = stockCostmap();
  pruned.window_limit = 0.7;
  expectOnly(vf::validateConfig(config, pruned), {issue::kBrakingExceedsWindow}, {});
  pruned.window_limit = 2.0;
  expectOnly(vf::validateConfig(config, pruned), {}, {});
  // Costmap rules need a costmap.
  EXPECT_FALSE(vf::validateConfig(config, std::nullopt).hasError(issue::kBrakingExceedsWindow));
}

TEST(ConfigReject, FeedbackLinearizationNeedsPositiveOffset)
{
  for (double d : {0.0, -0.1}) {
    CoreConfig config = tc();
    config.obstacles.obstacle_mode = ObstacleMode::kNone;
    config.motion.control_point_offset = d;
    expectOnly(vf::validateConfig(config, stockCostmap()), {issue::kFlOffset}, {});
    config.motion.tracking_law = TrackingLaw::kHeadingTracking;  // HT does not use d
    expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
  }
}

// ------------------------------------------------------------------------------------------------
// Warn rules of math_spec §11, one test each.

TEST(ConfigWarn, RotateSpeedAboveMaxAngularVel)
{
  CoreConfig config = tc();
  config.phases.rotate_to_heading_angular_vel = 1.8;  // the rotation shim's default
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kRotateAboveMax});
}

TEST(ConfigWarn, CbfGain)
{
  CoreConfig config = tc();
  config.obstacles.obstacle_mode = ObstacleMode::kCbf;
  config.obstacles.cbf_gain = 0.5 * config.motion.max_linear_decel / config.field.desired_speed;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
  config.obstacles.cbf_gain = 1.01 * config.motion.max_linear_decel / config.field.desired_speed;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {issue::kCbfGainDeceleration});
  config.obstacles.cbf_gain = config.node.controller_frequency;
  expectOnly(
    vf::validateConfig(config, std::nullopt), {},
    {issue::kCbfGainDeceleration, issue::kCbfGainDiscrete});
  config.obstacles.obstacle_mode = ObstacleMode::kNone;  // the rules apply to cbf only
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
}

TEST(ConfigWarn, DiscreteOvershoot)
{
  CoreConfig config = tc();
  // G'(0) v_r dt = (4/pi) 0.4 / f >= 1 for f <= 0.509 Hz.
  config.node.controller_frequency = 0.5;
  const ValidationReport report = vf::validateConfig(config, std::nullopt);
  EXPECT_TRUE(report.ok()) << describe(report);
  EXPECT_TRUE(report.hasWarning(issue::kDiscreteOvershoot)) << describe(report);
  config.node.controller_frequency = 0.6;
  EXPECT_FALSE(vf::validateConfig(config, std::nullopt).hasWarning(issue::kDiscreteOvershoot));
}

TEST(ConfigWarn, FeedbackLinearizationGainOffset)
{
  CoreConfig config = tc();
  config.motion.control_point_offset = 0.4;  // G'(0) d = 0.509 > 0.5
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {issue::kFlGainOffset});
  config.motion.tracking_law = TrackingLaw::kHeadingTracking;  // FL only
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
}

TEST(ConfigWarn, TerminalBrakingTooShort)
{
  CoreConfig config = tc();
  config.phases.terminal_braking_distance = 0.15;  // < v_r^2 / a_dec = 0.16
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kBrakingTooShort});
  config.phases.terminal_braking_distance = 0.17;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigWarn, ContourDistanceBelowCircumscribedRadius)
{
  CoreConfig config = tc();
  config.obstacles.obstacle_contour_distance = 0.43;  // lambda - d = 0.23 >= r_circ = 0.22
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
  config.obstacles.obstacle_contour_distance = 0.418;  // 0.218: r_insc <= lambda - d < r_circ
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kContourBelowCircumscribed});
}

TEST(ConfigWarn, BlendZoneExceedsWindow)
{
  // D'_in + d = 1.02 >= 2.4/2 - 0.2 = 1.0.
  const CostmapInfo costmap{2.4, 2.4, 0.05, std::nullopt};
  expectOnly(vf::validateConfig(tc(), costmap), {}, {issue::kBlendExceedsWindow});
  CoreConfig no_contour = tc();
  no_contour.obstacles.obstacle_mode = ObstacleMode::kNone;
  expectOnly(vf::validateConfig(no_contour, costmap), {}, {});
}

TEST(ConfigWarn, HorizonTooShort)
{
  CoreConfig config = tc();
  // 0.5 * 0.5 = 0.25 < 0.5^2/2 + 0.5 (2/20 + 0.1) + 0.1 = 0.325.
  config.safety.collision_check_horizon = 0.5;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kHorizonTooShort});
  config.safety.collision_check_horizon = 0.7;  // 0.35 > 0.325
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigWarn, HorizonExceedsWindow)
{
  CoreConfig config = tc();
  // 3.0 * 0.5 = 1.5 > 3/2 - 0.22.
  config.safety.collision_check_horizon = 3.0;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kHorizonExceedsWindow});
  EXPECT_FALSE(vf::validateConfig(config, std::nullopt).hasWarning(issue::kHorizonExceedsWindow));
}

TEST(ConfigWarn, AckermannCurvature)
{
  CoreConfig config = ackermann();
  config.motion.min_turning_radius = 3.0;  // 3.0 * c_G k_f = 1.11 > 1 (kappa_path = 0)
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kAckermannCurvature});
  config.motion.min_turning_radius = 2.5;  // 0.93
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {});
}

TEST(ConfigWarn, AckermannContour)
{
  CoreConfig config = ackermann();
  config.obstacles.obstacle_mode = ObstacleMode::kContour;
  // Turn-in band shorter than R_min (the generic auto D_in = lambda + 0.15).
  config.obstacles.obstacle_full_switch_distance = 0.92;
  config.obstacles.obstacle_blend_start_distance = 1.12;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {issue::kAckermannContourTurnIn});
  config.obstacles.obstacle_full_switch_distance = vf::kAutoValue;
  config.obstacles.obstacle_blend_start_distance = vf::kAutoValue;
  // Gate lookahead shorter than D'_in = 1.47 (path_blocked only).
  config.obstacles.obstacle_lookahead = 1.0;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {issue::kAckermannContourLookahead});
  config.obstacles.obstacle_activation = vf::ObstacleActivation::kAlways;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
  config.obstacles.obstacle_activation = vf::ObstacleActivation::kPathBlocked;
  config.obstacles.obstacle_lookahead = vf::kAutoValue;
  // Watchdog shorter than D'_in - lambda + pi lambda = 0.7 + 0.77 pi = 3.119.
  config.obstacles.obstacle_max_contour_length = 3.0;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {issue::kAckermannContourWatchdog});
  config.obstacles.obstacle_max_contour_length = 3.2;
  expectOnly(vf::validateConfig(config, std::nullopt), {}, {});
}

TEST(ConfigWarn, YawRateForIntegralCurves)
{
  CoreConfig config = tc();
  config.motion.max_angular_vel = 0.1;  // v_r c_G k_f = 0.148 > 0.1
  config.phases.rotate_to_heading_angular_vel = 0.1;
  expectOnly(vf::validateConfig(config, stockCostmap()), {}, {issue::kYawRateForCurvature});
}

// ------------------------------------------------------------------------------------------------
// Per-plan advisor, domain checks and helpers.

TEST(Config, AdviseForPlanUsesWindowCurvature)
{
  const CoreConfig config = tc();
  EXPECT_TRUE(vf::adviseForPlan(config, 0.0).warnings.empty());
  // kappa_path d = 1.2 > 1; v_r (c_G k_f + kappa) = 2.55 > 1.5.
  const ValidationReport sharp = vf::adviseForPlan(config, 6.0);
  EXPECT_TRUE(sharp.ok());
  EXPECT_TRUE(sharp.hasWarning(issue::kFlPathCurvature)) << describe(sharp);
  EXPECT_TRUE(sharp.hasWarning(issue::kYawRateForCurvature)) << describe(sharp);

  CoreConfig car = ackermann();
  // 0.5 (0.371 + 2) = 1.19 > 1.
  EXPECT_TRUE(vf::adviseForPlan(car, 2.0).hasWarning(issue::kAckermannCurvature));
  EXPECT_FALSE(vf::adviseForPlan(car, 1.0).hasWarning(issue::kAckermannCurvature));
  car.field.convergence_gain = -1.0;  // invalid: no advice, validateConfig reports it
  EXPECT_TRUE(vf::adviseForPlan(car, 2.0).warnings.empty());
}

TEST(Config, DomainErrorsSuppressTheRules)
{
  CoreConfig config = tc();
  config.field.desired_speed = vf::kNaN;
  config.obstacles.obstacle_contour_distance = 0.4;  // would break the contour rules
  const ValidationReport report = vf::validateConfig(config, stockCostmap());
  expectOnly(report, {issue::kInvalidValue}, {});
  EXPECT_NE(report.errors.front().message.find("desired_speed"), std::string::npos);

  const auto rejects = [](auto mutate) {
    CoreConfig c = tc();
    mutate(&c);
    return vf::validateConfig(c, std::nullopt).hasError(issue::kInvalidValue);
  };
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->field.desired_speed = 0.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->field.convergence_gain = -2.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->field.direction = 0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->field.assumption3_margin = 1.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->motion.max_angular_vel = 0.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->motion.max_linear_decel = vf::kInf; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->phases.rotate_to_field_release_angle = 1.6; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->phases.terminal_braking_distance = 0.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->phases.final_approach_distance = vf::kNaN; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->obstacles.obstacle_contour_distance = 0.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->obstacles.obstacle_lookahead = vf::kNaN; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->obstacles.cbf_gain = 0.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->obstacles.cbf_margin = -0.01; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->obstacles.cbf_sites = 0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->obstacles.cbf_sites = 4; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->safety.collision_check_horizon = 0.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->node.controller_frequency = 0.0; }));
  EXPECT_FALSE(rejects([](CoreConfig * c) { c->phases.final_approach_distance = 0.0; }));
  EXPECT_FALSE(rejects([](CoreConfig * c) { c->obstacles.obstacle_lookahead = -5.0; }));  // auto
  // Skid slip compensation: eta in (0, 1]; |x_ICR| < d / 2 on skid steer only.
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->motion.skid_yaw_rate_efficiency = 0.0; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->motion.skid_yaw_rate_efficiency = 1.2; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->motion.skid_yaw_rate_efficiency = vf::kNaN; }));
  EXPECT_FALSE(rejects([](CoreConfig * c) { c->motion.skid_yaw_rate_efficiency = 1.0; }));
  EXPECT_FALSE(rejects([](CoreConfig * c) { c->motion.skid_yaw_rate_efficiency = 0.75; }));
  EXPECT_TRUE(rejects([](CoreConfig * c) { c->motion.skid_icr_offset = vf::kInf; }));
  EXPECT_FALSE(rejects([](CoreConfig * c) { c->motion.skid_icr_offset = 0.15; }));  // diff
  EXPECT_TRUE(rejects([](CoreConfig * c) {
    c->motion.motion_model = MotionModel::kSkidSteer;
    c->motion.skid_icr_offset = -0.1;  // d / 2 = 0.1
  }));
  EXPECT_FALSE(rejects([](CoreConfig * c) {
    c->motion.motion_model = MotionModel::kSkidSteer;
    c->motion.skid_icr_offset = 0.05;
  }));

  CoreConfig bad_footprint = tc();
  bad_footprint.node.footprint.polygon = {Vec2(0.0, 0.0), Vec2(1.0, 0.0), Vec2(2.0, 0.0)};
  expectOnly(vf::validateConfig(bad_footprint, std::nullopt), {issue::kInvalidFootprint}, {});
}

TEST(Config, InvalidCostmapSkipsOnlyCostmapRules)
{
  CoreConfig config = tc();
  config.field.desired_speed = 0.6;
  const CostmapInfo broken{0.0, 3.0, 0.05, std::nullopt};
  expectOnly(
    vf::validateConfig(config, broken), {issue::kInvalidCostmap, issue::kSpeedAboveMax}, {});
  CostmapInfo bad_limit = stockCostmap();
  bad_limit.window_limit = 0.0;
  EXPECT_TRUE(vf::validateConfig(tc(), bad_limit).hasError(issue::kInvalidCostmap));
}

TEST(Config, FootprintRadii)
{
  vf::FootprintSpec gon;
  gon.polygon = sixteenGon(0.22);
  const vf::FootprintRadii tc_radii = vf::computeFootprintRadii(gon);
  EXPECT_NEAR(tc_radii.w_lat, 0.22, 1e-15);
  EXPECT_NEAR(tc_radii.r_circ, 0.22, 1e-15);
  EXPECT_NEAR(tc_radii.r_insc, 0.22 * std::cos(vf::kPi / 16.0), 1e-12);
  EXPECT_NEAR(tc_radii.r_insc, 0.2158, 1e-4);

  vf::FootprintSpec rectangle;  // base_link off-centre, clockwise
  rectangle.polygon = {Vec2(-0.3, 0.15), Vec2(0.2, 0.15), Vec2(0.2, -0.15), Vec2(-0.3, -0.15)};
  const vf::FootprintRadii radii = vf::computeFootprintRadii(rectangle);
  EXPECT_NEAR(radii.w_lat, 0.15, 1e-15);
  EXPECT_NEAR(radii.r_circ, std::hypot(0.3, 0.15), 1e-15);
  EXPECT_NEAR(radii.r_insc, 0.15, 1e-15);

  vf::FootprintSpec outside;  // base_link outside the polygon
  outside.polygon = {Vec2(0.1, -0.1), Vec2(0.5, -0.1), Vec2(0.5, 0.1), Vec2(0.1, 0.1)};
  EXPECT_DOUBLE_EQ(vf::computeFootprintRadii(outside).r_insc, 0.0);

  vf::FootprintSpec circle;
  circle.radius = 0.3;
  const vf::FootprintRadii disc = vf::computeFootprintRadii(circle);
  EXPECT_DOUBLE_EQ(disc.w_lat, 0.3);
  EXPECT_DOUBLE_EQ(disc.r_insc, 0.3);
  EXPECT_DOUBLE_EQ(disc.r_circ, 0.3);
}

TEST(Config, FootprintValidity)
{
  std::string reason;
  vf::FootprintSpec point;
  point.radius = 0.0;
  EXPECT_TRUE(vf::footprintIsValid(point));
  vf::FootprintSpec negative;
  negative.radius = -0.1;
  EXPECT_FALSE(vf::footprintIsValid(negative, &reason));
  EXPECT_FALSE(reason.empty());
  vf::FootprintSpec two;
  two.polygon = {Vec2(0.0, 0.0), Vec2(1.0, 0.0)};
  EXPECT_FALSE(vf::footprintIsValid(two));
  vf::FootprintSpec nan;
  nan.polygon = {Vec2(0.0, 0.0), Vec2(1.0, vf::kNaN), Vec2(0.0, 1.0)};
  EXPECT_FALSE(vf::footprintIsValid(nan));
  vf::FootprintSpec gon;
  gon.polygon = sixteenGon(0.22);
  EXPECT_TRUE(vf::footprintIsValid(gon));
}

TEST(Config, LawResolution)
{
  EXPECT_EQ(
    vf::resolveTrackingLaw(MotionModel::kDifferential, TrackingLaw::kAuto),
    TrackingLaw::kFeedbackLinearization);
  EXPECT_EQ(
    vf::resolveTrackingLaw(MotionModel::kSkidSteer, TrackingLaw::kAuto),
    TrackingLaw::kFeedbackLinearization);
  EXPECT_EQ(
    vf::resolveTrackingLaw(MotionModel::kOmnidirectional, TrackingLaw::kAuto),
    TrackingLaw::kHolonomic);
  EXPECT_EQ(
    vf::resolveTrackingLaw(MotionModel::kAckermann, TrackingLaw::kAuto),
    TrackingLaw::kHeadingTracking);
  EXPECT_EQ(
    vf::resolveTrackingLaw(MotionModel::kAckermann, TrackingLaw::kHolonomic),
    TrackingLaw::kHolonomic);
  EXPECT_TRUE(vf::isValidCombination(MotionModel::kSkidSteer, TrackingLaw::kHeadingTracking));
  EXPECT_TRUE(vf::isValidCombination(MotionModel::kAckermann, TrackingLaw::kAuto));
  EXPECT_FALSE(
    vf::isValidCombination(MotionModel::kAckermann, TrackingLaw::kFeedbackLinearization));
}

TEST(Config, EnumStringsRoundTrip)
{
  for (auto value : {vf::GainType::kAtan, vf::GainType::kRational}) {
    vf::GainType parsed{};
    ASSERT_TRUE(vf::fromString(vf::toString(value), &parsed));
    EXPECT_EQ(parsed, value);
  }
  for (auto value :
       {MotionModel::kDifferential, MotionModel::kSkidSteer, MotionModel::kOmnidirectional,
        MotionModel::kAckermann}) {
    MotionModel parsed{};
    ASSERT_TRUE(vf::fromString(vf::toString(value), &parsed));
    EXPECT_EQ(parsed, value);
  }
  for (auto value :
       {TrackingLaw::kAuto, TrackingLaw::kFeedbackLinearization, TrackingLaw::kHeadingTracking,
        TrackingLaw::kHolonomic}) {
    TrackingLaw parsed{};
    ASSERT_TRUE(vf::fromString(vf::toString(value), &parsed));
    EXPECT_EQ(parsed, value);
  }
  for (auto value :
       {vf::OmniHeadingPolicy::kPathTangent, vf::OmniHeadingPolicy::kField,
        vf::OmniHeadingPolicy::kFixed}) {
    vf::OmniHeadingPolicy parsed{};
    ASSERT_TRUE(vf::fromString(vf::toString(value), &parsed));
    EXPECT_EQ(parsed, value);
  }
  for (auto value : {ObstacleMode::kNone, ObstacleMode::kContour, ObstacleMode::kCbf}) {
    ObstacleMode parsed{};
    ASSERT_TRUE(vf::fromString(vf::toString(value), &parsed));
    EXPECT_EQ(parsed, value);
  }
  for (auto value : {vf::ObstacleActivation::kPathBlocked, vf::ObstacleActivation::kAlways}) {
    vf::ObstacleActivation parsed{};
    ASSERT_TRUE(vf::fromString(vf::toString(value), &parsed));
    EXPECT_EQ(parsed, value);
  }
  EXPECT_STREQ(vf::toString(MotionModel::kSkidSteer), "skid_steer");
  EXPECT_STREQ(vf::toString(TrackingLaw::kFeedbackLinearization), "feedback_linearization");
  MotionModel unchanged = MotionModel::kAckermann;
  EXPECT_FALSE(vf::fromString("Differential", &unchanged));
  EXPECT_EQ(unchanged, MotionModel::kAckermann);
}

TEST(Config, ReportHelpers)
{
  CoreConfig config = tc();
  config.field.desired_speed = 0.6;
  config.phases.rotate_to_heading_angular_vel = 1.8;
  const ValidationReport report = vf::validateConfig(config, std::nullopt);
  EXPECT_FALSE(report.ok());
  const std::string summary = report.summary();
  EXPECT_NE(summary.find(std::string(issue::kSpeedAboveMax)), std::string::npos);
  EXPECT_NE(summary.find(std::string(issue::kRotateAboveMax)), std::string::npos);
  EXPECT_THROW(report.throwOnError(), vf::ConfigError);
  EXPECT_NO_THROW(vf::validateConfig(tc(), std::nullopt).throwOnError());
}

TEST(Config, CurveOptionsDomain)
{
  EXPECT_TRUE(vf::validateCurveOptions(vf::CurveOptions{}).ok());
  const auto rejects = [](auto mutate) {
    vf::CurveOptions options;
    mutate(&options);
    return vf::validateCurveOptions(options).hasError(issue::kInvalidCurveOptions);
  };
  EXPECT_TRUE(rejects([](vf::CurveOptions * o) { o->max_curve_points = 1; }));
  EXPECT_TRUE(rejects([](vf::CurveOptions * o) { o->default_sample_spacing = 0.0; }));
  EXPECT_TRUE(rejects([](vf::CurveOptions * o) { o->search_window = vf::kNaN; }));
  EXPECT_TRUE(rejects([](vf::CurveOptions * o) { o->global_search_trigger = -0.1; }));
  EXPECT_TRUE(rejects([](vf::CurveOptions * o) { o->global_search_hysteresis = vf::kInf; }));
  EXPECT_TRUE(rejects([](vf::CurveOptions * o) { o->global_search_samples = 3; }));
  EXPECT_FALSE(rejects([](vf::CurveOptions * o) { o->global_search_samples = 4; }));
}
