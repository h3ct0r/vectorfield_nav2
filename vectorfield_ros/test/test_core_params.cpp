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

// Node-level parameter handling:
// setNodeDerivedFields(), controllerFrequencyOf(), validateParams(), validateFromNode() and the
// CrossValidator.

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_ros/core_params.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace vr = vectorfield_ros;

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

const ::testing::Environment * const kRclcpp =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

const std::string kPlugin = "FollowPath";

// nav2_bringup's TB3: robot_radius 0.22, as Costmap2DROS::getRobotFootprint() reports it.
std::vector<geometry_msgs::msg::Point> tb3Footprint()
{
  return nav2_costmap_2d::makeFootprintFromRadius(0.22);
}

// The stock local costmap: 3 x 3 m at 0.05 m.
vf::CostmapInfo stockCostmap()
{
  vf::CostmapInfo costmap;
  costmap.width_m = 3.0;
  costmap.height_m = 3.0;
  costmap.resolution = 0.05;
  return costmap;
}

rclcpp::Node::SharedPtr makeNode(
  const std::string & name, const std::vector<rclcpp::Parameter> & overrides = {})
{
  return std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().parameter_overrides(overrides));
}

// ---------------------------------------------------------------------------------------------
// Node-derived fields and controller_frequency

TEST(CoreParams, SetNodeDerivedFields)
{
  vf::CoreConfig config;
  vr::setNodeDerivedFields(10.0, tb3Footprint(), &config);
  EXPECT_EQ(config.node.controller_frequency, 10.0);
  ASSERT_EQ(config.node.footprint.polygon.size(), 16U);
  const vf::FootprintRadii radii = vf::computeFootprintRadii(config.node.footprint);
  EXPECT_NEAR(radii.r_circ, 0.22, 1e-9);
  EXPECT_NEAR(radii.w_lat, 0.22, 1e-9);

  vr::setNodeDerivedFields(25.0, {}, &config);
  EXPECT_EQ(config.node.controller_frequency, 25.0);
  EXPECT_TRUE(config.node.footprint.polygon.empty());
  EXPECT_EQ(config.node.footprint.radius, 0.0);
}

TEST(CoreParams, ControllerFrequencyOf)
{
  // Neither declared nor overridden: the fallback.
  auto plain = makeNode("vf_freq_plain");
  EXPECT_EQ(vr::controllerFrequencyOf(*plain->get_node_parameters_interface()), 20.0);
  EXPECT_EQ(vr::controllerFrequencyOf(*plain->get_node_parameters_interface(), 7.0), 7.0);

  // An override (params file) before declaration, double or integer.
  auto overridden = makeNode("vf_freq_override", {rclcpp::Parameter("controller_frequency", 12.5)});
  EXPECT_EQ(vr::controllerFrequencyOf(*overridden->get_node_parameters_interface()), 12.5);
  auto integer = makeNode("vf_freq_integer", {rclcpp::Parameter("controller_frequency", 30)});
  EXPECT_EQ(vr::controllerFrequencyOf(*integer->get_node_parameters_interface()), 30.0);

  // The declared value wins over the override.
  overridden->declare_parameter("controller_frequency", 20.0);
  overridden->set_parameter(rclcpp::Parameter("controller_frequency", 15.0));
  EXPECT_EQ(vr::controllerFrequencyOf(*overridden->get_node_parameters_interface()), 15.0);

  // Non-numeric: the fallback.
  auto text = makeNode("vf_freq_text", {rclcpp::Parameter("controller_frequency", "fast")});
  EXPECT_EQ(vr::controllerFrequencyOf(*text->get_node_parameters_interface(), 9.0), 9.0);
}

// ---------------------------------------------------------------------------------------------
// validateParams() and validateFromNode()

TEST(CoreParams, ValidateParamsReportsUnusableRows)
{
  vr::CommonParams params;
  params.motion_model = "hovercraft";
  vf::CoreConfig config;
  const vf::ValidationReport report =
    vr::validateParams(params, 20.0, stockCostmap(), tb3Footprint(), &config);
  ASSERT_EQ(report.errors.size(), 1U) << report.summary();
  EXPECT_EQ(report.errors[0].code, vf::issue::kInvalidValue);
  EXPECT_NE(report.errors[0].message.find("motion_model"), std::string::npos);
  // The configuration is still filled (the failing row keeps its default).
  EXPECT_EQ(config.node.footprint.polygon.size(), 16U);
}

TEST(CoreParams, ValidateParamsAppliesTheNodeDerivedFields)
{
  vf::CoreConfig config;
  const vf::ValidationReport report =
    vr::validateParams(vr::CommonParams(), 0.0, std::nullopt, tb3Footprint(), &config);
  EXPECT_TRUE(report.hasError(vf::issue::kInvalidValue)) << report.summary();
  EXPECT_EQ(config.node.controller_frequency, 0.0);

  const vf::ValidationReport segment = vr::validateParams(
    vr::CommonParams(), 20.0, std::nullopt, {tb3Footprint()[0], tb3Footprint()[4]});
  EXPECT_TRUE(segment.hasError(vf::issue::kInvalidFootprint)) << segment.summary();
}

// §5.4 "Tests", plugin layer 2: the gpl defaults with the TB3 footprint and the stock costmap
// give no errors and no warnings in every obstacle mode.
TEST(CoreParams, ValidateFromNodeDefaultsAreClean)
{
  for (const std::string & mode : std::vector<std::string>{"none", "contour", "cbf"}) {
    auto node = makeNode(
      "vf_validate_defaults_" + mode, {rclcpp::Parameter(kPlugin + ".obstacle_mode", mode)});
    vf::CoreConfig config;
    const vf::ValidationReport report =
      vr::validateFromNode(node, kPlugin, stockCostmap(), tb3Footprint(), &config);
    EXPECT_TRUE(report.errors.empty()) << mode << ":\n" << report.summary();
    EXPECT_TRUE(report.warnings.empty()) << mode << ":\n" << report.summary();
    EXPECT_STREQ(vf::toString(config.obstacles.obstacle_mode), mode.c_str());
    EXPECT_EQ(config.node.controller_frequency, 20.0);
    EXPECT_EQ(config.node.footprint.polygon.size(), 16U);
    // The parameters stay declared; the temporary listener's callback is gone.
    EXPECT_TRUE(node->has_parameter(kPlugin + ".desired_speed"));
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(kPlugin + ".desired_speed", 0.3)).successful);
  }
}

TEST(CoreParams, ValidateFromNodeUsesOverridesAndControllerFrequency)
{
  auto node = makeNode(
    "vf_validate_overrides", {
                               rclcpp::Parameter(kPlugin + ".desired_speed", 0.6),
                               rclcpp::Parameter(kPlugin + ".max_linear_vel", 0.5),
                               rclcpp::Parameter("controller_frequency", 10.0),
                             });
  vf::CoreConfig config;
  const vf::ValidationReport report =
    vr::validateFromNode(node, kPlugin, stockCostmap(), tb3Footprint(), &config);
  EXPECT_TRUE(report.hasError(vf::issue::kSpeedAboveMax)) << report.summary();
  EXPECT_EQ(config.field.desired_speed, 0.6);
  EXPECT_EQ(config.node.controller_frequency, 10.0);
}

TEST(CoreParams, ValidateFromNodeFollowerPrefix)
{
  auto node = makeNode("vf_validate_follower");
  const vf::ValidationReport report =
    vr::validateFromNode(node, "", stockCostmap(), tb3Footprint());
  EXPECT_TRUE(report.errors.empty()) << report.summary();
  EXPECT_TRUE(node->has_parameter("desired_speed"));
}

// ---------------------------------------------------------------------------------------------
// CrossValidator

class CrossValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = makeNode("vf_cross_validator");
    listener_ = std::make_shared<vr::CommonParamListener>(node_, kPlugin);
    std::vector<vr::ParamSetter> setters;
    vr::appendSetters(vr::kCommonCoreParams, &setters);
    validator_ = makeValidator(std::move(setters));
  }

  std::unique_ptr<vr::CrossValidator> makeValidator(
    std::vector<vr::ParamSetter> setters, vr::CrossValidator::ExtraCheck extra = nullptr)
  {
    const std::shared_ptr<vr::CommonParamListener> listener = listener_;
    return std::make_unique<vr::CrossValidator>(
      node_->get_node_parameters_interface(), kPlugin, std::move(setters), stockCostmap(),
      [listener]() {
        vr::CoreParamTarget target;
        target.config = vr::toCoreConfig(listener->get_params());
        vr::setNodeDerivedFields(20.0, tb3Footprint(), &target.config);
        return target;
      },
      node_->get_logger(), std::move(extra));
  }

  double committed(const std::string & key) const
  {
    return node_->get_parameter(kPlugin + "." + key).as_double();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<vr::CommonParamListener> listener_;
  std::unique_ptr<vr::CrossValidator> validator_;
};

TEST_F(CrossValidatorTest, PrefixGetsATrailingDot)
{
  EXPECT_EQ(validator_->prefix(), kPlugin + ".");
}

TEST_F(CrossValidatorTest, RejectsAnInvalidChange)
{
  const double max_linear_vel = committed("max_linear_vel");
  const rcl_interfaces::msg::SetParametersResult result =
    validator_->validate({rclcpp::Parameter(kPlugin + ".desired_speed", max_linear_vel + 0.1)});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find(kPlugin + ".desired_speed"), std::string::npos) << result.reason;
  EXPECT_NE(result.reason.find(vf::issue::kSpeedAboveMax), std::string::npos) << result.reason;

  // Registered on the node: the change is refused and the committed value is unchanged.
  const double before = committed("desired_speed");
  EXPECT_FALSE(
    node_->set_parameter(rclcpp::Parameter(kPlugin + ".desired_speed", max_linear_vel + 0.1))
      .successful);
  EXPECT_EQ(committed("desired_speed"), before);
  EXPECT_EQ(listener_->get_params().desired_speed, before);
}

TEST_F(CrossValidatorTest, AcceptsAValidChange)
{
  const double speed = 0.5 * committed("max_linear_vel");
  EXPECT_TRUE(
    validator_->validate({rclcpp::Parameter(kPlugin + ".desired_speed", speed)}).successful);
  EXPECT_TRUE(
    node_->set_parameter(rclcpp::Parameter(kPlugin + ".desired_speed", speed)).successful);
  EXPECT_EQ(committed("desired_speed"), speed);
  EXPECT_EQ(listener_->get_params().desired_speed, speed);

  // A batch is judged as a whole: raising both together is valid.
  const double faster = committed("max_linear_vel") + 0.1;
  const rcl_interfaces::msg::SetParametersResult batch = node_->set_parameters_atomically({
    rclcpp::Parameter(kPlugin + ".max_linear_vel", faster + 0.1),
    rclcpp::Parameter(kPlugin + ".desired_speed", faster),
  });
  EXPECT_TRUE(batch.successful) << batch.reason;
  EXPECT_EQ(committed("desired_speed"), faster);
}

TEST_F(CrossValidatorTest, IgnoresForeignAndNonCoreNames)
{
  const double too_fast = committed("max_linear_vel") + 1.0;
  for (const std::string & name : std::vector<std::string>{
         "Other.desired_speed", "FollowPathX.desired_speed", "desired_speed"}) {
    EXPECT_TRUE(validator_->validate({rclcpp::Parameter(name, too_fast)}).successful) << name;
  }
  EXPECT_TRUE(
    validator_->validate({rclcpp::Parameter(kPlugin + ".plugin", "vectorfield_nav2_controller")})
      .successful);
  EXPECT_TRUE(validator_->validate({rclcpp::Parameter(kPlugin + ".visualize", false)}).successful);
  EXPECT_TRUE(validator_->validate({}).successful);

  // A foreign name in a batch with one of ours does not hide ours.
  EXPECT_FALSE(validator_
                 ->validate({
                   rclcpp::Parameter(kPlugin + ".plugin", "x"),
                   rclcpp::Parameter(kPlugin + ".desired_speed", too_fast),
                 })
                 .successful);
}

TEST_F(CrossValidatorTest, RejectsUnusableValuesByName)
{
  const rcl_interfaces::msg::SetParametersResult result =
    validator_->validate({rclcpp::Parameter(kPlugin + ".obstacle_mode", "sometimes")});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find(kPlugin + ".obstacle_mode"), std::string::npos) << result.reason;
}

TEST_F(CrossValidatorTest, ExtraChecksCanReject)
{
  std::vector<vr::ParamSetter> setters;
  vr::appendSetters(vr::kCommonCoreParams, &setters);
  validator_ = makeValidator(
    std::move(setters), [](const vr::CoreParamTarget & candidate, vf::ValidationReport * report) {
      if (candidate.config.field.desired_speed > 0.35) {
        report->errors.push_back(vf::ValidationIssue{"extra", "desired_speed above 0.35"});
      }
    });
  EXPECT_TRUE(
    validator_->validate({rclcpp::Parameter(kPlugin + ".desired_speed", 0.3)}).successful);
  const rcl_interfaces::msg::SetParametersResult result =
    validator_->validate({rclcpp::Parameter(kPlugin + ".desired_speed", 0.4)});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("[extra]"), std::string::npos) << result.reason;
}

TEST_F(CrossValidatorTest, DestructorRemovesTheCallback)
{
  const double too_fast = committed("max_linear_vel") + 0.1;
  EXPECT_FALSE(
    node_->set_parameter(rclcpp::Parameter(kPlugin + ".desired_speed", too_fast)).successful);
  validator_.reset();
  // gpl alone checks each parameter on its own, so the cross-parameter violation goes through.
  EXPECT_TRUE(
    node_->set_parameter(rclcpp::Parameter(kPlugin + ".desired_speed", too_fast)).successful);
}

TEST(CrossValidatorFollower, EmptyPrefixValidatesRootNames)
{
  auto node = makeNode("vf_cross_validator_follower");
  auto listener = std::make_shared<vr::CommonParamListener>(node, "");
  std::vector<vr::ParamSetter> setters;
  vr::appendSetters(vr::kCommonCoreParams, &setters);
  const vr::CrossValidator validator(
    node->get_node_parameters_interface(), "", std::move(setters), std::nullopt,
    [listener]() {
      vr::CoreParamTarget target;
      target.config = vr::toCoreConfig(listener->get_params());
      return target;
    },
    node->get_logger());
  EXPECT_EQ(validator.prefix(), "");
  const double max_linear_vel = node->get_parameter("max_linear_vel").as_double();
  EXPECT_FALSE(
    validator.validate({rclcpp::Parameter("desired_speed", max_linear_vel + 0.1)}).successful);
  EXPECT_TRUE(validator.validate({rclcpp::Parameter("use_sim_time", true)}).successful);
  EXPECT_TRUE(
    validator.validate({rclcpp::Parameter("desired_speed", 0.5 * max_linear_vel)}).successful);
}

}  // namespace
