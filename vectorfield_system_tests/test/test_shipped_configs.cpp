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
/// @brief Shipped-configuration validation (plan §5.4 "Tests"): every installed
/// `vectorfield_bringup/config/controller/*.yaml`, loaded with `--params-file` into a node named
/// controller_server, passes vectorfield_ros::validateFromNode() with the stock 3 × 3 m / 0.05 m
/// local costmap and the TB3 footprint in both obstacle modes, without errors or warnings. The
/// exception is ackermann.yaml with contour, which reports exactly the Ackermann+contour error.
/// The copy-paste templates `config/examples/nav2_params_vectorfield_<model>.yaml` pass the same
/// checks, as written and in both obstacle modes, and so do the two rotation-shim examples under
/// their distro's plugin prefix.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// get_package_share_path() exists from Jazzy on; Lyrical deprecates get_package_share_directory().
#if __has_include("ament_index_cpp/get_package_share_path.hpp")
#include "ament_index_cpp/get_package_share_path.hpp"
#define VF_HAS_SHARE_PATH 1
#else
#include "ament_index_cpp/get_package_share_directory.hpp"
#define VF_HAS_SHARE_PATH 0
#endif
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_ros/core_params.hpp"

namespace
{

namespace fs = std::filesystem;
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

// The installed YAML files of vectorfield_bringup's `config/<subdirectory>` whose names start
// with `prefix`, sorted by name.
std::vector<fs::path> installedConfigs(const std::string & subdirectory, const std::string & prefix)
{
#if VF_HAS_SHARE_PATH
  const fs::path share = ament_index_cpp::get_package_share_path("vectorfield_bringup");
#else
  const fs::path share = ament_index_cpp::get_package_share_directory("vectorfield_bringup");
#endif
  const fs::path directory = share / "config" / subdirectory;
  std::vector<fs::path> files;
  for (const fs::directory_entry & entry : fs::directory_iterator(directory)) {
    const std::string name = entry.path().filename().string();
    if (entry.path().extension() == ".yaml" && name.rfind(prefix, 0) == 0) {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

// The installed controller overlays.
std::vector<fs::path> controllerOverlays() { return installedConfigs("controller", ""); }

const std::string kTemplatePrefix = "nav2_params_vectorfield_";

// The installed copy-paste templates.
std::vector<fs::path> exampleTemplates() { return installedConfigs("examples", kTemplatePrefix); }

// The motion model a template is written for: the part of its stem after the prefix.
std::string templateModel(const fs::path & file)
{
  return file.stem().string().substr(kTemplatePrefix.size());
}

// Loads `file` as controller_server's parameter file, with `<plugin>.obstacle_mode` overridden
// unless `mode` is empty, and validates the keys under `plugin`.
vf::ValidationReport validate(
  const fs::path & file, const std::string & mode, vf::CoreConfig * config,
  const std::string & plugin = kPlugin)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.arguments({"--ros-args", "--params-file", file.string()});
  if (!mode.empty()) {
    options.parameter_overrides({rclcpp::Parameter(plugin + ".obstacle_mode", mode)});
  }
  const auto node = std::make_shared<rclcpp::Node>("controller_server", options);
  return vr::validateFromNode(node, plugin, stockCostmap(), tb3Footprint(), config);
}

TEST(ShippedConfigs, EveryMotionModelHasAnOverlay)
{
  std::vector<std::string> stems;
  for (const fs::path & file : controllerOverlays()) {
    stems.push_back(file.stem().string());
  }
  const std::vector<std::string> expected{
    "ackermann", "differential", "omnidirectional", "skid_steer"};
  EXPECT_EQ(stems, expected);
}

TEST(ShippedConfigs, ValidateInBothObstacleModes)
{
  for (const fs::path & file : controllerOverlays()) {
    const std::string model = file.stem().string();
    for (const std::string & mode : std::vector<std::string>{"none", "contour"}) {
      SCOPED_TRACE(file.filename().string() + " with obstacle_mode " + mode);
      vf::CoreConfig config;
      const vf::ValidationReport report = validate(file, mode, &config);
      // The file and the override both reached the node.
      EXPECT_STREQ(vf::toString(config.motion.motion_model), model.c_str());
      EXPECT_STREQ(vf::toString(config.obstacles.obstacle_mode), mode.c_str());
      if (model == "ackermann" && mode == "contour") {
        ASSERT_EQ(report.errors.size(), 1U) << report.summary();
        EXPECT_EQ(report.errors[0].code, vf::issue::kAckermannContour) << report.summary();
      } else {
        EXPECT_TRUE(report.errors.empty()) << report.summary();
      }
      EXPECT_TRUE(report.warnings.empty()) << report.summary();
    }
  }
}

TEST(ShippedConfigs, EveryTemplateModelExists)
{
  std::vector<std::string> models;
  for (const fs::path & file : exampleTemplates()) {
    models.push_back(templateModel(file));
  }
  const std::vector<std::string> expected{"ackermann", "differential", "omnidirectional"};
  EXPECT_EQ(models, expected);
}

TEST(ShippedConfigs, TemplatesValidate)
{
  for (const fs::path & file : exampleTemplates()) {
    const std::string model = templateModel(file);
    // As written (empty: no override), then in both obstacle modes.
    for (const std::string & mode : std::vector<std::string>{"", "none", "contour"}) {
      const std::string label = mode.empty() ? std::string("as written") : mode;
      SCOPED_TRACE(file.filename().string() + " with obstacle_mode " + label);
      vf::CoreConfig config;
      const vf::ValidationReport report = validate(file, mode, &config);
      EXPECT_STREQ(vf::toString(config.motion.motion_model), model.c_str());
      if (model == "ackermann" && mode == "contour") {
        ASSERT_EQ(report.errors.size(), 1U) << report.summary();
        EXPECT_EQ(report.errors[0].code, vf::issue::kAckermannContour) << report.summary();
      } else {
        EXPECT_TRUE(report.errors.empty()) << report.summary();
      }
      EXPECT_TRUE(report.warnings.empty()) << report.summary();
    }
  }
}

TEST(ShippedConfigs, RotationShimExamplesValidate)
{
  // Humble and Jazzy: the vector-field keys sit next to the shim's under FollowPath; Lyrical and
  // later: under FollowPath.primary_controller.
  const std::vector<std::pair<std::string, std::string>> examples{
    {"rotation_shim_humble_jazzy", kPlugin},
    {"rotation_shim_lyrical", kPlugin + ".primary_controller"}};
  for (const auto & [stem, plugin] : examples) {
    const std::vector<fs::path> files = installedConfigs("examples", stem + ".yaml");
    ASSERT_EQ(files.size(), 1U) << stem;
    SCOPED_TRACE(files[0].filename().string() + " under " + plugin);
    vf::CoreConfig config;
    const vf::ValidationReport report = validate(files[0], "", &config, plugin);
    EXPECT_STREQ(vf::toString(config.motion.motion_model), "differential");
    EXPECT_FALSE(config.phases.use_rotate_to_field);
    EXPECT_TRUE(report.errors.empty()) << report.summary();
    EXPECT_TRUE(report.warnings.empty()) << report.summary();
  }
}

}  // namespace
