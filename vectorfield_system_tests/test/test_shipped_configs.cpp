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
/// local costmap (or the local costmap size the file sets: the Ackermann files use 4 × 4 m for the
/// car's contour band) and the TB3 footprint in both obstacle modes, without errors or warnings.
/// With the stock 3 × 3 m window, the Ackermann files with contour report exactly the
/// blend-zone warning. The copy-paste templates `config/examples/nav2_params_vectorfield_<model>.yaml` pass the same
/// checks, as written and in both obstacle modes, and so do the two rotation-shim examples under
/// their distro's plugin prefix.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
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

// The local costmap of `file`: the stock one, with the width and height (m) that the file sets
// for /local_costmap/local_costmap, if any.
vf::CostmapInfo fileCostmap(const fs::path & file)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.arguments({"--ros-args", "--params-file", file.string()});
  options.automatically_declare_parameters_from_overrides(true);
  const auto node = std::make_shared<rclcpp::Node>("local_costmap", "local_costmap", options);
  vf::CostmapInfo costmap = stockCostmap();
  for (const auto & [name, size] :
       {std::pair<std::string, double *>{"width", &costmap.width_m},
        {"height", &costmap.height_m}}) {
    rclcpp::Parameter value;
    if (node->get_parameter(name, value)) {
      *size = value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                ? static_cast<double>(value.as_int())
                : value.as_double();
    }
  }
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
// unless `mode` is empty, and validates the keys under `plugin` with `costmap` (default: the
// file's local costmap).
vf::ValidationReport validate(
  const fs::path & file, const std::string & mode, vf::CoreConfig * config,
  const std::string & plugin = kPlugin, std::optional<vf::CostmapInfo> costmap = std::nullopt)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.arguments({"--ros-args", "--params-file", file.string()});
  if (!mode.empty()) {
    options.parameter_overrides({rclcpp::Parameter(plugin + ".obstacle_mode", mode)});
  }
  const auto node = std::make_shared<rclcpp::Node>("controller_server", options);
  return vr::validateFromNode(
    node, plugin, costmap ? *costmap : fileCostmap(file), tb3Footprint(), config);
}

// The Ackermann files with contour in the stock 3 x 3 m window: valid, with exactly the
// blend-zone warning (D'_in = 1.47 m >= 1.5 - 0.2).
void expectStockWindowWarning(const fs::path & file)
{
  vf::CoreConfig config;
  const vf::ValidationReport report = validate(file, "contour", &config, kPlugin, stockCostmap());
  EXPECT_TRUE(report.errors.empty()) << report.summary();
  ASSERT_EQ(report.warnings.size(), 1U) << report.summary();
  EXPECT_EQ(report.warnings[0].code, vf::issue::kBlendExceedsWindow) << report.summary();
}

// The Ackermann files with cbf in the stock 3 x 3 m window: valid without warnings, the turning
// margin fits (x_r + delta_safe + R_min + v_r / gamma_b = 0 + 0.27 + 0.5 + 0.2 <= 1.5).
void expectCbfInStockWindow(const fs::path & file)
{
  SCOPED_TRACE(file.filename().string() + " with cbf in the stock window");
  vf::CoreConfig config;
  const vf::ValidationReport report = validate(file, "cbf", &config, kPlugin, stockCostmap());
  EXPECT_STREQ(vf::toString(config.obstacles.obstacle_mode), "cbf");
  EXPECT_TRUE(report.errors.empty()) << report.summary();
  EXPECT_TRUE(report.warnings.empty()) << report.summary();
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

TEST(ShippedConfigs, ValidateInEveryObstacleMode)
{
  for (const fs::path & file : controllerOverlays()) {
    const std::string model = file.stem().string();
    for (const std::string & mode : std::vector<std::string>{"none", "contour", "cbf"}) {
      SCOPED_TRACE(file.filename().string() + " with obstacle_mode " + mode);
      vf::CoreConfig config;
      const vf::ValidationReport report = validate(file, mode, &config);
      // The file and the override both reached the node (Ackermann with cbf too, since 0.3.0).
      EXPECT_STREQ(vf::toString(config.motion.motion_model), model.c_str());
      EXPECT_STREQ(vf::toString(config.obstacles.obstacle_mode), mode.c_str());
      EXPECT_TRUE(report.errors.empty()) << report.summary();
      EXPECT_TRUE(report.warnings.empty()) << report.summary();
    }
    if (model == "ackermann") {
      SCOPED_TRACE(file.filename().string() + " with contour in the stock window");
      expectStockWindowWarning(file);
      expectCbfInStockWindow(file);
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
    // As written (empty: no override), then in every obstacle mode.
    for (const std::string & mode : std::vector<std::string>{"", "none", "contour", "cbf"}) {
      const std::string label = mode.empty() ? std::string("as written") : mode;
      SCOPED_TRACE(file.filename().string() + " with obstacle_mode " + label);
      vf::CoreConfig config;
      const vf::ValidationReport report = validate(file, mode, &config);
      EXPECT_STREQ(vf::toString(config.motion.motion_model), model.c_str());
      EXPECT_TRUE(report.errors.empty()) << report.summary();
      EXPECT_TRUE(report.warnings.empty()) << report.summary();
    }
    if (model == "ackermann") {
      SCOPED_TRACE(file.filename().string() + " with contour in the stock window");
      expectStockWindowWarning(file);
      expectCbfInStockWindow(file);
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
