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

// The plugin is discoverable by its type string through pluginlib, as controller_server loads it
// (plan §5.3 "Plugin XML"), and it overrides every pure virtual of this distro's
// nav2_core::Controller. Nothing is configured here (layer 2 does that through the harness).

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "vectorfield_nav2_controller/vectorfield_controller.hpp"
#include "vectorfield_ros/compat.hpp"

namespace
{

using vectorfield_nav2_controller::VectorFieldController;

// The probes seen by a dependent package (through the exported vectorfield_ros target) agree with
// the CMake detection exported by vectorfield_ros (nav2_api_matrix.md §2.3: probes depend on the
// link line).
static_assert(
  VF_CMAKE_HAS_ROS_COMMON == VF_HAS_ROS_COMMON,
  "a dependent of vectorfield_ros sees a different nav2_ros_common than vectorfield_ros found");
static_assert(std::is_base_of_v<nav2_core::Controller, VectorFieldController>);
static_assert(
  !std::is_abstract_v<VectorFieldController>,
  "VectorFieldController must override every pure virtual of this distro's nav2_core::Controller");

constexpr const char * kPluginType = "vectorfield_nav2_controller::VectorFieldController";

TEST(PluginDiscovery, TheClassIsDeclaredForNav2CoreController)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  const std::vector<std::string> declared = loader.getDeclaredClasses();
  EXPECT_NE(std::find(declared.begin(), declared.end(), kPluginType), declared.end())
    << kPluginType << " is not declared for nav2_core::Controller";
  ASSERT_TRUE(loader.isClassAvailable(kPluginType));
  EXPECT_EQ(loader.getClassPackage(kPluginType), "vectorfield_nav2_controller");
  EXPECT_EQ(loader.getBaseClassType(), "nav2_core::Controller");
}

TEST(PluginDiscovery, AnInstanceIsCreatedFromTheTypeString)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  std::shared_ptr<nav2_core::Controller> controller = loader.createSharedInstance(kPluginType);
  ASSERT_NE(controller, nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<VectorFieldController>(controller), nullptr);
}

}  // namespace
