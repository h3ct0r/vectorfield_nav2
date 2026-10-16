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

// Layer 2 (plan §9), through the server harness: RotationShimController with our plugin as its
// primary (plan §5.3; nav2_api_matrix.md §10). Without explicit parameters it configures,
// activates, publishes under `<ns>` and runs a goal; with obstacle_mode contour our configure()
// warns once, except on Jazzy (and Kilted) with rotate_to_heading_once.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "harness_fixtures.hpp"
#include "rcutils/logging.h"
#include "vectorfield_ros/compat.hpp"
#include "vectorfield_ros/topics.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace compat = vectorfield_ros::compat;
using vectorfield_nav2_controller::test::CycleRecord;
using vectorfield_nav2_controller::test::describe;
using vectorfield_nav2_controller::test::GoalStatus;
using vectorfield_nav2_controller::test::HarnessOptions;
using vectorfield_nav2_controller::test::mapPlan;
using vectorfield_nav2_controller::test::ServerHarness;
using vectorfield_nav2_controller::test::straightPlan;

const auto * const kRosEnvironment =
  ::testing::AddGlobalTestEnvironment(new vectorfield_nav2_controller::test::
                                        RosEnvironment);  // NOLINT(cppcoreguidelines-owning-memory)

/// Every cycle threw nothing.
void expectNoThrow(const ServerHarness & harness)
{
  for (const CycleRecord & record : harness.history()) {
    EXPECT_TRUE(record.exception.empty()) << "t = " << record.t << ": " << record.message;
  }
}

/// Our parameter prefix under the shim named "FollowPath".
std::string shimPrimaryName()
{
  return compat::kApiGeneration == 'C' ? "FollowPath.primary_controller" : "FollowPath";
}

/// Options for the shim with our plugin as primary: a 4 m costmap centred on (0.5, 0) and no
/// plugin parameters at all.
HarnessOptions shimOptions()
{
  HarnessOptions options;
  options.rotation_shim = true;
  options.costmap_size = 4.0;
  options.costmap_center = vf::Vec2(0.5, 0.0);
  return options;
}

/// Spins the harness node until `done` or a wall-clock timeout (DDS discovery only).
template <typename Done>
bool spinUntil(ServerHarness & harness, Done && done, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    harness.spinSome();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}

TEST(RotationShim, ConfiguresWithoutExplicitParametersAndPublishesUnderItsNamespace)
{
  ServerHarness harness(shimOptions());
  harness.configure();
  harness.activate();
  const auto node = harness.node();
  const std::string name = shimPrimaryName();
  EXPECT_TRUE(node->has_parameter(name + ".desired_speed"));
  EXPECT_TRUE(node->has_parameter(name + ".obstacle_mode"));
  const std::string markers =
    "/" + vectorfield_ros::topics::topicName(
            vectorfield_ros::topics::pluginNamespace(name), vectorfield_ros::topics::kMarkers);
  EXPECT_TRUE(spinUntil(
    harness, [&] { return node->count_publishers(markers) > 0; }, std::chrono::seconds(5)))
    << markers;

  harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(1.2, 0.0), 0.05, 0.0)));
  ASSERT_EQ(harness.runUntilDone(15.0), GoalStatus::kSucceeded) << describe(harness);
  expectNoThrow(harness);
  harness.deactivate();
  harness.cleanup();
}

/// WARN messages logged while it is alive (chained to the previous output handler).
class WarningCapture
{
public:
  WarningCapture() : previous_(rcutils_logging_get_output_handler())
  {
    active_ = this;
    rcutils_logging_set_output_handler(&WarningCapture::handle);
  }
  ~WarningCapture()
  {
    rcutils_logging_set_output_handler(previous_);
    active_ = nullptr;
  }
  WarningCapture(const WarningCapture &) = delete;
  WarningCapture & operator=(const WarningCapture &) = delete;

  /// Messages containing `text`.
  std::size_t count(const std::string & text) const
  {
    std::size_t n = 0;
    for (const std::string & message : messages_) {
      n += message.find(text) != std::string::npos ? 1U : 0U;
    }
    return n;
  }

private:
  static void handle(
    const rcutils_log_location_t * location, int severity, const char * name,
    rcutils_time_point_value_t timestamp, const char * format, va_list * args)
  {
    WarningCapture * self = active_;
    if (self != nullptr && severity == RCUTILS_LOG_SEVERITY_WARN) {
      va_list copy;
      va_copy(copy, *args);
      char buffer[1024];
      // The format comes from the logging macro that called this handler.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
      std::vsnprintf(buffer, sizeof(buffer), format, copy);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
      va_end(copy);
      self->messages_.emplace_back(buffer);
    }
    if (self != nullptr && self->previous_ != nullptr) {
      self->previous_(location, severity, name, timestamp, format, args);
    }
  }

  static inline WarningCapture * active_ = nullptr;
  rcutils_logging_output_handler_t previous_;
  std::vector<std::string> messages_;
};

/// (obstacle_mode contour, FollowPath.rotate_to_heading_once).
using ShimWarningCase = std::tuple<bool, bool>;

class RotationShimWarning : public ::testing::TestWithParam<ShimWarningCase>
{
};

TEST_P(RotationShimWarning, IsLoggedOnceWithContourUnlessJazzyRotatesToHeadingOnce)
{
  const auto & [contour, once] = GetParam();
  HarnessOptions options = shimOptions();
  if (contour) {
    options.parameters.emplace_back(shimPrimaryName() + ".obstacle_mode", std::string("contour"));
  }
  if (once) {
    // The shim's own key (declared by the Jazzy shim; unknown to Humble's).
    options.parameters.emplace_back("FollowPath.rotate_to_heading_once", true);
  }
  ServerHarness harness(options);
  std::size_t warnings = 0;
  {
    WarningCapture capture;
    harness.configure();
    harness.activate();
    harness.followPath(mapPlan(straightPlan(vf::Vec2::Zero(), vf::Vec2(1.0, 0.0), 0.05, 0.0)));
    for (int i = 0; i < 10; ++i) {
      harness.spinOnce();
    }
    warnings = capture.count("rotation shim re-aims");
  }
  const bool expected = contour && !(once && compat::kApiGeneration == 'B');
  EXPECT_EQ(warnings, expected ? 1U : 0U);
}

INSTANTIATE_TEST_SUITE_P(
  Layer2, RotationShimWarning,
  ::testing::Values(
    ShimWarningCase{false, false}, ShimWarningCase{true, false}, ShimWarningCase{true, true}),
  [](const ::testing::TestParamInfo<ShimWarningCase> & case_info) {
    return std::string(std::get<0>(case_info.param) ? "contour" : "none") +
           (std::get<1>(case_info.param) ? "_once" : "");
  });

}  // namespace
