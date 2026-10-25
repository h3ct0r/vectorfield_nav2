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

// The binding table kCommonCoreParams, toCoreConfig() and toVizConfig().
//
// Every vectorfield_core::CoreConfig field has the same name as its parameter (config.hpp), so a
// row is written once as VF_CORE_ROW(group, name): the stringised name, the gpl Params member and
// the CoreConfig member cannot drift apart, and a key missing from the YAML or from CoreConfig is
// a compile error.

#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

#include "rclcpp/parameter_value.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_ros/core_params.hpp"

namespace vectorfield_ros
{
namespace
{

rclcpp::ParameterValue toValue(double value) { return rclcpp::ParameterValue(value); }
rclcpp::ParameterValue toValue(bool value) { return rclcpp::ParameterValue(value); }
rclcpp::ParameterValue toValue(int value)
{
  return rclcpp::ParameterValue(static_cast<std::int64_t>(value));
}
rclcpp::ParameterValue toValue(std::int64_t value) { return rclcpp::ParameterValue(value); }
rclcpp::ParameterValue toValue(const std::string & value) { return rclcpp::ParameterValue(value); }

template <typename Enum, std::enable_if_t<std::is_enum_v<Enum>, int> = 0>
rclcpp::ParameterValue toValue(Enum value)
{
  return rclcpp::ParameterValue(std::string(vectorfield_core::toString(value)));
}

// Doubles also accept an integer value (rclcpp rejects a type change of a declared parameter
// before any callback runs, so this only matters for callers that build values by hand).
bool setValue(const rclcpp::ParameterValue & value, double * out)
{
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      *out = value.get<double>();
      return true;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      *out = static_cast<double>(value.get<std::int64_t>());
      return true;
    default:
      return false;
  }
}

bool setValue(const rclcpp::ParameterValue & value, int * out)
{
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
    return false;
  }
  const std::int64_t v = value.get<std::int64_t>();
  if (v < INT_MIN || v > INT_MAX) {
    return false;
  }
  *out = static_cast<int>(v);
  return true;
}

bool setValue(const rclcpp::ParameterValue & value, bool * out)
{
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    return false;
  }
  *out = value.get<bool>();
  return true;
}

template <typename Enum, std::enable_if_t<std::is_enum_v<Enum>, int> = 0>
bool setValue(const rclcpp::ParameterValue & value, Enum * out)
{
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    return false;
  }
  return vectorfield_core::fromString(value.get<std::string>(), out);
}

#define VF_CORE_ROW(GROUP, NAME)                                                       \
  CommonParamRow                                                                       \
  {                                                                                    \
    #NAME, [](const CommonParams & params) { return toValue(params.NAME); },           \
      [](CoreParamTarget & target, const rclcpp::ParameterValue & value) {             \
        return setValue(value, &target.config.GROUP.NAME);                             \
      },                                                                               \
      [](const CoreParamTarget & target) { return toValue(target.config.GROUP.NAME); } \
  }

// YAML order (params/vectorfield_common_params.yaml), Visualization group excluded.
constexpr CommonParamRow kRows[] = {
  // Field
  VF_CORE_ROW(field, desired_speed),
  VF_CORE_ROW(field, convergence_gain),
  VF_CORE_ROW(field, gain_function),
  // Motion
  VF_CORE_ROW(motion, motion_model),
  VF_CORE_ROW(motion, tracking_law),
  VF_CORE_ROW(motion, control_point_offset),
  VF_CORE_ROW(motion, drive_backward),
  VF_CORE_ROW(motion, allow_reversing),
  VF_CORE_ROW(motion, heading_gain),
  VF_CORE_ROW(motion, omni_heading_policy),
  VF_CORE_ROW(motion, max_linear_vel),
  VF_CORE_ROW(motion, max_reverse_vel),
  VF_CORE_ROW(motion, max_lateral_vel),
  VF_CORE_ROW(motion, max_angular_vel),
  VF_CORE_ROW(motion, max_linear_accel),
  VF_CORE_ROW(motion, max_linear_decel),
  VF_CORE_ROW(motion, max_angular_accel),
  VF_CORE_ROW(motion, min_in_place_angular_vel),
  VF_CORE_ROW(motion, min_turning_radius),
  VF_CORE_ROW(motion, ackermann_min_speed),
  VF_CORE_ROW(motion, reference_point_offset),
  VF_CORE_ROW(motion, skid_yaw_rate_efficiency),
  VF_CORE_ROW(motion, skid_icr_offset),
  // Phases
  VF_CORE_ROW(phases, use_rotate_to_field),
  VF_CORE_ROW(phases, rotate_to_field_angle),
  VF_CORE_ROW(phases, rotate_to_field_release_angle),
  VF_CORE_ROW(phases, use_rotate_to_heading),
  VF_CORE_ROW(phases, rotate_to_heading_angular_vel),
  VF_CORE_ROW(phases, final_approach_distance),
  VF_CORE_ROW(phases, terminal_braking_distance),
  VF_CORE_ROW(phases, min_approach_speed),
  VF_CORE_ROW(phases, cancel_deceleration),
  VF_CORE_ROW(phases, cusp_tolerance),
  // Obstacles
  VF_CORE_ROW(obstacles, obstacle_mode),
  VF_CORE_ROW(obstacles, obstacle_activation),
  VF_CORE_ROW(obstacles, obstacle_contour_distance),
  VF_CORE_ROW(obstacles, obstacle_full_switch_distance),
  VF_CORE_ROW(obstacles, obstacle_blend_start_distance),
  VF_CORE_ROW(obstacles, obstacle_lookahead),
  VF_CORE_ROW(obstacles, obstacle_gate_tolerance),
  VF_CORE_ROW(obstacles, obstacle_gate_release_margin),
  VF_CORE_ROW(obstacles, obstacle_release_time),
  VF_CORE_ROW(obstacles, obstacle_max_contour_length),
  VF_CORE_ROW(obstacles, obstacle_max_contour_time),
  VF_CORE_ROW(obstacles, unknown_is_obstacle),
  VF_CORE_ROW(obstacles, clearance_slowdown_distance),
  VF_CORE_ROW(obstacles, clearance_slowdown_gain),
  VF_CORE_ROW(obstacles, cbf_gain),
  VF_CORE_ROW(obstacles, cbf_margin),
  VF_CORE_ROW(obstacles, cbf_sites),
  // Safety
  VF_CORE_ROW(safety, use_collision_detection),
  VF_CORE_ROW(safety, collision_check_horizon),
  VF_CORE_ROW(safety, stop_margin),
};

#undef VF_CORE_ROW

static_assert(
  std::size(kRows) == kCommonCoreParamCount,
  "kCommonCoreParamCount must equal the number of rows of kCommonCoreParams");

template <std::size_t... I>
constexpr std::array<CommonParamRow, sizeof...(I)> makeTable(std::index_sequence<I...>)
{
  return {{kRows[I]...}};
}

}  // namespace

const std::array<CommonParamRow, kCommonCoreParamCount> kCommonCoreParams =
  makeTable(std::make_index_sequence<kCommonCoreParamCount>{});

vectorfield_core::CoreConfig toCoreConfig(const CommonParams & params)
{
  CoreParamTarget target;
  std::string failed;
  if (!applyRows(kCommonCoreParams, params, &target, &failed)) {
    throw vectorfield_core::ConfigError(
      "parameter '" + failed + "' has a value vectorfield_core cannot use");
  }
  return target.config;
}

VizConfig toVizConfig(const CommonParams & params)
{
  VizConfig config;
  config.visualize = params.visualize;
  config.field_grid_size = params.field_grid_size;
  config.field_grid_resolution = params.field_grid_resolution;
  config.field_grid_rate = params.field_grid_rate;
  if (!fromString(params.field_grid_style, &config.field_grid_style)) {
    throw vectorfield_core::ConfigError(
      "parameter 'field_grid_style' = '" + params.field_grid_style +
      "' must be 'arrows' or 'streamlines'");
  }
  config.publish_obstacle_distance = params.publish_obstacle_distance;
  return config;
}

}  // namespace vectorfield_ros
