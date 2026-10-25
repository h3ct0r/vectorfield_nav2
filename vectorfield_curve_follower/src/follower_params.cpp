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

// The binding table kFollowerCoreParams. As in vectorfield_ros' kCommonCoreParams, a
// row is written once from the member name, so the parameter name, the gpl Params member and
// the core member cannot drift apart.

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/parameter_value.hpp"
#include "vectorfield_curve_follower/follower_params.hpp"

namespace vectorfield_curve_follower
{
namespace
{

namespace vr = vectorfield_ros;

rclcpp::ParameterValue toValue(double value) { return rclcpp::ParameterValue(value); }
rclcpp::ParameterValue toValue(int value)
{
  return rclcpp::ParameterValue(static_cast<std::int64_t>(value));
}
rclcpp::ParameterValue toValue(std::int64_t value) { return rclcpp::ParameterValue(value); }

// Doubles also accept an integer value (see vectorfield_ros' table).
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
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) {
    return false;
  }
  *out = static_cast<int>(v);
  return true;
}

#define VF_FOLLOWER_ROW(MEMBER, NAME)                                                \
  FollowerParamRow                                                                   \
  {                                                                                  \
    #NAME, [](const FollowerParams & params) { return toValue(params.NAME); },       \
      [](vr::CoreParamTarget & target, const rclcpp::ParameterValue & value) {       \
        return setValue(value, &target.MEMBER.NAME);                                 \
      },                                                                             \
      [](const vr::CoreParamTarget & target) { return toValue(target.MEMBER.NAME); } \
  }

// YAML order (params/vectorfield_follower_params.yaml).
constexpr FollowerParamRow kRows[] = {
  // Curve intake
  VF_FOLLOWER_ROW(curve_options, max_curve_points),
  VF_FOLLOWER_ROW(curve_options, default_sample_spacing),
  VF_FOLLOWER_ROW(curve_options, search_window),
  VF_FOLLOWER_ROW(curve_options, global_search_trigger),
  VF_FOLLOWER_ROW(curve_options, global_search_hysteresis),
  VF_FOLLOWER_ROW(curve_options, global_search_samples),
  // Time-varying curves
  VF_FOLLOWER_ROW(config.field, assumption3_margin),
};

#undef VF_FOLLOWER_ROW

static_assert(
  std::size(kRows) == kFollowerCoreParamCount,
  "kFollowerCoreParamCount must equal the number of rows of kFollowerCoreParams");

template <std::size_t... I>
constexpr std::array<FollowerParamRow, sizeof...(I)> makeTable(std::index_sequence<I...>)
{
  return {{kRows[I]...}};
}

}  // namespace

const std::array<FollowerParamRow, kFollowerCoreParamCount> kFollowerCoreParams =
  makeTable(std::make_index_sequence<kFollowerCoreParamCount>{});

std::vector<vectorfield_ros::ParamSetter> followerSetters()
{
  std::vector<vectorfield_ros::ParamSetter> setters;
  setters.reserve(vectorfield_ros::kCommonCoreParamCount + kFollowerCoreParamCount);
  vectorfield_ros::appendSetters(vectorfield_ros::kCommonCoreParams, &setters);
  vectorfield_ros::appendSetters(kFollowerCoreParams, &setters);
  return setters;
}

bool toFollowerTarget(
  const vectorfield_ros::CommonParams & common, const FollowerParams & follower,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  vectorfield_ros::CoreParamTarget * target, std::string * error)
{
  *target = vectorfield_ros::CoreParamTarget{};
  std::string first;
  bool ok = vectorfield_ros::applyRows(vectorfield_ros::kCommonCoreParams, common, target, &first);
  std::string second;
  if (!vectorfield_ros::applyRows(kFollowerCoreParams, follower, target, &second) && ok) {
    ok = false;
    first = second;
  }
  if (!ok && error != nullptr) {
    *error = first;
  }
  if (footprint.empty()) {
    target->config.node.controller_frequency = follower.controller_frequency;
  } else {
    vectorfield_ros::setNodeDerivedFields(
      follower.controller_frequency, footprint, &target->config);
  }
  return ok;
}

}  // namespace vectorfield_curve_follower
