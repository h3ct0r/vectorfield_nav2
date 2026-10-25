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

// Node-level parameter handling: node-derived fields, validateParams(), validateFromNode() and
// the CrossValidator. The binding table itself is in core_params_table.cpp.

#include <cstdint>
#include <exception>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "vectorfield_ros/conversions.hpp"
#include "vectorfield_ros/core_params.hpp"

namespace vectorfield_ros
{

namespace vf = vectorfield_core;

namespace
{

/// The controller_server's (and the follower's) control-frequency parameter.
constexpr const char * kControllerFrequency = "controller_frequency";

/// A numeric parameter value as a double; nullopt for any other type.
std::optional<double> toDouble(const rclcpp::ParameterValue & value)
{
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return value.get<double>();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(value.get<std::int64_t>());
    default:
      return std::nullopt;
  }
}

/// The errors of a report on one line ("[code] message; ..."), for a rejection reason.
std::string errorLine(const vf::ValidationReport & report)
{
  std::string text;
  for (const vf::ValidationIssue & issue : report.errors) {
    if (!text.empty()) {
      text += "; ";
    }
    text += "[" + issue.code + "] " + issue.message;
  }
  return text;
}

}  // namespace

void setNodeDerivedFields(
  double controller_frequency, const std::vector<geometry_msgs::msg::Point> & robot_footprint,
  vf::CoreConfig * config)
{
  config->node.controller_frequency = controller_frequency;
  config->node.footprint = toFootprintSpec(robot_footprint);
}

double controllerFrequencyOf(
  const rclcpp::node_interfaces::NodeParametersInterface & parameters, double fallback)
{
  rclcpp::Parameter declared;
  if (parameters.get_parameter(kControllerFrequency, declared)) {
    return toDouble(declared.get_parameter_value()).value_or(fallback);
  }
  const std::map<std::string, rclcpp::ParameterValue> & overrides =
    parameters.get_parameter_overrides();
  const auto it = overrides.find(kControllerFrequency);
  if (it != overrides.end()) {
    return toDouble(it->second).value_or(fallback);
  }
  return fallback;
}

vf::ValidationReport validateParams(
  const CommonParams & params, double controller_frequency,
  const std::optional<vf::CostmapInfo> & costmap,
  const std::vector<geometry_msgs::msg::Point> & robot_footprint, vf::CoreConfig * config)
{
  // The rows one by one rather than toCoreConfig(), to report every failing row as an error.
  vf::ValidationReport report;
  CoreParamTarget target;
  for (const CommonParamRow & row : kCommonCoreParams) {
    if (!row.set(target, row.from_params(params))) {
      report.errors.push_back(
        vf::ValidationIssue{
          std::string(vf::issue::kInvalidValue),
          std::string("parameter '") + row.name + "' has a value vectorfield_core cannot use"});
    }
  }
  setNodeDerivedFields(controller_frequency, robot_footprint, &target.config);
  if (config != nullptr) {
    *config = target.config;
  }
  // As in validateConfig(): when a value is unusable, the cross-parameter rules are meaningless.
  if (!report.ok()) {
    return report;
  }
  return vf::validateConfig(target.config, costmap);
}

vf::ValidationReport validateFromNode(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const rclcpp::Logger & logger, const std::string & plugin_name, const vf::CostmapInfo & costmap,
  const std::vector<geometry_msgs::msg::Point> & robot_footprint, vf::CoreConfig * config)
{
  CommonParams params;
  {
    // Declares the keys if needed (adopting overrides) and reads the committed values; the
    // listener's on-set callback is removed with it at the end of this scope.
    const CommonParamListener listener(parameters, logger, plugin_name);
    params = listener.get_params();
  }
  return validateParams(
    params, controllerFrequencyOf(*parameters), costmap, robot_footprint, config);
}

CrossValidator::CrossValidator(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
  const std::string & prefix, std::vector<ParamSetter> setters,
  std::optional<vf::CostmapInfo> costmap, BaseProvider base, rclcpp::Logger logger,
  ExtraCheck extra)
: parameters_(std::move(parameters)),
  prefix_(prefix),
  setters_(std::move(setters)),
  costmap_(std::move(costmap)),
  base_(std::move(base)),
  extra_(std::move(extra)),
  logger_(std::move(logger))
{
  if (!prefix_.empty() && prefix_.back() != '.') {
    prefix_ += '.';
  }
  handle_ = parameters_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & batch) { return validate(batch); });
}

CrossValidator::~CrossValidator()
{
  // Explicit removal (not only dropping the handle) waits for a callback in flight: rclcpp holds
  // the parameter mutex while it runs the callbacks.
  if (handle_ != nullptr) {
    try {
      parameters_->remove_on_set_parameters_callback(handle_.get());
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "CrossValidator: could not remove its callback: %s", e.what());
    }
  }
}

rcl_interfaces::msg::SetParametersResult CrossValidator::validate(
  const std::vector<rclcpp::Parameter> & parameters) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 1. Our names: under the prefix and held by a setter.
  std::vector<std::pair<const ParamSetter *, const rclcpp::Parameter *>> ours;
  for (const rclcpp::Parameter & parameter : parameters) {
    const std::string & name = parameter.get_name();
    if (name.size() <= prefix_.size() || name.compare(0, prefix_.size(), prefix_) != 0) {
      continue;
    }
    const std::string_view key = std::string_view(name).substr(prefix_.size());
    for (const ParamSetter & setter : setters_) {
      if (setter.name == key) {
        ours.emplace_back(&setter, &parameter);
        break;
      }
    }
  }
  if (ours.empty()) {
    return result;
  }

  // 2. The candidate: the committed configuration with this batch applied.
  CoreParamTarget candidate = base_ ? base_() : CoreParamTarget{};
  std::string names;
  std::string unusable;
  for (const auto & [setter, parameter] : ours) {
    names += (names.empty() ? "" : ", ") + parameter->get_name();
    if (!setter->set(candidate, parameter->get_parameter_value())) {
      unusable += (unusable.empty() ? "" : ", ") + parameter->get_name();
    }
  }
  if (!unusable.empty()) {
    result.successful = false;
    result.reason = "invalid value for " + unusable;
    RCLCPP_ERROR(logger_, "Parameter change rejected: %s", result.reason.c_str());
    return result;
  }

  // 3. The cross-parameter rules; warnings never reject.
  vf::ValidationReport report = vf::validateConfig(candidate.config, costmap_);
  if (extra_) {
    extra_(candidate, &report);
  }
  if (!report.ok()) {
    result.successful = false;
    result.reason = "rejected " + names + ": " + errorLine(report);
    RCLCPP_ERROR(logger_, "Parameter change rejected: %s", result.reason.c_str());
  }
  return result;
}

const std::string & CrossValidator::prefix() const noexcept { return prefix_; }

}  // namespace vectorfield_ros
