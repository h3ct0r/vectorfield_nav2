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

#include "vectorfield_behavior_tree/follow_curve_action.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vectorfield_behavior_tree/bt_compat.hpp"

namespace vectorfield_behavior_tree
{

namespace
{

using Curve = vectorfield_msgs::msg::Curve;
using Result = vectorfield_msgs::action::FollowCurve::Result;

/// The longest max_duration that rclcpp::Duration holds in int64 nanoseconds, rounded down.
constexpr double kMaxDurationLimit = 9.0e9;

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(" \t\n\r");
  return text.substr(first, last - first + 1);
}

std::optional<double> parseNumber(const std::string & text)
{
  double value = 0.0;
  const char * end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  if (text.empty() || ec != std::errc() || ptr != end || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

/// True when the tree sets `port` (an attribute or a remapping) to a non-empty value.
bool portGiven(const BT::TreeNode & node, const char * port)
{
  const auto & ports = node.config().input_ports;
  const auto it = ports.find(port);
  return it != ports.end() && !it->second.empty();
}

/// Reads `port` into `value`. A port the tree does not set keeps `value`, its default: BT.CPP v3
/// leaves ports whose default is an empty string out of input_ports.
template <class T>
std::optional<std::string> readPort(const BT::TreeNode & node, const char * port, T & value)
{
  if (node.config().input_ports.count(port) == 0) {
    return std::nullopt;
  }
  if (auto read = node.getInput(port, value); !read) {
    return std::string(port) + ": " + read.error();
  }
  return std::nullopt;
}

}  // namespace

std::optional<std::string> parseShapeParameters(
  const std::string & text, std::vector<std::string> & names, std::vector<double> & values)
{
  names.clear();
  values.clear();
  std::size_t start = 0;
  while (true) {
    const auto end = text.find_first_of(";,", start);
    const std::string item =
      trim(text.substr(start, end == std::string::npos ? std::string::npos : end - start));
    if (!item.empty()) {
      const auto equals = item.find('=');
      if (equals == std::string::npos) {
        return "'" + item + "' is not name=value";
      }
      const std::string name = trim(item.substr(0, equals));
      if (name.empty()) {
        return "'" + item + "' has no name";
      }
      const auto value = parseNumber(trim(item.substr(equals + 1)));
      if (!value) {
        return "the value of '" + name + "' is not a finite number";
      }
      if (std::find(names.begin(), names.end(), name) != names.end()) {
        return "'" + name + "' is given twice";
      }
      names.push_back(name);
      values.push_back(*value);
    }
    if (end == std::string::npos) {
      return std::nullopt;
    }
    start = end + 1;
  }
}

FollowCurveAction::FollowCurveAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

BT::PortsList FollowCurveAction::providedPorts()
{
  // On Lyrical the basic ports already contain error_code_id and error_msg, with these types.
  return providedBasicPorts({
    BT::InputPort<Curve>(
      "curve", "The curve to follow, usually a blackboard entry; exclusive with shape"),
    BT::InputPort<std::string>(
      "shape",
      "A built-in closed shape (ellipse, lemniscate_bernoulli, ...); exclusive "
      "with curve"),
    BT::InputPort<std::string>(
      "parameters", "", "The built-in's parameters, name=value pairs: a=2.0;b=1.0"),
    BT::InputPort<std::string>(
      "frame_id", "", "The built-in's frame; empty: the follower's global_frame"),
    BT::InputPort<std::uint32_t>("laps", 1U, "Laps of a closed curve; 0: until the node is halted"),
    BT::InputPort<double>(
      "desired_speed", 0.0, "Cruise speed (m/s); <= 0: the follower's parameter"),
    BT::InputPort<bool>("reverse", false, "Follow the curve against its direction"),
    BT::InputPort<std::string>(
      "frame_motion", "static", "static, or moving: the curve frame moves"),
    BT::InputPort<std::string>(
      "frame_odom_topic", "", "moving: the frame's Odometry; empty: TF differencing"),
    BT::InputPort<double>("max_duration", 0.0, "Time limit (s); 0: unlimited"),
    BT::OutputPort<std::uint16_t>("error_code_id", "FollowCurve result error_code"),
    BT::OutputPort<std::string>("error_msg", "FollowCurve result error_msg"),
    BT::OutputPort<std::uint32_t>("laps_completed", "FollowCurve result laps_completed"),
    BT::OutputPort<double>("distance_travelled", "FollowCurve result distance_travelled (m)"),
  });
}

std::optional<std::string> FollowCurveAction::makeGoal(Action::Goal & goal) const
{
  goal = Action::Goal();
  const bool has_curve = portGiven(*this, "curve");
  const bool has_shape = portGiven(*this, "shape");
  if (has_curve && has_shape) {
    return "set either the curve or the shape port, not both";
  }
  if (!has_curve && !has_shape) {
    return "set the curve or the shape port";
  }
  std::optional<std::string> error;
  if (has_curve) {
    error = readPort(*this, "curve", goal.curve);
  } else {
    std::string parameters;
    goal.curve.type = Curve::BUILTIN;
    goal.curve.closed = true;
    error = readPort(*this, "shape", goal.curve.builtin_shape);
    error = error ? error : readPort(*this, "parameters", parameters);
    error = error ? error : readPort(*this, "frame_id", goal.curve.header.frame_id);
    if (!error) {
      if (
        auto malformed = parseShapeParameters(
          parameters, goal.curve.parameter_names, goal.curve.parameter_values)) {
        error = "parameters: " + *malformed;
      }
    }
  }

  // The defaults of providedPorts().
  goal.laps = 1;
  goal.desired_speed = 0.0;
  goal.reverse_direction = false;
  std::string frame_motion = "static";
  double max_duration = 0.0;
  error = error ? error : readPort(*this, "laps", goal.laps);
  error = error ? error : readPort(*this, "desired_speed", goal.desired_speed);
  error = error ? error : readPort(*this, "reverse", goal.reverse_direction);
  error = error ? error : readPort(*this, "frame_motion", frame_motion);
  error = error ? error : readPort(*this, "frame_odom_topic", goal.frame_odom_topic);
  error = error ? error : readPort(*this, "max_duration", max_duration);
  if (error) {
    return error;
  }
  if (frame_motion == "static") {
    goal.frame_motion = Action::Goal::FRAME_STATIC;
  } else if (frame_motion == "moving") {
    goal.frame_motion = Action::Goal::FRAME_MOVING;
  } else {
    return "frame_motion: '" + frame_motion + "' is neither static nor moving";
  }
  if (!std::isfinite(max_duration) || max_duration < 0.0 || max_duration >= kMaxDurationLimit) {
    return "max_duration: " + std::to_string(max_duration) + " s is not in [0, 9e9)";
  }
  goal.max_duration = rclcpp::Duration::from_seconds(max_duration);
  return std::nullopt;
}

void FollowCurveAction::writeOutputs(const Action::Result * result)
{
  const Result reset;
  const Result & values = result != nullptr ? *result : reset;
  setOutput<std::uint16_t>("error_code_id", values.error_code);
  setOutput<std::string>("error_msg", values.error_msg);
  setOutput<std::uint32_t>("laps_completed", values.laps_completed);
  setOutput<double>("distance_travelled", values.distance_travelled);
}

void FollowCurveAction::on_tick()
{
  writeOutputs(nullptr);
  if (auto error = makeGoal(goal_)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: no goal sent: %s", name().c_str(), error->c_str());
    setOutput<std::uint16_t>("error_code_id", Result::INVALID_GOAL);
    setOutput<std::string>("error_msg", *error);
    should_send_goal_ = false;
  }
}

void FollowCurveAction::on_wait_for_result(std::shared_ptr<const Action::Feedback> /*feedback*/)
{
  Action::Goal goal;
  if (auto error = makeGoal(goal)) {
    // The running goal stays; the ports are checked again at the next start.
    RCLCPP_DEBUG(node_->get_logger(), "%s: goal not updated: %s", name().c_str(), error->c_str());
    return;
  }
  if (goal != goal_) {
    RCLCPP_INFO(node_->get_logger(), "%s: the goal changed; sending it", name().c_str());
    goal_ = goal;
    goal_updated_ = true;
  }
}

BT::NodeStatus FollowCurveAction::on_success()
{
  writeOutputs(result_.result.get());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowCurveAction::on_aborted()
{
  if (result_.result && result_.result->error_code != Result::NONE) {
    writeOutputs(result_.result.get());
    return BT::NodeStatus::FAILURE;
  }
  Result unknown = result_.result ? *result_.result : Result();
  unknown.error_code = Result::UNKNOWN;
  if (unknown.error_msg.empty()) {
    unknown.error_msg = "the follower aborted the goal without an error code (preempted?)";
  }
  writeOutputs(&unknown);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowCurveAction::on_cancelled()
{
  if (result_.result) {
    writeOutputs(result_.result.get());
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace vectorfield_behavior_tree

BT_REGISTER_NODES(factory)
{
  const BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<vectorfield_behavior_tree::FollowCurveAction>(
        name, "follow_curve", config);
    };
  factory.registerBuilder<vectorfield_behavior_tree::FollowCurveAction>("FollowCurve", builder);
}
