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

#include "vectorfield_follower_3d/goal_setup.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"
#include "vectorfield_follower_3d/conversions.hpp"

namespace vectorfield_follower_3d
{

namespace vf = vectorfield_core;
using vectorfield_curve_follower::result_code::kInvalidCurve;
using vectorfield_curve_follower::result_code::kInvalidGoal;
using vectorfield_curve_follower::result_code::kUnsupported;
using FollowCurve3 = vectorfield_msgs::action::FollowCurve3;
using Curve3Msg = vectorfield_msgs::msg::Curve3;

namespace
{

/// The pose parameters every 3D built-in takes.
constexpr std::array<const char *, 6> kPoseParameters = {"cx", "cy", "cz", "yaw", "pitch", "roll"};

/// Tolerance of a pose quaternion's norm.
constexpr double kQuaternionNormTolerance = 1e-3;

std::optional<GoalRejection> reject(std::uint16_t code, std::string message)
{
  return GoalRejection{code, std::move(message)};
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

bool fail(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

/// Whether a pose is the identity (zero position, identity or all-zero quaternion).
bool isIdentity(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const bool zero_quaternion = q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0;
  const bool unit_quaternion = q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 1.0;
  return pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
         (zero_quaternion || unit_quaternion);
}

}  // namespace

std::optional<vf::d3::CurveSpec3> toCurveSpec3(
  const Curve3Msg & msg, double t0, std::string * error)
{
  vf::d3::CurveSpec3 spec;
  switch (msg.type) {
    case Curve3Msg::POLYLINE:
      spec.type = vf::d3::CurveSpec3::Type::kPolyline;
      break;
    case Curve3Msg::EXPRESSION:
      spec.type = vf::d3::CurveSpec3::Type::kExpression;
      break;
    case Curve3Msg::BUILTIN:
      spec.type = vf::d3::CurveSpec3::Type::kBuiltin;
      break;
    default:
      fail(
        error,
        "unknown curve type " + std::to_string(msg.type) + " (POLYLINE, EXPRESSION or BUILTIN)");
      return std::nullopt;
  }
  if (!std::isfinite(msg.sample_spacing)) {
    fail(error, "sample_spacing must be finite");
    return std::nullopt;
  }
  spec.closed = msg.closed;
  spec.points.reserve(msg.points.size());
  for (const auto & point : msg.points) {
    spec.points.push_back(toVec3(point));
  }
  spec.x_expression = msg.x_expression;
  spec.y_expression = msg.y_expression;
  spec.z_expression = msg.z_expression;
  spec.s_min = msg.s_min;
  spec.s_max = msg.s_max;
  spec.builtin_shape = msg.builtin_shape;
  spec.parameter_names = msg.parameter_names;
  spec.parameter_values = msg.parameter_values;
  spec.t0 = t0;
  if (isIdentity(msg.pose)) {
    return spec;
  }
  if (msg.type != Curve3Msg::BUILTIN) {
    fail(error, "pose places BUILTIN curves only; it must be the identity for other types");
    return std::nullopt;
  }
  for (const char * name : kPoseParameters) {
    if (
      std::find(spec.parameter_names.begin(), spec.parameter_names.end(), name) !=
      spec.parameter_names.end()) {
      fail(
        error,
        std::string("pose and the pose parameter '") + name + "' are both given; use one of them");
      return std::nullopt;
    }
  }
  const auto & q = msg.pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  const bool zero_quaternion = norm == 0.0;
  if (
    !std::isfinite(norm) || (!zero_quaternion && std::abs(norm - 1.0) > kQuaternionNormTolerance)) {
    fail(error, "pose.orientation must be a unit quaternion (or all zero)");
    return std::nullopt;
  }
  const vf::d3::Vec3 c = toVec3(msg.pose.position);
  if (!vf::d3::isFinite(c)) {
    fail(error, "pose.position must be finite");
    return std::nullopt;
  }
  const EulerZYX angles = eulerZYX(toQuaternion(q).normalized().toRotationMatrix());
  const std::array<double, 6> values = {c.x(), c.y(), c.z(), angles.yaw, angles.pitch, angles.roll};
  for (std::size_t i = 0; i < kPoseParameters.size(); ++i) {
    spec.parameter_names.emplace_back(kPoseParameters[i]);
    spec.parameter_values.push_back(values[i]);
  }
  return spec;
}

Curve3Msg toCurve3Msg(const vf::d3::CurveSpec3 & spec)
{
  Curve3Msg msg;
  switch (spec.type) {
    case vf::d3::CurveSpec3::Type::kPolyline:
      msg.type = Curve3Msg::POLYLINE;
      break;
    case vf::d3::CurveSpec3::Type::kExpression:
      msg.type = Curve3Msg::EXPRESSION;
      break;
    case vf::d3::CurveSpec3::Type::kBuiltin:
      msg.type = Curve3Msg::BUILTIN;
      break;
  }
  msg.closed = spec.closed;
  for (const vf::d3::Vec3 & p : spec.points) {
    msg.points.push_back(toPointMsg(p));
  }
  msg.x_expression = spec.x_expression;
  msg.y_expression = spec.y_expression;
  msg.z_expression = spec.z_expression;
  msg.s_min = spec.s_min;
  msg.s_max = spec.s_max;
  msg.builtin_shape = spec.builtin_shape;
  msg.parameter_names = spec.parameter_names;
  msg.parameter_values = spec.parameter_values;
  msg.pose.orientation.w = 1.0;
  return msg;
}

vf::d3::Tracker3Config goalScopedConfig3(
  const vf::d3::Tracker3Config & base, double desired_speed, bool reverse_direction)
{
  vf::d3::Tracker3Config config = base;
  if (desired_speed > 0.0) {
    config.desired_speed = desired_speed;
  }
  config.direction = reverse_direction ? -1 : 1;
  return config;
}

std::optional<GoalRejection> prepareGoal3(
  const FollowCurve3::Goal & goal, const GoalSetupContext3 & context, PreparedGoal3 * out)
{
  // 1. Goal fields.
  if (
    goal.frame_motion != FollowCurve3::Goal::FRAME_STATIC &&
    goal.frame_motion != FollowCurve3::Goal::FRAME_MOVING) {
    return reject(
      kInvalidGoal, "unknown frame_motion " + std::to_string(goal.frame_motion) +
                      " (FRAME_STATIC or FRAME_MOVING)");
  }
  if (goal.max_duration.sec < 0) {
    return reject(kInvalidGoal, "max_duration must not be negative");
  }
  if (!std::isfinite(goal.desired_speed)) {
    return reject(kInvalidGoal, "desired_speed must be finite");
  }
  // 2. The curve (makeCurve3 is the single validation point).
  const bool zero_stamp = goal.curve.header.stamp.sec == 0 && goal.curve.header.stamp.nanosec == 0;
  const double t0 = zero_stamp ? context.now : stampSeconds(goal.curve.header.stamp);
  std::string error;
  auto spec = toCurveSpec3(goal.curve, t0, &error);
  if (!spec) {
    return reject(kInvalidCurve, error);
  }
  out->spec = std::move(*spec);
  try {
    out->curve = vf::d3::makeCurve3(out->spec, context.curve_options);
  } catch (const vf::CurveError & e) {
    return reject(
      e.code() == vf::CurveError::Code::kUnsupported ? kUnsupported : kInvalidCurve, e.what());
  } catch (const std::exception & e) {
    return reject(kInvalidCurve, e.what());
  }
  if (goal.laps > 0 && !out->curve->isClosed()) {
    return reject(kInvalidCurve, "laps > 0 needs a closed curve");
  }
  out->sample_spacing = goal.curve.sample_spacing > 0.0
                          ? goal.curve.sample_spacing
                          : context.curve_options.default_sample_spacing;
  // 3. Frame sources.
  out->frame_id =
    goal.curve.header.frame_id.empty() ? context.global_frame : goal.curve.header.frame_id;
  out->moving = goal.frame_motion == FollowCurve3::Goal::FRAME_MOVING;
  out->frame_odom_topic = out->moving ? goal.frame_odom_topic : std::string();
  if (out->moving && out->frame_odom_topic.empty()) {
    if (context.moving_frame_reference.empty()) {
      return reject(
        kUnsupported,
        "FRAME_MOVING without frame_odom_topic needs the moving_frame_reference parameter");
    }
    if (context.moving_frame_reference == out->frame_id) {
      return reject(
        kUnsupported, "moving_frame_reference equals the curve frame '" + out->frame_id + "'");
    }
  }
  // 4. Goal-scoped configuration.
  out->desired_speed = goal.desired_speed;
  out->reverse_direction = goal.reverse_direction;
  out->laps = goal.laps;
  out->max_duration = static_cast<double>(goal.max_duration.sec) +
                      1e-9 * static_cast<double>(goal.max_duration.nanosec);
  out->config = goalScopedConfig3(context.config, goal.desired_speed, goal.reverse_direction);
  try {
    vf::d3::validateTracker3Config(out->config);
  } catch (const std::exception & e) {
    return reject(kInvalidGoal, std::string("goal configuration rejected: ") + e.what());
  }
  return std::nullopt;
}

bool keepsTrackerState3(const PreparedGoal3 & current, const PreparedGoal3 & next)
{
  if (
    !current.curve || !next.curve || current.frame_id != next.frame_id ||
    current.moving != next.moving || current.frame_odom_topic != next.frame_odom_topic ||
    current.desired_speed != next.desired_speed ||
    current.reverse_direction != next.reverse_direction) {
    return false;
  }
  if (current.curve->isTimeVarying() || next.curve->isTimeVarying()) {
    return current.spec == next.spec;
  }
  vf::d3::CurveSpec3 a = current.spec;
  a.t0 = next.spec.t0;
  return a == next.spec;
}

}  // namespace vectorfield_follower_3d
