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

#include "vectorfield_curve_follower/goal_setup.hpp"

#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include "vectorfield_core/errors.hpp"
#include "vectorfield_curve_follower/goal_supervisor.hpp"
#include "vectorfield_ros/conversions.hpp"

namespace vectorfield_curve_follower
{

namespace vf = vectorfield_core;
using FollowCurve = vectorfield_msgs::action::FollowCurve;

namespace
{

GoalRejection reject(std::uint16_t code, std::string message)
{
  return GoalRejection{code, std::move(message)};
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

}  // namespace

vf::CoreConfig goalScopedConfig(
  const vf::CoreConfig & base, double desired_speed, bool reverse_direction)
{
  vf::CoreConfig config = base;
  if (desired_speed > 0.0) {
    config.field.desired_speed = desired_speed;
  }
  config.field.direction = reverse_direction ? -1 : 1;
  config.phases.use_rotate_to_heading = false;
  return config;
}

std::optional<GoalRejection> prepareGoal(
  const FollowCurve::Goal & goal, const GoalSetupContext & context, PreparedGoal * out)
{
  // 1. Goal fields.
  if (
    goal.frame_motion != FollowCurve::Goal::FRAME_STATIC &&
    goal.frame_motion != FollowCurve::Goal::FRAME_MOVING) {
    return reject(
      result_code::kInvalidGoal, "unknown frame_motion " + std::to_string(goal.frame_motion) +
                                   " (FRAME_STATIC or FRAME_MOVING)");
  }
  if (goal.max_duration.sec < 0) {
    return reject(result_code::kInvalidGoal, "max_duration must not be negative");
  }
  if (!std::isfinite(goal.desired_speed)) {
    return reject(result_code::kInvalidGoal, "desired_speed must be finite");
  }

  // 2. The curve (makeCurve is the single validation point).
  const bool zero_stamp = goal.curve.header.stamp.sec == 0 && goal.curve.header.stamp.nanosec == 0;
  const double t0 = zero_stamp ? context.now : stampSeconds(goal.curve.header.stamp);
  try {
    out->spec = vectorfield_ros::toCurveSpec(goal.curve, t0);
    out->curve = vf::makeCurve(out->spec, context.curve_options);
  } catch (const vf::CurveError & e) {
    return reject(
      e.code() == vf::CurveError::Code::kUnsupported ? result_code::kUnsupported
                                                     : result_code::kInvalidCurve,
      e.what());
  } catch (const std::exception & e) {
    return reject(result_code::kInvalidCurve, e.what());
  }
  if (goal.laps > 0 && !out->curve->isClosed()) {
    return reject(result_code::kInvalidCurve, "laps > 0 needs a closed curve");
  }

  // 3. Frame sources.
  out->frame_id =
    goal.curve.header.frame_id.empty() ? context.global_frame : goal.curve.header.frame_id;
  out->moving = goal.frame_motion == FollowCurve::Goal::FRAME_MOVING;
  out->frame_odom_topic = out->moving ? goal.frame_odom_topic : std::string();
  if (out->moving && out->frame_odom_topic.empty()) {
    if (context.moving_frame_reference.empty()) {
      return reject(
        result_code::kUnsupported,
        "FRAME_MOVING without frame_odom_topic needs the moving_frame_reference parameter");
    }
    if (context.moving_frame_reference == out->frame_id) {
      return reject(
        result_code::kUnsupported,
        "moving_frame_reference equals the curve frame '" + out->frame_id + "'");
    }
  }

  // 4. Goal-scoped configuration.
  out->desired_speed = goal.desired_speed;
  out->reverse_direction = goal.reverse_direction;
  out->laps = goal.laps;
  out->max_duration = static_cast<double>(goal.max_duration.sec) +
                      1e-9 * static_cast<double>(goal.max_duration.nanosec);
  try {
    out->config = goalScopedConfig(context.config, goal.desired_speed, goal.reverse_direction);
  } catch (const std::exception & e) {
    return reject(result_code::kInvalidGoal, e.what());
  }
  const vf::ValidationReport report = vf::validateConfig(out->config, std::nullopt);
  if (!report.ok()) {
    return reject(result_code::kInvalidGoal, "goal configuration rejected: " + report.summary());
  }
  return std::nullopt;
}

bool keepsTrackerState(const PreparedGoal & current, const PreparedGoal & next)
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
  vf::CurveSpec a = current.spec;
  a.t0 = next.spec.t0;
  return a == next.spec;
}

}  // namespace vectorfield_curve_follower
