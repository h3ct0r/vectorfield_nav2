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

#include "vectorfield_ros/field_state.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_ros
{

namespace
{

void toPoint(const vectorfield_core::Vec2 & v, geometry_msgs::msg::Point * out) noexcept
{
  out->x = v.x();
  out->y = v.y();
  out->z = 0.0;
}

void toVector(const vectorfield_core::Vec2 & v, geometry_msgs::msg::Vector3 * out) noexcept
{
  out->x = v.x();
  out->y = v.y();
  out->z = 0.0;
}

void toTwist(const vectorfield_core::Twist2 & u, geometry_msgs::msg::Twist * out) noexcept
{
  out->linear.x = u.vx;
  out->linear.y = u.vy;
  out->linear.z = 0.0;
  out->angular.x = 0.0;
  out->angular.y = 0.0;
  out->angular.z = u.wz;
}

}  // namespace

void toFieldState(
  const vectorfield_core::TrackerOutput & output, double compute_time_ms,
  vectorfield_msgs::msg::FieldState * msg) noexcept
{
  const vectorfield_core::FieldSample & field = output.field;
  msg->phase = vectorfield_core::toUnderlying(output.phase);
  msg->status = output.status;
  toPoint(field.x_eval, &msg->control_point);
  toPoint(field.x_star, &msg->closest_point);
  toVector(field.T_d, &msg->tangent);
  msg->distance = field.D;
  msg->arc_length = field.s_star;
  msg->distance_to_end = field.sigma;
  msg->g = field.G;
  msg->h = field.H;
  msg->eta = field.eta;
  toVector(field.Phi_T, &msg->feedforward);
  toVector(field.F, &msg->field);
  msg->obstacle_distance = field.contour.D_o;
  msg->blend = field.contour.theta;
  msg->contour_direction = field.contour.sigma_c;
  toTwist(output.cmd_target, &msg->cmd_raw);
  toTwist(output.cmd, &msg->cmd);
  msg->limiter_scale = output.limiter_alpha;
  msg->free_arc_length = output.s_free;
  msg->compute_time_ms = compute_time_ms;
}

}  // namespace vectorfield_ros
