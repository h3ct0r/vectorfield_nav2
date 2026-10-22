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

#include "vectorfield_follower_3d/markers.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "std_msgs/msg/color_rgba.hpp"
#include "vectorfield_follower_3d/conversions.hpp"

namespace vectorfield_follower_3d
{

namespace d3 = vectorfield_core::d3;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

/// Length of the tangent arrow (m).
constexpr double kTangentLength = 0.4;
/// Length of a field arrow at the reference speed (m).
constexpr double kFieldArrowLength = 0.25;
/// Size of a drawn plane (m).
constexpr double kPlaneSize = 6.0;

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Marker base(const MarkerInput3 & in, const char * ns, std::int32_t type)
{
  Marker m;
  m.header.frame_id = in.frame_id;
  m.header.stamp = in.stamp;
  m.ns = ns;
  m.id = 0;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color = color(1.0F, 1.0F, 1.0F);
  return m;
}

Marker arrow(
  const MarkerInput3 & in, const char * ns, const d3::Vec3 & from, const d3::Vec3 & to,
  const std_msgs::msg::ColorRGBA & c, double shaft)
{
  Marker m = base(in, ns, Marker::ARROW);
  m.points.push_back(toPointMsg(from));
  m.points.push_back(toPointMsg(to));
  m.scale.x = shaft;
  m.scale.y = 2.0 * shaft;
  m.scale.z = 3.0 * shaft;
  m.color = c;
  return m;
}

Marker sphere(
  const MarkerInput3 & in, const char * ns, const d3::Vec3 & at, double diameter,
  const std_msgs::msg::ColorRGBA & c)
{
  Marker m = base(in, ns, Marker::SPHERE);
  m.pose.position = toPointMsg(at);
  m.scale.x = diameter;
  m.scale.y = diameter;
  m.scale.z = diameter;
  m.color = c;
  return m;
}

void addCurve(const MarkerInput3 & in, MarkerArray * out)
{
  const d3::Curve3 & curve = *in.curve;
  Marker m = base(in, "curve", Marker::LINE_STRIP);
  m.scale.x = 0.03;
  m.color = color(0.1F, 0.6F, 1.0F);
  const double length = curve.length(in.t);
  if (!(length > 0.0) || !std::isfinite(length)) {
    return;
  }
  const double spacing = std::max(in.sample_spacing, length / kMaxMarkerPoints);
  const int count = static_cast<int>(std::ceil(length / spacing));
  for (int k = 0; k <= count; ++k) {
    const double s = std::min(length, spacing * k);
    m.points.push_back(toPointMsg(in.frame.T_eval_curve * curve.pointAt(s, in.t)));
  }
  if (curve.isClosed() && !m.points.empty()) {
    m.points.push_back(m.points.front());
  }
  out->markers.push_back(m);
}

void addField(const MarkerInput3 & in, MarkerArray * out)
{
  if (in.field == nullptr || in.curve == nullptr || !(in.field_extent > 0.0)) {
    return;
  }
  Marker m = base(in, "field", Marker::LINE_LIST);
  m.scale.x = 0.01;
  m.color = color(0.6F, 0.6F, 0.6F, 0.8F);
  const int half = static_cast<int>(std::floor(in.field_extent / in.field_step + 1e-9));
  const d3::Isometry3 inverse = in.frame.T_eval_curve.inverse();
  const double v_r = in.field->referenceSpeed();
  for (int i = -half; i <= half; ++i) {
    for (int j = -half; j <= half; ++j) {
      for (int k = -half; k <= half; ++k) {
        const d3::Vec3 x = in.pose.p + in.field_step * d3::Vec3(i, j, k);
        const d3::Vec3 phi =
          in.frame.T_eval_curve.linear() * in.field->at(*in.curve, inverse * x, in.t);
        if (!d3::isFinite(phi) || !(v_r > 0.0)) {
          continue;
        }
        m.points.push_back(toPointMsg(x));
        m.points.push_back(toPointMsg(x + (kFieldArrowLength / v_r) * phi));
      }
    }
  }
  if (!m.points.empty()) {
    out->markers.push_back(m);
  }
}

void addObstacles(const MarkerInput3 & in, MarkerArray * out)
{
  if (in.obstacles == nullptr) {
    return;
  }
  const auto obstacle_color = color(0.55F, 0.55F, 0.55F, 0.7F);
  std::int32_t id = 0;
  for (const d3::Primitive3 & p : in.obstacles->primitives()) {
    Marker m = base(in, "obstacles", Marker::SPHERE);
    m.id = id++;
    m.color = obstacle_color;
    m.pose.position = toPointMsg(p.center);
    switch (p.type) {
      case d3::PrimitiveType3::kSphere:
        m.scale.x = m.scale.y = m.scale.z = 2.0 * p.radius;
        break;
      case d3::PrimitiveType3::kBox:
        m.type = Marker::CUBE;
        m.pose.orientation = toQuaternionMsg(Eigen::Quaterniond(p.rotation));
        m.scale.x = 2.0 * p.half_extents.x();
        m.scale.y = 2.0 * p.half_extents.y();
        m.scale.z = 2.0 * p.half_extents.z();
        break;
      case d3::PrimitiveType3::kCylinder:
        m.type = Marker::CYLINDER;
        m.scale.x = m.scale.y = 2.0 * p.radius;
        m.scale.z = std::isfinite(p.half_height) ? 2.0 * p.half_height : 2.0 * kPlaneSize;
        break;
      case d3::PrimitiveType3::kPlane: {
        // A thin square slab behind the plane, on its normal.
        m.type = Marker::CUBE;
        const Eigen::Quaterniond q =
          Eigen::Quaterniond::FromTwoVectors(d3::Vec3::UnitZ(), p.normal.normalized());
        m.pose.orientation = toQuaternionMsg(q);
        m.pose.position = toPointMsg(p.center - 0.01 * p.normal.normalized());
        m.scale.x = kPlaneSize;
        m.scale.y = kPlaneSize;
        m.scale.z = 0.02;
        m.color = color(0.55F, 0.55F, 0.55F, 0.25F);
        break;
      }
    }
    out->markers.push_back(m);
  }
  if (in.obstacles->hasVoxels()) {
    const d3::VoxelGrid3 & grid = in.obstacles->voxels();
    Marker m = base(in, "voxels", Marker::CUBE_LIST);
    m.scale.x = m.scale.y = m.scale.z = grid.resolution();
    m.color = color(0.8F, 0.3F, 0.3F, 0.6F);
    for (int iz = 0; iz < grid.nz(); ++iz) {
      for (int iy = 0; iy < grid.ny(); ++iy) {
        for (int ix = 0; ix < grid.nx(); ++ix) {
          if (grid.isSite(ix, iy, iz) && static_cast<int>(m.points.size()) < kMaxMarkerPoints) {
            m.points.push_back(toPointMsg(grid.cellCenter(ix, iy, iz)));
          }
        }
      }
    }
    if (!m.points.empty()) {
      out->markers.push_back(m);
    }
  }
}

}  // namespace

MarkerArray buildMarkers3(const MarkerInput3 & in)
{
  MarkerArray out = clearMarkers3(in.frame_id, in.stamp);
  if (in.curve != nullptr) {
    addCurve(in, &out);
  }
  if (in.output != nullptr) {
    const d3::Tracker3Output & o = *in.output;
    if (o.field.valid) {
      out.markers.push_back(
        sphere(in, "closest_point", o.field.x_star, 0.08, color(1.0F, 0.6F, 0.0F)));
      out.markers.push_back(arrow(
        in, "tangent", o.field.x_star, o.field.x_star + kTangentLength * o.field.T_d,
        color(1.0F, 0.6F, 0.0F), 0.02));
    }
    out.markers.push_back(
      arrow(in, "command", in.pose.p, in.pose.p + o.cmd.v, color(0.2F, 0.9F, 0.2F), 0.03));
    if (o.has_goal) {
      out.markers.push_back(sphere(in, "goal", o.goal, 0.12, color(0.9F, 0.1F, 0.9F)));
    }
  }
  addField(in, &out);
  addObstacles(in, &out);
  if (in.body_radius > 0.0) {
    out.markers.push_back(
      sphere(in, "body", in.pose.p, 2.0 * in.body_radius, color(0.2F, 0.9F, 0.2F, 0.25F)));
  }
  if (!in.status_text.empty()) {
    Marker m = base(in, "status", Marker::TEXT_VIEW_FACING);
    m.pose.position = toPointMsg(in.pose.p + d3::Vec3(0.0, 0.0, 0.5));
    m.scale.z = 0.15;
    m.text = in.status_text;
    out.markers.push_back(m);
  }
  return out;
}

MarkerArray clearMarkers3(const std::string & frame_id, const builtin_interfaces::msg::Time & stamp)
{
  MarkerArray out;
  Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.action = Marker::DELETEALL;
  out.markers.push_back(m);
  return out;
}

}  // namespace vectorfield_follower_3d
