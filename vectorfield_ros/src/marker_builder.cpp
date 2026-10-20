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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/footprint.hpp"
#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracking_law.hpp"
#include "vectorfield_ros/marker_builder.hpp"

namespace vectorfield_ros
{

namespace
{

namespace vf = vectorfield_core;
using legend::Ns;
using vf::Vec2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/// @name Marker geometry (m unless stated)
/// @{
/// Width of thin lines (curve, distances, arcs, chevrons).
constexpr double kLineWidth = 0.02;
/// Width of emphasised lines (footprint, braking zone, rollout).
constexpr double kThickLineWidth = 0.04;
/// Diameter of the large spheres (x*, g, x*_o, curve start).
constexpr double kSphereLarge = 0.10;
/// Diameter of the small spheres (control point, g_ref, virtual tail vertex).
constexpr double kSphereSmall = 0.06;
/// Shaft diameter, head diameter and head length of the thin arrows.
constexpr double kArrowShaft = 0.025;
constexpr double kArrowHead = 0.06;
constexpr double kArrowHeadLength = 0.08;
/// Shaft and head diameters of the thick arrows (resulting field F).
constexpr double kThickArrowShaft = 0.05;
constexpr double kThickArrowHead = 0.10;
/// Height of the text markers.
constexpr double kTextHeight = 0.12;
/// Height of the text of `prediction`, `contour` and `curve_info` above their anchor.
constexpr double kTextLift = 0.3;
/// Thickness of the CYLINDER rings.
constexpr double kRingHeight = 0.01;
/// Length of the tangent arrow at x*.
constexpr double kTangentLength = 0.3;
/// Length of the target-heading arrow, and radius of the heading arc.
constexpr double kHeadingLength = 0.6;
constexpr double kHeadingArcRadius = 0.35;
/// Length of the goal-yaw arrow.
constexpr double kGoalYawLength = 0.4;
/// Length of the contour side arrow \f$\sigma_c J n_o\f$.
constexpr double kSideArrowLength = 0.4;
/// Velocity arrows (field terms, command, frame velocity) show this much motion (s).
constexpr double kVelocityHorizon = 1.0;
/// Steps of the 1 s command arc and of the heading arc.
constexpr int kCommandArcSteps = 20;
constexpr int kHeadingArcSteps = 24;
/// Segments of a circular footprint.
constexpr int kCircleSegments = 32;
/// Radius drawn for a point robot.
constexpr double kPointRobotRadius = 0.05;
/// Extension rays: dashes of kDashLength every 2 kDashLength over kRayLength.
constexpr double kRayLength = 1.0;
constexpr double kDashLength = 0.1;
/// Vectors (m or m/s) shorter than this are not drawn.
constexpr double kMinVector = 1.0e-4;
/// Most points of a sampled curve (follower), and the smallest sampling step.
constexpr std::size_t kMaxCurvePoints = 1000;
constexpr double kMinCurveSpacing = 0.02;
/// `curve_info` chevrons: one every kChevronSpacing of arc length, at most kMaxChevrons.
constexpr double kChevronSpacing = 1.0;
constexpr std::size_t kMaxChevrons = 500;
constexpr double kChevronLength = 0.15;
constexpr double kChevronHalfWidth = 0.1;
/// Samples of the braking-zone overlay.
constexpr std::size_t kBrakingSamples = 17;
/// Side of the collision CUBE.
constexpr double kCollisionCube = 0.12;
/// Size of the blocked gate-sample POINTS.
constexpr double kGatePointSize = 0.06;
/// Thickness of K's cells.
constexpr double kCellHeight = 0.02;
/// Points of one `field_grid` streamline (step `field_grid_resolution`/2).
constexpr std::size_t kStreamlinePoints = 8;
/// Grid nodes per side are capped at this (a misconfigured resolution cannot explode the grid).
constexpr int kMaxGridSide = 100;
/// `obstacle_distance` scale beyond \f$D'_{\mathrm{in}}\f$ (plan §4.1).
constexpr double kDistanceMargin = 0.5;
/// @}

/// @brief The marker type of every (namespace, id), in legend order.
constexpr std::array<std::int32_t, legend::kMarkerCount> kMarkerTypes{{
  // status
  Marker::TEXT_VIEW_FACING,
  // footprint
  Marker::LINE_STRIP,
  // curve
  Marker::LINE_STRIP,
  Marker::LINE_LIST,
  Marker::SPHERE,
  // closest_point
  Marker::SPHERE,
  Marker::ARROW,
  Marker::LINE_LIST,
  // control_point
  Marker::SPHERE,
  Marker::LINE_LIST,
  // field
  Marker::ARROW,
  Marker::ARROW,
  Marker::ARROW,
  Marker::ARROW,
  // command
  Marker::ARROW,
  Marker::LINE_STRIP,
  // heading
  Marker::ARROW,
  Marker::LINE_STRIP,
  // prediction
  Marker::LINE_STRIP,
  Marker::CUBE,
  Marker::TEXT_VIEW_FACING,
  // goal
  Marker::SPHERE,
  Marker::SPHERE,
  Marker::CYLINDER,
  Marker::CYLINDER,
  Marker::ARROW,
  Marker::LINE_STRIP,
  // contour
  Marker::CUBE_LIST,
  Marker::POINTS,
  Marker::SPHERE,
  Marker::LINE_LIST,
  Marker::CYLINDER,
  Marker::CYLINDER,
  Marker::CYLINDER,
  Marker::ARROW,
  Marker::TEXT_VIEW_FACING,
  // curve_info
  Marker::LINE_LIST,
  Marker::SPHERE,
  Marker::TEXT_VIEW_FACING,
  Marker::ARROW,
}};

/// @brief Index of the first marker of each namespace in `<ns>/markers`.
constexpr std::array<std::size_t, legend::kNamespaceCount> namespaceOffsets() noexcept
{
  std::array<std::size_t, legend::kNamespaceCount> offsets{};
  std::size_t next = 0;
  for (std::size_t i = 0; i < legend::kNamespaceCount; ++i) {
    offsets[i] = next;
    next += static_cast<std::size_t>(legend::kNamespaces[i].id_count);
  }
  return offsets;
}

constexpr std::array<std::size_t, legend::kNamespaceCount> kOffsets = namespaceOffsets();

static_assert(
  kOffsets.back() + static_cast<std::size_t>(legend::kNamespaces.back().id_count) ==
    legend::kMarkerCount,
  "kOffsets does not cover the legend");

/// @brief A fixed-capacity text filled with snprintf (no allocation).
class Text
{
public:
  char * tail() noexcept { return data_.data() + size_; }
  std::size_t room() const noexcept { return data_.size() - size_; }
  /// @brief Accounts for the return value of one snprintf at tail() (truncation-safe).
  void advance(int written) noexcept
  {
    if (written > 0) {
      size_ = std::min(size_ + static_cast<std::size_t>(written), data_.size() - 1);
    }
  }
  void clear() noexcept
  {
    size_ = 0;
    data_[0] = '\0';
  }
  std::string_view view() const noexcept { return std::string_view(data_.data(), size_); }

private:
  std::array<char, 256> data_{};
  std::size_t size_{0};
};

std_msgs::msg::ColorRGBA toColor(const legend::Rgba & color) noexcept
{
  std_msgs::msg::ColorRGBA out;
  out.r = color.r;
  out.g = color.g;
  out.b = color.b;
  out.a = color.a;
  return out;
}

/// @brief The colour of a CYLINDER ring: the legend RGB with legend::kRingAlpha.
constexpr legend::Rgba ringColor(const legend::Rgba & color) noexcept
{
  return legend::Rgba{color.r, color.g, color.b, legend::kRingAlpha};
}

geometry_msgs::msg::Point toPoint(const Vec2 & v, double z = 0.0) noexcept
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = z;
  return p;
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw) noexcept
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

/// @brief Unit vector of a heading.
Vec2 heading(double yaw) noexcept { return Vec2(std::cos(yaw), std::sin(yaw)); }

/// @brief Turns a marker back into a bare DELETE (no geometry, identity pose).
void drop(Marker * m) noexcept
{
  m->action = Marker::DELETE;
  m->pose = geometry_msgs::msg::Pose();
  m->scale = geometry_msgs::msg::Vector3();
  m->color = std_msgs::msg::ColorRGBA();
  m->points.clear();
  m->colors.clear();
  m->text.clear();
}

/// @brief Header of the cycle, then drop().
void reset(Marker * m, const builtin_interfaces::msg::Time & stamp, std::string_view frame_id)
{
  m->header.stamp = stamp;
  m->header.frame_id.assign(frame_id.data(), frame_id.size());
  drop(m);
}

bool isFinite(const geometry_msgs::msg::Point & p) noexcept
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/// @brief True iff every coordinate of the marker (pose, scale, points) is finite.
bool isFinite(const Marker & m) noexcept
{
  const auto & q = m.pose.orientation;
  if (
    !isFinite(m.pose.position) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w) || !std::isfinite(m.scale.x) ||
    !std::isfinite(m.scale.y) || !std::isfinite(m.scale.z)) {
    return false;
  }
  return std::all_of(m.points.begin(), m.points.end(), [](const geometry_msgs::msg::Point & p) {
    return isFinite(p);
  });
}

/// @brief Finite and longer than kMinVector.
bool drawable(const Vec2 & v) noexcept { return vf::isFinite(v) && v.norm() > kMinVector; }

/// @brief Writes one cycle into the preallocated `<ns>/markers` array.
class MarkerWriter
{
public:
  MarkerWriter(MarkerArray * array, const MarkerContext & context, std::vector<Vec2> * points)
  : array_(*array),
    c_(context),
    in_(*context.input),
    out_(*context.output),
    f_(context.output->field),
    config_(context.tracker->config()),
    points_(*points)
  {
  }

  void write()
  {
    const std::size_t curve_points = sampleCurve(c_, &points_);
    status();
    footprint();
    curve(curve_points);
    closestPoint();
    controlPoint();
    field();
    command();
    headingTarget();
    prediction();
    goal();
    contour();
    curveInfo();
  }

  /// @brief The curve the field uses: the window, or `input->curve` sampled and mapped to the
  /// evaluation frame. Returns the number of points written to `*points`.
  static std::size_t sampleCurve(const MarkerContext & c, std::vector<Vec2> * points)
  {
    if (!c.window.empty()) {
      points->assign(c.window.begin(), c.window.end());
      return points->size();
    }
    const vf::Curve * curve = c.input->curve;
    if (curve == nullptr) {
      points->clear();
      return 0;
    }
    const double t = c.input->t;
    const double length = curve->length(t);
    if (!(length > 0.0) || !std::isfinite(length)) {
      points->clear();
      return 0;
    }
    const double ds = std::max(kMinCurveSpacing, length / static_cast<double>(kMaxCurvePoints - 2));
    points->resize(kMaxCurvePoints);
    const std::size_t n =
      curve->sampleStations(0.0, length, ds, t, vf::Span<Vec2>(points->data(), points->size()));
    points->resize(n);
    const vf::Isometry2 & frame = c.input->frame.T_eval_curve;
    for (Vec2 & p : *points) {
      p = frame.apply(p);
    }
    return n;
  }

private:
  Marker & add(Ns ns, std::int32_t id, const legend::Rgba & color)
  {
    Marker & m =
      array_.markers[kOffsets[static_cast<std::size_t>(ns)] + static_cast<std::size_t>(id)];
    m.action = Marker::ADD;
    m.color = toColor(color);
    m.pose.orientation.w = 1.0;
    return m;
  }

  static void line(Marker * m, double width) noexcept
  {
    m->scale.x = width;
    m->scale.y = 0.0;
    m->scale.z = 0.0;
  }

  static void sphere(Marker * m, const Vec2 & at, double diameter, double z = 0.0) noexcept
  {
    m->pose.position = toPoint(at, z);
    m->scale.x = diameter;
    m->scale.y = diameter;
    m->scale.z = diameter;
  }

  static void arrow(
    Marker * m, const Vec2 & from, const Vec2 & to, double shaft = kArrowShaft,
    double head = kArrowHead)
  {
    m->points.push_back(toPoint(from));
    m->points.push_back(toPoint(to));
    m->scale.x = shaft;
    m->scale.y = head;
    m->scale.z = kArrowHeadLength;
  }

  static void ring(Marker * m, const Vec2 & centre, double radius) noexcept
  {
    m->pose.position = toPoint(centre);
    m->scale.x = 2.0 * radius;
    m->scale.y = 2.0 * radius;
    m->scale.z = kRingHeight;
  }

  static void text(Marker * m, const Vec2 & at, double z, std::string_view value)
  {
    m->pose.position = toPoint(at, z);
    m->scale.x = 0.0;
    m->scale.y = 0.0;
    m->scale.z = kTextHeight;
    m->text.assign(value.data(), value.size());
  }

  static void segment(Marker * m, const Vec2 & a, const Vec2 & b)
  {
    m->points.push_back(toPoint(a));
    m->points.push_back(toPoint(b));
  }

  void status()
  {
    const std::uint32_t bits = out_.status | f_.status;
    text_.clear();
    text_.advance(
      std::snprintf(
        text_.tail(), text_.room(), "%s | v %.2f w %.2f", vf::phaseName(out_.phase), out_.cmd.vx,
        out_.cmd.wz));
    if (std::abs(out_.cmd.vy) > kMinVector) {
      text_.advance(std::snprintf(text_.tail(), text_.room(), " vy %.2f", out_.cmd.vy));
    }
    if (f_.valid && std::isfinite(f_.D)) {
      text_.advance(std::snprintf(text_.tail(), text_.room(), " | D %.2f m", f_.D));
    }
    if (std::isfinite(f_.sigma)) {
      text_.advance(std::snprintf(text_.tail(), text_.room(), " | sigma %.2f m", f_.sigma));
    }
    bool first_flag = true;
    const auto flag = [this, &first_flag](std::string_view name) {
      if (first_flag) {
        text_.advance(std::snprintf(text_.tail(), text_.room(), " |"));
        first_flag = false;
      }
      text_.advance(
        std::snprintf(
          text_.tail(), text_.room(), " %.*s", static_cast<int>(name.size()), name.data()));
    };
    if (vf::Status::has(bits, vf::Status::kBlocked)) {
      flag(legend::kFlagBlocked);
    }
    if (f_.contour.active) {
      flag(legend::kFlagContour);
      text_.advance(
        std::snprintf(text_.tail(), text_.room(), " %+d", static_cast<int>(f_.contour.sigma_c)));
    }
    if (f_.cbf.active) {
      flag(legend::kFlagCbf);
    }
    if (vf::Status::has(bits, vf::Status::kAssumption3Clamped)) {
      flag(legend::kFlagAssumption3);
    }
    if (vf::Status::has(bits, vf::Status::kCurvatureSaturated)) {
      flag(legend::kFlagCurvature);
    }
    if (in_.cancel) {
      flag(legend::kFlagCancel);
    }
    if (in_.drive_sign < 0) {
      flag(legend::kFlagReverse);
    }
    Marker & m = add(Ns::kStatus, legend::id::kStatusText, legend::phaseColor(out_.phase));
    text(&m, in_.pose.p, legend::kStatusTextHeight, text_.view());
  }

  void footprint()
  {
    legend::Rgba color = legend::kFootprintNormal;
    if (vf::Status::has(out_.status, vf::Status::kBlocked)) {
      color = legend::kFootprintBlocked;
    } else if (out_.shaping.collisionAhead()) {
      color = legend::kFootprintCapped;
    }
    Marker & m = add(Ns::kFootprint, legend::id::kFootprintOutline, color);
    line(&m, kThickLineWidth);
    const vf::Isometry2 pose = vf::Isometry2::fromPose(in_.pose);
    const vf::Footprint & shape = c_.tracker->footprint();
    const vf::Span<const Vec2> vertices = shape.vertices();
    if (!shape.isCircle() && vertices.size() >= 3) {
      for (const Vec2 & v : vertices) {
        m.points.push_back(toPoint(pose.apply(v)));
      }
      m.points.push_back(m.points.front());
      return;
    }
    const double radius = std::max(shape.radius(), kPointRobotRadius);
    for (int k = 0; k <= kCircleSegments; ++k) {
      const double a = vf::kTwoPi * k / kCircleSegments;
      m.points.push_back(toPoint(pose.apply(Vec2(radius * std::cos(a), radius * std::sin(a)))));
    }
  }

  /// @brief Direction of the extension ray at one end of `points_` (outward), if any.
  bool rayDirection(std::size_t n, bool at_start, Vec2 * direction) const
  {
    for (std::size_t k = 1; k < n; ++k) {
      const Vec2 d =
        at_start ? Vec2(points_[0] - points_[k]) : Vec2(points_[n - 1] - points_[n - 1 - k]);
      const double norm = d.norm();
      if (norm > 1.0e-9 && std::isfinite(norm)) {
        *direction = d / norm;
        return true;
      }
    }
    return false;
  }

  void curve(std::size_t n)
  {
    if (n >= 2) {
      Marker & m = add(Ns::kCurve, legend::id::kCurveLine, legend::kCurveColor);
      line(&m, kLineWidth);
      for (std::size_t k = 0; k < n; ++k) {
        m.points.push_back(toPoint(points_[k]));
      }
      if (in_.curve != nullptr && !in_.curve->isClosed()) {
        Marker & rays = add(Ns::kCurve, legend::id::kCurveRays, legend::kCurveRays);
        line(&rays, kLineWidth);
        const std::array<std::pair<bool, std::size_t>, 2> ends{{{true, 0}, {false, n - 1}}};
        for (const auto & end : ends) {
          Vec2 d;
          if (!rayDirection(n, end.first, &d)) {
            continue;
          }
          for (double s = 0.0; s + kDashLength <= kRayLength + 1.0e-9; s += 2.0 * kDashLength) {
            segment(
              &rays, points_[end.second] + s * d, points_[end.second] + (s + kDashLength) * d);
          }
        }
        if (rays.points.empty()) {
          drop(&rays);
        }
      }
    }
    if (
      !c_.window.empty() && c_.virtual_vertex >= 0 &&
      static_cast<std::size_t>(c_.virtual_vertex) < c_.window.size()) {
      Marker & m = add(Ns::kCurve, legend::id::kCurveVirtualVertex, legend::kCurveRays);
      sphere(&m, c_.window[static_cast<std::size_t>(c_.virtual_vertex)], kSphereSmall);
    }
  }

  void closestPoint()
  {
    if (!f_.valid) {
      return;
    }
    Marker & point = add(Ns::kClosestPoint, legend::id::kClosestPoint, legend::kClosestPointColor);
    sphere(&point, f_.x_star, kSphereLarge);
    if (drawable(f_.T)) {
      Marker & tangent =
        add(Ns::kClosestPoint, legend::id::kClosestTangent, legend::kClosestPointColor);
      arrow(&tangent, f_.x_star, f_.x_star + kTangentLength * f_.T.normalized());
    }
    if (drawable(f_.x_star - f_.x_eval)) {
      Marker & distance =
        add(Ns::kClosestPoint, legend::id::kClosestDistance, legend::kClosestPointColor);
      line(&distance, kLineWidth);
      segment(&distance, f_.x_eval, f_.x_star);
    }
  }

  void controlPoint()
  {
    Marker & point = add(Ns::kControlPoint, legend::id::kControlPoint, legend::kControlPointColor);
    sphere(&point, f_.x_eval, kSphereSmall);
    if (drawable(f_.x_eval - in_.pose.p)) {
      Marker & link = add(Ns::kControlPoint, legend::id::kControlLine, legend::kControlPointColor);
      line(&link, kLineWidth);
      segment(&link, in_.pose.p, f_.x_eval);
    }
  }

  void field()
  {
    if (!f_.valid) {
      return;
    }
    const std::array<std::pair<const Vec2 *, std::int32_t>, 4> terms{{
      {&f_.conv, legend::id::kFieldConvergence},
      {&f_.circ, legend::id::kFieldCirculation},
      {&f_.Phi_T, legend::id::kFieldFeedforward},
      {&f_.F, legend::id::kFieldResult},
    }};
    const std::array<legend::Rgba, 4> colors{
      legend::kFieldConvergence, legend::kFieldCirculation, legend::kFieldFeedforward,
      legend::kFieldResult};
    for (std::size_t k = 0; k < terms.size(); ++k) {
      const Vec2 & v = *terms[k].first;
      if (!drawable(v)) {
        continue;
      }
      Marker & m = add(Ns::kField, terms[k].second, colors[k]);
      const bool thick = terms[k].second == legend::id::kFieldResult;
      arrow(
        &m, f_.x_eval, f_.x_eval + kVelocityHorizon * v, thick ? kThickArrowShaft : kArrowShaft,
        thick ? kThickArrowHead : kArrowHead);
    }
  }

  void command()
  {
    const Vec2 body(out_.cmd.vx, out_.cmd.vy);
    if (!drawable(body) || !std::isfinite(out_.cmd.wz)) {
      return;  // a pure rotation is shown by `heading`
    }
    const vf::Isometry2 pose = vf::Isometry2::fromPose(in_.pose);
    Marker & m = add(Ns::kCommand, legend::id::kCommandArrow, legend::kCommandColor);
    arrow(&m, in_.pose.p, in_.pose.p + kVelocityHorizon * pose.rotate(body));
    Marker & arc = add(Ns::kCommand, legend::id::kCommandArc, legend::kCommandColor);
    line(&arc, kLineWidth);
    const vf::Kinematics & kinematics = c_.tracker->kinematics();
    for (int k = 0; k <= kCommandArcSteps; ++k) {
      const double dt = kVelocityHorizon * k / kCommandArcSteps;
      arc.points.push_back(toPoint(vf::sim::integrate(kinematics, in_.pose, out_.cmd, dt).p));
    }
  }

  void headingTarget()
  {
    const bool to_field = out_.phase == vf::Phase::kRotateToField;
    const bool to_goal = out_.phase == vf::Phase::kRotateToHeading;
    if (
      (!to_field && !to_goal) || !f_.has_target_heading || !std::isfinite(f_.target_heading) ||
      !std::isfinite(in_.pose.yaw)) {
      return;
    }
    const legend::Rgba color = to_field ? legend::kHeadingToField : legend::kHeadingToGoal;
    Marker & target = add(Ns::kHeading, legend::id::kHeadingTarget, color);
    arrow(&target, in_.pose.p, in_.pose.p + kHeadingLength * heading(f_.target_heading));
    const double error = vf::wrapAngle(f_.target_heading - in_.pose.yaw);
    if (std::abs(error) > 1.0e-3) {
      Marker & arc = add(Ns::kHeading, legend::id::kHeadingArc, color);
      line(&arc, kLineWidth);
      for (int k = 0; k <= kHeadingArcSteps; ++k) {
        const double a = in_.pose.yaw + error * k / kHeadingArcSteps;
        arc.points.push_back(toPoint(in_.pose.p + kHeadingArcRadius * heading(a)));
      }
    }
  }

  void prediction()
  {
    const vf::Span<const vf::Pose2> poses = out_.predicted;
    if (poses.empty()) {
      return;
    }
    const std::size_t collision =
      out_.first_collision >= 0 && static_cast<std::size_t>(out_.first_collision) < poses.size()
        ? static_cast<std::size_t>(out_.first_collision)
        : poses.size();
    if (poses.size() >= 2) {
      Marker & path = add(Ns::kPrediction, legend::id::kPredictionPath, legend::kPredictionFree);
      line(&path, kThickLineWidth);
      const std_msgs::msg::ColorRGBA free_color = toColor(legend::kPredictionFree);
      const std_msgs::msg::ColorRGBA hit_color = toColor(legend::kPredictionCollision);
      for (std::size_t k = 0; k < poses.size(); ++k) {
        path.points.push_back(toPoint(poses[k].p));
        path.colors.push_back(k < collision ? free_color : hit_color);
      }
    }
    if (collision < poses.size()) {
      Marker & cube =
        add(Ns::kPrediction, legend::id::kPredictionCollision, legend::kPredictionCollision);
      sphere(&cube, poses[collision].p, kCollisionCube);
      cube.pose.orientation = yawQuaternion(poses[collision].yaw);
    }
    if (std::isfinite(out_.s_free)) {
      text_.clear();
      text_.advance(std::snprintf(text_.tail(), text_.room(), "s_free %.2f m", out_.s_free));
      Marker & label = add(Ns::kPrediction, legend::id::kPredictionText, legend::kPredictionFree);
      const std::size_t anchor = collision < poses.size() ? collision : poses.size() - 1;
      text(&label, poses[anchor].p, kTextLift, text_.view());
    }
  }

  /// @brief g, the rings and the goal yaw at the goal pose; g_ref and the braking zone only in
  /// terminal mode (the curve ends at the goal), since \f$s_{\mathrm{goal}}\f$ is the curve end:
  /// a window that stops short of the goal would put them mid-path.
  void goal()
  {
    if (!in_.goal.has_value()) {
      return;
    }
    const vf::GoalInfo & goal = *in_.goal;
    const Vec2 & g = goal.pose.p;
    Marker & point = add(Ns::kGoal, legend::id::kGoalPoint, legend::kGoalColor);
    sphere(&point, g, kSphereLarge);
    if (f_.terminal && std::abs(f_.x_ref) > 1.0e-9 && vf::isFinite(f_.g_ref)) {
      Marker & ref = add(Ns::kGoal, legend::id::kGoalReference, legend::kGoalColor);
      sphere(&ref, f_.g_ref, kSphereSmall);
    }
    if (goal.xy_tol > 0.0) {
      Marker & tol =
        add(Ns::kGoal, legend::id::kGoalToleranceRing, ringColor(legend::kGoalTolerance));
      ring(&tol, g, goal.xy_tol);
      Marker & stop = add(Ns::kGoal, legend::id::kGoalStopRing, ringColor(legend::kGoalTolerance));
      ring(&stop, g, 0.5 * goal.xy_tol);
    }
    if (goal.has_yaw && goal.yaw_tol > 1.0e-6 && std::isfinite(goal.pose.yaw)) {
      Marker & yaw = add(Ns::kGoal, legend::id::kGoalYaw, legend::kGoalTolerance);
      arrow(&yaw, g, g + kGoalYawLength * heading(goal.pose.yaw));
    }
    brakingZone();
  }

  /// @brief The last \f$\sigma_b\f$ of the curve before the goal station, against the traversal
  /// direction (terminal mode only).
  void brakingZone()
  {
    const double sigma_b = config_.phases.terminal_braking_distance;
    if (!f_.terminal || in_.curve == nullptr || !std::isfinite(f_.s_goal) || !(sigma_b > 0.0)) {
      return;
    }
    const double direction = config_.field.direction < 0 ? -1.0 : 1.0;
    const double s0 = std::min(f_.s_goal, f_.s_goal - direction * sigma_b);
    const double s1 = std::max(f_.s_goal, f_.s_goal - direction * sigma_b);
    std::array<Vec2, kBrakingSamples + 1> samples;
    const std::size_t n = in_.curve->sampleStations(
      s0, s1, (s1 - s0) / static_cast<double>(kBrakingSamples - 1), in_.t,
      vf::Span<Vec2>(samples.data(), samples.size()));
    if (n < 2) {
      return;
    }
    Marker & m = add(Ns::kGoal, legend::id::kGoalBrakingZone, legend::kGoalBrakingZone);
    line(&m, kThickLineWidth);
    for (std::size_t k = 0; k < n; ++k) {
      m.points.push_back(toPoint(in_.frame.T_eval_curve.apply(samples[k])));
    }
  }

  void contour()
  {
    if (config_.obstacles.obstacle_mode == vf::ObstacleMode::kCbf) {
      cbf();
      return;
    }
    const vf::ContourSample & cs = f_.contour;
    if (!cs.gate_open && !cs.active) {
      return;
    }
    contourCells();
    const vf::StepScratch & scratch = *c_.scratch;
    const std::size_t samples = std::min(scratch.gate_samples.size(), scratch.gate_blocked.size());
    Marker & gate = add(Ns::kContour, legend::id::kContourGateSamples, legend::kContourObstacle);
    gate.scale.x = kGatePointSize;
    gate.scale.y = kGatePointSize;
    for (std::size_t k = 0; k < samples; ++k) {
      if (scratch.gate_blocked[k] != 0U) {
        gate.points.push_back(toPoint(scratch.gate_samples[k]));
      }
    }
    if (gate.points.empty()) {
      drop(&gate);
    }
    const bool has_obstacle = std::isfinite(cs.D_o) && vf::isFinite(cs.x_o);
    if (has_obstacle) {
      Marker & point =
        add(Ns::kContour, legend::id::kContourObstaclePoint, legend::kContourObstacle);
      sphere(&point, cs.x_o, kSphereLarge);
      Marker & link = add(Ns::kContour, legend::id::kContourObstacleLine, legend::kContourObstacle);
      line(&link, kLineWidth);
      segment(&link, f_.x_eval, cs.x_o);
      const vf::ObstacleConfig & obstacles = config_.obstacles;
      const std::array<std::pair<std::int32_t, double>, 3> rings{{
        {legend::id::kContourLambdaRing, obstacles.obstacle_contour_distance},
        {legend::id::kContourFullSwitchRing, obstacles.obstacle_full_switch_distance},
        {legend::id::kContourBlendStartRing, obstacles.obstacle_blend_start_distance},
      }};
      for (const auto & r : rings) {
        if (r.second > 0.0) {
          const legend::Rgba color = r.first == legend::id::kContourLambdaRing
                                       ? legend::kContourLambda
                                       : legend::kContourBlend;
          Marker & m = add(Ns::kContour, r.first, ringColor(color));
          ring(&m, cs.x_o, r.second);
        }
      }
    }
    if (cs.sigma_c != 0 && drawable(cs.n_o)) {
      const Vec2 side = static_cast<double>(cs.sigma_c) * vf::J(cs.n_o.normalized());
      Marker & m = add(Ns::kContour, legend::id::kContourSide, legend::kContourLambda);
      arrow(&m, f_.x_eval, f_.x_eval + kSideArrowLength * side);
    }
    text_.clear();
    if (cs.active) {
      text_.advance(
        std::snprintf(
          text_.tail(), text_.room(), "contour %+d | %.1f/%.1f m | %.0f/%.0f s",
          static_cast<int>(cs.sigma_c), cs.travelled_m,
          config_.obstacles.obstacle_max_contour_length, cs.elapsed_s,
          config_.obstacles.obstacle_max_contour_time));
    } else {
      text_.advance(std::snprintf(text_.tail(), text_.room(), "contour gate open"));
    }
    Marker & label = add(Ns::kContour, legend::id::kContourText, legend::kContourObstacle);
    text(&label, has_obstacle ? cs.x_o : f_.x_eval, kTextLift, text_.view());
  }

  /// @brief `obstacle_mode: cbf` in the contour namespace (math_spec §6.7): the constraint sites
  /// (gate-sample points; the nearest one as the obstacle sphere), the normals of the binding
  /// constraints (site → evaluation point), the \f$\delta_{\mathrm{safe}}\f$ ring about the
  /// evaluation point (red while h < 0), the filter's correction \f$F^\star - F\f$ (side arrow) and
  /// a text with h, the binding count and \f$\lVert F^\star\rVert/\lVert F\rVert\f$. Drawn while the
  /// filter binds or the nearest site is within \f$2\delta_{\mathrm{safe}}\f$.
  void cbf()
  {
    const vf::CbfSample & cbf = f_.cbf;
    if (!cbf.on || cbf.count <= 0 || !(cbf.active || cbf.h < cbf.delta)) {
      return;
    }
    const auto count = static_cast<std::size_t>(std::min(cbf.count, vf::kCbfMaxSites));
    Marker & sites = add(Ns::kContour, legend::id::kContourGateSamples, legend::kContourObstacle);
    sites.scale.x = kGatePointSize;
    sites.scale.y = kGatePointSize;
    Marker & normals =
      add(Ns::kContour, legend::id::kContourObstacleLine, legend::kContourObstacle);
    line(&normals, kLineWidth);
    int binding = 0;
    for (std::size_t j = 0; j < count; ++j) {
      sites.points.push_back(toPoint(cbf.x_o[j]));
      if (cbf.binds[j]) {
        segment(&normals, cbf.x_o[j], f_.x_eval);
        ++binding;
      }
    }
    if (normals.points.empty()) {
      drop(&normals);
    }
    Marker & nearest =
      add(Ns::kContour, legend::id::kContourObstaclePoint, legend::kContourObstacle);
    sphere(&nearest, cbf.x_o[0], kSphereLarge);
    Marker & disc = add(
      Ns::kContour, legend::id::kContourLambdaRing,
      ringColor(cbf.h < 0.0 ? legend::kContourObstacle : legend::kContourLambda));
    ring(&disc, f_.x_eval, cbf.delta);
    const Vec2 correction = f_.F - cbf.F_in;
    if (cbf.active && drawable(correction)) {
      Marker & m = add(Ns::kContour, legend::id::kContourSide, legend::kContourLambda);
      arrow(&m, f_.x_eval, f_.x_eval + kSideArrowLength * correction.normalized());
    }
    text_.clear();
    text_.advance(
      std::snprintf(
        text_.tail(), text_.room(), "cbf h %.2f m | %d/%zu bind | %.0f %%%s", cbf.h, binding, count,
        100.0 * cbf.speed_ratio, cbf.hold ? " | hold" : ""));
    Marker & label = add(Ns::kContour, legend::id::kContourText, legend::kContourObstacle);
    text(&label, cbf.x_o[0], kTextLift, text_.view());
  }

  /// @brief K's cells, drawn in the grid's orientation: the marker pose carries the grid
  /// rotation and the points are the cell centres relative to the evaluation origin.
  void contourCells()
  {
    const vf::StepScratch & scratch = *c_.scratch;
    if (!scratch.k_valid || !in_.obstacles.has_value() || in_.obstacles->grid == nullptr) {
      return;
    }
    const vf::OccupancyGrid & grid = *in_.obstacles->grid;
    const int width = scratch.k_width;
    const int height = scratch.k_height;
    if (
      width != grid.width() || height != grid.height() || width <= 0 || height <= 0 ||
      scratch.k_mask.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
      return;
    }
    const vf::Isometry2 & T_grid_eval = in_.obstacles->T_grid_eval;
    Marker & m = add(Ns::kContour, legend::id::kContourCells, legend::kContourCells);
    m.pose.orientation = yawQuaternion(-T_grid_eval.yaw());
    m.scale.x = grid.resolution();
    m.scale.y = grid.resolution();
    m.scale.z = kCellHeight;
    for (int iy = 0; iy < height; ++iy) {
      for (int ix = 0; ix < width; ++ix) {
        if (
          scratch.k_mask
            [static_cast<std::size_t>(iy) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(ix)] != 0U) {
          m.points.push_back(toPoint(grid.cellCenter(ix, iy) - T_grid_eval.translation()));
        }
      }
    }
    if (m.points.empty()) {
      drop(&m);
    }
  }

  void curveInfo()
  {
    if (c_.curve_info == nullptr || in_.curve == nullptr) {
      return;
    }
    const CurveInfo & info = *c_.curve_info;
    const vf::Curve & curve = *in_.curve;
    const double t = in_.t;
    const double length = curve.length(t);
    const vf::Isometry2 & frame = in_.frame.T_eval_curve;
    const double direction = info.direction < 0 ? -1.0 : 1.0;
    if (length > 0.0 && std::isfinite(length)) {
      Marker & chevrons =
        add(Ns::kCurveInfo, legend::id::kCurveInfoDirection, legend::kCurveInfoColor);
      line(&chevrons, kLineWidth);
      for (std::size_t k = 0; k < kMaxChevrons; ++k) {
        const double s = kChevronSpacing * (static_cast<double>(k) + 0.5);
        if (s > length) {
          break;
        }
        const Vec2 tip = frame.apply(curve.pointAt(s, t));
        const Vec2 d = direction * frame.rotate(curve.tangentAt(s, t));
        if (!drawable(d) || !vf::isFinite(tip)) {
          continue;
        }
        const Vec2 u = d.normalized();
        const Vec2 back = tip - kChevronLength * u;
        segment(&chevrons, tip, back + kChevronHalfWidth * vf::J(u));
        segment(&chevrons, tip, back - kChevronHalfWidth * vf::J(u));
      }
      if (chevrons.points.empty()) {
        drop(&chevrons);
      }
    }
    const double s_start = !curve.isClosed() && direction < 0.0 ? length : 0.0;
    const Vec2 start = frame.apply(curve.pointAt(std::isfinite(s_start) ? s_start : 0.0, t));
    Marker & sphere_marker =
      add(Ns::kCurveInfo, legend::id::kCurveInfoStart, legend::kCurveInfoColor);
    sphere(&sphere_marker, start, kSphereLarge);
    text_.clear();
    if (info.laps > 0U) {
      text_.advance(
        std::snprintf(
          text_.tail(), text_.room(), "lap %u/%u", static_cast<unsigned>(info.laps_completed),
          static_cast<unsigned>(info.laps)));
    } else {
      text_.advance(
        std::snprintf(
          text_.tail(), text_.room(), "lap %u", static_cast<unsigned>(info.laps_completed)));
    }
    if (std::isfinite(info.remaining_arc_length)) {
      text_.advance(
        std::snprintf(text_.tail(), text_.room(), " | rem %.1f m", info.remaining_arc_length));
    }
    Marker & label = add(Ns::kCurveInfo, legend::id::kCurveInfoText, legend::kCurveInfoColor);
    text(&label, start, kTextLift, text_.view());
    if (info.frame_moving && drawable(info.frame_velocity) && vf::isFinite(info.frame_origin)) {
      Marker & velocity =
        add(Ns::kCurveInfo, legend::id::kCurveInfoFrameVelocity, legend::kCurveInfoFrame);
      arrow(
        &velocity, info.frame_origin, info.frame_origin + kVelocityHorizon * info.frame_velocity);
    }
  }

  MarkerArray & array_;
  const MarkerContext & c_;
  const vf::TrackerInput & in_;
  const vf::TrackerOutput & out_;
  const vf::FieldSample & f_;
  const vf::CoreConfig & config_;
  std::vector<Vec2> & points_;
  Text text_;
};

/// @brief True iff every required pointer of the context is set.
bool complete(const MarkerContext & c) noexcept
{
  return c.tracker != nullptr && c.input != nullptr && c.output != nullptr && c.state != nullptr &&
         c.scratch != nullptr;
}

/// @brief Viridis colour map at \f$u \in [0, 1]\f$ (11 stops, linear in between).
std_msgs::msg::ColorRGBA viridis(double u) noexcept
{
  static constexpr std::array<std::array<float, 3>, 11> kStops{{
    {0.267004F, 0.004874F, 0.329415F},
    {0.282623F, 0.140926F, 0.457517F},
    {0.253935F, 0.265254F, 0.529983F},
    {0.206756F, 0.371758F, 0.553117F},
    {0.163625F, 0.471133F, 0.558148F},
    {0.127568F, 0.566949F, 0.550556F},
    {0.134692F, 0.658636F, 0.517649F},
    {0.266941F, 0.748751F, 0.440573F},
    {0.477504F, 0.821444F, 0.318195F},
    {0.741388F, 0.873449F, 0.149561F},
    {0.993248F, 0.906157F, 0.143936F},
  }};
  const double x = std::isfinite(u) ? std::clamp(u, 0.0, 1.0) * 10.0 : 10.0;
  const std::size_t k = std::min<std::size_t>(static_cast<std::size_t>(x), 9);
  const float w = static_cast<float>(x - static_cast<double>(k));
  std_msgs::msg::ColorRGBA color;
  color.r = kStops[k][0] + w * (kStops[k + 1][0] - kStops[k][0]);
  color.g = kStops[k][1] + w * (kStops[k + 1][1] - kStops[k][1]);
  color.b = kStops[k][2] + w * (kStops[k + 1][2] - kStops[k][2]);
  color.a = 1.0F;
  return color;
}

/// @brief Fills a Path with poses oriented along the local tangent of `points`.
void fillPath(nav_msgs::msg::Path * path, const std::vector<Vec2> & points, std::size_t n)
{
  path->poses.resize(n);
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t a = k > 0 ? k - 1 : 0;
    const std::size_t b = k + 1 < n ? k + 1 : n - 1;
    const Vec2 d = points[b] - points[a];
    const double yaw = d.squaredNorm() > 0.0 ? std::atan2(d.y(), d.x()) : 0.0;
    geometry_msgs::msg::PoseStamped & pose = path->poses[k];
    pose.header = path->header;
    pose.pose.position = toPoint(points[k]);
    pose.pose.orientation = yawQuaternion(yaw);
  }
}

}  // namespace

MarkerBuilder::MarkerBuilder()
{
  markers_.markers.resize(legend::kMarkerCount);
  std::size_t index = 0;
  for (const legend::NamespaceSpec & ns : legend::kNamespaces) {
    for (std::int32_t id = 0; id < ns.id_count; ++id, ++index) {
      Marker & m = markers_.markers[index];
      m.ns.assign(ns.name.data(), ns.name.size());
      m.id = id;
      m.type = kMarkerTypes[index];
      m.action = Marker::DELETE;
      m.pose.orientation.w = 1.0;
    }
  }
  delete_all_.markers.resize(1);
  delete_all_.markers.front().action = Marker::DELETEALL;
  delete_all_.markers.front().pose.orientation.w = 1.0;
  grid_.markers.reserve(1 + 12 * 12);
  points_.reserve(kMaxCurvePoints);
}

const visualization_msgs::msg::MarkerArray & MarkerBuilder::markers(const MarkerContext & context)
{
  for (Marker & m : markers_.markers) {
    reset(&m, context.stamp, context.frame_id);
  }
  if (!complete(context)) {
    return markers_;
  }
  MarkerWriter writer(&markers_, context, &points_);
  writer.write();
  for (Marker & m : markers_.markers) {
    if (m.action == Marker::ADD && !isFinite(m)) {
      drop(&m);
    }
  }
  return markers_;
}

const visualization_msgs::msg::MarkerArray & MarkerBuilder::fieldGrid(
  const MarkerContext & context, const VizConfig & config)
{
  std::size_t used = 0;
  const auto next = [this, &context, &used]() -> Marker & {
    if (used == grid_.markers.size()) {
      grid_.markers.emplace_back();
    }
    Marker & m = grid_.markers[used++];
    reset(&m, context.stamp, context.frame_id);
    return m;
  };
  constexpr std::string_view kGridNs = "field_grid";
  Marker & clear = next();
  clear.ns.assign(kGridNs.data(), kGridNs.size());
  clear.id = 0;
  clear.action = Marker::DELETEALL;
  clear.pose.orientation.w = 1.0;

  const double size = config.field_grid_size;
  const double res = config.field_grid_resolution;
  if (
    !complete(context) || !(size > 0.0) || !(res > 0.0) || !std::isfinite(size) ||
    !std::isfinite(res) || !vf::isFinite(context.input->pose.p)) {
    grid_.markers.resize(used);
    return grid_;
  }
  const vf::Tracker & tracker = *context.tracker;
  const vf::TrackerInput & input = *context.input;
  const vf::TrackerState & state = *context.state;
  const vf::StepScratch & scratch = *context.scratch;
  const int side = std::clamp(static_cast<int>(std::lround(size / res)), 1, kMaxGridSide);
  const double v_ref =
    tracker.config().field.desired_speed > 0.0 ? tracker.config().field.desired_speed : 1.0;
  const double d_scale = 0.5 * size;
  const bool streamlines = config.field_grid_style == FieldGridStyle::kStreamlines;
  const auto probe_field = [&](const Vec2 & x) -> Vec2 {
    return tracker.probe(x, input, state, scratch).F;
  };
  if (streamlines && points_.size() < kStreamlinePoints) {
    points_.resize(kStreamlinePoints);
  }
  std::int32_t id = 0;
  for (int j = 0; j < side; ++j) {
    for (int i = 0; i < side; ++i) {
      const Vec2 node = input.pose.p + res * Vec2(i + 0.5 - 0.5 * side, j + 0.5 - 0.5 * side);
      const vf::ProbeSample sample = tracker.probe(node, input, state, scratch);
      if (!vf::isFinite(sample.F) || !std::isfinite(sample.D)) {
        continue;
      }
      if (streamlines) {
        const std::size_t count = vf::streamline(
          probe_field, node, 0.5 * res, vf::Span<Vec2>(points_.data(), kStreamlinePoints));
        if (count < 2) {
          continue;
        }
        Marker & m = next();
        m.ns.assign(kGridNs.data(), kGridNs.size());
        m.id = id++;
        m.type = Marker::LINE_STRIP;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.08 * res;
        m.color = viridis(sample.D / d_scale);
        for (std::size_t k = 0; k < count; ++k) {
          m.points.push_back(toPoint(points_[k]));
        }
        continue;
      }
      const double norm = sample.F.norm();
      if (!(norm > 1.0e-9)) {
        continue;
      }
      const double length = 0.9 * res * std::min(1.0, norm / v_ref);
      const Vec2 half = 0.5 * length * sample.F / norm;
      Marker & m = next();
      m.ns.assign(kGridNs.data(), kGridNs.size());
      m.id = id++;
      m.type = Marker::ARROW;
      m.action = Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.08 * res;
      m.scale.y = 0.2 * res;
      m.scale.z = 0.25 * res;
      m.color = viridis(sample.D / d_scale);
      m.points.push_back(toPoint(node - half));
      m.points.push_back(toPoint(node + half));
    }
  }
  grid_.markers.resize(used);
  return grid_;
}

const nav_msgs::msg::Path & MarkerBuilder::window(const MarkerContext & context)
{
  window_.header.stamp = context.stamp;
  window_.header.frame_id.assign(context.frame_id.data(), context.frame_id.size());
  if (!complete(context)) {
    window_.poses.clear();
    return window_;
  }
  const std::size_t n = MarkerWriter::sampleCurve(context, &points_);
  fillPath(&window_, points_, n);
  return window_;
}

const nav_msgs::msg::Path & MarkerBuilder::predictedPath(const MarkerContext & context)
{
  predicted_.header.stamp = context.stamp;
  predicted_.header.frame_id.assign(context.frame_id.data(), context.frame_id.size());
  if (context.output == nullptr) {
    predicted_.poses.clear();
    return predicted_;
  }
  const vf::Span<const vf::Pose2> poses = context.output->predicted;
  predicted_.poses.resize(poses.size());
  for (std::size_t k = 0; k < poses.size(); ++k) {
    geometry_msgs::msg::PoseStamped & pose = predicted_.poses[k];
    pose.header = predicted_.header;
    pose.pose.position = toPoint(poses[k].p);
    pose.pose.orientation = yawQuaternion(poses[k].yaw);
  }
  return predicted_;
}

const nav_msgs::msg::OccupancyGrid & MarkerBuilder::obstacleDistance(const MarkerContext & context)
{
  distance_.header.stamp = context.stamp;
  distance_.header.frame_id.assign(context.frame_id.data(), context.frame_id.size());
  distance_.info.map_load_time = context.stamp;
  const bool has_edt = complete(context) && context.input->obstacles.has_value() &&
                       context.input->obstacles->edt != nullptr &&
                       context.input->obstacles->edt->hasSites();
  if (!has_edt) {
    distance_.info.width = 0;
    distance_.info.height = 0;
    distance_.info.resolution = 0.0F;
    distance_.info.origin = geometry_msgs::msg::Pose();
    distance_.data.clear();
    return distance_;
  }
  const vf::ObstacleMap & obstacles = *context.input->obstacles;
  const vf::EuclideanDistanceTransform & edt = *obstacles.edt;
  const int width = std::max(edt.width(), 0);
  const int height = std::max(edt.height(), 0);
  distance_.info.width = static_cast<std::uint32_t>(width);
  distance_.info.height = static_cast<std::uint32_t>(height);
  distance_.info.resolution = static_cast<float>(edt.resolution());
  // The grid frame maps to the evaluation frame through T_grid_eval⁻¹.
  distance_.info.origin.position = toPoint(obstacles.T_grid_eval.applyInverse(edt.origin()));
  distance_.info.origin.orientation = yawQuaternion(-obstacles.T_grid_eval.yaw());
  const double d_max =
    context.tracker->config().obstacles.obstacle_blend_start_distance + kDistanceMargin;
  distance_.data.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
  for (int iy = 0; iy < height; ++iy) {
    for (int ix = 0; ix < width; ++ix) {
      const double d = edt.distance(ix, iy);
      double value = 0.0;
      if (d_max > 0.0 && !(d >= d_max)) {
        value = std::round(100.0 * (1.0 - std::max(d, 0.0) / d_max));
      }
      distance_.data
        [static_cast<std::size_t>(iy) * static_cast<std::size_t>(width) +
         static_cast<std::size_t>(ix)] = static_cast<std::int8_t>(std::clamp(value, 0.0, 100.0));
    }
  }
  return distance_;
}

const visualization_msgs::msg::MarkerArray & MarkerBuilder::deleteAll(
  const builtin_interfaces::msg::Time & stamp, std::string_view frame_id)
{
  Marker & m = delete_all_.markers.front();
  m.header.stamp = stamp;
  m.header.frame_id.assign(frame_id.data(), frame_id.size());
  m.action = Marker::DELETEALL;
  return delete_all_;
}

}  // namespace vectorfield_ros
