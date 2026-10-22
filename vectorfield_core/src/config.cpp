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

#include "vectorfield_core/config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>

#include "vectorfield_core/gains.hpp"

namespace vectorfield_core
{

namespace
{

// ---------------------------------------------------------------------------------------------
// Enum <-> parameter string tables (plan §5.4 values; the first entry is the default).

template <typename Enum>
struct EnumName
{
  Enum value;
  const char * name;
};

constexpr std::array<EnumName<GainType>, 2> kGainTypeNames{{
  {GainType::kAtan, "atan"},
  {GainType::kRational, "rational"},
}};

constexpr std::array<EnumName<MotionModel>, 4> kMotionModelNames{{
  {MotionModel::kDifferential, "differential"},
  {MotionModel::kSkidSteer, "skid_steer"},
  {MotionModel::kOmnidirectional, "omnidirectional"},
  {MotionModel::kAckermann, "ackermann"},
}};

constexpr std::array<EnumName<TrackingLaw>, 4> kTrackingLawNames{{
  {TrackingLaw::kAuto, "auto"},
  {TrackingLaw::kFeedbackLinearization, "feedback_linearization"},
  {TrackingLaw::kHeadingTracking, "heading_tracking"},
  {TrackingLaw::kHolonomic, "holonomic"},
}};

constexpr std::array<EnumName<OmniHeadingPolicy>, 3> kOmniHeadingPolicyNames{{
  {OmniHeadingPolicy::kPathTangent, "path_tangent"},
  {OmniHeadingPolicy::kField, "field"},
  {OmniHeadingPolicy::kFixed, "fixed"},
}};

constexpr std::array<EnumName<ObstacleMode>, 3> kObstacleModeNames{{
  {ObstacleMode::kNone, "none"},
  {ObstacleMode::kContour, "contour"},
  {ObstacleMode::kCbf, "cbf"},
}};

constexpr std::array<EnumName<ObstacleActivation>, 2> kObstacleActivationNames{{
  {ObstacleActivation::kPathBlocked, "path_blocked"},
  {ObstacleActivation::kAlways, "always"},
}};

template <typename Enum, std::size_t N>
const char * nameOf(const std::array<EnumName<Enum>, N> & table, Enum value) noexcept
{
  for (const auto & entry : table) {
    if (entry.value == value) {
      return entry.name;
    }
  }
  return "unknown";
}

template <typename Enum, std::size_t N>
bool valueOf(
  const std::array<EnumName<Enum>, N> & table, std::string_view text, Enum * out) noexcept
{
  for (const auto & entry : table) {
    if (text == entry.name) {
      if (out != nullptr) {
        *out = entry.value;
      }
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------------------------
// Message helpers.

std::string num(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.6g", value);
  return buffer;
}

void addIssue(std::vector<ValidationIssue> * list, std::string_view code, std::string message)
{
  list->push_back(ValidationIssue{std::string(code), std::move(message)});
}

bool finite(double value) noexcept { return std::isfinite(value); }

// Domain checks of single parameters (issue::kInvalidValue). Each check appends at most one
// error naming the parameter.
class DomainChecker
{
public:
  explicit DomainChecker(ValidationReport * report) : report_(report) {}

  void finiteValue(const char * name, double value)
  {
    if (!finite(value)) {
      fail(name, value, "must be finite");
    }
  }

  void positive(const char * name, double value)
  {
    if (!finite(value) || !(value > 0.0)) {
      fail(name, value, "must be finite and > 0");
    }
  }

  void nonNegative(const char * name, double value)
  {
    if (!finite(value) || !(value >= 0.0)) {
      fail(name, value, "must be finite and >= 0");
    }
  }

  // Auto-capable distance: negative selects auto, otherwise finite and > 0.
  void autoOrPositive(const char * name, double value)
  {
    if (std::isnan(value) || (!isAuto(value) && (!finite(value) || !(value > 0.0)))) {
      fail(name, value, "must be negative (auto) or finite and > 0");
    }
  }

  // Auto-capable distance: negative selects auto, otherwise finite and >= 0.
  void autoOrNonNegative(const char * name, double value)
  {
    if (std::isnan(value) || (!isAuto(value) && !finite(value))) {
      fail(name, value, "must be negative (auto) or finite and >= 0");
    }
  }

  void openUnitInterval(const char * name, double value)
  {
    if (!finite(value) || !(value > 0.0) || !(value < 1.0)) {
      fail(name, value, "must be in (0, 1)");
    }
  }

  void halfOpenUnitInterval(const char * name, double value)
  {
    if (!finite(value) || !(value > 0.0) || !(value <= 1.0)) {
      fail(name, value, "must be in (0, 1]");
    }
  }

  void angle(const char * name, double value)
  {
    if (!finite(value) || !(value > 0.0) || !(value <= kPi)) {
      fail(name, value, "must be in (0, pi]");
    }
  }

  void require(bool condition, std::string message)
  {
    if (!condition) {
      addIssue(&report_->errors, issue::kInvalidValue, std::move(message));
    }
  }

private:
  void fail(const char * name, double value, const char * rule)
  {
    addIssue(
      &report_->errors, issue::kInvalidValue, std::string(name) + " = " + num(value) + " " + rule);
  }

  ValidationReport * report_;
};

void checkDomain(const CoreConfig & config, ValidationReport * report)
{
  DomainChecker check(report);

  const FieldConfig & field = config.field;
  check.positive("desired_speed", field.desired_speed);
  check.positive("convergence_gain", field.convergence_gain);
  check.require(
    field.direction == 1 || field.direction == -1,
    "direction = " + std::to_string(field.direction) + " must be +1 or -1");
  check.openUnitInterval("assumption3_margin", field.assumption3_margin);

  const MotionConfig & motion = config.motion;
  check.finiteValue("control_point_offset", motion.control_point_offset);
  check.nonNegative("heading_gain", motion.heading_gain);
  check.positive("max_linear_vel", motion.max_linear_vel);
  check.autoOrPositive("max_reverse_vel", motion.max_reverse_vel);
  check.nonNegative("max_lateral_vel", motion.max_lateral_vel);
  check.positive("max_angular_vel", motion.max_angular_vel);
  check.positive("max_linear_accel", motion.max_linear_accel);
  check.positive("max_linear_decel", motion.max_linear_decel);
  check.positive("max_angular_accel", motion.max_angular_accel);
  check.nonNegative("min_in_place_angular_vel", motion.min_in_place_angular_vel);
  // min_turning_radius <= 0 is a math_spec §11 rule for Ackermann; only finiteness here.
  check.finiteValue("min_turning_radius", motion.min_turning_radius);
  check.nonNegative("ackermann_min_speed", motion.ackermann_min_speed);
  check.finiteValue("reference_point_offset", motion.reference_point_offset);
  check.halfOpenUnitInterval("skid_yaw_rate_efficiency", motion.skid_yaw_rate_efficiency);
  check.finiteValue("skid_icr_offset", motion.skid_icr_offset);
  // The compensation scales omega by s / (s - x_ICR), s = q d: keep it within (2/3, 2).
  check.require(
    motion.motion_model != MotionModel::kSkidSteer || motion.skid_icr_offset == 0.0 ||
      std::abs(motion.skid_icr_offset) < 0.5 * std::abs(motion.control_point_offset),
    "skid_icr_offset = " + num(motion.skid_icr_offset) +
      " must be below control_point_offset / 2 = " +
      num(0.5 * std::abs(motion.control_point_offset)) + " in magnitude");

  const PhaseConfig & phases = config.phases;
  check.angle("rotate_to_field_angle", phases.rotate_to_field_angle);
  check.angle("rotate_to_field_release_angle", phases.rotate_to_field_release_angle);
  check.require(
    !(phases.rotate_to_field_release_angle > phases.rotate_to_field_angle),
    "rotate_to_field_release_angle = " + num(phases.rotate_to_field_release_angle) +
      " must not exceed rotate_to_field_angle = " + num(phases.rotate_to_field_angle) +
      " (the phase hysteresis needs release <= entry)");
  check.positive("rotate_to_heading_angular_vel", phases.rotate_to_heading_angular_vel);
  check.autoOrNonNegative("final_approach_distance", phases.final_approach_distance);
  check.positive("terminal_braking_distance", phases.terminal_braking_distance);
  check.nonNegative("min_approach_speed", phases.min_approach_speed);
  check.positive("cancel_deceleration", phases.cancel_deceleration);
  check.positive("cusp_tolerance", phases.cusp_tolerance);

  const ObstacleConfig & obstacles = config.obstacles;
  check.autoOrPositive("obstacle_contour_distance", obstacles.obstacle_contour_distance);
  check.autoOrPositive("obstacle_full_switch_distance", obstacles.obstacle_full_switch_distance);
  check.autoOrPositive("obstacle_blend_start_distance", obstacles.obstacle_blend_start_distance);
  check.autoOrPositive("obstacle_lookahead", obstacles.obstacle_lookahead);
  check.finiteValue("obstacle_gate_tolerance", obstacles.obstacle_gate_tolerance);
  check.nonNegative("obstacle_release_time", obstacles.obstacle_release_time);
  check.positive("obstacle_max_contour_length", obstacles.obstacle_max_contour_length);
  check.positive("obstacle_max_contour_time", obstacles.obstacle_max_contour_time);
  check.nonNegative("clearance_slowdown_distance", obstacles.clearance_slowdown_distance);
  check.positive("clearance_slowdown_gain", obstacles.clearance_slowdown_gain);
  check.positive("cbf_gain", obstacles.cbf_gain);
  check.nonNegative("cbf_margin", obstacles.cbf_margin);
  check.require(
    obstacles.cbf_sites >= 1 && obstacles.cbf_sites <= 3,
    "cbf_sites = " + std::to_string(obstacles.cbf_sites) + " must be in [1, 3]");

  check.positive("collision_check_horizon", config.safety.collision_check_horizon);
  check.nonNegative("stop_margin", config.safety.stop_margin);

  check.positive("controller_frequency", config.node.controller_frequency);

  std::string reason;
  if (!footprintIsValid(config.node.footprint, &reason)) {
    addIssue(&report->errors, issue::kInvalidFootprint, "footprint: " + reason);
  }
}

// Returns true iff the costmap geometry is usable for the costmap rules.
bool checkCostmap(const CostmapInfo & costmap, ValidationReport * report)
{
  const bool size_ok = finite(costmap.width_m) && costmap.width_m > 0.0 &&
                       finite(costmap.height_m) && costmap.height_m > 0.0;
  const bool resolution_ok = finite(costmap.resolution) && costmap.resolution > 0.0;
  const bool window_ok =
    !costmap.window_limit || (finite(*costmap.window_limit) && *costmap.window_limit > 0.0);
  if (size_ok && resolution_ok && window_ok) {
    return true;
  }
  std::string message = "costmap " + num(costmap.width_m) + " x " + num(costmap.height_m) +
                        " m, resolution " + num(costmap.resolution) + " m";
  if (costmap.window_limit) {
    message += ", window_limit " + num(*costmap.window_limit) + " m";
  }
  message += ": sizes, resolution and window_limit must be finite and > 0";
  addIssue(&report->errors, issue::kInvalidCostmap, std::move(message));
  return false;
}

// Distance from the origin to segment [a, b].
double originToSegment(const Vec2 & a, const Vec2 & b) noexcept
{
  const Vec2 ab = b - a;
  const double length_sq = ab.squaredNorm();
  double tau = 0.0;
  if (length_sq > 0.0) {
    tau = std::clamp(-a.dot(ab) / length_sq, 0.0, 1.0);
  }
  return (a + tau * ab).norm();
}

// Crossing-number test of the origin against a polygon (boundary points may go either way; the
// caller only uses it where the boundary distance is then 0).
bool originInside(const std::vector<Vec2> & polygon) noexcept
{
  bool inside = false;
  const std::size_t n = polygon.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Vec2 & a = polygon[j];
    const Vec2 & b = polygon[i];
    if ((a.y() > 0.0) != (b.y() > 0.0)) {
      const double x_cross = a.x() + (0.0 - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
      if (x_cross > 0.0) {
        inside = !inside;
      }
    }
  }
  return inside;
}

double polygonArea(const std::vector<Vec2> & polygon) noexcept
{
  double twice_area = 0.0;
  const std::size_t n = polygon.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    twice_area += cross2(polygon[j], polygon[i]);
  }
  return 0.5 * twice_area;
}

constexpr double kMinFootprintArea = 1e-9;  // m^2

}  // namespace

// -----------------------------------------------------------------------------------------------
// Enum strings.

const char * toString(GainType value) noexcept { return nameOf(kGainTypeNames, value); }
const char * toString(MotionModel value) noexcept { return nameOf(kMotionModelNames, value); }
const char * toString(TrackingLaw value) noexcept { return nameOf(kTrackingLawNames, value); }
const char * toString(OmniHeadingPolicy value) noexcept
{
  return nameOf(kOmniHeadingPolicyNames, value);
}
const char * toString(ObstacleMode value) noexcept { return nameOf(kObstacleModeNames, value); }
const char * toString(ObstacleActivation value) noexcept
{
  return nameOf(kObstacleActivationNames, value);
}

bool fromString(std::string_view text, GainType * out) noexcept
{
  return valueOf(kGainTypeNames, text, out);
}
bool fromString(std::string_view text, MotionModel * out) noexcept
{
  return valueOf(kMotionModelNames, text, out);
}
bool fromString(std::string_view text, TrackingLaw * out) noexcept
{
  return valueOf(kTrackingLawNames, text, out);
}
bool fromString(std::string_view text, OmniHeadingPolicy * out) noexcept
{
  return valueOf(kOmniHeadingPolicyNames, text, out);
}
bool fromString(std::string_view text, ObstacleMode * out) noexcept
{
  return valueOf(kObstacleModeNames, text, out);
}
bool fromString(std::string_view text, ObstacleActivation * out) noexcept
{
  return valueOf(kObstacleActivationNames, text, out);
}

// -----------------------------------------------------------------------------------------------
// ValidationReport.

bool ValidationReport::hasError(std::string_view code) const noexcept
{
  return std::any_of(
    errors.begin(), errors.end(), [code](const ValidationIssue & i) { return i.code == code; });
}

bool ValidationReport::hasWarning(std::string_view code) const noexcept
{
  return std::any_of(
    warnings.begin(), warnings.end(), [code](const ValidationIssue & i) { return i.code == code; });
}

std::string ValidationReport::summary() const
{
  std::string text;
  for (const ValidationIssue & issue : errors) {
    text += "error [" + issue.code + "]: " + issue.message + "\n";
  }
  for (const ValidationIssue & issue : warnings) {
    text += "warning [" + issue.code + "]: " + issue.message + "\n";
  }
  return text;
}

void ValidationReport::throwOnError() const
{
  if (errors.empty()) {
    return;
  }
  std::string text = "invalid configuration:";
  for (const ValidationIssue & issue : errors) {
    text += " [" + issue.code + "] " + issue.message + ";";
  }
  text.pop_back();
  throw ConfigError(text);
}

// -----------------------------------------------------------------------------------------------
// Laws, footprint and auto values.

bool isValidCombination(MotionModel model, TrackingLaw law) noexcept
{
  if (law == TrackingLaw::kAuto) {
    return true;
  }
  switch (model) {
    case MotionModel::kDifferential:
    case MotionModel::kSkidSteer:
      return law == TrackingLaw::kFeedbackLinearization || law == TrackingLaw::kHeadingTracking;
    case MotionModel::kOmnidirectional:
      return law == TrackingLaw::kHolonomic;
    case MotionModel::kAckermann:
      return law == TrackingLaw::kHeadingTracking;
  }
  return false;
}

TrackingLaw resolveTrackingLaw(MotionModel model, TrackingLaw law) noexcept
{
  if (law != TrackingLaw::kAuto) {
    return law;
  }
  switch (model) {
    case MotionModel::kDifferential:
    case MotionModel::kSkidSteer:
      return TrackingLaw::kFeedbackLinearization;
    case MotionModel::kOmnidirectional:
      return TrackingLaw::kHolonomic;
    case MotionModel::kAckermann:
      return TrackingLaw::kHeadingTracking;
  }
  return TrackingLaw::kFeedbackLinearization;
}

FootprintRadii computeFootprintRadii(const FootprintSpec & footprint) noexcept
{
  FootprintRadii radii;
  if (footprint.polygon.empty()) {
    radii.w_lat = footprint.radius;
    radii.r_insc = footprint.radius;
    radii.r_circ = footprint.radius;
    return radii;
  }
  for (const Vec2 & vertex : footprint.polygon) {
    radii.w_lat = std::max(radii.w_lat, std::abs(vertex.y()));
    radii.r_circ = std::max(radii.r_circ, vertex.norm());
  }
  const std::size_t n = footprint.polygon.size();
  if (n >= 3 && originInside(footprint.polygon)) {
    double nearest = kInf;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
      nearest = std::min(nearest, originToSegment(footprint.polygon[j], footprint.polygon[i]));
    }
    radii.r_insc = nearest;
  }
  return radii;
}

bool footprintIsValid(const FootprintSpec & footprint, std::string * reason) noexcept
{
  auto fail = [reason](const std::string & why) {
    if (reason != nullptr) {
      *reason = why;
    }
    return false;
  };
  if (footprint.polygon.empty()) {
    if (!finite(footprint.radius) || footprint.radius < 0.0) {
      return fail("radius " + num(footprint.radius) + " must be finite and >= 0");
    }
    return true;
  }
  if (footprint.polygon.size() < 3) {
    return fail(
      "polygon has " + std::to_string(footprint.polygon.size()) + " vertices; at least 3 needed");
  }
  for (const Vec2 & vertex : footprint.polygon) {
    if (!isFinite(vertex)) {
      return fail("polygon has a non-finite vertex");
    }
  }
  if (!(std::abs(polygonArea(footprint.polygon)) > kMinFootprintArea)) {
    return fail("polygon has zero area");
  }
  return true;
}

double evaluationOffset(const CoreConfig & config) noexcept
{
  const MotionModel model = config.motion.motion_model;
  if (model == MotionModel::kAckermann) {
    return std::abs(config.motion.reference_point_offset);
  }
  const TrackingLaw law = resolveTrackingLaw(model, config.motion.tracking_law);
  if (law == TrackingLaw::kFeedbackLinearization) {
    return config.motion.control_point_offset;
  }
  return 0.0;
}

double autoContourDistance(const FootprintRadii & radii, double evaluation_offset) noexcept
{
  const double d = evaluation_offset;
  const double lateral = radii.w_lat + 0.10;
  return std::max(std::sqrt(lateral * lateral + d * d), radii.r_circ + d + 0.05);
}

double ackermannAutoContourDistance(
  const FootprintRadii & radii, double evaluation_offset, double min_turning_radius) noexcept
{
  return std::max(
    autoContourDistance(radii, evaluation_offset), min_turning_radius + radii.w_lat + 0.05);
}

CoreConfig resolveAutoValues(const CoreConfig & config)
{
  CoreConfig resolved = config;
  resolved.motion.tracking_law =
    resolveTrackingLaw(config.motion.motion_model, config.motion.tracking_law);

  // Ackermann (math_spec §6.6 [Design, M7]): the circulation and the turn-in grow with R_min.
  const bool ackermann = resolved.motion.motion_model == MotionModel::kAckermann &&
                         resolved.motion.min_turning_radius > 0.0;
  const double r_min = ackermann ? resolved.motion.min_turning_radius : 0.0;
  const FootprintRadii radii = computeFootprintRadii(resolved.node.footprint);
  ObstacleConfig & obstacles = resolved.obstacles;
  if (isAuto(obstacles.obstacle_contour_distance)) {
    obstacles.obstacle_contour_distance =
      ackermann ? ackermannAutoContourDistance(radii, evaluationOffset(resolved), r_min)
                : autoContourDistance(radii, evaluationOffset(resolved));
  }
  const double lambda = obstacles.obstacle_contour_distance;
  if (isAuto(obstacles.obstacle_full_switch_distance)) {
    obstacles.obstacle_full_switch_distance = lambda + std::max(0.15, r_min);
  }
  if (isAuto(obstacles.obstacle_blend_start_distance)) {
    obstacles.obstacle_blend_start_distance =
      ackermann ? obstacles.obstacle_full_switch_distance + 0.2 : lambda + 0.35;
  }
  resolved.motion.max_reverse_vel = reverseSpeedLimit(config);
  if (isAuto(obstacles.obstacle_lookahead)) {
    obstacles.obstacle_lookahead =
      std::max(1.0, resolved.field.desired_speed * resolved.safety.collision_check_horizon);
    if (ackermann) {
      obstacles.obstacle_lookahead =
        std::max(obstacles.obstacle_lookahead, obstacles.obstacle_blend_start_distance + 0.1);
    }
  }
  return resolved;
}

double reverseSpeedLimit(const CoreConfig & config) noexcept
{
  return isAuto(config.motion.max_reverse_vel) ? config.motion.max_linear_vel
                                               : config.motion.max_reverse_vel;
}

double referenceSpeed(const CoreConfig & config, int drive_sign) noexcept
{
  const double v_r = config.field.desired_speed;
  if (drive_sign >= 0 || config.motion.motion_model == MotionModel::kOmnidirectional) {
    return v_r;
  }
  return std::min(v_r, reverseSpeedLimit(config));
}

double finalApproachDistance(const CoreConfig & config, double xy_tol) noexcept
{
  if (!isAuto(config.phases.final_approach_distance)) {
    return config.phases.final_approach_distance;
  }
  const TrackingLaw law =
    resolveTrackingLaw(config.motion.motion_model, config.motion.tracking_law);
  if (law == TrackingLaw::kFeedbackLinearization) {
    const double d = config.motion.control_point_offset;
    return std::max(2.0 * d, xy_tol + d);
  }
  return xy_tol;
}

// -----------------------------------------------------------------------------------------------
// Validators.

namespace
{

/// @brief The Ackermann contour warnings of math_spec §6.6 [Design, M7] on a resolved config.
void addAckermannContourWarnings(
  const CoreConfig & r, const FootprintRadii & radii, std::vector<ValidationIssue> * warnings)
{
  const double r_min = r.motion.min_turning_radius;
  const ObstacleConfig & obstacles = r.obstacles;
  const double lambda = obstacles.obstacle_contour_distance;
  const double d_in = obstacles.obstacle_full_switch_distance;
  const double d_in_prime = obstacles.obstacle_blend_start_distance;
  if (lambda < r_min + radii.w_lat + 0.05) {
    addIssue(
      warnings, issue::kAckermannContourCurvature,
      "obstacle_contour_distance " + num(lambda) +
        " < min_turning_radius + w_lat + 0.05 = " + num(r_min + radii.w_lat + 0.05) +
        ": the circulation leaves little curvature for corrections (kCurvatureSaturated)");
  }
  if (d_in < lambda + r_min) {
    addIssue(
      warnings, issue::kAckermannContourTurnIn,
      "obstacle_full_switch_distance " + num(d_in) + " < obstacle_contour_distance + " +
        "min_turning_radius = " + num(lambda + r_min) +
        ": a face across the approach is met inside lambda (kCurvatureSaturated)");
  }
  if (
    obstacles.obstacle_activation == ObstacleActivation::kPathBlocked &&
    obstacles.obstacle_lookahead < d_in_prime) {
    addIssue(
      warnings, issue::kAckermannContourLookahead,
      "obstacle_lookahead " + num(obstacles.obstacle_lookahead) +
        " < obstacle_blend_start_distance " + num(d_in_prime) +
        ": the gate can open after the car is inside the turn-in band");
  }
  const double half_circulation = d_in_prime - lambda + kPi * lambda;
  if (obstacles.obstacle_max_contour_length < half_circulation) {
    addIssue(
      warnings, issue::kAckermannContourWatchdog,
      "obstacle_max_contour_length " + num(obstacles.obstacle_max_contour_length) +
        " < D'_in - lambda + pi lambda = " + num(half_circulation) +
        ": the watchdog can trip during a plain half circulation");
  }
}

}  // namespace

ValidationReport validateConfig(
  const CoreConfig & config, const std::optional<CostmapInfo> & costmap) noexcept
{
  ValidationReport report;
  checkDomain(config, &report);
  if (!report.errors.empty()) {
    return report;  // the §11 formulas would be meaningless
  }
  const bool costmap_ok = costmap.has_value() && checkCostmap(*costmap, &report);

  const CoreConfig r = resolveAutoValues(config);
  const FootprintRadii radii = computeFootprintRadii(r.node.footprint);
  const GainFunction gain(r.field.gain_function, r.field.convergence_gain);  // k_f checked above

  const MotionModel model = r.motion.motion_model;
  const TrackingLaw law = r.motion.tracking_law;
  const bool fl = law == TrackingLaw::kFeedbackLinearization;
  const bool ackermann = model == MotionModel::kAckermann;
  const bool contour = r.obstacles.obstacle_mode == ObstacleMode::kContour;
  const bool cbf = r.obstacles.obstacle_mode == ObstacleMode::kCbf;

  const double v_r = r.field.desired_speed;
  const double k_f = r.field.convergence_gain;
  const double v_max = r.motion.max_linear_vel;
  const double w_max = r.motion.max_angular_vel;
  const double a_dec = r.motion.max_linear_decel;
  const double d = r.motion.control_point_offset;
  const double d_eval = evaluationOffset(r);
  const double f = r.node.controller_frequency;
  const double dt = 1.0 / f;
  const double sigma_b = r.phases.terminal_braking_distance;
  const double horizon = r.safety.collision_check_horizon;
  const double lambda = r.obstacles.obstacle_contour_distance;
  const double d_in = r.obstacles.obstacle_full_switch_distance;
  const double d_in_prime = r.obstacles.obstacle_blend_start_distance;
  const double r_min = r.motion.min_turning_radius;
  const double rotate_vel = r.phases.rotate_to_heading_angular_vel;
  const double in_place_vel = r.motion.min_in_place_angular_vel;
  const double kappa_path = 0.0;  // configure time (Appendix C item 16)
  const double half_window =
    costmap_ok ? 0.5 * std::min(costmap->width_m, costmap->height_m) : kInf;

  std::vector<ValidationIssue> * errors = &report.errors;
  std::vector<ValidationIssue> * warnings = &report.warnings;

  // --- Reject (math_spec §11) -------------------------------------------------------------------
  if (v_r > v_max) {
    addIssue(
      errors, issue::kSpeedAboveMax,
      "desired_speed " + num(v_r) + " > max_linear_vel " + num(v_max) +
        ": the field speed must be commandable");
  }
  const double v_rev = r.motion.max_reverse_vel;
  if (v_rev > v_max) {
    addIssue(
      errors, issue::kReverseAboveMax,
      "max_reverse_vel " + num(v_rev) + " > max_linear_vel " + num(v_max) +
        ": the reverse bound must not exceed the forward one");
  }
  if (ackermann && !isAuto(config.motion.max_reverse_vel) && v_rev < r.motion.ackermann_min_speed) {
    addIssue(
      errors, issue::kAckermannReverseBelowBand,
      "max_reverse_vel " + num(v_rev) + " < ackermann_min_speed " +
        num(r.motion.ackermann_min_speed) + ": the band cannot be commanded in reverse");
  }
  if (in_place_vel > rotate_vel) {
    addIssue(
      errors, issue::kInPlaceAboveRotate,
      "min_in_place_angular_vel " + num(in_place_vel) + " > rotate_to_heading_angular_vel " +
        num(rotate_vel) + ": the floor would exceed the rotate speed");
  }
  if (contour && !(lambda < d_in && d_in <= d_in_prime)) {
    addIssue(
      errors, issue::kContourOrder,
      "obstacle distances must satisfy lambda < D_in <= D'_in, got lambda " + num(lambda) +
        ", D_in " + num(d_in) + ", D'_in " + num(d_in_prime));
  }
  if (contour && lambda - d_eval < radii.r_insc) {
    addIssue(
      errors, issue::kContourBelowInscribed,
      "obstacle_contour_distance " + num(lambda) + " - evaluation offset " + num(d_eval) +
        " < inscribed radius " + num(radii.r_insc) + ": the circulation itself would collide");
  }
  if (!isValidCombination(model, law)) {
    addIssue(
      errors, issue::kInvalidLaw,
      std::string("tracking_law ") + toString(law) + " is not valid for motion_model " +
        toString(model) +
        " (diff/skid: feedback_linearization or heading_tracking; "
        "omnidirectional: holonomic; ackermann: heading_tracking)");
  }
  if (ackermann && contour && r_min > 0.0 && lambda < r_min + 0.05) {
    addIssue(
      errors, issue::kAckermannContourRadius,
      "obstacle_contour_distance " + num(lambda) + " < min_turning_radius " + num(r_min) +
        " + 0.05: the car cannot follow the circulation around a point obstacle");
  }
  if (ackermann && cbf) {
    addIssue(
      errors, issue::kAckermannCbfUnsupported,
      "obstacle_mode cbf is not supported for motion_model ackermann in 0.x: a car cannot follow "
      "an arbitrary filtered field; use contour");
  }
  if (ackermann && !(r_min > 0.0)) {
    addIssue(
      errors, issue::kAckermannRadius,
      "min_turning_radius " + num(r_min) + " must be > 0 for motion_model ackermann");
  }
  if (costmap_ok) {
    const double needed = sigma_b + d_eval + 0.1;
    const bool exceeds_window = costmap->window_limit && needed > *costmap->window_limit;
    if (needed > half_window || exceeds_window) {
      std::string message = "terminal_braking_distance " + num(sigma_b) + " + evaluation offset " +
                            num(d_eval) + " + 0.1 = " + num(needed) +
                            " exceeds half the costmap size " + num(half_window);
      if (costmap->window_limit) {
        message += " or the window limit " + num(*costmap->window_limit);
      }
      message += ": the goal must enter the window before braking must start";
      addIssue(errors, issue::kBrakingExceedsWindow, std::move(message));
    }
  }
  if (fl && !(d > 0.0)) {
    addIssue(
      errors, issue::kFlOffset,
      "control_point_offset " + num(d) +
        " must be > 0 for feedback_linearization (M(psi) "
        "divides by d)");
  }

  // --- Warn (math_spec §11) ---------------------------------------------------------------------
  if (rotate_vel > w_max) {
    addIssue(
      warnings, issue::kRotateAboveMax,
      "rotate_to_heading_angular_vel " + num(rotate_vel) + " > max_angular_vel " + num(w_max) +
        ": the effective rate is the minimum");
  }
  if (gain.dG0() * v_r * dt >= 1.0) {
    addIssue(
      warnings, issue::kDiscreteOvershoot,
      "G'(0) v_r dt = " + num(gain.dG0() * v_r * dt) +
        " >= 1: the discrete approach overshoots the curve");
  }
  if (fl && gain.dG0() * d > 0.5) {
    addIssue(
      warnings, issue::kFlGainOffset,
      "G'(0) d = " + num(gain.dG0() * d) +
        " > 0.5: the field turns faster than feedback linearisation's heading can follow");
  }
  if (sigma_b < v_r * v_r / a_dec) {
    addIssue(
      warnings, issue::kBrakingTooShort,
      "terminal_braking_distance " + num(sigma_b) + " < v_r^2/a_dec = " + num(v_r * v_r / a_dec) +
        ": terminal braking exceeds max_linear_decel");
  }
  if (contour && lambda - d_eval < radii.r_circ) {
    addIssue(
      warnings, issue::kContourBelowCircumscribed,
      "obstacle_contour_distance " + num(lambda) + " - evaluation offset " + num(d_eval) +
        " < circumscribed radius " + num(radii.r_circ) +
        ": a footprint corner can reach the obstacle during circulation");
  }
  if (contour && costmap_ok && d_in_prime + d_eval >= half_window - 0.2) {
    addIssue(
      warnings, issue::kBlendExceedsWindow,
      "obstacle_blend_start_distance " + num(d_in_prime) + " + evaluation offset " + num(d_eval) +
        " >= half the costmap size - 0.2 = " + num(half_window - 0.2) +
        ": the blend zone does not fit the rolling window");
  }
  const double reach = horizon * v_max;
  const double stopping = v_max * v_max / (2.0 * a_dec) + v_max * (2.0 / f + 0.1) + 0.1;
  if (reach < stopping) {
    addIssue(
      warnings, issue::kHorizonTooShort,
      "collision_check_horizon * max_linear_vel = " + num(reach) + " < stopping distance " +
        "plus two cycles and 0.1 s of latency plus 0.1 m = " + num(stopping));
  }
  if (costmap_ok && reach > half_window - radii.r_circ) {
    addIssue(
      warnings, issue::kHorizonExceedsWindow,
      "collision_check_horizon * max_linear_vel = " + num(reach) +
        " > half the costmap size - circumscribed radius = " + num(half_window - radii.r_circ) +
        ": the rollout leaves the window");
  }
  const double curvature = gain.cG() * k_f + kappa_path;
  if (ackermann && r_min * curvature > 1.0) {
    addIssue(
      warnings, issue::kAckermannCurvature,
      "min_turning_radius " + num(r_min) + " * (c_G k_f + kappa_path) = " + num(r_min * curvature) +
        " > 1: the field's integral curves need more curvature than "
        "the car has");
  }
  if (v_r * curvature > w_max) {
    addIssue(
      warnings, issue::kYawRateForCurvature,
      "desired_speed * (c_G k_f + kappa_path) = " + num(v_r * curvature) + " > max_angular_vel " +
        num(w_max) + ": the yaw rate needed to follow the integral curves is not available");
  }
  if (ackermann && contour && r_min > 0.0) {
    addAckermannContourWarnings(r, radii, warnings);
  }
  const double cbf_gain = r.obstacles.cbf_gain;
  if (cbf && cbf_gain * v_r > a_dec) {
    addIssue(
      warnings, issue::kCbfGainDeceleration,
      "cbf_gain * desired_speed = " + num(cbf_gain * v_r) + " > max_linear_decel " + num(a_dec) +
        ": a head-on approach asks for more deceleration than the limiter allows, so the robot "
        "enters the margin (the collision cap still stops it)");
  }
  if (cbf && cbf_gain * dt >= 1.0) {
    addIssue(
      warnings, issue::kCbfGainDiscrete,
      "cbf_gain / controller_frequency = " + num(cbf_gain * dt) +
        " >= 1: the barrier can cross zero within one cycle");
  }
  return report;
}

ValidationReport adviseForPlan(const CoreConfig & config, double kappa_path) noexcept
{
  ValidationReport report;
  const double k_f = config.field.convergence_gain;
  if (!finite(k_f) || !(k_f > 0.0) || !finite(kappa_path)) {
    return report;  // validateConfig() reports the domain error
  }
  const GainFunction gain(config.field.gain_function, k_f);
  const TrackingLaw law =
    resolveTrackingLaw(config.motion.motion_model, config.motion.tracking_law);
  const double d = config.motion.control_point_offset;
  const double r_min = config.motion.min_turning_radius;
  const double v_r = config.field.desired_speed;
  const double w_max = config.motion.max_angular_vel;
  const double curvature = gain.cG() * k_f + kappa_path;

  if (law == TrackingLaw::kFeedbackLinearization && kappa_path * d > 1.0) {
    addIssue(
      &report.warnings, issue::kFlPathCurvature,
      "kappa_path " + num(kappa_path) + " * control_point_offset " + num(d) +
        " > 1: the control point cuts the path's corners");
  }
  if (config.motion.motion_model == MotionModel::kAckermann && r_min * curvature > 1.0) {
    addIssue(
      &report.warnings, issue::kAckermannCurvature,
      "min_turning_radius " + num(r_min) + " * (c_G k_f + kappa_path) = " + num(r_min * curvature) +
        " > 1 for this plan");
  }
  if (v_r * curvature > w_max) {
    addIssue(
      &report.warnings, issue::kYawRateForCurvature,
      "desired_speed * (c_G k_f + kappa_path) = " + num(v_r * curvature) + " > max_angular_vel " +
        num(w_max) + " for this plan");
  }
  return report;
}

ValidationReport validateCurveOptions(const CurveOptions & options) noexcept
{
  ValidationReport report;
  auto fail = [&report](const std::string & message) {
    addIssue(&report.errors, issue::kInvalidCurveOptions, message);
  };
  if (options.max_curve_points < 2) {
    fail("max_curve_points = " + std::to_string(options.max_curve_points) + " must be >= 2");
  }
  if (!finite(options.default_sample_spacing) || !(options.default_sample_spacing > 0.0)) {
    fail("default_sample_spacing = " + num(options.default_sample_spacing) + " must be > 0");
  }
  if (!finite(options.search_window) || !(options.search_window > 0.0)) {
    fail("search_window = " + num(options.search_window) + " must be > 0");
  }
  if (!finite(options.global_search_trigger) || !(options.global_search_trigger >= 0.0)) {
    fail("global_search_trigger = " + num(options.global_search_trigger) + " must be >= 0");
  }
  if (!finite(options.global_search_hysteresis) || !(options.global_search_hysteresis >= 0.0)) {
    fail("global_search_hysteresis = " + num(options.global_search_hysteresis) + " must be >= 0");
  }
  if (options.global_search_samples < 4) {
    fail(
      "global_search_samples = " + std::to_string(options.global_search_samples) + " must be >= 4");
  }
  return report;
}

}  // namespace vectorfield_core
