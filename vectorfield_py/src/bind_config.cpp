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

// Configuration: the parameter enums, CoreConfig and its groups, CurveOptions, CostmapInfo, the
// validators and their issue codes (config.hpp).

#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "bindings.hpp"

namespace vectorfield_py
{

namespace
{

// A parameter enum: upper-case values, plus construction from (and implicit conversion of) the
// parameter string, to_string() and from_string().
template <typename E>
void bindParameterEnum(
  py::module_ & m, const char * name, const char * doc,
  std::initializer_list<std::pair<const char *, E>> values)
{
  py::enum_<E> e(m, name, doc);
  for (const auto & [value_name, value] : values) {
    e.value(value_name, value);
  }
  const auto parse = [name](const std::string & text) {
    E out{};
    if (!vf::fromString(text, &out)) {
      throw py::value_error("unknown " + std::string(name) + " '" + text + "'");
    }
    return out;
  };
  e.def(py::init(parse), py::arg("text"), "From the parameter string, e.g. \"atan\".");
  e.def_static("from_string", parse, py::arg("text"), "From the parameter string.");
  e.def(
    "to_string", [](E value) { return std::string(vf::toString(value)); },
    "The parameter string, e.g. \"atan\".");
  py::implicitly_convertible<py::str, E>();
}

void bindEnums(py::module_ & m)
{
  bindParameterEnum<vf::GainType>(
    m, "GainType", "`gain_function` (math_spec section 3).",
    {{"ATAN", vf::GainType::kAtan}, {"RATIONAL", vf::GainType::kRational}});
  bindParameterEnum<vf::MotionModel>(
    m, "MotionModel", "`motion_model`.",
    {{"DIFFERENTIAL", vf::MotionModel::kDifferential},
     {"SKID_STEER", vf::MotionModel::kSkidSteer},
     {"OMNIDIRECTIONAL", vf::MotionModel::kOmnidirectional},
     {"ACKERMANN", vf::MotionModel::kAckermann}});
  bindParameterEnum<vf::TrackingLaw>(
    m, "TrackingLaw",
    "`tracking_law` (math_spec section 7); AUTO resolves per motion model (resolve_tracking_law).",
    {{"AUTO", vf::TrackingLaw::kAuto},
     {"FEEDBACK_LINEARIZATION", vf::TrackingLaw::kFeedbackLinearization},
     {"HEADING_TRACKING", vf::TrackingLaw::kHeadingTracking},
     {"HOLONOMIC", vf::TrackingLaw::kHolonomic}});
  bindParameterEnum<vf::OmniHeadingPolicy>(
    m, "OmniHeadingPolicy", "`omni_heading_policy` (math_spec section 7.2).",
    {{"PATH_TANGENT", vf::OmniHeadingPolicy::kPathTangent},
     {"FIELD", vf::OmniHeadingPolicy::kField},
     {"FIXED", vf::OmniHeadingPolicy::kFixed}});
  bindParameterEnum<vf::ObstacleMode>(
    m, "ObstacleMode", "`obstacle_mode` (math_spec section 6).",
    {{"NONE", vf::ObstacleMode::kNone},
     {"CONTOUR", vf::ObstacleMode::kContour},
     {"CBF", vf::ObstacleMode::kCbf}});
  bindParameterEnum<vf::ObstacleActivation>(
    m, "ObstacleActivation", "`obstacle_activation` (math_spec section 6.4).",
    {{"PATH_BLOCKED", vf::ObstacleActivation::kPathBlocked},
     {"ALWAYS", vf::ObstacleActivation::kAlways}});
}

void bindGroups(py::module_ & m)
{
  StructBinder<vf::FieldConfig>(m, "FieldConfig", "Field parameter group (plan section 5.4).")
    .field("desired_speed", &vf::FieldConfig::desired_speed, "v_r (m/s).")
    .field("convergence_gain", &vf::FieldConfig::convergence_gain, "k_f (1/m).")
    .field("gain_function", &vf::FieldConfig::gain_function, "GainType.")
    .field("direction", &vf::FieldConfig::direction, "Traversal direction +1 or -1.")
    .field("assumption3_margin", &vf::FieldConfig::assumption3_margin, "epsilon_v.")
    .finish();

  using vf::MotionConfig;
  StructBinder<MotionConfig>(m, "MotionConfig", "Motion parameter group (plan section 5.4).")
    .field("motion_model", &MotionConfig::motion_model, "MotionModel.")
    .field("tracking_law", &MotionConfig::tracking_law, "TrackingLaw.")
    .field("control_point_offset", &MotionConfig::control_point_offset, "FL offset d (m).")
    .field("drive_backward", &MotionConfig::drive_backward, "Drive backward (q = -1).")
    .field("allow_reversing", &MotionConfig::allow_reversing, "Allow q v < 0.")
    .field("heading_gain", &MotionConfig::heading_gain, "k_psi (1/s).")
    .field("omni_heading_policy", &MotionConfig::omni_heading_policy, "OmniHeadingPolicy.")
    .field("max_linear_vel", &MotionConfig::max_linear_vel, "v_max (m/s).")
    .field(
      "max_reverse_vel", &MotionConfig::max_reverse_vel,
      "v_rev (m/s) while q = -1 (not omni); negative: auto (max_linear_vel).")
    .field("max_lateral_vel", &MotionConfig::max_lateral_vel, "vy_max (m/s), omni only.")
    .field("max_angular_vel", &MotionConfig::max_angular_vel, "omega_max (rad/s).")
    .field("max_linear_accel", &MotionConfig::max_linear_accel, "a_acc (m/s^2).")
    .field("max_linear_decel", &MotionConfig::max_linear_decel, "a_dec (m/s^2, positive).")
    .field("max_angular_accel", &MotionConfig::max_angular_accel, "alpha_max (rad/s^2).")
    .field(
      "min_in_place_angular_vel", &MotionConfig::min_in_place_angular_vel,
      "Rotate-law floor (rad/s).")
    .field("min_turning_radius", &MotionConfig::min_turning_radius, "Ackermann R_min (m).")
    .field("ackermann_min_speed", &MotionConfig::ackermann_min_speed, "v_ack (m/s).")
    .field(
      "reference_point_offset", &MotionConfig::reference_point_offset,
      "Ackermann rear-axle x_r (m; negative = behind base_link).")
    .field(
      "skid_yaw_rate_efficiency", &MotionConfig::skid_yaw_rate_efficiency,
      "Skid slip compensation eta_psi in (0, 1]: omega is divided by it (1 = off).")
    .field(
      "skid_icr_offset", &MotionConfig::skid_icr_offset,
      "Skid slip compensation: body-x of the centre of rotation x_ICR (m; 0 = off).")
    .finish();

  using vf::PhaseConfig;
  StructBinder<PhaseConfig>(m, "PhaseConfig", "Phases parameter group (plan section 5.4).")
    .field("use_rotate_to_field", &PhaseConfig::use_rotate_to_field, "Enable ROTATE_TO_FIELD.")
    .field("rotate_to_field_angle", &PhaseConfig::rotate_to_field_angle, "Entry |e| (rad).")
    .field(
      "rotate_to_field_release_angle", &PhaseConfig::rotate_to_field_release_angle,
      "Exit |e| (rad).")
    .field(
      "use_rotate_to_heading", &PhaseConfig::use_rotate_to_heading, "Enable ROTATE_TO_HEADING.")
    .field(
      "rotate_to_heading_angular_vel", &PhaseConfig::rotate_to_heading_angular_vel,
      "omega_rth (rad/s).")
    .field(
      "final_approach_distance", &PhaseConfig::final_approach_distance,
      "FINAL_APPROACH entry (m); negative = auto.")
    .field("terminal_braking_distance", &PhaseConfig::terminal_braking_distance, "sigma_b (m).")
    .field("min_approach_speed", &PhaseConfig::min_approach_speed, "v_min (m/s).")
    .field("cancel_deceleration", &PhaseConfig::cancel_deceleration, "a_can (m/s^2).")
    .field("cusp_tolerance", &PhaseConfig::cusp_tolerance, "Cusp settle radius (m).")
    .finish();

  using vf::ObstacleConfig;
  StructBinder<ObstacleConfig>(m, "ObstacleConfig", "Obstacles parameter group (plan section 5.4).")
    .field("obstacle_mode", &ObstacleConfig::obstacle_mode, "ObstacleMode.")
    .field("obstacle_activation", &ObstacleConfig::obstacle_activation, "ObstacleActivation.")
    .field(
      "obstacle_contour_distance", &ObstacleConfig::obstacle_contour_distance,
      "lambda (m); negative = auto.")
    .field(
      "obstacle_full_switch_distance", &ObstacleConfig::obstacle_full_switch_distance,
      "D_in (m); negative = auto (lambda + 0.15).")
    .field(
      "obstacle_blend_start_distance", &ObstacleConfig::obstacle_blend_start_distance,
      "D'_in (m); negative = auto (lambda + 0.35).")
    .field(
      "obstacle_lookahead", &ObstacleConfig::obstacle_lookahead,
      "Gate lookahead (m); negative = auto (max(1, v_r T_h)).")
    .field("obstacle_gate_tolerance", &ObstacleConfig::obstacle_gate_tolerance, "(m).")
    .field("obstacle_release_time", &ObstacleConfig::obstacle_release_time, "(s).")
    .field(
      "obstacle_max_contour_length", &ObstacleConfig::obstacle_max_contour_length, "Watchdog (m).")
    .field("obstacle_max_contour_time", &ObstacleConfig::obstacle_max_contour_time, "Watchdog (s).")
    .field("unknown_is_obstacle", &ObstacleConfig::unknown_is_obstacle, "Unknown cells are sites.")
    .field(
      "clearance_slowdown_distance", &ObstacleConfig::clearance_slowdown_distance,
      "d_cl (m); 0 = off.")
    .field("clearance_slowdown_gain", &ObstacleConfig::clearance_slowdown_gain, "k_cl.")
    .field("cbf_gain", &ObstacleConfig::cbf_gain, "CBF gain gamma_b (1/s), obstacle_mode cbf.")
    .field("cbf_margin", &ObstacleConfig::cbf_margin, "CBF margin m (m): delta_safe = r_e + m.")
    .field("cbf_sites", &ObstacleConfig::cbf_sites, "Obstacle sites the CBF constrains, 1-3.")
    .finish();

  using vf::SafetyConfig;
  StructBinder<SafetyConfig>(m, "SafetyConfig", "Safety parameter group (plan section 5.4).")
    .field(
      "use_collision_detection", &SafetyConfig::use_collision_detection,
      "Run the collision predictor.")
    .field("collision_check_horizon", &SafetyConfig::collision_check_horizon, "T_h (s).")
    .field("stop_margin", &SafetyConfig::stop_margin, "m_s (m).")
    .finish();

  StructBinder<vf::FootprintSpec>(
    m, "FootprintSpec",
    "Robot footprint (robot frame): a polygon (N, 2), or a circle of `radius` when the polygon "
    "is empty.")
    .field("polygon", &vf::FootprintSpec::polygon, "Polygon vertices (N, 2) (m).")
    .field("radius", &vf::FootprintSpec::radius, "Circle radius (m) when polygon is empty.")
    .finish();

  StructBinder<vf::FootprintRadii>(
    m, "FootprintRadii", "w_lat, r_insc and r_circ of a footprint (math_spec section 10.1).")
    .field("w_lat", &vf::FootprintRadii::w_lat, "Max |y| over the footprint (m).")
    .field("r_insc", &vf::FootprintRadii::r_insc, "Inscribed radius (m).")
    .field("r_circ", &vf::FootprintRadii::r_circ, "Circumscribed radius (m).")
    .finish();

  StructBinder<vf::NodeConfig>(m, "NodeConfig", "Node-derived configuration.")
    .field("controller_frequency", &vf::NodeConfig::controller_frequency, "f (Hz); period 1/f.")
    .field("footprint", &vf::NodeConfig::footprint, "FootprintSpec.")
    .finish();

  using vf::CoreConfig;
  StructBinder<CoreConfig>(
    m, "CoreConfig",
    "Complete Tracker configuration. The defaults are the plan section 5.4 defaults (TB3 "
    "footprint) and pass validate_config() with no error and no warning.")
    .field("field", &CoreConfig::field, "FieldConfig.")
    .field("motion", &CoreConfig::motion, "MotionConfig.")
    .field("phases", &CoreConfig::phases, "PhaseConfig.")
    .field("obstacles", &CoreConfig::obstacles, "ObstacleConfig.")
    .field("safety", &CoreConfig::safety, "SafetyConfig.")
    .field("node", &CoreConfig::node, "NodeConfig.")
    .finish();

  using vf::CurveOptions;
  StructBinder<CurveOptions>(
    m, "CurveOptions", "Curve-intake options (plan section 6), validated by make_curve().")
    .field("max_curve_points", &CurveOptions::max_curve_points, "Max vertices after conditioning.")
    .field(
      "default_sample_spacing", &CurveOptions::default_sample_spacing,
      "Polyline resampling (m) when the spec's sample_spacing <= 0.")
    .field("search_window", &CurveOptions::search_window, "Hinted search window w (m).")
    .field("global_search_trigger", &CurveOptions::global_search_trigger, "D_trig (m).")
    .field("global_search_hysteresis", &CurveOptions::global_search_hysteresis, "h_g (m).")
    .field("global_search_samples", &CurveOptions::global_search_samples, "M samples.")
    .finish();

  StructBinder<vf::CostmapInfo>(
    m, "CostmapInfo", "Costmap geometry for the costmap-dependent validation rules.")
    .field("width_m", &vf::CostmapInfo::width_m, "Width W (m).")
    .field("height_m", &vf::CostmapInfo::height_m, "Height H_c (m).")
    .field("resolution", &vf::CostmapInfo::resolution, "Resolution (m per cell).")
    .field("window_limit", &vf::CostmapInfo::window_limit, "Gen A/B prune_distance, or None.")
    .finish();
}

void bindValidation(py::module_ & m)
{
  py::class_<vf::ValidationIssue>(m, "ValidationIssue", "One finding of a validator.")
    .def_readonly("code", &vf::ValidationIssue::code, "Stable code (see vectorfield.issue).")
    .def_readonly("message", &vf::ValidationIssue::message, "Human-readable explanation.")
    .def("__repr__", [](const vf::ValidationIssue & s) {
      return "ValidationIssue(code=" + py::repr(py::str(s.code)).cast<std::string>() +
             ", message=" + py::repr(py::str(s.message)).cast<std::string>() + ")";
    });

  using vf::ValidationReport;
  py::class_<ValidationReport>(
    m, "ValidationReport", "Result of a validator: errors reject, warnings advise.")
    .def_readonly("errors", &ValidationReport::errors, "List of ValidationIssue.")
    .def_readonly("warnings", &ValidationReport::warnings, "List of ValidationIssue.")
    .def_property_readonly("ok", &ValidationReport::ok, "True iff there is no error.")
    .def(
      "has_error",
      [](const ValidationReport & r, const std::string & code) { return r.hasError(code); },
      py::arg("code"), "True iff an error with this code is present.")
    .def(
      "has_warning",
      [](const ValidationReport & r, const std::string & code) { return r.hasWarning(code); },
      py::arg("code"), "True iff a warning with this code is present.")
    .def("summary", &ValidationReport::summary, "Every message, errors first, one per line.")
    .def(
      "raise_on_error", &ValidationReport::throwOnError,
      "Raises ConfigError listing every error; does nothing when ok.")
    .def("__repr__", [](const ValidationReport & r) {
      return "ValidationReport(errors=" + std::to_string(r.errors.size()) +
             ", warnings=" + std::to_string(r.warnings.size()) + ")";
    });

  m.def(
    "validate_config", &vf::validateConfig, py::arg("config"), py::arg("costmap_info") = py::none(),
    "The cross-parameter validator of math_spec section 11: domain checks, then (with auto "
    "values resolved) every reject and warn rule; costmap rules only with costmap_info.");
  m.def(
    "advise_for_plan", &vf::adviseForPlan, py::arg("config"), py::arg("kappa_path"),
    "Per-plan warnings for a path curvature kappa_path (math_spec section 11).");
  m.def(
    "validate_curve_options", &vf::validateCurveOptions, py::arg("options"),
    "Domain checks of CurveOptions (code invalid_curve_options).");
  m.def(
    "resolve_auto_values", &vf::resolveAutoValues, py::arg("config"),
    "A copy with every configure-time auto value filled (final_approach_distance stays auto).");
  m.def(
    "reverse_speed_limit", &vf::reverseSpeedLimit, py::arg("config"),
    "v_rev: max_reverse_vel, or max_linear_vel when auto.");
  m.def(
    "reference_speed", &vf::referenceSpeed, py::arg("config"), py::arg("drive_sign"),
    "Field speed of drive sign q: desired_speed, min(desired_speed, v_rev) for q < 0 (not omni).");
  m.def(
    "final_approach_distance", &vf::finalApproachDistance, py::arg("config"), py::arg("xy_tol"),
    "FINAL_APPROACH entry distance for a goal XY tolerance.");
  m.def(
    "evaluation_offset", &vf::evaluationOffset, py::arg("config"),
    "Evaluation-point offset of the validation rules: d for FL, |x_r| for Ackermann, else 0.");
  m.def(
    "auto_contour_distance", &vf::autoContourDistance, py::arg("radii"),
    py::arg("evaluation_offset"), "The auto contour distance lambda.");
  m.def(
    "ackermann_auto_contour_distance", &vf::ackermannAutoContourDistance, py::arg("radii"),
    py::arg("evaluation_offset"), py::arg("min_turning_radius"),
    "The Ackermann auto contour distance: max(auto lambda, R_min + w_lat + 0.05).");
  m.def(
    "compute_footprint_radii", &vf::computeFootprintRadii, py::arg("footprint"),
    "FootprintRadii of a FootprintSpec.");
  m.def(
    "footprint_is_valid",
    [](const vf::FootprintSpec & spec) {
      std::string reason;
      const bool valid = vf::footprintIsValid(spec, &reason);
      return py::make_tuple(valid, reason);
    },
    py::arg("footprint"), "(valid, reason) for a FootprintSpec.");
  m.def(
    "is_valid_combination", &vf::isValidCombination, py::arg("model"), py::arg("law"),
    "True for the valid (motion_model, tracking_law) pairs; AUTO is valid for every model.");
  m.def(
    "resolve_tracking_law", &vf::resolveTrackingLaw, py::arg("model"), py::arg("law"),
    "Resolves AUTO per motion model; other values are returned unchanged.");

  const py::module_ issue = m.def_submodule("issue", "Stable issue codes of the validators.");
  const std::pair<const char *, std::string_view> codes[] = {
    {"SPEED_ABOVE_MAX", vf::issue::kSpeedAboveMax},
    {"IN_PLACE_ABOVE_ROTATE", vf::issue::kInPlaceAboveRotate},
    {"CONTOUR_ORDER", vf::issue::kContourOrder},
    {"CONTOUR_BELOW_INSCRIBED", vf::issue::kContourBelowInscribed},
    {"INVALID_LAW", vf::issue::kInvalidLaw},
    {"ACKERMANN_CONTOUR_RADIUS", vf::issue::kAckermannContourRadius},
    {"ACKERMANN_RADIUS", vf::issue::kAckermannRadius},
    {"BRAKING_EXCEEDS_WINDOW", vf::issue::kBrakingExceedsWindow},
    {"FL_OFFSET", vf::issue::kFlOffset},
    {"REVERSE_ABOVE_MAX", vf::issue::kReverseAboveMax},
    {"ACKERMANN_REVERSE_BELOW_BAND", vf::issue::kAckermannReverseBelowBand},
    {"ROTATE_ABOVE_MAX", vf::issue::kRotateAboveMax},
    {"DISCRETE_OVERSHOOT", vf::issue::kDiscreteOvershoot},
    {"FL_GAIN_OFFSET", vf::issue::kFlGainOffset},
    {"BRAKING_TOO_SHORT", vf::issue::kBrakingTooShort},
    {"CONTOUR_BELOW_CIRCUMSCRIBED", vf::issue::kContourBelowCircumscribed},
    {"BLEND_EXCEEDS_WINDOW", vf::issue::kBlendExceedsWindow},
    {"HORIZON_TOO_SHORT", vf::issue::kHorizonTooShort},
    {"HORIZON_EXCEEDS_WINDOW", vf::issue::kHorizonExceedsWindow},
    {"ACKERMANN_CURVATURE", vf::issue::kAckermannCurvature},
    {"YAW_RATE_FOR_CURVATURE", vf::issue::kYawRateForCurvature},
    {"FL_PATH_CURVATURE", vf::issue::kFlPathCurvature},
    {"ACKERMANN_CONTOUR_CURVATURE", vf::issue::kAckermannContourCurvature},
    {"ACKERMANN_CONTOUR_TURN_IN", vf::issue::kAckermannContourTurnIn},
    {"ACKERMANN_CONTOUR_LOOKAHEAD", vf::issue::kAckermannContourLookahead},
    {"ACKERMANN_CONTOUR_WATCHDOG", vf::issue::kAckermannContourWatchdog},
    {"CBF_GAIN_DECELERATION", vf::issue::kCbfGainDeceleration},
    {"CBF_GAIN_DISCRETE", vf::issue::kCbfGainDiscrete},
    {"ACKERMANN_CBF_WINDOW", vf::issue::kAckermannCbfWindow},
    {"ACKERMANN_CBF_MARGIN", vf::issue::kAckermannCbfMargin},
    {"INVALID_VALUE", vf::issue::kInvalidValue},
    {"INVALID_FOOTPRINT", vf::issue::kInvalidFootprint},
    {"INVALID_COSTMAP", vf::issue::kInvalidCostmap},
    {"INVALID_CURVE_OPTIONS", vf::issue::kInvalidCurveOptions},
  };
  for (const auto & [name, code] : codes) {
    issue.attr(name) = py::str(std::string(code));
  }
}

}  // namespace

void bindConfig(py::module_ & m)
{
  bindEnums(m);
  bindGroups(m);
  bindValidation(m);
}

}  // namespace vectorfield_py
