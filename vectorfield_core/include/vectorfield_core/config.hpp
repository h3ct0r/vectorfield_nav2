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

/// @file
/// @brief Configuration of the core: enums, `CoreConfig`, `CurveOptions`, auto values and the
/// cross-parameter validator.
///
/// `CoreConfig` mirrors the parameter groups of plan §5.4 (Field, Motion, Phases, Obstacles,
/// Safety) field by field, with the same names and defaults, plus the node-derived fields
/// (controller frequency, footprint) and the follower's goal-scoped fields (direction,
/// assumption3_margin). The vectorfield_ros binding tables (`kCommonCoreParams`,
/// `kFollowerCoreParams`) are the only name↔field mapping; the Visualization and Legacy groups
/// are ROS-only and have no core field.
///
/// **Auto sentinels.** A negative value of `final_approach_distance`, `obstacle_contour_distance`,
/// `obstacle_full_switch_distance`, `obstacle_blend_start_distance` or `obstacle_lookahead`
/// selects the auto value (−1 is the canonical sentinel, see kAutoValue). `tracking_law: auto`
/// resolves per motion model. resolveAutoValues() fills every auto value that is known at
/// configure time; `final_approach_distance` depends on the goal's xy_tol and is resolved per
/// cycle by finalApproachDistance().
///
/// **Evaluation offset.** Wherever math_spec §11 writes \f$d\f$ in a rule that is about the
/// law's evaluation point (the auto \f$\lambda\f$, \f$\lambda - d < r_{\mathrm{insc}}\f$,
/// \f$\lambda - d < r_{\mathrm{circ}}\f$, \f$D'_{\mathrm{in}} + d\f$, \f$\sigma_b + d + 0.1\f$),
/// the core uses evaluationOffset(): \f$d\f$ for feedback linearisation (where it is the
/// control-point offset, so every documented value is unchanged), \f$\lvert x_r\rvert\f$ for
/// Ackermann and 0 for heading tracking and holonomic, whose field is evaluated at base_link
/// (math_spec §7.1). The FL-only rules (\f$d \le 0\f$, \f$G'(0)\,d > 0.5\f$,
/// \f$\kappa_{\mathrm{path}}\,d > 1\f$) always use `control_point_offset`.

#ifndef VECTORFIELD_CORE__CONFIG_HPP_
#define VECTORFIELD_CORE__CONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @name Enumerations (parameter values; the first listed value is the default)
/// @{

/// @brief `gain_function` (math_spec §3, sec-gains).
enum class GainType : std::uint8_t
{
  /// "atan": \f$G = \frac{2}{\pi}\arctan(k_f D)\f$ (thesis experiments).
  kAtan = 0,
  /// "rational": \f$G = k_f D/\sqrt{1 + (k_f D)^2}\f$.
  kRational = 1,
};

/// @brief `motion_model` (read-only parameter).
enum class MotionModel : std::uint8_t
{
  /// "differential".
  kDifferential = 0,
  /// "skid_steer".
  kSkidSteer = 1,
  /// "omnidirectional".
  kOmnidirectional = 2,
  /// "ackermann".
  kAckermann = 3,
};

/// @brief `tracking_law` (math_spec §7, sec-laws). Valid combinations are listed at
/// isValidCombination().
enum class TrackingLaw : std::uint8_t
{
  /// "auto": feedback_linearization for diff/skid, holonomic for omni, heading_tracking for
  /// Ackermann (plan §3.1).
  kAuto = 0,
  /// "feedback_linearization" (thesis eq. 2.1; math_spec §7.3).
  kFeedbackLinearization = 1,
  /// "heading_tracking" (math_spec §7.4).
  kHeadingTracking = 2,
  /// "holonomic" (math_spec §7.2).
  kHolonomic = 3,
};

/// @brief `omni_heading_policy` (math_spec §7.2, sec-laws-holo).
enum class OmniHeadingPolicy : std::uint8_t
{
  /// "path_tangent": face \f$T_d\f$.
  kPathTangent = 0,
  /// "field": face \f$F\f$ (held while \f$\lVert F\rVert < 0.05\,v_r\f$).
  kField = 1,
  /// "fixed": \f$\omega = 0\f$.
  kFixed = 2,
};

/// @brief `obstacle_mode` (math_spec §6, sec-contour).
enum class ObstacleMode : std::uint8_t
{
  /// "none": obstacles only stop the robot (collision prediction).
  kNone = 0,
  /// "contour": circulate around blocking obstacles.
  kContour = 1,
};

/// @brief `obstacle_activation` (math_spec §6.4, sec-contour-mods).
enum class ObstacleActivation : std::uint8_t
{
  /// "path_blocked": the gate opens only when the path ahead is blocked.
  kPathBlocked = 0,
  /// "always": the thesis behaviour.
  kAlways = 1,
};
/// @}

/// @name Enum ↔ parameter string
/// The strings are the parameter values of plan §5.4 ("atan", "skid_steer", ...).
/// `fromString` returns false and leaves `*out` unchanged for an unknown string.
/// @{
const char * toString(GainType value) noexcept;
const char * toString(MotionModel value) noexcept;
const char * toString(TrackingLaw value) noexcept;
const char * toString(OmniHeadingPolicy value) noexcept;
const char * toString(ObstacleMode value) noexcept;
const char * toString(ObstacleActivation value) noexcept;
bool fromString(std::string_view text, GainType * out) noexcept;
bool fromString(std::string_view text, MotionModel * out) noexcept;
bool fromString(std::string_view text, TrackingLaw * out) noexcept;
bool fromString(std::string_view text, OmniHeadingPolicy * out) noexcept;
bool fromString(std::string_view text, ObstacleMode * out) noexcept;
bool fromString(std::string_view text, ObstacleActivation * out) noexcept;
/// @}

/// @brief Canonical "auto" sentinel. Any negative value of an auto-capable distance selects auto.
inline constexpr double kAutoValue = -1.0;

/// @brief True iff `value` selects the auto value (it is negative). NaN is not auto.
inline bool isAuto(double value) noexcept { return value < 0.0; }

/// @brief Field group of plan §5.4, plus the follower's goal-scoped direction and
/// Assumption-3 margin.
struct FieldConfig
{
  /// `desired_speed` \f$v_r\f$ (m/s). The follower overrides it per goal when the goal's
  /// desired_speed > 0.
  double desired_speed{0.4};
  /// `convergence_gain` \f$k_f\f$ (1/m).
  double convergence_gain{2.0};
  /// `gain_function`.
  GainType gain_function{GainType::kAtan};
  /// Traversal direction \f$\mathrm{dir} = \pm 1\f$ (follower goal: `reverse_direction` → −1;
  /// plugin: +1). math_spec §1.
  int direction{1};
  /// `assumption3_margin` \f$\varepsilon_v\f$ (follower-only key; plugin curves are static).
  double assumption3_margin{0.1};
};

/// @brief Motion group of plan §5.4.
struct MotionConfig
{
  /// `motion_model` (read-only parameter).
  MotionModel motion_model{MotionModel::kDifferential};
  /// `tracking_law`.
  TrackingLaw tracking_law{TrackingLaw::kAuto};
  /// `control_point_offset` \f$d\f$ (m), FL control point ahead of base_link.
  double control_point_offset{0.2};
  /// `drive_backward`: \f$q = -1\f$ (math_spec §1).
  bool drive_backward{false};
  /// `allow_reversing`: when false, \f$q\,v \ge 0\f$ is enforced (math_spec §9.2 Backstop).
  bool allow_reversing{false};
  /// `heading_gain` \f$k_\psi\f$ (1/s).
  double heading_gain{2.0};
  /// `omni_heading_policy`.
  OmniHeadingPolicy omni_heading_policy{OmniHeadingPolicy::kPathTangent};
  /// `max_linear_vel` \f$v_{\max}\f$ (m/s).
  double max_linear_vel{0.5};
  /// `max_lateral_vel` \f$v_{y,\max}\f$ (m/s); omnidirectional only.
  double max_lateral_vel{0.5};
  /// `max_angular_vel` \f$\omega_{\max}\f$ (rad/s).
  double max_angular_vel{1.5};
  /// `max_linear_accel` \f$a_{\mathrm{acc}}\f$ (m/s²).
  double max_linear_accel{1.0};
  /// `max_linear_decel` \f$a_{\mathrm{dec}}\f$ (m/s², positive).
  double max_linear_decel{1.0};
  /// `max_angular_accel` \f$\alpha_{\max}\f$ (rad/s²).
  double max_angular_accel{3.2};
  /// `min_in_place_angular_vel` \f$\omega_{\mathrm{floor}}\f$ (rad/s); skid: 0.25–0.3.
  double min_in_place_angular_vel{0.0};
  /// `min_turning_radius` \f$R_{\min}\f$ (m); Ackermann requires > 0.
  double min_turning_radius{0.0};
  /// `ackermann_min_speed` \f$v_{\mathrm{ack}}\f$ (m/s).
  double ackermann_min_speed{0.05};
  /// `reference_point_offset` \f$x_r\f$ (m): body-x of the rear-axle centre relative to
  /// base_link; negative = behind.
  double reference_point_offset{0.0};
};

/// @brief Phases group of plan §5.4.
struct PhaseConfig
{
  /// `use_rotate_to_field`.
  bool use_rotate_to_field{true};
  /// `rotate_to_field_angle` (rad): entry when \f$\lvert e\rvert\f$ exceeds it.
  double rotate_to_field_angle{1.57};
  /// `rotate_to_field_release_angle` (rad): exit when \f$\lvert e\rvert\f$ drops below it.
  double rotate_to_field_release_angle{0.785};
  /// `use_rotate_to_heading`.
  bool use_rotate_to_heading{true};
  /// `rotate_to_heading_angular_vel` \f$\omega_{\mathrm{rth}}\f$ (rad/s).
  double rotate_to_heading_angular_vel{1.5};
  /// `final_approach_distance` (m); negative = auto, see finalApproachDistance().
  double final_approach_distance{kAutoValue};
  /// `terminal_braking_distance` \f$\sigma_b\f$ (m).
  double terminal_braking_distance{0.5};
  /// `min_approach_speed` \f$v_{\min}\f$ (m/s).
  double min_approach_speed{0.05};
  /// `cancel_deceleration` \f$a_{\mathrm{can}}\f$ (m/s²).
  double cancel_deceleration{1.0};
};

/// @brief Obstacles group of plan §5.4.
struct ObstacleConfig
{
  /// `obstacle_mode` (demos: contour).
  ObstacleMode obstacle_mode{ObstacleMode::kNone};
  /// `obstacle_activation`.
  ObstacleActivation obstacle_activation{ObstacleActivation::kPathBlocked};
  /// `obstacle_contour_distance` \f$\lambda\f$ (m); negative = auto (autoContourDistance()).
  double obstacle_contour_distance{kAutoValue};
  /// `obstacle_full_switch_distance` \f$D_{\mathrm{in}}\f$ (m); negative = auto
  /// \f$\lambda + 0.15\f$.
  double obstacle_full_switch_distance{kAutoValue};
  /// `obstacle_blend_start_distance` \f$D'_{\mathrm{in}}\f$ (m); negative = auto
  /// \f$\lambda + 0.35\f$.
  double obstacle_blend_start_distance{kAutoValue};
  /// `obstacle_lookahead` \f$\ell_{\mathrm{look}}\f$ (m); negative = auto
  /// \f$\max(1.0, v_r T_h)\f$.
  double obstacle_lookahead{kAutoValue};
  /// `obstacle_gate_tolerance` (m).
  double obstacle_gate_tolerance{0.05};
  /// `obstacle_release_time` (s).
  double obstacle_release_time{0.5};
  /// `obstacle_max_contour_length` (m), watchdog.
  double obstacle_max_contour_length{5.0};
  /// `obstacle_max_contour_time` (s), watchdog.
  double obstacle_max_contour_time{20.0};
  /// `unknown_is_obstacle`: unknown cells are sites (applied when the ROS layer builds the mask).
  bool unknown_is_obstacle{false};
  /// `clearance_slowdown_distance` \f$d_{\mathrm{cl}}\f$ (m); 0 = off.
  double clearance_slowdown_distance{0.0};
  /// `clearance_slowdown_gain` \f$k_{\mathrm{cl}}\f$.
  double clearance_slowdown_gain{1.0};
};

/// @brief Safety group of plan §5.4.
struct SafetyConfig
{
  /// `use_collision_detection`.
  bool use_collision_detection{true};
  /// `collision_check_horizon` \f$T_h\f$ (s).
  double collision_check_horizon{1.5};
  /// `stop_margin` \f$m_s\f$ (m).
  double stop_margin{0.10};
};

/// @brief Robot footprint in the robot (base_link) frame: a polygon, or a circle when the
/// polygon is empty.
///
/// A radius of 0 with no polygon is a point robot. A non-empty polygon needs at least three
/// finite vertices and a non-zero area; it may be convex or not.
struct FootprintSpec
{
  /// Polygon vertices (m, robot frame), in either orientation. Empty: use `radius`.
  std::vector<Vec2> polygon;
  /// Circle radius (m), used when `polygon` is empty (Nav2 `robot_radius`).
  double radius{0.22};
};

/// @brief Footprint radii of math_spec §10.1 (sec-footprint).
struct FootprintRadii
{
  /// \f$w_{\mathrm{lat}}\f$: max \f$\lvert y\rvert\f$ over the polygon's vertices (radius if
  /// circular).
  double w_lat{0.0};
  /// \f$r_{\mathrm{insc}}\f$: distance from base_link to the nearest polygon edge (radius if
  /// circular); 0 when base_link is not strictly inside the polygon.
  double r_insc{0.0};
  /// \f$r_{\mathrm{circ}}\f$: distance from base_link to the farthest vertex (radius if
  /// circular).
  double r_circ{0.0};
};

/// @brief Node-derived configuration: set by the ROS layer after the parameter tables
/// (plan §4, "Node-derived fields").
///
/// The defaults are nav2_bringup's simulated robot: 20 Hz and `robot_radius` 0.22 m.
struct NodeConfig
{
  /// Controller frequency \f$f\f$ (Hz): the controller_server's in the plugin (read at
  /// configure), the follower's own `controller_frequency`. The nominal control period is 1/f.
  double controller_frequency{20.0};
  /// Robot footprint (plugin: `getRobotFootprint()`; follower: `published_footprint`).
  FootprintSpec footprint;
};

/// @brief Complete configuration of a `Tracker`.
///
/// Default-constructed, it equals the plan §5.4 defaults with the TB3 (nav2_bringup) footprint and
/// passes validateConfig() with no error and no warning.
struct CoreConfig
{
  /// Field group.
  FieldConfig field;
  /// Motion group.
  MotionConfig motion;
  /// Phases group.
  PhaseConfig phases;
  /// Obstacles group.
  ObstacleConfig obstacles;
  /// Safety group.
  SafetyConfig safety;
  /// Node-derived fields.
  NodeConfig node;
};

/// @brief Curve-intake options of the follower (plan §6 parameters), validated by `makeCurve`.
struct CurveOptions
{
  /// `max_curve_points`: maximum vertex count after conditioning.
  int max_curve_points{20000};
  /// `default_sample_spacing` (m): POLYLINE resampling when the goal's sample_spacing <= 0.
  double default_sample_spacing{0.05};
  /// `search_window` \f$w\f$ (m): hinted local search window (math_spec §2.4).
  double search_window{1.0};
  /// `global_search_trigger` \f$D_{\mathrm{trig}}\f$ (m).
  double global_search_trigger{0.5};
  /// `global_search_hysteresis` \f$h_g\f$ (m).
  double global_search_hysteresis{0.10};
  /// `global_search_samples` \f$M\f$.
  int global_search_samples{400};
};

/// @brief Costmap geometry for the costmap-dependent rules (plan §4 "CostmapInfo").
struct CostmapInfo
{
  /// Width \f$W\f$ (m).
  double width_m{0.0};
  /// Height \f$H_c\f$ (m).
  double height_m{0.0};
  /// Resolution (m per cell).
  double resolution{0.05};
  /// Gen A/B `prune_distance`; nullopt on Gen C and in the follower.
  std::optional<double> window_limit;
};

/// @brief One finding of a validator.
struct ValidationIssue
{
  /// Stable machine-readable code; one of the constants in namespace `issue`.
  std::string code;
  /// Human-readable explanation naming the parameters and values involved.
  std::string message;
};

/// @brief Result of a validator: errors reject the configuration, warnings only advise.
struct ValidationReport
{
  /// Rejections.
  std::vector<ValidationIssue> errors;
  /// Advice.
  std::vector<ValidationIssue> warnings;

  /// @brief True iff there is no error (warnings allowed).
  bool ok() const noexcept { return errors.empty(); }
  /// @brief True iff an error with this code is present.
  bool hasError(std::string_view code) const noexcept;
  /// @brief True iff a warning with this code is present.
  bool hasWarning(std::string_view code) const noexcept;
  /// @brief All messages, errors first, one per line ("error [code]: message").
  std::string summary() const;
  /// @brief Throws ConfigError listing every error message; does nothing when ok().
  void throwOnError() const;
};

/// @brief Stable issue codes reported by the validators of this header.
namespace issue
{
/// @name Errors of math_spec §11 (sec-math-validation)
/// @{
/// \f$v_r > v_{\max}\f$.
inline constexpr std::string_view kSpeedAboveMax = "desired_speed_above_max_linear_vel";
/// `min_in_place_angular_vel` > `rotate_to_heading_angular_vel`.
inline constexpr std::string_view kInPlaceAboveRotate = "min_in_place_above_rotate_vel";
/// Not \f$\lambda < D_{\mathrm{in}} \le D'_{\mathrm{in}}\f$ (contour only).
inline constexpr std::string_view kContourOrder = "contour_distance_order";
/// \f$\lambda - d < r_{\mathrm{insc}}\f$ (contour only).
inline constexpr std::string_view kContourBelowInscribed = "contour_distance_below_inscribed";
/// Invalid (motion_model, tracking_law) pair.
inline constexpr std::string_view kInvalidLaw = "invalid_tracking_law";
/// Ackermann with obstacle_mode contour (unsupported in 0.x).
inline constexpr std::string_view kAckermannContour = "ackermann_contour_unsupported";
/// Ackermann with \f$R_{\min} \le 0\f$.
inline constexpr std::string_view kAckermannRadius = "ackermann_min_turning_radius";
/// \f$\sigma_b + d + 0.1 > \tfrac12\min(W, H_c)\f$ or > window_limit (costmap).
inline constexpr std::string_view kBrakingExceedsWindow = "braking_distance_exceeds_window";
/// FL with \f$d \le 0\f$.
inline constexpr std::string_view kFlOffset = "fl_control_point_offset";
/// @}

/// @name Warnings of math_spec §11 (sec-math-validation)
/// @{
/// `rotate_to_heading_angular_vel` > \f$\omega_{\max}\f$.
inline constexpr std::string_view kRotateAboveMax = "rotate_vel_above_max_angular_vel";
/// \f$G'(0)\,v_r\,\Delta t \ge 1\f$.
inline constexpr std::string_view kDiscreteOvershoot = "discrete_overshoot";
/// FL: \f$G'(0)\,d > 0.5\f$.
inline constexpr std::string_view kFlGainOffset = "fl_gain_offset";
/// \f$\sigma_b < v_r^2/a_{\mathrm{dec}}\f$.
inline constexpr std::string_view kBrakingTooShort = "terminal_braking_too_short";
/// \f$\lambda - d < r_{\mathrm{circ}}\f$ (contour only).
inline constexpr std::string_view kContourBelowCircumscribed =
  "contour_distance_below_circumscribed";
/// \f$D'_{\mathrm{in}} + d \ge \tfrac12\min(W, H_c) - 0.2\f$ (contour and costmap).
inline constexpr std::string_view kBlendExceedsWindow = "blend_zone_exceeds_window";
/// \f$T_h v_{\max} < v_{\max}^2/(2a_{\mathrm{dec}}) + v_{\max}(2/f + 0.1) + 0.1\f$.
inline constexpr std::string_view kHorizonTooShort = "horizon_too_short";
/// \f$T_h v_{\max} > \tfrac12\min(W, H_c) - r_{\mathrm{circ}}\f$ (costmap).
inline constexpr std::string_view kHorizonExceedsWindow = "horizon_exceeds_window";
/// Ackermann: \f$R_{\min}(c_G k_f + \kappa_{\mathrm{path}}) > 1\f$.
inline constexpr std::string_view kAckermannCurvature = "ackermann_curvature";
/// \f$v_r(c_G k_f + \kappa_{\mathrm{path}}) > \omega_{\max}\f$.
inline constexpr std::string_view kYawRateForCurvature = "yaw_rate_for_curvature";
/// Per plan (adviseForPlan()): FL with \f$\kappa_{\mathrm{path}}\,d > 1\f$.
inline constexpr std::string_view kFlPathCurvature = "fl_path_curvature";
/// @}

/// @name Domain errors (not in math_spec §11)
/// Parameter-domain checks that the generate_parameter_library validators enforce in ROS; they
/// are repeated here so that every caller (Python, tests) gets the same protection. When any of
/// them fails, validateConfig() reports only the domain errors, because the §11 formulas would
/// be meaningless.
/// @{
/// A parameter is non-finite or outside its domain; the message names it.
inline constexpr std::string_view kInvalidValue = "invalid_value";
/// The footprint polygon is invalid (fewer than 3 vertices, non-finite, zero area) or the
/// radius is negative.
inline constexpr std::string_view kInvalidFootprint = "invalid_footprint";
/// The CostmapInfo is invalid (non-positive size or resolution, non-positive window_limit).
inline constexpr std::string_view kInvalidCostmap = "invalid_costmap";
/// A CurveOptions value is out of its domain.
inline constexpr std::string_view kInvalidCurveOptions = "invalid_curve_options";
/// @}
}  // namespace issue

/// @brief Valid (motion_model, tracking_law) pairs (plan §3.1; math_spec §7.1): diff and skid
/// with feedback_linearization or heading_tracking; omni with holonomic; Ackermann with
/// heading_tracking. `kAuto` is valid for every model.
bool isValidCombination(MotionModel model, TrackingLaw law) noexcept;

/// @brief Resolves `kAuto` per motion model (diff/skid → FL, omni → holonomic, Ackermann →
/// heading_tracking); any other value is returned unchanged, valid or not.
TrackingLaw resolveTrackingLaw(MotionModel model, TrackingLaw law) noexcept;

/// @brief Footprint radii \f$w_{\mathrm{lat}}, r_{\mathrm{insc}}, r_{\mathrm{circ}}\f$ of
/// math_spec §10.1 (sec-footprint).
///
/// For a polygon: \f$w_{\mathrm{lat}} = \max\lvert y\rvert\f$ and \f$r_{\mathrm{circ}} =
/// \max\lVert v\rVert\f$ over the vertices; \f$r_{\mathrm{insc}}\f$ = distance from the origin to
/// the nearest edge if the origin is strictly inside the polygon, else 0. For a circle all three
/// equal the radius. An invalid spec gives the radii of whatever it contains (check
/// footprintIsValid() first).
FootprintRadii computeFootprintRadii(const FootprintSpec & footprint) noexcept;

/// @brief True iff the footprint is usable: radius finite and >= 0 when the polygon is empty;
/// otherwise at least 3 finite vertices and a non-zero area.
/// @param footprint Footprint to check.
/// @param reason If not null and invalid, receives a short explanation.
bool footprintIsValid(const FootprintSpec & footprint, std::string * reason = nullptr) noexcept;

/// @brief Evaluation-point offset used by the evaluation-point rules of math_spec §11: the
/// `control_point_offset` \f$d\f$ for feedback linearisation, \f$\lvert x_r\rvert\f$ for Ackermann
/// and 0 for heading tracking (diff/skid) and holonomic (see the file comment).
/// `tracking_law: auto` is resolved first.
double evaluationOffset(const CoreConfig & config) noexcept;

/// @brief Auto contour distance (math_spec §11 auto values, §6.5 sec-contour-notes):
/// \f$\lambda = \max\big(\sqrt{(w_{\mathrm{lat}} + 0.10)^2 + d^2},\ r_{\mathrm{circ}} + d +
/// 0.05\big)\f$ with \f$d\f$ = evaluationOffset() (TB3 with FL: 0.47).
double autoContourDistance(const FootprintRadii & radii, double evaluation_offset) noexcept;

/// @brief A copy of `config` with every configure-time auto value filled.
///
/// Fills `tracking_law` (resolveTrackingLaw), \f$\lambda\f$, \f$D_{\mathrm{in}} = \lambda +
/// 0.15\f$, \f$D'_{\mathrm{in}} = \lambda + 0.35\f$ and `obstacle_lookahead` \f$= \max(1.0, v_r
/// T_h)\f$, each only when auto (math_spec §11). `final_approach_distance` stays auto: it needs
/// the goal's xy_tol (finalApproachDistance()). The footprint radii come from
/// computeFootprintRadii(). No validation is done.
/// @throws std::bad_alloc only (the footprint polygon is copied).
CoreConfig resolveAutoValues(const CoreConfig & config);

/// @brief FINAL_APPROACH entry distance for a goal tolerance (math_spec §9.2, §11): the explicit
/// `final_approach_distance` when set; auto: \f$\max(2d,\ \mathrm{xy\_tol} + d)\f$ for feedback
/// linearisation, xy_tol otherwise (TB3, xy_tol 0.25: 0.45).
/// @param config Configuration (tracking_law may be auto).
/// @param xy_tol Goal XY tolerance of this cycle (`GoalInfo::xy_tol`).
double finalApproachDistance(const CoreConfig & config, double xy_tol) noexcept;

/// @brief The cross-parameter validator: math_spec §11 (sec-math-validation), exactly.
///
/// Steps:
/// 1. Domain checks (issue::kInvalidValue, kInvalidFootprint, kInvalidCostmap). If any
///    parameter fails, only these errors are returned.
/// 2. resolveAutoValues(), then every reject and warn rule of §11 with
///    \f$\Delta t = 1/f\f$, \f$\kappa_{\mathrm{path}} = 0\f$ (configure time, Appendix C item 16)
///    and \f$d\f$ per evaluationOffset() where the rule concerns the evaluation point.
///    Rules with \f$\lambda\f$, \f$D_{\mathrm{in}}\f$, \f$D'_{\mathrm{in}}\f$ or the lookahead
///    apply only with `obstacle_mode: contour`; costmap rules only when `costmap` is given
///    (and valid).
///
/// The defaults (CoreConfig{}) pass with no error and no warning, in both obstacle modes, with a
/// 3×3 m / 0.05 m costmap.
/// @param config Configuration to check (auto values allowed).
/// @param costmap Costmap geometry captured at configure, if known.
ValidationReport validateConfig(
  const CoreConfig & config, const std::optional<CostmapInfo> & costmap) noexcept;

/// @brief Per-plan advisor (math_spec §11 "Runtime, once per plan" and the two
/// \f$\kappa_{\mathrm{path}}\f$ warn rules with the window's value): FL with
/// \f$\kappa_{\mathrm{path}}\,d > 1\f$ (issue::kFlPathCurvature); Ackermann with
/// \f$R_{\min}(c_G k_f + \kappa_{\mathrm{path}}) > 1\f$ (kAckermannCurvature);
/// \f$v_r(c_G k_f + \kappa_{\mathrm{path}}) > \omega_{\max}\f$ (kYawRateForCurvature).
/// Warnings only. The unsmoothed-plan and \f$\sigma_{\mathrm{end}} < \sigma_b\f$ checks need
/// the plan itself and live in the controller (M2).
/// @param config Configuration (auto values allowed).
/// @param kappa_path \f$\kappa_{\mathrm{path}}\f$ of the conditioned window (conditioning.hpp
/// `kappaPath`).
ValidationReport adviseForPlan(const CoreConfig & config, double kappa_path) noexcept;

/// @brief Domain checks of CurveOptions, used by `makeCurve` (errors with code
/// issue::kInvalidCurveOptions): max_curve_points >= 2, default_sample_spacing > 0,
/// search_window > 0, global_search_trigger >= 0, global_search_hysteresis >= 0,
/// global_search_samples >= 4 (the regularity check needs adjacent samples less than 90° apart
/// on a circle), all finite.
ValidationReport validateCurveOptions(const CurveOptions & options) noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__CONFIG_HPP_
