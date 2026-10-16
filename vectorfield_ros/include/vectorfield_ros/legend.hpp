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
/// @brief The visualization legend: marker namespaces, their stable ids and their colours
/// (plan §4.1). **The single source of truth** shared by MarkerBuilder, the RViz/Foxglove configs
/// of vectorfield_bringup (M3) and docs/visualization.md (M6); change them together.
///
/// Every `<ns>/markers` message contains exactly one marker per (namespace, id) listed here:
/// `ADD` when it applies this cycle, `DELETE` otherwise. The output is therefore a pure function
/// of the cycle and never leaves stale markers behind; a `DELETEALL` is sent only on goal end,
/// reset and deactivation. The total is kMarkerCount (≤ 80, plan §9 layer 2).
///
/// Header-only data; constexpr everywhere.

#ifndef VECTORFIELD_ROS__LEGEND_HPP_
#define VECTORFIELD_ROS__LEGEND_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "vectorfield_core/status.hpp"

namespace vectorfield_ros
{

/// @brief Marker namespaces, ids and colours.
namespace legend
{

/// @brief An RGBA colour with components in [0, 1] (std_msgs/ColorRGBA).
struct Rgba
{
  /// Red.
  float r;
  /// Green.
  float g;
  /// Blue.
  float b;
  /// Alpha (1 = opaque).
  float a;
};

/// @brief Exact equality (for tests).
constexpr bool operator==(const Rgba & lhs, const Rgba & rhs) noexcept
{
  return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b && lhs.a == rhs.a;
}

/// @name Palette
/// @{
/// Green: FOLLOW, normal footprint, collision-free prediction, circulation arrow.
inline constexpr Rgba kGreen{0.20F, 0.80F, 0.20F, 1.0F};
/// Yellow: ROTATE_TO_FIELD, closest point, contour D_in and D'_in rings.
inline constexpr Rgba kYellow{1.00F, 0.85F, 0.10F, 1.0F};
/// Cyan: FINAL_APPROACH, command, goal rings and goal yaw.
inline constexpr Rgba kCyan{0.00F, 0.85F, 0.90F, 1.0F};
/// Blue: ROTATE_TO_HEADING, feedforward arrow, moving-frame velocity.
inline constexpr Rgba kBlue{0.20F, 0.45F, 1.00F, 1.0F};
/// Red: STOPPED, blocked footprint, convergence arrow, collision, contour obstacle.
inline constexpr Rgba kRed{0.90F, 0.10F, 0.10F, 1.0F};
/// Orange: speed-capped footprint, braking zone, contour lambda ring and side arrow.
inline constexpr Rgba kOrange{1.00F, 0.55F, 0.00F, 1.0F};
/// White: curve, resulting field, goal, curve_info.
inline constexpr Rgba kWhite{1.00F, 1.00F, 1.00F, 1.0F};
/// Grey: extension rays and the virtual tail vertex.
inline constexpr Rgba kGrey{0.60F, 0.60F, 0.60F, 1.0F};
/// Magenta: control point (where the field is evaluated).
inline constexpr Rgba kMagenta{0.90F, 0.20F, 0.90F, 1.0F};
/// Translucent red: the cells of the contoured obstacle component K.
inline constexpr Rgba kTranslucentRed{0.90F, 0.10F, 0.10F, 0.35F};
/// @}

/// @brief Text colour of the `status` namespace per phase: FOLLOW green, ROTATE_TO_FIELD
/// yellow, FINAL_APPROACH cyan, ROTATE_TO_HEADING blue, STOPPED red.
constexpr Rgba phaseColor(vectorfield_core::Phase phase) noexcept
{
  switch (phase) {
    case vectorfield_core::Phase::kRotateToField:
      return kYellow;
    case vectorfield_core::Phase::kFollow:
      return kGreen;
    case vectorfield_core::Phase::kFinalApproach:
      return kCyan;
    case vectorfield_core::Phase::kRotateToHeading:
      return kBlue;
    case vectorfield_core::Phase::kStopped:
      return kRed;
  }
  return kWhite;
}

/// @name Colours per namespace (plan §4.1 table)
/// @{
/// footprint: normal.
inline constexpr Rgba kFootprintNormal = kGreen;
/// footprint: speed-capped (the collision cap or the clearance slowdown binds,
/// `SpeedShaping::collisionAhead()`).
inline constexpr Rgba kFootprintCapped = kOrange;
/// footprint: kBlocked.
inline constexpr Rgba kFootprintBlocked = kRed;
/// curve: the polyline the field uses.
inline constexpr Rgba kCurveColor = kWhite;
/// curve: extension rays and the virtual tail vertex.
inline constexpr Rgba kCurveRays = kGrey;
/// closest_point.
inline constexpr Rgba kClosestPointColor = kYellow;
/// control_point.
inline constexpr Rgba kControlPointColor = kMagenta;
/// field: convergence \f$-\eta G n\f$.
inline constexpr Rgba kFieldConvergence = kRed;
/// field: circulation \f$\eta H T_d\f$.
inline constexpr Rgba kFieldCirculation = kGreen;
/// field: feedforward \f$\Phi_T\f$.
inline constexpr Rgba kFieldFeedforward = kBlue;
/// field: the resulting field F (thick).
inline constexpr Rgba kFieldResult = kWhite;
/// command.
inline constexpr Rgba kCommandColor = kCyan;
/// heading: ROTATE_TO_FIELD target.
inline constexpr Rgba kHeadingToField = kYellow;
/// heading: ROTATE_TO_HEADING target.
inline constexpr Rgba kHeadingToGoal = kBlue;
/// prediction: collision-free part of the rollout and the s_free text.
inline constexpr Rgba kPredictionFree = kGreen;
/// prediction: the first colliding pose and the part after it.
inline constexpr Rgba kPredictionCollision = kRed;
/// goal: g and g_ref.
inline constexpr Rgba kGoalColor = kWhite;
/// goal: the braking zone (the last sigma_b of the curve).
inline constexpr Rgba kGoalBrakingZone = kOrange;
/// goal: the xy_tol and sigma_stop rings and the goal-yaw arrow.
inline constexpr Rgba kGoalTolerance = kCyan;
/// contour: K's cells.
inline constexpr Rgba kContourCells = kTranslucentRed;
/// contour: blocked gate samples, x*_o, the line to it and the text.
inline constexpr Rgba kContourObstacle = kRed;
/// contour: the lambda ring and the sigma_c side arrow.
inline constexpr Rgba kContourLambda = kOrange;
/// contour: the D_in and D'_in rings.
inline constexpr Rgba kContourBlend = kYellow;
/// curve_info: direction arrows, start and text.
inline constexpr Rgba kCurveInfoColor = kWhite;
/// curve_info: moving-frame velocity.
inline constexpr Rgba kCurveInfoFrame = kBlue;
/// @}

/// @brief The marker namespaces of `<ns>/markers`, in message order.
enum class Ns : std::uint8_t
{
  /// Text above base_link: phase, command, D, sigma, flags.
  kStatus = 0,
  /// Robot outline at the current pose.
  kFootprint,
  /// The curve the field uses, its rays and the virtual tail vertex.
  kCurve,
  /// Closest point, tangent and distance.
  kClosestPoint,
  /// Where the field is evaluated.
  kControlPoint,
  /// The field decomposition at the evaluation point.
  kField,
  /// The command and 1 s of motion under it.
  kCommand,
  /// Target heading in the rotate phases.
  kHeading,
  /// Collision prediction.
  kPrediction,
  /// Goal, tolerance rings, goal yaw, braking zone.
  kGoal,
  /// Obstacle contour (only while the gate is open or an episode is active).
  kContour,
  /// Follower only: direction, start, laps, moving frame.
  kCurveInfo,
};

/// @brief Number of namespaces.
inline constexpr std::size_t kNamespaceCount = 12;

/// @brief One namespace: its RViz name and its number of stable ids (0 .. id_count − 1).
struct NamespaceSpec
{
  /// The marker `ns` string.
  std::string_view name;
  /// Number of ids; every id appears in every message (ADD or DELETE).
  std::int32_t id_count;
};

/// @brief Stable ids per namespace. Each constant is the marker id within its namespace.
namespace id
{
/// status: TEXT_VIEW_FACING 0.6 m above base_link.
inline constexpr std::int32_t kStatusText = 0;

/// footprint: LINE_STRIP (closed) at the current pose.
inline constexpr std::int32_t kFootprintOutline = 0;

/// curve: LINE_STRIP of the curve (plugin: the conditioned window).
inline constexpr std::int32_t kCurveLine = 0;
/// curve: LINE_LIST dashes along the backward and forward extension rays (open curves).
inline constexpr std::int32_t kCurveRays = 1;
/// curve: small SPHERE at the virtual tail vertex (plugin, tail-conditioned windows).
inline constexpr std::int32_t kCurveVirtualVertex = 2;

/// closest_point: SPHERE at x*.
inline constexpr std::int32_t kClosestPoint = 0;
/// closest_point: ARROW of T at x*.
inline constexpr std::int32_t kClosestTangent = 1;
/// closest_point: LINE_LIST segment from the evaluation point to x* (length D).
inline constexpr std::int32_t kClosestDistance = 2;

/// control_point: SPHERE at the evaluation point (p_c for FL, the rear axle for Ackermann).
inline constexpr std::int32_t kControlPoint = 0;
/// control_point: LINE_LIST segment from base_link to the evaluation point.
inline constexpr std::int32_t kControlLine = 1;

/// field: ARROW of the convergence term.
inline constexpr std::int32_t kFieldConvergence = 0;
/// field: ARROW of the circulation term.
inline constexpr std::int32_t kFieldCirculation = 1;
/// field: ARROW of the feedforward (only when non-zero).
inline constexpr std::int32_t kFieldFeedforward = 2;
/// field: thick ARROW of F.
inline constexpr std::int32_t kFieldResult = 3;

/// command: ARROW from base_link along the commanded body velocity.
inline constexpr std::int32_t kCommandArrow = 0;
/// command: LINE_STRIP of 1 s of motion under the command (sim::integrate).
inline constexpr std::int32_t kCommandArc = 1;

/// heading: ARROW of the target heading (rotate phases).
inline constexpr std::int32_t kHeadingTarget = 0;
/// heading: LINE_STRIP arc from the current heading to the target.
inline constexpr std::int32_t kHeadingArc = 1;

/// prediction: LINE_STRIP of the rollout poses (per-vertex colours: green, red after the first
/// collision).
inline constexpr std::int32_t kPredictionPath = 0;
/// prediction: CUBE at the first colliding pose.
inline constexpr std::int32_t kPredictionCollision = 1;
/// prediction: TEXT `s_free 0.42 m`.
inline constexpr std::int32_t kPredictionText = 2;

/// goal: SPHERE at g.
inline constexpr std::int32_t kGoalPoint = 0;
/// goal: small SPHERE at g_ref (only when x_ref != 0).
inline constexpr std::int32_t kGoalReference = 1;
/// goal: CYLINDER ring of radius xy_tol about g.
inline constexpr std::int32_t kGoalToleranceRing = 2;
/// goal: CYLINDER ring of radius sigma_stop = xy_tol/2 about g.
inline constexpr std::int32_t kGoalStopRing = 3;
/// goal: ARROW of the goal yaw (only when the yaw is checked).
inline constexpr std::int32_t kGoalYaw = 4;
/// goal: LINE_STRIP overlay of the last sigma_b of the curve (the braking zone).
inline constexpr std::int32_t kGoalBrakingZone = 5;

/// contour: CUBE_LIST of K's cells.
inline constexpr std::int32_t kContourCells = 0;
/// contour: POINTS at the gate samples that tripped the gate.
inline constexpr std::int32_t kContourGateSamples = 1;
/// contour: SPHERE at x*_o.
inline constexpr std::int32_t kContourObstaclePoint = 2;
/// contour: LINE_LIST segment from the evaluation point to x*_o.
inline constexpr std::int32_t kContourObstacleLine = 3;
/// contour: CYLINDER ring of radius lambda about x*_o.
inline constexpr std::int32_t kContourLambdaRing = 4;
/// contour: CYLINDER ring of radius D_in about x*_o.
inline constexpr std::int32_t kContourFullSwitchRing = 5;
/// contour: CYLINDER ring of radius D'_in about x*_o.
inline constexpr std::int32_t kContourBlendStartRing = 6;
/// contour: ARROW of the circulation side sigma_c J n_o at the evaluation point.
inline constexpr std::int32_t kContourSide = 7;
/// contour: TEXT `contour +1 · 1.3/5.0 m · 4/20 s`.
inline constexpr std::int32_t kContourText = 8;

/// curve_info: LINE_LIST chevrons every 1 m along the curve (traversal direction).
inline constexpr std::int32_t kCurveInfoDirection = 0;
/// curve_info: SPHERE at the curve start.
inline constexpr std::int32_t kCurveInfoStart = 1;
/// curve_info: TEXT `lap 2/3 · rem 7.4 m`.
inline constexpr std::int32_t kCurveInfoText = 2;
/// curve_info: ARROW of the moving frame's velocity (FRAME_MOVING only).
inline constexpr std::int32_t kCurveInfoFrameVelocity = 3;
}  // namespace id

/// @brief The namespaces, indexed by Ns.
inline constexpr std::array<NamespaceSpec, kNamespaceCount> kNamespaces{{
  {"status", 1},
  {"footprint", 1},
  {"curve", 3},
  {"closest_point", 3},
  {"control_point", 2},
  {"field", 4},
  {"command", 2},
  {"heading", 2},
  {"prediction", 3},
  {"goal", 6},
  {"contour", 9},
  {"curve_info", 4},
}};

/// @brief The spec of a namespace.
constexpr const NamespaceSpec & spec(Ns ns) noexcept
{
  return kNamespaces[static_cast<std::size_t>(ns)];
}

/// @brief Total number of (namespace, id) pairs: the size of every `<ns>/markers` message.
constexpr std::size_t markerCount() noexcept
{
  std::size_t count = 0;
  for (const NamespaceSpec & ns : kNamespaces) {
    count += static_cast<std::size_t>(ns.id_count);
  }
  return count;
}

/// @brief kMarkerCount = markerCount().
inline constexpr std::size_t kMarkerCount = markerCount();

static_assert(kMarkerCount <= 80, "plan §9: at most 80 markers besides the field grid");
static_assert(
  static_cast<std::size_t>(Ns::kCurveInfo) + 1 == kNamespaceCount, "Ns and kNamespaces differ");

/// @name Status flags (the `status` text; docs/visualization.md "Status flags")
/// @{
/// kBlocked.
inline constexpr std::string_view kFlagBlocked = "BLOCKED";
/// An active contour episode; followed by the side, e.g. `CONTOUR +1`.
inline constexpr std::string_view kFlagContour = "CONTOUR";
/// kAssumption3Clamped.
inline constexpr std::string_view kFlagAssumption3 = "A3-CLAMP";
/// kCurvatureSaturated.
inline constexpr std::string_view kFlagCurvature = "CURV-SAT";
/// A cancel in progress.
inline constexpr std::string_view kFlagCancel = "CANCEL";
/// @}

/// @brief Height of the status text above base_link (m).
inline constexpr double kStatusTextHeight = 0.6;

}  // namespace legend
}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__LEGEND_HPP_
