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
/// @brief MarkerBuilder: pure conversion of one Tracker cycle into the visualization messages
/// (plan §4.1; math_spec Appendix A).
///
/// Implemented in src/marker_builder.cpp (M2 wave 2, ros-viz). No node, no I/O, no clock: every
/// output is a function of the MarkerContext (and the VizConfig) only, so it is unit-testable.
/// The builder owns reusable message buffers that are preallocated at construction and refilled
/// in place, so a steady-state cycle does not allocate once the variable-size fields (rollout
/// points, K cells, grid arrows) have reached their peak size.

#ifndef VECTORFIELD_ROS__MARKER_BUILDER_HPP_
#define VECTORFIELD_ROS__MARKER_BUILDER_HPP_

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/tracker.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_ros/legend.hpp"
#include "vectorfield_ros/viz_config.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vectorfield_ros
{

/// @brief Follower-only data of the `curve_info` namespace (M5). The plugin passes none, and
/// `curve_info` is then DELETEd.
struct CurveInfo
{
  /// Traversal direction ±1 (arrows point along it).
  int direction{1};
  /// Laps completed.
  std::uint32_t laps_completed{0};
  /// Laps requested; 0 = until cancelled (the text then shows only the completed laps).
  std::uint32_t laps{0};
  /// Remaining arc length (m); +Inf when laps = 0.
  double remaining_arc_length{vectorfield_core::kInf};
  /// The curve frame moves (FRAME_MOVING).
  bool frame_moving{false};
  /// Origin of the curve frame (control frame), where the frame-velocity arrow starts.
  vectorfield_core::Vec2 frame_origin{vectorfield_core::Vec2::Zero()};
  /// Velocity of the curve frame's origin (control frame, m/s).
  vectorfield_core::Vec2 frame_velocity{vectorfield_core::Vec2::Zero()};
};

/// @brief Everything the builder reads for one cycle. Non-owning; every pointer and span must
/// stay valid for the call. All geometry is in the control frame (plugin: the costmap global
/// frame; follower: `global_frame`), which is the Tracker's evaluation frame.
struct MarkerContext
{
  /// Frame of every message (the control frame).
  std::string_view frame_id;
  /// Stamp of every message (the cycle stamp).
  builtin_interfaces::msg::Time stamp;
  /// The Tracker that ran the cycle: resolved configuration (lambda, D_in, D'_in, sigma_b,
  /// watchdog limits), footprint, kinematics (the 1 s command arc) and probe() (field grid).
  /// Required.
  const vectorfield_core::Tracker * tracker{nullptr};
  /// The cycle's input: base pose, curve, goal (xy_tol, yaw), obstacles (`T_grid_eval` to draw
  /// K's cells), cancel flag. Required.
  const vectorfield_core::TrackerInput * input{nullptr};
  /// The cycle's output (field sample, phase, status, rollout view). Required.
  const vectorfield_core::TrackerOutput * output{nullptr};
  /// The state step() wrote (probe hint for the field grid). Required.
  const vectorfield_core::TrackerState * state{nullptr};
  /// The scratch step() used: K's mask and grid geometry (`k_mask`, `k_width`, `k_height`,
  /// `k_valid`, `edt_k`), the gate samples and their flags. Required.
  const vectorfield_core::StepScratch * scratch{nullptr};
  /// Plugin: the conditioned window vertices exactly as passed to `PolylineCurve::assign`.
  /// Empty: the builder samples `input->curve` instead (follower).
  vectorfield_core::Span<const vectorfield_core::Vec2> window;
  /// Index of the virtual tail vertex in `window`, or −1 (no tail conditioning this cycle).
  int virtual_vertex{-1};
  /// Follower: the curve_info data; null for the plugin.
  const CurveInfo * curve_info{nullptr};
  /// Wall time of the cycle's compute (ms), for FieldState.
  double compute_time_ms{0.0};
};

/// @brief Builds the visualization messages of one cycle (plan §4.1).
///
/// **Contract of markers()** (tested per phase × status by the layer-2 MarkerBuilder gtest):
/// - exactly one marker per (namespace, id) of the legend (legend::kNamespaces), in legend order:
///   `ADD` when it applies this cycle, `DELETE` otherwise, so the message size is always
///   legend::kMarkerCount and ids are stable across calls;
/// - namespaces that do not apply (e.g. `heading` outside the rotate phases, `contour` while the
///   gate is closed and no episode is active, `curve_info` without CurveInfo, `goal` without a
///   goal) are DELETEd as a whole; sub-markers that do not apply (the feedforward arrow when
///   \f$\Phi_T = 0\f$, `g_ref` when \f$x_{\mathrm{ref}} = 0\f$, `g_ref` and the braking zone
///   while the curve does not end at the goal (no terminal mode), the collision cube without a
///   collision, the virtual vertex without tail conditioning) are DELETEd individually;
/// - every marker has `header.frame_id = frame_id`, `header.stamp = stamp`, finite coordinates
///   (non-finite inputs DELETE the marker instead), `frame_locked = false` and `lifetime = 0`;
/// - colours come from the legend only.
///
/// **Not thread-safe:** one builder per publishing thread. The returned references stay valid
/// until the next call of the same method.
class MarkerBuilder
{
public:
  /// @brief Preallocates the marker buffers (legend::kMarkerCount markers).
  MarkerBuilder();

  /// @brief `<ns>/markers` of one cycle (see the class contract).
  const visualization_msgs::msg::MarkerArray & markers(const MarkerContext & context);

  /// @brief `<ns>/field_grid` of one cycle.
  ///
  /// A `DELETEALL` marker (namespace `field_grid`) followed by, for `arrows`, one ARROW per node
  /// of the `field_grid_size` square centred on base_link at `field_grid_resolution` (length
  /// proportional to \f$\lVert F\rVert\f$, colour viridis of \f$D\f$, ids 0..N−1), or, for
  /// `streamlines`, one LINE_STRIP per node seed integrated with vectorfield_core::streamline at
  /// step `field_grid_resolution`/2. Every node is probed with `Tracker::probe()` on the cycle's
  /// input, state and scratch, so the grid shows the exact field of the cycle, contour included
  /// (math_spec Appendix A). Nodes whose probe is not finite are skipped.
  const visualization_msgs::msg::MarkerArray & fieldGrid(
    const MarkerContext & context, const VizConfig & config);

  /// @brief `<ns>/window`: the curve the field uses (the window vertices, or the sampled curve
  /// for the follower), each pose oriented along the local tangent.
  const nav_msgs::msg::Path & window(const MarkerContext & context);

  /// @brief `<ns>/predicted_path`: the checked rollout poses (`output->predicted`); empty when
  /// collision prediction did not run.
  const nav_msgs::msg::Path & predictedPath(const MarkerContext & context);

  /// @brief `<ns>/obstacle_distance`: \f$\mathrm{EDT}_{\mathrm{all}}\f$
  /// (`input->obstacles->edt`) as an OccupancyGrid in the grid geometry, scaled linearly to
  /// 0–100 up to \f$D'_{\mathrm{in}}\f$ + 0.5 m (100 = at an obstacle, 0 = that far or farther).
  /// Empty (0 × 0) when the cycle had no obstacles or the EDT has no sites.
  const nav_msgs::msg::OccupancyGrid & obstacleDistance(const MarkerContext & context);

  /// @brief A single `DELETEALL` marker (goal end, reset, deactivation).
  const visualization_msgs::msg::MarkerArray & deleteAll(
    const builtin_interfaces::msg::Time & stamp, std::string_view frame_id);

private:
  visualization_msgs::msg::MarkerArray markers_;
  visualization_msgs::msg::MarkerArray grid_;
  visualization_msgs::msg::MarkerArray delete_all_;
  nav_msgs::msg::Path window_;
  nav_msgs::msg::Path predicted_;
  nav_msgs::msg::OccupancyGrid distance_;
  std::vector<vectorfield_core::Vec2> points_;
};

/// @brief Viridis colour map at \f$u \in [0, 1]\f$ (clamped; NaN maps to 1): the field grid's
/// colour by distance, shared with the 3D follower's field arrows.
std_msgs::msg::ColorRGBA viridis(double u) noexcept;

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__MARKER_BUILDER_HPP_
