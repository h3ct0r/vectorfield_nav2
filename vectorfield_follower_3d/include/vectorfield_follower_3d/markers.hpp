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
/// @brief The 3D follower's RViz markers (`~/markers`). Pure: a MarkerArray from one cycle.
///
/// Namespaces: `curve` (the curve in `global_frame`), `closest_point`, `tangent`, `command`,
/// `field` (arrows of \f$\Phi\f$ on a lattice around the robot), `goal` (open curves),
/// `obstacles` (primitives), `voxels` (occupied voxels), `body` (the CBF sphere) and `status`
/// (a text above the robot). The array starts with a DELETEALL, so a namespace that is not
/// drawn this time disappears.

#ifndef VECTORFIELD_FOLLOWER_3D__MARKERS_HPP_
#define VECTORFIELD_FOLLOWER_3D__MARKERS_HPP_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/d3/field.hpp"
#include "vectorfield_core/d3/frame.hpp"
#include "vectorfield_core/d3/obstacles.hpp"
#include "vectorfield_core/d3/tracker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vectorfield_follower_3d
{

/// @brief Upper bound of the curve and voxel markers' points.
inline constexpr int kMaxMarkerPoints = 20000;

/// @brief One cycle as drawn (pointers are not owned and may be null).
struct MarkerInput3
{
  /// Frame of every marker (`global_frame`).
  std::string frame_id;
  /// Stamp of every marker.
  builtin_interfaces::msg::Time stamp;
  /// The curve, in its frame.
  const vectorfield_core::d3::Curve3 * curve{nullptr};
  /// The curve frame in `frame_id` at `t`.
  vectorfield_core::d3::CurveFrame3 frame;
  /// Time t (s).
  double t{0.0};
  /// Spacing of the curve samples (m).
  double sample_spacing{0.05};
  /// The field of the lattice arrows (null: none).
  const vectorfield_core::d3::DistanceField3 * field{nullptr};
  /// Half extent of the lattice (m); 0: none.
  double field_extent{0.0};
  /// Lattice spacing (m).
  double field_step{0.5};
  /// The body pose.
  vectorfield_core::d3::Pose3 pose;
  /// The tracker's output of this cycle (null: none).
  const vectorfield_core::d3::Tracker3Output * output{nullptr};
  /// The obstacles (null: none).
  const vectorfield_core::d3::ObstacleSet3 * obstacles{nullptr};
  /// Body radius drawn when the CBF filter is on (m); <= 0: no body.
  double body_radius{0.0};
  /// Status text.
  std::string status_text;
};

/// @brief The markers of one cycle (see the file comment).
visualization_msgs::msg::MarkerArray buildMarkers3(const MarkerInput3 & input);

/// @brief A single DELETEALL marker (clears the namespace set at the end of a goal).
visualization_msgs::msg::MarkerArray clearMarkers3(
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp);

}  // namespace vectorfield_follower_3d

#endif  // VECTORFIELD_FOLLOWER_3D__MARKERS_HPP_
