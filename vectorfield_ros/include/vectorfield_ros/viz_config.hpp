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
/// @brief Visualization parameters (the Visualization parameter group).
///
/// These keys live in `vectorfield_common_params` but have no vectorfield_core field: they are
/// the ROS-only allow-list of the parameter-coverage test (core_params.hpp,
/// kCommonRosOnlyParams). toVizConfig() (core_params.hpp) fills a VizConfig from the listener.

#ifndef VECTORFIELD_ROS__VIZ_CONFIG_HPP_
#define VECTORFIELD_ROS__VIZ_CONFIG_HPP_

#include <cstdint>
#include <string_view>

namespace vectorfield_ros
{

/// @brief `field_grid_style` (the first listed value is the default).
enum class FieldGridStyle : std::uint8_t
{
  /// "arrows": one ARROW per grid node, length proportional to \f$\lVert F\rVert\f$, coloured by
  /// \f$D\f$ (viridis).
  kArrows = 0,
  /// "streamlines": LINE_STRIPs integrated with vectorfield_core::streamline (RK4, step
  /// `field_grid_resolution`/2), seeded at the grid nodes.
  kStreamlines = 1,
};

/// @brief The parameter string of a style ("arrows", "streamlines").
constexpr const char * toString(FieldGridStyle style) noexcept
{
  return style == FieldGridStyle::kStreamlines ? "streamlines" : "arrows";
}

/// @brief Parses a style; returns false and leaves `*out` unchanged for an unknown string.
constexpr bool fromString(std::string_view text, FieldGridStyle * out) noexcept
{
  if (text == "arrows") {
    *out = FieldGridStyle::kArrows;
    return true;
  }
  if (text == "streamlines") {
    *out = FieldGridStyle::kStreamlines;
    return true;
  }
  return false;
}

/// @brief Visualization settings. Defaults equal the parameter defaults.
struct VizConfig
{
  /// `visualize`: master switch. Even when true, each topic is built and published only while it
  /// has subscribers.
  bool visualize{true};
  /// `field_grid_size` (m): side of the square grid centred on base_link.
  double field_grid_size{3.0};
  /// `field_grid_resolution` (m): node spacing of the grid.
  double field_grid_resolution{0.25};
  /// `field_grid_rate` (Hz): `<ns>/field_grid` is published at most this often (decimated in the
  /// control thread by the cycle stamps; no timer).
  double field_grid_rate{2.0};
  /// `field_grid_style`.
  FieldGridStyle field_grid_style{FieldGridStyle::kArrows};
  /// `publish_obstacle_distance`: publish `<ns>/obstacle_distance` (still only with subscribers).
  bool publish_obstacle_distance{false};
};

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__VIZ_CONFIG_HPP_
