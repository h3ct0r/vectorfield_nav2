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
/// @brief Robot footprint and the footprint collision checker (math_spec §10.1,
/// sec-footprint).
///
/// Implemented in src/footprint.cpp.

#ifndef VECTORFIELD_CORE__FOOTPRINT_HPP_
#define VECTORFIELD_CORE__FOOTPRINT_HPP_

#include <cstdint>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Robot footprint in the robot frame: a filled polygon or a disc, with its radii.
///
/// The radii are computeFootprintRadii() of config.hpp (math_spec §10.1): \f$w_{\mathrm{lat}}\f$,
/// \f$r_{\mathrm{insc}}\f$ (0 if base_link is not strictly inside the polygon) and
/// \f$r_{\mathrm{circ}}\f$. A default-constructed footprint is a point robot (radius 0).
class Footprint
{
public:
  /// @brief A point robot.
  Footprint() noexcept;

  /// @brief A disc of the given radius (Nav2 `robot_radius`).
  /// @throws ConfigError if the radius is negative or not finite.
  static Footprint circle(double radius);

  /// @brief A filled polygon (robot frame, either orientation, convex or not).
  /// @throws ConfigError if it has fewer than 3 vertices, a non-finite vertex or zero area.
  static Footprint polygon(Span<const Vec2> vertices);

  /// @brief From a FootprintSpec (polygon if non-empty, else circle).
  /// @throws ConfigError as circle() or polygon().
  static Footprint fromSpec(const FootprintSpec & spec);

  /// @brief True for a disc (including the point robot).
  bool isCircle() const noexcept;
  /// @brief Disc radius (0 for a polygon).
  double radius() const noexcept;
  /// @brief Polygon vertices in the robot frame (empty for a disc).
  Span<const Vec2> vertices() const noexcept;
  /// @brief \f$w_{\mathrm{lat}}, r_{\mathrm{insc}}, r_{\mathrm{circ}}\f$.
  const FootprintRadii & radii() const noexcept;
  /// @brief \f$w_{\mathrm{lat}}\f$.
  double wLat() const noexcept { return radii().w_lat; }
  /// @brief \f$r_{\mathrm{insc}}\f$.
  double rInsc() const noexcept { return radii().r_insc; }
  /// @brief \f$r_{\mathrm{circ}}\f$.
  double rCirc() const noexcept { return radii().r_circ; }
  /// @brief Radius \f$r_e\f$ of the smallest disc centred on a robot-frame point that contains
  /// the footprint (math_spec §6.7): the largest distance from the point to a vertex, or
  /// radius + \f$\lVert\f$point\f$\rVert\f$ for a disc. Equal to \f$r_{\mathrm{circ}}\f$ at
  /// base_link.
  double coverRadius(const Vec2 & point) const noexcept;

private:
  std::vector<Vec2> vertices_;
  double radius_{0.0};
  FootprintRadii radii_;
};

/// @brief Which rule decided a footprint check (for tests of oracle O53).
enum class FootprintDecision : std::uint8_t
{
  /// Free by the shortcut \f$D_{\mathrm{lo}} > r_{\mathrm{circ}} + \mathrm{res}\f$.
  kFreeShortcut = 0,
  /// Colliding by the shortcut \f$D_{\mathrm{hi}} < r_{\mathrm{insc}}\f$.
  kCollisionShortcut = 1,
  /// Free by rasterisation (ground truth).
  kFreeRasterized = 2,
  /// Colliding by rasterisation (ground truth).
  kCollisionRasterized = 3,
  /// The pose's cell is outside the grid; nothing was decided (callers treat it as off-grid).
  kOffGrid = 4,
};

/// @brief Footprint collision test against an occupancy grid (math_spec §10.1 [Design]).
///
/// **Ground truth** [Spec]: a pose collides iff some site cell's closed square intersects the
/// filled footprint. **Shortcuts**, with \f$c_p\f$ the cell containing the base position \f$p\f$:
/// \f$D_{\mathrm{lo}} = \mathrm{EDT}_{\mathrm{all}}(c_p) - \lVert p - c_p\rVert\f$ and
/// \f$D_{\mathrm{hi}} = \lVert p - f(c_p)\rVert\f$; free if \f$D_{\mathrm{lo}} > r_{\mathrm{circ}} +
/// \mathrm{res}\f$, colliding if \f$D_{\mathrm{hi}} < r_{\mathrm{insc}}\f$, otherwise rasterise
/// the filled footprint and test the ground truth. Both shortcuts are sound (oracle O53).
///
/// Poses are in the **grid frame** (callers map with `ObstacleMap::T_grid_eval`). Every check is
/// `noexcept` and allocation-free (no transformed-polygon buffer: cells are mapped into the
/// robot frame instead). Const methods are thread-safe.
class FootprintChecker
{
public:
  /// @brief A checker for a point robot.
  FootprintChecker() noexcept;
  /// @brief A checker for the given footprint.
  explicit FootprintChecker(Footprint footprint) noexcept;

  /// @brief The footprint.
  const Footprint & footprint() const noexcept;

  /// @brief True iff the pose collides (shortcuts, then ground truth). An off-grid pose (see
  /// offGrid()) is tested against the in-grid cells only.
  bool collides(
    const Pose2 & pose_grid, const OccupancyGrid & grid,
    const EuclideanDistanceTransform & edt_all) const noexcept;

  /// @brief Like collides(), reporting which rule decided.
  FootprintDecision classify(
    const Pose2 & pose_grid, const OccupancyGrid & grid,
    const EuclideanDistanceTransform & edt_all) const noexcept;

  /// @brief Ground truth only: rasterises the filled footprint against every site cell it may
  /// touch.
  bool collidesGroundTruth(const Pose2 & pose_grid, const OccupancyGrid & grid) const noexcept;

  /// @brief True iff the disc of radius \f$r_{\mathrm{circ}} + \mathrm{res}\f$ around the pose
  /// leaves the grid (math_spec §10.1); the rollout ends there (math_spec §10.2).
  bool offGrid(const Pose2 & pose_grid, const OccupancyGrid & grid) const noexcept;

private:
  Footprint footprint_;
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__FOOTPRINT_HPP_
