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

// Robot footprint and the footprint collision checker (math_spec §10.1).

#include "vectorfield_core/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

#include "vectorfield_core/errors.hpp"

namespace vectorfield_core
{

namespace
{

constexpr double kSqrt2 = 1.4142135623730951;

/// Slack (m) on the per-cell circumscribed-disc rejection of the ground truth, so rounding can
/// never reject a cell whose square touches the footprint.
constexpr double kRejectSlack = 1e-9;

/// \f$R(\psi)v\f$ from a cached cosine and sine.
inline Vec2 rotated(double c, double s, const Vec2 & v) noexcept
{
  return Vec2(c * v.x() - s * v.y(), s * v.x() + c * v.y());
}

/// True iff the closed segment [a, b] meets the closed square \f$[-h, h]^2\f$.
///
/// Separating-axis test: two convex sets in the plane are disjoint iff an edge normal of one of
/// them separates them; the candidates are the square's two axes and the segment's normal.
/// Touching counts as meeting (closed sets).
bool segmentTouchesSquare(const Vec2 & a, const Vec2 & b, double h) noexcept
{
  if (std::max(a.x(), b.x()) < -h || std::min(a.x(), b.x()) > h) {
    return false;
  }
  if (std::max(a.y(), b.y()) < -h || std::min(a.y(), b.y()) > h) {
    return false;
  }
  // Projection onto the segment normal (−e_y, e_x): one value for the whole segment, against the
  // square's support h(|e_x| + |e_y|).
  const Vec2 e = b - a;
  const double offset = a.y() * e.x() - a.x() * e.y();
  return std::abs(offset) <= h * (std::abs(e.x()) + std::abs(e.y()));
}

/// True iff the closed disc of radius r centred at q meets the closed square \f$[-h, h]^2\f$.
bool discTouchesSquare(const Vec2 & q, double r, double h) noexcept
{
  const double dx = std::max(std::abs(q.x()) - h, 0.0);
  const double dy = std::max(std::abs(q.y()) - h, 0.0);
  return dx * dx + dy * dy <= r * r;
}

/// True iff the closed square \f$[-h, h]^2\f$ meets the filled polygon with vertices
/// \f$d + R(\psi)v_i\f$ (closed, even-odd fill).
///
/// Either some edge meets the square, or none does; then the square lies wholly inside or wholly
/// outside the polygon, and the crossing parity of its centre (the origin, at distance at least
/// h from every edge) decides. The vertices are mapped on the fly, so no buffer is needed.
bool polygonTouchesSquare(
  Span<const Vec2> vertices, const Vec2 & d, double c, double s, double h) noexcept
{
  const std::size_t n = vertices.size();
  bool inside = false;
  Vec2 a = d + rotated(c, s, vertices[n - 1]);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 b = d + rotated(c, s, vertices[i]);
    if (segmentTouchesSquare(a, b, h)) {
      return true;
    }
    if ((a.y() > 0.0) != (b.y() > 0.0)) {
      const double x_cross = a.x() + (0.0 - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
      if (x_cross > 0.0) {
        inside = !inside;
      }
    }
    a = b;
  }
  return inside;
}

/// The shortcuts need \f$\mathrm{EDT}_{\mathrm{all}}\f$ of exactly this grid content: an
/// unfiltered build whose stamp equals the grid's (a filtered or stale transform could miss sites,
/// and the free shortcut would then be unsound).
bool shortcutsApply(const OccupancyGrid & grid, const EuclideanDistanceTransform & edt) noexcept
{
  return grid.version() != 0 && edt.builtVersion() == grid.version() &&
         edt.width() == grid.width() && edt.height() == grid.height();
}

[[noreturn]] void throwInvalid(const std::string & reason)
{
  throw ConfigError("invalid footprint: " + reason);
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Footprint

Footprint::Footprint() noexcept {}

Footprint Footprint::circle(double radius)
{
  FootprintSpec spec;
  spec.radius = radius;
  std::string reason;
  if (!footprintIsValid(spec, &reason)) {
    throwInvalid(reason);
  }
  Footprint footprint;
  footprint.radius_ = radius;
  footprint.radii_ = computeFootprintRadii(spec);
  return footprint;
}

Footprint Footprint::polygon(Span<const Vec2> vertices)
{
  if (vertices.size() < 3) {
    throwInvalid("polygon has " + std::to_string(vertices.size()) + " vertices; at least 3 needed");
  }
  FootprintSpec spec;
  spec.polygon.assign(vertices.begin(), vertices.end());
  std::string reason;
  if (!footprintIsValid(spec, &reason)) {
    throwInvalid(reason);
  }
  Footprint footprint;
  footprint.radii_ = computeFootprintRadii(spec);
  footprint.vertices_ = std::move(spec.polygon);
  return footprint;
}

Footprint Footprint::fromSpec(const FootprintSpec & spec)
{
  if (!spec.polygon.empty()) {
    return polygon(spec.polygon);
  }
  return circle(spec.radius);
}

bool Footprint::isCircle() const noexcept { return vertices_.empty(); }

double Footprint::radius() const noexcept { return radius_; }

Span<const Vec2> Footprint::vertices() const noexcept
{
  return Span<const Vec2>(vertices_.data(), vertices_.size());
}

const FootprintRadii & Footprint::radii() const noexcept { return radii_; }

double Footprint::coverRadius(const Vec2 & point) const noexcept
{
  if (vertices_.empty()) {
    return radius_ + point.norm();
  }
  double radius = 0.0;
  for (const Vec2 & vertex : vertices_) {
    radius = std::max(radius, (vertex - point).norm());
  }
  return radius;
}

// -----------------------------------------------------------------------------------------------
// FootprintChecker

FootprintChecker::FootprintChecker() noexcept {}

FootprintChecker::FootprintChecker(Footprint footprint) noexcept : footprint_(std::move(footprint))
{
}

const Footprint & FootprintChecker::footprint() const noexcept { return footprint_; }

bool FootprintChecker::collides(
  const Pose2 & pose_grid, const OccupancyGrid & grid,
  const EuclideanDistanceTransform & edt_all) const noexcept
{
  switch (classify(pose_grid, grid, edt_all)) {
    case FootprintDecision::kFreeShortcut:
    case FootprintDecision::kFreeRasterized:
      return false;
    case FootprintDecision::kCollisionShortcut:
    case FootprintDecision::kCollisionRasterized:
      return true;
    case FootprintDecision::kOffGrid:
      break;
  }
  // The base cell is outside the grid (or the pose is non-finite): the in-grid cells decide.
  return collidesGroundTruth(pose_grid, grid);
}

FootprintDecision FootprintChecker::classify(
  const Pose2 & pose_grid, const OccupancyGrid & grid,
  const EuclideanDistanceTransform & edt_all) const noexcept
{
  int cx = 0;
  int cy = 0;
  if (!isFinite(pose_grid) || !grid.cellAt(pose_grid.p, &cx, &cy)) {
    return FootprintDecision::kOffGrid;
  }
  if (shortcutsApply(grid, edt_all)) {
    if (!edt_all.hasSites()) {
      return FootprintDecision::kFreeShortcut;
    }
    const FootprintRadii & radii = footprint_.radii();
    // D_lo bounds the distance from p to every site centre from below (triangle inequality).
    const double d_lo = edt_all.distance(cx, cy) - (pose_grid.p - grid.cellCenter(cx, cy)).norm();
    if (d_lo > radii.r_circ + grid.resolution()) {
      return FootprintDecision::kFreeShortcut;
    }
    // D_hi is the distance to an actual site; inside the inscribed disc it lies in the polygon.
    int fx = 0;
    int fy = 0;
    edt_all.feature(cx, cy, &fx, &fy);
    const double d_hi = (pose_grid.p - grid.cellCenter(fx, fy)).norm();
    if (d_hi < radii.r_insc) {
      return FootprintDecision::kCollisionShortcut;
    }
  }
  return collidesGroundTruth(pose_grid, grid) ? FootprintDecision::kCollisionRasterized
                                              : FootprintDecision::kFreeRasterized;
}

bool FootprintChecker::collidesGroundTruth(
  const Pose2 & pose_grid, const OccupancyGrid & grid) const noexcept
{
  if (!isFinite(pose_grid)) {
    return true;  // never call a non-finite pose free
  }
  const int width = grid.width();
  const int height = grid.height();
  const Span<const std::uint8_t> mask = grid.mask();
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (cells == 0 || mask.size() != cells) {
    return false;
  }
  const double res = grid.resolution();
  const Vec2 & origin = grid.origin();
  const Vec2 & p = pose_grid.p;
  const double c = std::cos(pose_grid.yaw);
  const double s = std::sin(pose_grid.yaw);
  const Span<const Vec2> vertices = footprint_.vertices();
  const bool disc = vertices.empty();
  const FootprintRadii & radii = footprint_.radii();

  // Grid-frame bounding box of the footprint, then every cell whose closed square can touch it
  // (one cell of slack per side covers squares that share the box's boundary, and rounding).
  Vec2 lo;
  Vec2 hi;
  if (disc) {
    const Vec2 r(footprint_.radius(), footprint_.radius());
    lo = p - r;
    hi = p + r;
  } else {
    lo = Vec2(kInf, kInf);
    hi = Vec2(-kInf, -kInf);
    for (const Vec2 & v : vertices) {
      const Vec2 g = p + rotated(c, s, v);
      lo = lo.cwiseMin(g);
      hi = hi.cwiseMax(g);
    }
  }
  const double x_lo = std::floor((lo.x() - origin.x()) / res) - 1.0;
  const double x_hi = std::floor((hi.x() - origin.x()) / res) + 1.0;
  const double y_lo = std::floor((lo.y() - origin.y()) / res) - 1.0;
  const double y_hi = std::floor((hi.y() - origin.y()) / res) + 1.0;
  if (x_hi < 0.0 || y_hi < 0.0 || x_lo > width - 1.0 || y_lo > height - 1.0) {
    return false;
  }
  const int ix_lo = static_cast<int>(std::max(x_lo, 0.0));
  const int ix_hi = static_cast<int>(std::min(x_hi, width - 1.0));
  const int iy_lo = static_cast<int>(std::max(y_lo, 0.0));
  const int iy_hi = static_cast<int>(std::min(y_hi, height - 1.0));

  const double h = 0.5 * res;
  // The footprint lies in the closed disc (p, r_circ), a cell square in the disc (centre, h√2).
  const double reject = radii.r_circ + h * kSqrt2 + kRejectSlack;
  for (int iy = iy_lo; iy <= iy_hi; ++iy) {
    const std::uint8_t * row =
      mask.data() + static_cast<std::size_t>(iy) * static_cast<std::size_t>(width);
    for (int ix = ix_lo; ix <= ix_hi; ++ix) {
      if (row[ix] == 0) {
        continue;
      }
      const Vec2 d = p - grid.cellCenter(ix, iy);  // base_link relative to the cell centre
      const double dist = d.norm();
      if (dist > reject) {
        continue;
      }
      if (dist < radii.r_insc) {
        return true;  // the cell centre lies in the inscribed disc, hence in the footprint
      }
      if (
        disc ? discTouchesSquare(d, footprint_.radius(), h)
             : polygonTouchesSquare(vertices, d, c, s, h)) {
        return true;
      }
    }
  }
  return false;
}

bool FootprintChecker::offGrid(const Pose2 & pose_grid, const OccupancyGrid & grid) const noexcept
{
  if (!isFinite(pose_grid) || grid.width() <= 0 || grid.height() <= 0) {
    return true;
  }
  const double reach = footprint_.radii().r_circ + grid.resolution();
  const Vec2 & lo = grid.origin();
  const double x_hi = lo.x() + grid.width() * grid.resolution();
  const double y_hi = lo.y() + grid.height() * grid.resolution();
  const Vec2 & p = pose_grid.p;
  return p.x() - reach < lo.x() || p.x() + reach > x_hi || p.y() - reach < lo.y() ||
         p.y() + reach > y_hi;
}

}  // namespace vectorfield_core
