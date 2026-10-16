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

#include "vectorfield_sim/ray_caster.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace vectorfield_sim
{

namespace
{

using vectorfield_core::Isometry2;
using vectorfield_core::Pose2;
using vectorfield_core::Vec2;

constexpr double kInf = std::numeric_limits<double>::infinity();
/// Direction components below this are treated as parallel to the axis.
constexpr double kParallel = 1e-12;

/// Cell index of coordinate `c` (cell units) for a ray moving with component `d`: a ray that
/// starts on a cell boundary and moves towards lower indices is in the lower cell.
int cellIndex(double c, double d, int n) noexcept
{
  const double index = d < 0.0 ? std::ceil(c) - 1.0 : std::floor(c);
  return static_cast<int>(std::clamp(index, 0.0, static_cast<double>(n - 1)));
}

/// Entry and exit parameters of the ray `q + t e` in the slab |x| <= h along one axis.
/// @return False when the ray is parallel to the slab and outside it.
bool slab(double q, double e, double h, double * t_near, double * t_far) noexcept
{
  if (std::abs(e) < kParallel) {
    return std::abs(q) <= h;
  }
  double a = (-h - q) / e;
  double b = (h - q) / e;
  if (a > b) {
    std::swap(a, b);
  }
  *t_near = std::max(*t_near, a);
  *t_far = std::min(*t_far, b);
  return true;
}

}  // namespace

bool GridMap::valid() const noexcept
{
  return width > 0 && height > 0 && resolution > 0.0 && std::isfinite(resolution) &&
         data.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

bool GridMap::occupied(int ix, int iy) const noexcept
{
  if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
    return false;
  }
  const auto index =
    static_cast<std::size_t>(iy) * static_cast<std::size_t>(width) + static_cast<std::size_t>(ix);
  return data[index] >= occupied_threshold;
}

ScanGeometry ScanGeometry::fullCircle(int beams, double range_max) noexcept
{
  ScanGeometry geometry;
  geometry.beams = std::max(beams, 1);
  geometry.angle_min = -vectorfield_core::kPi;
  geometry.angle_increment = vectorfield_core::kTwoPi / geometry.beams;
  geometry.range_max = range_max;
  return geometry;
}

void RayCaster::setMap(GridMap map) { map_ = map.valid() ? std::move(map) : GridMap{}; }

double RayCaster::cast(const Vec2 & origin, double angle, double max_range) const
{
  const Vec2 direction = vectorfield_core::heading(angle);
  double range = castGrid(origin, angle, max_range);
  for (const Circle & circle : circles_) {
    range = std::min(range, intersectCircle(origin, direction, circle));
  }
  for (const Box & box : boxes_) {
    range = std::min(range, intersectBox(origin, direction, box));
  }
  return range <= max_range ? range : kInf;
}

std::vector<double> RayCaster::castScan(
  const Pose2 & sensor_pose, const ScanGeometry & geometry) const
{
  std::vector<double> ranges(static_cast<std::size_t>(std::max(geometry.beams, 0)));
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    ranges[i] = cast(
      sensor_pose.p, sensor_pose.yaw + geometry.angle(static_cast<int>(i)), geometry.range_max);
  }
  return ranges;
}

double RayCaster::castGrid(const Vec2 & origin, double angle, double max_range) const
{
  if (!map_.valid() || !(max_range > 0.0)) {
    return kInf;
  }
  // Grid frame in cell units: the grid covers [0, width] x [0, height].
  const double res = map_.resolution;
  const Vec2 start = Isometry2::fromPose(map_.origin).applyInverse(origin) / res;
  const Vec2 d = vectorfield_core::heading(angle - map_.origin.yaw);
  const int size[2] = {map_.width, map_.height};

  // Clip the ray [0, max_range] to the grid's bounding box.
  double t_near = 0.0;
  double t_far = max_range / res;
  for (int axis = 0; axis < 2; ++axis) {
    const double half = 0.5 * size[axis];
    if (!slab(start[axis] - half, d[axis], half, &t_near, &t_far)) {
      return kInf;
    }
  }
  if (t_near > t_far) {
    return kInf;
  }

  // Amanatides–Woo traversal from the entry point.
  const Vec2 entry = start + t_near * d;
  int cell[2] = {0, 0};
  int step[2] = {0, 0};
  double t_next[2] = {kInf, kInf};
  double t_delta[2] = {kInf, kInf};
  for (int axis = 0; axis < 2; ++axis) {
    cell[axis] = cellIndex(entry[axis], d[axis], size[axis]);
    if (d[axis] > kParallel) {
      step[axis] = 1;
      t_delta[axis] = 1.0 / d[axis];
      t_next[axis] = t_near + (cell[axis] + 1 - entry[axis]) / d[axis];
    } else if (d[axis] < -kParallel) {
      step[axis] = -1;
      t_delta[axis] = -1.0 / d[axis];
      t_next[axis] = t_near + (entry[axis] - cell[axis]) / -d[axis];
    }
  }
  double t = t_near;
  while (t <= t_far) {
    if (map_.occupied(cell[0], cell[1])) {
      return t * res;
    }
    const int axis = t_next[0] < t_next[1] ? 0 : 1;
    t = t_next[axis];
    t_next[axis] += t_delta[axis];
    cell[axis] += step[axis];
    if (cell[axis] < 0 || cell[axis] >= size[axis]) {
      return kInf;
    }
  }
  return kInf;
}

double intersectCircle(const Vec2 & origin, const Vec2 & direction, const Circle & circle) noexcept
{
  const Vec2 f = origin - circle.center;
  const double b = f.dot(direction);
  const double c = f.squaredNorm() - circle.radius * circle.radius;
  if (c <= 0.0) {
    return 0.0;
  }
  const double discriminant = b * b - c;
  if (discriminant < 0.0 || b > 0.0) {
    return kInf;
  }
  return -b - std::sqrt(discriminant);
}

double intersectBox(const Vec2 & origin, const Vec2 & direction, const Box & box) noexcept
{
  const Isometry2 frame(box.yaw, box.center);
  const Vec2 q = frame.applyInverse(origin);
  const Vec2 e = frame.rotateInverse(direction);
  double t_near = -kInf;
  double t_far = kInf;
  if (
    !slab(q.x(), e.x(), 0.5 * box.size.x(), &t_near, &t_far) ||
    !slab(q.y(), e.y(), 0.5 * box.size.y(), &t_near, &t_far)) {
    return kInf;
  }
  if (t_near > t_far || t_far < 0.0) {
    return kInf;
  }
  return std::max(t_near, 0.0);
}

}  // namespace vectorfield_sim
