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

// Analytic obstacles, the obstacle set and the 3D CBF filter (math_spec §14.8 [Design, 0.3.0]).
// The voxel grid and its distance transform are in voxel_edt.cpp.

#include "vectorfield_core/d3/obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "vectorfield_core/cbf.hpp"
#include "vectorfield_core/errors.hpp"

namespace vectorfield_core::d3
{

// -----------------------------------------------------------------------------------------------
// Primitives

Primitive3 Primitive3::sphere(const Vec3 & center, double radius) noexcept
{
  Primitive3 p;
  p.type = PrimitiveType3::kSphere;
  p.center = center;
  p.radius = radius;
  return p;
}

Primitive3 Primitive3::box(
  const Vec3 & center, const Vec3 & half_extents, const Mat3 & rotation) noexcept
{
  Primitive3 p;
  p.type = PrimitiveType3::kBox;
  p.center = center;
  p.half_extents = half_extents;
  p.rotation = rotation;
  return p;
}

Primitive3 Primitive3::cylinder(const Vec3 & center, double radius, double half_height) noexcept
{
  Primitive3 p;
  p.type = PrimitiveType3::kCylinder;
  p.center = center;
  p.radius = radius;
  p.half_height = half_height;
  return p;
}

Primitive3 Primitive3::plane(const Vec3 & point, const Vec3 & normal) noexcept
{
  Primitive3 p;
  p.type = PrimitiveType3::kPlane;
  p.center = point;
  p.normal = unitOr(normal, normal);
  return p;
}

namespace
{

const char * typeName(PrimitiveType3 type) noexcept
{
  switch (type) {
    case PrimitiveType3::kSphere:
      return "sphere";
    case PrimitiveType3::kBox:
      return "box";
    case PrimitiveType3::kCylinder:
      return "cylinder";
    case PrimitiveType3::kPlane:
      return "plane";
  }
  return "unknown";
}

void requirePrimitive(bool condition, const Primitive3 & p, const std::string & what)
{
  if (!condition) {
    throw ConfigError(std::string("Primitive3 (") + typeName(p.type) + "): " + what);
  }
}

bool positive(double value) noexcept { return std::isfinite(value) && value > 0.0; }

}  // namespace

void validatePrimitive3(const Primitive3 & p)
{
  requirePrimitive(isFinite(p.center), p, "center must be finite");
  switch (p.type) {
    case PrimitiveType3::kSphere:
      requirePrimitive(
        positive(p.radius), p, "radius = " + std::to_string(p.radius) + " must be finite and > 0");
      return;
    case PrimitiveType3::kBox: {
      for (int i = 0; i < 3; ++i) {
        requirePrimitive(positive(p.half_extents[i]), p, "half_extents must be finite and > 0");
      }
      requirePrimitive(p.rotation.allFinite(), p, "rotation must be finite");
      const double orthogonality = (p.rotation.transpose() * p.rotation - Mat3::Identity()).norm();
      requirePrimitive(
        orthogonality <= 1e-9 && p.rotation.determinant() > 0.0, p,
        "rotation must be a rotation matrix (orthonormal to 1e-9, det +1)");
      return;
    }
    case PrimitiveType3::kCylinder:
      requirePrimitive(
        positive(p.radius), p, "radius = " + std::to_string(p.radius) + " must be finite and > 0");
      requirePrimitive(
        p.half_height > 0.0 && !std::isnan(p.half_height), p,
        "half_height = " + std::to_string(p.half_height) + " must be > 0 (inf allowed)");
      return;
    case PrimitiveType3::kPlane: {
      const double norm = p.normal.norm();
      requirePrimitive(isFinite(p.normal) && norm > 0.0, p, "normal must be finite and nonzero");
      return;
    }
  }
  requirePrimitive(false, p, "unknown type");
}

ObstacleDistance3 signedDistance(const Primitive3 & p, const Vec3 & x) noexcept
{
  ObstacleDistance3 out;
  if (!isFinite(x)) {
    return out;
  }
  switch (p.type) {
    case PrimitiveType3::kSphere: {
      const Vec3 v = x - p.center;
      const double r = v.norm();
      out.n = r > 0.0 ? Vec3(v / r) : Vec3(Vec3::UnitZ());
      out.D = r - p.radius;
      break;
    }
    case PrimitiveType3::kBox: {
      const Vec3 q = p.rotation.transpose() * (x - p.center);
      const Vec3 d = q.cwiseAbs() - p.half_extents;
      Vec3 n_local = Vec3::Zero();
      if (d.maxCoeff() > 0.0) {
        // Outside: the distance to the nearest face, edge or corner.
        const Vec3 w = d.cwiseMax(0.0);
        out.D = w.norm();
        for (int i = 0; i < 3; ++i) {
          n_local[i] = (q[i] < 0.0 ? -w[i] : w[i]) / out.D;
        }
      } else {
        // Inside: the nearest face (the first on ties).
        int axis = 0;
        for (int i = 1; i < 3; ++i) {
          if (d[i] > d[axis]) {
            axis = i;
          }
        }
        out.D = d[axis];
        n_local[axis] = q[axis] < 0.0 ? -1.0 : 1.0;
      }
      out.n = p.rotation * n_local;
      break;
    }
    case PrimitiveType3::kCylinder: {
      const Vec3 q = x - p.center;
      const double rho = std::hypot(q.x(), q.y());
      const Vec3 radial = rho > 0.0 ? Vec3(q.x() / rho, q.y() / rho, 0.0) : Vec3(Vec3::UnitX());
      const Vec3 axial(0.0, 0.0, q.z() < 0.0 ? -1.0 : 1.0);
      const double dr = rho - p.radius;
      const double dz = std::abs(q.z()) - p.half_height;  // -inf for an infinite pole
      if (dr > 0.0 || dz > 0.0) {
        const double a = std::max(dr, 0.0);
        const double b = std::max(dz, 0.0);
        out.D = std::hypot(a, b);
        out.n = (a * radial + b * axial) / out.D;
      } else if (dr >= dz) {
        out.D = dr;
        out.n = radial;
      } else {
        out.D = dz;
        out.n = axial;
      }
      break;
    }
    case PrimitiveType3::kPlane:
      out.n = p.normal;
      out.D = p.normal.dot(x - p.center);
      break;
  }
  out.x_o = x - out.D * out.n;
  out.valid = std::isfinite(out.D);
  return out;
}

// -----------------------------------------------------------------------------------------------
// Obstacle set

void ObstacleSet3::reservePrimitives(std::size_t count) { primitives_.reserve(count); }

void ObstacleSet3::add(const Primitive3 & primitive)
{
  validatePrimitive3(primitive);
  Primitive3 stored = primitive;
  if (stored.type == PrimitiveType3::kPlane) {
    stored.normal.normalize();
  }
  primitives_.push_back(stored);
}

bool ObstacleSet3::setVoxels(const VoxelGrid3 & grid)
{
  if (grid.version() == grid_.version() && edt_.builtVersion() == grid.version()) {
    return false;
  }
  grid_ = grid;
  return edt_.update(grid_);
}

void ObstacleSet3::clearVoxels() noexcept
{
  grid_ = VoxelGrid3();
  edt_.build(grid_);
}

ObstacleDistance3 ObstacleSet3::nearest(const Vec3 & x) const noexcept
{
  ObstacleDistance3 best;
  for (const Primitive3 & p : primitives_) {
    const ObstacleDistance3 d = signedDistance(p, x);
    if (d.valid && (!best.valid || d.D < best.D)) {
      best = d;
    }
  }
  if (edt_.hasSites()) {
    const ObstacleDistance3 d = edt_.query(x);
    if (d.valid && (!best.valid || d.D < best.D)) {
      best = d;
    }
  }
  return best;
}

namespace
{

/// @brief Keeps out[0, *found) sorted by D (ties keep insertion order), at most `capacity`.
void insertSite(const ObstacleSite3 & site, int capacity, ObstacleSite3 * out, int * found) noexcept
{
  if (*found == capacity && !(site.D < out[*found - 1].D)) {
    return;
  }
  int i = *found < capacity ? (*found)++ : *found - 1;
  while (i > 0 && out[i - 1].D > site.D) {
    out[i] = out[i - 1];
    --i;
  }
  out[i] = site;
}

/// @brief The normal is more than 45° from every voxel normal already taken.
bool distinctVoxelSite(const Vec3 & n, const ObstacleSite3 * taken, int count) noexcept
{
  for (int j = 0; j < count; ++j) {
    if (n.dot(taken[j].n) > kCbfSameObstacleCos) {
      return false;
    }
  }
  return true;
}

/// @brief The voxel sites at x: the continuous query, then the nearest distinct window features.
int voxelSites(
  const DistanceTransform3 & edt, const Vec3 & x, int count, ObstacleSite3 * out) noexcept
{
  const ObstacleDistance3 q = edt.query(x);
  if (!q.valid) {
    return 0;
  }
  int found = 0;
  if (q.D > 0.0) {
    out[found++] = ObstacleSite3{q.x_o, q.n, q.D, -1};
  }
  const double res = edt.resolution();
  const Vec3 rel = (x - edt.origin()) / res;
  const int cx = std::clamp(static_cast<int>(std::floor(rel.x())), 0, edt.nx() - 1);
  const int cy = std::clamp(static_cast<int>(std::floor(rel.y())), 0, edt.ny() - 1);
  const int cz = std::clamp(static_cast<int>(std::floor(rel.z())), 0, edt.nz() - 1);
  const int w = kCbf3WindowCells;
  while (found < count) {
    ObstacleSite3 best;
    bool have = false;
    for (int iz = std::max(cz - w, 0); iz <= std::min(cz + w, edt.nz() - 1); ++iz) {
      for (int iy = std::max(cy - w, 0); iy <= std::min(cy + w, edt.ny() - 1); ++iy) {
        for (int ix = std::max(cx - w, 0); ix <= std::min(cx + w, edt.nx() - 1); ++ix) {
          int fx = 0;
          int fy = 0;
          int fz = 0;
          if (!edt.feature(ix, iy, iz, &fx, &fy, &fz)) {
            continue;
          }
          const Vec3 site = edt.origin() + res * Vec3(fx + 0.5, fy + 0.5, fz + 0.5);
          const Vec3 v = x - site;
          const double D = v.norm();
          if (!(D > 0.0) || (have && !(D < best.D))) {
            continue;
          }
          const Vec3 n = v / D;
          if (distinctVoxelSite(n, out, found)) {
            best = ObstacleSite3{site, n, D, -1};
            have = true;
          }
        }
      }
    }
    if (!have) {
      break;
    }
    out[found++] = best;
  }
  return found;
}

}  // namespace

int ObstacleSet3::sites(const Vec3 & x, int count, ObstacleSite3 * out) const noexcept
{
  count = std::clamp(count, 0, kCbf3MaxSites);
  if (count == 0 || !isFinite(x)) {
    return 0;
  }
  int found = 0;
  for (std::size_t i = 0; i < primitives_.size(); ++i) {
    const ObstacleDistance3 d = signedDistance(primitives_[i], x);
    if (d.valid) {
      insertSite(ObstacleSite3{d.x_o, d.n, d.D, static_cast<int>(i)}, count, out, &found);
    }
  }
  if (edt_.hasSites()) {
    ObstacleSite3 voxel[kCbf3MaxSites];
    const int n = voxelSites(edt_, x, count, voxel);
    for (int j = 0; j < n; ++j) {
      insertSite(voxel[j], count, out, &found);
    }
  }
  return found;
}

// -----------------------------------------------------------------------------------------------
// CBF filter

namespace
{

/// @brief Every constraint holds at F' (with an absolute tolerance).
bool feasible(
  const Vec3 & F, const Cbf3Constraint * c, const double * b, int count, double tol) noexcept
{
  for (int j = 0; j < count; ++j) {
    if (c[j].n.dot(F) < b[j] - tol) {
      return false;
    }
  }
  return true;
}

/// @brief Below this Gram determinant (the squared volume spanned by unit normals) a set of
/// constraints is treated as linearly dependent; a smaller independent subset covers it.
constexpr double kGramSingular = 1e-10;

/// @brief The projection of F onto \f$\{F' : n_j^\top F' = b_j,\ j \in S\}\f$ for |S| ≤ 3;
/// false when the normals are (nearly) dependent.
bool projectOntoActiveSet(
  const Vec3 & F, const Cbf3Constraint * c, const double * b, const int * set, int size,
  Vec3 * out) noexcept
{
  if (size == 1) {
    const Vec3 & n = c[set[0]].n;
    *out = F + (b[set[0]] - n.dot(F)) / n.squaredNorm() * n;
    return true;
  }
  if (size == 2) {
    const Vec3 & a = c[set[0]].n;
    const Vec3 & e = c[set[1]].n;
    const double g11 = a.squaredNorm();
    const double g12 = a.dot(e);
    const double g22 = e.squaredNorm();
    const double det = g11 * g22 - g12 * g12;
    if (!(det > kGramSingular)) {
      return false;
    }
    const double r1 = b[set[0]] - a.dot(F);
    const double r2 = b[set[1]] - e.dot(F);
    const double l1 = (g22 * r1 - g12 * r2) / det;
    const double l2 = (g11 * r2 - g12 * r1) / det;
    *out = F + l1 * a + l2 * e;
    return true;
  }
  Mat3 N;
  N << c[set[0]].n, c[set[1]].n, c[set[2]].n;  // columns
  const Mat3 G = N.transpose() * N;
  const double det = G.determinant();
  if (!(det > kGramSingular)) {
    return false;
  }
  const Vec3 r(
    b[set[0]] - N.col(0).dot(F), b[set[1]] - N.col(1).dot(F), b[set[2]] - N.col(2).dot(F));
  *out = F + N * (G.inverse() * r);
  return true;
}

/// @brief Least-cost feasible candidate over F and every active set of at most three
/// constraints; false if none is feasible.
bool solveActiveSets(
  const Vec3 & F, const Cbf3Constraint * c, const double * b, int count, double tol,
  Vec3 * out) noexcept
{
  bool found = false;
  double best = kInf;
  const auto consider = [&](const Vec3 & candidate) {
    if (!isFinite(candidate) || !feasible(candidate, c, b, count, tol)) {
      return;
    }
    const double cost = (candidate - F).squaredNorm();
    if (cost < best) {
      best = cost;
      *out = candidate;
      found = true;
    }
  };
  consider(F);
  const unsigned subsets = 1U << static_cast<unsigned>(count);
  for (unsigned mask = 1; mask < subsets; ++mask) {
    int set[kCbf3MaxSites] = {0, 0, 0, 0};
    int size = 0;
    for (int j = 0; j < count; ++j) {
      if ((mask & (1U << static_cast<unsigned>(j))) != 0U) {
        set[size++] = j;
      }
    }
    if (size > 3) {
      continue;  // four constraints in R^3 are dependent: a subset of three covers them
    }
    if (size == 1 && !(b[set[0]] - c[set[0]].n.dot(F) > 0.0)) {
      continue;  // a satisfied single constraint: F itself is the candidate
    }
    Vec3 candidate = Vec3::Zero();
    if (projectOntoActiveSet(F, c, b, set, size, &candidate)) {
      consider(candidate);
    }
  }
  return found;
}

}  // namespace

Cbf3Solution solveCbf3(
  const Vec3 & F, const Cbf3Constraint * constraints, int count, double gain) noexcept
{
  Cbf3Solution solution;
  solution.F = F;
  count = std::clamp(count, 0, kCbf3MaxSites);
  if (count == 0 || !isFinite(F)) {
    return solution;
  }
  double b[kCbf3MaxSites] = {0.0, 0.0, 0.0, 0.0};
  double scale = std::max(1.0, F.norm());
  for (int j = 0; j < count; ++j) {
    b[j] = -gain * constraints[j].h;
    scale = std::max(scale, std::abs(b[j]));
  }
  const double tol = 1e-9 * scale;
  Vec3 filtered = F;
  if (!solveActiveSets(F, constraints, b, count, tol, &filtered)) {
    // Conflicting push-outs: hold instead (F' = 0 is feasible once every b_j <= 0).
    solution.relaxed = true;
    for (int j = 0; j < count; ++j) {
      b[j] = std::min(b[j], 0.0);
    }
    if (!solveActiveSets(F, constraints, b, count, tol, &filtered)) {
      filtered = Vec3::Zero();
    }
  }
  // Never faster than the field: binds only with a push-out (some h_j < 0).
  const double norm = filtered.norm();
  const double limit = F.norm();
  if (norm > limit && norm > 0.0) {
    filtered *= limit / norm;
  }
  solution.F = filtered;
  solution.active = !(filtered == F);
  if (solution.active) {
    for (int j = 0; j < count; ++j) {
      if (constraints[j].n.dot(filtered) <= b[j] + tol) {
        solution.binds =
          static_cast<std::uint8_t>(solution.binds | (1U << static_cast<unsigned>(j)));
      }
    }
  }
  return solution;
}

}  // namespace vectorfield_core::d3
