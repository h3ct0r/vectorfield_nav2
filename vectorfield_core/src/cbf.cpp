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

#include "vectorfield_core/cbf.hpp"

#include <algorithm>
#include <cmath>

namespace vectorfield_core
{

namespace
{

/// @brief The normal is more than 45° from every normal already taken.
bool distinctObstacle(const Vec2 & n, const CbfSite * taken, int count) noexcept
{
  for (int j = 0; j < count; ++j) {
    if (n.dot(taken[j].n) > kCbfSameObstacleCos) {
      return false;
    }
  }
  return true;
}

/// @brief The nearest site centre among the window's features that is a distinct obstacle.
bool nearestDistinctFeature(
  const EuclideanDistanceTransform & edt, const Vec2 & x, int cx, int cy, const CbfSite * taken,
  int count, CbfSite * best) noexcept
{
  const double res = edt.resolution();
  const Vec2 & origin = edt.origin();
  bool found = false;
  for (int dy = -kCbfWindowCells; dy <= kCbfWindowCells; ++dy) {
    const int iy = cy + dy;
    if (iy < 0 || iy >= edt.height()) {
      continue;
    }
    for (int dx = -kCbfWindowCells; dx <= kCbfWindowCells; ++dx) {
      const int ix = cx + dx;
      if (ix < 0 || ix >= edt.width()) {
        continue;
      }
      int fx = 0;
      int fy = 0;
      edt.feature(ix, iy, &fx, &fy);
      const Vec2 site(origin.x() + (fx + 0.5) * res, origin.y() + (fy + 0.5) * res);
      const Vec2 v = x - site;
      const double D = v.norm();
      if (!(D > 0.0) || (found && !(D < best->D))) {
        continue;
      }
      const Vec2 n = v / D;
      if (distinctObstacle(n, taken, count)) {
        *best = CbfSite{site, n, D};
        found = true;
      }
    }
  }
  return found;
}

/// @brief Every constraint holds at F' (with a relative tolerance).
bool feasible(
  const Vec2 & F, const CbfConstraint * c, const double * b, int count, double tol) noexcept
{
  for (int j = 0; j < count; ++j) {
    if (c[j].n.dot(F) < b[j] - tol) {
      return false;
    }
  }
  return true;
}

/// @brief Least-cost feasible candidate of the active-set enumeration; false if none is feasible.
bool solveActiveSets(
  const Vec2 & F, const CbfConstraint * c, const double * b, int count, double tol,
  Vec2 * out) noexcept
{
  bool found = false;
  double best = kInf;
  const auto consider = [&](const Vec2 & candidate) {
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
  for (int i = 0; i < count; ++i) {
    const double violation = b[i] - c[i].n.dot(F);
    if (violation > 0.0) {
      consider(Vec2(F + violation * c[i].n));
    }
  }
  for (int i = 0; i < count; ++i) {
    for (int j = i + 1; j < count; ++j) {
      const Vec2 & a = c[i].n;
      const Vec2 & e = c[j].n;
      const double det = a.x() * e.y() - a.y() * e.x();
      if (std::abs(det) < 1e-9) {
        continue;  // parallel: a single projection covers the pair
      }
      consider(Vec2((b[i] * e.y() - b[j] * a.y()) / det, (a.x() * b[j] - e.x() * b[i]) / det));
    }
  }
  return found;
}

}  // namespace

int nearestCbfSites(
  const EuclideanDistanceTransform & edt, const Vec2 & x_grid, int count, CbfSite * out) noexcept
{
  count = std::clamp(count, 0, kCbfMaxSites);
  if (count == 0 || !edt.hasSites()) {
    return 0;
  }
  const ObstacleQuery query = edt.query(x_grid);
  if (!query.valid) {
    return 0;
  }
  int found = 0;
  if (query.D_o > 0.0) {
    out[found++] = CbfSite{query.x_o, query.n_o, query.D_o};
  }
  if (found == count) {
    return found;
  }
  const double res = edt.resolution();
  const Vec2 rel = (x_grid - edt.origin()) / res;
  const int cx = std::clamp(static_cast<int>(std::floor(rel.x())), 0, edt.width() - 1);
  const int cy = std::clamp(static_cast<int>(std::floor(rel.y())), 0, edt.height() - 1);
  while (found < count) {
    CbfSite site;
    if (!nearestDistinctFeature(edt, x_grid, cx, cy, out, found, &site)) {
      break;
    }
    out[found++] = site;
  }
  return found;
}

CbfSolution solveCbf(
  const Vec2 & F, const CbfConstraint * constraints, int count, double gain) noexcept
{
  CbfSolution solution;
  solution.F = F;
  count = std::clamp(count, 0, kCbfMaxSites);
  if (count == 0 || !isFinite(F)) {
    return solution;
  }
  double b[kCbfMaxSites] = {0.0, 0.0, 0.0};
  double scale = std::max(1.0, F.norm());
  for (int j = 0; j < count; ++j) {
    b[j] = -gain * constraints[j].h;
    scale = std::max(scale, std::abs(b[j]));
  }
  const double tol = 1e-9 * scale;
  Vec2 filtered = F;
  if (!solveActiveSets(F, constraints, b, count, tol, &filtered)) {
    // Conflicting push-outs: hold instead (F' = 0 is feasible once every b_j <= 0).
    solution.relaxed = true;
    for (int j = 0; j < count; ++j) {
      b[j] = std::min(b[j], 0.0);
    }
    if (!solveActiveSets(F, constraints, b, count, tol, &filtered)) {
      filtered = Vec2::Zero();
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
        solution.binds = static_cast<std::uint8_t>(solution.binds | (1U << j));
      }
    }
  }
  return solution;
}

}  // namespace vectorfield_core
