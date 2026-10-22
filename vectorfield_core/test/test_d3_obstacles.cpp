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

// 3D obstacles (math_spec §14.8): primitive signed distances, the voxel EDT against brute force
// (O3D-15), the CBF3 solver against its KKT conditions (O3D-16), sites and validation.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "vectorfield_core/d3.hpp"
#include "vectorfield_core/errors.hpp"

namespace
{

namespace d3 = vectorfield_core::d3;
using d3::Vec3;
using vectorfield_core::ConfigError;

Vec3 randomUnit(std::mt19937 & rng)
{
  std::normal_distribution<double> g(0.0, 1.0);
  Vec3 v(g(rng), g(rng), g(rng));
  return v.normalized();
}

Vec3 randomPoint(std::mt19937 & rng, double half)
{
  std::uniform_real_distribution<double> u(-half, half);
  return Vec3(u(rng), u(rng), u(rng));
}

/// Points on the surface of a primitive (planes: near `around`).
std::vector<Vec3> surfaceSamples(const d3::Primitive3 & p, std::mt19937 & rng, const Vec3 & around)
{
  std::vector<Vec3> out;
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const int n = 3000;
  for (int i = 0; i < n; ++i) {
    switch (p.type) {
      case d3::PrimitiveType3::kSphere:
        out.push_back(p.center + p.radius * randomUnit(rng));
        break;
      case d3::PrimitiveType3::kBox: {
        Vec3 q(u(rng), u(rng), u(rng));
        const int axis = i % 3;
        q[axis] = (i / 3) % 2 == 0 ? 1.0 : -1.0;
        out.push_back(p.center + p.rotation * q.cwiseProduct(p.half_extents));
        break;
      }
      case d3::PrimitiveType3::kCylinder: {
        const double a = M_PI * u(rng);
        const double H = std::isfinite(p.half_height) ? p.half_height : 6.0;
        if (i % 3 == 0 && std::isfinite(p.half_height)) {
          const double r = p.radius * std::sqrt(0.5 * (u(rng) + 1.0));
          out.push_back(
            p.center + Vec3(r * std::cos(a), r * std::sin(a), (i % 2 == 0 ? 1.0 : -1.0) * H));
        } else {
          out.push_back(
            p.center + Vec3(p.radius * std::cos(a), p.radius * std::sin(a), H * u(rng)));
        }
        break;
      }
      case d3::PrimitiveType3::kPlane: {
        const Vec3 t1 = p.normal.unitOrthogonal();
        const Vec3 t2 = p.normal.cross(t1);
        const Vec3 base = around - p.normal.dot(around - p.center) * p.normal;
        out.push_back(base + u(rng) * t1 + u(rng) * t2);
        break;
      }
    }
  }
  return out;
}

void checkPrimitive(const d3::Primitive3 & p, std::uint32_t seed)
{
  d3::validatePrimitive3(p);
  std::mt19937 rng(seed);
  for (int k = 0; k < 150; ++k) {
    const Vec3 x = p.center + randomPoint(rng, 2.5);
    const d3::ObstacleDistance3 d = d3::signedDistance(p, x);
    ASSERT_TRUE(d.valid);
    EXPECT_NEAR(d.n.norm(), 1.0, 1e-12);
    // x_o = x - D n lies on the surface, so |D| is attained ...
    EXPECT_NEAR(d3::signedDistance(p, d.x_o).D, 0.0, 1e-9) << x.transpose();
    EXPECT_NEAR((x - d.x_o).norm(), std::abs(d.D), 1e-12);
    // ... and no surface point is nearer (|D| is the distance to the surface).
    double nearest = vectorfield_core::kInf;
    for (const Vec3 & y : surfaceSamples(p, rng, x)) {
      nearest = std::min(nearest, (x - y).norm());
    }
    EXPECT_LE(std::abs(d.D), nearest + 1e-12) << x.transpose();
    EXPECT_LE(nearest - std::abs(d.D), 0.05) << x.transpose();  // sampling density
    // n is the gradient of D (away from the medial set).
    const double e = 1e-6;
    const double slope =
      (d3::signedDistance(p, x + e * d.n).D - d3::signedDistance(p, x - e * d.n).D) / (2 * e);
    if (std::abs(d.D) > 1e-3) {
      EXPECT_NEAR(slope, 1.0, 1e-5) << x.transpose();
    }
  }
}

}  // namespace

TEST(D3Obstacles, SphereSignedDistance)
{
  checkPrimitive(d3::Primitive3::sphere(Vec3(0.3, -0.2, 1.0), 0.7), 1);
  const auto d = d3::signedDistance(d3::Primitive3::sphere(Vec3(1, 2, 3), 0.5), Vec3(1, 2, 3));
  EXPECT_DOUBLE_EQ(d.D, -0.5);  // the centre: a fixed normal
  EXPECT_EQ(d.n, Vec3::UnitZ());
}

TEST(D3Obstacles, BoxSignedDistance)
{
  checkPrimitive(d3::Primitive3::box(Vec3(0.1, 0.2, 0.3), Vec3(0.4, 0.8, 0.25)), 2);
  checkPrimitive(
    d3::Primitive3::box(Vec3(-0.5, 0.0, 1.0), Vec3(0.6, 0.3, 0.9), d3::rotationZYX(0.7, -0.4, 0.3)),
    3);
  const auto box = d3::Primitive3::box(Vec3::Zero(), Vec3(1, 1, 1));
  EXPECT_NEAR(d3::signedDistance(box, Vec3(2, 2, 1)).D, std::sqrt(2.0), 1e-15);  // an edge
  EXPECT_NEAR(d3::signedDistance(box, Vec3(2, 2, 2)).D, std::sqrt(3.0), 1e-15);  // a corner
  EXPECT_NEAR(d3::signedDistance(box, Vec3(0.2, 0.9, 0.0)).D, -0.1, 1e-15);      // inside
}

TEST(D3Obstacles, CylinderSignedDistance)
{
  checkPrimitive(d3::Primitive3::cylinder(Vec3(0.2, -0.1, 0.5), 0.35, 0.8), 4);
  checkPrimitive(d3::Primitive3::cylinder(Vec3(0.2, -0.1, 0.5), 0.35), 5);  // an infinite pole
  const auto pole = d3::Primitive3::cylinder(Vec3(1, 1, 0), 0.5);
  const auto d = d3::signedDistance(pole, Vec3(1, 3, 100));
  EXPECT_DOUBLE_EQ(d.D, 1.5);
  EXPECT_EQ(d.n, Vec3::UnitY());
}

TEST(D3Obstacles, PlaneSignedDistance)
{
  checkPrimitive(d3::Primitive3::plane(Vec3(0, 0, 0.2), Vec3(0.2, -0.3, 1.0)), 6);
  const auto ground = d3::Primitive3::plane(Vec3::Zero(), Vec3(0, 0, 3));
  EXPECT_EQ(ground.normal, Vec3::UnitZ());
  EXPECT_DOUBLE_EQ(d3::signedDistance(ground, Vec3(5, -2, -0.25)).D, -0.25);
}

TEST(D3Obstacles, PrimitiveValidation)
{
  EXPECT_THROW(d3::validatePrimitive3(d3::Primitive3::sphere(Vec3::Zero(), 0.0)), ConfigError);
  EXPECT_THROW(
    d3::validatePrimitive3(d3::Primitive3::sphere(Vec3(0, vectorfield_core::kNaN, 0), 1.0)),
    ConfigError);
  d3::Mat3 shear = d3::Mat3::Identity();
  shear(0, 1) = 0.1;
  EXPECT_THROW(
    d3::validatePrimitive3(d3::Primitive3::box(Vec3::Zero(), Vec3(1, 1, 1), shear)), ConfigError);
  EXPECT_THROW(
    d3::validatePrimitive3(d3::Primitive3::box(Vec3::Zero(), Vec3(1, 0, 1))), ConfigError);
  EXPECT_THROW(
    d3::validatePrimitive3(d3::Primitive3::cylinder(Vec3::Zero(), 1.0, 0.0)), ConfigError);
  d3::Primitive3 plane = d3::Primitive3::plane(Vec3::Zero(), Vec3::UnitZ());
  plane.normal = Vec3::Zero();
  EXPECT_THROW(d3::validatePrimitive3(plane), ConfigError);
  d3::ObstacleSet3 set;
  EXPECT_THROW(set.add(plane), ConfigError);
  EXPECT_TRUE(set.empty());
}

// O3D-15: the voxel EDT equals brute force (exact integer squared distances, and each feature is
// an occupied voxel at that distance), on random grids of several shapes and densities.
TEST(D3Obstacles, VoxelEdtMatchesBruteForce)
{
  std::mt19937 rng(42);
  const int shapes[][3] = {{7, 5, 9}, {16, 16, 16}, {1, 1, 23}, {3, 31, 2}, {12, 1, 9}, {1, 1, 1}};
  const double densities[] = {0.002, 0.03, 0.2, 0.6};
  for (const auto & shape : shapes) {
    for (const double density : densities) {
      const int nx = shape[0];
      const int ny = shape[1];
      const int nz = shape[2];
      std::vector<std::uint8_t> mask(static_cast<std::size_t>(nx * ny * nz));
      std::bernoulli_distribution site(density);
      for (auto & m : mask) {
        m = site(rng) ? 1 : 0;
      }
      d3::VoxelGrid3 grid;
      ASSERT_TRUE(grid.setFromMask(mask, nx, ny, nz, 0.1, Vec3(-1, 0.5, 2)));
      d3::DistanceTransform3 edt;
      ASSERT_TRUE(edt.update(grid));
      EXPECT_FALSE(edt.update(grid));  // unchanged: no rebuild
      const bool any = std::any_of(mask.begin(), mask.end(), [](auto m) { return m != 0; });
      ASSERT_EQ(edt.hasSites(), any);
      for (int iz = 0; iz < nz; ++iz) {
        for (int iy = 0; iy < ny; ++iy) {
          for (int ix = 0; ix < nx; ++ix) {
            double best = vectorfield_core::kInf;
            for (int z = 0; z < nz; ++z) {
              for (int y = 0; y < ny; ++y) {
                for (int x = 0; x < nx; ++x) {
                  if (grid.isSite(x, y, z)) {
                    const double d2 =
                      (x - ix) * (x - ix) + (y - iy) * (y - iy) + (z - iz) * (z - iz);
                    best = std::min(best, d2);
                  }
                }
              }
            }
            ASSERT_EQ(edt.squaredDistanceCells(ix, iy, iz), best) << ix << " " << iy << " " << iz;
            int fx = 0;
            int fy = 0;
            int fz = 0;
            ASSERT_EQ(edt.feature(ix, iy, iz, &fx, &fy, &fz), any);
            if (any) {
              ASSERT_TRUE(grid.isSite(fx, fy, fz));
              EXPECT_EQ(
                (fx - ix) * (fx - ix) + (fy - iy) * (fy - iy) + (fz - iz) * (fz - iz), best);
            }
          }
        }
      }
    }
  }
}

TEST(D3Obstacles, VoxelGridVersionAndQuery)
{
  const int n = 20;
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(n * n * n), 0);
  mask[static_cast<std::size_t>(10 + n * (10 + n * 10))] = 1;  // one voxel at the centre
  d3::VoxelGrid3 grid;
  ASSERT_FALSE(grid.setFromMask(mask, n, n, n, 0.0, Vec3::Zero()));  // bad resolution
  ASSERT_TRUE(grid.setFromMask(mask, n, n, n, 0.1, Vec3::Zero()));
  const std::uint64_t v1 = grid.version();
  ASSERT_TRUE(grid.setFromMask(mask, n, n, n, 0.1, Vec3::Zero()));
  EXPECT_EQ(grid.version(), v1);  // same content, same stamp
  d3::ObstacleSet3 set;
  EXPECT_TRUE(set.setVoxels(grid));
  EXPECT_FALSE(set.setVoxels(grid));
  ASSERT_TRUE(set.hasVoxels());
  // The continuous query of a single site is the exact distance to its centre.
  const Vec3 centre = grid.cellCenter(10, 10, 10);
  std::mt19937 rng(7);
  for (int k = 0; k < 200; ++k) {
    const Vec3 x = centre + randomPoint(rng, 0.9);
    const d3::ObstacleDistance3 q = set.voxelDistance().query(x);
    ASSERT_TRUE(q.valid);
    EXPECT_NEAR(q.D, (x - centre).norm(), 1e-12);
    EXPECT_NEAR((q.x_o - centre).norm(), 0.0, 1e-12);
  }
  EXPECT_FALSE(set.voxelDistance().query(Vec3(-0.01, 1, 1)).valid);  // outside the grid
  mask[0] = 1;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, n, 0.1, Vec3::Zero()));
  EXPECT_NE(grid.version(), v1);
  EXPECT_TRUE(set.setVoxels(grid));
  set.clearVoxels();
  EXPECT_TRUE(set.empty());
}

TEST(D3Obstacles, SitesAreNearestFirstOnePerPrimitive)
{
  d3::ObstacleSet3 set;
  set.add(d3::Primitive3::sphere(Vec3(2, 0, 1), 0.5));
  set.add(d3::Primitive3::plane(Vec3::Zero(), Vec3::UnitZ()));
  set.add(d3::Primitive3::box(Vec3(0, -3, 1), Vec3(0.5, 0.5, 0.5)));
  set.add(d3::Primitive3::cylinder(Vec3(-1.5, 0, 0), 0.2));
  set.add(d3::Primitive3::sphere(Vec3(0, 5, 1), 0.5));
  const Vec3 x(0, 0, 1);
  d3::ObstacleSite3 sites[d3::kCbf3MaxSites];
  const int count = set.sites(x, 4, sites);
  ASSERT_EQ(count, 4);
  EXPECT_EQ(sites[0].source, 1);  // the plane, D = 1
  EXPECT_EQ(sites[1].source, 3);  // the pole, D = 1.3
  EXPECT_EQ(sites[2].source, 0);  // the sphere, D = 1.5
  EXPECT_EQ(sites[3].source, 2);  // the box, D = 2.5
  EXPECT_NEAR(sites[1].D, 1.3, 1e-12);
  EXPECT_EQ(set.sites(x, 2, sites), 2);
  EXPECT_EQ(set.sites(x, 9, sites), 4);  // clamped to kCbf3MaxSites
  EXPECT_NEAR(set.nearest(x).D, 1.0, 1e-15);
}

TEST(D3Obstacles, VoxelSitesFindBothWallsOfACorridor)
{
  // Two voxel slabs at y = 0 and y = 1.0 (0.1 m voxels); the body midway sees both.
  const int nx = 20;
  const int ny = 11;
  const int nz = 10;
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(nx * ny * nz), 0);
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      mask[static_cast<std::size_t>(ix + nx * (0 + ny * iz))] = 1;
      mask[static_cast<std::size_t>(ix + nx * (10 + ny * iz))] = 1;
    }
  }
  d3::VoxelGrid3 grid;
  ASSERT_TRUE(grid.setFromMask(mask, nx, ny, nz, 0.1, Vec3::Zero()));
  d3::ObstacleSet3 set;
  set.setVoxels(grid);
  d3::ObstacleSite3 sites[d3::kCbf3MaxSites];
  const int count = set.sites(Vec3(1.05, 0.68, 0.55), 3, sites);
  ASSERT_EQ(count, 2);  // one site per wall: the rest of each wall is within 45 degrees
  EXPECT_NEAR(sites[0].n.y(), -1.0, 1e-9);  // the upper wall, 0.37 m away
  EXPECT_NEAR(sites[0].D, 0.37, 1e-9);
  EXPECT_NEAR(sites[1].n.y(), 1.0, 1e-9);  // the lower wall, 0.63 m away
  EXPECT_NEAR(sites[1].D, 0.63, 1e-9);
  EXPECT_EQ(sites[0].source, -1);
}

namespace
{

/// The KKT conditions of min |F' - F|^2 s.t. n_j^T F' >= b_j at F' = Fs (sufficient for this
/// convex QP): F' - F = sum lambda_j n_j with lambda_j >= 0 over the binding constraints only
/// (non-negative least squares by subsets), an oracle independent of the primal enumeration.
bool kktHolds(const Vec3 & F, const Vec3 & Fs, const d3::Cbf3Constraint * c, int count, double gain)
{
  const Vec3 r = Fs - F;
  int binding[d3::kCbf3MaxSites];
  int nb = 0;
  for (int j = 0; j < count; ++j) {
    if (c[j].n.dot(Fs) <= -gain * c[j].h + 1e-9) {
      binding[nb++] = j;
    }
  }
  for (unsigned mask = 0; mask < (1U << nb); ++mask) {
    Eigen::Matrix<double, 3, Eigen::Dynamic> N(3, __builtin_popcount(mask));
    int col = 0;
    for (int k = 0; k < nb; ++k) {
      if ((mask & (1U << k)) != 0U) {
        N.col(col++) = c[binding[k]].n;
      }
    }
    if (col == 0) {
      if (r.norm() < 1e-9) {
        return true;
      }
      continue;
    }
    const Eigen::VectorXd lambda = N.colPivHouseholderQr().solve(r);
    if ((N * lambda - r).norm() < 1e-9 && lambda.minCoeff() >= -1e-9) {
      return true;
    }
  }
  return false;
}

}  // namespace

// O3D-16: with every h_j >= 0 the solver returns the exact projection (feasible and KKT).
TEST(D3Obstacles, CbfSolverIsTheExactProjection)
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> h(0.0, 0.4);
  std::uniform_int_distribution<int> k(1, d3::kCbf3MaxSites);
  int active = 0;
  for (int trial = 0; trial < 4000; ++trial) {
    const int count = k(rng);
    d3::Cbf3Constraint c[d3::kCbf3MaxSites];
    for (int j = 0; j < count; ++j) {
      c[j].n = randomUnit(rng);
      c[j].h = h(rng);
    }
    const Vec3 F = 0.5 * randomUnit(rng);
    const d3::Cbf3Solution s = d3::solveCbf3(F, c, count, 2.0);
    ASSERT_FALSE(s.relaxed);
    for (int j = 0; j < count; ++j) {
      EXPECT_GE(c[j].n.dot(s.F), -2.0 * c[j].h - 1e-12);
    }
    EXPECT_LE(s.F.norm(), F.norm() + 1e-15);
    EXPECT_TRUE(kktHolds(F, s.F, c, count, 2.0)) << "trial " << trial;
    active += s.active ? 1 : 0;
  }
  EXPECT_GT(active, 1000);
}

TEST(D3Obstacles, CbfSolverCornersAndPushOuts)
{
  // Floor and two walls: F into the corner is stopped on all three faces.
  d3::Cbf3Constraint c[4];
  c[0] = {Vec3::UnitZ(), 0.0};
  c[1] = {Vec3::UnitX(), 0.0};
  c[2] = {Vec3::UnitY(), 0.0};
  c[3] = {Vec3(1, 1, 0).normalized(), 0.0};  // dependent on the walls
  d3::Cbf3Solution s = d3::solveCbf3(Vec3(-0.3, -0.2, -0.1), c, 4, 2.0);
  EXPECT_TRUE(s.active);
  EXPECT_NEAR(s.F.norm(), 0.0, 1e-15);
  // Sliding along the floor.
  s = d3::solveCbf3(Vec3(0.3, 0.2, -0.1), c, 1, 2.0);
  EXPECT_NEAR((s.F - Vec3(0.3, 0.2, 0.0)).norm(), 0.0, 1e-15);
  EXPECT_EQ(s.binds, 1U);
  // Conflicting push-outs (inside two opposite walls) are dropped: a hold.
  c[0] = {Vec3::UnitY(), -0.1};
  c[1] = {-Vec3::UnitY(), -0.1};
  s = d3::solveCbf3(Vec3(0.5, 0.0, 0.0), c, 2, 2.0);
  EXPECT_TRUE(s.relaxed);
  EXPECT_NEAR((s.F - Vec3(0.5, 0.0, 0.0)).norm(), 0.0, 1e-15);
  // A push-out is bounded by |F|.
  c[0] = {Vec3::UnitY(), -1.0};
  s = d3::solveCbf3(Vec3(0.3, 0.0, 0.0), c, 1, 2.0);
  EXPECT_NEAR(s.F.norm(), 0.3, 1e-12);
  EXPECT_GT(s.F.y(), 0.0);
  // Inactive: F unchanged bit for bit.
  c[0] = {Vec3::UnitY(), 1.0};
  s = d3::solveCbf3(Vec3(0.3, -0.4, 0.0), c, 1, 2.0);
  EXPECT_FALSE(s.active);
  EXPECT_EQ(s.F, Vec3(0.3, -0.4, 0.0));
}

TEST(D3Obstacles, ConfigValidation)
{
  const auto throws = [](auto mutate) {
    d3::Tracker3Config c;
    mutate(c);
    EXPECT_THROW(d3::validateTracker3Config(c), ConfigError);
  };
  throws([](auto & c) { c.obstacles.obstacle_mode = vectorfield_core::ObstacleMode::kContour; });
  throws([](auto & c) { c.obstacles.body_radius = -0.1; });
  throws([](auto & c) { c.obstacles.cbf_margin = vectorfield_core::kNaN; });
  throws([](auto & c) { c.obstacles.cbf_gain = 0.0; });
  throws([](auto & c) { c.obstacles.cbf_gain = 20.0; });  // gamma_b dt = 1 at 20 Hz
  throws([](auto & c) { c.obstacles.cbf_sites = 0; });
  throws([](auto & c) { c.obstacles.cbf_sites = 5; });
  d3::Tracker3Config ok;
  ok.obstacles.obstacle_mode = vectorfield_core::ObstacleMode::kCbf;
  EXPECT_NO_THROW(d3::validateTracker3Config(ok));
}
