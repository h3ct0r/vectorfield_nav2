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

// The voxel grid and its exact 3D Euclidean distance and feature transform (math_spec §14.8),
// the separable Felzenszwalb–Huttenlocher transform of occupancy.cpp with one more axis.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

#include "vectorfield_core/d3/obstacles.hpp"

namespace vectorfield_core::d3
{

namespace
{

/// Largest accepted side (voxels). Squared distances stay below 3·2^40 and the envelope's
/// cross-multiplications below 2^62, so both are exact (double, int64).
constexpr int kMaxSide = 1 << 20;

/// Feature entry of a voxel when the transform has no site.
constexpr int kNoFeature = -1;

/// Source of the process-wide content stamps of VoxelGrid3; 0 is the empty grid.
std::atomic<std::uint64_t> g_next_voxel_version{1};

std::uint64_t newVersion() noexcept
{
  return g_next_voxel_version.fetch_add(1, std::memory_order_relaxed);
}

inline int clampIndex(int i, int n) noexcept { return i < 0 ? 0 : (i >= n ? n - 1 : i); }

/// One 1-D pass on a gathered line: the lower envelope of the parabolas \f$(q - p)^2 + g(p)\f$
/// over the finite g (Felzenszwalb and Huttenlocher, Theory of Computing 8, 2012, Algorithm 1),
/// tracking the argmin's feature. Comparisons use exact int64 cross-multiplication of
/// \f$F(p) = g(p) + p^2\f$ (see lowerEnvelopeRow() in occupancy.cpp); ties go to the lower p.
/// @param g In: squared distance per position (+inf: none).
/// @param feature In: feature per position.
/// @param n Line length.
/// @param apex Work (n doubles).
/// @param vertices Work (n ints).
/// @param out Out: squared distance per position.
/// @param feature_out Out: feature per position.
void lowerEnvelope(
  const double * g, const int * feature, int n, double * apex, int * vertices, double * out,
  int * feature_out) noexcept
{
  int k = -1;
  for (int q = 0; q < n; ++q) {
    if (!(g[q] < kInf)) {
      continue;
    }
    apex[q] = g[q] + static_cast<double>(q) * static_cast<double>(q);
    const auto apex_q = static_cast<std::int64_t>(apex[q]);
    while (k >= 1) {
      const int a = vertices[k - 1];
      const int b = vertices[k];
      const auto apex_a = static_cast<std::int64_t>(apex[a]);
      const auto apex_b = static_cast<std::int64_t>(apex[b]);
      if ((apex_q - apex_b) * (b - a) <= (apex_b - apex_a) * (q - b)) {
        --k;
      } else {
        break;
      }
    }
    vertices[++k] = q;
  }
  if (k < 0) {
    std::fill(out, out + n, kInf);
    std::fill(feature_out, feature_out + n, kNoFeature);
    return;
  }
  int j = 0;
  for (int q = 0; q < n; ++q) {
    while (j < k) {
      const int a = vertices[j];
      const int b = vertices[j + 1];
      const std::int64_t rise =
        static_cast<std::int64_t>(apex[b]) - static_cast<std::int64_t>(apex[a]);
      if (rise < 2 * static_cast<std::int64_t>(q) * (b - a)) {
        ++j;
      } else {
        break;
      }
    }
    const int p = vertices[j];
    const double d = static_cast<double>(q - p);
    out[q] = d * d + g[p];
    feature_out[q] = feature[p];
  }
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// VoxelGrid3

VoxelGrid3::VoxelGrid3() noexcept {}

void VoxelGrid3::reserve(int nx, int ny, int nz)
{
  if (nx > 0 && ny > 0 && nz > 0) {
    mask_.reserve(
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz));
  }
}

bool VoxelGrid3::setFromMask(
  Span<const std::uint8_t> mask, int nx, int ny, int nz, double resolution,
  const Vec3 & origin) noexcept
{
  if (nx < 1 || ny < 1 || nz < 1 || nx > kMaxSide || ny > kMaxSide || nz > kMaxSide) {
    return false;
  }
  const std::size_t cells =
    static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
  if (
    cells > static_cast<std::size_t>(std::numeric_limits<int>::max()) || mask.size() != cells ||
    !(std::isfinite(resolution) && resolution > 0.0) || !isFinite(origin)) {
    return false;
  }
  bool changed = nx != nx_ || ny != ny_ || nz != nz_ || resolution != resolution_ ||
                 origin != origin_ || mask_.size() != cells;
  if (mask_.size() != cells) {  // allocates only beyond the reserved capacity
    mask_.resize(cells);
  }
  for (std::size_t i = 0; i < cells; ++i) {
    const std::uint8_t site = mask[i] != 0 ? 1 : 0;
    if (mask_[i] != site) {
      mask_[i] = site;
      changed = true;
    }
  }
  nx_ = nx;
  ny_ = ny;
  nz_ = nz;
  resolution_ = resolution;
  origin_ = origin;
  if (changed || version_ == 0) {
    version_ = newVersion();
  }
  return true;
}

Span<const std::uint8_t> VoxelGrid3::mask() const noexcept
{
  return Span<const std::uint8_t>(mask_.data(), mask_.size());
}

bool VoxelGrid3::inBounds(int ix, int iy, int iz) const noexcept
{
  return ix >= 0 && iy >= 0 && iz >= 0 && ix < nx_ && iy < ny_ && iz < nz_;
}

bool VoxelGrid3::isSite(int ix, int iy, int iz) const noexcept
{
  if (!inBounds(ix, iy, iz)) {
    return false;
  }
  const std::size_t i =
    static_cast<std::size_t>(ix) +
    static_cast<std::size_t>(nx_) *
      (static_cast<std::size_t>(iy) + static_cast<std::size_t>(ny_) * static_cast<std::size_t>(iz));
  return mask_[i] != 0;
}

Vec3 VoxelGrid3::cellCenter(int ix, int iy, int iz) const noexcept
{
  return origin_ + resolution_ * Vec3(ix + 0.5, iy + 0.5, iz + 0.5);
}

// -----------------------------------------------------------------------------------------------
// DistanceTransform3

DistanceTransform3::DistanceTransform3() noexcept {}

void DistanceTransform3::reserve(int nx, int ny, int nz)
{
  if (nx < 1 || ny < 1 || nz < 1) {
    return;
  }
  const std::size_t cells =
    static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
  const auto side = static_cast<std::size_t>(std::max({nx, ny, nz}));
  squared_.reserve(cells);
  feature_.reserve(cells);
  plane_.reserve(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny));
  line_g_.resize(std::max(line_g_.size(), side));
  line_apex_.resize(std::max(line_apex_.size(), side));
  line_out_.resize(std::max(line_out_.size(), side));
  line_feature_.resize(std::max(line_feature_.size(), side));
  line_vertices_.resize(std::max(line_vertices_.size(), side));
  line_feature_out_.resize(std::max(line_feature_out_.size(), side));
}

bool DistanceTransform3::update(const VoxelGrid3 & grid) noexcept
{
  if (
    grid.version() == built_version_ && grid.nx() == nx_ && grid.ny() == ny_ && grid.nz() == nz_) {
    return false;
  }
  build(grid);
  return true;
}

void DistanceTransform3::build(const VoxelGrid3 & grid) noexcept
{
  nx_ = grid.nx();
  ny_ = grid.ny();
  nz_ = grid.nz();
  resolution_ = grid.resolution();
  origin_ = grid.origin();
  built_version_ = grid.version();
  has_sites_ = false;
  const std::size_t cells = grid.size();
  if (cells == 0) {
    squared_.clear();
    feature_.clear();
    return;
  }
  // Growing here is the documented exception (a larger grid than reserved): the vectors never
  // shrink, so steady-state rebuilds do not allocate.
  reserve(nx_, ny_, nz_);
  squared_.resize(cells);
  feature_.resize(cells);
  const Span<const std::uint8_t> mask = grid.mask();
  const std::size_t sx = 1;
  const std::size_t sy = static_cast<std::size_t>(nx_);
  const std::size_t sz = sy * static_cast<std::size_t>(ny_);

  double * g = line_g_.data();
  int * f = line_feature_.data();
  double * apex = line_apex_.data();
  int * vertices = line_vertices_.data();
  double * out = line_out_.data();
  int * f_out = line_feature_out_.data();

  // Pass 1 along z, by whole xy planes for locality (the planar transform's column pass): the
  // nearest occupied voxel of each column, ties to the lower z. `plane_` holds the z of the last
  // site seen per column; the downward sweep parks it in feature_.
  bool any = false;
  plane_.assign(sz, kNoFeature);
  for (int iz = 0; iz < nz_; ++iz) {
    const std::size_t base = static_cast<std::size_t>(iz) * sz;
    for (std::size_t c = 0; c < sz; ++c) {
      if (mask[base + c] != 0) {
        plane_[c] = iz;
        any = true;
      }
      feature_[base + c] = plane_[c];
    }
  }
  std::fill(plane_.begin(), plane_.end(), kNoFeature);
  for (int iz = nz_ - 1; iz >= 0; --iz) {
    const std::size_t base = static_cast<std::size_t>(iz) * sz;
    for (std::size_t c = 0; c < sz; ++c) {
      if (mask[base + c] != 0) {
        plane_[c] = iz;
      }
      const int below = feature_[base + c];
      const int above = plane_[c];
      const int d_below = below == kNoFeature ? std::numeric_limits<int>::max() : iz - below;
      const int d_above = above == kNoFeature ? std::numeric_limits<int>::max() : above - iz;
      const int z = d_below <= d_above ? below : above;
      if (z == kNoFeature) {
        squared_[base + c] = kInf;
        feature_[base + c] = kNoFeature;
      } else {
        const double d = static_cast<double>(z - iz);
        squared_[base + c] = d * d;
        feature_[base + c] = static_cast<int>(static_cast<std::size_t>(z) * sz + c);
      }
    }
  }
  if (!any) {
    std::fill(squared_.begin(), squared_.end(), kInf);
    std::fill(feature_.begin(), feature_.end(), kNoFeature);
    return;
  }
  // Passes 2 and 3 along y, then x, on the previous pass's squared distances and features.
  const auto pass = [&](
                      int n, std::size_t stride, int outer_a, std::size_t stride_a, int outer_b,
                      std::size_t stride_b) {
    for (int b = 0; b < outer_b; ++b) {
      for (int a = 0; a < outer_a; ++a) {
        const std::size_t base =
          static_cast<std::size_t>(a) * stride_a + static_cast<std::size_t>(b) * stride_b;
        for (int q = 0; q < n; ++q) {
          const std::size_t i = base + static_cast<std::size_t>(q) * stride;
          g[q] = squared_[i];
          f[q] = feature_[i];
        }
        lowerEnvelope(g, f, n, apex, vertices, out, f_out);
        for (int q = 0; q < n; ++q) {
          const std::size_t i = base + static_cast<std::size_t>(q) * stride;
          squared_[i] = out[q];
          feature_[i] = f_out[q];
        }
      }
    }
  };
  pass(ny_, sy, nx_, sx, nz_, sz);
  pass(nx_, sx, ny_, sy, nz_, sz);
  has_sites_ = true;
}

double DistanceTransform3::squaredDistanceCells(int ix, int iy, int iz) const noexcept
{
  const std::size_t i =
    static_cast<std::size_t>(ix) +
    static_cast<std::size_t>(nx_) *
      (static_cast<std::size_t>(iy) + static_cast<std::size_t>(ny_) * static_cast<std::size_t>(iz));
  return squared_[i];
}

double DistanceTransform3::distance(int ix, int iy, int iz) const noexcept
{
  return resolution_ * std::sqrt(squaredDistanceCells(ix, iy, iz));
}

bool DistanceTransform3::feature(
  int ix, int iy, int iz, int * fx, int * fy, int * fz) const noexcept
{
  const std::size_t i =
    static_cast<std::size_t>(ix) +
    static_cast<std::size_t>(nx_) *
      (static_cast<std::size_t>(iy) + static_cast<std::size_t>(ny_) * static_cast<std::size_t>(iz));
  const int index = feature_[i];
  if (index < 0) {
    *fx = -1;
    *fy = -1;
    *fz = -1;
    return false;
  }
  *fx = index % nx_;
  *fy = (index / nx_) % ny_;
  *fz = index / (nx_ * ny_);
  return true;
}

ObstacleDistance3 DistanceTransform3::query(const Vec3 & x) const noexcept
{
  ObstacleDistance3 result;
  const std::size_t cells =
    static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) * static_cast<std::size_t>(nz_);
  if (!has_sites_ || cells == 0 || feature_.size() < cells || !isFinite(x)) {
    return result;
  }
  const Vec3 s = (x - origin_) / resolution_;
  if (!(s.x() >= 0.0 && s.y() >= 0.0 && s.z() >= 0.0 && s.x() < nx_ && s.y() < ny_ &&
        s.z() < nz_)) {
    return result;  // outside the grid: no voxel site
  }
  const auto site_vector = [this, &x](int ix, int iy, int iz) -> Vec3 {
    const std::size_t i = static_cast<std::size_t>(ix) +
                          static_cast<std::size_t>(nx_) *
                            (static_cast<std::size_t>(iy) +
                             static_cast<std::size_t>(ny_) * static_cast<std::size_t>(iz));
    const int index = feature_[i];
    const int fx = index % nx_;
    const int fy = (index / nx_) % ny_;
    const int fz = index / (nx_ * ny_);
    return x - (origin_ + resolution_ * Vec3(fx + 0.5, fy + 0.5, fz + 0.5));
  };
  // The eight voxel centres around x, indices clamped, trilinear weights from the unclamped
  // fractions.
  const Vec3 u = s - Vec3::Constant(0.5);
  const double i0 = std::floor(u.x());
  const double j0 = std::floor(u.y());
  const double k0 = std::floor(u.z());
  const double tx = u.x() - i0;
  const double ty = u.y() - j0;
  const double tz = u.z() - k0;
  const int ix[2] = {
    clampIndex(static_cast<int>(i0), nx_), clampIndex(static_cast<int>(i0) + 1, nx_)};
  const int iy[2] = {
    clampIndex(static_cast<int>(j0), ny_), clampIndex(static_cast<int>(j0) + 1, ny_)};
  const int iz[2] = {
    clampIndex(static_cast<int>(k0), nz_), clampIndex(static_cast<int>(k0) + 1, nz_)};
  const double wx[2] = {1.0 - tx, tx};
  const double wy[2] = {1.0 - ty, ty};
  const double wz[2] = {1.0 - tz, tz};
  Vec3 v[8];
  double w[8] = {};
  for (int c = 0; c < 8; ++c) {
    const int a = c & 1;
    const int b = (c >> 1) & 1;
    const int d = (c >> 2) & 1;
    v[c] = site_vector(ix[a], iy[b], iz[d]);
    w[c] = wx[a] * wy[b] * wz[d];
  }
  bool blend = true;
  for (int a = 0; a < 8 && blend; ++a) {
    if (v[a].isZero(0.0)) {
      blend = false;
    }
    for (int b = a + 1; b < 8 && blend; ++b) {
      if (v[a].dot(v[b]) < 0.0) {
        blend = false;
      }
    }
  }
  Vec3 vec = Vec3::Zero();
  if (blend) {
    for (int c = 0; c < 8; ++c) {
      vec += w[c] * v[c];
    }
  } else {
    vec = site_vector(
      clampIndex(static_cast<int>(std::floor(s.x())), nx_),
      clampIndex(static_cast<int>(std::floor(s.y())), ny_),
      clampIndex(static_cast<int>(std::floor(s.z())), nz_));
  }
  result.D = vec.norm();
  result.n = result.D > 0.0 ? Vec3(vec / result.D) : Vec3(Vec3::Zero());
  result.x_o = x - vec;
  result.valid = true;
  return result;
}

}  // namespace vectorfield_core::d3
