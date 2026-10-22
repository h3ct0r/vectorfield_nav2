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
/// @brief 3D core (experimental): obstacles and the 3D CBF safety filter (math_spec §14.8
/// [Design, 0.3.0]).
///
/// Obstacles live in the evaluation frame and are static. They are analytic primitives (spheres,
/// boxes, vertical cylinders, planes) with an exact signed distance and outward normal, and an
/// optional voxel grid with an exact 3D Euclidean distance and feature transform. The robot body
/// is a sphere of radius `body_radius`. The filter is the planar CBF of math_spec §6.7 in
/// \f$\mathbb R^3\f$ with up to kCbf3MaxSites constraints.

#ifndef VECTORFIELD_CORE__D3__OBSTACLES_HPP_
#define VECTORFIELD_CORE__D3__OBSTACLES_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/span.hpp"

namespace vectorfield_core::d3
{

/// @brief Kind of an analytic obstacle.
enum class PrimitiveType3 : std::uint8_t
{
  /// A ball: `center`, `radius`.
  kSphere = 0,
  /// An oriented box: `center`, `half_extents` along the columns of `rotation`.
  kBox = 1,
  /// A vertical cylinder (pole): `center`, `radius`, `half_height` along +z (inf: unbounded).
  kCylinder = 2,
  /// A solid half-space: the plane through `center` with unit `normal` pointing into free space.
  kPlane = 3,
};

/// @brief An analytic obstacle (a convex solid). Fields a kind does not use are ignored.
struct Primitive3
{
  /// Kind.
  PrimitiveType3 type{PrimitiveType3::kSphere};
  /// Centre (sphere, box, cylinder) or a point on the plane (m).
  Vec3 center{Vec3::Zero()};
  /// Radius of a sphere or cylinder (m, > 0).
  double radius{0.0};
  /// Half-extents of a box along its axes (m, > 0).
  Vec3 half_extents{Vec3::Zero()};
  /// Box orientation: its columns are the box axes in the evaluation frame (identity: an
  /// axis-aligned box).
  Mat3 rotation{Mat3::Identity()};
  /// Half-height of a cylinder along z about `center` (m, > 0; +inf: an infinite pole).
  double half_height{kInf};
  /// Plane: unit normal pointing into free space; the half-space behind the plane is solid.
  Vec3 normal{Vec3::UnitZ()};

  /// @brief A sphere.
  static Primitive3 sphere(const Vec3 & center, double radius) noexcept;
  /// @brief A box; `rotation` = identity is axis-aligned (rotationZYX() for yaw, pitch, roll).
  static Primitive3 box(
    const Vec3 & center, const Vec3 & half_extents,
    const Mat3 & rotation = Mat3::Identity()) noexcept;
  /// @brief A vertical cylinder; the default half-height makes an infinite pole.
  static Primitive3 cylinder(
    const Vec3 & center, double radius, double half_height = kInf) noexcept;
  /// @brief The solid half-space behind the plane through `point`; `normal` (normalized here)
  /// points into free space.
  static Primitive3 plane(const Vec3 & point, const Vec3 & normal) noexcept;
};

/// @brief Throws ConfigError unless `primitive` is finite and well formed (positive sizes, a
/// rotation matrix to 1e-9, a nonzero plane normal).
void validatePrimitive3(const Primitive3 & primitive);

/// @brief A signed distance with its gradient: the distance to one obstacle.
struct ObstacleDistance3
{
  /// Signed distance \f$D\f$ (m): positive outside, negative inside; \f$+\infty\f$ when invalid.
  double D{kInf};
  /// Unit outward normal \f$n = \nabla D\f$ (zero when invalid or undefined).
  Vec3 n{Vec3::Zero()};
  /// Nearest surface point (primitives) or site centre (voxels), \f$x - D n\f$.
  Vec3 x_o{Vec3::Zero()};
  /// False without an obstacle (or outside the voxel grid).
  bool valid{false};
};

/// @brief Exact signed distance and outward normal of a primitive at x (math_spec §14.8).
///
/// On the medial set (the centre of a sphere, the axis of a cylinder, equidistant box faces) the
/// normal is one of the valid ones, chosen deterministically. Allocation-free.
ObstacleDistance3 signedDistance(const Primitive3 & primitive, const Vec3 & x) noexcept;

/// @brief A voxel occupancy snapshot: uint8 per voxel, nonzero = occupied (an obstacle site).
///
/// Voxel (ix, iy, iz) has index `ix + nx (iy + ny iz)` (x fastest) and spans
/// `origin + res [ix, ix + 1) × [iy, iy + 1) × [iz, iz + 1)`.
class VoxelGrid3
{
public:
  /// @brief An empty grid (0 voxels).
  VoxelGrid3() noexcept;
  /// @brief Reserves storage, so setFromMask() of up to that many voxels never allocates.
  void reserve(int nx, int ny, int nz);
  /// @brief Copies a mask (nonzero = occupied), reusing capacity (it allocates only beyond the
  /// reserved size). version() changes iff the geometry or any voxel's flag changed.
  /// @param mask Voxels, x fastest, `mask.size() == nx * ny * nz`.
  /// @param nx Voxels along x (in [1, 2^20]).
  /// @param ny Voxels along y (in [1, 2^20]).
  /// @param nz Voxels along z (in [1, 2^20]); `nx ny nz` must fit an int.
  /// @param resolution Voxel edge (m, > 0).
  /// @param origin Position of the corner of voxel (0, 0, 0) (m).
  /// @return False (grid unchanged) on inconsistent or non-finite arguments.
  bool setFromMask(
    Span<const std::uint8_t> mask, int nx, int ny, int nz, double resolution,
    const Vec3 & origin) noexcept;
  /// @brief Voxels along x.
  int nx() const noexcept { return nx_; }
  /// @brief Voxels along y.
  int ny() const noexcept { return ny_; }
  /// @brief Voxels along z.
  int nz() const noexcept { return nz_; }
  /// @brief Voxel edge (m).
  double resolution() const noexcept { return resolution_; }
  /// @brief Corner of voxel (0, 0, 0).
  const Vec3 & origin() const noexcept { return origin_; }
  /// @brief The mask, x fastest, 1 = occupied.
  Span<const std::uint8_t> mask() const noexcept;
  /// @brief Total voxels.
  std::size_t size() const noexcept { return mask_.size(); }
  /// @brief True iff the voxel is inside the grid.
  bool inBounds(int ix, int iy, int iz) const noexcept;
  /// @brief True iff the voxel is inside the grid and occupied.
  bool isSite(int ix, int iy, int iz) const noexcept;
  /// @brief Centre of a voxel; it need not be in bounds.
  Vec3 cellCenter(int ix, int iy, int iz) const noexcept;
  /// @brief A process-wide unique stamp of the content (0 for an empty grid).
  std::uint64_t version() const noexcept { return version_; }

private:
  std::vector<std::uint8_t> mask_;
  int nx_{0};
  int ny_{0};
  int nz_{0};
  double resolution_{0.05};
  Vec3 origin_{Vec3::Zero()};
  std::uint64_t version_{0};
};

/// @brief Exact squared Euclidean distance and feature transform of a voxel grid
/// (Felzenszwalb–Huttenlocher, separable: one 1-D lower envelope per line along z, y, then x;
/// math_spec §14.8).
///
/// Squared distances between voxel centres are exact integers (oracle O3D-15). Owns its result
/// and work arrays; they grow on demand and never shrink, so a rebuild of a same-size grid never
/// allocates. Const methods are thread-safe.
class DistanceTransform3
{
public:
  /// @brief An empty transform.
  DistanceTransform3() noexcept;
  /// @brief Reserves storage for `nx * ny * nz` voxels.
  void reserve(int nx, int ny, int nz);
  /// @brief Rebuilds iff `grid.version()` differs from the last build.
  /// @return True iff it rebuilt.
  bool update(const VoxelGrid3 & grid) noexcept;
  /// @brief Rebuilds unconditionally.
  void build(const VoxelGrid3 & grid) noexcept;
  /// @brief True iff the last build had at least one occupied voxel.
  bool hasSites() const noexcept { return has_sites_; }
  /// @brief Voxels along x of the last build.
  int nx() const noexcept { return nx_; }
  /// @brief Voxels along y of the last build.
  int ny() const noexcept { return ny_; }
  /// @brief Voxels along z of the last build.
  int nz() const noexcept { return nz_; }
  /// @brief Voxel edge of the last build (m).
  double resolution() const noexcept { return resolution_; }
  /// @brief Origin of the last build.
  const Vec3 & origin() const noexcept { return origin_; }
  /// @brief `grid.version()` of the last build.
  std::uint64_t builtVersion() const noexcept { return built_version_; }
  /// @brief Squared distance between the centre of (ix, iy, iz) and the nearest occupied
  /// voxel centre, in voxel units² (an exact integer); +inf without sites. Precondition: in
  /// bounds.
  double squaredDistanceCells(int ix, int iy, int iz) const noexcept;
  /// @brief The same distance in metres.
  double distance(int ix, int iy, int iz) const noexcept;
  /// @brief Feature: the nearest occupied voxel of (ix, iy, iz); false (and -1s) without sites.
  /// Precondition: in bounds.
  bool feature(int ix, int iy, int iz, int * fx, int * fy, int * fz) const noexcept;
  /// @brief Continuous query at x: the trilinear blend of the vectors \f$x - f(c)\f$ of the
  /// eight voxel centres around x when they pairwise agree (\f$v_i^\top v_j \ge 0\f$, none
  /// zero), else the feature of the voxel containing x (the 3D form of math_spec §6.1).
  /// Invalid outside the grid or without sites. Allocation-free.
  ObstacleDistance3 query(const Vec3 & x) const noexcept;

private:
  std::vector<double> squared_;
  std::vector<int> feature_;
  std::vector<double> line_g_;
  std::vector<double> line_apex_;
  std::vector<double> line_out_;
  std::vector<int> line_feature_;
  std::vector<int> line_vertices_;
  std::vector<int> line_feature_out_;
  std::vector<int> plane_;
  int nx_{0};
  int ny_{0};
  int nz_{0};
  double resolution_{0.05};
  Vec3 origin_{Vec3::Zero()};
  std::uint64_t built_version_{0};
  bool has_sites_{false};
};

/// @brief At most this many constraints in the 3D filter (three bind independently in
/// \f$\mathbb R^3\f$; the fourth covers a corner such as two walls, a floor and a pole).
inline constexpr int kCbf3MaxSites = 4;
/// @brief Half-width (voxels) of the window whose features give the extra voxel sites (5³).
inline constexpr int kCbf3WindowCells = 2;

/// @brief One obstacle site of the 3D filter.
struct ObstacleSite3
{
  /// Nearest surface point or voxel site centre.
  Vec3 x_o{Vec3::Zero()};
  /// Unit outward normal.
  Vec3 n{Vec3::Zero()};
  /// Signed distance \f$D\f$ (m).
  double D{kInf};
  /// Index of the primitive, or -1 for a voxel site.
  int source{-1};
};

/// @brief The static obstacles of a scene, in the evaluation frame: primitives and an optional
/// voxel grid with its distance transform (rebuilt only when the grid changes).
class ObstacleSet3
{
public:
  /// @brief Reserves storage for primitives.
  void reservePrimitives(std::size_t count);
  /// @brief Adds a primitive (a plane's normal is normalized).
  /// @throws ConfigError (validatePrimitive3()).
  void add(const Primitive3 & primitive);
  /// @brief Removes every primitive.
  void clearPrimitives() noexcept { primitives_.clear(); }
  /// @brief The primitives.
  const std::vector<Primitive3> & primitives() const noexcept { return primitives_; }
  /// @brief Copies a voxel grid and rebuilds its distance transform iff it changed.
  /// @return True iff the transform was rebuilt.
  bool setVoxels(const VoxelGrid3 & grid);
  /// @brief Removes the voxel grid.
  void clearVoxels() noexcept;
  /// @brief True iff a voxel grid with at least one occupied voxel is set.
  bool hasVoxels() const noexcept { return edt_.hasSites(); }
  /// @brief The voxel grid.
  const VoxelGrid3 & voxels() const noexcept { return grid_; }
  /// @brief Its distance transform.
  const DistanceTransform3 & voxelDistance() const noexcept { return edt_; }
  /// @brief No primitive and no occupied voxel.
  bool empty() const noexcept { return primitives_.empty() && !hasVoxels(); }
  /// @brief The nearest obstacle at x (least signed distance over the primitives and the voxel
  /// query); invalid when empty.
  ObstacleDistance3 nearest(const Vec3 & x) const noexcept;
  /// @brief The sites the filter constrains at x (math_spec §14.8): one per primitive and the
  /// voxel sites (the continuous query, then the nearest window features whose normals are more
  /// than 45° from the voxel normals taken), the `count` nearest by D, nearest first. Voxel
  /// sites at distance 0 are skipped. Allocation-free.
  /// @param x Point (evaluation frame).
  /// @param count Sites wanted, clamped to [0, kCbf3MaxSites].
  /// @param out Receives up to `count` sites.
  /// @return Sites found.
  int sites(const Vec3 & x, int count, ObstacleSite3 * out) const noexcept;

private:
  std::vector<Primitive3> primitives_;
  VoxelGrid3 grid_;
  DistanceTransform3 edt_;
};

/// @brief One constraint \f$n^\top F' \ge -\gamma_b h\f$.
struct Cbf3Constraint
{
  /// Unit normal (away from the obstacle).
  Vec3 n{Vec3::Zero()};
  /// Barrier \f$h = D - \delta_{\mathrm{safe}}\f$ (m).
  double h{kInf};
};

/// @brief The filtered field.
struct Cbf3Solution
{
  /// \f$F^\star\f$.
  Vec3 F{Vec3::Zero()};
  /// \f$F^\star \ne F\f$.
  bool active{false};
  /// Conflicting push-outs were dropped (solved again with \f$\min(-\gamma_b h_j, 0)\f$).
  bool relaxed{false};
  /// Bit j: constraint j binds at \f$F^\star\f$.
  std::uint8_t binds{0};
};

/// @brief The 3D CBF filter (math_spec §14.8): \f$F^\star = \arg\min \lVert F' - F\rVert^2\f$ s.t.
/// \f$n_j^\top F' \ge -\gamma_b h_j\f$, solved exactly by enumerating the active sets of at most
/// three linearly independent constraints (the least-cost feasible candidate is the optimum).
/// Conflicting push-outs are solved again without them (\f$F' = 0\f$ is then feasible). Finally
/// \f$\lVert F^\star\rVert \le \lVert F\rVert\f$. No allocation.
/// @param F Field before the filter.
/// @param constraints Constraints (at most kCbf3MaxSites are used).
/// @param count Number of constraints.
/// @param gain \f$\gamma_b\f$ (> 0).
Cbf3Solution solveCbf3(
  const Vec3 & F, const Cbf3Constraint * constraints, int count, double gain) noexcept;

/// @brief Obstacle handling of Tracker3 (math_spec §14.8).
struct Obstacles3Config
{
  /// kNone (obstacles ignored) or kCbf; kContour is planar only (ConfigError).
  ObstacleMode obstacle_mode{ObstacleMode::kNone};
  /// Radius \f$r_b\f$ of the spherical body (m, ≥ 0).
  double body_radius{0.25};
  /// `cbf_margin` \f$m\f$ (m, ≥ 0): \f$\delta_{\mathrm{safe}} = r_b + m\f$.
  double cbf_margin{0.05};
  /// `cbf_gain` \f$\gamma_b\f$ (1/s, > 0, \f$\gamma_b / f < 1\f$).
  double cbf_gain{2.0};
  /// `cbf_sites` \f$k \in [1, 4]\f$.
  int cbf_sites{kCbf3MaxSites};
};

/// @brief The filter's view of one Tracker3 cycle (Tracker3Output::cbf).
struct Cbf3Sample
{
  /// Mode kCbf with a non-empty ObstacleSet3.
  bool on{false};
  /// The filter changed the target.
  bool active{false};
  /// Conflicting push-outs were dropped.
  bool relaxed{false};
  /// The hold condition (\f$\lVert F^\star\rVert < 0.05\lVert F\rVert\f$, released at 0.10).
  bool hold{false};
  /// Constrained sites.
  int count{0};
  /// \f$\delta_{\mathrm{safe}} = r_b + m\f$ (m).
  double delta{0.0};
  /// \f$\min_j h_j\f$ (the nearest obstacle's barrier; +inf without sites).
  double h{kInf};
  /// \f$\lVert F^\star\rVert / \lVert F\rVert\f$ (1 when inactive).
  double speed_ratio{1.0};
  /// Target before the filter.
  Vec3 F_in{Vec3::Zero()};
  /// \f$F^\star\f$.
  Vec3 F{Vec3::Zero()};
  /// The sites, nearest first.
  ObstacleSite3 sites[kCbf3MaxSites];
  /// Bit j: site j binds.
  std::uint8_t binds{0};
};

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__OBSTACLES_HPP_
