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
/// @brief The voxel source of the 3D follower: a point cloud → a cube of voxels centred on the
/// robot (the core's VoxelGrid3 mask). Pure.

#ifndef VECTORFIELD_FOLLOWER_3D__VOXELIZER_HPP_
#define VECTORFIELD_FOLLOWER_3D__VOXELIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vectorfield_core/d3/frame.hpp"
#include "vectorfield_core/d3/types.hpp"

namespace vectorfield_follower_3d
{

/// @brief A cube of `size`³ voxels of edge `resolution`.
struct VoxelWindow
{
  /// Corner of voxel (0, 0, 0) (m), a multiple of `resolution` on each axis.
  vectorfield_core::d3::Vec3 origin{vectorfield_core::d3::Vec3::Zero()};
  /// Voxel edge (m).
  double resolution{0.1};
  /// Voxels per axis.
  int size{48};
};

/// @brief The window of `size`³ voxels whose centre is nearest `centre`, with its corner snapped
/// to the `resolution` lattice (so a robot that moves less than a voxel keeps the same window).
VoxelWindow windowAround(const vectorfield_core::d3::Vec3 & centre, double resolution, int size);

/// @brief Marks the voxels of `window` that contain a point (`mask[(iz * size + iy) * size + ix]`,
/// the layout of VoxelGrid3::setFromMask); points outside the window and non-finite points are
/// dropped.
/// @param points Points in the window's frame.
/// @param window The window.
/// @param mask Receives the mask (resized to size³ and cleared first).
/// @return The number of occupied voxels.
std::size_t voxelize(
  const std::vector<vectorfield_core::d3::Vec3> & points, const VoxelWindow & window,
  std::vector<std::uint8_t> * mask);

/// @brief The x, y, z fields (FLOAT32 or FLOAT64) of a cloud, mapped by `transform` (the cloud
/// frame into the window's frame). Invalid (NaN) points are skipped.
/// @return False if the cloud has no usable x, y, z fields.
bool cloudPoints(
  const sensor_msgs::msg::PointCloud2 & cloud, const vectorfield_core::d3::Isometry3 & transform,
  std::vector<vectorfield_core::d3::Vec3> * out);

}  // namespace vectorfield_follower_3d

#endif  // VECTORFIELD_FOLLOWER_3D__VOXELIZER_HPP_
