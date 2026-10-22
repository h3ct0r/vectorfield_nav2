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

#include "vectorfield_follower_3d/voxelizer.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"

namespace vectorfield_follower_3d
{

namespace d3 = vectorfield_core::d3;

VoxelWindow windowAround(const d3::Vec3 & centre, double resolution, int size)
{
  VoxelWindow window;
  window.resolution = resolution;
  window.size = size;
  const double half = 0.5 * static_cast<double>(size) * resolution;
  for (int i = 0; i < 3; ++i) {
    window.origin[i] = std::round((centre[i] - half) / resolution) * resolution;
  }
  return window;
}

std::size_t voxelize(
  const std::vector<d3::Vec3> & points, const VoxelWindow & window,
  std::vector<std::uint8_t> * mask)
{
  const auto n = static_cast<std::size_t>(window.size);
  mask->assign(n * n * n, 0U);
  std::size_t occupied = 0;
  for (const d3::Vec3 & p : points) {
    if (!d3::isFinite(p)) {
      continue;
    }
    const d3::Vec3 cell = (p - window.origin) / window.resolution;
    const double fx = std::floor(cell.x());
    const double fy = std::floor(cell.y());
    const double fz = std::floor(cell.z());
    const auto limit = static_cast<double>(window.size);
    if (fx < 0.0 || fy < 0.0 || fz < 0.0 || fx >= limit || fy >= limit || fz >= limit) {
      continue;
    }
    const std::size_t index =
      (static_cast<std::size_t>(fz) * n + static_cast<std::size_t>(fy)) * n +
      static_cast<std::size_t>(fx);
    if ((*mask)[index] == 0U) {
      (*mask)[index] = 1U;
      ++occupied;
    }
  }
  return occupied;
}

namespace
{

/// Reads one field of a point; NaN if the field is absent.
double readField(
  const std::uint8_t * point, const sensor_msgs::msg::PointField * field, bool big_endian)
{
  if (field == nullptr) {
    return std::nan("");
  }
  const std::uint8_t * data = point + field->offset;
  std::uint8_t bytes[8];
  const std::size_t size = field->datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8U : 4U;
  std::memcpy(bytes, data, size);
  // Every supported ROS platform is little endian; swap a big-endian cloud.
  if (big_endian) {
    for (std::size_t i = 0; i < size / 2; ++i) {
      const std::uint8_t tmp = bytes[i];
      bytes[i] = bytes[size - 1 - i];
      bytes[size - 1 - i] = tmp;
    }
  }
  if (size == 8U) {
    double value = 0.0;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
  }
  float value = 0.0F;
  std::memcpy(&value, bytes, sizeof(value));
  return static_cast<double>(value);
}

}  // namespace

bool cloudPoints(
  const sensor_msgs::msg::PointCloud2 & cloud, const d3::Isometry3 & transform,
  std::vector<d3::Vec3> * out)
{
  out->clear();
  const sensor_msgs::msg::PointField * fields[3] = {nullptr, nullptr, nullptr};
  const char * names[3] = {"x", "y", "z"};
  for (const auto & field : cloud.fields) {
    for (int i = 0; i < 3; ++i) {
      if (field.name == names[i] && field.count >= 1U) {
        const bool f32 = field.datatype == sensor_msgs::msg::PointField::FLOAT32;
        const bool f64 = field.datatype == sensor_msgs::msg::PointField::FLOAT64;
        if (f32 || f64) {
          fields[i] = &field;
        }
      }
    }
  }
  if (fields[0] == nullptr || fields[1] == nullptr || fields[2] == nullptr) {
    return false;
  }
  for (const auto * field : fields) {
    const std::size_t size = field->datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8U : 4U;
    if (field->offset + size > cloud.point_step) {
      return false;
    }
  }
  const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
  if (
    cloud.row_step < static_cast<std::size_t>(cloud.width) * cloud.point_step ||
    cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height) {
    return false;
  }
  out->reserve(count);
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::uint8_t * base = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step;
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::uint8_t * point = base + static_cast<std::size_t>(col) * cloud.point_step;
      const d3::Vec3 p(
        readField(point, fields[0], cloud.is_bigendian),
        readField(point, fields[1], cloud.is_bigendian),
        readField(point, fields[2], cloud.is_bigendian));
      if (d3::isFinite(p)) {
        out->push_back(transform * p);
      }
    }
  }
  return true;
}

}  // namespace vectorfield_follower_3d
