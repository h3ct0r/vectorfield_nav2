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

// O3D-9 (math_spec §14.6): after 10 warm-up cycles, 1000 Tracker3::step calls plus the plant
// perform zero allocations and deallocations, on an open built-in helix (terminal field), a closed
// moving expression curve (feedforward, eta-box), a closed 3D polyline, and a circle and the
// helix in a moving curve frame (math_spec §14.7, including the goal hold). The global operator
// new/delete are replaced by counting versions (skipped under sanitizers by the CMake glob).

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "vectorfield_core/d3.hpp"

namespace
{

std::atomic<bool> g_counting{false};
std::atomic<long> g_allocations{0};
std::atomic<long> g_deallocations{0};

void * countedAllocate(std::size_t size) noexcept
{
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  return std::malloc(size == 0 ? 1 : size);
}

void * countedAllocateAligned(std::size_t size, std::size_t alignment) noexcept
{
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  void * p = nullptr;
  const std::size_t a = std::max(alignment, sizeof(void *));
  if (posix_memalign(&p, a, size == 0 ? 1 : size) != 0) {
    return nullptr;
  }
  return p;
}

void countedFree(void * p) noexcept
{
  if (p != nullptr && g_counting.load(std::memory_order_relaxed)) {
    g_deallocations.fetch_add(1, std::memory_order_relaxed);
  }
  std::free(p);
}

}  // namespace

void * operator new(std::size_t size)
{
  void * p = countedAllocate(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void * operator new[](std::size_t size)
{
  void * p = countedAllocate(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  return countedAllocate(size);
}

void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  return countedAllocate(size);
}

void * operator new(std::size_t size, std::align_val_t alignment)
{
  void * p = countedAllocateAligned(size, static_cast<std::size_t>(alignment));
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void * operator new[](std::size_t size, std::align_val_t alignment)
{
  void * p = countedAllocateAligned(size, static_cast<std::size_t>(alignment));
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void * operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return countedAllocateAligned(size, static_cast<std::size_t>(alignment));
}

void * operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return countedAllocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(void * p) noexcept { countedFree(p); }
void operator delete[](void * p) noexcept { countedFree(p); }
void operator delete(void * p, std::size_t) noexcept { countedFree(p); }
void operator delete[](void * p, std::size_t) noexcept { countedFree(p); }
void operator delete(void * p, std::align_val_t) noexcept { countedFree(p); }
void operator delete[](void * p, std::align_val_t) noexcept { countedFree(p); }
void operator delete(void * p, std::size_t, std::align_val_t) noexcept { countedFree(p); }
void operator delete[](void * p, std::size_t, std::align_val_t) noexcept { countedFree(p); }
void operator delete(void * p, const std::nothrow_t &) noexcept { countedFree(p); }
void operator delete[](void * p, const std::nothrow_t &) noexcept { countedFree(p); }
void operator delete(void * p, std::align_val_t, const std::nothrow_t &) noexcept
{
  countedFree(p);
}
void operator delete[](void * p, std::align_val_t, const std::nothrow_t &) noexcept
{
  countedFree(p);
}

namespace
{

namespace d3 = vectorfield_core::d3;
using d3::Vec3;

void expectNoAllocation(
  const d3::Curve3 & curve, const Vec3 & start, const d3::CurveFrame3 & frame = {},
  const d3::ObstacleSet3 * obstacles = nullptr)
{
  d3::Tracker3Config config;
  if (obstacles != nullptr) {
    config.obstacles.obstacle_mode = vectorfield_core::ObstacleMode::kCbf;
  }
  const d3::Tracker3 tracker{config};
  d3::Tracker3State state = d3::Tracker3::initialState();
  d3::sim::PlantState3 plant;
  plant.pose.p = start;
  d3::sim::PlantParams3 params;
  params.velocity_time_constant = 0.1;
  d3::Tracker3Input input;
  input.curve = &curve;
  input.obstacles = obstacles;
  input.dt = 0.05;
  const auto cycle = [&](int k) {
    input.pose = plant.pose;
    input.measured = plant.twist;
    input.t = 0.05 * k;
    input.frame = frame;
    input.frame.T_eval_curve = d3::extrapolate(frame, input.t);
    const d3::Tracker3Output out = tracker.step(input, state, &state);
    plant = d3::sim::integrate(plant, out.cmd, input.dt, params);
  };
  for (int k = 0; k < 10; ++k) {
    cycle(k);
  }
  g_allocations.store(0);
  g_deallocations.store(0);
  g_counting.store(true);
  for (int k = 10; k < 1010; ++k) {
    cycle(k);
  }
  g_counting.store(false);
  EXPECT_EQ(g_allocations.load(), 0);
  EXPECT_EQ(g_deallocations.load(), 0);
  EXPECT_TRUE(d3::isFinite(plant.pose));
}

d3::CurveSpec3 builtin(const std::string & name)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kBuiltin;
  spec.builtin_shape = name;
  return spec;
}

}  // namespace

TEST(D3NoAlloc, OpenHelix)
{
  const auto curve = d3::makeCurve3(builtin("helix"), {});
  expectNoAllocation(*curve, Vec3(1.3, 0.2, -0.3));
}

TEST(D3NoAlloc, MovingExpression)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kExpression;
  spec.x_expression = "2*cos(s) + 0.05*t";
  spec.y_expression = "sin(s)";
  spec.z_expression = "0.3*sin(2*s) + 0.02*t";
  spec.s_max = 6.283185307179586;
  spec.closed = true;
  const auto curve = d3::makeCurve3(spec, {});
  expectNoAllocation(*curve, Vec3(2.4, 0.0, 0.2));
}

TEST(D3NoAlloc, ClosedPolyline)
{
  const d3::PolylineCurve3 curve(
    {Vec3(0, 0, 0), Vec3(3, 0, 0.5), Vec3(3, 3, 1.0), Vec3(0, 3, 0.5)}, true);
  expectNoAllocation(curve, Vec3(0.5, -0.4, 0.3));
}

// A moving curve frame (math_spec §14.7): a closed circle in a rotating, translating frame and
// the open helix in a rotating frame (transported terminal field, then the goal hold).
TEST(D3NoAlloc, MovingFrame)
{
  d3::CurveFrame3 frame;
  frame.moving = true;
  frame.twist.v = Vec3(0.05, 0.02, 0.0);
  frame.twist.w = Vec3(0.0, 0.05, 0.15);
  const auto circle = d3::makeCurve3(builtin("circle"), {});
  expectNoAllocation(*circle, Vec3(1.3, 0.2, -0.3), frame);
  frame.twist.v = Vec3::Zero();
  const auto helix = d3::makeCurve3(builtin("helix"), {});
  expectNoAllocation(*helix, Vec3(1.3, 0.2, -0.3), frame);
}

// The CBF filter (math_spec §14.8): primitives and a voxel grid on the helix (filter active,
// several sites), and a sphere centred on a line (the hold, kBlocked).
TEST(D3NoAlloc, CbfObstacles)
{
  d3::ObstacleSet3 obstacles;
  obstacles.add(d3::Primitive3::sphere(Vec3(1.35, 0.0, 0.5), 0.3));
  obstacles.add(d3::Primitive3::plane(Vec3(0, 0, -0.1), Vec3::UnitZ()));
  obstacles.add(d3::Primitive3::box(Vec3(-1.4, 0, 0.5), Vec3(0.2, 0.2, 2.0)));
  obstacles.add(d3::Primitive3::cylinder(Vec3(0, -1.4, 0), 0.15));
  const int n = 32;
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(n * n * n), 0);
  for (int iz = 0; iz < n; ++iz) {
    for (int iy = 12; iy < 20; ++iy) {
      mask[static_cast<std::size_t>(2 + n * (iy + n * iz))] = 1;  // a voxel post near (0, 1.2)
    }
  }
  d3::VoxelGrid3 grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, n, 0.1, Vec3(-0.2, 0.0, -0.5)));
  obstacles.setVoxels(grid);
  const auto helix = d3::makeCurve3(builtin("helix"), {});
  expectNoAllocation(*helix, Vec3(1.0, 0.0, 0.0), {}, &obstacles);
  d3::ObstacleSet3 centred;
  centred.add(d3::Primitive3::sphere(Vec3(2, 0, 1), 0.5));
  const d3::PolylineCurve3 line({Vec3(-1, 0, 1), Vec3(5, 0, 1)}, false);
  expectNoAllocation(line, Vec3(0.5, 0, 1), {}, &centred);
}

// Rebuilding the voxel EDT of a same-size grid does not allocate.
TEST(D3NoAlloc, VoxelEdtRebuild)
{
  const int n = 24;
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(n * n * n), 0);
  mask[100] = 1;
  d3::VoxelGrid3 grid;
  ASSERT_TRUE(grid.setFromMask(mask, n, n, n, 0.1, Vec3::Zero()));
  d3::DistanceTransform3 edt;
  edt.update(grid);
  g_allocations.store(0);
  g_counting.store(true);
  for (int k = 0; k < 20; ++k) {
    mask[static_cast<std::size_t>(37 * k + 5)] = 1;
    grid.setFromMask(mask, n, n, n, 0.1, Vec3::Zero());
    EXPECT_TRUE(edt.update(grid));
    edt.query(Vec3(1.2, 1.1, 0.7));
  }
  g_counting.store(false);
  EXPECT_EQ(g_allocations.load(), 0);
}
