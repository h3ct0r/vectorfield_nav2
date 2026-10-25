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

// O55 SteadyStateNoAlloc (math_spec §12.8): a separate executable that
// replaces the global operator new/delete with counting versions (the CMake glob skips files named
// test_noalloc*.cpp under sanitizers). After StepScratch::reserve() and 10 warm-up cycles, 1000
// Tracker::step calls (plus sim::integrate) on a fixed grid, with the contour active and the
// rollout on, perform zero allocations and zero deallocations: FL on a closed polyline, HT
// (FieldProbe path) on a built-in rounded rectangle, holonomic on a built-in ellipse. The closed
// loop is written out here because ClosedLoopSimulator records every step and may allocate.
// No gtest assertion runs inside the counted window.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <optional>
#include <vector>

#include "test_support.hpp"

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

using namespace vf_test;  // NOLINT(build/namespaces)

struct SteadyStateResult
{
  long allocations{0};
  long deallocations{0};
  int episode_starts{0};
  int active_cycles{0};
  int rollout_cycles{0};
  int probe_cycles{0};
  int cbf_active_cycles{0};
};

// Closed loop on Tracker::step + sim::integrate; counts allocations over the measured cycles.
SteadyStateResult countSteadyState(
  const vf::Tracker & tracker, const vf::TrackerInput & tmpl, Pose2 pose,
  const ObstacleWorld & world, int warmup = 10, int measured = 1000)
{
  vf::StepScratch scratch;
  scratch.reserve(
    static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height()), 8192, 1024);
  vf::TrackerState state = vf::Tracker::initialState();
  vf::TrackerInput input = tmpl;
  const vf::Kinematics kinematics = tracker.kinematics();
  Twist2 measured_twist;
  double t = 0.0;
  SteadyStateResult r;
  for (int k = 0; k < warmup + measured; ++k) {
    if (k == warmup) {
      g_allocations.store(0);
      g_deallocations.store(0);
      g_counting.store(true);
    }
    input.pose = pose;
    input.measured = measured_twist;
    input.t = t;
    input.dt = kPeriod;
    const bool was_active = state.contour.active;
    const vf::TrackerOutput out = tracker.step(input, state, &state, scratch);
    if (k >= warmup) {
      r.episode_starts += (!was_active && state.contour.active) ? 1 : 0;
      r.active_cycles += state.contour.active ? 1 : 0;
      r.rollout_cycles += out.predicted.size() > 0 ? 1 : 0;
      r.probe_cycles += out.field.psi_d_dot != 0.0 ? 1 : 0;
      r.cbf_active_cycles += out.field.cbf.active ? 1 : 0;
    }
    pose = vf::sim::integrate(kinematics, pose, out.cmd, kPeriod);
    measured_twist = out.cmd;
    t += kPeriod;
  }
  g_counting.store(false);
  r.allocations = g_allocations.load();
  r.deallocations = g_deallocations.load();
  return r;
}

// Stadium loop: straights of 3 m along y = ±1, semicircles of radius 1, counter-clockwise.
std::vector<Vec2> stadium()
{
  std::vector<Vec2> p = linePoints(Vec2(-1.5, -1.0), Vec2(1.5, -1.0), 0.05);
  appendPoints(&p, arcPoints(Vec2(1.5, 0.0), 1.0, -vf::kHalfPi, vf::kHalfPi, 0.05));
  appendPoints(&p, linePoints(Vec2(1.5, 1.0), Vec2(-1.5, 1.0), 0.05));
  appendPoints(&p, arcPoints(Vec2(-1.5, 0.0), 1.0, vf::kHalfPi, 1.5 * vf::kPi, 0.05));
  p.pop_back();  // the closing duplicate of the first vertex
  return p;
}

}  // namespace

// O55 with FL on a closed polyline (Hinted search) and a 0.3 m box on the bottom straight.
TEST(OracleNoAlloc, O55_SteadyStateNoAlloc_PolylineFL)
{
  ObstacleWorld world(Vec2(-4.0, -3.5), 8.0, 7.0);
  world.addBox(Vec2(-0.15, -1.15), Vec2(0.15, -0.85));
  world.commit();
  const auto curve = makePolyline(stadium(), true);
  const vf::Tracker tracker(tcContourConfig());
  vf::TrackerInput input;
  input.curve = curve.get();
  input.obstacles = world.map();
  const SteadyStateResult r = countSteadyState(tracker, input, Pose2{Vec2(-1.2, -1.0), 0.0}, world);
  EXPECT_EQ(r.allocations, 0);
  EXPECT_EQ(r.deallocations, 0);
  EXPECT_GE(r.episode_starts, 1) << "no episode started in the measured window";
  EXPECT_GE(r.active_cycles, 20);
  EXPECT_GE(r.rollout_cycles, 900);
}

// O55 with HT (FieldProbe path) on the built-in rounded rectangle and a box on its bottom side.
TEST(OracleNoAlloc, O55_SteadyStateNoAlloc_BuiltinHeadingTracking)
{
  ObstacleWorld world(Vec2(-4.0, -3.5), 8.0, 7.0);
  world.addBox(Vec2(-0.15, -1.15), Vec2(0.15, -0.85));
  world.commit();
  const auto curve =
    makeBuiltin("rounded_rectangle", {{"width", 4.0}, {"height", 2.0}, {"radius", 0.5}});
  vf::CoreConfig config =
    tcConfig(vf::MotionModel::kDifferential, vf::TrackingLaw::kHeadingTracking);
  config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  const vf::Tracker tracker(config);
  vf::TrackerInput input;
  input.curve = curve.get();
  input.obstacles = world.map();
  const SteadyStateResult r =
    countSteadyState(tracker, input, Pose2{Vec2(2.0, 0.0), vf::kHalfPi}, world);
  EXPECT_EQ(r.allocations, 0);
  EXPECT_EQ(r.deallocations, 0);
  EXPECT_GE(r.episode_starts, 1) << "no episode started in the measured window";
  EXPECT_GE(r.active_cycles, 20);
  EXPECT_GE(r.rollout_cycles, 900);
  EXPECT_GE(r.probe_cycles, 500) << "the ψ̇_d probes were not exercised";
}

// O55 with the holonomic law on a built-in ellipse and a box on it.
TEST(OracleNoAlloc, O55_SteadyStateNoAlloc_BuiltinHolonomic)
{
  ObstacleWorld world(Vec2(-4.0, -3.5), 8.0, 7.0);
  world.addBox(Vec2(-0.15, -1.35), Vec2(0.15, -1.05));
  world.commit();
  const auto curve = makeBuiltin("ellipse", {{"a", 2.0}, {"b", 1.2}});
  vf::CoreConfig config = tcConfig(vf::MotionModel::kOmnidirectional, vf::TrackingLaw::kHolonomic);
  config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  const vf::Tracker tracker(config);
  vf::TrackerInput input;
  input.curve = curve.get();
  input.obstacles = world.map();
  const SteadyStateResult r =
    countSteadyState(tracker, input, Pose2{Vec2(2.0, 0.0), vf::kHalfPi}, world);
  EXPECT_EQ(r.allocations, 0);
  EXPECT_EQ(r.deallocations, 0);
  EXPECT_GE(r.episode_starts, 1) << "no episode started in the measured window";
  EXPECT_GE(r.active_cycles, 20);
  EXPECT_GE(r.rollout_cycles, 900);
}

// O55 with the CBF filter (math_spec §6.7): heading tracking (filtered ψ̇_d probes) and FL on the
// stadium, a box beside the bottom straight that the footprint would graze, three sites.
TEST(OracleNoAlloc, O55_SteadyStateNoAlloc_Cbf)
{
  ObstacleWorld world(Vec2(-4.0, -3.5), 8.0, 7.0);
  world.addBox(Vec2(-0.15, -0.95), Vec2(0.15, -0.75));
  world.addBox(Vec2(0.6, -1.5), Vec2(0.9, -1.3));
  world.commit();
  const auto curve = makePolyline(stadium(), true);
  for (const vf::TrackingLaw law :
       {vf::TrackingLaw::kHeadingTracking, vf::TrackingLaw::kFeedbackLinearization}) {
    SCOPED_TRACE(vf::toString(law));
    vf::CoreConfig config = tcConfig(vf::MotionModel::kDifferential, law);
    config.obstacles.obstacle_mode = vf::ObstacleMode::kCbf;
    const vf::Tracker tracker(config);
    vf::TrackerInput input;
    input.curve = curve.get();
    input.obstacles = world.map();
    const SteadyStateResult r =
      countSteadyState(tracker, input, Pose2{Vec2(-1.2, -1.0), 0.0}, world);
    EXPECT_EQ(r.allocations, 0);
    EXPECT_EQ(r.deallocations, 0);
    EXPECT_GE(r.cbf_active_cycles, 10) << "the filter never bound in the measured window";
    EXPECT_GE(r.rollout_cycles, 900);
  }
}
