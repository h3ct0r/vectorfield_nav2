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

// Steady-state allocation check of the Tracker WITHOUT StepScratch::reserve() (plan §3.1
// "Allocation"; the reserved case is oracle O55 in test_noalloc_tracker.cpp): the Tracker grows
// its scratch buffers to their bounds on the first cycle with a map, so 1000 later steps on a fixed
// grid, with contour episodes starting and ending and the rollout on, allocate nothing, even when
// the first episode (and its side probe) starts after the warm-up. Every global operator new is
// counted, so this file is not built under the sanitizers (the test_noalloc prefix).

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <vector>

#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/tracker.hpp"

namespace
{

std::atomic<long long> g_allocations{0};

void * countedAlloc(std::size_t size) noexcept
{
  g_allocations.fetch_add(1, std::memory_order_relaxed);
  return std::malloc(size == 0 ? 1 : size);
}

void * countedAlignedAlloc(std::size_t size, std::align_val_t alignment) noexcept
{
  g_allocations.fetch_add(1, std::memory_order_relaxed);
  std::size_t align = static_cast<std::size_t>(alignment);
  if (align < sizeof(void *)) {
    align = sizeof(void *);
  }
  void * p = nullptr;
  return posix_memalign(&p, align, size == 0 ? 1 : size) == 0 ? p : nullptr;
}

void * orThrow(void * p)
{
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

}  // namespace

// Replacements of every global allocation function (C++17), all counted.
void * operator new(std::size_t size) { return orThrow(countedAlloc(size)); }
void * operator new[](std::size_t size) { return orThrow(countedAlloc(size)); }
void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  return countedAlloc(size);
}
void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  return countedAlloc(size);
}
void * operator new(std::size_t size, std::align_val_t alignment)
{
  return orThrow(countedAlignedAlloc(size, alignment));
}
void * operator new[](std::size_t size, std::align_val_t alignment)
{
  return orThrow(countedAlignedAlloc(size, alignment));
}
void * operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return countedAlignedAlloc(size, alignment);
}
void * operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return countedAlignedAlloc(size, alignment);
}
void operator delete(void * p) noexcept { std::free(p); }
void operator delete[](void * p) noexcept { std::free(p); }
void operator delete(void * p, std::size_t) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t) noexcept { std::free(p); }
void operator delete(void * p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void * p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete(void * p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void * p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void * p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete(void * p, std::align_val_t, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void * p, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(p);
}

namespace vf = vectorfield_core;
using vf::Pose2;
using vf::Twist2;
using vf::Vec2;

namespace
{

constexpr double kPeriod = 0.05;
constexpr int kWarmUp = 10;
constexpr int kCounted = 1000;

/// A fixed grid of 0.05 m cells covering [-3, 3] x [-3, 3] with discs of sites.
class DiscWorld
{
public:
  explicit DiscWorld(const std::vector<Vec2> & centres, double radius)
  : mask_(static_cast<std::size_t>(kCells) * kCells, 0)
  {
    for (int iy = 0; iy < kCells; ++iy) {
      for (int ix = 0; ix < kCells; ++ix) {
        const Vec2 centre = kOrigin + Vec2((ix + 0.5) * kRes, (iy + 0.5) * kRes);
        for (const Vec2 & disc : centres) {
          if ((centre - disc).norm() <= radius) {
            mask_[static_cast<std::size_t>(iy) * kCells + ix] = 1;
          }
        }
      }
    }
    EXPECT_TRUE(grid_.setFromMask(mask_, kCells, kCells, kRes, kOrigin));
    edt_.update(grid_);
  }

  vf::ObstacleMap map() const { return vf::ObstacleMap{&grid_, &edt_, vf::Isometry2()}; }

private:
  static constexpr int kCells = 120;
  static constexpr double kRes = 0.05;
  inline static const Vec2 kOrigin{-3.0, -3.0};
  std::vector<std::uint8_t> mask_;
  vf::OccupancyGrid grid_;
  vf::EuclideanDistanceTransform edt_;
};

/// Counts of one steady-state run.
struct RunCount
{
  long long allocations{0};
  int contour_cycles{0};
  /// Cycle of the first episode start, -1 if none.
  int first_start{-1};
};

/// Runs kWarmUp + kCounted steps of the plant under the Tracker and counts the allocations of
/// the last kCounted. The robot restarts at `start` whenever it passes `restart_x`.
RunCount countSteadyState(
  const vf::Tracker & tracker, vf::TrackerInput input, const Pose2 & start, double restart_x)
{
  vf::TrackerState state = vf::Tracker::initialState();
  vf::StepScratch scratch;
  Pose2 pose = start;
  Twist2 twist;
  RunCount count;
  long long before = 0;
  for (int k = 0; k < kWarmUp + kCounted; ++k) {
    if (k == kWarmUp) {
      before = g_allocations.load();
    }
    input.pose = pose;
    input.measured = twist;
    input.t = kPeriod * k;
    const bool was_active = state.contour.active;
    const vf::TrackerOutput out = tracker.step(input, state, &state, scratch);
    if (state.contour.active) {
      ++count.contour_cycles;
      if (!was_active && count.first_start < 0) {
        count.first_start = k;
      }
    }
    pose = vf::sim::integrate(tracker.kinematics(), pose, out.cmd, kPeriod);
    twist = out.cmd;
    if (pose.p.x() > restart_x) {
      pose = start;
      twist = Twist2{};
    }
  }
  count.allocations = g_allocations.load() - before;
  return count;
}

vf::CoreConfig contourConfig(vf::TrackingLaw law)
{
  vf::CoreConfig config;
  config.obstacles.obstacle_mode = vf::ObstacleMode::kContour;
  config.motion.tracking_law = law;
  return config;
}

TEST(NoAllocTrackerUnreserved, TheCounterSeesLibraryAllocations)
{
  // A zero count below is meaningful only if allocations made inside libvectorfield_core reach
  // the replaced operator new (it is a shared library by default).
  const long long before = g_allocations.load();
  const vf::Tracker tracker{vf::CoreConfig{}};
  vf::StepScratch scratch;
  scratch.reserve(100, 100, 30);
  EXPECT_GT(g_allocations.load(), before);
}

TEST(NoAllocTrackerUnreserved, PolylineWithEpisodesStartingAfterTheWarmUp)
{
  // An open sinusoidal polyline with a disc beside it: the gate opens, episodes start through
  // the side probe and end again on every pass.
  std::vector<Vec2> points;
  for (int i = 0; i < 90; ++i) {
    const double x = -2.0 + 0.05 * i;
    points.push_back(Vec2(x, 0.2 * std::sin(1.5 * x)));
  }
  vf::PolylineCurve curve;
  ASSERT_EQ(curve.assign(points, false), vf::AssignStatus::kOk);
  const DiscWorld world({Vec2(0.0, 0.05)}, 0.15);
  {
    const vf::TrackingLaw law = vf::TrackingLaw::kFeedbackLinearization;
    const vf::Tracker tracker{contourConfig(law)};
    vf::TrackerInput input;
    input.curve = &curve;
    input.dt = kPeriod;
    vf::GoalInfo goal;
    goal.pose = Pose2{points.back(), 0.0};
    input.goal = goal;
    input.obstacles = world.map();
    const RunCount count = countSteadyState(tracker, input, Pose2{Vec2(-1.8, 0.0), 0.0}, 1.5);
    EXPECT_GT(count.contour_cycles, 0) << vf::toString(law);
    EXPECT_GE(count.first_start, kWarmUp);  // the first side probe runs in the counted window
    EXPECT_EQ(count.allocations, 0) << vf::toString(law);
  }
}

TEST(NoAllocTrackerUnreserved, BuiltinCurveUnderHeadingTracking)
{
  // A closed built-in circle of radius 1.5 m with a disc on it: one episode per lap.
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kBuiltin;
  spec.builtin_shape = "ellipse";
  spec.parameter_names = {"a", "b"};
  spec.parameter_values = {1.5, 1.5};
  const std::shared_ptr<const vf::Curve> curve = vf::makeCurve(spec, vf::CurveOptions{});
  const DiscWorld world({Vec2(-1.5, 0.0)}, 0.15);
  {
    const vf::TrackingLaw law = vf::TrackingLaw::kHeadingTracking;  // the FieldProbe path
    const vf::Tracker tracker{contourConfig(law)};
    vf::TrackerInput input;
    input.curve = curve.get();
    input.dt = kPeriod;
    input.obstacles = world.map();
    const RunCount count =
      countSteadyState(tracker, input, Pose2{Vec2(1.5, 0.0), vf::kHalfPi}, vf::kInf);
    EXPECT_GT(count.contour_cycles, 0) << vf::toString(law);
    EXPECT_EQ(count.allocations, 0) << vf::toString(law);
  }
}

}  // namespace
