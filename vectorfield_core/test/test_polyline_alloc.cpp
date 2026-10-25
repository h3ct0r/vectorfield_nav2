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

// Allocation behaviour of the polyline hot path: closestPoint() and the
// curve queries never allocate, assign() reuses capacity, and the plugin's per-cycle
// conditioning helpers are allocation-free once their outputs have capacity.
//
// A replacement global operator new counts allocations. Under AddressSanitizer or
// ThreadSanitizer the replacement is not compiled (it would fight the sanitizer's own allocator
// bookkeeping) and the tests are skipped, as for the test_noalloc* executables.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <vector>

#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/types.hpp"

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
  __has_feature(memory_sanitizer)
#define VF_POLYLINE_ALLOC_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define VF_POLYLINE_ALLOC_SANITIZED 1
#endif

namespace
{

std::atomic<bool> g_counting{false};
std::atomic<long> g_allocations{0};

}  // namespace

#ifndef VF_POLYLINE_ALLOC_SANITIZED
void * operator new(std::size_t size)
{
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  if (void * memory = std::malloc(size == 0 ? 1 : size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new[](std::size_t size) { return operator new(size); }

void operator delete(void * memory) noexcept { std::free(memory); }

void operator delete[](void * memory) noexcept { std::free(memory); }

void operator delete(void * memory, std::size_t) noexcept { std::free(memory); }

void operator delete[](void * memory, std::size_t) noexcept { std::free(memory); }
#endif

namespace vf = vectorfield_core;
using vf::CurveHint;
using vf::EvalContext;
using vf::PolylineCurve;
using vf::SearchPolicy;
using vf::Vec2;

namespace
{

// Counts the allocations made while it is alive.
class AllocationScope
{
public:
  AllocationScope()
  {
    g_allocations.store(0);
    g_counting.store(true);
  }
  ~AllocationScope() { g_counting.store(false); }
  AllocationScope(const AllocationScope &) = delete;
  AllocationScope & operator=(const AllocationScope &) = delete;

  long count() const { return g_allocations.load(); }
};

#ifdef VF_POLYLINE_ALLOC_SANITIZED
constexpr bool kCounting = false;
#else
constexpr bool kCounting = true;
#endif

std::vector<Vec2> zigzag(int count, double phase)
{
  std::vector<Vec2> points;
  for (int i = 0; i < count; ++i) {
    points.emplace_back(0.05 * i, 0.3 * std::sin(0.4 * i + phase) + (i % 7 == 3 ? 0.2 : 0.0));
  }
  return points;
}

}  // namespace

TEST(PolylineAlloc, CountingSeesLibraryAllocations)
{
  if (!kCounting) {
    GTEST_SKIP() << "allocation counting is disabled under sanitizers";
  }
  const std::vector<Vec2> input = zigzag(10, 0.0);
  std::vector<Vec2> out;  // no capacity: dedupe must allocate inside the library
  long count = 0;
  {
    AllocationScope scope;
    vf::dedupe(input, false, &out);
    count = scope.count();
  }
  EXPECT_GT(count, 0);
}

TEST(PolylineAlloc, ClosestPointAndQueriesDoNotAllocate)
{
  if (!kCounting) {
    GTEST_SKIP() << "allocation counting is disabled under sanitizers";
  }
  std::vector<Vec2> square;
  for (int i = 0; i < 40; ++i) {
    square.emplace_back(0.1 * i, 0.0);
  }
  for (int i = 0; i < 40; ++i) {
    square.emplace_back(4.0, 0.1 * i);
  }
  for (int i = 0; i < 40; ++i) {
    square.emplace_back(4.0 - 0.1 * i, 4.0);
  }
  for (int i = 0; i < 40; ++i) {
    square.emplace_back(0.0, 4.0 - 0.1 * i);
  }
  PolylineCurve closed(SearchPolicy::hinted(1.0, 0.5, 0.1));
  PolylineCurve open(SearchPolicy::windowScan(-0.5, 0.7));
  ASSERT_EQ(closed.assign(square, true), vf::AssignStatus::kOk);
  const std::vector<Vec2> path = zigzag(120, 0.3);
  ASSERT_EQ(open.assign(path, false), vf::AssignStatus::kOk);

  std::array<Vec2, 64> samples;
  EvalContext ctx;
  ctx.last_tangent = Vec2(1.0, 0.0);
  long count = 0;
  double sink = 0.0;
  {
    AllocationScope scope;
    CurveHint hint;
    for (int i = 0; i < 2000; ++i) {
      // Hinted: local window, wrap across the seam, edge walks and global-stage jumps.
      const double s = 0.0137 * i;
      const Vec2 near = closed.pointAt(s, 0.0) + 0.05 * Vec2(std::cos(i), std::sin(i));
      const Vec2 far = i % 50 == 0 ? Vec2(2.0, 1.2) : near;
      const vf::ClosestPoint cp = closed.closestPoint(far, ctx, hint);
      hint = vf::makeHint(cp);
      sink += cp.s_star + closed.tangentAt(s, 0.0).x();
      // WindowScan: the window, edge extension and rays.
      const Vec2 x(0.05 * (i % 200) - 2.0, 0.2 * std::sin(0.1 * i));
      sink += open.closestPoint(x, ctx, vf::makeHint(cp)).D;
      sink += static_cast<double>(open.sampleStations(0.0, 3.0, 0.05, 0.0, samples));
    }
    count = scope.count();
  }
  EXPECT_EQ(count, 0);
  EXPECT_TRUE(std::isfinite(sink));
}

TEST(PolylineAlloc, ReassignWithinCapacityDoesNotAllocate)
{
  if (!kCounting) {
    GTEST_SKIP() << "allocation counting is disabled under sanitizers";
  }
  PolylineCurve window(SearchPolicy::windowScan(-0.5, 0.7));
  window.reserve(150);
  std::vector<std::vector<Vec2>> windows;
  for (int cycle = 0; cycle < 30; ++cycle) {
    windows.push_back(zigzag(20 + 4 * cycle, 0.1 * cycle));
  }
  long count = 0;
  {
    AllocationScope scope;
    for (const std::vector<Vec2> & points : windows) {
      ASSERT_EQ(window.assign(points, false), vf::AssignStatus::kOk);
      ASSERT_EQ(window.assign(points, true), vf::AssignStatus::kOk);
    }
    count = scope.count();
  }
  EXPECT_EQ(count, 0);
}

TEST(ConditioningAlloc, PluginHelpersDoNotAllocateOnceWarm)
{
  if (!kCounting) {
    GTEST_SKIP() << "allocation counting is disabled under sanitizers";
  }
  std::vector<Vec2> deduped;
  std::vector<Vec2> tail;
  std::vector<Vec2> decimated;
  deduped.reserve(256);
  tail.reserve(256);
  decimated.reserve(256);
  std::vector<std::vector<Vec2>> windows;
  for (int cycle = 0; cycle < 30; ++cycle) {
    windows.push_back(zigzag(30 + 5 * cycle, 0.2 * cycle));
  }
  long count = 0;
  double sink = 0.0;
  {
    AllocationScope scope;
    for (const std::vector<Vec2> & window : windows) {
      const vf::EndTangent end = vf::endTangent(window, 0.1);
      vf::tailCondition(window, end.tangent, 0.1, &tail);
      vf::dedupe(tail, false, &deduped);
      vf::decimateChords(window, 0.1, &decimated);
      sink += vf::hasCusp(decimated) ? 1.0 : 0.0;
      sink += vf::kappaPath(deduped, false);
      const std::array<Vec2, 2> one_pose = vf::onePoseWindow(window[0], vf::Pose2{window[1], 0.3});
      sink += one_pose[0].x();
    }
    count = scope.count();
  }
  EXPECT_EQ(count, 0);
  EXPECT_TRUE(std::isfinite(sink));
}
