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

// Basic types and helpers (types.hpp), Span and FunctionRef.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <type_traits>
#include <vector>

#include "vectorfield_core/function_ref.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::Vec2;

namespace
{

int twice(int x) { return 2 * x; }

int applyTo(vf::FunctionRef<int(int)> f, int x) { return f(x); }

// Non-owning views must not bind to temporaries or drop const.
static_assert(std::is_constructible_v<vf::Span<const int>, std::vector<int> &>, "vector");
static_assert(std::is_constructible_v<vf::Span<const int>, const std::vector<int> &>, "const");
static_assert(!std::is_constructible_v<vf::Span<int>, const std::vector<int> &>, "drops const");
static_assert(!std::is_constructible_v<vf::Span<const int>, std::vector<int> &&>, "temporary");
static_assert(std::is_constructible_v<vf::Span<const int>, vf::Span<int>>, "add const");
static_assert(!std::is_constructible_v<vf::Span<int>, vf::Span<const int>>, "remove const");

}  // namespace

TEST(Types, WrapAngleIsHalfOpen)
{
  EXPECT_EQ(vf::wrapAngle(vf::kPi), vf::kPi);
  EXPECT_EQ(vf::wrapAngle(-vf::kPi), vf::kPi);
  EXPECT_EQ(vf::wrapAngle(0.0), 0.0);
  EXPECT_NEAR(vf::wrapAngle(vf::kTwoPi + 0.1), 0.1, 1e-15);
  EXPECT_NEAR(vf::wrapAngle(-vf::kTwoPi - 0.1), -0.1, 1e-15);
  EXPECT_TRUE(std::isnan(vf::wrapAngle(vf::kNaN)));
  EXPECT_TRUE(std::isnan(vf::wrapAngle(vf::kInf)));

  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> angle(-1e3, 1e3);
  for (int i = 0; i < 10000; ++i) {
    const double a = angle(rng);
    const double w = vf::wrapAngle(a);
    ASSERT_GT(w, -vf::kPi);
    ASSERT_LE(w, vf::kPi);
    ASSERT_NEAR(std::cos(w), std::cos(a), 1e-12);
    ASSERT_NEAR(std::sin(w), std::sin(a), 1e-12);
  }
}

TEST(Types, WrapPeriodic)
{
  EXPECT_DOUBLE_EQ(vf::wrapPeriodic(0.5, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(vf::wrapPeriodic(-0.5, 1.0), 0.5);
  EXPECT_NEAR(vf::wrapPeriodic(0.7, 1.0), -0.3, 1e-15);
  EXPECT_NEAR(vf::wrapPeriodic(-7.25, 2.0), 0.75, 1e-15);
  EXPECT_DOUBLE_EQ(vf::wrapPeriodic(3.0, 0.0), 3.0);
  EXPECT_DOUBLE_EQ(vf::wrapPeriodic(3.0, vf::kInf), 3.0);
}

TEST(Types, ClampAndSign)
{
  EXPECT_DOUBLE_EQ(vf::clamp(2.0, -1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(vf::clamp(-2.0, -1.0, 1.0), -1.0);
  EXPECT_DOUBLE_EQ(vf::clamp(0.3, -1.0, 1.0), 0.3);
  EXPECT_DOUBLE_EQ(vf::clamp(0.3, 1.0, -1.0), -1.0);  // min(u, max(l, a)) with l > u
  EXPECT_TRUE(std::isnan(vf::clamp(vf::kNaN, -1.0, 1.0)));

  EXPECT_EQ(vf::sgn(0.0), 0.0);
  EXPECT_EQ(vf::sgn(-0.0), 0.0);
  EXPECT_EQ(vf::sgn(vf::kNaN), 0.0);
  EXPECT_EQ(vf::sgn(3.0), 1.0);
  EXPECT_EQ(vf::sgn(-1e-300), -1.0);
}

TEST(Types, PlanarAlgebra)
{
  const Vec2 v(0.3, -1.7);
  EXPECT_EQ(vf::J(Vec2(1.0, 0.0)), Vec2(0.0, 1.0));
  EXPECT_EQ(vf::J(vf::J(v)), -v);
  EXPECT_DOUBLE_EQ(vf::cross2(Vec2(1.0, 0.0), Vec2(0.0, 1.0)), 1.0);
  // Not exactly 0: compilers may contract a*b - c*d into a fused multiply-add.
  EXPECT_NEAR(vf::cross2(v, v), 0.0, 1e-15);
  EXPECT_NEAR(v.dot(vf::J(v)), 0.0, 1e-15);

  const double psi = 0.83;
  EXPECT_NEAR((vf::heading(psi) - vf::rotation(psi) * Vec2::UnitX()).norm(), 0.0, 1e-15);
  EXPECT_NEAR((vf::rotate(psi, v) - vf::rotation(psi) * v).norm(), 0.0, 1e-15);
  const vf::Mat2 j_matrix = vf::rotation(vf::kHalfPi);
  EXPECT_NEAR((j_matrix * v - vf::J(v)).norm(), 0.0, 1e-15);
  EXPECT_NEAR(vf::angleOf(vf::heading(psi)), psi, 1e-15);

  EXPECT_EQ(vf::unitOr(Vec2(3.0, 4.0), Vec2::Zero()), Vec2(0.6, 0.8));
  EXPECT_EQ(vf::unitOr(Vec2::Zero(), Vec2::UnitY()), Vec2::UnitY());
  EXPECT_EQ(vf::unitOr(Vec2(vf::kNaN, 1.0), Vec2::UnitX()), Vec2::UnitX());

  EXPECT_TRUE(vf::isFinite(vf::Pose2{}));
  EXPECT_FALSE(vf::isFinite(vf::Twist2{0.0, vf::kNaN, 0.0}));
  EXPECT_FALSE(vf::isFinite(Vec2(vf::kInf, 0.0)));
}

TEST(Types, SpecDerivedLengths)
{
  EXPECT_DOUBLE_EQ(vf::chordLength(0.05), 0.10);
  EXPECT_DOUBLE_EQ(vf::chordLength(0.10), 0.20);
  EXPECT_DOUBLE_EQ(vf::blockedReleaseBand(0.05), 0.10);
  EXPECT_DOUBLE_EQ(vf::blockedReleaseBand(0.01), 0.05);
}

TEST(Types, IsometryComposesAndInverts)
{
  const vf::Isometry2 a(0.7, Vec2(1.0, -2.0));
  const vf::Isometry2 b(-2.9, Vec2(0.25, 3.5));
  const Vec2 x(-0.4, 1.3);

  EXPECT_NEAR((a.applyInverse(a.apply(x)) - x).norm(), 0.0, 1e-14);
  EXPECT_NEAR((a.inverse().apply(a.apply(x)) - x).norm(), 0.0, 1e-14);
  EXPECT_NEAR(((a * b).apply(x) - a.apply(b.apply(x))).norm(), 0.0, 1e-14);
  EXPECT_NEAR(((a * a.inverse()).apply(x) - x).norm(), 0.0, 1e-14);
  EXPECT_NEAR((a.rotation() - vf::rotation(0.7)).norm(), 0.0, 1e-15);
  EXPECT_NEAR((a.rotateInverse(a.rotate(x)) - x).norm(), 0.0, 1e-15);
  EXPECT_NEAR(vf::wrapAngle((a * b).yaw() - (0.7 - 2.9)), 0.0, 1e-15);

  const vf::Pose2 pose{Vec2(0.5, 0.5), 2.0};
  const vf::Pose2 mapped = a.apply(pose);
  EXPECT_NEAR((mapped.p - a.apply(pose.p)).norm(), 0.0, 1e-15);
  EXPECT_NEAR(vf::wrapAngle(mapped.yaw - 2.7), 0.0, 1e-15);
  const vf::Pose2 back = a.applyInverse(mapped);
  EXPECT_NEAR((back.p - pose.p).norm(), 0.0, 1e-14);
  EXPECT_NEAR(vf::wrapAngle(back.yaw - pose.yaw), 0.0, 1e-15);

  // A pose's transform maps body coordinates into the parent frame.
  const vf::Isometry2 body = vf::Isometry2::fromPose(pose);
  EXPECT_NEAR((body.apply(Vec2::UnitX()) - (pose.p + vf::heading(pose.yaw))).norm(), 0.0, 1e-15);
  EXPECT_NEAR((body.toPose().p - pose.p).norm(), 0.0, 0.0);

  const vf::Isometry2 identity;
  EXPECT_EQ(identity.apply(x), x);
  EXPECT_EQ(vf::Isometry2::identity().yaw(), 0.0);
}

TEST(Span, ViewsContainersWithoutCopying)
{
  std::vector<int> values{1, 2, 3, 4, 5};
  const vf::Span<int> mutable_view(values);
  mutable_view[0] = 10;
  EXPECT_EQ(values[0], 10);

  const vf::Span<const int> view = mutable_view;
  EXPECT_EQ(view.size(), 5U);
  EXPECT_EQ(view.data(), values.data());
  EXPECT_EQ(std::accumulate(view.begin(), view.end(), 0), 24);
  EXPECT_EQ(view.front(), 10);
  EXPECT_EQ(view.back(), 5);
  EXPECT_EQ(view.first(2).size(), 2U);
  EXPECT_EQ(view.last(2)[0], 4);
  EXPECT_EQ(view.subspan(3, 10).size(), 2U);
  EXPECT_EQ(view.subspan(1, 2)[1], 3);

  const std::array<double, 3> array{0.5, 1.5, 2.5};
  const vf::Span<const double> array_view(array);
  EXPECT_EQ(array_view.size(), 3U);

  int raw[2] = {7, 8};
  const vf::Span<int> raw_view(raw);
  EXPECT_EQ(raw_view[1], 8);

  const vf::Span<const Vec2> empty;
  EXPECT_TRUE(empty.empty());
  EXPECT_EQ(empty.begin(), empty.end());
}

TEST(FunctionRef, ReferencesCallablesWithoutOwning)
{
  int calls = 0;
  auto counter = [&calls](int x) {
    ++calls;
    return x + 1;
  };
  const vf::FunctionRef<int(int)> ref = counter;
  EXPECT_TRUE(static_cast<bool>(ref));
  EXPECT_EQ(ref(1), 2);
  EXPECT_EQ(ref(5), 6);
  EXPECT_EQ(calls, 2);

  // A temporary lambda passed as an argument lives for the duration of the call.
  EXPECT_EQ(applyTo([](int x) { return x * x; }, 7), 49);

  // Functions and function pointers are stored by value.
  vf::FunctionRef<int(int)> from_function = twice;
  EXPECT_EQ(from_function(4), 8);
  {
    int (*pointer)(int) = &twice;
    from_function = pointer;
  }
  EXPECT_EQ(from_function(5), 10);

  // A mutable callable keeps its state in the referenced object.
  struct Accumulator
  {
    double sum = 0.0;
    void operator()(double x) { sum += x; }
  } accumulator;
  const vf::FunctionRef<void(double)> add = accumulator;
  add(1.5);
  add(2.0);
  EXPECT_DOUBLE_EQ(accumulator.sum, 3.5);

  // Return-type conversion and an empty reference.
  const vf::FunctionRef<double(const Vec2 &)> norm = [](const Vec2 & v) { return v.norm(); };
  EXPECT_DOUBLE_EQ(norm(Vec2(3.0, 4.0)), 5.0);
  const vf::FunctionRef<int(int)> empty;
  EXPECT_FALSE(static_cast<bool>(empty));
  int (*null_function)(int) = nullptr;
  const vf::FunctionRef<int(int)> from_null = null_function;
  EXPECT_FALSE(static_cast<bool>(from_null));
}
