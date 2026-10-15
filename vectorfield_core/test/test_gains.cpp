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

// Gain functions: oracles O15 and O16 of math_spec §12.2.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::GainFunction;
using vf::GainType;
using vf::Vec2;

namespace
{

constexpr std::array<GainType, 2> kTypes{GainType::kAtan, GainType::kRational};
constexpr std::array<double, 3> kGains{0.5, 2.0, 7.3};

std::string label(GainType type, double k)
{
  return std::string(vf::toString(type)) + " k_f=" + std::to_string(k);
}

}  // namespace

// O15: G(0) = 0, G strictly increasing, |G^2 + H^2 - 1| <= 1e-15, G'(0) as tabulated (1e-12),
// G(G^-1(y)) = y (1e-12), on k_f D in [0, 50].
TEST(Gains, O15Identities)
{
  constexpr int kSamples = 200001;
  for (GainType type : kTypes) {
    for (double k : kGains) {
      SCOPED_TRACE(label(type, k));
      const GainFunction gain(type, k);
      EXPECT_EQ(gain.G(0.0), 0.0);
      EXPECT_EQ(gain.H(0.0), 1.0);

      const double expected_dG0 = type == GainType::kAtan ? 2.0 * k / vf::kPi : k;
      EXPECT_NEAR(gain.dG0(), expected_dG0, 1e-12);
      EXPECT_NEAR(gain.dG(0.0), gain.dG0(), 1e-12);

      double previous = -1.0;
      double worst_unit = 0.0;
      double worst_inverse = 0.0;
      for (int i = 0; i < kSamples; ++i) {
        const double u = 50.0 * static_cast<double>(i) / (kSamples - 1);
        const double D = u / k;
        const double g = gain.G(D);
        const double h = gain.H(D);
        ASSERT_GT(g, previous) << "G not strictly increasing at u = " << u;
        previous = g;
        ASSERT_GE(g, 0.0);
        ASSERT_LT(g, 1.0);
        ASSERT_GT(h, 0.0);
        worst_unit = std::max(worst_unit, std::abs(g * g + h * h - 1.0));
        worst_inverse = std::max(worst_inverse, std::abs(gain.G(gain.Ginv(g)) - g));
      }
      EXPECT_LE(worst_unit, 1e-15);
      EXPECT_LE(worst_inverse, 1e-12);

      for (int i = 0; i <= 10000; ++i) {
        const double y = 0.999999 * static_cast<double>(i) / 10000.0;
        ASSERT_NEAR(gain.G(gain.Ginv(y)), y, 1e-12) << "y = " << y;
      }
    }
  }
}

TEST(Gains, DerivativeMatchesFiniteDifference)
{
  for (GainType type : kTypes) {
    const GainFunction gain(type, 2.0);
    SCOPED_TRACE(label(type, 2.0));
    for (double D : {0.01, 0.1, 0.4, 1.0, 3.0, 10.0}) {
      const double h = 1e-6 * std::max(1.0, D);
      const double numeric = (gain.G(D + h) - gain.G(D - h)) / (2.0 * h);
      EXPECT_NEAR(gain.dG(D), numeric, 1e-7 * std::max(1.0, gain.dG(D))) << "D = " << D;
    }
  }
}

TEST(Gains, ExtendsOddlyAndSaturates)
{
  for (GainType type : kTypes) {
    const GainFunction gain(type, 2.0);
    SCOPED_TRACE(label(type, 2.0));
    for (double D : {0.05, 0.7, 4.0}) {
      EXPECT_DOUBLE_EQ(gain.G(-D), -gain.G(D));
      EXPECT_DOUBLE_EQ(gain.H(-D), gain.H(D));
      EXPECT_DOUBLE_EQ(gain.Ginv(-gain.G(D)), -gain.Ginv(gain.G(D)));
    }
    EXPECT_EQ(gain.Ginv(1.0), vf::kInf);
    EXPECT_EQ(gain.Ginv(-1.0), -vf::kInf);
    EXPECT_TRUE(std::isnan(gain.Ginv(vf::kNaN)));
    EXPECT_LT(gain.G(1e12), 1.0 + 1e-15);
    EXPECT_GE(gain.H(1e12), 0.0);
  }
}

TEST(Gains, RejectsInvalidConvergenceGain)
{
  EXPECT_THROW(GainFunction(GainType::kAtan, 0.0), vf::ConfigError);
  EXPECT_THROW(GainFunction(GainType::kRational, -1.0), vf::ConfigError);
  EXPECT_THROW(GainFunction(GainType::kAtan, vf::kNaN), vf::ConfigError);
  EXPECT_THROW(GainFunction(GainType::kAtan, vf::kInf), vf::ConfigError);

  vf::FieldConfig field;
  field.gain_function = GainType::kRational;
  field.convergence_gain = 3.0;
  const GainFunction gain = GainFunction::fromConfig(field);
  EXPECT_EQ(gain.type(), GainType::kRational);
  EXPECT_DOUBLE_EQ(gain.k(), 3.0);

  const GainFunction defaults;
  EXPECT_EQ(defaults.type(), GainType::kAtan);
  EXPECT_DOUBLE_EQ(defaults.k(), 2.0);
}

// O16 (first part): max of G G'/H over a 1e6-point grid of k_f D in (0, 20] equals
// 0.18557 k_f (atan) and 0.38490 k_f (rational) within 1e-4 k_f.
TEST(Gains, O16CurvatureConstant)
{
  constexpr int kGrid = 1000000;
  for (GainType type : kTypes) {
    for (double k : kGains) {
      SCOPED_TRACE(label(type, k));
      const GainFunction gain(type, k);
      double best = 0.0;
      for (int i = 1; i <= kGrid; ++i) {
        const double u = 20.0 * static_cast<double>(i) / kGrid;
        best = std::max(best, gain.integralCurveCurvature(u / k));
      }
      const double tabulated = type == GainType::kAtan ? 0.18557 : 0.38490;
      EXPECT_NEAR(best, tabulated * k, 1e-4 * k);
      EXPECT_NEAR(gain.maxIntegralCurvature(), best, 1e-9 * k);
      EXPECT_NEAR(gain.cG() * k, gain.maxIntegralCurvature(), 1e-15 * k);
    }
  }
  EXPECT_NEAR(vf::kCgRational, 2.0 / (3.0 * std::sqrt(3.0)), 1e-16);
}

// O16 (second part): the curvature of a static field's integral curve near a straight path,
// integrated by RK4, never exceeds c_G k_f + 1e-3.
TEST(Gains, O16IntegralCurveCurvatureByRk4)
{
  for (GainType type : kTypes) {
    for (double k : {1.0, 2.0, 5.0}) {
      SCOPED_TRACE(label(type, k));
      const GainFunction gain(type, k);
      // Path y = 0 traversed in +x: T = (1, 0), D = |y|, n = (0, sgn y); unit-speed field.
      auto field = [&gain](const Vec2 & x) {
        const double D = std::abs(x.y());
        return Vec2(gain.H(D), -gain.G(D) * vf::sgn(x.y()));
      };
      double worst = 0.0;
      for (double start : {0.05, 0.5, 2.0, 8.0}) {
        Vec2 x(0.0, start / k);
        const double step = 1e-3 / k;
        for (int i = 0; i < 40000 && x.y() > 1e-6 / k; ++i) {
          const Vec2 k1 = field(x);
          const Vec2 k2 = field(x + 0.5 * step * k1);
          const Vec2 k3 = field(x + 0.5 * step * k2);
          const Vec2 k4 = field(x + step * k3);
          const Vec2 next = x + step / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
          const double turn = std::abs(vf::wrapAngle(vf::angleOf(field(next)) - vf::angleOf(k1)));
          worst = std::max(worst, turn / (next - x).norm());
          x = next;
        }
      }
      EXPECT_LE(worst, gain.cG() * k + 1e-3);
      EXPECT_GT(worst, 0.9 * gain.cG() * k) << "the run must pass the curvature peak";
    }
  }
}
