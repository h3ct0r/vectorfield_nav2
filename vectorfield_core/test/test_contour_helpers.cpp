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

// Unit tests of the pure contour helpers (contour.hpp; math_spec §6.2–§6.4).

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "vectorfield_core/contour.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/occupancy.hpp"

namespace vectorfield_core
{
namespace
{

TEST(ContourBlendWeight, BandIsLinearAndClamped)
{
  EXPECT_EQ(contourBlendWeight(0.50, 0.62, 0.82), 0.0);
  EXPECT_NEAR(contourBlendWeight(0.72, 0.62, 0.82), 0.5, 1e-12);
  EXPECT_EQ(contourBlendWeight(0.82, 0.62, 0.82), 1.0);
  EXPECT_EQ(contourBlendWeight(0.85, 0.62, 0.82), 1.0);  // latched band: F = Phi
  EXPECT_EQ(contourBlendWeight(kInf, 0.62, 0.82), 1.0);
}

TEST(ContourBlendWeight, EqualDistancesAreAHardSwitch)
{
  // math_spec §6.4 and upstream U15: theta is defined, 0 below D_in and 1 otherwise.
  EXPECT_EQ(contourBlendWeight(0.59, 0.6, 0.6), 0.0);
  EXPECT_EQ(contourBlendWeight(0.60, 0.6, 0.6), 1.0);
  EXPECT_EQ(contourBlendWeight(0.61, 0.6, 0.6), 1.0);
}

TEST(ContourField, HasTheShapedSpeedAndAttractsToTheLambdaSet)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const double lambda = 0.47;
  const double speed = 0.4;
  const Vec2 n_o(0.6, 0.8);
  for (const double D_o : {0.1, 0.3, 0.6, 1.2}) {
    for (const int side : {1, -1}) {
      const Vec2 psi = contourField(n_o, D_o, lambda, side, gain, speed);
      EXPECT_NEAR(psi.norm(), speed, 1e-12);
      // Radially away from the obstacle inside lambda, toward it outside.
      EXPECT_EQ(sgn(psi.dot(n_o)), D_o < lambda ? 1.0 : -1.0) << "D_o " << D_o;
      // The tangential part follows the latched side.
      EXPECT_EQ(sgn(psi.dot(J(n_o))), static_cast<double>(side));
    }
  }
  // On the lambda set the field is pure circulation, finite (upstream U16).
  const Vec2 on_set = contourField(n_o, lambda, lambda, -1, gain, speed);
  EXPECT_NEAR((on_set + speed * J(n_o)).norm(), 0.0, 1e-15);
}

TEST(BlendContour, EndpointsAreExactAndTheNormIsKept)
{
  const Vec2 phi(0.3, -0.2);
  const Vec2 psi = rotate(2.0, phi);  // same speed, as contourField() builds it
  EXPECT_EQ(blendContour(phi, psi, 1.0), phi);
  EXPECT_EQ(blendContour(phi, psi, 1.5), phi);
  EXPECT_EQ(blendContour(phi, psi, 0.0), psi);
  for (const double theta : {0.1, 0.5, 0.9}) {
    EXPECT_NEAR(blendContour(phi, psi, theta).norm(), phi.norm(), 1e-15);
  }
}

TEST(BlendContour, DegenerateBlendFallsBackToPsi)
{
  const Vec2 phi(0.4, 0.0);
  const Vec2 psi(-0.4, 0.0);
  EXPECT_EQ(blendContour(phi, psi, 0.5), psi);
  EXPECT_EQ(blendContour(Vec2::Zero(), Vec2::Zero(), 0.5), Vec2::Zero());
}

TEST(ContourConditions, StartExitAndTheLatchedBand)
{
  const double d_in_prime = 0.82;
  const Vec2 n_o(1.0, 0.0);
  const Vec2 toward(-0.4, 0.05);  // n_o^T Phi < 0
  const Vec2 away(0.1, 0.39);     // n_o^T Phi = 0.1 >= 0.05 ||Phi||
  EXPECT_TRUE(contourStartCondition(0.80, n_o, toward, d_in_prime));
  EXPECT_FALSE(contourStartCondition(0.83, n_o, toward, d_in_prime));
  EXPECT_FALSE(contourStartCondition(0.50, n_o, away, d_in_prime));
  EXPECT_FALSE(contourStartCondition(0.50, n_o, Vec2(0.0, 0.4), d_in_prime));  // n_o^T Phi = 0

  EXPECT_TRUE(contourExitCondition(0.50, n_o, away, d_in_prime));
  EXPECT_FALSE(contourExitCondition(0.50, n_o, Vec2(0.01, 0.4), d_in_prime));  // below 5 %
  EXPECT_FALSE(contourExitCondition(0.86, n_o, toward, d_in_prime));  // inside (D'_in, + h_c]
  EXPECT_TRUE(contourExitCondition(0.88, n_o, toward, d_in_prime));
}

TEST(UpdateGate, OpensOnABlockedSampleAndClosesAfterTheReleaseTime)
{
  // dt = 0.125 s is exact in binary, so four clear cycles reach the 0.5 s release exactly.
  GateUpdate gate = updateGate(false, 0.0, true, 0.125, 0.5);
  EXPECT_TRUE(gate.open);
  EXPECT_EQ(gate.clear_s, 0.0);
  int clear_cycles = 0;
  while (gate.open && clear_cycles < 100) {
    gate = updateGate(gate.open, gate.clear_s, false, 0.125, 0.5);
    ++clear_cycles;
  }
  EXPECT_EQ(clear_cycles, 4);
  EXPECT_FALSE(gate.open);

  // A blocked sample re-arms the timer; a closed gate stays closed while the path is clear.
  gate = updateGate(true, 0.375, true, 0.125, 0.5);
  EXPECT_TRUE(gate.open);
  EXPECT_EQ(gate.clear_s, 0.0);
  gate = updateGate(false, 0.0, false, 0.125, 0.5);
  EXPECT_FALSE(gate.open);
  EXPECT_EQ(gate.clear_s, 0.0);
}

TEST(SideProbeHorizon, CoversTheRadialApproachAndHalfACirculation)
{
  EXPECT_NEAR(sideProbeHorizon(1.5, 0.82, 0.47, 0.4), (0.82 - 0.47 + kPi * 0.47) / 0.4, 1e-12);
  EXPECT_NEAR(sideProbeHorizon(1.5, 0.82, 0.47, 0.4), 4.566, 1e-3);  // TB3, math_spec App. B
  EXPECT_EQ(sideProbeHorizon(10.0, 0.82, 0.47, 0.4), 10.0);
  EXPECT_EQ(sideProbeHorizon(1.5, 0.82, 0.47, 0.0), 1.5);
}

SideProbeResult openSide()
{
  SideProbeResult side;
  side.collision_free = true;
  side.exit_reached = false;
  side.clearance = 0.3;
  side.path_to_exit = kInf;
  side.s_free = kInf;
  return side;
}

TEST(ChooseContourSide, RanksTheSidesInTheSpecifiedOrder)
{
  const double res = 0.05;
  const double m_s = 0.1;
  const double min_free = 0.18;  // v_r^2 / (2 a_dec) + m_s
  const double phi_norm = 0.4;
  const SideProbeResult base = openSide();

  // 7. Everything ties and (3.67) is undefined: +1.
  EXPECT_EQ(chooseContourSide(base, base, res, m_s, min_free, 0.0, phi_norm), 1);
  // 6. The side (3.67) picks, only when |(J n_o)^T Phi| >= 0.1 ||Phi||.
  EXPECT_EQ(chooseContourSide(base, base, res, m_s, min_free, -0.05, phi_norm), -1);
  EXPECT_EQ(chooseContourSide(base, base, res, m_s, min_free, -0.03, phi_norm), 1);

  // The remaining checks pass jn = +0.3, so a -1 proves the higher rank decided.
  const double jn = 0.3;
  SideProbeResult plus = base;
  SideProbeResult minus = base;
  // 5. Larger s_free.
  plus.s_free = 1.0;
  minus.s_free = 2.0;
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), -1);
  // 4. Shorter path to the exit (both reach it).
  plus = base;
  minus = base;
  plus.exit_reached = minus.exit_reached = true;
  plus.path_to_exit = 2.0;
  minus.path_to_exit = 1.5;
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), -1);
  // 3. Larger saturated clearance; equal within res/2; saturated at 2 m_s.
  plus = base;
  minus = base;
  plus.clearance = 0.05;
  minus.clearance = 0.15;
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), -1);
  plus.clearance = 0.10;
  minus.clearance = 0.12;  // within res/2: tie, decided by rank 6
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), 1);
  plus.clearance = 0.30;
  minus.clearance = 0.50;  // both saturate at 0.2: tie
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), 1);
  // 2. Exit reached beats clearance.
  plus = base;
  minus = base;
  plus.clearance = 0.2;
  minus.clearance = 0.0;
  minus.exit_reached = true;
  minus.path_to_exit = 3.0;
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), -1);
  // 1. Collision-free beats everything.
  plus = base;
  minus = base;
  plus.collision_free = false;
  plus.exit_reached = true;
  plus.path_to_exit = 0.5;
  plus.s_free = 2.0;
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), -1);
  // Neither side leaves room to brake: no episode.
  plus = base;
  minus = base;
  plus.collision_free = minus.collision_free = false;
  plus.s_free = 0.10;
  minus.s_free = 0.15;
  EXPECT_EQ(chooseContourSide(plus, minus, res, m_s, min_free, jn, phi_norm), 0);
}

TEST(ContourWatchdog, TripsOnLengthOrTime)
{
  const ObstacleConfig config;  // 5 m, 20 s
  EXPECT_FALSE(contourWatchdogTripped(5.0, 20.0, config));
  EXPECT_TRUE(contourWatchdogTripped(5.01, 0.0, config));
  EXPECT_TRUE(contourWatchdogTripped(0.0, 20.01, config));
}

TEST(MarkBlockedSamples, FlagsSamplesWithinTheLateralHalfWidthMinusTheTolerance)
{
  // 40 x 40 cells of 0.05 m from the origin; one site at cell (20, 20), centre (1.025, 1.025).
  std::vector<std::uint8_t> mask(40 * 40, 0);
  mask[20 * 40 + 20] = 1;
  OccupancyGrid grid;
  ASSERT_TRUE(grid.setFromMask(Span<const std::uint8_t>(mask), 40, 40, 0.05, Vec2(0.0, 0.0)));
  EuclideanDistanceTransform edt;
  edt.update(grid);
  const std::vector<Vec2> samples = {
    Vec2(0.3, 1.025),    // 0.725 m from the site: clear
    Vec2(0.9, 1.025),    // 0.125 m < 0.22 - 0.05: blocked
    Vec2(1.025, 1.025),  // on the site
    Vec2(5.0, 5.0)};     // off the grid: never blocked
  std::vector<std::uint8_t> blocked(samples.size(), 9);
  const int first =
    markBlockedSamples(Span<const Vec2>(samples), edt, 0.22, 0.05, Span<std::uint8_t>(blocked));
  EXPECT_EQ(first, 1);
  EXPECT_EQ(blocked[0], 0);
  EXPECT_EQ(blocked[1], 1);
  EXPECT_EQ(blocked[2], 1);
  EXPECT_EQ(blocked[3], 0);
}

}  // namespace
}  // namespace vectorfield_core
