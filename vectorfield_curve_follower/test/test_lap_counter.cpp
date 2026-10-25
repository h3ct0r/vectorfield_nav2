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

// LapCounter: wrap of s*/L, laps_completed, remaining_arc_length,
// ±5 cm dithering and a start 5 cm before s = 0.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "vectorfield_curve_follower/lap_counter.hpp"

namespace
{

using vectorfield_curve_follower::LapCounter;

constexpr double kLength = 10.0;

/// Drives the counter along a closed curve from `s0` by `distance` (signed) in `step` increments.
void travel(LapCounter * counter, double s0, double distance, double step = 0.05)
{
  const int n = static_cast<int>(std::lround(std::abs(distance) / step));
  const double ds = distance >= 0.0 ? step : -step;
  for (int k = 1; k <= n; ++k) {
    counter->update(std::fmod(std::fmod(s0 + ds * k, kLength) + kLength, kLength), kLength);
  }
}

TEST(LapCounter, CountsFullLaps)
{
  LapCounter counter(1, 2, true);
  EXPECT_EQ(counter.remainingArcLength(), std::numeric_limits<double>::infinity());
  counter.update(2.0, kLength);
  EXPECT_TRUE(counter.started());
  EXPECT_EQ(counter.lapsCompleted(), 0U);
  EXPECT_NEAR(counter.remainingArcLength(), 2.0 * kLength, 1e-12);
  travel(&counter, 2.0, 9.95);
  EXPECT_EQ(counter.lapsCompleted(), 0U);
  EXPECT_FALSE(counter.lapsDone());
  travel(&counter, 1.95, 0.05);
  EXPECT_EQ(counter.lapsCompleted(), 1U);
  EXPECT_NEAR(counter.progress(), 1.0, 1e-9);
  EXPECT_NEAR(counter.remainingArcLength(), kLength, 1e-9);
  travel(&counter, 2.0, kLength);
  EXPECT_EQ(counter.lapsCompleted(), 2U);
  EXPECT_TRUE(counter.lapsDone());
  EXPECT_NEAR(counter.remainingArcLength(), 0.0, 1e-9);
}

TEST(LapCounter, DitheringAroundTheSeamNeverCountsALap)
{
  LapCounter counter(1, 1, true);
  counter.update(0.0, kLength);
  for (int k = 0; k < 500; ++k) {
    counter.update(k % 2 == 0 ? 0.05 : kLength - 0.05, kLength);
    EXPECT_EQ(counter.lapsCompleted(), 0U);
    EXPECT_LE(std::abs(counter.progress()), 0.0051);
  }
  // ±5 cm around an interior station too.
  for (int k = 0; k < 100; ++k) {
    counter.update(k % 2 == 0 ? 5.05 : 4.95, kLength);
  }
  EXPECT_EQ(counter.lapsCompleted(), 0U);
}

TEST(LapCounter, AStartJustBeforeTheSeamCountsTheLapOnReturn)
{
  LapCounter counter(1, 1, true);
  counter.update(kLength - 0.05, kLength);
  // Crossing s = 0 five centimetres later is not a lap.
  travel(&counter, kLength - 0.05, 0.10);
  EXPECT_EQ(counter.lapsCompleted(), 0U);
  EXPECT_NEAR(counter.progress(), 0.01, 1e-9);
  // Back to the start station: one lap.
  travel(&counter, 0.05, kLength - 0.15);
  EXPECT_EQ(counter.lapsCompleted(), 0U);
  travel(&counter, kLength - 0.10, 0.05);
  EXPECT_EQ(counter.lapsCompleted(), 1U);
  EXPECT_TRUE(counter.lapsDone());
}

TEST(LapCounter, ReverseDirectionCountsPositively)
{
  LapCounter counter(-1, 1, true);
  counter.update(3.0, kLength);
  travel(&counter, 3.0, -kLength);
  EXPECT_EQ(counter.lapsCompleted(), 1U);
  EXPECT_NEAR(counter.progress(), 1.0, 1e-9);
  // Going the wrong way only decreases the progress.
  LapCounter wrong(1, 1, true);
  wrong.update(3.0, kLength);
  travel(&wrong, 3.0, -2.0);
  EXPECT_NEAR(wrong.progress(), -0.2, 1e-9);
  EXPECT_EQ(wrong.lapsCompleted(), 0U);
}

TEST(LapCounter, RemainingArcLengthRules)
{
  // laps = 0: until cancelled.
  LapCounter forever(1, 0, true);
  forever.update(1.0, kLength);
  travel(&forever, 1.0, 25.0);
  EXPECT_EQ(forever.lapsCompleted(), 2U);
  EXPECT_FALSE(forever.lapsDone());
  EXPECT_TRUE(std::isinf(forever.remainingArcLength()));

  // Open curve: sigma = dir (s_goal - s*), never a lap.
  LapCounter open(1, 0, false);
  open.update(4.0, kLength);
  EXPECT_NEAR(open.remainingArcLength(), 6.0, 1e-12);
  EXPECT_EQ(open.lapsCompleted(), 0U);
  LapCounter open_reverse(-1, 0, false);
  open_reverse.update(4.0, kLength);
  EXPECT_NEAR(open_reverse.remainingArcLength(), 4.0, 1e-12);
  open_reverse.update(-0.5, kLength);
  EXPECT_EQ(open_reverse.remainingArcLength(), 0.0);

  // A time-varying length: progress is measured in s*/L(t).
  LapCounter varying(1, 1, true);
  varying.update(0.0, 10.0);
  varying.update(5.5, 11.0);
  EXPECT_NEAR(varying.progress(), 0.5, 1e-12);

  // Invalid lengths are ignored.
  varying.update(1.0, 0.0);
  varying.update(1.0, std::nan(""));
  EXPECT_NEAR(varying.progress(), 0.5, 1e-12);
}

}  // namespace
