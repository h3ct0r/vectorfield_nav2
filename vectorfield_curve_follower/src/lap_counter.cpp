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

#include "vectorfield_curve_follower/lap_counter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "vectorfield_core/types.hpp"

namespace vectorfield_curve_follower
{

namespace
{

/// Rounding slack of the accumulated progress: a robot back exactly at its start station has
/// completed the lap even if the sum of the wrapped increments lands a few ulps short.
constexpr double kLapEpsilon = 1e-9;

}  // namespace

LapCounter::LapCounter(int direction, std::uint32_t laps, bool closed) noexcept
{
  reset(direction, laps, closed);
}

void LapCounter::reset(int direction, std::uint32_t laps, bool closed) noexcept
{
  direction_ = direction < 0 ? -1 : 1;
  laps_ = laps;
  closed_ = closed;
  started_ = false;
  theta_prev_ = 0.0;
  progress_ = 0.0;
  s_star_ = 0.0;
  length_ = 0.0;
}

void LapCounter::update(double s_star, double length) noexcept
{
  if (!std::isfinite(s_star) || !std::isfinite(length) || !(length > 0.0)) {
    return;
  }
  s_star_ = s_star;
  length_ = length;
  if (!closed_) {
    started_ = true;
    return;
  }
  const double theta = s_star / length;
  if (!started_) {
    started_ = true;
    theta_prev_ = theta;
    progress_ = 0.0;
    return;
  }
  progress_ +=
    static_cast<double>(direction_) * vectorfield_core::wrapPeriodic(theta - theta_prev_, 1.0);
  theta_prev_ = theta;
}

std::uint32_t LapCounter::lapsCompleted() const noexcept
{
  if (!closed_ || !(progress_ + kLapEpsilon >= 1.0)) {
    return 0;
  }
  const double laps = std::floor(progress_ + kLapEpsilon);
  constexpr auto kMax = static_cast<double>(std::numeric_limits<std::uint32_t>::max());
  return static_cast<std::uint32_t>(std::min(laps, kMax));
}

bool LapCounter::lapsDone() const noexcept
{
  return closed_ && laps_ > 0 && lapsCompleted() >= laps_;
}

double LapCounter::remainingArcLength() const noexcept
{
  if (!closed_) {
    if (!started_) {
      return vectorfield_core::kInf;
    }
    const double s_goal = direction_ > 0 ? length_ : 0.0;
    return std::max(0.0, static_cast<double>(direction_) * (s_goal - s_star_));
  }
  if (laps_ == 0 || !started_) {
    return vectorfield_core::kInf;
  }
  return std::max(0.0, (static_cast<double>(laps_) - progress_) * length_);
}

}  // namespace vectorfield_curve_follower
