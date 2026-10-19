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

/// @file
/// @brief LapCounter: progress and laps of a FollowCurve goal (plan §6 "Progress and laps";
/// math_spec §9.3). Goal-context state, not TrackerState. Pure.
///
/// - At the first update of a goal, \f$\theta_0 = s^\star/L\f$ and \f$p = 0\f$.
/// - Each update: \f$p \mathrel{+}= \mathrm{dir}\cdot\mathrm{wrap}(s^\star/L(t) -
///   \theta_{\mathrm{prev}})\f$, the wrap into \f$(-\tfrac12, \tfrac12]\f$, so dithering around
///   a station (and around \f$s = 0\f$) never counts a lap.
/// - `laps_completed` = \f$\lfloor p + 10^{-9}\rfloor\f$ (0 while \f$p < 0\f$; the slack
///   absorbs the rounding of the accumulated increments).
/// - Success: laps > 0 and laps_completed >= laps (closed); `output.settled` (open, decided by
///   the node).
/// - `remaining_arc_length`: closed with laps > 0: \f$(\mathrm{laps} - p)\,L\f$ (>= 0); laps = 0:
///   \f$+\infty\f$; open: \f$\sigma = \mathrm{dir}\,(s_{\mathrm{goal}} - s^\star)\f$ (>= 0), with
///   \f$s_{\mathrm{goal}}\f$ = L (dir +1) or 0 (dir −1).

#ifndef VECTORFIELD_CURVE_FOLLOWER__LAP_COUNTER_HPP_
#define VECTORFIELD_CURVE_FOLLOWER__LAP_COUNTER_HPP_

#include <cstdint>

namespace vectorfield_curve_follower
{

/// @brief Progress along a closed curve in laps, and the remaining arc length.
class LapCounter
{
public:
  /// @brief A counter for a new goal.
  /// @param direction Traversal direction ±1 (`reverse_direction` → −1).
  /// @param laps Requested laps (0 = until cancelled).
  /// @param closed The curve is closed (open curves never count laps).
  LapCounter(int direction = 1, std::uint32_t laps = 0, bool closed = true) noexcept;

  /// @brief Restarts for a new goal (the first update() sets \f$\theta_0\f$).
  void reset(int direction, std::uint32_t laps, bool closed) noexcept;

  /// @brief Feeds one cycle's station.
  /// @param s_star Closest-point station \f$s^\star\f$ (m), in \f$[0, L)\f$ on closed curves.
  /// @param length Curve length \f$L(t)\f$ (m); non-positive or non-finite values are ignored.
  void update(double s_star, double length) noexcept;

  /// @brief The first update happened.
  bool started() const noexcept { return started_; }
  /// @brief Progress \f$p\f$ in laps, positive along the traversal direction.
  double progress() const noexcept { return progress_; }
  /// @brief \f$\lfloor p\rfloor\f$, 0 while \f$p < 0\f$ (and on open curves).
  std::uint32_t lapsCompleted() const noexcept;
  /// @brief laps > 0 and lapsCompleted() >= laps (closed curves only).
  bool lapsDone() const noexcept;
  /// @brief Remaining arc length (see the file comment) for the last update.
  double remainingArcLength() const noexcept;

private:
  int direction_{1};
  std::uint32_t laps_{0};
  bool closed_{true};
  bool started_{false};
  double theta_prev_{0.0};
  double progress_{0.0};
  double s_star_{0.0};
  double length_{0.0};
};

}  // namespace vectorfield_curve_follower

#endif  // VECTORFIELD_CURVE_FOLLOWER__LAP_COUNTER_HPP_
