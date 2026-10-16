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

#include "vectorfield_core/contour.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace vectorfield_core
{

namespace
{

/// @brief Outcome of comparing the two sides on one ranking criterion.
enum class Preference : int
{
  kPlus = 1,
  kMinus = -1,
  kTie = 0,
};

/// @brief Prefers the side for which `better` holds alone; a tie when both or neither do.
Preference preferTrue(bool plus, bool minus) noexcept
{
  if (plus == minus) {
    return Preference::kTie;
  }
  return plus ? Preference::kPlus : Preference::kMinus;
}

/// @brief Prefers the larger value; equal values (including two infinities) tie.
Preference preferLarger(double plus, double minus) noexcept
{
  if (plus > minus) {
    return Preference::kPlus;
  }
  if (minus > plus) {
    return Preference::kMinus;
  }
  return Preference::kTie;
}

}  // namespace

double contourBlendWeight(double D_o, double D_in, double D_in_prime) noexcept
{
  if (std::isnan(D_o)) {
    return 1.0;  // no usable obstacle distance: the path field alone
  }
  if (!(D_in_prime > D_in)) {
    // Hard switch (math_spec §6.4; the thesis cone experiment): no blending band.
    return D_o < D_in ? 0.0 : 1.0;
  }
  return clamp((D_o - D_in) / (D_in_prime - D_in), 0.0, 1.0);
}

Vec2 contourField(
  const Vec2 & n_o, double D_o, double lambda, int sigma_c, const GainFunction & gain,
  double speed) noexcept
{
  // math_spec §6.2 with the latched side of §6.4: D_lambda = |D_o - lambda|,
  // Psi_G = -G(D_lambda) sgn(D_o - lambda) n_o and Psi_H = H(D_lambda) sigma_c J n_o. The two
  // terms are orthogonal and G^2 + H^2 = 1, so ||Psi|| = speed.
  const double offset = D_o - lambda;
  const double D_lambda = std::abs(offset);
  const Vec2 attract = -gain.G(D_lambda) * sgn(offset) * n_o;
  const Vec2 circulate = gain.H(D_lambda) * sgn(static_cast<double>(sigma_c)) * J(n_o);
  return speed * (attract + circulate);
}

Vec2 blendContour(const Vec2 & Phi, const Vec2 & Psi, double theta) noexcept
{
  // The end points are returned exactly, so the latched band (theta = 1) reproduces the
  // no-contour command to rounding (oracle O32) and theta = 0 gives Psi itself.
  if (theta >= 1.0) {
    return Phi;
  }
  if (theta <= 0.0) {
    return Psi;
  }
  const double phi_norm = Phi.norm();
  const Vec2 mix = theta * Phi + (1.0 - theta) * Psi;
  const double mix_norm = mix.norm();
  if (!(mix_norm > 0.0) || mix_norm < kBlendDegenerateFraction * phi_norm) {
    return Psi;  // degenerate blend [Spec]: cannot happen while n_o^T Phi < 0 and D_o > lambda
  }
  return (phi_norm / mix_norm) * mix;
}

bool contourStartCondition(
  double D_o, const Vec2 & n_o, const Vec2 & Phi_nominal, double D_in_prime) noexcept
{
  return D_o <= D_in_prime && n_o.dot(Phi_nominal) < 0.0;
}

bool contourExitCondition(
  double D_o, const Vec2 & n_o, const Vec2 & Phi_nominal, double D_in_prime) noexcept
{
  return n_o.dot(Phi_nominal) >= kContourExitFraction * Phi_nominal.norm() ||
         D_o > D_in_prime + kHc;
}

GateUpdate updateGate(
  bool was_open, double clear_s, bool blocked_now, double dt, double release_time) noexcept
{
  GateUpdate update;
  if (blocked_now) {
    update.open = true;
    update.clear_s = 0.0;
    return update;
  }
  if (!was_open) {
    return update;  // closed and clear: nothing to time
  }
  const double step = (dt > 0.0 && std::isfinite(dt)) ? dt : 0.0;
  const double clear = (std::isfinite(clear_s) ? std::max(clear_s, 0.0) : 0.0) + step;
  if (clear >= release_time) {
    return update;  // released: closed, timer cleared
  }
  update.open = true;
  update.clear_s = clear;
  return update;
}

int markBlockedSamples(
  Span<const Vec2> samples_grid, const EuclideanDistanceTransform & edt_all, double w_lat,
  double tolerance, Span<std::uint8_t> blocked) noexcept
{
  const double threshold = w_lat - tolerance;
  const std::size_t count = std::min(samples_grid.size(), blocked.size());
  int first = -1;
  for (std::size_t i = 0; i < count; ++i) {
    const ObstacleQuery query = edt_all.query(samples_grid[i]);
    const bool is_blocked = query.valid && query.D_o < threshold;
    blocked[i] = is_blocked ? 1U : 0U;
    if (is_blocked && first < 0) {
      first = static_cast<int>(i);
    }
  }
  return first;
}

double sideProbeHorizon(
  double horizon, double D_in_prime, double lambda, double v_r, double blocking_extent) noexcept
{
  if (!(v_r > 0.0)) {
    return horizon;
  }
  const double extent = blocking_extent > 0.0 ? blocking_extent : 0.0;
  return std::max(horizon, (D_in_prime - lambda + kPi * lambda + extent) / v_r);
}

int chooseContourSide(
  const SideProbeResult & plus, const SideProbeResult & minus, double resolution,
  double stop_margin, double min_free, double jn_dot_phi, double phi_norm) noexcept
{
  if (!(plus.s_free >= min_free) && !(minus.s_free >= min_free)) {
    return 0;  // neither side leaves room to brake from v_r: no episode
  }
  // 1. collision-free; 2. exit reached.
  Preference preference = preferTrue(plus.collision_free, minus.collision_free);
  if (preference == Preference::kTie) {
    preference = preferTrue(plus.exit_reached, minus.exit_reached);
  }
  // 3. larger saturated clearance, equal within res/2.
  if (preference == Preference::kTie) {
    const double cap = 2.0 * stop_margin;
    const double c_plus = std::min(plus.clearance, cap);
    const double c_minus = std::min(minus.clearance, cap);
    if (!(std::abs(c_plus - c_minus) <= 0.5 * resolution)) {
      preference = preferLarger(c_plus, c_minus);
    }
  }
  // 4. shorter path to the exit.
  if (preference == Preference::kTie) {
    preference = preferLarger(-plus.path_to_exit, -minus.path_to_exit);
  }
  // 5. larger free arc length.
  if (preference == Preference::kTie) {
    preference = preferLarger(plus.s_free, minus.s_free);
  }
  if (preference != Preference::kTie) {
    return static_cast<int>(preference);
  }
  // 6. the side (3.67) would pick, when it is well defined; 7. +1.
  if (std::abs(jn_dot_phi) >= kSideTieFraction * phi_norm && jn_dot_phi != 0.0) {
    return jn_dot_phi > 0.0 ? 1 : -1;
  }
  return 1;
}

bool contourWatchdogTripped(
  double travelled_m, double elapsed_s, const ObstacleConfig & obstacles) noexcept
{
  return travelled_m > obstacles.obstacle_max_contour_length ||
         elapsed_s > obstacles.obstacle_max_contour_time;
}

}  // namespace vectorfield_core
