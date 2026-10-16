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
/// @brief Pure building blocks of the obstacle contour: blend weight, contour field, blend,
/// start and exit predicates, gate, side ranking and watchdog (plan §3.2 ObstacleContour;
/// math_spec §6.2–§6.4).
///
/// Implemented in src/contour.cpp (M1 wave 3, tracker). The orchestration (K identity, side-probe
/// rollouts, episode latch) lives in the Tracker, which composes these functions. All functions
/// are `noexcept` and allocation-free.

#ifndef VECTORFIELD_CORE__CONTOUR_HPP_
#define VECTORFIELD_CORE__CONTOUR_HPP_

#include <cstdint>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/occupancy.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Blend weight (math_spec §6.4): \f$\theta = \operatorname{clamp}((D_o -
/// D_{\mathrm{in}})/(D'_{\mathrm{in}} - D_{\mathrm{in}}), 0, 1)\f$; when \f$D'_{\mathrm{in}} =
/// D_{\mathrm{in}}\f$, 0 below \f$D_{\mathrm{in}}\f$ and 1 otherwise. In the latched band
/// \f$(D'_{\mathrm{in}}, D'_{\mathrm{in}} + h_c]\f$ this gives 1, so \f$F = \Phi\f$.
double contourBlendWeight(double D_o, double D_in, double D_in_prime) noexcept;

/// @brief Contour field with a latched side (math_spec §6.2 with §6.4):
/// \f$\Psi = \mathrm{speed}\,(\Psi_G + \Psi_H)\f$, \f$\Psi_G = -G(D_\lambda)\operatorname{sgn}(D_o -
/// \lambda)n_o\f$, \f$\Psi_H = H(D_\lambda)\,\sigma_c J n_o\f$, \f$D_\lambda = \lvert D_o -
/// \lambda\rvert\f$. \f$\Psi_T = 0\f$ (costmap obstacles carry no velocity).
/// @param n_o Obstacle normal (unit).
/// @param D_o Obstacle distance.
/// @param lambda \f$\lambda\f$.
/// @param sigma_c Latched side ±1.
/// @param gain The field's gain.
/// @param speed \f$\lVert\Phi_\gamma\rVert\f$ (the speed of the shaped path field).
Vec2 contourField(
  const Vec2 & n_o, double D_o, double lambda, int sigma_c, const GainFunction & gain,
  double speed) noexcept;

/// @brief Blend (math_spec §6.4): \f$F = \lVert\Phi\rVert\operatorname{unit}(\theta\Phi +
/// (1 - \theta)\Psi)\f$, or \f$\Psi\f$ when \f$\lVert\theta\Phi + (1-\theta)\Psi\rVert <
/// 10^{-9}\lVert\Phi\rVert\f$ [Spec].
/// @param Phi Shaped path field \f$\Phi_\gamma\f$.
/// @param Psi contourField().
/// @param theta contourBlendWeight().
Vec2 blendContour(const Vec2 & Phi, const Vec2 & Psi, double theta) noexcept;

/// @brief Start predicate 2 of math_spec §6.4: \f$D_o \le D'_{\mathrm{in}}\f$ and \f$n_o^\top\Phi
/// < 0\f$, measured to K at the law's point with the nominal \f$\Phi\f$ (Appendix C item 8).
bool contourStartCondition(
  double D_o, const Vec2 & n_o, const Vec2 & Phi_nominal, double D_in_prime) noexcept;

/// @brief Field part of the exit test (math_spec §6.4): \f$n_o^\top\Phi \ge
/// 0.05\lVert\Phi\rVert\f$ or \f$D_o > D'_{\mathrm{in}} + h_c\f$ (gate closing, K lost and the
/// goal veto are checked by the Tracker).
bool contourExitCondition(
  double D_o, const Vec2 & n_o, const Vec2 & Phi_nominal, double D_in_prime) noexcept;

/// @brief Gate state after one cycle.
struct GateUpdate
{
  /// The gate is open.
  bool open{false};
  /// Time the path has stayed clear (s).
  double clear_s{0.0};
};

/// @brief Gate timer (math_spec §6.4): the gate is open in a cycle with a blocked sample; the
/// clear timer resets on any blocked sample and accumulates \f$\Delta t\f$ otherwise; an open
/// gate closes once the timer reaches `release_time`.
GateUpdate updateGate(
  bool was_open, double clear_s, bool blocked_now, double dt, double release_time) noexcept;

/// @brief Marks the blocked gate samples (math_spec §6.4): a sample is blocked iff
/// \f$D_o^{\mathrm{all}}(r(s)) < w_{\mathrm{lat}} - \f$ `obstacle_gate_tolerance`.
/// @param samples_grid Path samples \f$r(s)\f$ in the grid frame.
/// @param edt_all \f$\mathrm{EDT}_{\mathrm{all}}\f$.
/// @param w_lat \f$w_{\mathrm{lat}}\f$.
/// @param tolerance `obstacle_gate_tolerance`.
/// @param blocked Output flags, one per sample (1 = blocked); at least samples_grid.size().
/// @return Index of the first blocked sample, or −1.
int markBlockedSamples(
  Span<const Vec2> samples_grid, const EuclideanDistanceTransform & edt_all, double w_lat,
  double tolerance, Span<std::uint8_t> blocked) noexcept;

/// @brief Side-probe horizon \f$T_{\mathrm{side}} = \max(T_h, (D'_{\mathrm{in}} - \lambda +
/// \pi\lambda + L_B)/v_r)\f$ (math_spec §6.4; TB3 with a point obstacle ≈ 4.6 s): the radial
/// approach, half a circulation and the station extent \f$L_B\f$ of the blocking set (0 for a
/// point obstacle and in `always` mode), so extended obstacles are passed within the probe.
double sideProbeHorizon(
  double horizon, double D_in_prime, double lambda, double v_r,
  double blocking_extent = 0.0) noexcept;

/// @brief Outcome of one side-probe rollout (math_spec §6.4).
struct SideProbeResult
{
  /// No collision.
  bool collision_free{false};
  /// The episode-exit test was reached.
  bool exit_reached{false};
  /// \f$c_{\mathrm{cl}} = \min \mathrm{EDT}_{\mathrm{all}}(p) - w_{\mathrm{lat}}\f$ over the
  /// rollout's base positions (m).
  double clearance{kInf};
  /// Base path length to the exit (m); \f$+\infty\f$ if not reached.
  double path_to_exit{kInf};
  /// Free arc length of the rollout (m).
  double s_free{kInf};
};

/// @brief Ranks the two sides (math_spec §6.4): collision-free; exit reached; larger saturated
/// clearance \f$\min(c_{\mathrm{cl}}, 2m_s)\f$ (equal within res/2); shorter path to the exit;
/// larger \f$s_{\mathrm{free}}\f$; \f$\operatorname{sgn}((Jn_o)^\top\Phi)\f$ if
/// \f$\lvert(Jn_o)^\top\Phi\rvert \ge 0.1\lVert\Phi\rVert\f$; +1. Returns 0 (no episode) when
/// neither side has \f$s_{\mathrm{free}} \ge v_r^2/(2a_{\mathrm{dec}}) + m_s\f$.
/// @param plus Rollout with \f$\sigma_c = +1\f$.
/// @param minus Rollout with \f$\sigma_c = -1\f$.
/// @param resolution Grid resolution.
/// @param stop_margin \f$m_s\f$.
/// @param min_free \f$v_r^2/(2a_{\mathrm{dec}}) + m_s\f$.
/// @param jn_dot_phi \f$(Jn_o)^\top\Phi\f$ at the start.
/// @param phi_norm \f$\lVert\Phi\rVert\f$ at the start.
/// @return +1, −1 or 0.
int chooseContourSide(
  const SideProbeResult & plus, const SideProbeResult & minus, double resolution,
  double stop_margin, double min_free, double jn_dot_phi, double phi_norm) noexcept;

/// @brief Watchdog (math_spec §6.4): travelled_m > `obstacle_max_contour_length` or elapsed_s >
/// `obstacle_max_contour_time`.
bool contourWatchdogTripped(
  double travelled_m, double elapsed_s, const ObstacleConfig & obstacles) noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__CONTOUR_HPP_
