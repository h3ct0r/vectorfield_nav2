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
/// @brief Planar types, shared constants and angle helpers of vectorfield_core.
///
/// Conventions follow math_spec §0 (sec-math-scope): \f$J\f$ is the rotation by \f$+\pi/2\f$,
/// \f$R(\psi) = \cos\psi\, I + \sin\psi\, J\f$, \f$\mathbf h(\psi) = (\cos\psi, \sin\psi)^\top\f$,
/// \f$a \times b = a_x b_y - a_y b_x\f$, \f$\operatorname{wrap}(a) \in (-\pi, \pi]\f$,
/// \f$\operatorname{clamp}(a, l, u) = \min(u, \max(l, a))\f$ and \f$\operatorname{sgn}(0) = 0\f$.
/// Units are SI and angles are radians. Everything is 2D; the 3D core (namespace d3)
/// has its own types.
///
/// All types here are fixed-size and trivially destructible, so they may appear in
/// `TrackerState`. Eigen types are not trivially copyable: copy member-wise, never `memcpy`.

#ifndef VECTORFIELD_CORE__TYPES_HPP_
#define VECTORFIELD_CORE__TYPES_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

namespace vectorfield_core
{

/// @brief Planar vector (metres or metres per second, by context).
using Vec2 = Eigen::Vector2d;
/// @brief 2×2 matrix.
using Mat2 = Eigen::Matrix2d;

/// @name Numeric constants
/// @{

/// \f$\pi\f$.
inline constexpr double kPi = 3.141592653589793238462643383279502884;
/// \f$2\pi\f$.
inline constexpr double kTwoPi = 2.0 * kPi;
/// \f$\pi/2\f$.
inline constexpr double kHalfPi = 0.5 * kPi;
/// Positive infinity (e.g. \f$s_{\mathrm{free}}\f$ of a collision-free rollout).
inline constexpr double kInf = std::numeric_limits<double>::infinity();
/// Quiet NaN (e.g. \f$\sigma\f$ on a closed curve).
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
/// @}

/// @name Tolerances and fixed thresholds of the math specification
/// Every value is fixed by math_spec §1 (sec-math-symbols) or the section cited; none is a
/// parameter.
/// @{

/// \f$\varepsilon_D\f$ = 1e-9 m: below this distance a point is on the curve, \f$n := 0\f$
/// (math_spec §2.3, sec-wedge).
inline constexpr double kEpsD = 1e-9;
/// \f$\varepsilon_{\mathrm{hp}}\f$ = 1e-3: hairpin threshold on \f$\lVert t_{v-1} + t_v\rVert\f$
/// (math_spec §2.3, sec-wedge).
inline constexpr double kEpsHp = 1e-3;
/// \f$h_c\f$ = 0.05 m: contour-exit hysteresis band above \f$D'_{\mathrm{in}}\f$
/// (math_spec §6.4, sec-contour-mods).
inline constexpr double kHc = 0.05;
/// Closest-point tie tolerance, 1e-9 m (math_spec §2.4, sec-search; Appendix C item 1).
inline constexpr double kTieTolerance = 1e-9;
/// \f$v_{\mathrm{stop}}\f$ = 0.02 m/s for the kBlocked set test; Ackermann uses
/// \f$v_{\mathrm{ack}}\f$ instead (math_spec §10.4, sec-collision-blocked).
inline constexpr double kVStop = 0.02;
/// Dedupe spacing, 1 mm: consecutive vertices closer than this are merged
/// (math_spec §2.8, sec-conditioning).
inline constexpr double kDedupeDistance = 1e-3;
/// Terminal mode is entered when \f$\lVert g - g_c\rVert\f$ < 0.05 m (math_spec §5.5,
/// sec-terminal-mode).
inline constexpr double kTerminalEnterDistance = 0.05;
/// Terminal mode is left when \f$\lVert g - g_c\rVert\f$ > 0.25 m (math_spec §5.5,
/// sec-terminal-mode).
inline constexpr double kTerminalLeaveDistance = 0.25;
/// The plugin window "ends at the goal" when \f$\lVert W_{\mathrm{back}} -
/// \mathrm{global\_goal}\rVert\f$ < 0.05 m (math_spec §2.8, sec-conditioning).
inline constexpr double kWindowAtGoalDistance = 0.05;
/// Fraction of \f$v_r\f$ below which a field direction is not used: \f$\psi_d\f$ is held,
/// ROTATE_TO_FIELD is not entered and probes return NaN (math_spec §7.2, §7.4, §9.2).
inline constexpr double kHoldSpeedFraction = 0.05;
/// yaw_tol at or below 1e-6 rad means the goal yaw is unchecked (math_spec §9.1,
/// sec-phases-predicates).
inline constexpr double kYawTolUnchecked = 1e-6;
/// A cancel completes once \f$\lvert v\rvert, \lvert v_y\rvert\f$ < 0.01 m/s ...
/// (math_spec §9.2, sec-phases-table).
inline constexpr double kStopLinearSpeed = 0.01;
/// ... and \f$\lvert\omega\rvert\f$ < 0.01 rad/s (math_spec §9.2, sec-phases-table).
inline constexpr double kStopAngularSpeed = 0.01;
/// Contour exit when \f$n_o^\top\Phi \ge 0.05\lVert\Phi\rVert\f$ (math_spec §6.4,
/// sec-contour-mods).
inline constexpr double kContourExitFraction = 0.05;
/// Side-probe rank 6 applies when \f$\lvert(J n_o)^\top\Phi\rvert \ge 0.1\lVert\Phi\rVert\f$
/// (math_spec §6.4, sec-contour-mods).
inline constexpr double kSideTieFraction = 0.1;
/// The contour blend degenerates when \f$\lVert\theta\Phi + (1-\theta)\Psi\rVert <
/// 10^{-9}\lVert\Phi\rVert\f$ (math_spec §6.4, sec-contour-mods).
inline constexpr double kBlendDegenerateFraction = 1e-9;
/// Minimum heading-tracking probe offset: \f$\delta = \max(0.02, \lvert v\rvert\Delta t,
/// \min(\bar\ell, 0.10))\f$ (math_spec §7.4, sec-laws-ht).
inline constexpr double kProbeMinOffset = 0.02;
/// Cap on the vertex-spacing term \f$\min(\bar\ell, 0.10)\f$ of the heading-tracking probe offset
/// (math_spec §7.4; Curve::vertexSpacing()).
inline constexpr double kProbeSpacingCap = 0.10;
/// Relative slack of the heading-tracking jump threshold \f$1.05\,(2\delta + 2\max(D_+, D_-))\f$:
/// on a polyline the arc between the probes' feet exceeds their chord by up to
/// \f$1/\cos(\Theta/2)\f$, \f$\Theta\f$ the turn between them (math_spec §7.4).
inline constexpr double kProbeJumpSlack = 1.05;
/// Lower bound of the chord \f$\ell_c = \max(0.10, 2\,\mathrm{res})\f$ (math_spec §1).
inline constexpr double kChordMinLength = 0.10;
/// Lower bound of the kBlocked release band \f$h_b = \max(2\,\mathrm{res}, 0.05)\f$
/// (math_spec §10.4, sec-collision-blocked).
inline constexpr double kBlockedBandMin = 0.05;
/// One-pose rule: \f$[p_{\mathrm{last}} - 0.01\,\mathbf h(\psi_{\mathrm{last}}),
/// p_{\mathrm{last}}]\f$ (math_spec §2.8, sec-conditioning).
inline constexpr double kOnePoseOffset = 0.01;
/// @}

/// @brief Pose of `base_link` in a planar frame: position and yaw.
struct Pose2
{
  /// Position \f$p\f$ (m).
  Vec2 p{Vec2::Zero()};
  /// Yaw \f$\psi\f$ (rad). Not required to be wrapped.
  double yaw{0.0};
};

/// @brief Planar body twist at `base_link`: \f$(v_x, v_y, \omega)\f$ in the body frame.
///
/// Differential, skid-steer and Ackermann commands have \f$v_y = 0\f$ (math_spec §7.5: the
/// Ackermann twist is \f$(v, 0, \omega)\f$).
struct Twist2
{
  /// Longitudinal velocity (m/s).
  double vx{0.0};
  /// Lateral velocity (m/s).
  double vy{0.0};
  /// Yaw rate (rad/s).
  double wz{0.0};
};

/// @brief True iff x is finite (neither infinite nor NaN).
inline bool isFinite(double x) noexcept { return std::isfinite(x); }
/// @brief True iff both components are finite.
inline bool isFinite(const Vec2 & v) noexcept
{
  return std::isfinite(v.x()) && std::isfinite(v.y());
}
/// @brief True iff position and yaw are finite.
inline bool isFinite(const Pose2 & pose) noexcept
{
  return isFinite(pose.p) && std::isfinite(pose.yaw);
}
/// @brief True iff every component is finite.
inline bool isFinite(const Twist2 & u) noexcept
{
  return std::isfinite(u.vx) && std::isfinite(u.vy) && std::isfinite(u.wz);
}

/// @brief Wraps an angle into \f$(-\pi, \pi]\f$ (math_spec §0).
///
/// Exact for every finite input (uses `std::remainder`); \f$-\pi\f$ maps to \f$+\pi\f$.
/// @return The wrapped angle; NaN for a non-finite input.
inline double wrapAngle(double angle) noexcept
{
  double r = std::remainder(angle, kTwoPi);
  if (r <= -kPi) {
    r += kTwoPi;
  }
  return r;
}

/// @brief Wraps x into \f$(-P/2, P/2]\f$ for a period \f$P > 0\f$.
///
/// Used for arc-length differences on closed curves (math_spec §2.2, sec-polyline: "every
/// arc-length difference is taken modulo L into (−L/2, L/2]") and for lap counting
/// (math_spec §9.3, with \f$P = 1\f$).
/// @return The wrapped value; x itself when the period is not positive and finite.
inline double wrapPeriodic(double x, double period) noexcept
{
  if (!(period > 0.0) || !std::isfinite(period)) {
    return x;
  }
  double r = std::remainder(x, period);
  if (r <= -0.5 * period) {
    r += period;
  }
  return r;
}

/// @brief \f$\operatorname{clamp}(x, l, u) = \min(u, \max(l, x))\f$ (math_spec §0).
///
/// Unlike `std::clamp` it is defined for \f$l > u\f$ (returns u) and it propagates NaN, so the
/// limiter's non-finite guard sees it.
inline double clamp(double x, double lo, double hi) noexcept
{
  if (std::isnan(x)) {
    return x;
  }
  return std::min(hi, std::max(lo, x));
}

/// @brief Sign with \f$\operatorname{sgn}(0) = 0\f$ (math_spec §0); NaN gives 0.
inline double sgn(double x) noexcept
{
  if (x > 0.0) {
    return 1.0;
  }
  if (x < 0.0) {
    return -1.0;
  }
  return 0.0;
}

/// @brief \f$J v\f$: rotation of v by \f$+\pi/2\f$, \f$(v_x, v_y) \mapsto (-v_y, v_x)\f$.
inline Vec2 J(const Vec2 & v) noexcept { return Vec2(-v.y(), v.x()); }

/// @brief Heading unit vector \f$\mathbf h(\psi) = (\cos\psi, \sin\psi)^\top\f$.
inline Vec2 heading(double psi) noexcept { return Vec2(std::cos(psi), std::sin(psi)); }

/// @brief Planar cross product \f$a \times b = a_x b_y - a_y b_x\f$.
inline double cross2(const Vec2 & a, const Vec2 & b) noexcept
{
  return a.x() * b.y() - a.y() * b.x();
}

/// @brief Angle of a vector, \f$\angle v = \operatorname{atan2}(v_y, v_x)\f$ in
/// \f$[-\pi, \pi]\f$.
inline double angleOf(const Vec2 & v) noexcept { return std::atan2(v.y(), v.x()); }

/// @brief Rotation matrix \f$R(\psi) = \cos\psi\, I + \sin\psi\, J\f$.
inline Mat2 rotation(double psi) noexcept
{
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  Mat2 r;
  r << c, -s, s, c;
  return r;
}

/// @brief \f$R(\psi)\, v\f$ without forming the matrix.
inline Vec2 rotate(double psi, const Vec2 & v) noexcept
{
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  return Vec2(c * v.x() - s * v.y(), s * v.x() + c * v.y());
}

/// @brief \f$\operatorname{unit}(v) = v/\lVert v\rVert\f$, or `fallback` when \f$\lVert v\rVert\f$
/// is not positive and finite.
inline Vec2 unitOr(const Vec2 & v, const Vec2 & fallback) noexcept
{
  const double norm = v.norm();
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return fallback;
  }
  return v / norm;
}

/// @brief Chord length \f$\ell_c = \max(0.10\ \mathrm{m}, 2\,\mathrm{res})\f$ for cusp decimation
/// and tail conditioning (math_spec §1, §2.8).
inline double chordLength(double resolution) noexcept
{
  return std::max(kChordMinLength, 2.0 * resolution);
}

/// @brief kBlocked release band \f$h_b = \max(2\,\mathrm{res}, 0.05\ \mathrm{m})\f$
/// (math_spec §10.4, sec-collision-blocked).
inline double blockedReleaseBand(double resolution) noexcept
{
  return std::max(2.0 * resolution, kBlockedBandMin);
}

/// @brief Rigid planar transform: a rotation by `yaw()` followed by a translation.
///
/// Naming convention used throughout the core: `T_a_b` maps coordinates in frame b into frame a,
/// \f$x_a = R\,x_b + t\f$. So `CurveFrame::T_eval_curve` maps curve-frame points into the
/// evaluation frame, and `ObstacleMap::T_grid_eval` maps evaluation-frame points into the grid
/// frame. The cosine and sine are cached; the type is trivially destructible.
class Isometry2
{
public:
  /// @brief The identity transform.
  Isometry2() noexcept = default;

  /// @brief Rotation by `yaw` (rad), then translation by `translation`.
  Isometry2(double yaw, const Vec2 & translation) noexcept
  : yaw_(wrapAngle(yaw)), cos_(std::cos(yaw_)), sin_(std::sin(yaw_)), t_(translation)
  {
  }

  /// @brief The identity transform.
  static Isometry2 identity() noexcept { return Isometry2(); }

  /// @brief The transform whose child frame is the given pose: body coordinates → parent.
  static Isometry2 fromPose(const Pose2 & pose) noexcept { return Isometry2(pose.yaw, pose.p); }

  /// @brief The pose of the child frame in the parent frame.
  Pose2 toPose() const noexcept { return Pose2{t_, yaw_}; }

  /// @brief Rotation angle in \f$(-\pi, \pi]\f$.
  double yaw() const noexcept { return yaw_; }
  /// @brief Translation.
  const Vec2 & translation() const noexcept { return t_; }
  /// @brief Rotation matrix \f$R\f$.
  Mat2 rotation() const noexcept
  {
    Mat2 r;
    r << cos_, -sin_, sin_, cos_;
    return r;
  }

  /// @brief \f$R\,x + t\f$ (maps a point).
  Vec2 apply(const Vec2 & x) const noexcept { return rotate(x) + t_; }
  /// @brief \f$R^\top (x - t)\f$ (maps a point back), without forming the inverse.
  Vec2 applyInverse(const Vec2 & x) const noexcept { return rotateInverse(x - t_); }
  /// @brief \f$R\,v\f$ (maps a free vector: tangents, normals, velocities).
  Vec2 rotate(const Vec2 & v) const noexcept
  {
    return Vec2(cos_ * v.x() - sin_ * v.y(), sin_ * v.x() + cos_ * v.y());
  }
  /// @brief \f$R^\top v\f$.
  Vec2 rotateInverse(const Vec2 & v) const noexcept
  {
    return Vec2(cos_ * v.x() + sin_ * v.y(), -sin_ * v.x() + cos_ * v.y());
  }
  /// @brief Maps a pose: position by apply(), yaw by adding yaw() (wrapped).
  Pose2 apply(const Pose2 & pose) const noexcept
  {
    return Pose2{apply(pose.p), wrapAngle(pose.yaw + yaw_)};
  }
  /// @brief Maps a pose back (inverse of apply(const Pose2&)).
  Pose2 applyInverse(const Pose2 & pose) const noexcept
  {
    return Pose2{applyInverse(pose.p), wrapAngle(pose.yaw - yaw_)};
  }

  /// @brief The inverse transform.
  Isometry2 inverse() const noexcept { return Isometry2(-yaw_, -rotateInverse(t_)); }

  /// @brief Composition: `(a * b).apply(x) == a.apply(b.apply(x))`.
  Isometry2 operator*(const Isometry2 & other) const noexcept
  {
    return Isometry2(yaw_ + other.yaw_, apply(other.t_));
  }

private:
  double yaw_{0.0};
  double cos_{1.0};
  double sin_{0.0};
  Vec2 t_{Vec2::Zero()};
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__TYPES_HPP_
