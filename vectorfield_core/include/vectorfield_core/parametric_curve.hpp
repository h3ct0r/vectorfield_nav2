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
/// @brief Parametric curves \f$r(u, t)\f$: golden-section local search, global M-sample stage,
/// adoption rule, arc length (math_spec §2.5–§2.6).
///
/// Implemented in src/parametric_curve.cpp. The concrete
/// subclasses (built-in shapes, ExpressionCurve) are internal to the library and reached through
/// makeCurve() (curve_spec.hpp).

#ifndef VECTORFIELD_CORE__PARAMETRIC_CURVE_HPP_
#define VECTORFIELD_CORE__PARAMETRIC_CURVE_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @name Numerics of parametric curves (math_spec §2.5–§2.6, Appendix C item 2)
/// @{
/// Derivative step \f$h_u = 6\cdot10^{-6}(u_f - u_i)\f$ (central differences; second-order
/// one-sided at the ends of an open domain).
inline constexpr double kParamDerivativeStepFraction = 6e-6;
/// Time-derivative step \f$\delta_t = 10^{-4}\f$ s for \f$\partial_t r\f$ (central difference).
inline constexpr double kTimeDerivativeStep = 1e-4;
/// Golden-section stop: bracket width \f$\le 10^{-10}\max(1, u_f - u_i)\f$ ...
inline constexpr double kGoldenSectionTolerance = 1e-10;
/// ... or after 100 iterations.
inline constexpr int kGoldenSectionMaxIterations = 100;
/// Gauss–Legendre points per panel (arc length).
inline constexpr int kGaussLegendrePoints = 5;
/// Time-varying arc length uses \f$\lceil 64\,\lvert\Delta u\rvert/(u_f - u_i)\rceil\f$ panels.
inline constexpr int kArcLengthPanelsPerDomain = 64;
/// Regularity: reject when \f$\min\lVert r'\rVert < 10^{-3}\operatorname{mean}\lVert r'\rVert\f$.
inline constexpr double kRegularitySpeedRatio = 1e-3;
/// Regularity: reject when adjacent sample tangents differ by more than 90°.
inline constexpr double kRegularityMaxTurn = kHalfPi;
/// Closure: reject a "closed" expression whose end tangents differ by more than 1°.
inline constexpr double kClosureMaxTangentAngle = kPi / 180.0;
/// @}

/// @brief A curve given by a parametrisation \f$r(u, t)\f$, \f$u \in [u_i, u_f]\f$ (math_spec
/// §2.5, sec-parametric).
///
/// Contract of the search (closestPoint()):
/// - local stage: golden section on \f$[u_{\mathrm{prev}} - \Delta u_b, u_{\mathrm{prev}} +
///   \Delta u_f]\f$ (wrapping only when closed); static curves take \f$\Delta u_{b,f} =
///   \lvert u(s_{\mathrm{prev}} \pm w/2) - u_{\mathrm{prev}}\rvert\f$ from the u↔s table,
///   time-varying curves \f$\Delta u = w/(2\lVert\partial r/\partial u(u_{\mathrm{prev}},
///   t)\rVert)\f$; a result on a bracket edge that is not a domain end moves the bracket by
///   \f$w/2\f$ and repeats, at most one lap;
/// - global stage: M samples equally spaced in u (closed: \f$[u_i, u_f)\f$; open: inclusive)
///   plus the two rays, argmin, golden section on \f$[u_{k^\star-1}, u_{k^\star+1}]\f$; it runs
///   at initialisation (`!hint.valid`, adopted unconditionally), when the local result was on an
///   edge, and when \f$D_{\mathrm{loc}} > D_{\mathrm{trig}}\f$; adoption by
///   adoptGlobalCandidate() sets Status::kAmbiguousClosest;
/// - open curves have straight rays \f$r(u_f) + \tau T(u_f)\f$, \f$\tau \ge 0\f$ and
///   \f$r(u_i) + \tau T(u_i)\f$, \f$\tau \le 0\f$; \f$r\f$ is never evaluated outside
///   \f$[u_i, u_f]\f$;
/// - \f$T = \operatorname{unit}(\partial r/\partial u)\f$; \f$n =
///   \operatorname{unit}((I - TT^\top)(x - x^\star))\f$, \f$D = \lVert x - x^\star\rVert\f$, the
///   on-curve rule below \f$\varepsilon_D\f$ (the re-orthogonalisation keeps \f$n^\top T = 0\f$);
/// - a speed below the construction-time floor sets Status::kNonFinite;
/// - `s_star` is the arc length at time t: from the u↔s table (5-point Gauss–Legendre per
///   interval) for static curves, by fixed-panel Gauss–Legendre without allocation for
///   time-varying curves; `u_star` is \f$u^\star\f$ (the hint parameter).
///
/// Subclasses implement r() (and may override drdu()/drdt() with analytic derivatives), then
/// call initialize() at the end of their constructor, when virtual dispatch reaches them.
class ParametricCurve : public Curve
{
public:
  ~ParametricCurve() override;

  /// @brief Lower end \f$u_i\f$ of the parameter domain.
  double uMin() const noexcept;
  /// @brief Upper end \f$u_f\f$ of the parameter domain.
  double uMax() const noexcept;
  /// @brief The options the curve was built with (search window, global stage).
  const CurveOptions & options() const noexcept;

  /// @brief Arc length \f$\int_{u_0}^{u_1}\lVert\partial r/\partial u\rVert\,du\f$ at time t
  /// (signed; closed curves do not wrap), by the static table when available, else fixed-panel
  /// Gauss–Legendre. Allocation-free.
  double arcLength(double u0, double u1, double t) const noexcept;
  /// @brief Station \f$s(u)\f$ from the curve's start at time t.
  double stationAt(double u, double t) const noexcept;
  /// @brief Parameter \f$u(s)\f$ at station s within the domain (s clamped to \f$[0, L]\f$ for
  /// open curves, wrapped for closed curves).
  double parameterAt(double s, double t) const noexcept;
  /// @brief \f$\lVert\partial r/\partial u\rVert\f$ floor below which the curve is irregular
  /// (\f$10^{-3}\f$ times the mean speed at \f$t_0\f$).
  double speedFloor() const noexcept;

  /// @brief Records the spec this curve was built from (makeCurve only); returned by spec().
  void setSpec(std::shared_ptr<const CurveSpec> spec) noexcept;

  ClosestPoint closestPoint(
    const Vec2 & x, const EvalContext & context, const CurveHint & hint) const noexcept override;
  bool isClosed() const noexcept override;
  bool isTimeVarying() const noexcept override;
  double length(double t) const noexcept override;
  /// @brief \f$\partial_t r(u^\star, t)\f$ via drdt(); zero on a ray or for static curves.
  Vec2 velocity(const ClosestPoint & cp, double t) const noexcept override;
  Vec2 pointAt(double s, double t) const noexcept override;
  Vec2 tangentAt(double s, double t) const noexcept override;
  std::size_t sampleStations(
    double s0, double s1, double ds, double t, Span<Vec2> out) const noexcept override;
  const CurveSpec * spec() const noexcept override;

  /// @brief \f$r(u, t)\f$ in the curve frame. Must be finite inside the domain; may return NaN
  /// on an evaluation failure (reported as Status::kNonFinite).
  virtual Vec2 r(double u, double t) const noexcept = 0;
  /// @brief \f$\partial r/\partial u\f$. Default: central differences with \f$h_u\f$, the
  /// second-order one-sided formula at the ends of an open domain, wrapping when closed.
  virtual Vec2 drdu(double u, double t) const noexcept;
  /// @brief \f$\partial r/\partial t\f$. Default: zero when static, else a central difference
  /// with \f$\delta_t\f$ = kTimeDerivativeStep at fixed u.
  virtual Vec2 drdt(double u, double t) const noexcept;

protected:
  /// @brief Stores the domain and options; no evaluation happens here.
  /// @param u_min \f$u_i\f$.
  /// @param u_max \f$u_f\f$ (> u_min).
  /// @param closed True for a closed curve.
  /// @param time_varying True iff r depends on t.
  /// @param options Search window and global-stage parameters.
  /// @throws ConfigError if the domain is empty or non-finite, or the options are invalid
  /// (validateCurveOptions()).
  ParametricCurve(
    double u_min, double u_max, bool closed, bool time_varying, const CurveOptions & options);

  ParametricCurve(const ParametricCurve &) = default;
  ParametricCurve(ParametricCurve &&) = default;
  ParametricCurve & operator=(const ParametricCurve &) = default;
  ParametricCurve & operator=(ParametricCurve &&) = default;

  /// @brief Builds the global-stage samples and, for static curves, the u↔s table at \f$t_0\f$,
  /// and runs the regularity and closure checks of math_spec §2.6. Call once, at the end of the
  /// most-derived constructor.
  /// @throws CurveError (code kInvalidCurve) on an irregular or non-closing curve.
  void initialize(double t0);

private:
  double u_min_{0.0};
  double u_max_{1.0};
  bool closed_{false};
  bool time_varying_{false};
  CurveOptions options_;
  double speed_floor_{0.0};
  double length_{0.0};
  std::vector<double> table_u_;
  std::vector<double> table_s_;
  // r(u_k, t0) at the M global-stage samples, interleaved x0, y0, x1, y1, ... (static curves
  // only; empty when time-varying).
  std::vector<double> samples_;
  std::shared_ptr<const CurveSpec> spec_;
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__PARAMETRIC_CURVE_HPP_
