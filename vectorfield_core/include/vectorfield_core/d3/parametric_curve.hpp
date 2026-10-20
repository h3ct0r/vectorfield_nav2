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
/// @brief 3D core (experimental): parametric curves \f$r(u, t) \in \mathbb R^3\f$
/// (math_spec §2.5–§2.6, §14.1).

#ifndef VECTORFIELD_CORE__D3__PARAMETRIC_CURVE_HPP_
#define VECTORFIELD_CORE__D3__PARAMETRIC_CURVE_HPP_

#include <memory>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/d3/types.hpp"

namespace vectorfield_core::d3
{

/// @brief Newton polish steps after the golden-section stage (math_spec §14.1).
inline constexpr int kNewtonPolishSteps = 3;

/// @brief A curve given by a parametrisation \f$r(u, t)\f$, \f$u \in [u_i, u_f]\f$.
///
/// The search is that of the planar ParametricCurve (parametric_curve.hpp): a local golden
/// section on \f$[u_{\mathrm{prev}} \mp w/(2\lVert r_u\rVert)]\f$ (moved by \f$w/2\f$ while the
/// result sits on a bracket edge, at most one lap), the global stage of `global_search_samples`
/// samples plus the rays at initialisation, after an edge result or when
/// \f$D_{\mathrm{loc}} > D_{\mathrm{trig}}\f$, and adoptGlobalCandidate(). The foot is then
/// polished by at most kNewtonPolishSteps Newton steps on \f$g(u) = r_u^\top(r - x)\f$ (math_spec
/// §14.1). Open curves have straight rays at both ends. \f$T = \operatorname{unit}(r_u)\f$,
/// \f$n = \operatorname{unit}(\Pi_T(x - x^\star))\f$, \f$n := 0\f$ below \f$\varepsilon_D\f$.
/// Static curves keep a u↔s table (5-point Gauss–Legendre per interval) and the global samples;
/// time-varying curves integrate with fixed panels and sample on the fly, without allocation.
class ParametricCurve3 : public Curve3
{
public:
  ~ParametricCurve3() override;

  /// @brief Lower end \f$u_i\f$ of the domain.
  double uMin() const noexcept { return u_min_; }
  /// @brief Upper end \f$u_f\f$ of the domain.
  double uMax() const noexcept { return u_max_; }
  /// @brief The options the curve was built with.
  const CurveOptions & options() const noexcept { return options_; }
  /// @brief Signed arc length \f$\int_{u_0}^{u_1}\lVert r_u\rVert\,du\f$ at time t.
  double arcLength(double u0, double u1, double t) const noexcept;
  /// @brief Station \f$s(u)\f$ from the start at time t.
  double stationAt(double u, double t) const noexcept;
  /// @brief Parameter \f$u(s)\f$ (s clamped to \f$[0, L]\f$ when open, wrapped when closed).
  double parameterAt(double s, double t) const noexcept;
  /// @brief \f$\lVert r_u\rVert\f$ floor below which the curve counts as irregular.
  double speedFloor() const noexcept { return speed_floor_; }
  /// @brief Records the spec this curve was built from (makeCurve3 only).
  void setSpec(std::shared_ptr<const CurveSpec3> spec) noexcept;

  ClosestPoint3 closestPoint(
    const Vec3 & x, const EvalContext3 & context, const CurveHint3 & hint) const noexcept override;
  bool isClosed() const noexcept override { return closed_; }
  bool isTimeVarying() const noexcept override { return time_varying_; }
  double length(double t) const noexcept override;
  Vec3 velocity(const ClosestPoint3 & cp, double t) const noexcept override;
  Vec3 pointAt(double s, double t) const noexcept override;
  Vec3 tangentAt(double s, double t) const noexcept override;
  const CurveSpec3 * spec() const noexcept override;

  /// @brief \f$r(u, t)\f$; must be finite inside the domain (NaN reports kNonFinite).
  virtual Vec3 r(double u, double t) const noexcept = 0;
  /// @brief \f$\partial r/\partial u\f$. Default: central differences with
  /// \f$h_u = 6\cdot10^{-6}(u_f - u_i)\f$, one-sided second order at open ends, wrapping when
  /// closed.
  virtual Vec3 drdu(double u, double t) const noexcept;
  /// @brief \f$\partial r/\partial t\f$. Default: zero when static, else a central difference
  /// with \f$\delta_t\f$ = kTimeDerivativeStep.
  virtual Vec3 drdt(double u, double t) const noexcept;

protected:
  /// @brief Stores the domain and options; no evaluation happens here.
  /// @throws ConfigError on an empty or non-finite domain or invalid options.
  ParametricCurve3(
    double u_min, double u_max, bool closed, bool time_varying, const CurveOptions & options);
  ParametricCurve3(const ParametricCurve3 &) = default;
  ParametricCurve3(ParametricCurve3 &&) = default;
  ParametricCurve3 & operator=(const ParametricCurve3 &) = default;
  ParametricCurve3 & operator=(ParametricCurve3 &&) = default;

  /// @brief Builds the samples and the u↔s table at \f$t_0\f$ and runs the regularity and
  /// closure checks (math_spec §2.6). Call at the end of the most-derived constructor.
  /// @throws CurveError (kInvalidCurve).
  void initialize(double t0);

private:
  struct Candidate;
  double wrapU(double u) const noexcept;
  double clampU(double u) const noexcept;
  Candidate golden(const Vec3 & x, double a, double b, double t) const noexcept;
  Candidate globalStage(const Vec3 & x, const EvalContext3 & context) const noexcept;
  Candidate localStage(const Vec3 & x, double u_prev, double t, bool * on_edge) const noexcept;
  Candidate withRays(const Vec3 & x, const Candidate & c, double t) const noexcept;
  void polish(const Vec3 & x, double lo, double hi, double t, Candidate * c) const noexcept;
  ClosestPoint3 finish(const Vec3 & x, const Candidate & c, double t) const noexcept;
  double panelIntegral(double a, double b, double t) const noexcept;

  double u_min_{0.0};
  double u_max_{1.0};
  bool closed_{false};
  bool time_varying_{false};
  CurveOptions options_;
  double speed_floor_{0.0};
  double length_{0.0};
  std::vector<double> table_u_;
  std::vector<double> table_s_;
  // r(u_k, t0) at the global-stage samples (static curves only).
  std::vector<Vec3> samples_;
  std::shared_ptr<const CurveSpec3> spec_;
};

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__PARAMETRIC_CURVE_HPP_
