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
/// @brief The G-type gain functions \f$G\f$ and \f$H\f$ of the field (math_spec §3, sec-gains).

#ifndef VECTORFIELD_CORE__GAINS_HPP_
#define VECTORFIELD_CORE__GAINS_HPP_

#include "vectorfield_core/config.hpp"

namespace vectorfield_core
{

/// @brief \f$c_G\f$ of the atan gain: \f$\max_u \frac{G G'}{H}\f$ for \f$k_f = 1\f$, attained at
/// \f$u \approx 0.87865\f$ (math_spec §3 and Appendix B, computed numerically).
inline constexpr double kCgAtan = 0.18557464863191414;
/// @brief \f$c_G\f$ of the rational gain: \f$2/(3\sqrt3)\f$, attained at \f$u = 1/\sqrt2\f$.
inline constexpr double kCgRational = 0.38490017945975050;

/// @brief A G-type gain function [Thesis Def. 10, 11] and its derived quantities.
///
/// | | atan | rational |
/// |---|---|---|
/// | \f$G(D)\f$ | \f$\frac{2}{\pi}\arctan(k_f D)\f$ | \f$k_f D/\sqrt{1 + (k_f D)^2}\f$ |
/// | \f$H(D)\f$ | \f$\sqrt{(1 - G)(1 + G)}\f$ | \f$1/\sqrt{1 + (k_f D)^2}\f$ |
/// | \f$G'(D)\f$ | \f$\frac{2 k_f}{\pi}\frac{1}{1 + (k_f D)^2}\f$ | \f$k_f(1 + (k_f D)^2)^{-3/2}\f$ |
/// | \f$G'(0)\f$ | \f$2k_f/\pi\f$ | \f$k_f\f$ |
/// | \f$G^{-1}(y)\f$ | \f$\tan(\pi y/2)/k_f\f$ | \f$y/(k_f\sqrt{1 - y^2})\f$ |
/// | \f$c_G\f$ | 0.18557 | 0.38490 |
///
/// \f$G^2 + H^2 = 1\f$ holds to rounding (oracle O15, \f$10^{-15}\f$), which makes
/// \f$\lVert\Phi_S\rVert = 1\f$. \f$G\f$ and \f$G'\f$ are extended oddly and \f$H\f$ evenly to
/// \f$D < 0\f$ (the field only uses \f$D \ge 0\f$). A cheap value type; every method is `noexcept`
/// and allocation-free.
class GainFunction
{
public:
  /// @brief The default gain: atan with \f$k_f = 2\f$ 1/m (the parameter defaults).
  GainFunction() noexcept = default;

  /// @brief A gain of the given type and convergence gain.
  /// @throws ConfigError if `k_f` is not finite and positive.
  GainFunction(GainType type, double k_f);

  /// @brief The gain of a field configuration (`gain_function`, `convergence_gain`).
  /// @throws ConfigError if `convergence_gain` is not finite and positive.
  static GainFunction fromConfig(const FieldConfig & field);

  /// @brief Gain type.
  GainType type() const noexcept { return type_; }
  /// @brief Convergence gain \f$k_f\f$ (1/m).
  double k() const noexcept { return k_; }

  /// @brief \f$G(D) \in [0, 1)\f$ for \f$D \ge 0\f$.
  double G(double D) const noexcept;
  /// @brief \f$H(D) = \sqrt{1 - G^2} \in (0, 1]\f$, in the cancellation-free forms of the table.
  double H(double D) const noexcept;
  /// @brief \f$G'(D)\f$ (1/m).
  double dG(double D) const noexcept;
  /// @brief \f$G^{-1}(y)\f$ for \f$y \in [0, 1)\f$ (odd for \f$y < 0\f$); \f$+\infty\f$ for
  /// \f$y \ge 1\f$; NaN for NaN.
  double Ginv(double y) const noexcept;
  /// @brief \f$G'(0)\f$: \f$2k_f/\pi\f$ (atan) or \f$k_f\f$ (rational).
  double dG0() const noexcept;
  /// @brief The dimensionless constant \f$c_G\f$ (kCgAtan or kCgRational).
  double cG() const noexcept;
  /// @brief \f$\kappa_F = c_G k_f\f$: the maximum curvature of a static field's integral curve
  /// near a straight path (math_spec §3 "Integral-curve curvature").
  double maxIntegralCurvature() const noexcept { return cG() * k_; }
  /// @brief \f$G(D)\,G'(D)/H(D)\f$: the curvature of a static field's integral curve at height
  /// \f$D\f$ above a straight path (math_spec §3).
  double integralCurveCurvature(double D) const noexcept;

private:
  GainType type_{GainType::kAtan};
  double k_{2.0};
};

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__GAINS_HPP_
