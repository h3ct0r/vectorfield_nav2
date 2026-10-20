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
/// @brief Internal: ExpressionCurve, r(s, t) from two muParser expressions (not installed;
/// muParser headers stay in the .cpp).

#ifndef VECTORFIELD_CORE__DETAIL__EXPRESSION_CURVE_HPP_
#define VECTORFIELD_CORE__DETAIL__EXPRESSION_CURVE_HPP_

#include <memory>
#include <string>

#include "expression_parser.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core::detail
{

// kMaxExpressionNameLength and isReservedExpressionName() are declared in expression_parser.hpp.

/// @brief A curve \f$r(s, t) = (x(s, t), y(s, t))\f$ given by two muParser expressions in `s`,
/// `t` and named parameters (plan §3.2, §6; upstream audit U01–U05).
///
/// **Hardening** (from construction onward):
/// - declared parameters are bound with `DefineConst`, so assigning to one is a parse error;
/// - `s` and `t` are bound with `DefineVar` and written immediately before every evaluation;
/// - `ClearFun()`, then muParser's default functions are re-defined except `rnd`: sin cos tan cot
///   asin acos atan atan2 sinh cosh tanh coth asinh acosh atanh log2 log10 log ln exp sqrt sign
///   rint abs sum avg min max;
/// - after the first evaluation each expression must have exactly one result;
/// - at most kMaxExpressionLength characters per expression and kMaxExpressionParameters
///   parameters, each an identifier (`[A-Za-z_][A-Za-z0-9_]*`, at most 100 characters) that is
///   not reserved (isReservedExpressionName()) and not repeated, with a finite value;
/// - no variable factory, so an undefined name is a parse error.
///
/// **Time.** The expression's `t` is the curve time minus `CurveSpec::t0`: `r(u, t)` evaluates
/// the expressions at \f$(s, t) = (u, t - t_0)\f$, so `t` = 0 at the time origin, where the
/// regularity checks run. The curve is time-varying iff `GetUsedVar()` of either expression
/// contains `t`; an open time-varying curve throws CurveError::Code::kUnsupported.
///
/// **Threads.** Every muParser call after construction goes through the private evalNoThrow():
/// it holds a mutex, catches everything and returns NaN on failure (reported by ParametricCurve
/// as Status::kNonFinite). The class is non-copyable and has no `clone()`; per-thread copies come
/// from `makeCurve(*curve.spec(), options)`, which re-parses.
class ExpressionCurve final : public ParametricCurve
{
public:
  /// @brief Parses, validates and runs the regularity and closure checks at `spec.t0`.
  /// @throws CurveError (kInvalidCurve; kUnsupported for an open time-varying curve), or
  /// ConfigError from ParametricCurve for invalid `options` (makeCurve() validates them first).
  ExpressionCurve(const CurveSpec & spec, const CurveOptions & options);
  ~ExpressionCurve() override;

  ExpressionCurve(const ExpressionCurve &) = delete;
  ExpressionCurve & operator=(const ExpressionCurve &) = delete;
  ExpressionCurve(ExpressionCurve &&) = delete;
  ExpressionCurve & operator=(ExpressionCurve &&) = delete;

  /// @brief \f$(x, y)\f$ at \f$s = u\f$ and expression time \f$t - t_0\f$; NaN on an
  /// evaluation failure. A parameter outside \f$[s_{\min}, s_{\max}]\f$ is wrapped (closed) or
  /// clamped (open), so the expressions are never evaluated outside their domain.
  Vec2 r(double u, double t) const noexcept override;

  /// @brief \f$\partial r/\partial t\f$ by a central difference with \f$\delta_t\f$ =
  /// kTimeDerivativeStep around \f$t - t_0\f$ (math_spec §4.2); zero when static.
  Vec2 drdt(double u, double t) const noexcept override;

private:
  struct Impl;

  ExpressionCurve(std::unique_ptr<Impl> impl, const CurveSpec & spec, const CurveOptions & options);

  // Validates the spec and builds both parsers; throws CurveError.
  static std::unique_ptr<Impl> parse(const CurveSpec & spec);

  // u inside [s_min, s_max] unchanged; outside: wrapped (closed) or clamped (open).
  double domainParameter(double u) const noexcept;

  // The only muParser entry point after construction: mutex, try/catch, NaN on failure.
  Vec2 evalNoThrow(double s, double t) const noexcept;

  std::unique_ptr<Impl> impl_;
};

}  // namespace vectorfield_core::detail

#endif  // VECTORFIELD_CORE__DETAIL__EXPRESSION_CURVE_HPP_
