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

// ExpressionCurve on the vendored muParser (SECURITY.md;
// upstream audit U01–U05). muParser headers are included in the sources only.

#include "expression_curve.hpp"

#include "expression_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <muParser.h>

#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/parametric_curve.hpp"

namespace vectorfield_core::detail
{

// The hardening helpers (validation, parser configuration) live in expression_parser.cpp.

struct ExpressionCurve::Impl
{
  mu::Parser x_parser;
  mu::Parser y_parser;
  // Storage bound to the expression variables s and t (DefineVar keeps these addresses).
  double s{0.0};
  double t{0.0};
  // Time origin t0 (CurveSpec::t0).
  double t0{0.0};
  bool time_varying{false};
  std::mutex mutex;
};

std::unique_ptr<ExpressionCurve::Impl> ExpressionCurve::parse(const CurveSpec & spec)
{
  checkExpressionText(spec.x_expression, "x_expression");
  checkExpressionText(spec.y_expression, "y_expression");
  checkExpressionInputs(
    spec.s_min, spec.s_max, spec.t0, spec.parameter_names, spec.parameter_values);
  auto impl = std::make_unique<Impl>();
  impl->t0 = spec.t0;
  impl->s = 0.5 * spec.s_min + 0.5 * spec.s_max;
  impl->t = 0.0;
  const bool x_uses_t = configureExpressionParser(
    impl->x_parser, &impl->s, &impl->t, spec.parameter_names, spec.parameter_values,
    spec.x_expression, "x_expression");
  const bool y_uses_t = configureExpressionParser(
    impl->y_parser, &impl->s, &impl->t, spec.parameter_names, spec.parameter_values,
    spec.y_expression, "y_expression");
  impl->time_varying = x_uses_t || y_uses_t;
  return impl;
}

ExpressionCurve::ExpressionCurve(const CurveSpec & spec, const CurveOptions & options)
: ExpressionCurve(parse(spec), spec, options)
{
}

ExpressionCurve::ExpressionCurve(
  std::unique_ptr<Impl> impl, const CurveSpec & spec, const CurveOptions & options)
: ParametricCurve(spec.s_min, spec.s_max, spec.closed, impl->time_varying, options),
  impl_(std::move(impl))
{
  if (!isClosed() && isTimeVarying()) {
    throw CurveError(
      CurveError::Code::kUnsupported,
      "an open EXPRESSION curve that uses t (time-varying) is not supported: the terminal field "
      "is defined for static curves only; close the curve or remove t");
  }
  initialize(spec.t0);
}

ExpressionCurve::~ExpressionCurve() = default;

double ExpressionCurve::domainParameter(double u) const noexcept
{
  if (u >= uMin() && u <= uMax()) {
    return u;
  }
  if (isClosed() && std::isfinite(u)) {
    const double period = uMax() - uMin();
    double v = std::fmod(u - uMin(), period);
    if (v < 0.0) {
      v += period;
    }
    return uMin() + (v < period ? v : 0.0);
  }
  return std::min(uMax(), std::max(uMin(), u));
}

Vec2 ExpressionCurve::r(double u, double t) const noexcept
{
  if (std::isnan(u)) {
    return Vec2(kNaN, kNaN);
  }
  return evalNoThrow(domainParameter(u), t - impl_->t0);
}

Vec2 ExpressionCurve::drdt(double u, double t) const noexcept
{
  if (!isTimeVarying()) {
    return Vec2::Zero();
  }
  if (std::isnan(u)) {
    return Vec2(kNaN, kNaN);
  }
  const double s = domainParameter(u);
  // Difference around the expression time t − t0, so a large absolute t loses no precision.
  const double tau = t - impl_->t0;
  const Vec2 ahead = evalNoThrow(s, tau + kTimeDerivativeStep);
  const Vec2 behind = evalNoThrow(s, tau - kTimeDerivativeStep);
  return (ahead - behind) / (2.0 * kTimeDerivativeStep);
}

Vec2 ExpressionCurve::evalNoThrow(double s, double t) const noexcept
{
  try {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->s = s;
    impl_->t = t;
    const double x = impl_->x_parser.Eval();
    // Written again: an expression may assign to s or t.
    impl_->s = s;
    impl_->t = t;
    const double y = impl_->y_parser.Eval();
    return Vec2(x, y);
  } catch (...) {
    return Vec2(kNaN, kNaN);
  }
}

}  // namespace vectorfield_core::detail
