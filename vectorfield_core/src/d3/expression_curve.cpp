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

// ExpressionCurve3: three muParser expressions with the shared hardening of
// src/expression_parser.hpp (plan §3.2; math_spec §14.1).

#include "expression_curve.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <utility>

#include <muParser.h>

#include "../expression_parser.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/parametric_curve.hpp"

namespace vectorfield_core::d3::detail
{

struct ExpressionCurve3::Impl
{
  mu::Parser x_parser;
  mu::Parser y_parser;
  mu::Parser z_parser;
  // Storage bound to the expression variables s and t.
  double s{0.0};
  double t{0.0};
  double t0{0.0};
  bool time_varying{false};
  std::mutex mutex;
};

std::unique_ptr<ExpressionCurve3::Impl> ExpressionCurve3::parse(const CurveSpec3 & spec)
{
  namespace vd = vectorfield_core::detail;
  vd::checkExpressionText(spec.x_expression, "x_expression");
  vd::checkExpressionText(spec.y_expression, "y_expression");
  vd::checkExpressionText(spec.z_expression, "z_expression");
  vd::checkExpressionInputs(
    spec.s_min, spec.s_max, spec.t0, spec.parameter_names, spec.parameter_values);
  auto impl = std::make_unique<Impl>();
  impl->t0 = spec.t0;
  impl->s = 0.5 * spec.s_min + 0.5 * spec.s_max;
  impl->t = 0.0;
  bool uses_t = false;
  uses_t |= vd::configureExpressionParser(
    impl->x_parser, &impl->s, &impl->t, spec.parameter_names, spec.parameter_values,
    spec.x_expression, "x_expression");
  uses_t |= vd::configureExpressionParser(
    impl->y_parser, &impl->s, &impl->t, spec.parameter_names, spec.parameter_values,
    spec.y_expression, "y_expression");
  uses_t |= vd::configureExpressionParser(
    impl->z_parser, &impl->s, &impl->t, spec.parameter_names, spec.parameter_values,
    spec.z_expression, "z_expression");
  impl->time_varying = uses_t;
  return impl;
}

ExpressionCurve3::ExpressionCurve3(const CurveSpec3 & spec, const CurveOptions & options)
: ExpressionCurve3(parse(spec), spec, options)
{
}

ExpressionCurve3::ExpressionCurve3(
  std::unique_ptr<Impl> impl, const CurveSpec3 & spec, const CurveOptions & options)
: ParametricCurve3(spec.s_min, spec.s_max, spec.closed, impl->time_varying, options),
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

ExpressionCurve3::~ExpressionCurve3() = default;

double ExpressionCurve3::domainParameter(double u) const noexcept
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

Vec3 ExpressionCurve3::r(double u, double t) const noexcept
{
  if (std::isnan(u)) {
    return Vec3::Constant(kNaN);
  }
  return evalNoThrow(domainParameter(u), t - impl_->t0);
}

Vec3 ExpressionCurve3::drdt(double u, double t) const noexcept
{
  if (!isTimeVarying()) {
    return Vec3::Zero();
  }
  if (std::isnan(u)) {
    return Vec3::Constant(kNaN);
  }
  const double s = domainParameter(u);
  const double tau = t - impl_->t0;
  const Vec3 ahead = evalNoThrow(s, tau + kTimeDerivativeStep);
  const Vec3 behind = evalNoThrow(s, tau - kTimeDerivativeStep);
  return (ahead - behind) / (2.0 * kTimeDerivativeStep);
}

Vec3 ExpressionCurve3::evalNoThrow(double s, double t) const noexcept
{
  try {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    Vec3 out;
    mu::Parser * const parsers[] = {&impl_->x_parser, &impl_->y_parser, &impl_->z_parser};
    for (int i = 0; i < 3; ++i) {
      // Written before every expression: an expression may assign to s or t.
      impl_->s = s;
      impl_->t = t;
      out[i] = parsers[i]->Eval();
    }
    return out;
  } catch (...) {
    return Vec3::Constant(kNaN);
  }
}

}  // namespace vectorfield_core::d3::detail
