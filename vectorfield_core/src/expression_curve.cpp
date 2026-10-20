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

// ExpressionCurve on the vendored muParser (plan §3.2 expression hardening, §6 SECURITY.md;
// upstream audit U01–U05). muParser headers are included here only.

#include "expression_curve.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <memory>
#include <mutex>
#include <new>
#include <sstream>
#include <string>
#include <utility>

#include <muParser.h>

#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/parametric_curve.hpp"

namespace vectorfield_core::detail
{

namespace
{

using Math = mu::MathImpl<mu::value_type>;

struct UnaryFunction
{
  const char * name;
  mu::value_type (*function)(mu::value_type);
};

struct BinaryFunction
{
  const char * name;
  mu::value_type (*function)(mu::value_type, mu::value_type);
};

struct VariadicFunction
{
  const char * name;
  mu::value_type (*function)(const mu::value_type *, int);
};

// muParser 2.3.6's default functions (Parser::InitFun), all deterministic, without rnd.
constexpr UnaryFunction kUnaryFunctions[] = {
  {"sin", &Math::Sin},     {"cos", &Math::Cos},     {"tan", &Math::Tan},   {"cot", &Math::Cot},
  {"asin", &Math::ASin},   {"acos", &Math::ACos},   {"atan", &Math::ATan}, {"sinh", &Math::Sinh},
  {"cosh", &Math::Cosh},   {"tanh", &Math::Tanh},   {"coth", &Math::Coth}, {"asinh", &Math::ASinh},
  {"acosh", &Math::ACosh}, {"atanh", &Math::ATanh}, {"log2", &Math::Log2}, {"log10", &Math::Log10},
  {"log", &Math::Log},     {"ln", &Math::Log},      {"exp", &Math::Exp},   {"sqrt", &Math::Sqrt},
  {"sign", &Math::Sign},   {"rint", &Math::Rint},   {"abs", &Math::Abs},
};
constexpr BinaryFunction kBinaryFunctions[] = {{"atan2", &Math::ATan2}};
constexpr VariadicFunction kVariadicFunctions[] = {
  {"sum", &Math::Sum}, {"avg", &Math::Avg}, {"min", &Math::Min}, {"max", &Math::Max}};

// Names that are never parameters: the variables, muParser's constants and the removed rnd.
constexpr const char * kReservedNames[] = {"s", "t", "_pi", "_e", "rnd"};

std::string fmt(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(10) << value;
  return stream.str();
}

[[noreturn]] void reject(const std::string & message)
{
  throw CurveError(CurveError::Code::kInvalidCurve, message);
}

bool isIdentifier(const std::string & name) noexcept
{
  if (name.empty() || name.size() > kMaxExpressionNameLength) {
    return false;
  }
  const auto letter = [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
  };
  if (!letter(name[0])) {
    return false;
  }
  return std::all_of(name.begin() + 1, name.end(), [&letter](char c) {
    return letter(c) || (c >= '0' && c <= '9');
  });
}

void checkExpressionText(const std::string & expression, const char * which)
{
  if (expression.find_first_not_of(" \t\r\n") == std::string::npos) {
    reject(std::string(which) + " is empty");
  }
  if (expression.size() > kMaxExpressionLength) {
    reject(
      std::string(which) + " has " + std::to_string(expression.size()) +
      " characters; the limit is " + std::to_string(kMaxExpressionLength));
  }
  // muParser folds `a && b` and `a || b` on constants as (int)a && (int)b, which is undefined
  // behaviour for operands outside the int range (found by the libFuzzer target). A curve needs
  // no logical operators; comparisons and the ternary `?:` stay available.
  if (expression.find("&&") != std::string::npos || expression.find("||") != std::string::npos) {
    reject(std::string(which) + " uses && or ||; logical operators are not supported");
  }
}

void validateSpec(const CurveSpec & spec)
{
  checkExpressionText(spec.x_expression, "x_expression");
  checkExpressionText(spec.y_expression, "y_expression");
  if (!std::isfinite(spec.s_min) || !std::isfinite(spec.s_max) || !(spec.s_min < spec.s_max)) {
    reject(
      "the domain [s_min, s_max] = [" + fmt(spec.s_min) + ", " + fmt(spec.s_max) +
      "] must be finite with s_min < s_max");
  }
  if (!std::isfinite(spec.t0)) {
    reject("t0 = " + fmt(spec.t0) + " must be finite");
  }
  const std::size_t count = spec.parameter_names.size();
  if (count != spec.parameter_values.size()) {
    reject(
      std::to_string(count) + " parameter names but " +
      std::to_string(spec.parameter_values.size()) + " values");
  }
  if (count > kMaxExpressionParameters) {
    reject(
      std::to_string(count) + " parameters; the limit is " +
      std::to_string(kMaxExpressionParameters));
  }
  for (std::size_t i = 0; i < count; ++i) {
    const std::string & name = spec.parameter_names[i];
    if (!isIdentifier(name)) {
      reject(
        "parameter name '" + name +
        "' is not an identifier ([A-Za-z_][A-Za-z0-9_]*, at most 100 characters)");
    }
    if (isReservedExpressionName(name)) {
      reject("parameter name '" + name + "' is reserved (the variables s and t, or a function)");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (spec.parameter_names[j] == name) {
        reject("parameter name '" + name + "' is declared twice");
      }
    }
    if (!std::isfinite(spec.parameter_values[i])) {
      reject("parameter '" + name + "' = " + fmt(spec.parameter_values[i]) + " is not finite");
    }
  }
}

void defineFunctions(mu::Parser & parser)
{
  parser.ClearFun();
  for (const UnaryFunction & entry : kUnaryFunctions) {
    parser.DefineFun(entry.name, entry.function);
  }
  for (const BinaryFunction & entry : kBinaryFunctions) {
    parser.DefineFun(entry.name, entry.function);
  }
  for (const VariadicFunction & entry : kVariadicFunctions) {
    parser.DefineFun(entry.name, entry.function);
  }
}

}  // namespace

bool isReservedExpressionName(const std::string & name) noexcept
{
  for (const char * reserved : kReservedNames) {
    if (name == reserved) {
      return true;
    }
  }
  for (const UnaryFunction & entry : kUnaryFunctions) {
    if (name == entry.name) {
      return true;
    }
  }
  for (const BinaryFunction & entry : kBinaryFunctions) {
    if (name == entry.name) {
      return true;
    }
  }
  for (const VariadicFunction & entry : kVariadicFunctions) {
    if (name == entry.name) {
      return true;
    }
  }
  return false;
}

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

namespace
{

// Configures one parser and parses its expression; returns true iff the expression uses t.
bool configureParser(
  mu::Parser & parser, double * s, double * t, const CurveSpec & spec,
  const std::string & expression, const char * which)
{
  bool uses_t = false;
  try {
    defineFunctions(parser);
    parser.DefineVar("s", s);
    parser.DefineVar("t", t);
    for (std::size_t i = 0; i < spec.parameter_names.size(); ++i) {
      parser.DefineConst(spec.parameter_names[i], spec.parameter_values[i]);
    }
    parser.SetExpr(expression);
    // The first evaluation parses: syntax errors, undefined names (rnd included), assignments
    // to parameters (constants) and string results throw here.
    static_cast<void>(parser.Eval());
    const int results = parser.GetNumResults();
    if (results != 1) {
      reject(
        std::string(which) + " must have exactly one result, it has " + std::to_string(results) +
        " (comma-separated expressions are not allowed)");
    }
    for (const auto & used : parser.GetUsedVar()) {
      if (used.first == "t") {
        uses_t = true;
      } else if (used.first != "s") {
        reject(std::string(which) + ": unknown variable '" + used.first + "'");
      }
    }
    // GetUsedVar() leaves the parser in string mode; evaluate once more to rebuild bytecode.
    static_cast<void>(parser.Eval());
  } catch (const mu::ParserError & error) {
    reject(std::string(which) + ": " + error.GetMsg());
  } catch (const CurveError &) {
    throw;
  } catch (const std::bad_alloc &) {
    throw;
  } catch (const std::exception & error) {
    reject(std::string(which) + ": " + error.what());
  }
  return uses_t;
}

}  // namespace

std::unique_ptr<ExpressionCurve::Impl> ExpressionCurve::parse(const CurveSpec & spec)
{
  validateSpec(spec);
  auto impl = std::make_unique<Impl>();
  impl->t0 = spec.t0;
  impl->s = 0.5 * spec.s_min + 0.5 * spec.s_max;
  impl->t = 0.0;
  const bool x_uses_t =
    configureParser(impl->x_parser, &impl->s, &impl->t, spec, spec.x_expression, "x_expression");
  const bool y_uses_t =
    configureParser(impl->y_parser, &impl->s, &impl->t, spec, spec.y_expression, "y_expression");
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
