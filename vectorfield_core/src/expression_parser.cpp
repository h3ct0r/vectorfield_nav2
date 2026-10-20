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
// The muParser hardening of plan §3.2 (SECURITY.md; upstream audit U01–U05), shared by the 2D
// ExpressionCurve and the 3D ExpressionCurve3. muParser headers are included here and in the two
// curve sources only.

#include "expression_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <new>
#include <sstream>
#include <string>
#include <vector>

#include <muParser.h>

#include "vectorfield_core/curve_spec.hpp"

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

void checkExpressionInputs(
  double s_min, double s_max, double t0, const std::vector<std::string> & names,
  const std::vector<double> & values)
{
  if (!std::isfinite(s_min) || !std::isfinite(s_max) || !(s_min < s_max)) {
    reject(
      "the domain [s_min, s_max] = [" + fmt(s_min) + ", " + fmt(s_max) +
      "] must be finite with s_min < s_max");
  }
  if (!std::isfinite(t0)) {
    reject("t0 = " + fmt(t0) + " must be finite");
  }
  const std::size_t count = names.size();
  if (count != values.size()) {
    reject(
      std::to_string(count) + " parameter names but " + std::to_string(values.size()) + " values");
  }
  if (count > kMaxExpressionParameters) {
    reject(
      std::to_string(count) + " parameters; the limit is " +
      std::to_string(kMaxExpressionParameters));
  }
  for (std::size_t i = 0; i < count; ++i) {
    const std::string & name = names[i];
    if (!isIdentifier(name)) {
      reject(
        "parameter name '" + name +
        "' is not an identifier ([A-Za-z_][A-Za-z0-9_]*, at most 100 characters)");
    }
    if (isReservedExpressionName(name)) {
      reject("parameter name '" + name + "' is reserved (the variables s and t, or a function)");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (names[j] == name) {
        reject("parameter name '" + name + "' is declared twice");
      }
    }
    if (!std::isfinite(values[i])) {
      reject("parameter '" + name + "' = " + fmt(values[i]) + " is not finite");
    }
  }
}

bool configureExpressionParser(
  mu::Parser & parser, double * s, double * t, const std::vector<std::string> & names,
  const std::vector<double> & values, const std::string & expression, const char * which)
{
  bool uses_t = false;
  try {
    defineFunctions(parser);
    parser.DefineVar("s", s);
    parser.DefineVar("t", t);
    for (std::size_t i = 0; i < names.size(); ++i) {
      parser.DefineConst(names[i], values[i]);
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

}  // namespace vectorfield_core::detail
