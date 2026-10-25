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
/// @brief Internal: the muParser hardening shared by ExpressionCurve (2D) and ExpressionCurve3
/// (math_spec §14.1). Not installed; muParser is only forward-declared here.

#ifndef VECTORFIELD_CORE__DETAIL__EXPRESSION_PARSER_HPP_
#define VECTORFIELD_CORE__DETAIL__EXPRESSION_PARSER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace mu
{
class Parser;
}  // namespace mu

namespace vectorfield_core::detail
{

/// @brief Longest accepted parameter name (muParser's identifier limit).
inline constexpr std::size_t kMaxExpressionNameLength = 100;

/// @brief True iff `name` is reserved in expressions: the variables `s` and `t`, muParser's
/// constants `_pi` and `_e`, and every function name of muParser's default set (including the
/// removed `rnd`).
bool isReservedExpressionName(const std::string & name) noexcept;

/// @brief Rejects an empty expression, one longer than kMaxExpressionLength, or one that uses the
/// logical operators `&&` or `||`.
/// @param expression The expression text.
/// @param which Its name in messages, e.g. "x_expression".
/// @throws CurveError (kInvalidCurve).
void checkExpressionText(const std::string & expression, const char * which);

/// @brief Validates the domain \f$[s_{\min}, s_{\max}]\f$, \f$t_0\f$ and the parameter list
/// (count, identifiers, reserved names, duplicates, finite values).
/// @throws CurveError (kInvalidCurve).
void checkExpressionInputs(
  double s_min, double s_max, double t0, const std::vector<std::string> & names,
  const std::vector<double> & values);

/// @brief Configures one hardened parser and parses its expression: the default functions
/// without `rnd`, `s` and `t` bound to the given storage, the parameters as constants, exactly one
/// result and no variable other than `s` and `t`.
/// @param parser The parser to configure.
/// @param s Storage bound to `s`.
/// @param t Storage bound to `t`.
/// @param names Parameter names (validated by checkExpressionInputs()).
/// @param values Parameter values.
/// @param expression The expression text (validated by checkExpressionText()).
/// @param which Its name in messages.
/// @return True iff the expression uses `t`.
/// @throws CurveError (kInvalidCurve) on any parse error; std::bad_alloc passes through.
bool configureExpressionParser(
  mu::Parser & parser, double * s, double * t, const std::vector<std::string> & names,
  const std::vector<double> & values, const std::string & expression, const char * which);

}  // namespace vectorfield_core::detail

#endif  // VECTORFIELD_CORE__DETAIL__EXPRESSION_PARSER_HPP_
