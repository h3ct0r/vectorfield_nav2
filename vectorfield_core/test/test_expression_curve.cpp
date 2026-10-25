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

// ExpressionCurve on muParser: hardening, oracle O12 of math_spec §12.1 and the
// upstream regressions U01–U03 (docs/design/upstream_audit.md).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "expression_curve.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::Vec2;

namespace
{

using Parameters = std::vector<std::pair<std::string, double>>;

vf::CurveSpec expressionSpec(
  const std::string & x, const std::string & y, const Parameters & parameters = {},
  bool closed = true, double s_min = 0.0, double s_max = vf::kTwoPi)
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kExpression;
  spec.x_expression = x;
  spec.y_expression = y;
  spec.s_min = s_min;
  spec.s_max = s_max;
  spec.closed = closed;
  for (const auto & parameter : parameters) {
    spec.parameter_names.push_back(parameter.first);
    spec.parameter_values.push_back(parameter.second);
  }
  return spec;
}

std::shared_ptr<const vf::ParametricCurve> makeExpression(const vf::CurveSpec & spec)
{
  auto curve =
    std::dynamic_pointer_cast<const vf::ParametricCurve>(vf::makeCurve(spec, vf::CurveOptions{}));
  EXPECT_NE(curve, nullptr);
  return curve;
}

// The error of makeCurve(spec), or nullopt-like "" when it is accepted.
std::string rejection(const vf::CurveSpec & spec, vf::CurveError::Code * code = nullptr)
{
  try {
    vf::makeCurve(spec, vf::CurveOptions{});
  } catch (const vf::CurveError & error) {
    if (code != nullptr) {
      *code = error.code();
    }
    return error.what();
  }
  return "";
}

void expectInvalid(const vf::CurveSpec & spec, const std::string & needle)
{
  vf::CurveError::Code code = vf::CurveError::Code::kUnsupported;
  const std::string message = rejection(spec, &code);
  ASSERT_FALSE(message.empty()) << "accepted: x = '" << spec.x_expression << "', y = '"
                                << spec.y_expression << "'";
  EXPECT_EQ(code, vf::CurveError::Code::kInvalidCurve) << message;
  EXPECT_NE(message.find(needle), std::string::npos) << message;
}

}  // namespace

// O12 (also libFuzzer seeds): rnd, assignment to a declared parameter and two results.
TEST(ExpressionCurve, O12Rejects)
{
  const Parameters a{{"a", 1.0}};
  for (const bool on_x : {true, false}) {
    SCOPED_TRACE(on_x ? "x_expression" : "y_expression");
    const auto spec = [on_x, &a](const std::string & bad) {
      return on_x ? expressionSpec(bad, "sin(s)", a) : expressionSpec("cos(s)", bad, a);
    };
    expectInvalid(spec("cos(s)+rnd()"), "rnd");
    expectInvalid(spec("a=a+1e-4, a*cos(s)"), "Unexpected operator \"=\"");
    expectInvalid(spec("cos(s), sin(s)"), "exactly one result");
  }
}

// U01: no code execution: Python payloads, assignment to a parameter, rnd() and two-result
// expressions are INVALID_CURVE.
TEST(UpstreamRegression, U01_ExpressionCannotExecuteCode)
{
  const std::vector<std::string> payloads{
    "__import__('os').system('ls')",
    "__import__(\"os\").system(\"rm -rf /\")",
    "eval('1')",
    "exec(\"import os\")",
    "open('/etc/passwd').read()",
    "lambda: 0",
    "[cos(s), sin(s), 0]",
    "os.system('id')",
    "cos(s); sin(s)",
    "a = 1",
    "a=a+1e-4, a*cos(s)",
    "rnd()",
    "cos(s) + rnd()",
    "cos(s), sin(s)",
    "\"string\"",
    "cos(s) # comment",
    "`id`",
    "$HOME",
  };
  for (const std::string & payload : payloads) {
    SCOPED_TRACE(payload);
    vf::CurveError::Code code = vf::CurveError::Code::kUnsupported;
    const std::string message = rejection(expressionSpec(payload, "sin(s)", {{"a", 1.0}}), &code);
    EXPECT_FALSE(message.empty());
    EXPECT_EQ(code, vf::CurveError::Code::kInvalidCurve);
  }
}

// U02: identifiers bind by name: parameters u and us next to sqrt, cos, tanh and exp; a parameter
// named s or sin is INVALID_CURVE.
TEST(UpstreamRegression, U02_IdentifiersBoundByName)
{
  const auto curve = makeExpression(expressionSpec(
    "u*cos(s) + 0.1*tanh(us)", "sqrt(us)*sin(s) + 0.05*exp(-u)", {{"u", 2.0}, {"us", 0.25}}));
  ASSERT_NE(curve, nullptr);
  for (const double s : {0.0, 0.7, 2.0, 4.4}) {
    const Vec2 expected(
      2.0 * std::cos(s) + 0.1 * std::tanh(0.25),
      std::sqrt(0.25) * std::sin(s) + 0.05 * std::exp(-2.0));
    EXPECT_LE((curve->r(s, 0.0) - expected).norm(), 1e-14) << "s = " << s;
  }
  for (const char * name : {"s", "t", "sin", "sqrt", "tanh", "exp", "atan2", "rnd", "_pi"}) {
    SCOPED_TRACE(name);
    expectInvalid(expressionSpec("cos(s)", "sin(s)", {{name, 1.0}}), "reserved");
  }
}

// U03: two ExpressionCurves, created in either order and evaluated interleaved from two threads,
// each return their own values (also under TSan).
TEST(UpstreamRegression, U03_IndependentCurveInstances)
{
  const vf::CurveSpec spec_a = expressionSpec("2*cos(s)", "sin(s)");
  const vf::CurveSpec spec_b = expressionSpec("cos(s) + 5", "3*sin(s)");
  const auto expect_a = [](double s) { return Vec2(2.0 * std::cos(s), std::sin(s)); };
  const auto expect_b = [](double s) { return Vec2(std::cos(s) + 5.0, 3.0 * std::sin(s)); };
  for (const bool a_first : {true, false}) {
    SCOPED_TRACE(a_first ? "A created first" : "B created first");
    std::shared_ptr<const vf::ParametricCurve> a;
    std::shared_ptr<const vf::ParametricCurve> b;
    if (a_first) {
      a = makeExpression(spec_a);
      b = makeExpression(spec_b);
    } else {
      b = makeExpression(spec_b);
      a = makeExpression(spec_a);
    }
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    double worst_a = 0.0;
    double worst_b = 0.0;
    const auto run = [](
                       const vf::ParametricCurve & curve,
                       const std::function<Vec2(double)> & expected, double * worst) {
      vf::CurveHint hint;
      for (int i = 0; i < 4000; ++i) {
        const double s = 0.0015 * i;
        *worst = std::max(*worst, (curve.r(s, 0.0) - expected(s)).norm());
        const Vec2 x = 1.2 * expected(s);
        const vf::ClosestPoint cp = curve.closestPoint(x, vf::EvalContext{}, hint);
        hint = vf::makeHint(cp);
        *worst = std::max(*worst, cp.valid ? 0.0 : 1.0);
      }
    };
    std::thread thread_a([&]() { run(*a, expect_a, &worst_a); });
    std::thread thread_b([&]() { run(*b, expect_b, &worst_b); });
    thread_a.join();
    thread_b.join();
    EXPECT_LE(worst_a, 1e-15);
    EXPECT_LE(worst_b, 1e-15);
  }
}

// Logical operators are rejected before parsing: muParser folds constant `&&`/`||` through an
// int cast, undefined for operands outside the int range (fuzzer seed logical_or_int_overflow).
// Comparisons and the ternary operator stay available.
TEST(ExpressionCurve, LogicalOperatorsRejected)
{
  expectInvalid(expressionSpec("-1e45 || 1", "sin(s)"), "logical operators are not supported");
  expectInvalid(expressionSpec("cos(s)", "s > 0 && s < 1"), "y_expression uses && or ||");
  EXPECT_EQ(rejection(expressionSpec("s < 10 ? cos(s) : 0", "sin(s) * (s >= 0)")), "");
}

// Size limits: <= 1024 characters per expression, <= 32 parameters, identifier names of at
// most 100 characters, no duplicates, finite values.
TEST(ExpressionCurve, Limits)
{
  std::string longest = "cos(s)";
  while (longest.size() < vf::kMaxExpressionLength) {
    longest += "+0";
  }
  ASSERT_EQ(longest.size(), vf::kMaxExpressionLength);
  EXPECT_EQ(rejection(expressionSpec(longest, "sin(s)")), "");
  expectInvalid(expressionSpec(longest + " ", "sin(s)"), "the limit is 1024");
  expectInvalid(expressionSpec("cos(s)", longest + "0"), "y_expression has 1025 characters");

  Parameters many;
  for (std::size_t i = 0; i < vf::kMaxExpressionParameters; ++i) {
    many.emplace_back("p" + std::to_string(i), static_cast<double>(i));
  }
  EXPECT_EQ(rejection(expressionSpec("cos(s) + 0*p31", "sin(s)", many)), "");
  many.emplace_back("p32", 32.0);
  expectInvalid(expressionSpec("cos(s)", "sin(s)", many), "the limit is 32");

  for (const char * name : {"1a", "a-b", "", "a b", "\xc3\xa9", "a.b", "a(b)", "a+b"}) {
    SCOPED_TRACE(name);
    expectInvalid(expressionSpec("cos(s)", "sin(s)", {{name, 1.0}}), "not an identifier");
  }
  const std::string name_100(100, 'k');
  EXPECT_EQ(rejection(expressionSpec("cos(s) + 0*" + name_100, "sin(s)", {{name_100, 1.0}})), "");
  expectInvalid(expressionSpec("cos(s)", "sin(s)", {{name_100 + "k", 1.0}}), "not an identifier");
  expectInvalid(expressionSpec("cos(s)", "sin(s)", {{"k", 1.0}, {"k", 2.0}}), "declared twice");
  expectInvalid(expressionSpec("cos(s)", "sin(s)", {{"k", vf::kNaN}}), "not finite");
  expectInvalid(expressionSpec("cos(s)", "sin(s)", {{"k", vf::kInf}}), "not finite");
  vf::CurveSpec mismatch = expressionSpec("cos(s)", "sin(s)", {{"k", 1.0}});
  mismatch.parameter_values.clear();
  expectInvalid(mismatch, "parameter names but");
  expectInvalid(expressionSpec("", "sin(s)"), "x_expression is empty");
  expectInvalid(expressionSpec("cos(s)", " \t "), "y_expression is empty");
  vf::CurveSpec bad_t0 = expressionSpec("cos(s)", "sin(s)");
  bad_t0.t0 = vf::kInf;
  expectInvalid(bad_t0, "t0");
  for (const auto & [s_min, s_max] : std::vector<std::pair<double, double>>{
         {1.0, 1.0}, {2.0, 1.0}, {vf::kNaN, 1.0}, {0.0, vf::kInf}}) {
    expectInvalid(expressionSpec("s", "s^2", {}, false, s_min, s_max), "domain");
  }
}

// The function table is muParser's deterministic defaults, without rnd, each bound by name.
TEST(ExpressionCurve, DeterministicFunctionSetWithoutRnd)
{
  struct Function
  {
    std::string call;
    double value;
  };
  const std::vector<Function> functions{
    {"sin(0.5)", std::sin(0.5)},
    {"cos(0.5)", std::cos(0.5)},
    {"tan(0.5)", std::tan(0.5)},
    {"cot(0.5)", 1.0 / std::tan(0.5)},
    {"asin(0.5)", std::asin(0.5)},
    {"acos(0.5)", std::acos(0.5)},
    {"atan(0.5)", std::atan(0.5)},
    {"atan2(1, 2)", std::atan2(1.0, 2.0)},
    {"sinh(0.5)", std::sinh(0.5)},
    {"cosh(0.5)", std::cosh(0.5)},
    {"tanh(0.5)", std::tanh(0.5)},
    {"coth(0.5)", 1.0 / std::tanh(0.5)},
    {"asinh(0.5)", std::asinh(0.5)},
    {"acosh(1.5)", std::acosh(1.5)},
    {"atanh(0.5)", std::atanh(0.5)},
    {"log2(8)", 3.0},
    {"log10(1000)", 3.0},
    {"log(2)", std::log(2.0)},
    {"ln(2)", std::log(2.0)},
    {"exp(0.5)", std::exp(0.5)},
    {"sqrt(2)", std::sqrt(2.0)},
    {"sign(-3)", -1.0},
    {"rint(2.6)", 3.0},
    {"abs(-0.5)", 0.5},
    {"sum(1, 2, 3)", 6.0},
    {"avg(1, 2, 3)", 2.0},
    {"min(1, -2, 3)", -2.0},
    {"max(1, -2, 3)", 3.0},
    {"_pi", vf::kPi},
    {"_e", std::exp(1.0)},
  };
  for (const Function & function : functions) {
    SCOPED_TRACE(function.call);
    const auto curve = makeExpression(expressionSpec("cos(s) + 0.001*" + function.call, "sin(s)"));
    ASSERT_NE(curve, nullptr);
    EXPECT_NEAR(curve->r(0.0, 0.0).x(), 1.0 + 0.001 * function.value, 1e-15);
  }
  expectInvalid(expressionSpec("cos(s) + 0*rnd()", "sin(s)"), "rnd");
  for (const char * name : {"sin", "atan2", "sum", "rnd", "s", "t", "_pi", "_e", "coth"}) {
    EXPECT_TRUE(vf::detail::isReservedExpressionName(name)) << name;
  }
  for (const char * name : {"a", "u", "us", "pi", "e", "sine", "s1", "t_0"}) {
    EXPECT_FALSE(vf::detail::isReservedExpressionName(name)) << name;
  }
}

// No variable factory: an undefined name is a parse error.
TEST(ExpressionCurve, UndefinedNamesRejected)
{
  expectInvalid(expressionSpec("cos(s) + k", "sin(s)"), "Unexpected token \"k\"");
  expectInvalid(expressionSpec("foo(s)", "sin(s)"), "foo");
  expectInvalid(expressionSpec("pi*cos(s)", "sin(s)"), "pi");
  expectInvalid(expressionSpec("cos(u)", "sin(u)"), "u");
}

// Time-varying iff an expression uses t; an open time-varying curve is UNSUPPORTED.
TEST(ExpressionCurve, TimeVaryingDetectionAndOpenCurves)
{
  EXPECT_FALSE(makeExpression(expressionSpec("cos(s)", "sin(s)"))->isTimeVarying());
  EXPECT_TRUE(makeExpression(expressionSpec("cos(s) + 0*t", "sin(s)"))->isTimeVarying());
  EXPECT_TRUE(makeExpression(expressionSpec("cos(s)", "sin(s)*(1 + 0.1*t)"))->isTimeVarying());

  vf::CurveError::Code code = vf::CurveError::Code::kInvalidCurve;
  const std::string message = rejection(expressionSpec("s", "0.1*t", {}, false, 0.0, 1.0), &code);
  EXPECT_EQ(code, vf::CurveError::Code::kUnsupported);
  EXPECT_NE(message.find("time-varying"), std::string::npos) << message;
  EXPECT_STREQ(vf::toString(code), "UNSUPPORTED");
  // A static open expression is fine.
  EXPECT_EQ(rejection(expressionSpec("s", "0.1*s^2", {}, false, 0.0, 1.0)), "");
}

// The expression's t is the curve time minus t0: regularity is checked at expression time 0.
TEST(ExpressionCurve, TimeOriginShiftsExpressionTime)
{
  vf::CurveSpec spec = expressionSpec("(1 - t)*cos(s)", "(1 - t)*sin(s)");
  spec.t0 = 1.0;  // absolute t = 1 would be the degenerate radius 0
  const auto curve = makeExpression(spec);
  ASSERT_NE(curve, nullptr);
  EXPECT_LE((curve->r(0.3, 1.0) - Vec2(std::cos(0.3), std::sin(0.3))).norm(), 1e-15);
  EXPECT_LE((curve->r(0.3, 1.5) - 0.5 * Vec2(std::cos(0.3), std::sin(0.3))).norm(), 1e-15);
  EXPECT_NEAR(curve->length(1.0), vf::kTwoPi, 1e-8);
  EXPECT_NEAR(curve->length(1.25), 0.75 * vf::kTwoPi, 1e-8);
}

// ∂_t r is differenced around t − t0, so it stays accurate at large absolute times (ROS time).
TEST(ExpressionCurve, TimeDerivativeAtLargeAbsoluteTime)
{
  vf::CurveSpec spec = expressionSpec("cos(s + 0.5*t)", "sin(s + 0.5*t)");
  spec.t0 = 1.7e9;
  const auto curve = makeExpression(spec);
  ASSERT_NE(curve, nullptr);
  for (const double u : {0.0, 1.0, 4.0}) {
    const Vec2 expected = 0.5 * Vec2(-std::sin(u + 1.5), std::cos(u + 1.5));
    EXPECT_LE((curve->drdt(u, 1.7e9 + 3.0) - expected).norm(), 1e-8) << "u = " << u;
  }
  EXPECT_EQ(makeExpression(expressionSpec("cos(s)", "sin(s)"))->drdt(1.0, 5.0), Vec2::Zero());
}

// s and t are written before every evaluation: an assignment inside x never leaks into y or into
// the next evaluation.
TEST(ExpressionCurve, VariablesRewrittenBeforeEachEvaluation)
{
  const auto curve = makeExpression(expressionSpec("(s=s+1)*0 + cos(s)", "sin(s)"));
  ASSERT_NE(curve, nullptr);
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (const double u : {0.0, 0.5, 2.0}) {
      EXPECT_LE((curve->r(u, 0.0) - Vec2(std::cos(u + 1.0), std::sin(u))).norm(), 1e-15);
    }
  }
}

// Non-copyable, no clone(): per-thread copies come from makeCurve(*curve.spec()).
TEST(ExpressionCurve, PerThreadCopiesFromSpec)
{
  static_assert(!std::is_copy_constructible_v<vf::detail::ExpressionCurve>, "non-copyable");
  static_assert(!std::is_copy_assignable_v<vf::detail::ExpressionCurve>, "non-copyable");
  static_assert(!std::is_move_constructible_v<vf::detail::ExpressionCurve>, "non-movable");
  const vf::CurveSpec spec = expressionSpec("a*cos(s)", "sin(s) + 0*t", {{"a", 2.0}});
  const std::shared_ptr<const vf::Curve> original = vf::makeCurve(spec, vf::CurveOptions{});
  ASSERT_NE(original->spec(), nullptr);
  EXPECT_TRUE(*original->spec() == spec);
  const std::shared_ptr<const vf::Curve> copy =
    vf::makeCurve(*original->spec(), vf::CurveOptions{});
  EXPECT_NE(copy.get(), original.get());
  const Vec2 x(1.0, 2.0);
  vf::EvalContext context;
  context.t = 0.4;
  const vf::ClosestPoint a = original->closestPoint(x, context, vf::CurveHint{});
  const vf::ClosestPoint b = copy->closestPoint(x, context, vf::CurveHint{});
  EXPECT_EQ(a.x_star, b.x_star);
  EXPECT_EQ(a.s_star, b.s_star);
  EXPECT_EQ(original->velocity(a, 0.4), copy->velocity(b, 0.4));
}

// Direct construction (no spec stored), and r() clamps its parameter to the domain.
TEST(ExpressionCurve, DirectConstructionAndDomainClamp)
{
  const vf::CurveSpec spec =
    expressionSpec("s", "0.5*s^2 + 0*sqrt(s - 2) + 0*sqrt(5 - s)", {}, false, 2.0, 5.0);
  const vf::detail::ExpressionCurve curve(spec, vf::CurveOptions{});
  EXPECT_EQ(curve.spec(), nullptr);
  EXPECT_FALSE(curve.isTimeVarying());
  EXPECT_LE((curve.r(1.0, 0.0) - Vec2(2.0, 2.0)).norm(), 1e-15);
  EXPECT_LE((curve.r(9.0, 0.0) - Vec2(5.0, 12.5)).norm(), 1e-15);
  EXPECT_FALSE(vf::isFinite(curve.r(vf::kNaN, 0.0)));
  EXPECT_THROW(
    vf::detail::ExpressionCurve(expressionSpec("cos(s)+rnd()", "sin(s)"), vf::CurveOptions{}),
    vf::CurveError);
  // A closed expression is periodic outside its domain, and r(s_max) is the expression at s_max.
  const vf::detail::ExpressionCurve closed(
    expressionSpec("2*cos(s) + 1e-8*s", "sin(s)", {}, true, 0.0, vf::kTwoPi), vf::CurveOptions{});
  EXPECT_LE((closed.r(vf::kTwoPi + 0.3, 0.0) - closed.r(0.3, 0.0)).norm(), 1e-12);
  EXPECT_LE((closed.r(-0.3, 0.0) - closed.r(vf::kTwoPi - 0.3, 0.0)).norm(), 1e-12);
  EXPECT_NEAR(closed.r(vf::kTwoPi, 0.0).x(), 2.0 + 1e-8 * vf::kTwoPi, 1e-14);
}

// One instance shared by several threads: evaluations are serialised and stay correct.
TEST(ExpressionCurve, ConcurrentEvaluationOfOneInstance)
{
  const auto curve = makeExpression(expressionSpec("3*cos(s) + 0.1*t", "2*sin(s)"));
  ASSERT_NE(curve, nullptr);
  constexpr int kThreads = 4;
  std::vector<double> worst(kThreads, 0.0);
  std::vector<std::thread> threads;
  for (int k = 0; k < kThreads; ++k) {
    threads.emplace_back([&curve, &worst, k]() {
      for (int i = 0; i < 3000; ++i) {
        const double s = 0.002 * i + 0.1 * k;
        const double t = 0.01 * k;
        const Vec2 expected(3.0 * std::cos(s) + 0.1 * t, 2.0 * std::sin(s));
        worst[static_cast<std::size_t>(k)] =
          std::max(worst[static_cast<std::size_t>(k)], (curve->r(s, t) - expected).norm());
      }
    });
  }
  for (std::thread & thread : threads) {
    thread.join();
  }
  for (const double value : worst) {
    EXPECT_LE(value, 1e-15);
  }
}

// Per-thread copies may be parsed concurrently (muParser's shared state is read-only).
TEST(ExpressionCurve, ConcurrentConstruction)
{
  const vf::CurveSpec spec = expressionSpec("a*cos(s) + 0.1*t", "sin(s)", {{"a", 2.0}});
  constexpr int kThreads = 4;
  std::vector<double> worst(kThreads, 1.0);
  std::vector<std::thread> threads;
  for (int k = 0; k < kThreads; ++k) {
    threads.emplace_back([&spec, &worst, k]() {
      double error = 0.0;
      for (int copy = 0; copy < 5; ++copy) {
        const std::shared_ptr<const vf::Curve> curve = vf::makeCurve(spec, vf::CurveOptions{});
        const auto parametric = std::dynamic_pointer_cast<const vf::ParametricCurve>(curve);
        if (parametric == nullptr) {
          return;
        }
        for (int i = 0; i < 200; ++i) {
          const double s = 0.03 * i;
          const Vec2 expected(2.0 * std::cos(s) + 0.1 * k, std::sin(s));
          error = std::max(error, (parametric->r(s, static_cast<double>(k)) - expected).norm());
        }
      }
      worst[static_cast<std::size_t>(k)] = error;
    });
  }
  for (std::thread & thread : threads) {
    thread.join();
  }
  for (const double value : worst) {
    EXPECT_LE(value, 1e-15);
  }
}
