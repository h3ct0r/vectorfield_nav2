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

// Built-in shapes (math_spec §2.7; oracles O11 and O13 of §12.1).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_curves.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::Vec2;

namespace
{

using Parameters = std::vector<std::pair<std::string, double>>;

vf::CurveSpec builtinSpec(const std::string & shape, const Parameters & parameters = {})
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kBuiltin;
  spec.builtin_shape = shape;
  for (const auto & parameter : parameters) {
    spec.parameter_names.push_back(parameter.first);
    spec.parameter_values.push_back(parameter.second);
  }
  return spec;
}

std::shared_ptr<const vf::ParametricCurve> makeBuiltin(
  const std::string & shape, const Parameters & parameters = {})
{
  auto curve = std::dynamic_pointer_cast<const vf::ParametricCurve>(
    vf::makeCurve(builtinSpec(shape, parameters), vf::CurveOptions{}));
  EXPECT_NE(curve, nullptr);
  return curve;
}

void expectInvalid(const vf::CurveSpec & spec, const std::string & needle)
{
  try {
    vf::makeCurve(spec, vf::CurveOptions{});
    ADD_FAILURE() << "accepted; expected an error mentioning '" << needle << "'";
  } catch (const vf::CurveError & error) {
    EXPECT_EQ(error.code(), vf::CurveError::Code::kInvalidCurve);
    EXPECT_NE(std::string(error.what()).find(needle), std::string::npos) << error.what();
  }
}

// Superellipse polar radius ρ(s) (math_spec §2.7).
double superellipseRho(double s, double a, double b, double n)
{
  return std::pow(
    std::pow(std::abs(std::cos(s) / a), n) + std::pow(std::abs(std::sin(s) / b), n), -1.0 / n);
}

}  // namespace

// The table of shapes: the seven names in Curve.msg's order, with defaults and pose parameters.
TEST(BuiltinCurves, ShapeTable)
{
  const vf::Span<const vf::BuiltinShapeInfo> shapes = vf::builtinShapes();
  const std::array<const char *, 7> names{
    "ellipse",   "lemniscate_bernoulli", "lemniscate_gerono", "superellipse", "rounded_rectangle",
    "lissajous", "regular_polygon"};
  ASSERT_EQ(shapes.size(), names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    SCOPED_TRACE(names[i]);
    EXPECT_STREQ(shapes[i].name, names[i]);
    EXPECT_NE(std::string(shapes[i].formula), "");
    EXPECT_EQ(vf::detail::findBuiltinShape(names[i]), &shapes[i]);
    std::vector<std::string> parameters;
    for (const vf::BuiltinParameterInfo & parameter : shapes[i].parameters) {
      parameters.emplace_back(parameter.name);
    }
    for (const char * pose : {"cx", "cy", "yaw"}) {
      EXPECT_NE(std::find(parameters.begin(), parameters.end(), pose), parameters.end()) << pose;
    }
  }
  EXPECT_EQ(vf::detail::findBuiltinShape("circle"), nullptr);
  const auto defaultOf = [&shapes](const char * shape, const char * name) {
    for (const vf::BuiltinShapeInfo & info : shapes) {
      if (std::string(info.name) == shape) {
        for (const vf::BuiltinParameterInfo & parameter : info.parameters) {
          if (std::string(parameter.name) == name) {
            return parameter.default_value;
          }
        }
      }
    }
    return vf::kNaN;
  };
  // Defaults of math_spec §2.7.
  EXPECT_EQ(defaultOf("ellipse", "a"), 1.0);
  EXPECT_EQ(defaultOf("lemniscate_gerono", "b"), 0.5);
  EXPECT_EQ(defaultOf("superellipse", "n"), 4.0);
  EXPECT_EQ(defaultOf("rounded_rectangle", "width"), 2.0);
  EXPECT_EQ(defaultOf("rounded_rectangle", "height"), 1.0);
  EXPECT_EQ(defaultOf("rounded_rectangle", "radius"), 0.2);
  EXPECT_EQ(defaultOf("lissajous", "fy"), 2.0);
  EXPECT_EQ(defaultOf("lissajous", "phase"), vf::kHalfPi);
  EXPECT_EQ(defaultOf("regular_polygon", "radius"), 1.0);
  EXPECT_EQ(defaultOf("regular_polygon", "sides"), 6.0);
  EXPECT_EQ(defaultOf("regular_polygon", "corner_radius"), 0.1);
}

// Every built-in is closed and static, ignores spec.closed and passes the regularity checks at its
// defaults (O11, positive part).
TEST(BuiltinCurves, DefaultsAreClosedStaticAndRegular)
{
  for (const vf::BuiltinShapeInfo & shape : vf::builtinShapes()) {
    SCOPED_TRACE(shape.name);
    vf::CurveSpec spec = builtinSpec(shape.name);
    spec.closed = false;  // ignored for BUILTIN
    const auto curve =
      std::dynamic_pointer_cast<const vf::ParametricCurve>(vf::makeCurve(spec, vf::CurveOptions{}));
    ASSERT_NE(curve, nullptr);
    EXPECT_TRUE(curve->isClosed());
    EXPECT_FALSE(curve->isTimeVarying());
    EXPECT_EQ(curve->uMin(), 0.0);
    EXPECT_EQ(curve->uMax(), vf::kTwoPi);
    EXPECT_GT(curve->length(0.0), 0.0);
    EXPECT_EQ(curve->drdt(1.0, 0.0), Vec2::Zero());
    ASSERT_NE(curve->spec(), nullptr);
    EXPECT_EQ(curve->spec()->builtin_shape, shape.name);
  }
}

// O13: r(0) and r(π/2) of every built-in at its defaults match the formulas within 1e-12.
TEST(BuiltinCurves, O13FormulasAtZeroAndHalfPi)
{
  const double h = vf::kHalfPi;
  const double alpha = vf::kPi / 6.0;
  const std::vector<std::pair<std::string, std::array<Vec2, 2>>> expected{
    {"ellipse", {Vec2(1.0, 0.0), Vec2(std::cos(h), std::sin(h))}},
    {"lemniscate_bernoulli",
     {Vec2(1.0, 0.0),
      Vec2(std::cos(h), std::sin(h) * std::cos(h)) / (1.0 + std::sin(h) * std::sin(h))}},
    {"lemniscate_gerono", {Vec2(1.0, 0.0), Vec2(std::cos(h), 0.5 * std::sin(2.0 * h))}},
    {"superellipse",
     {Vec2(1.0, 0.0), superellipseRho(h, 1.0, 1.0, 4.0) * Vec2(std::cos(h), std::sin(h))}},
    // ς(π/2) = L/4: the middle of the top side.
    {"rounded_rectangle", {Vec2(1.0, 0.0), Vec2(0.0, 0.5)}},
    {"lissajous", {Vec2(std::sin(h), 0.0), Vec2(std::sin(h + h), std::sin(2.0 * h))}},
    // Hexagon: ς(π/2) = 1.5 side units from the +x side midpoint, the middle of corner arc 1.
    {"regular_polygon", {Vec2(std::cos(alpha), 0.0), Vec2(0.0, 1.0 - 0.1 / std::cos(alpha) + 0.1)}},
  };
  for (const auto & [shape, points] : expected) {
    SCOPED_TRACE(shape);
    const auto curve = makeBuiltin(shape);
    ASSERT_NE(curve, nullptr);
    EXPECT_LE((curve->r(0.0, 0.0) - points[0]).norm(), 1e-12);
    EXPECT_LE((curve->r(h, 0.0) - points[1]).norm(), 1e-12)
      << curve->r(h, 0.0).transpose() << " vs " << points[1].transpose();
  }
}

// O13: rounded_rectangle and regular_polygon start at the +x side midpoint, are continuous
// (max step over 2e5 samples below 2L/2e5) and have the stated length (1e-9).
TEST(BuiltinCurves, O13ArcLengthShapes)
{
  struct Case
  {
    std::string shape;
    Parameters parameters;
    Vec2 start;
    double length;
  };
  const auto rectangleLength = [](double w, double h, double rho) {
    return 2.0 * (w - 2.0 * rho) + 2.0 * (h - 2.0 * rho) + vf::kTwoPi * rho;
  };
  const auto polygonLength = [](double radius, double n, double rc) {
    const double side = 2.0 * radius * std::sin(vf::kPi / n);
    return n * (side - 2.0 * rc * std::tan(vf::kPi / n)) + vf::kTwoPi * rc;
  };
  const std::vector<Case> cases{
    {"rounded_rectangle", {}, Vec2(1.0, 0.0), rectangleLength(2.0, 1.0, 0.2)},
    {"rounded_rectangle",
     {{"width", 3.0}, {"height", 1.0}, {"radius", 0.5}},
     Vec2(1.5, 0.0),
     rectangleLength(3.0, 1.0, 0.5)},
    {"rounded_rectangle",
     {{"width", 0.5}, {"height", 2.0}, {"radius", 0.01}},
     Vec2(0.25, 0.0),
     rectangleLength(0.5, 2.0, 0.01)},
    {"regular_polygon", {}, Vec2(std::cos(vf::kPi / 6.0), 0.0), polygonLength(1.0, 6.0, 0.1)},
    {"regular_polygon",
     {{"sides", 3.0}, {"radius", 2.0}, {"corner_radius", 0.3}},
     Vec2(2.0 * std::cos(vf::kPi / 3.0), 0.0),
     polygonLength(2.0, 3.0, 0.3)},
    {"regular_polygon",
     {{"sides", 4.0}, {"corner_radius", std::cos(vf::kPi / 4.0)}},
     Vec2(std::cos(vf::kPi / 4.0), 0.0),
     polygonLength(1.0, 4.0, std::cos(vf::kPi / 4.0))},
    {"regular_polygon",
     {{"sides", 7.0}, {"corner_radius", 0.02}},
     Vec2(std::cos(vf::kPi / 7.0), 0.0),
     polygonLength(1.0, 7.0, 0.02)},
  };
  for (const Case & c : cases) {
    SCOPED_TRACE(c.shape + " " + std::to_string(c.parameters.size()) + " parameters");
    const auto curve = makeBuiltin(c.shape, c.parameters);
    ASSERT_NE(curve, nullptr);
    EXPECT_LE((curve->r(0.0, 0.0) - c.start).norm(), 1e-12);
    EXPECT_NEAR(curve->length(0.0), c.length, 1e-9);
    // The start is the +x side midpoint: the tangent there is +y.
    EXPECT_LE((curve->drdu(0.0, 0.0).normalized() - Vec2(0.0, 1.0)).norm(), 1e-12);
    // Speed L/(2π) everywhere (arc-length parametrisation).
    constexpr int kSamples = 200000;
    double max_step = 0.0;
    Vec2 previous = curve->r(0.0, 0.0);
    for (int k = 1; k <= kSamples; ++k) {
      const double u = vf::kTwoPi * k / kSamples;
      const Vec2 point = curve->r(k == kSamples ? 0.0 : u, 0.0);
      max_step = std::max(max_step, (point - previous).norm());
      previous = point;
      if (k % 1000 == 0) {
        ASSERT_NEAR(curve->drdu(u, 0.0).norm(), c.length / vf::kTwoPi, 1e-12);
      }
    }
    EXPECT_LT(max_step, 2.0 * c.length / kSamples);
  }
}

// O13: the superellipse speed is at least ρ(s) for n in {2, 4, 8} (1e-9).
TEST(BuiltinCurves, O13SuperellipseSpeedAtLeastRho)
{
  for (const double n : {2.0, 4.0, 8.0}) {
    const std::array<std::pair<double, double>, 2> sizes{{{1.0, 1.0}, {2.0, 0.7}}};
    for (const auto & [a, b] : sizes) {
      SCOPED_TRACE("n = " + std::to_string(n) + ", a = " + std::to_string(a));
      const auto curve = makeBuiltin("superellipse", {{"a", a}, {"b", b}, {"n", n}});
      ASSERT_NE(curve, nullptr);
      for (int k = 0; k < 20000; ++k) {
        const double s = vf::kTwoPi * k / 20000.0;
        const double rho = superellipseRho(s, a, b, n);
        ASSERT_LE((curve->r(s, 0.0) - rho * Vec2(std::cos(s), std::sin(s))).norm(), 1e-12);
        ASSERT_GE(curve->drdu(s, 0.0).norm(), rho - 1e-9) << "s = " << s;
      }
    }
  }
}

// The analytic derivatives of every shape (with a pose) match central differences of r.
TEST(BuiltinCurves, AnalyticDerivativesMatchFiniteDifferences)
{
  const std::vector<std::pair<std::string, Parameters>> cases{
    {"ellipse", {{"a", 2.0}, {"b", 0.5}}},
    {"lemniscate_bernoulli", {{"a", 1.7}}},
    {"lemniscate_gerono", {{"a", 4.0}, {"b", 1.5}}},
    {"superellipse", {{"a", 1.5}, {"b", 0.8}, {"n", 2.0}}},
    {"superellipse", {{"n", 6.5}}},
    {"rounded_rectangle", {{"width", 3.0}, {"height", 2.0}, {"radius", 0.4}}},
    {"lissajous", {{"ax", 2.0}, {"ay", 1.0}, {"fx", 3.0}, {"fy", 2.0}, {"phase", 0.3}}},
    {"regular_polygon", {{"sides", 5.0}, {"corner_radius", 0.2}}},
  };
  const Parameters pose{{"cx", 0.7}, {"cy", -1.2}, {"yaw", 0.9}};
  constexpr double kH = 1e-6;
  for (auto [shape, parameters] : cases) {
    SCOPED_TRACE(shape);
    parameters.insert(parameters.end(), pose.begin(), pose.end());
    const auto curve = makeBuiltin(shape, parameters);
    ASSERT_NE(curve, nullptr);
    for (int k = 0; k < 997; ++k) {
      const double u = vf::kTwoPi * (k + 0.37) / 997.0;
      const Vec2 numeric = (curve->r(u + kH, 0.0) - curve->r(u - kH, 0.0)) / (2.0 * kH);
      const Vec2 analytic = curve->drdu(u, 0.0);
      // 1e-5: the arc-length shapes are only C1, and a step across a curvature jump costs O(h).
      ASSERT_LE((numeric - analytic).norm(), 1e-5 * std::max(1.0, analytic.norm())) << "u = " << u;
    }
  }
}

// The pose [cx, cy] + R(yaw)(·) maps the curve-frame formula.
TEST(BuiltinCurves, PoseMapping)
{
  const double yaw = 0.6;
  const Vec2 c(1.5, -2.0);
  const auto curve =
    makeBuiltin("ellipse", {{"a", 2.0}, {"b", 1.0}, {"cx", c.x()}, {"cy", c.y()}, {"yaw", yaw}});
  ASSERT_NE(curve, nullptr);
  for (const double u : {0.0, 0.4, 2.0, 4.5}) {
    const Vec2 local(2.0 * std::cos(u), std::sin(u));
    EXPECT_LE((curve->r(u, 0.0) - (c + vf::rotation(yaw) * local)).norm(), 1e-12);
    const Vec2 local_d(-2.0 * std::sin(u), std::cos(u));
    EXPECT_LE((curve->drdu(u, 0.0) - vf::rotation(yaw) * local_d).norm(), 1e-12);
  }
}

// Bernoulli speed a/sqrt(1 + sin² s) (math_spec §2.7, Appendix B: 4e-12).
TEST(BuiltinCurves, LemniscateBernoulliSpeed)
{
  const auto curve = makeBuiltin("lemniscate_bernoulli", {{"a", 2.0}});
  ASSERT_NE(curve, nullptr);
  for (int k = 0; k < 1000; ++k) {
    const double s = vf::kTwoPi * k / 1000.0;
    EXPECT_NEAR(
      curve->drdu(s, 0.0).norm(), 2.0 / std::sqrt(1.0 + std::sin(s) * std::sin(s)), 1e-12);
  }
}

// Parameter rules: unknown names, duplicates, non-finite or out-of-range values are rejected.
TEST(BuiltinCurves, ParameterValidation)
{
  expectInvalid(builtinSpec("circle"), "unknown builtin_shape");
  expectInvalid(builtinSpec(""), "unknown builtin_shape");
  expectInvalid(builtinSpec("ellipse", {{"c", 1.0}}), "unknown parameter 'c'");
  expectInvalid(builtinSpec("ellipse", {{"a", 1.0}, {"a", 2.0}}), "duplicate parameter");
  expectInvalid(builtinSpec("ellipse", {{"a", vf::kNaN}}), "not finite");
  expectInvalid(builtinSpec("ellipse", {{"cx", vf::kInf}}), "not finite");
  expectInvalid(builtinSpec("ellipse", {{"a", 0.0}}), "must be > 0");
  expectInvalid(builtinSpec("ellipse", {{"b", -1.0}}), "must be > 0");
  expectInvalid(builtinSpec("lemniscate_bernoulli", {{"a", -2.0}}), "must be > 0");
  expectInvalid(builtinSpec("lemniscate_gerono", {{"b", 0.0}}), "must be > 0");
  expectInvalid(builtinSpec("superellipse", {{"n", 1.5}}), "must be >= 2");
  expectInvalid(builtinSpec("superellipse", {{"n", vf::kNaN}}), "not finite");
  expectInvalid(builtinSpec("lissajous", {{"fx", 1.5}}), "must be an integer");
  expectInvalid(builtinSpec("lissajous", {{"fy", 0.0}}), "must be an integer");
  expectInvalid(builtinSpec("lissajous", {{"fx", 2.0}, {"fy", 4.0}}), "gcd(fx, fy) = 2");
  expectInvalid(builtinSpec("lissajous", {{"fx", 3.0}, {"fy", 6.0}}), "gcd(fx, fy) = 3");
  expectInvalid(builtinSpec("lissajous", {{"fx", 2e6}}), "must be an integer");
  expectInvalid(builtinSpec("lissajous", {{"ax", 0.0}}), "must be > 0");
  expectInvalid(builtinSpec("rounded_rectangle", {{"radius", 0.6}}), "radius");
  expectInvalid(builtinSpec("rounded_rectangle", {{"radius", 0.0}}), "must be > 0");
  expectInvalid(builtinSpec("rounded_rectangle", {{"width", -1.0}}), "must be > 0");
  expectInvalid(builtinSpec("regular_polygon", {{"sides", 2.0}}), "must be an integer in [3");
  expectInvalid(builtinSpec("regular_polygon", {{"sides", 4.5}}), "must be an integer");
  expectInvalid(builtinSpec("regular_polygon", {{"corner_radius", 0.0}}), "must be > 0");
  expectInvalid(builtinSpec("regular_polygon", {{"corner_radius", 0.87}}), "corner_radius");
  vf::CurveSpec mismatch = builtinSpec("ellipse", {{"a", 1.0}});
  mismatch.parameter_values.push_back(2.0);
  expectInvalid(mismatch, "parameter names but");
  vf::CurveSpec bad_t0 = builtinSpec("ellipse");
  bad_t0.t0 = vf::kNaN;
  expectInvalid(bad_t0, "t0");
  // Boundary values are accepted.
  EXPECT_NO_THROW(
    vf::makeCurve(
      builtinSpec("rounded_rectangle", {{"width", 2.0}, {"height", 1.0}, {"radius", 0.5}}),
      vf::CurveOptions{}));
  EXPECT_NO_THROW(vf::makeCurve(builtinSpec("superellipse", {{"n", 2.0}}), vf::CurveOptions{}));
  EXPECT_NO_THROW(
    vf::makeCurve(builtinSpec("lissajous", {{"fx", 3.0}, {"fy", 2.0}}), vf::CurveOptions{}));
}

// Domain rules: [0, 0] is [0, 2π); otherwise exactly one period, starting anywhere.
TEST(BuiltinCurves, DomainRules)
{
  vf::CurveSpec shifted = builtinSpec("ellipse", {{"a", 2.0}});
  shifted.s_min = vf::kHalfPi;
  shifted.s_max = vf::kHalfPi + vf::kTwoPi;
  const auto curve = std::dynamic_pointer_cast<const vf::ParametricCurve>(
    vf::makeCurve(shifted, vf::CurveOptions{}));
  ASSERT_NE(curve, nullptr);
  EXPECT_EQ(curve->uMin(), vf::kHalfPi);
  EXPECT_LE((curve->pointAt(0.0, 0.0) - Vec2(0.0, 1.0)).norm(), 1e-12);
  for (const auto & [s_min, s_max] : std::vector<std::pair<double, double>>{
         {0.0, vf::kPi}, {0.0, 2.0 * vf::kTwoPi}, {1.0, 1.0}, {2.0, 1.0}, {vf::kNaN, 1.0}}) {
    vf::CurveSpec spec = builtinSpec("ellipse");
    spec.s_min = s_min;
    spec.s_max = s_max;
    expectInvalid(spec, "domain");
  }
}

// O11 built-in rejects: lissajous 1:2 with phase π/4 (a parabola traced back and forth),
// superellipse n = 1.5, lissajous fx = 2, fy = 4.
TEST(BuiltinCurves, O11BuiltinRejects)
{
  expectInvalid(builtinSpec("lissajous", {{"phase", vf::kPi / 4.0}}), "not regular");
  expectInvalid(builtinSpec("superellipse", {{"n", 1.5}}), "n");
  expectInvalid(builtinSpec("lissajous", {{"fx", 2.0}, {"fy", 4.0}}), "gcd");
}

// A built-in is a ParametricCurve built by the internal factory.
TEST(BuiltinCurves, InternalFactory)
{
  const std::unique_ptr<vf::ParametricCurve> curve =
    vf::detail::makeBuiltinCurve(builtinSpec("lemniscate_gerono"), vf::CurveOptions{});
  ASSERT_NE(curve, nullptr);
  EXPECT_EQ(curve->spec(), nullptr);
  EXPECT_LE((curve->r(0.3, 0.0) - Vec2(std::cos(0.3), 0.5 * std::sin(0.6))).norm(), 1e-15);
}
