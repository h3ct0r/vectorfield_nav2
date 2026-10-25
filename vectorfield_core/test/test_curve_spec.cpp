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

// CurveSpec and makeCurve, the single validation point of curves
// (math_spec §2.6–§2.8).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::Vec2;

namespace
{

vf::CurveSpec polylineSpec(const std::vector<Vec2> & points, bool closed = false)
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kPolyline;
  spec.points = points;
  spec.closed = closed;
  return spec;
}

vf::CurveSpec expressionSpec(const std::string & x, const std::string & y, bool closed = true)
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kExpression;
  spec.x_expression = x;
  spec.y_expression = y;
  spec.s_min = 0.0;
  spec.s_max = closed ? vf::kTwoPi : 1.0;
  spec.closed = closed;
  return spec;
}

vf::CurveSpec builtinSpec(const std::string & shape)
{
  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kBuiltin;
  spec.builtin_shape = shape;
  return spec;
}

std::shared_ptr<const vf::PolylineCurve> makePolyline(
  const vf::CurveSpec & spec, const vf::CurveOptions & options = vf::CurveOptions{})
{
  auto curve = std::dynamic_pointer_cast<const vf::PolylineCurve>(vf::makeCurve(spec, options));
  EXPECT_NE(curve, nullptr);
  return curve;
}

void expectError(
  const vf::CurveSpec & spec, vf::CurveError::Code code, const std::string & needle,
  const vf::CurveOptions & options = vf::CurveOptions{})
{
  try {
    vf::makeCurve(spec, options);
    ADD_FAILURE() << "accepted; expected '" << needle << "'";
  } catch (const vf::CurveError & error) {
    EXPECT_EQ(error.code(), code) << error.what();
    EXPECT_NE(std::string(error.what()).find(needle), std::string::npos) << error.what();
  }
}

double minSegment(const vf::PolylineCurve & curve)
{
  double shortest = vf::kInf;
  for (std::size_t k = 0; k < curve.segmentCount(); ++k) {
    shortest = std::min(shortest, curve.segmentLength(k));
  }
  return shortest;
}

double maxSegment(const vf::PolylineCurve & curve)
{
  double longest = 0.0;
  for (std::size_t k = 0; k < curve.segmentCount(); ++k) {
    longest = std::max(longest, curve.segmentLength(k));
  }
  return longest;
}

}  // namespace

// Field-wise equality (the follower keeps TrackerState across an identical preemption).
TEST(CurveSpec, EqualityIsFieldWise)
{
  vf::CurveSpec base = expressionSpec("a*cos(s)", "sin(s)");
  base.parameter_names = {"a"};
  base.parameter_values = {2.0};
  base.points = {Vec2(0.0, 0.0), Vec2(1.0, 0.0)};
  base.builtin_shape = "ellipse";
  EXPECT_TRUE(base == base);
  EXPECT_FALSE(base != base);
  std::vector<vf::CurveSpec> changed(13, base);
  changed[0].type = vf::CurveSpec::Type::kBuiltin;
  changed[1].closed = !base.closed;
  changed[2].points[1].x() = 1.5;
  changed[3].x_expression = "a*cos(s) ";
  changed[4].y_expression = "2*sin(s)";
  changed[5].s_min = -1.0;
  changed[6].s_max = 7.0;
  changed[7].builtin_shape = "superellipse";
  changed[8].parameter_names = {"b"};
  changed[9].parameter_values = {3.0};
  changed[10].sample_spacing = 0.1;
  changed[11].max_corner_deviation = 0.2;
  changed[12].t0 = 5.0;
  for (std::size_t i = 0; i < changed.size(); ++i) {
    EXPECT_FALSE(changed[i] == base) << "field " << i;
    EXPECT_TRUE(changed[i] != base) << "field " << i;
  }
  EXPECT_TRUE(vf::CurveSpec{} == vf::CurveSpec{});
}

// Result-code names (FollowCurve.action) and the exception hierarchy.
TEST(CurveSpec, CurveErrorCodes)
{
  EXPECT_STREQ(vf::toString(vf::CurveError::Code::kInvalidCurve), "INVALID_CURVE");
  EXPECT_STREQ(vf::toString(vf::CurveError::Code::kUnsupported), "UNSUPPORTED");
  const vf::CurveError error(vf::CurveError::Code::kUnsupported, "message");
  EXPECT_EQ(error.code(), vf::CurveError::Code::kUnsupported);
  EXPECT_STREQ(error.what(), "message");
  EXPECT_THROW(vf::makeCurve(polylineSpec({}), vf::CurveOptions{}), vf::ConfigError);
  EXPECT_THROW(vf::makeCurve(polylineSpec({}), vf::CurveOptions{}), std::invalid_argument);
  // Open + time-varying is UNSUPPORTED; everything else is INVALID_CURVE.
  expectError(
    expressionSpec("s", "0.2*t + s^2", false), vf::CurveError::Code::kUnsupported, "time-varying");
  expectError(
    expressionSpec("cos(s", "sin(s)"), vf::CurveError::Code::kInvalidCurve, "x_expression");
  vf::CurveSpec unknown = builtinSpec("ellipse");
  unknown.type = static_cast<vf::CurveSpec::Type>(7);
  expectError(unknown, vf::CurveError::Code::kInvalidCurve, "unknown curve type 7");
}

// makeCurve validates CurveOptions first (INVALID_CURVE).
TEST(CurveSpec, InvalidOptionsRejected)
{
  const auto check = [](vf::CurveOptions options) {
    expectError(
      builtinSpec("ellipse"), vf::CurveError::Code::kInvalidCurve, "invalid curve options",
      options);
  };
  vf::CurveOptions options;
  options.max_curve_points = 1;
  check(options);
  options = vf::CurveOptions{};
  options.default_sample_spacing = 0.0;
  check(options);
  options = vf::CurveOptions{};
  options.search_window = vf::kNaN;
  check(options);
  options = vf::CurveOptions{};
  options.global_search_trigger = -1.0;
  check(options);
  options = vf::CurveOptions{};
  options.global_search_hysteresis = vf::kInf;
  check(options);
  options = vf::CurveOptions{};
  options.global_search_samples = 3;
  check(options);
  options.global_search_samples = 4;
  EXPECT_NO_THROW(vf::makeCurve(builtinSpec("ellipse"), options));
}

// Every type stores a copy of its spec, and makeCurve(*curve.spec()) rebuilds an equal curve.
TEST(CurveSpec, SpecRoundTripAndPerThreadCopies)
{
  vf::CurveSpec polyline = polylineSpec({Vec2(0.0, 0.0), Vec2(2.0, 0.0), Vec2(2.0, 1.0)});
  vf::CurveSpec expression = expressionSpec("2*cos(s)", "sin(s) + 0*t");
  vf::CurveSpec builtin = builtinSpec("lemniscate_bernoulli");
  builtin.parameter_names = {"a"};
  builtin.parameter_values = {2.0};
  for (const vf::CurveSpec & spec : {polyline, expression, builtin}) {
    SCOPED_TRACE(static_cast<int>(spec.type));
    const std::shared_ptr<const vf::Curve> curve = vf::makeCurve(spec, vf::CurveOptions{});
    ASSERT_NE(curve, nullptr);
    ASSERT_NE(curve->spec(), nullptr);
    EXPECT_TRUE(*curve->spec() == spec);
    const std::shared_ptr<const vf::Curve> copy = vf::makeCurve(*curve->spec(), vf::CurveOptions{});
    ASSERT_NE(copy, nullptr);
    EXPECT_NE(copy.get(), curve.get());
    EXPECT_EQ(copy->isClosed(), curve->isClosed());
    EXPECT_EQ(copy->isTimeVarying(), curve->isTimeVarying());
    EXPECT_DOUBLE_EQ(copy->length(0.3), curve->length(0.3));
    const Vec2 x(0.7, 0.9);
    vf::EvalContext context;
    context.t = 0.3;
    EXPECT_EQ(
      copy->closestPoint(x, context, vf::CurveHint{}).x_star,
      curve->closestPoint(x, context, vf::CurveHint{}).x_star);
  }
  EXPECT_NE(
    std::dynamic_pointer_cast<const vf::PolylineCurve>(vf::makeCurve(polyline, {})), nullptr);
  EXPECT_NE(
    std::dynamic_pointer_cast<const vf::ParametricCurve>(vf::makeCurve(expression, {})), nullptr);
  EXPECT_NE(
    std::dynamic_pointer_cast<const vf::ParametricCurve>(vf::makeCurve(builtin, {})), nullptr);
}

// POLYLINE basics: resampled at the spec's spacing (or the default), endpoints kept, Hinted search.
TEST(CurveSpec, PolylineBasics)
{
  const auto curve = makePolyline(polylineSpec({Vec2(0.0, 0.0), Vec2(1.0, 0.0)}));
  ASSERT_NE(curve, nullptr);
  EXPECT_FALSE(curve->isClosed());
  EXPECT_FALSE(curve->isTimeVarying());
  EXPECT_NEAR(curve->length(0.0), 1.0, 1e-12);
  EXPECT_EQ(curve->size(), 21U);  // ⌈1/0.05⌉ pieces
  EXPECT_LE(maxSegment(*curve), 0.05 + 1e-12);
  EXPECT_EQ(curve->vertices().front(), Vec2(0.0, 0.0));
  EXPECT_EQ(curve->vertices().back(), Vec2(1.0, 0.0));
  EXPECT_EQ(curve->searchPolicy().kind, vf::SearchPolicy::Kind::kHinted);
  EXPECT_EQ(curve->searchPolicy().window, vf::CurveOptions{}.search_window);
  const vf::ClosestPoint cp =
    curve->closestPoint(Vec2(0.5, 0.2), vf::EvalContext{}, vf::CurveHint{});
  ASSERT_TRUE(cp.valid);
  EXPECT_NEAR(cp.D, 0.2, 1e-12);

  vf::CurveSpec coarse = polylineSpec({Vec2(0.0, 0.0), Vec2(1.0, 0.0)});
  coarse.sample_spacing = 0.25;
  EXPECT_EQ(makePolyline(coarse)->size(), 5U);
  vf::CurveOptions options;
  options.default_sample_spacing = 0.5;
  options.search_window = 2.5;
  const auto by_default = makePolyline(polylineSpec({Vec2(0.0, 0.0), Vec2(1.0, 0.0)}), options);
  EXPECT_EQ(by_default->size(), 3U);
  EXPECT_EQ(by_default->searchPolicy().window, 2.5);
}

// POLYLINE rejects: too few points, non-finite values, too many points.
TEST(CurveSpec, PolylineRejects)
{
  const auto invalid = vf::CurveError::Code::kInvalidCurve;
  expectError(polylineSpec({}), invalid, "at least 2 points");
  expectError(polylineSpec({Vec2(1.0, 1.0)}), invalid, "at least 2 points");
  expectError(polylineSpec({Vec2(0.0, 0.0), Vec2(vf::kNaN, 1.0)}), invalid, "not finite");
  expectError(polylineSpec({Vec2(1.0, 2.0), Vec2(1.0, 2.0)}), invalid, "2 distinct points");
  expectError(polylineSpec({Vec2(0.0, 0.0), Vec2(1.0, 0.0)}, true), invalid, "3 distinct points");
  // dedupe keeps both endpoints of an open polyline, even closer than 1 mm.
  EXPECT_NO_THROW(vf::makeCurve(polylineSpec({Vec2(0.0, 0.0), Vec2(0.0005, 0.0)}), {}));
  vf::CurveSpec bad_spacing = polylineSpec({Vec2(0.0, 0.0), Vec2(1.0, 0.0)});
  bad_spacing.sample_spacing = vf::kNaN;
  expectError(bad_spacing, invalid, "must be finite");
  vf::CurveSpec bad_deviation = polylineSpec({Vec2(0.0, 0.0), Vec2(1.0, 0.0)});
  bad_deviation.max_corner_deviation = vf::kInf;
  expectError(bad_deviation, invalid, "must be finite");

  // 100 m at 0.05 m needs 2001 points; max_curve_points is checked before resampling allocates.
  vf::CurveOptions options;
  options.max_curve_points = 1000;
  expectError(
    polylineSpec({Vec2(0.0, 0.0), Vec2(100.0, 0.0)}), invalid, "max_curve_points", options);
  vf::CurveSpec tiny_spacing = polylineSpec({Vec2(0.0, 0.0), Vec2(100.0, 0.0)});
  tiny_spacing.sample_spacing = 1e-12;
  expectError(tiny_spacing, invalid, "max_curve_points");
  vf::CurveSpec coarse = polylineSpec({Vec2(0.0, 0.0), Vec2(100.0, 0.0)});
  coarse.sample_spacing = 0.2;
  EXPECT_NO_THROW(vf::makeCurve(coarse, options));
}

// Conditioning order dedupe → fillet → densify (math_spec §2.8):
// - dedupe first: a near-duplicate at the corner would otherwise cap the fillet radius at 0;
// - fillet before densify: densified segments would cap R at spacing/(2 tan(θ/2)), a 1 cm
//   deviation instead of δ = 5 cm.
TEST(CurveSpec, PolylineConditioningOrder)
{
  constexpr double kDelta = 0.05;
  const Vec2 corner(2.0, 0.0);
  for (const bool near_duplicate : {false, true}) {
    SCOPED_TRACE(near_duplicate ? "with a near-duplicate corner vertex" : "plain corner");
    std::vector<Vec2> points{Vec2(0.0, 0.0), corner};
    if (near_duplicate) {
      points.emplace_back(2.0, 0.0005);
    }
    points.emplace_back(2.0, 2.0);
    vf::CurveSpec spec = polylineSpec(points);
    spec.max_corner_deviation = kDelta;
    const auto curve = makePolyline(spec);
    ASSERT_NE(curve, nullptr);
    // Endpoints preserved, segments at most the spacing, no segment shorter than the dedupe step.
    EXPECT_EQ(curve->vertices().front(), Vec2(0.0, 0.0));
    EXPECT_EQ(curve->vertices().back(), Vec2(2.0, 2.0));
    EXPECT_LE(maxSegment(*curve), 0.05 + 1e-12);
    EXPECT_GE(minSegment(*curve), vf::kDedupeDistance);
    // The corner is cut by about δ.
    const vf::ClosestPoint cp = curve->closestPoint(corner, vf::EvalContext{}, vf::CurveHint{});
    ASSERT_TRUE(cp.valid);
    EXPECT_GE(cp.D, 0.9 * kDelta);
    EXPECT_LE(cp.D, 1.2 * kDelta);
  }
  // Without max_corner_deviation the corner is kept exactly.
  const auto sharp = makePolyline(polylineSpec({Vec2(0.0, 0.0), corner, Vec2(2.0, 2.0)}));
  ASSERT_NE(sharp, nullptr);
  EXPECT_NEAR(sharp->closestPoint(corner, vf::EvalContext{}, vf::CurveHint{}).D, 0.0, 1e-12);
}

// A closed POLYLINE with a closing duplicate: dedupe drops it, and the curve closes implicitly.
TEST(CurveSpec, PolylineClosedWithClosingDuplicate)
{
  const auto curve = makePolyline(polylineSpec(
    {Vec2(0.0, 0.0), Vec2(1.0, 0.0), Vec2(1.0, 1.0), Vec2(0.0, 1.0), Vec2(0.0, 0.0)}, true));
  ASSERT_NE(curve, nullptr);
  EXPECT_TRUE(curve->isClosed());
  EXPECT_NEAR(curve->length(0.0), 4.0, 1e-12);
  EXPECT_EQ(curve->size(), 80U);
  EXPECT_LE(maxSegment(*curve), 0.05 + 1e-12);
}

// The other types reach their own validation through makeCurve.
TEST(CurveSpec, DispatchesByType)
{
  EXPECT_TRUE(vf::makeCurve(builtinSpec("ellipse"), vf::CurveOptions{})->isClosed());
  expectError(
    builtinSpec("triangle"), vf::CurveError::Code::kInvalidCurve, "unknown builtin_shape");
  const auto open = vf::makeCurve(expressionSpec("s", "s^2", false), vf::CurveOptions{});
  EXPECT_FALSE(open->isClosed());
  EXPECT_NEAR(open->length(0.0), 0.5 * std::sqrt(5.0) + 0.25 * std::asinh(2.0), 1e-8);
}
