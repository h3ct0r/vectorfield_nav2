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

// ParametricCurve: closest-point search, arc length, rays, time-varying curves and regularity
// (math_spec §2.4–§2.6; oracles O1, O2, O7, O11 of §12.1; upstream audit U04, U05, U07).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/status.hpp"
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

vf::CurveSpec expressionSpec(
  const std::string & x, const std::string & y, double s_min, double s_max, bool closed,
  const Parameters & parameters = {})
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

std::shared_ptr<const vf::ParametricCurve> makeParametric(
  const vf::CurveSpec & spec, const vf::CurveOptions & options = vf::CurveOptions{})
{
  const std::shared_ptr<const vf::Curve> curve = vf::makeCurve(spec, options);
  auto parametric = std::dynamic_pointer_cast<const vf::ParametricCurve>(curve);
  EXPECT_NE(parametric, nullptr);
  return parametric;
}

vf::CurveHint hintAt(double u)
{
  vf::CurveHint hint;
  hint.s = u;
  hint.valid = true;
  return hint;
}

vf::EvalContext contextAt(double t = 0.0)
{
  vf::EvalContext context;
  context.t = t;
  return context;
}

// Axis-aligned box around the curve at time t, enlarged by `margin`.
std::pair<Vec2, Vec2> boundingBox(const vf::ParametricCurve & curve, double t, double margin)
{
  Vec2 lo(vf::kInf, vf::kInf);
  Vec2 hi(-vf::kInf, -vf::kInf);
  constexpr int kSamples = 2000;
  for (int k = 0; k <= kSamples; ++k) {
    const double u = curve.uMin() + (curve.uMax() - curve.uMin()) * k / kSamples;
    const Vec2 p = curve.r(u, t);
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  return {lo - Vec2(margin, margin), hi + Vec2(margin, margin)};
}

// O1 on one result: |nᵀT| <= 1e-12, |‖T‖ − 1| <= 1e-12, n zero or unit.
void expectOrthonormal(const vf::ClosestPoint & cp)
{
  ASSERT_TRUE(cp.valid);
  EXPECT_LE(std::abs(cp.n.dot(cp.T)), 1e-12);
  EXPECT_LE(std::abs(cp.T.norm() - 1.0), 1e-12);
  if (cp.n.norm() != 0.0) {
    EXPECT_LE(std::abs(cp.n.norm() - 1.0), 1e-12);
  }
  EXPECT_GE(cp.D, 0.0);
}

// Reference Φ = v_r(−G n + H T) (static field, dir = +1).
Vec2 staticField(const vf::ClosestPoint & cp, const vf::GainFunction & gain, double v_r)
{
  return v_r * (-gain.G(cp.D) * cp.n + gain.H(cp.D) * cp.T);
}

// A circle of radius R about the origin with numerical derivatives, counting r() calls
// (single-threaded tests only). `time_varying` selects the window rule without changing the shape.
class CountingCircle final : public vf::ParametricCurve
{
public:
  CountingCircle(double radius, bool time_varying, const vf::CurveOptions & options)
  : ParametricCurve(0.0, vf::kTwoPi, true, time_varying, options), radius_(radius)
  {
    initialize(0.0);
  }

  Vec2 r(double u, double /*t*/) const noexcept override
  {
    ++count_;
    return radius_ * Vec2(std::cos(u), std::sin(u));
  }

  long count() const noexcept { return count_; }
  void resetCount() const noexcept { count_ = 0; }

private:
  double radius_;
  mutable long count_{0};
};

// An open parabola (u, u²/2) on [−1, 2] that records the parameter range r() was called with.
class DomainRecorder final : public vf::ParametricCurve
{
public:
  explicit DomainRecorder(const vf::CurveOptions & options)
  : ParametricCurve(-1.0, 2.0, false, false, options)
  {
    initialize(0.0);
    resetRange();
  }

  Vec2 r(double u, double /*t*/) const noexcept override
  {
    min_u_ = std::min(min_u_, u);
    max_u_ = std::max(max_u_, u);
    return Vec2(u, 0.5 * u * u);
  }

  void resetRange() const noexcept
  {
    min_u_ = vf::kInf;
    max_u_ = -vf::kInf;
  }
  double minU() const noexcept { return min_u_; }
  double maxU() const noexcept { return max_u_; }

private:
  mutable double min_u_{vf::kInf};
  mutable double max_u_{-vf::kInf};
};

// A closed unit circle whose r() is NaN on a small interval between global samples.
class HoleCircle final : public vf::ParametricCurve
{
public:
  explicit HoleCircle(const vf::CurveOptions & options)
  : ParametricCurve(0.0, vf::kTwoPi, true, false, options)
  {
    initialize(0.0);
  }

  Vec2 r(double u, double /*t*/) const noexcept override
  {
    if (u > 0.1001 && u < 0.1002) {
      return Vec2(vf::kNaN, vf::kNaN);
    }
    return Vec2(std::cos(u), std::sin(u));
  }
};

// Complete elliptic integral of the second kind E(m), m = k², by the AGM.
double ellipticE(double m)
{
  double a = 1.0;
  double b = std::sqrt(1.0 - m);
  double c = std::sqrt(m);
  double sum = 0.5 * c * c;
  double power = 0.5;
  for (int n = 0; n < 20 && std::abs(c) > 1e-17; ++n) {
    const double a_next = 0.5 * (a + b);
    const double b_next = std::sqrt(a * b);
    c = 0.5 * (a - b);
    a = a_next;
    b = b_next;
    power *= 2.0;
    sum += power * c * c;
  }
  const double K = vf::kPi / (2.0 * a);
  return K * (1.0 - sum);
}

// ∫_a^b f by composite Simpson with n (even) intervals.
template <typename F>
double simpson(F f, double a, double b, int n)
{
  const double h = (b - a) / n;
  double sum = f(a) + f(b);
  for (int i = 1; i < n; ++i) {
    sum += (i % 2 == 1 ? 4.0 : 2.0) * f(a + h * i);
  }
  return sum * h / 3.0;
}

}  // namespace

// O1: random x around every built-in and several expressions, with and without hints.
TEST(ParametricCurve, O1OrthogonalityBuiltinsAndExpressions)
{
  std::vector<std::pair<std::string, vf::CurveSpec>> cases;
  for (const vf::BuiltinShapeInfo & shape : vf::builtinShapes()) {
    cases.emplace_back(shape.name, builtinSpec(shape.name));
  }
  cases.emplace_back(
    "ellipse expression", expressionSpec("2*cos(s)", "sin(s)", 0, vf::kTwoPi, true));
  cases.emplace_back(
    "open cubic expression", expressionSpec("s", "0.3*s^2 - 0.2*s^3", -1.5, 1.5, false));
  cases.emplace_back(
    "time-varying expression",
    expressionSpec("(1 + 0.2*t)*cos(s)", "sin(s) + 0.1*t", 0, vf::kTwoPi, true));

  // "Random" means at least 1e4 seeded samples (math_spec §12): 1500 per curve, 15000 in total.
  std::mt19937_64 rng(20261013);
  constexpr int kSamples = 1500;
  for (const auto & [label, spec] : cases) {
    SCOPED_TRACE(label);
    const auto curve = makeParametric(spec);
    ASSERT_NE(curve, nullptr);
    const double t = curve->isTimeVarying() ? 1.5 : 0.0;
    const auto box = boundingBox(*curve, t, 1.0);
    std::uniform_real_distribution<double> ux(box.first.x(), box.second.x());
    std::uniform_real_distribution<double> uy(box.first.y(), box.second.y());
    vf::CurveHint hint;
    for (int i = 0; i < kSamples; ++i) {
      const Vec2 x(ux(rng), uy(rng));
      // Even samples initialise (no hint); odd samples chain the previous result as hint.
      const vf::CurveHint use = i % 2 == 0 ? vf::CurveHint{} : hint;
      const vf::ClosestPoint cp = curve->closestPoint(x, contextAt(t), use);
      expectOrthonormal(cp);
      if (::testing::Test::HasFailure()) {
        FAIL() << "at x = (" << x.x() << ", " << x.y() << "), sample " << i;
      }
      hint = vf::makeHint(cp);
    }
  }
}

// O2: points exactly on parametric curves: ‖T‖ = 1, n = 0, ‖Φ‖ = v_r.
TEST(ParametricCurve, O2DegeneratePointsOnCurve)
{
  const vf::GainFunction gain(vf::GainType::kAtan, 2.0);
  constexpr double kVr = 0.4;
  std::vector<vf::CurveSpec> specs;
  for (const vf::BuiltinShapeInfo & shape : vf::builtinShapes()) {
    specs.push_back(builtinSpec(shape.name));
  }
  specs.push_back(expressionSpec("s", "0.3*s^2 - 0.2*s^3", -1.5, 1.5, false));
  specs.push_back(expressionSpec("2*cos(s)", "sin(s)", 0, vf::kTwoPi, true));
  std::mt19937_64 rng(7);
  for (const vf::CurveSpec & spec : specs) {
    SCOPED_TRACE(spec.builtin_shape.empty() ? spec.x_expression : spec.builtin_shape);
    const auto curve = makeParametric(spec);
    ASSERT_NE(curve, nullptr);
    std::uniform_real_distribution<double> uu(curve->uMin(), curve->uMax());
    std::vector<double> parameters{curve->uMin(), curve->uMax()};
    for (int i = 0; i < 50; ++i) {
      parameters.push_back(uu(rng));
    }
    for (const double u : parameters) {
      const Vec2 x = curve->r(u, 0.0);
      for (const bool with_hint : {false, true}) {
        const vf::ClosestPoint cp =
          curve->closestPoint(x, contextAt(), with_hint ? hintAt(u) : vf::CurveHint{});
        ASSERT_TRUE(cp.valid);
        EXPECT_LE(std::abs(cp.T.norm() - 1.0), 1e-12) << "u = " << u;
        EXPECT_EQ(cp.n, Vec2::Zero()) << "u = " << u << ", D = " << cp.D;
        EXPECT_LT(cp.D, vf::kEpsD);
        EXPECT_LE(std::abs(staticField(cp, gain, kVr).norm() - kVr), 1e-12);
      }
    }
  }
}

// O7: the `ellipse` built-in with a = b = 1.3 centred at (0.4, −0.2) is a circle.
TEST(ParametricCurve, O7CircleClosestPoint)
{
  const Vec2 c(0.4, -0.2);
  constexpr double kR = 1.3;
  constexpr double kVr = 0.4;
  const auto curve =
    makeParametric(builtinSpec("ellipse", {{"a", kR}, {"b", kR}, {"cx", c.x()}, {"cy", c.y()}}));
  ASSERT_NE(curve, nullptr);
  const vf::GainFunction gain(vf::GainType::kAtan, 2.0);
  std::mt19937_64 rng(1300);
  std::uniform_real_distribution<double> radius(0.05, 5.0);
  std::uniform_real_distribution<double> angle(-vf::kPi, vf::kPi);
  vf::CurveHint chained;
  Vec2 walker = c + Vec2(kR + 0.3, 0.0);
  std::normal_distribution<double> step(0.0, 0.05);
  for (int i = 0; i < 10000; ++i) {
    // Even samples: random points, no hint. Odd samples: a random walk with chained hints.
    Vec2 x;
    vf::CurveHint hint;
    if (i % 2 == 0) {
      const double rho = radius(rng);
      const double phi = angle(rng);
      x = c + rho * Vec2(std::cos(phi), std::sin(phi));
    } else {
      walker += Vec2(step(rng), step(rng));
      if ((walker - c).norm() < 0.05 || (walker - c).norm() > 5.0) {
        walker = c + Vec2(kR + 0.3, 0.0);
      }
      x = walker;
      hint = chained;
    }
    const vf::ClosestPoint cp = curve->closestPoint(x, contextAt(), hint);
    ASSERT_TRUE(cp.valid);
    const Vec2 radial = (x - c).normalized();
    const Vec2 expected_foot = c + kR * radial;
    const double expected_D = std::abs((x - c).norm() - kR);
    ASSERT_LE((cp.x_star - expected_foot).norm(), 1e-9) << "x = (" << x.x() << ", " << x.y() << ")";
    ASSERT_LE(std::abs(cp.D - expected_D), 1e-9);
    const Vec2 expected_n = (x - c).norm() > kR ? radial : Vec2(-radial);
    ASSERT_LE((cp.n - expected_n).norm(), 1e-9);
    ASSERT_LE((cp.T - vf::J(radial)).norm(), 1e-9);
    const Vec2 phi = staticField(cp, gain, kVr);
    EXPECT_NEAR(cp.n.dot(phi), -kVr * gain.G(expected_D), 1e-9);
    EXPECT_NEAR(cp.T.dot(phi), kVr * gain.H(expected_D), 1e-9);
    if (i % 2 == 1) {
      chained = vf::makeHint(cp);
    }
  }
}

// Local stage: with a valid hint and D_loc < D_trig the search stays on the hinted part, even when
// another part of the curve is closer (math_spec §2.4).
TEST(ParametricCurve, LocalWindowKeepsHintedBranch)
{
  const auto curve = makeParametric(builtinSpec("ellipse", {{"a", 3.0}, {"b", 0.2}}));
  ASSERT_NE(curve, nullptr);
  const Vec2 x(0.0, 0.05);  // 0.15 from the top, 0.25 from the bottom
  const vf::ClosestPoint initial = curve->closestPoint(x, contextAt(), vf::CurveHint{});
  ASSERT_TRUE(initial.valid);
  EXPECT_NEAR(initial.x_star.y(), 0.2, 1e-12);
  EXPECT_NEAR(initial.D, 0.15, 1e-12);

  const vf::ClosestPoint hinted = curve->closestPoint(x, contextAt(), hintAt(1.5 * vf::kPi));
  ASSERT_TRUE(hinted.valid);
  EXPECT_NEAR(hinted.x_star.y(), -0.2, 1e-12);
  EXPECT_NEAR(hinted.D, 0.25, 1e-12);
  EXPECT_FALSE(vf::Status::has(hinted.status_bits, vf::Status::kAmbiguousClosest));
}

// Edge continuation: a stale hint far along the curve; the bracket moves by w/2 until the minimum
// is interior, and the global candidate (the same foot) is not adopted.
TEST(ParametricCurve, EdgeContinuationFindsDistantFoot)
{
  const auto curve = makeParametric(builtinSpec("ellipse", {{"a", 5.0}, {"b", 3.0}}));
  ASSERT_NE(curve, nullptr);
  for (const double u_true : {1.0, 2.5, -2.0}) {
    const Vec2 foot = curve->r(u_true, 0.0);
    const Vec2 normal = vf::J(curve->drdu(u_true, 0.0).normalized()) * -1.0;  // outward (CCW)
    const Vec2 x = foot + 0.1 * normal;
    const vf::ClosestPoint cp = curve->closestPoint(x, contextAt(), hintAt(0.0));
    ASSERT_TRUE(cp.valid);
    EXPECT_LE((cp.x_star - foot).norm(), 1e-9) << "u_true = " << u_true;
    EXPECT_NEAR(cp.D, 0.1, 1e-9);
    EXPECT_FALSE(vf::Status::has(cp.status_bits, vf::Status::kAmbiguousClosest));
  }
}

// Edge continuation is bounded (at most one lap) and still converges on the antipode of the hint.
TEST(ParametricCurve, EdgeContinuationIsBounded)
{
  const CountingCircle circle(3.0, false, vf::CurveOptions{});
  for (const double hint_u : {0.0, 1.0, 4.0}) {
    const double target = hint_u + vf::kPi;
    const Vec2 x = 3.3 * Vec2(std::cos(target), std::sin(target));
    circle.resetCount();
    const vf::ClosestPoint cp = circle.closestPoint(x, contextAt(), hintAt(hint_u));
    ASSERT_TRUE(cp.valid);
    EXPECT_LE((cp.x_star - 3.0 * Vec2(std::cos(target), std::sin(target))).norm(), 1e-9);
    // ≈ 2πR/(w/2) = 38 brackets at most of ~50 evaluations, plus the global stage.
    EXPECT_LT(circle.count(), 20000);
  }
}

// Time-varying windows use Δu = w/(2‖∂r/∂u(u_prev)‖) (math_spec §2.5): a foot inside the window
// needs no global stage; one beyond the window edge triggers it (M = 400 evaluations).
TEST(ParametricCurve, TimeVaryingWindowUsesSpeed)
{
  const CountingCircle circle(2.0, true, vf::CurveOptions{});  // Δu = 1/(2·2) = 0.25
  ASSERT_TRUE(circle.isTimeVarying());
  const auto query = [&circle](double angle) {
    const Vec2 x = 2.1 * Vec2(std::cos(angle), std::sin(angle));
    circle.resetCount();
    const vf::ClosestPoint cp = circle.closestPoint(x, contextAt(0.3), hintAt(0.0));
    EXPECT_TRUE(cp.valid);
    EXPECT_NEAR(cp.u_star, angle, 1e-9);
    EXPECT_NEAR(cp.D, 0.1, 1e-9);
    return circle.count();
  };
  const long inside = query(0.2);   // arc 0.4 < w/2
  const long outside = query(0.3);  // arc 0.6 > w/2
  EXPECT_LT(inside, 400);
  EXPECT_GE(outside, 400 + inside / 2);
}

// O9 analogue on a stadium (rounded_rectangle with radius = height/2): legs w_u = 0.8 apart.
TEST(ParametricCurve, GlobalStageAdoptionAndDetourTest)
{
  const auto curve = makeParametric(
    builtinSpec("rounded_rectangle", {{"width", 8.0}, {"height", 0.8}, {"radius", 0.4}}));
  ASSERT_NE(curve, nullptr);
  // Leg 1 is the bottom side y = −0.4, traversed toward +x; the semicircle starts at x = 3.6.
  const auto hintOnLeg1 = [&curve](double x_coordinate) {
    const vf::ClosestPoint cp =
      curve->closestPoint(Vec2(x_coordinate, -0.45), contextAt(), vf::CurveHint{});
    EXPECT_TRUE(cp.valid);
    EXPECT_NEAR(cp.x_star.y(), -0.4, 1e-12);
    return vf::makeHint(cp);
  };

  // 2.5·w_u before the semicircle, 0.6 off leg 1: arc 2x + π w_u/2 = 5.26 > 3·0.8 + 2·0.8.
  {
    const vf::ClosestPoint cp =
      curve->closestPoint(Vec2(3.6 - 2.0, 0.2), contextAt(), hintOnLeg1(3.6 - 2.0));
    ASSERT_TRUE(cp.valid);
    EXPECT_NEAR(cp.x_star.y(), -0.4, 1e-9);
    EXPECT_NEAR(cp.D, 0.6, 1e-9);
    EXPECT_FALSE(vf::Status::has(cp.status_bits, vf::Status::kAmbiguousClosest));
  }
  // 0.5·w_u before the semicircle, 0.6 off leg 1: arc 2.06 <= 4.0, adopted and flagged.
  {
    const vf::ClosestPoint cp =
      curve->closestPoint(Vec2(3.6 - 0.4, 0.2), contextAt(), hintOnLeg1(3.6 - 0.4));
    ASSERT_TRUE(cp.valid);
    EXPECT_NEAR(cp.x_star.y(), 0.4, 1e-9);
    EXPECT_NEAR(cp.D, 0.2, 1e-9);
    EXPECT_TRUE(vf::Status::has(cp.status_bits, vf::Status::kAmbiguousClosest));
  }
  // 0.45 off leg 1 is below D_trig = 0.5: no global stage, the hinted leg is kept.
  {
    const vf::ClosestPoint cp =
      curve->closestPoint(Vec2(3.6 - 0.4, 0.05), contextAt(), hintOnLeg1(3.6 - 0.4));
    ASSERT_TRUE(cp.valid);
    EXPECT_NEAR(cp.x_star.y(), -0.4, 1e-9);
    EXPECT_FALSE(vf::Status::has(cp.status_bits, vf::Status::kAmbiguousClosest));
  }
}

// Gerono lemniscate (a = 4, b = 1.5, crossing angle 106°) followed through its crossing with
// 1–5 cm lateral noise: the foot stays on its branch, also when the global stage runs every
// call (trigger 0), and also without hysteresis (the detour test alone rejects the jump).
TEST(ParametricCurve, GeronoCrossingWithNoiseStaysOnBranch)
{
  vf::CurveOptions every_call;
  every_call.global_search_trigger = 0.0;
  vf::CurveOptions detour_only = every_call;
  detour_only.global_search_hysteresis = 0.0;
  const std::array<std::pair<const char *, vf::CurveOptions>, 3> configurations{{
    {"defaults", vf::CurveOptions{}},
    {"global stage every call", every_call},
    {"no hysteresis", detour_only},
  }};
  for (const auto & [label, options] : configurations) {
    SCOPED_TRACE(label);
    const auto curve =
      makeParametric(builtinSpec("lemniscate_gerono", {{"a", 4.0}, {"b", 1.5}}), options);
    ASSERT_NE(curve, nullptr);
    std::mt19937_64 rng(1061);
    std::uniform_real_distribution<double> magnitude(0.01, 0.05);
    std::bernoulli_distribution sign(0.5);
    vf::CurveHint hint;
    // Two laps in steps of 0.01 rad (≈ 5 cm), through both crossings (u = π/2 and 3π/2).
    for (double u = 0.2; u < 0.2 + 2.0 * vf::kTwoPi; u += 0.01) {
      const Vec2 tangent = curve->drdu(u, 0.0).normalized();
      const double offset = (sign(rng) ? 1.0 : -1.0) * magnitude(rng);
      const Vec2 x = curve->r(u, 0.0) + offset * vf::J(tangent);
      const vf::ClosestPoint cp = curve->closestPoint(x, contextAt(), hint);
      ASSERT_TRUE(cp.valid);
      const double error = std::abs(vf::wrapPeriodic(cp.u_star - u, vf::kTwoPi));
      ASSERT_LT(error, 0.05) << "branch jump at u = " << u;
      EXPECT_FALSE(vf::Status::has(cp.status_bits, vf::Status::kAmbiguousClosest));
      hint = vf::makeHint(cp);
    }
  }
}

// u↔s table: L, s(u) and u(s) of an ellipse (analytic derivatives) against the exact perimeter
// and an independent Simpson quadrature; the expression twin (numerical derivatives) to 1e-8.
TEST(ParametricCurve, ArcLengthTableAccuracy)
{
  constexpr double kA = 2.0;
  constexpr double kB = 1.0;
  const double exact_length = 4.0 * kA * ellipticE(1.0 - (kB * kB) / (kA * kA));
  const auto builtin = makeParametric(builtinSpec("ellipse", {{"a", kA}, {"b", kB}}));
  const auto expression =
    makeParametric(expressionSpec("2*cos(s)", "sin(s)", 0.0, vf::kTwoPi, true));
  ASSERT_NE(builtin, nullptr);
  ASSERT_NE(expression, nullptr);
  EXPECT_NEAR(builtin->length(0.0), exact_length, 1e-12);
  EXPECT_NEAR(expression->length(0.0), exact_length, 1e-8);

  const auto speed = [](double u) {
    return std::sqrt(kA * kA * std::sin(u) * std::sin(u) + kB * kB * std::cos(u) * std::cos(u));
  };
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> uu(0.0, vf::kTwoPi);
  for (int i = 0; i < 100; ++i) {
    const double u = uu(rng);
    const double reference = simpson(speed, 0.0, u, 20000);
    EXPECT_NEAR(builtin->stationAt(u, 0.0), reference, 1e-11) << "u = " << u;
    EXPECT_NEAR(expression->stationAt(u, 0.0), reference, 1e-8) << "u = " << u;
    EXPECT_NEAR(builtin->parameterAt(builtin->stationAt(u, 0.0), 0.0), u, 1e-11);
    EXPECT_NEAR(builtin->arcLength(0.0, u, 0.0), reference, 1e-11);
    const double s = uu(rng) / vf::kTwoPi * exact_length;
    EXPECT_NEAR(builtin->stationAt(builtin->parameterAt(s, 0.0), 0.0), s, 1e-11);
  }
  // Closed curves: stations wrap into [0, L); arcLength does not wrap.
  EXPECT_NEAR(
    builtin->parameterAt(exact_length + 0.25, 0.0), builtin->parameterAt(0.25, 0.0), 1e-12);
  EXPECT_NEAR(builtin->arcLength(0.0, 2.0 * vf::kTwoPi, 0.0), 2.0 * exact_length, 1e-10);
  EXPECT_NEAR(builtin->arcLength(1.0, 0.5, 0.0), -builtin->arcLength(0.5, 1.0, 0.0), 1e-14);
}

// U05: an EXPRESSION on [2, 5] is sampled over exactly [2, 5] (never outside), and its length
// matches the analytic value; s_min >= s_max is INVALID_CURVE.
TEST(UpstreamRegression, U05_DomainWithNonZeroStart)
{
  // sqrt(s − 2) and sqrt(5 − s) are NaN outside [2, 5]: an evaluation there would surface.
  const auto curve =
    makeParametric(expressionSpec("s", "0.5*s^2 + 0*sqrt(s - 2) + 0*sqrt(5 - s)", 2.0, 5.0, false));
  ASSERT_NE(curve, nullptr);
  EXPECT_EQ(curve->uMin(), 2.0);
  EXPECT_EQ(curve->uMax(), 5.0);
  const auto primitive = [](double s) {
    return 0.5 * (s * std::sqrt(1.0 + s * s) + std::asinh(s));
  };
  const double exact = primitive(5.0) - primitive(2.0);
  EXPECT_NEAR(curve->length(0.0), exact, 1e-6);
  EXPECT_LE((curve->pointAt(0.0, 0.0) - Vec2(2.0, 2.0)).norm(), 1e-12);
  EXPECT_LE((curve->pointAt(curve->length(0.0), 0.0) - Vec2(5.0, 12.5)).norm(), 1e-9);
  // The ends of the domain are found exactly.
  const vf::ClosestPoint start = curve->closestPoint(Vec2(2.0, 2.0), contextAt(), vf::CurveHint{});
  const vf::ClosestPoint end = curve->closestPoint(Vec2(5.0, 12.5), contextAt(), vf::CurveHint{});
  ASSERT_TRUE(start.valid);
  ASSERT_TRUE(end.valid);
  EXPECT_EQ(start.u_star, 2.0);
  EXPECT_EQ(end.u_star, 5.0);
  // Points everywhere around, including far beyond both ends, never see NaN.
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> ux(-20.0, 30.0);
  std::uniform_real_distribution<double> uy(-20.0, 40.0);
  vf::CurveHint hint;
  for (int i = 0; i < 2000; ++i) {
    const vf::ClosestPoint cp = curve->closestPoint(Vec2(ux(rng), uy(rng)), contextAt(), hint);
    ASSERT_TRUE(cp.valid);
    ASSERT_GE(cp.u_star, 2.0);
    ASSERT_LE(cp.u_star, 5.0);
    hint = i % 3 == 0 ? vf::CurveHint{} : vf::makeHint(cp);
  }
  for (const double s_max : {2.0, 1.0}) {
    try {
      vf::makeCurve(expressionSpec("s", "0.5*s^2", 2.0, s_max, false), vf::CurveOptions{});
      ADD_FAILURE() << "s_max = " << s_max << " accepted";
    } catch (const vf::CurveError & error) {
      EXPECT_EQ(error.code(), vf::CurveError::Code::kInvalidCurve);
    }
  }
}

// U04: the first closestPoint (no hint) runs the global stage: a robot near
// s = 0.6·(s_max − s_min) gets that foot, not one near s_min.
TEST(UpstreamRegression, U04_ParametricGlobalInitialisation)
{
  for (const double s_min : {0.0, 1.0}) {
    const double s_max = s_min + vf::kTwoPi;
    const std::string x = "2*cos(s - " + std::to_string(s_min) + ")";
    const std::string y = "sin(s - " + std::to_string(s_min) + ")";
    const auto curve = makeParametric(expressionSpec(x, y, s_min, s_max, true));
    ASSERT_NE(curve, nullptr);
    const double u_target = s_min + 0.6 * (s_max - s_min);
    const Vec2 normal = -vf::J(curve->drdu(u_target, 0.0).normalized());
    const Vec2 robot = curve->r(u_target, 0.0) + 0.1 * normal;
    const vf::ClosestPoint cp = curve->closestPoint(robot, contextAt(), vf::CurveHint{});
    ASSERT_TRUE(cp.valid);
    EXPECT_NEAR(cp.u_star, u_target, 1e-8);
    EXPECT_NEAR(cp.D, 0.1, 1e-9);
    EXPECT_NEAR(cp.s_star, curve->stationAt(u_target, 0.0), 1e-8);
  }
}

// U07: x = s², y = s³ on [−1, 1] has a stationary point (a cusp) and is INVALID_CURVE.
TEST(UpstreamRegression, U07_StationaryPointRejected)
{
  try {
    vf::makeCurve(expressionSpec("s^2", "s^3", -1.0, 1.0, false), vf::CurveOptions{});
    FAIL() << "the cusp was accepted";
  } catch (const vf::CurveError & error) {
    EXPECT_EQ(error.code(), vf::CurveError::Code::kInvalidCurve);
    EXPECT_NE(std::string(error.what()).find("not regular"), std::string::npos) << error.what();
  }
}

// O11: regularity and closure rejects; every built-in at its defaults passes.
TEST(ParametricCurve, O11RegularityRejects)
{
  const std::vector<std::pair<std::string, vf::CurveSpec>> rejects{
    {"lissajous 1:2, phase pi/4", builtinSpec("lissajous", {{"phase", vf::kPi / 4.0}})},
    {"x = s^2, y = s^3", expressionSpec("s^2", "s^3", -1.0, 1.0, false)},
    {"half ellipse marked closed", expressionSpec("2*cos(s)", "sin(s)", 0.0, vf::kPi, true)},
    {"superellipse n = 1.5", builtinSpec("superellipse", {{"n", 1.5}})},
    {"lissajous fx = 2, fy = 4", builtinSpec("lissajous", {{"fx", 2.0}, {"fy", 4.0}})},
  };
  for (const auto & [label, spec] : rejects) {
    SCOPED_TRACE(label);
    try {
      vf::makeCurve(spec, vf::CurveOptions{});
      ADD_FAILURE() << "accepted";
    } catch (const vf::CurveError & error) {
      EXPECT_EQ(error.code(), vf::CurveError::Code::kInvalidCurve) << error.what();
    }
  }
  for (const vf::BuiltinShapeInfo & shape : vf::builtinShapes()) {
    SCOPED_TRACE(shape.name);
    EXPECT_NO_THROW(vf::makeCurve(builtinSpec(shape.name), vf::CurveOptions{}));
  }
  // A closed expression that closes in position but whose end tangents differ by 5.7 degrees.
  try {
    vf::makeCurve(
      expressionSpec("cos(s) + 0.1*sin(s/2)", "sin(s)", 0.0, vf::kTwoPi, true), vf::CurveOptions{});
    ADD_FAILURE() << "a kink at the seam was accepted";
  } catch (const vf::CurveError & error) {
    EXPECT_EQ(error.code(), vf::CurveError::Code::kInvalidCurve);
    EXPECT_NE(std::string(error.what()).find("end tangents"), std::string::npos) << error.what();
  }
}

// Time-varying curves: Gauss–Legendre arc length without a table, s*, and ∂_t r.
TEST(ParametricCurve, TimeVaryingArcLengthStationAndVelocity)
{
  const auto curve = makeParametric(
    expressionSpec("(1 + 0.5*t)*cos(s)", "(1 + 0.5*t)*sin(s)", 0.0, vf::kTwoPi, true));
  ASSERT_NE(curve, nullptr);
  ASSERT_TRUE(curve->isTimeVarying());
  for (const double t : {0.0, 2.0, 3.5}) {
    const double radius = 1.0 + 0.5 * t;
    EXPECT_NEAR(curve->length(t), vf::kTwoPi * radius, 1e-8);
    const double angle = 2.2;
    const Vec2 x = (radius + 0.3) * Vec2(std::cos(angle), std::sin(angle));
    const vf::ClosestPoint cp = curve->closestPoint(x, contextAt(t), vf::CurveHint{});
    ASSERT_TRUE(cp.valid);
    EXPECT_LE((cp.x_star - radius * Vec2(std::cos(angle), std::sin(angle))).norm(), 1e-9);
    EXPECT_NEAR(cp.s_star, radius * angle, 1e-8);
    EXPECT_NEAR(cp.u_star, angle, 1e-9);
    const Vec2 v = curve->velocity(cp, t);
    EXPECT_LE((v - 0.5 * Vec2(std::cos(angle), std::sin(angle))).norm(), 1e-8);
    // u(s) by Newton on the panel integral.
    EXPECT_NEAR(curve->parameterAt(radius * angle, t), angle, 1e-9);
  }
}

// ∂_t r of a translating expression, and the time origin t0 (the expression's t is t − t0).
TEST(ParametricCurve, TranslatingExpressionVelocity)
{
  vf::CurveSpec spec = expressionSpec("cos(s) + 0.3*t", "sin(s) - 0.2*t", 0.0, vf::kTwoPi, true);
  const auto curve = makeParametric(spec);
  spec.t0 = 100.0;
  const auto shifted = makeParametric(spec);
  ASSERT_NE(curve, nullptr);
  ASSERT_NE(shifted, nullptr);
  const Vec2 x(2.0, 1.0);
  const vf::ClosestPoint cp = curve->closestPoint(x, contextAt(5.0), vf::CurveHint{});
  const vf::ClosestPoint cp_shifted = shifted->closestPoint(x, contextAt(105.0), vf::CurveHint{});
  ASSERT_TRUE(cp.valid);
  ASSERT_TRUE(cp_shifted.valid);
  EXPECT_LE((curve->velocity(cp, 5.0) - Vec2(0.3, -0.2)).norm(), 1e-9);
  EXPECT_LE((shifted->velocity(cp_shifted, 105.0) - Vec2(0.3, -0.2)).norm(), 1e-9);
  EXPECT_LE((cp.x_star - cp_shifted.x_star).norm(), 1e-12);
  EXPECT_LE((cp.x_star - (Vec2(1.5, -1.0) + (x - Vec2(1.5, -1.0)).normalized())).norm(), 1e-9);
  // Static curves and rays have zero velocity.
  const auto still = makeParametric(builtinSpec("ellipse"));
  const vf::ClosestPoint cp_still = still->closestPoint(x, contextAt(5.0), vf::CurveHint{});
  EXPECT_EQ(still->velocity(cp_still, 5.0), Vec2::Zero());
}

// Open expression curves are extended by straight rays along one-sided end tangents (O5-style).
TEST(ParametricCurve, RaysOnOpenExpressionCurves)
{
  const auto curve = makeParametric(expressionSpec("s", "0.25*s^2", -1.0, 1.0, false));
  ASSERT_NE(curve, nullptr);
  const double length = curve->length(0.0);
  const Vec2 start(-1.0, 0.25);
  const Vec2 end(1.0, 0.25);
  const Vec2 t_start = Vec2(1.0, -0.5).normalized();
  const Vec2 t_end = Vec2(1.0, 0.5).normalized();
  for (const bool with_hint : {false, true}) {
    const vf::ClosestPoint forward = curve->closestPoint(
      end + 100.0 * t_end + 0.3 * vf::J(t_end), contextAt(),
      with_hint ? hintAt(0.9) : vf::CurveHint{});
    ASSERT_TRUE(forward.valid);
    EXPECT_TRUE(forward.on_ray);
    EXPECT_EQ(forward.piece, vf::PieceKind::kForwardRay);
    EXPECT_NEAR(forward.s_star, length + 100.0, 1e-9);
    EXPECT_EQ(forward.u_star, 1.0);
    EXPECT_NEAR(forward.D, 0.3, 1e-9);
    EXPECT_LE((forward.T - t_end).norm(), 1e-9);
    EXPECT_LE((forward.n - vf::J(t_end)).norm(), 1e-9);
    expectOrthonormal(forward);

    const vf::ClosestPoint backward = curve->closestPoint(
      start - 100.0 * t_start + 0.2 * vf::J(t_start), contextAt(),
      with_hint ? hintAt(-0.9) : vf::CurveHint{});
    ASSERT_TRUE(backward.valid);
    EXPECT_TRUE(backward.on_ray);
    EXPECT_EQ(backward.piece, vf::PieceKind::kBackwardRay);
    EXPECT_NEAR(backward.s_star, -100.0, 1e-9);
    EXPECT_EQ(backward.u_star, -1.0);
    EXPECT_NEAR(backward.D, 0.2, 1e-9);
    expectOrthonormal(backward);
  }
  EXPECT_LE((curve->pointAt(length + 100.0, 0.0) - (end + 100.0 * t_end)).norm(), 1e-9);
  EXPECT_LE((curve->pointAt(-100.0, 0.0) - (start - 100.0 * t_start)).norm(), 1e-9);
  EXPECT_LE((curve->tangentAt(length + 3.0, 0.0) - t_end).norm(), 1e-9);
  EXPECT_LE((curve->tangentAt(-3.0, 0.0) - t_start).norm(), 1e-9);
  EXPECT_EQ(
    curve->velocity(curve->closestPoint(Vec2(5.0, 5.0), contextAt(), {}), 0.0), Vec2::Zero());
  EXPECT_DOUBLE_EQ(vf::remainingToEnd(*curve, curve->closestPoint(end, contextAt(), {}), 0.0), 0.0);
}

// r() is never evaluated outside [u_i, u_f] for an open curve, whatever the query.
TEST(ParametricCurve, OpenCurveNeverEvaluatedOutsideDomain)
{
  const DomainRecorder curve(vf::CurveOptions{});
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> ux(-50.0, 50.0);
  std::uniform_real_distribution<double> uy(-50.0, 50.0);
  vf::CurveHint hint;
  for (int i = 0; i < 3000; ++i) {
    const vf::ClosestPoint cp = curve.closestPoint(Vec2(ux(rng), uy(rng)), contextAt(), hint);
    ASSERT_TRUE(cp.valid);
    hint = i % 4 == 0 ? hintAt(ux(rng)) : vf::makeHint(cp);
  }
  for (double s = -10.0; s < curve.length(0.0) + 10.0; s += 0.37) {
    static_cast<void>(curve.pointAt(s, 0.0));
    static_cast<void>(curve.tangentAt(s, 0.0));
  }
  EXPECT_GE(curve.minU(), -1.0);
  EXPECT_LE(curve.maxU(), 2.0);
}

// sampleStations equals pointAt station by station (static and time-varying, open and closed).
TEST(ParametricCurve, SampleStationsMatchPointAt)
{
  const auto closed_static = makeParametric(builtinSpec("lemniscate_bernoulli", {{"a", 2.0}}));
  const auto closed_varying =
    makeParametric(expressionSpec("(1 + 0.1*t)*2*cos(s)", "sin(s)", 0.0, vf::kTwoPi, true));
  const auto open_static = makeParametric(expressionSpec("s", "0.25*s^2", -1.0, 1.0, false));
  std::array<Vec2, 400> buffer{};
  for (const auto & curve : {closed_static, closed_varying, open_static}) {
    ASSERT_NE(curve, nullptr);
    const double t = curve->isTimeVarying() ? 1.0 : 0.0;
    const double length = curve->length(t);
    const double s0 = curve->isClosed() ? 0.7 * length : -0.5;
    const double s1 = curve->isClosed() ? 1.6 * length : length + 0.5;
    const double ds = 0.05;
    const std::size_t count =
      curve->sampleStations(s0, s1, ds, t, vf::Span<Vec2>(buffer.data(), buffer.size()));
    ASSERT_GT(count, 2U);
    const double first = curve->isClosed() ? s0 : 0.0;
    const double last = curve->isClosed() ? s1 : length;
    for (std::size_t k = 0; k + 1 < count; ++k) {
      const double s = first + ds * static_cast<double>(k);
      ASSERT_LE((buffer[k] - curve->pointAt(s, t)).norm(), 1e-9) << "k = " << k;
    }
    EXPECT_LE((buffer[count - 1] - curve->pointAt(last, t)).norm(), 1e-9);
    // Invalid inputs give 0 points.
    EXPECT_EQ(curve->sampleStations(0.0, 1.0, 0.0, t, vf::Span<Vec2>(buffer.data(), 4)), 0U);
    EXPECT_EQ(curve->sampleStations(vf::kNaN, 1.0, 0.1, t, vf::Span<Vec2>(buffer.data(), 4)), 0U);
    EXPECT_EQ(curve->sampleStations(0.0, 1.0, 0.1, t, vf::Span<Vec2>()), 0U);
  }
}

// Non-finite inputs never throw: invalid results with Status::kNonFinite.
TEST(ParametricCurve, NonFiniteInputsAndEvaluations)
{
  const auto curve = makeParametric(builtinSpec("ellipse"));
  ASSERT_NE(curve, nullptr);
  const vf::ClosestPoint nan_x = curve->closestPoint(Vec2(vf::kNaN, 0.0), contextAt(), {});
  EXPECT_FALSE(nan_x.valid);
  EXPECT_TRUE(vf::Status::has(nan_x.status_bits, vf::Status::kNonFinite));
  const vf::ClosestPoint nan_t = curve->closestPoint(Vec2(2.0, 0.0), contextAt(vf::kNaN), {});
  EXPECT_FALSE(nan_t.valid);
  // A NaN hint parameter is treated as no hint.
  const vf::ClosestPoint nan_hint =
    curve->closestPoint(Vec2(2.0, 0.0), contextAt(), hintAt(vf::kNaN));
  ASSERT_TRUE(nan_hint.valid);
  EXPECT_NEAR(nan_hint.D, 1.0, 1e-12);
  EXPECT_TRUE(std::isnan(curve->stationAt(vf::kNaN, 0.0)));
  EXPECT_TRUE(std::isnan(curve->parameterAt(vf::kInf, 0.0)));
  EXPECT_FALSE(vf::isFinite(curve->pointAt(vf::kNaN, 0.0)));

  // A curve that is NaN on a small interval between the global samples.
  const HoleCircle hole(vf::CurveOptions{});
  for (double angle = 0.05; angle < 0.15; angle += 0.0005) {
    for (const double radius : {0.5, 1.0, 1.5}) {
      const vf::ClosestPoint cp = hole.closestPoint(
        radius * Vec2(std::cos(angle), std::sin(angle)), contextAt(), hintAt(angle));
      if (cp.valid) {
        EXPECT_TRUE(vf::isFinite(cp.x_star));
        EXPECT_TRUE(std::isfinite(cp.D));
      } else {
        EXPECT_TRUE(vf::Status::has(cp.status_bits, vf::Status::kNonFinite));
      }
    }
  }
}

// A time-varying curve whose speed collapses at runtime sets kNonFinite (math_spec §2.5).
TEST(ParametricCurve, RuntimeSpeedFloorSetsNonFinite)
{
  const auto curve =
    makeParametric(expressionSpec("(1 - t)*cos(s)", "(1 - t)*sin(s)", 0.0, vf::kTwoPi, true));
  ASSERT_NE(curve, nullptr);
  EXPECT_NEAR(curve->speedFloor(), 1e-3, 1e-12);
  const vf::ClosestPoint healthy = curve->closestPoint(Vec2(0.0, 1.0), contextAt(0.5), {});
  ASSERT_TRUE(healthy.valid);
  EXPECT_FALSE(vf::Status::has(healthy.status_bits, vf::Status::kNonFinite));
  const vf::ClosestPoint collapsed = curve->closestPoint(Vec2(0.0, 1.0), contextAt(0.9999999), {});
  EXPECT_TRUE(vf::Status::has(collapsed.status_bits, vf::Status::kNonFinite));
}

// The constructor validates the domain and the options.
TEST(ParametricCurve, ConstructorValidatesDomainAndOptions)
{
  struct Line final : vf::ParametricCurve
  {
    Line(double a, double b, const vf::CurveOptions & options)
    : ParametricCurve(a, b, false, false, options)
    {
      initialize(0.0);
    }
    Vec2 r(double u, double /*t*/) const noexcept override { return Vec2(u, 0.0); }
  };
  EXPECT_NO_THROW(Line(0.0, 1.0, vf::CurveOptions{}));
  EXPECT_THROW(Line(1.0, 1.0, vf::CurveOptions{}), vf::ConfigError);
  EXPECT_THROW(Line(0.0, vf::kInf, vf::CurveOptions{}), vf::ConfigError);
  vf::CurveOptions bad;
  bad.global_search_samples = 3;
  EXPECT_THROW(Line(0.0, 1.0, bad), vf::ConfigError);
  const Line line(0.0, 3.0, vf::CurveOptions{});
  // Central differences of a line carry rounding noise of about eps·|u|/h_u.
  EXPECT_NEAR(line.length(0.0), 3.0, 1e-10);
  EXPECT_FALSE(line.isClosed());
  EXPECT_FALSE(line.isTimeVarying());
  EXPECT_EQ(line.spec(), nullptr);
  EXPECT_EQ(line.options().global_search_samples, 400);
}

// Parametric and expression curves have no vertices: Curve::vertexSpacing() is 0, so heading
// tracking keeps delta = max(0.02, |v| dt) (math_spec §7.4).
TEST(ParametricCurve, HasNoVertexSpacing)
{
  const auto builtin = makeParametric(builtinSpec("ellipse", {{"a", 1.3}, {"b", 1.3}}));
  const auto expression =
    makeParametric(expressionSpec("3*cos(s)", "2*sin(s)", 0.0, vf::kTwoPi, true));
  ASSERT_NE(builtin, nullptr);
  ASSERT_NE(expression, nullptr);
  EXPECT_EQ(builtin->vertexSpacing(), 0.0);
  EXPECT_EQ(expression->vertexSpacing(), 0.0);
}

// Concurrent const use from several threads gives the serial answers (run under TSan too).
TEST(ParametricCurve, ConcurrentClosestPoint)
{
  const auto builtin = makeParametric(builtinSpec("lemniscate_gerono", {{"a", 4.0}, {"b", 1.5}}));
  const auto expression =
    makeParametric(expressionSpec("3*cos(s) + 0.2*cos(3*s)", "2*sin(s)", 0.0, vf::kTwoPi, true));
  ASSERT_NE(builtin, nullptr);
  ASSERT_NE(expression, nullptr);
  std::vector<Vec2> queries;
  std::mt19937_64 rng(99);
  std::uniform_real_distribution<double> coordinate(-5.0, 5.0);
  for (int i = 0; i < 300; ++i) {
    queries.emplace_back(coordinate(rng), coordinate(rng));
  }
  for (const auto & curve : {builtin, expression}) {
    std::vector<Vec2> serial;
    for (const Vec2 & x : queries) {
      serial.push_back(curve->closestPoint(x, contextAt(), {}).x_star);
    }
    constexpr int kThreads = 4;
    std::vector<std::vector<Vec2>> results(kThreads);
    std::vector<std::thread> threads;
    for (int k = 0; k < kThreads; ++k) {
      threads.emplace_back([&curve, &queries, &results, k]() {
        for (const Vec2 & x : queries) {
          results[static_cast<std::size_t>(k)].push_back(
            curve->closestPoint(x, contextAt(), {}).x_star);
        }
      });
    }
    for (std::thread & thread : threads) {
      thread.join();
    }
    for (const auto & result : results) {
      ASSERT_EQ(result.size(), serial.size());
      for (std::size_t i = 0; i < serial.size(); ++i) {
        EXPECT_EQ(result[i], serial[i]);
      }
    }
  }
}
