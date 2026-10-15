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

// The path field (math_spec §4–§5) and speed shaping (§8.2): oracles O17, O18, O19, O25 (profile
// part) and O26, the terminal field, the reference-point shift and the shaping branches.
//
// Closest points are built analytically (straight lines and circles with an exactly orthogonal
// frame) so that these tests do not depend on the curve implementations.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/field.hpp"
#include "vectorfield_core/gains.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/kinematics.hpp"
#include "vectorfield_core/status.hpp"
#include "vectorfield_core/tracking_law.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;
using vf::ClosestPoint;
using vf::DistanceField;
using vf::FieldSample;
using vf::GainFunction;
using vf::GainType;
using vf::Status;
using vf::TerminalProfile;
using vf::Vec2;

namespace
{

constexpr double kVr = 0.4;
constexpr double kEpsV = 0.1;
constexpr int kRandomSamples = 20000;

// Closest point on the straight line through p0 with unit tangent t. The frame is exactly
// orthogonal (n = ±J t), as a curve's frame is to rounding.
ClosestPoint lineClosestPoint(const Vec2 & x, const Vec2 & p0, const Vec2 & t)
{
  ClosestPoint cp;
  const Vec2 left = vf::J(t);
  const double y = (x - p0).dot(left);
  cp.T = t;
  cp.D = std::abs(y);
  cp.n = cp.D < vf::kEpsD ? Vec2(Vec2::Zero()) : Vec2(vf::sgn(y) * left);
  cp.x_star = x - y * left;
  cp.s_star = (x - p0).dot(t);
  cp.u_star = cp.s_star;
  cp.segment = 0;
  cp.piece = vf::PieceKind::kSegment;
  cp.valid = true;
  return cp;
}

// Closest point on the circle of centre c and radius R, counter-clockwise from +x.
ClosestPoint circleClosestPoint(const Vec2 & x, const Vec2 & c, double R)
{
  ClosestPoint cp;
  const Vec2 r = x - c;
  const double rho = r.norm();
  const Vec2 u = r / rho;
  cp.x_star = c + R * u;
  cp.D = std::abs(rho - R);
  cp.n = cp.D < vf::kEpsD ? Vec2(Vec2::Zero()) : Vec2(vf::sgn(rho - R) * u);
  cp.T = vf::J(u);
  double angle = std::atan2(u.y(), u.x());
  if (angle < 0.0) {
    angle += vf::kTwoPi;
  }
  cp.s_star = R * angle;
  cp.u_star = angle;
  cp.piece = vf::PieceKind::kParametric;
  cp.valid = true;
  return cp;
}

struct RandomCurve
{
  bool circle{false};
  Vec2 p0{Vec2::Zero()};
  Vec2 t{Vec2::UnitX()};
  double R{1.0};

  [[nodiscard]] ClosestPoint closest(const Vec2 & x) const
  {
    return circle ? circleClosestPoint(x, p0, R) : lineClosestPoint(x, p0, t);
  }
};

class Rng
{
public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}
  double uniform(double lo, double hi)
  {
    return std::uniform_real_distribution<double>(lo, hi)(engine_);
  }
  bool coin() { return uniform(0.0, 1.0) < 0.5; }
  Vec2 unit() { return vf::heading(uniform(-vf::kPi, vf::kPi)); }

  RandomCurve curve()
  {
    RandomCurve curve;
    curve.circle = coin();
    curve.p0 = Vec2(uniform(-5.0, 5.0), uniform(-5.0, 5.0));
    curve.t = unit();
    curve.R = uniform(0.3, 4.0);
    return curve;
  }

  // A point at distance up to max_D from the curve (inside or outside a circle, never at its
  // centre).
  Vec2 pointNear(const RandomCurve & curve, double max_D)
  {
    const double D = coin() ? uniform(0.0, max_D) : uniform(0.0, 0.05 * max_D);
    if (curve.circle) {
      const double radius = std::max(0.05 * curve.R, curve.R + (coin() ? D : -D));
      return curve.p0 + radius * unit();
    }
    return curve.p0 + uniform(-10.0, 10.0) * curve.t + (coin() ? D : -D) * vf::J(curve.t);
  }

private:
  std::mt19937_64 engine_;
};

GainFunction randomGain(Rng & rng)
{
  const GainType type = rng.coin() ? GainType::kAtan : GainType::kRational;
  return {type, rng.uniform(0.3, 8.0)};
}

// A curve velocity whose normal part has norm `normal_speed` (any tangential part).
Vec2 curveVelocity(Rng & rng, const ClosestPoint & cp, double normal_speed)
{
  const Vec2 normal = vf::J(cp.T);
  const double sign = rng.coin() ? 1.0 : -1.0;
  return sign * normal_speed * normal + rng.uniform(-1.0, 1.0) * cp.T;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Terminal profile.

// O26: phi is odd, |phi| >= v_min/v_r for |sigma| >= sigma_stop, and its Lipschitz constant is
// L_phi = max(1/sigma_b, v_min/(v_r sigma_stop)) (1e-12 on a 1e5-point grid), for xy_tol 0.25
// and 0.05. The floor binds only for xy_tol 0.05 (math_spec §5.1).
TEST(TerminalField, O26ProfileOddFloorAndLipschitz)
{
  constexpr double kVmin = 0.05;
  constexpr double kSigmaB = 0.5;
  const std::array<double, 2> tolerances{0.25, 0.05};
  const std::array<double, 2> expected_lipschitz{2.0, 5.0};
  for (std::size_t k = 0; k < tolerances.size(); ++k) {
    const double sigma_stop = tolerances[k] / 2.0;
    SCOPED_TRACE("xy_tol " + std::to_string(tolerances[k]));
    const TerminalProfile phi(kVr, kVmin, kSigmaB, sigma_stop);
    EXPECT_DOUBLE_EQ(phi.sigmaStop(), sigma_stop);
    EXPECT_DOUBLE_EQ(phi.sigmaBraking(), kSigmaB);
    EXPECT_NEAR(phi.lipschitz(), expected_lipschitz[k], 1e-12);

    constexpr int kGrid = 100000;
    const double span = 2.0 * kSigmaB;
    double worst_slope = 0.0;
    double worst_excess = -vf::kInf;
    bool floor_binds = false;
    double previous_sigma = -span;
    double previous = phi(-span);
    for (int i = 1; i <= kGrid; ++i) {
      const double sigma = -span + 2.0 * span * static_cast<double>(i) / kGrid;
      const double value = phi(sigma);
      const double step = sigma - previous_sigma;
      // |phi(a) - phi(b)| <= L_phi |a - b| to 1e-12.
      worst_excess = std::max(worst_excess, std::abs(value - previous) - phi.lipschitz() * step);
      worst_slope = std::max(worst_slope, std::abs(value - previous) / step);
      previous_sigma = sigma;
      previous = value;
      ASSERT_EQ(phi(-sigma), -value) << "sigma = " << sigma;
      ASSERT_LE(std::abs(value), 1.0);
      if (std::abs(sigma) >= sigma_stop) {
        ASSERT_GE(std::abs(value), kVmin / kVr - 1e-15) << "sigma = " << sigma;
      }
      if (std::abs(sigma) >= kSigmaB) {
        ASSERT_EQ(value, vf::sgn(sigma));
      }
      if (std::abs(value) > std::abs(sigma) / kSigmaB + 1e-12) {
        floor_binds = true;
      }
    }
    EXPECT_LE(worst_excess, 1e-12);
    EXPECT_GE(worst_slope, phi.lipschitz() - 1e-6) << "L_phi is attained";
    EXPECT_EQ(floor_binds, k == 1);
    EXPECT_EQ(phi(0.0), 0.0);
    EXPECT_EQ(phi(vf::kInf), 1.0);
    EXPECT_EQ(phi(-vf::kInf), -1.0);
    EXPECT_TRUE(std::isnan(phi(vf::kNaN)));
  }
}

// O25 (profile part): |phi| is non-decreasing in |sigma|, so on the path the terminal speed
// v_r |phi| never increases while sigma decreases toward the goal.
TEST(TerminalField, O25ProfileIsMonotone)
{
  for (const double xy_tol : {0.25, 0.05, 0.01}) {
    const TerminalProfile phi(kVr, 0.05, 0.5, xy_tol / 2.0);
    double previous = 0.0;
    for (int i = 0; i <= 100000; ++i) {
      const double sigma = 1.0 * static_cast<double>(i) / 100000.0;
      const double value = phi(sigma);
      ASSERT_GE(value, previous) << "xy_tol " << xy_tol << " sigma " << sigma;
      previous = value;
    }
  }
}

TEST(TerminalField, ProfileWithoutSettleRadius)
{
  // sigma_stop <= 0: the floor is v_min/v_r for every sigma != 0, so phi is not Lipschitz at 0.
  const TerminalProfile phi(kVr, 0.05, 0.5, 0.0);
  EXPECT_DOUBLE_EQ(phi(1e-9), 0.05 / kVr);
  EXPECT_DOUBLE_EQ(phi(-1e-9), -0.05 / kVr);
  EXPECT_EQ(phi(0.0), 0.0);
  EXPECT_TRUE(std::isinf(phi.lipschitz()));
  const TerminalProfile no_floor(kVr, 0.0, 0.5, 0.0);
  EXPECT_DOUBLE_EQ(no_floor.lipschitz(), 2.0);
}

// -----------------------------------------------------------------------------------------------
// Feedforward, clamp and eta.

TEST(Field, FeedforwardIsTheNormalCurveVelocity)
{
  Rng rng(11);
  for (int i = 0; i < 1000; ++i) {
    const Vec2 T = rng.unit();
    const Vec2 v_c(rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0));
    const Vec2 ff = vf::feedforward(T, v_c);
    EXPECT_NEAR(ff.dot(T), 0.0, 1e-15);
    EXPECT_NEAR((ff - (vf::Mat2::Identity() - T * T.transpose()) * v_c).norm(), 0.0, 1e-15);
  }
}

// O19: ||Phi_T||_in = 1.2 v_r is clamped to (1 - eps_v) v_r (1e-12), kAssumption3Clamped is set
// and eta >= eps_v v_r.
TEST(Field, O19Assumption3Clamp)
{
  Rng rng(19);
  for (int i = 0; i < 2000; ++i) {
    const GainFunction gain = randomGain(rng);
    const double v_r = rng.uniform(0.1, 1.5);
    const double eps_v = rng.uniform(0.02, 0.5);
    const DistanceField field(gain, v_r, rng.coin() ? 1 : -1, eps_v);
    const RandomCurve curve = rng.curve();
    const Vec2 x = rng.pointNear(curve, 2.0);
    const ClosestPoint cp = curve.closest(x);
    const Vec2 v_c = curveVelocity(rng, cp, 1.2 * v_r);

    const vf::ClampedFeedforward clamped =
      vf::clampFeedforward(vf::feedforward(cp.T, v_c), v_r, eps_v);
    EXPECT_TRUE(clamped.clamped);
    EXPECT_NEAR(clamped.Phi_T.norm(), (1.0 - eps_v) * v_r, 1e-12);

    FieldSample sample;
    field.evaluate(x, cp, v_c, &sample);
    ASSERT_TRUE(sample.valid);
    EXPECT_TRUE(Status::has(sample.status, Status::kAssumption3Clamped));
    EXPECT_NEAR(sample.Phi_T.norm(), (1.0 - eps_v) * v_r, 1e-12);
    EXPECT_NEAR(sample.Phi_T.normalized().dot(vf::feedforward(cp.T, v_c).normalized()), 1.0, 1e-12);
    EXPECT_GE(sample.eta, eps_v * v_r - 1e-12);
    EXPECT_NEAR(sample.Phi.norm(), v_r, 1e-12);
  }
  // Below the margin nothing happens.
  const vf::ClampedFeedforward below = vf::clampFeedforward(Vec2(0.0, 0.3), 0.4, 0.1);
  EXPECT_FALSE(below.clamped);
  EXPECT_EQ(below.Phi_T, Vec2(0.0, 0.3));
}

TEST(Field, SolveEtaIsTheLargerRootWithoutCancellation)
{
  // Collinear feedforward: (eta + p)^2 = v^2, so eta = v - p exactly; b > 0 is the
  // cancellation-prone branch.
  const Vec2 Phi_S(1.0, 0.0);
  for (const double p : {0.0, 0.1, 0.39, 0.399999, 0.3999999999}) {
    const double eta = vf::solveEta(Phi_S, Vec2(p, 0.0), 0.4);
    EXPECT_NEAR(eta, 0.4 - p, 1e-15 * 0.4) << "p = " << p;
    const double opposite = vf::solveEta(Phi_S, Vec2(-p, 0.0), 0.4);
    EXPECT_NEAR(opposite, 0.4 + p, 1e-15) << "p = " << p;
  }
  // Residual of the quadratic and Lemma 8's monotonicity in b.
  Rng rng(3);
  for (int i = 0; i < 5000; ++i) {
    const Vec2 S = rng.unit();
    const double v = rng.uniform(0.05, 2.0);
    const Vec2 P = rng.uniform(0.0, 0.999 * v) * rng.unit();
    const double eta = vf::solveEta(S, P, v);
    ASSERT_GT(eta, 0.0);
    EXPECT_NEAR((eta * S + P).norm(), v, 1e-14 * std::max(1.0, v));
  }
  // No real field: c_eta <= 0.
  EXPECT_TRUE(std::isnan(vf::solveEta(Phi_S, Vec2(0.0, 0.4), 0.4)));
  EXPECT_TRUE(std::isnan(vf::solveEta(Phi_S, Vec2(0.5, 0.0), 0.4)));
  EXPECT_TRUE(std::isnan(vf::solveEta(Phi_S, Vec2(vf::kNaN, 0.0), 0.4)));
}

// -----------------------------------------------------------------------------------------------
// Distance field.

// O17 (static): |‖Phi‖ - v_r| <= 1e-12 at random points, both gains, lines and circles, dir ±1.
TEST(Field, O17UnitNormStatic)
{
  Rng rng(17);
  double worst = 0.0;
  for (int i = 0; i < kRandomSamples; ++i) {
    const GainFunction gain = randomGain(rng);
    const double v_r = rng.uniform(0.05, 2.0);
    const DistanceField field(gain, v_r, rng.coin() ? 1 : -1, kEpsV);
    const RandomCurve curve = rng.curve();
    const Vec2 x = rng.pointNear(curve, 3.0);
    FieldSample sample;
    field.evaluate(x, curve.closest(x), Vec2::Zero(), &sample);
    ASSERT_TRUE(sample.valid);
    worst = std::max(worst, std::abs(sample.Phi.norm() - v_r));
    ASSERT_EQ(sample.Phi_T, Vec2::Zero());
    ASSERT_EQ(sample.eta, v_r);
    ASSERT_NEAR(sample.Phi_S.norm(), 1.0, 2e-15);
    ASSERT_EQ(sample.F, sample.Phi);
    ASSERT_EQ(sample.Phi_gamma, sample.Phi);
    ASSERT_EQ(sample.gamma, 1.0);
    ASSERT_TRUE(sample.uniform_shaping);
    ASSERT_EQ(sample.status, Status::kNone);
  }
  EXPECT_LE(worst, 1e-12);
}

// O17 with the rounding of a real curve: a frame orthogonal only to ~1e-12 still gives a unit
// field, because evaluate() re-orthogonalises n against T.
TEST(Field, O17UnitNormWithRoundedFrame)
{
  Rng rng(171);
  for (int i = 0; i < 5000; ++i) {
    const DistanceField field(randomGain(rng), kVr, 1, kEpsV);
    const RandomCurve curve = rng.curve();
    const Vec2 x = rng.pointNear(curve, 2.0);
    ClosestPoint cp = curve.closest(x);
    if (cp.n.isZero()) {
      continue;
    }
    cp.n = (cp.n + 1e-12 * rng.uniform(-1.0, 1.0) * cp.T).eval();
    cp.T = ((1.0 + 1e-13) * cp.T).eval();
    FieldSample sample;
    field.evaluate(x, cp, Vec2::Zero(), &sample);
    ASSERT_TRUE(sample.valid);
    EXPECT_NEAR(sample.Phi.norm(), kVr, 1e-15 * 8);
    EXPECT_NEAR(sample.n.dot(sample.T), 0.0, 1e-15);
  }
}

// O17 (time-varying): ||Phi_T|| up to 0.999 (1 - eps_v) v_r still gives ||Phi|| = v_r (1e-12).
TEST(Field, O17UnitNormTimeVarying)
{
  Rng rng(1717);
  double worst = 0.0;
  for (int i = 0; i < kRandomSamples; ++i) {
    const GainFunction gain = randomGain(rng);
    const double v_r = rng.uniform(0.05, 2.0);
    const double eps_v = rng.uniform(0.01, 0.5);
    const DistanceField field(gain, v_r, rng.coin() ? 1 : -1, eps_v);
    const RandomCurve curve = rng.curve();
    const Vec2 x = rng.pointNear(curve, 3.0);
    const ClosestPoint cp = curve.closest(x);
    const double normal_speed = rng.uniform(0.0, 0.999 * (1.0 - eps_v) * v_r);
    FieldSample sample;
    field.evaluate(x, cp, curveVelocity(rng, cp, normal_speed), &sample);
    ASSERT_TRUE(sample.valid);
    ASSERT_FALSE(Status::has(sample.status, Status::kAssumption3Clamped));
    ASSERT_NEAR(sample.Phi_T.norm(), normal_speed, 1e-14);
    worst = std::max(worst, std::abs(sample.Phi.norm() - v_r));
  }
  EXPECT_LE(worst, 1e-12);
}

// O18: Lemma 8, v_r - ||Phi_T|| <= eta <= v_r + ||Phi_T|| (1e-12), and D-dot = n^T(Phi - v_c)
// = -eta G (1e-12), with v_c carrying any tangential part.
TEST(Field, O18Lemma8AndDistanceRate)
{
  Rng rng(18);
  double worst_rate = 0.0;
  int clamped = 0;
  for (int i = 0; i < kRandomSamples; ++i) {
    const GainFunction gain = randomGain(rng);
    const double v_r = rng.uniform(0.05, 2.0);
    const double eps_v = rng.uniform(0.01, 0.5);
    const DistanceField field(gain, v_r, rng.coin() ? 1 : -1, eps_v);
    const RandomCurve curve = rng.curve();
    const Vec2 x = rng.pointNear(curve, 3.0);
    const ClosestPoint cp = curve.closest(x);
    // Up to 1.2 (1 - eps_v) v_r so that clamped samples are included: Lemma 8 then holds with
    // the clamped feedforward and eta >= eps_v v_r, but D-dot = -eta G needs Assumption 3.
    const Vec2 v_c = curveVelocity(rng, cp, rng.uniform(0.0, 1.2 * (1.0 - eps_v) * v_r));
    FieldSample sample;
    field.evaluate(x, cp, v_c, &sample);
    ASSERT_TRUE(sample.valid);
    const double p_T = sample.Phi_T.norm();
    ASSERT_GE(sample.eta, v_r - p_T - 1e-12);
    ASSERT_LE(sample.eta, v_r + p_T + 1e-12);
    ASSERT_GE(sample.eta, eps_v * v_r - 1e-12);
    if (Status::has(sample.status, Status::kAssumption3Clamped)) {
      ++clamped;
      continue;
    }
    const double D_dot = sample.n.dot(sample.Phi - v_c);
    worst_rate = std::max(worst_rate, std::abs(D_dot + sample.eta * sample.G));
  }
  EXPECT_LE(worst_rate, 1e-12);
  EXPECT_GT(clamped, 1000);
  EXPECT_LT(clamped, kRandomSamples / 2);
}

TEST(Field, StaticFieldMatchesTheDefinition)
{
  // math_spec §4.1 at a hand-checked point: line y = 0 in +x, point 0.2 below, atan k_f = 2.
  const GainFunction gain(GainType::kAtan, 2.0);
  const DistanceField field(gain, kVr, 1, kEpsV);
  FieldSample sample;
  const Vec2 x(1.5, -0.2);
  field.evaluate(x, lineClosestPoint(x, Vec2::Zero(), Vec2::UnitX()), Vec2::Zero(), &sample);
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.G, 0.242238, 1e-6);
  EXPECT_NEAR(sample.H, 0.970217, 1e-6);
  EXPECT_NEAR(sample.Phi_S.x(), sample.H, 1e-15);
  EXPECT_NEAR(sample.Phi_S.y(), sample.G, 1e-15);
  EXPECT_EQ(sample.x_eval, x);
  EXPECT_EQ(sample.x_star, Vec2(1.5, 0.0));
  EXPECT_DOUBLE_EQ(sample.s_star, 1.5);
  EXPECT_DOUBLE_EQ(sample.D, 0.2);
  EXPECT_EQ(sample.T_d, Vec2::UnitX());
  // conv + circ + Phi_T = Phi_gamma, with conv = -eta G n and circ = eta H T_d.
  EXPECT_NEAR((sample.conv + sample.circ + sample.Phi_T - sample.Phi_gamma).norm(), 0.0, 1e-15);
  EXPECT_NEAR((sample.conv - (-kVr * sample.G * sample.n)).norm(), 0.0, 1e-16);
  EXPECT_NEAR((sample.circ - kVr * sample.H * sample.T_d).norm(), 0.0, 1e-16);

  // Reverse direction: T_d = -T, the convergence part is unchanged.
  const DistanceField reverse(gain, kVr, -1, kEpsV);
  FieldSample back;
  reverse.evaluate(x, lineClosestPoint(x, Vec2::Zero(), Vec2::UnitX()), Vec2::Zero(), &back);
  EXPECT_EQ(back.T, Vec2::UnitX());
  EXPECT_EQ(back.T_d, -Vec2::UnitX());
  EXPECT_NEAR(back.Phi.y(), sample.Phi.y(), 1e-16);
  EXPECT_NEAR(back.Phi.x(), -sample.Phi.x(), 1e-16);

  // On the curve: n = 0, Phi = v_r T_d.
  FieldSample on;
  const Vec2 x_on(0.3, 0.0);
  field.evaluate(x_on, lineClosestPoint(x_on, Vec2::Zero(), Vec2::UnitX()), Vec2::Zero(), &on);
  EXPECT_EQ(on.n, Vec2::Zero());
  EXPECT_EQ(on.Phi, Vec2(kVr, 0.0));
}

TEST(Field, CircleRateIsMinusVrG)
{
  // O7's field part on an analytic circle: n^T Phi = -v_r G(D), T_d^T Phi = v_r H(D).
  const GainFunction gain(GainType::kAtan, 2.0);
  const DistanceField field(gain, kVr, 1, kEpsV);
  Rng rng(7);
  const Vec2 c(0.4, -0.2);
  for (int i = 0; i < 2000; ++i) {
    const Vec2 x = c + rng.uniform(0.05, 4.0) * rng.unit();
    const ClosestPoint cp = circleClosestPoint(x, c, 1.3);
    FieldSample sample;
    field.evaluate(x, cp, Vec2::Zero(), &sample);
    EXPECT_NEAR(sample.n.dot(sample.Phi), -kVr * gain.G(cp.D), 1e-15);
    EXPECT_NEAR(sample.T_d.dot(sample.Phi), kVr * gain.H(cp.D), 1e-15);
  }
}

TEST(Field, InvalidAndNonFiniteInputs)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  const Vec2 x(0.5, 0.3);
  const ClosestPoint good = lineClosestPoint(x, Vec2::Zero(), Vec2::UnitX());

  // Invalid closest point (empty curve): valid = false, the field is zero, no kNonFinite.
  ClosestPoint invalid;
  FieldSample a;
  field.evaluate(x, invalid, Vec2::Zero(), &a);
  EXPECT_FALSE(a.valid);
  EXPECT_EQ(a.F, Vec2::Zero());
  EXPECT_FALSE(Status::has(a.status, Status::kNonFinite));

  // Status bits of the search are passed on.
  ClosestPoint hairpin = good;
  hairpin.status_bits = Status::kHairpin;
  FieldSample b;
  field.evaluate(x, hairpin, Vec2::Zero(), &b);
  EXPECT_TRUE(b.valid);
  EXPECT_TRUE(Status::has(b.status, Status::kHairpin));

  // Non-finite inputs: valid = false, kNonFinite, a zero field, never an exception.
  FieldSample c;
  field.evaluate(Vec2(vf::kNaN, 0.0), good, Vec2::Zero(), &c);
  EXPECT_FALSE(c.valid);
  EXPECT_TRUE(Status::has(c.status, Status::kNonFinite));
  EXPECT_EQ(c.F, Vec2::Zero());

  FieldSample d;
  field.evaluate(x, good, Vec2(vf::kInf, 0.0), &d);
  EXPECT_FALSE(d.valid);
  EXPECT_TRUE(Status::has(d.status, Status::kNonFinite));

  ClosestPoint bad_frame = good;
  bad_frame.T = Vec2::Zero();
  FieldSample e;
  field.evaluate(x, bad_frame, Vec2::Zero(), &e);
  EXPECT_FALSE(e.valid);
  EXPECT_TRUE(Status::has(e.status, Status::kNonFinite));

  const TerminalProfile profile(kVr, 0.05, 0.5, 0.125);
  FieldSample f;
  field.evaluateTerminal(x, good, vf::kNaN, profile, &f);
  EXPECT_FALSE(f.valid);
  EXPECT_TRUE(Status::has(f.status, Status::kNonFinite));
  EXPECT_EQ(f.F, Vec2::Zero());

  // NaN gamma.
  FieldSample g;
  field.evaluate(x, good, Vec2::Zero(), &g);
  field.shape(vf::kNaN, 1.0, false, &g);
  EXPECT_FALSE(g.valid);
  EXPECT_TRUE(Status::has(g.status, Status::kNonFinite));
  EXPECT_EQ(g.F, Vec2::Zero());

  // A null sample is ignored.
  field.evaluate(x, good, Vec2::Zero(), nullptr);
  field.shape(0.5, 1.0, false, nullptr);
}

// -----------------------------------------------------------------------------------------------
// Terminal field (math_spec §5).

TEST(TerminalField, ValuesOnAStraightApproach)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const TerminalProfile profile(kVr, 0.05, 0.5, 0.125);
  Rng rng(51);
  for (const int dir : {1, -1}) {
    const DistanceField field(gain, kVr, dir, kEpsV);
    // Line through the origin along +x; the goal station is the end (dir +1, s = 4) or the
    // start (dir -1, s = 0).
    const double s_goal = dir > 0 ? 4.0 : 0.0;
    for (int i = 0; i < 5000; ++i) {
      const Vec2 x(rng.uniform(-3.0, 7.0), rng.uniform(-1.5, 1.5));
      const ClosestPoint cp = lineClosestPoint(x, Vec2::Zero(), Vec2::UnitX());
      const double sigma = dir * (s_goal - cp.s_star);
      FieldSample sample;
      field.evaluateTerminal(x, cp, sigma, profile, &sample);
      ASSERT_TRUE(sample.valid);
      const double phi = profile(sigma);
      EXPECT_TRUE(sample.terminal);
      EXPECT_EQ(sample.phi, phi);
      EXPECT_EQ(sample.Phi_T, Vec2::Zero());
      EXPECT_EQ(sample.v_c, Vec2::Zero());
      EXPECT_EQ(sample.eta, kVr);
      EXPECT_TRUE(sample.uniform_shaping);
      const double G = gain.G(cp.D);
      const double H = gain.H(cp.D);
      EXPECT_NEAR(sample.Phi.norm(), kVr * std::sqrt(G * G + H * H * phi * phi), 1e-12);
      EXPECT_NEAR(sample.n.dot(sample.Phi), -kVr * G, 1e-15);
      EXPECT_NEAR(sample.T_d.dot(sample.Phi), kVr * H * phi, 1e-15);
      EXPECT_NEAR((sample.circ - kVr * H * phi * sample.T_d).norm(), 0.0, 1e-15);
      EXPECT_NEAR((sample.conv + sample.circ - sample.Phi_gamma).norm(), 0.0, 1e-15);
    }
  }
}

// Reference-point shift (math_spec §5.2): a law whose point sits x_ref ahead of base_link
// evaluates sigma_ref = dir (s_goal + dir x_ref - s*_ref). On an aligned approach sigma_ref equals
// sigma_base, so the shifted law sees the speed demand of base_link and g_ref = g + x_ref T_end.
TEST(TerminalField, ReferenceShiftMatchesBaseLink)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const TerminalProfile profile(kVr, 0.05, 0.5, 0.125);
  for (const int dir : {1, -1}) {
    const DistanceField field(gain, kVr, dir, kEpsV);
    const double s_goal = dir > 0 ? 3.0 : 0.0;
    const Vec2 T_end = dir * Vec2::UnitX();
    for (const double x_ref : {-0.3, 0.0, 0.2, 0.3}) {
      for (const double s_base : {-1.0, 0.5, 1.7, 2.6, 2.95, 3.4}) {
        // base_link on the path, heading along T_d; the reference point x_ref ahead.
        const Vec2 base(s_base, 0.0);
        const Vec2 ref = base + x_ref * T_end;
        const ClosestPoint cp_base = lineClosestPoint(base, Vec2::Zero(), Vec2::UnitX());
        const ClosestPoint cp_ref = lineClosestPoint(ref, Vec2::Zero(), Vec2::UnitX());
        const double sigma_base = dir * (s_goal - cp_base.s_star);
        const double sigma_ref = dir * (s_goal + dir * x_ref - cp_ref.s_star);
        EXPECT_NEAR(sigma_ref, sigma_base, 1e-12);

        FieldSample at_base;
        FieldSample at_ref;
        field.evaluateTerminal(base, cp_base, sigma_base, profile, &at_base);
        field.evaluateTerminal(ref, cp_ref, sigma_ref, profile, &at_ref);
        EXPECT_NEAR(at_ref.phi, at_base.phi, 1e-12);
        EXPECT_NEAR((at_ref.Phi - at_base.Phi).norm(), 0.0, 1e-12);

        // g_ref = r(s_goal + dir x_ref) = g + x_ref T_end.
        const Vec2 g(s_goal, 0.0);
        const Vec2 g_ref(s_goal + dir * x_ref, 0.0);
        EXPECT_NEAR((g_ref - (g + x_ref * T_end)).norm(), 0.0, 1e-15);
      }
    }
  }

  // The reference offsets come from the evaluation-point table (math_spec §7.1).
  vf::Kinematics fl;
  fl.control_point_offset = 0.2;
  EXPECT_DOUBLE_EQ(
    vf::evaluationPoint(
      vf::Pose2{}, vf::Phase::kFollow, vf::TrackingLaw::kFeedbackLinearization, fl)
      .x_ref,
    0.2);
  vf::Kinematics car;
  car.type = vf::MotionModel::kAckermann;
  car.reference_point_offset = -0.3;
  EXPECT_DOUBLE_EQ(
    vf::evaluationPoint(vf::Pose2{}, vf::Phase::kFollow, vf::TrackingLaw::kHeadingTracking, car)
      .x_ref,
    -0.3);
}

// -----------------------------------------------------------------------------------------------
// Speed shaping (math_spec §8.2).

namespace
{

// A time-varying sample with feedforward norm p_T (line y = 0, point 0.3 below).
FieldSample timeVaryingSample(const DistanceField & field, double p_T, double tangential = 0.2)
{
  const Vec2 x(1.0, -0.3);
  const ClosestPoint cp = lineClosestPoint(x, Vec2::Zero(), Vec2::UnitX());
  FieldSample sample;
  field.evaluate(x, cp, Vec2(tangential, p_T), &sample);
  return sample;
}

}  // namespace

TEST(FieldShaping, StaticFieldScalesUniformly)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  Rng rng(81);
  for (int i = 0; i < 2000; ++i) {
    const RandomCurve curve = rng.curve();
    const Vec2 x = rng.pointNear(curve, 2.0);
    FieldSample sample;
    field.evaluate(x, curve.closest(x), Vec2::Zero(), &sample);
    const Vec2 Phi = sample.Phi;
    const double gamma = rng.uniform(0.0, 1.0);
    field.shape(gamma, rng.uniform(0.05, 1.0), false, &sample);
    ASSERT_TRUE(sample.valid);
    EXPECT_TRUE(sample.uniform_shaping);
    EXPECT_EQ(sample.gamma, gamma);
    EXPECT_NEAR((sample.Phi_gamma - gamma * Phi).norm(), 0.0, 1e-16);
    EXPECT_NEAR(sample.Phi_gamma.norm(), gamma * kVr, 1e-12);
    EXPECT_DOUBLE_EQ(sample.eta, gamma * kVr);
    EXPECT_EQ(sample.F, sample.Phi_gamma);
    EXPECT_EQ(sample.Phi, Phi) << "the nominal field is kept";
    EXPECT_NEAR((sample.conv + sample.circ + sample.Phi_T - sample.Phi_gamma).norm(), 0.0, 1e-15);
    EXPECT_EQ(sample.status, Status::kNone);
  }
}

TEST(FieldShaping, TimeVaryingReSolvesEtaAndKeepsTheFeedforward)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  // ||Phi_T|| = 0.2 < (1 - eps_v) gamma v_r = 0.9 * 0.8 * 0.4 = 0.288.
  FieldSample sample = timeVaryingSample(field, 0.2);
  ASSERT_TRUE(sample.valid);
  EXPECT_FALSE(sample.uniform_shaping) << "unshaped time-varying sample: eta branch";
  const Vec2 Phi_T = sample.Phi_T;
  field.shape(0.8, 1.0, false, &sample);
  EXPECT_FALSE(sample.uniform_shaping);
  EXPECT_EQ(sample.Phi_T, Phi_T);
  EXPECT_NEAR(sample.Phi_gamma.norm(), 0.8 * kVr, 1e-12);
  EXPECT_DOUBLE_EQ(sample.eta, vf::solveEta(sample.Phi_S, Phi_T, 0.8 * kVr));
  EXPECT_NEAR((sample.Phi_gamma - (sample.eta * sample.Phi_S + Phi_T)).norm(), 0.0, 1e-16);
  EXPECT_NEAR((sample.conv + sample.circ + sample.Phi_T - sample.Phi_gamma).norm(), 0.0, 1e-15);
  EXPECT_FALSE(Status::has(sample.status, Status::kAssumption3Clamped));
  // D-dot = -eta_1 G: the shaped field still converges.
  EXPECT_NEAR(sample.n.dot(sample.Phi_gamma - sample.v_c), -sample.eta * sample.G, 1e-15);
}

TEST(FieldShaping, SafetyLimitWinsAboveTheMargin)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  // gamma = 0.3 (collision cap): (1 - eps_v) gamma v_r = 0.108 <= ||Phi_T|| = 0.2.
  FieldSample sample = timeVaryingSample(field, 0.2);
  const Vec2 Phi = sample.Phi;
  const Vec2 Phi_T = sample.Phi_T;
  const double eta = sample.eta;
  field.shape(0.3, 1.0, false, &sample);
  EXPECT_TRUE(sample.uniform_shaping);
  EXPECT_NEAR((sample.Phi_gamma - 0.3 * Phi).norm(), 0.0, 1e-16);
  EXPECT_NEAR(sample.Phi_gamma.norm(), 0.3 * kVr, 1e-12);
  EXPECT_NEAR((sample.Phi_T - 0.3 * Phi_T).norm(), 0.0, 1e-16) << "the whole field is scaled";
  EXPECT_DOUBLE_EQ(sample.eta, 0.3 * eta);
  EXPECT_NEAR((sample.conv + sample.circ + sample.Phi_T - sample.Phi_gamma).norm(), 0.0, 1e-15);
  // ||Phi_T|| = 0.2 < (1 - eps_v) v_r min(1, gamma_lim) = 0.36: the collision cap is reported by
  // STATUS_COLLISION_AHEAD or kBlocked, never by kAssumption3Clamped.
  EXPECT_FALSE(Status::has(sample.status, Status::kAssumption3Clamped));
}

TEST(FieldShaping, Assumption3FlagFollowsTheExternalLimitOnly)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  // ||Phi_T|| = 0.2 >= (1 - eps_v) v_r gamma_lim = 0.9 * 0.4 * 0.5 = 0.18: the curve cannot be
  // tracked at the externally limited speed.
  FieldSample limited = timeVaryingSample(field, 0.2);
  field.shape(0.5, 0.5, false, &limited);
  EXPECT_TRUE(Status::has(limited.status, Status::kAssumption3Clamped));
  EXPECT_TRUE(limited.uniform_shaping);

  // The same gamma from the clearance or the collision cap: no flag.
  FieldSample clearance = timeVaryingSample(field, 0.2);
  field.shape(0.5, 1.0, false, &clearance);
  EXPECT_FALSE(Status::has(clearance.status, Status::kAssumption3Clamped));

  // gamma_lim above 1 counts as 1.
  FieldSample high = timeVaryingSample(field, 0.2);
  field.shape(1.0, 1.7, false, &high);
  EXPECT_FALSE(Status::has(high.status, Status::kAssumption3Clamped));

  // A feedforward clamped by evaluate() stays flagged; at gamma = 1 it sits at the margin, where
  // the safety limit wins deterministically.
  FieldSample clamped = timeVaryingSample(field, 0.5);
  EXPECT_TRUE(Status::has(clamped.status, Status::kAssumption3Clamped));
  EXPECT_TRUE(clamped.uniform_shaping);
  field.shape(1.0, 1.0, false, &clamped);
  EXPECT_TRUE(clamped.uniform_shaping);
  EXPECT_NEAR(clamped.Phi_gamma.norm(), kVr, 1e-12);
}

// The flag rule and the branches over random samples: kAssumption3Clamped iff
// ||Phi_T|| >= (1 - eps_v) v_r min(1, gamma_lim), the eta branch iff ||Phi_T|| <
// (1 - eps_v) gamma v_r, and ||Phi_gamma|| = gamma v_r in both.
TEST(FieldShaping, RandomBranchesAndFlagRule)
{
  Rng rng(82);
  int eta_branch = 0;
  int flagged = 0;
  for (int i = 0; i < kRandomSamples; ++i) {
    const double v_r = rng.uniform(0.1, 1.0);
    const double eps_v = rng.uniform(0.05, 0.3);
    const DistanceField field(randomGain(rng), v_r, rng.coin() ? 1 : -1, eps_v);
    const RandomCurve curve = rng.curve();
    const Vec2 x = rng.pointNear(curve, 2.0);
    const ClosestPoint cp = curve.closest(x);
    const double p_T = rng.uniform(0.0, (1.0 - eps_v) * v_r);
    FieldSample sample;
    field.evaluate(x, cp, curveVelocity(rng, cp, p_T), &sample);
    ASSERT_TRUE(sample.valid);
    const double gamma_lim = rng.uniform(0.05, 1.0);
    const double gamma = std::min(gamma_lim, rng.uniform(0.0, 1.2));
    const bool force = rng.uniform(0.0, 1.0) < 0.1;
    const double flag_threshold = (1.0 - eps_v) * v_r * std::min(1.0, gamma_lim);
    const double branch_threshold = (1.0 - eps_v) * gamma * v_r;
    // Stay away from the boundaries; they are exercised above.
    if (
      std::abs(p_T - flag_threshold) < 1e-9 || std::abs(p_T - branch_threshold) < 1e-9 ||
      p_T == 0.0) {
      continue;
    }
    const double clamped_gamma = std::min(1.0, gamma);
    field.shape(clamped_gamma, gamma_lim, force, &sample);
    ASSERT_TRUE(sample.valid);
    const bool expect_eta = !force && p_T < (1.0 - eps_v) * clamped_gamma * v_r;
    ASSERT_EQ(!sample.uniform_shaping, expect_eta);
    ASSERT_EQ(Status::has(sample.status, Status::kAssumption3Clamped), p_T >= flag_threshold);
    ASSERT_NEAR(sample.Phi_gamma.norm(), clamped_gamma * v_r, 1e-12);
    eta_branch += expect_eta ? 1 : 0;
    flagged += p_T >= flag_threshold ? 1 : 0;
  }
  EXPECT_GT(eta_branch, 1000);
  EXPECT_GT(flagged, 1000);
}

TEST(FieldShaping, ForcedUniformDuringAContourBlend)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  FieldSample sample = timeVaryingSample(field, 0.1);
  const Vec2 Phi = sample.Phi;
  field.shape(0.7, 1.0, true, &sample);
  EXPECT_TRUE(sample.uniform_shaping);
  EXPECT_NEAR((sample.Phi_gamma - 0.7 * Phi).norm(), 0.0, 1e-16);
  EXPECT_FALSE(Status::has(sample.status, Status::kAssumption3Clamped));
}

TEST(FieldShaping, ZeroGammaStopsWithoutTheFlag)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  FieldSample sample = timeVaryingSample(field, 0.2);
  field.shape(0.0, 1.0, false, &sample);
  EXPECT_TRUE(sample.valid);
  EXPECT_EQ(sample.Phi_gamma, Vec2::Zero());
  EXPECT_EQ(sample.F, Vec2::Zero());
  EXPECT_EQ(sample.eta, 0.0);
  EXPECT_FALSE(Status::has(sample.status, Status::kAssumption3Clamped));
  // A negative gamma counts as 0.
  FieldSample negative = timeVaryingSample(field, 0.2);
  field.shape(-0.5, 1.0, false, &negative);
  EXPECT_EQ(negative.F, Vec2::Zero());
}

// shape() works from the nominal quantities, so re-shaping one evaluated sample (the Tracker's
// re-run with the collision cap) equals shaping it once.
TEST(FieldShaping, IsIdempotentOnAnEvaluatedSample)
{
  const DistanceField field(GainFunction(), kVr, 1, kEpsV);
  for (const double p_T : {0.0, 0.1, 0.3}) {
    for (const double first : {1.0, 0.6, 0.2}) {
      for (const double second : {1.0, 0.5, 0.25, 0.0}) {
        FieldSample twice = timeVaryingSample(field, p_T);
        FieldSample once = twice;
        field.shape(first, 1.0, false, &twice);
        field.shape(second, 1.0, false, &twice);
        field.shape(second, 1.0, false, &once);
        EXPECT_EQ(twice.Phi_gamma, once.Phi_gamma);
        EXPECT_EQ(twice.Phi_T, once.Phi_T);
        EXPECT_EQ(twice.eta, once.eta);
        EXPECT_EQ(twice.uniform_shaping, once.uniform_shaping);
      }
    }
  }
  // Terminal samples too.
  const TerminalProfile profile(kVr, 0.05, 0.5, 0.125);
  const Vec2 x(1.0, -0.3);
  FieldSample terminal;
  field.evaluateTerminal(
    x, lineClosestPoint(x, Vec2::Zero(), Vec2::UnitX()), 0.3, profile, &terminal);
  const Vec2 Phi = terminal.Phi;
  field.shape(0.4, 1.0, false, &terminal);
  field.shape(0.6, 1.0, false, &terminal);
  EXPECT_NEAR((terminal.Phi_gamma - 0.6 * Phi).norm(), 0.0, 1e-16);
  EXPECT_NEAR(
    (terminal.circ - 0.6 * kVr * terminal.H * terminal.phi * terminal.T_d).norm(), 0.0, 1e-15);
}

// -----------------------------------------------------------------------------------------------
// Streamlines (math_spec Appendix A).

TEST(Field, StreamlineFollowsAStaticCircle)
{
  const GainFunction gain(GainType::kAtan, 2.0);
  const DistanceField field(gain, kVr, 1, kEpsV);
  const Vec2 centre(1.0, -0.5);
  const auto circle_field = [&](const Vec2 & x) {
    FieldSample sample;
    field.evaluate(x, circleClosestPoint(x, centre, 1.0), Vec2::Zero(), &sample);
    return sample.F;
  };
  std::vector<Vec2> points(200);
  const std::size_t count =
    vf::streamline(circle_field, centre + Vec2(1.0, 0.0), 0.05, vf::Span<Vec2>(points));
  ASSERT_EQ(count, points.size());
  for (const Vec2 & p : points) {
    EXPECT_NEAR((p - centre).norm(), 1.0, 1e-6);
  }
  // Counter-clockwise: after 199 steps of 0.05 m the angle is 9.95 rad.
  const double angle = std::atan2(points.back().y() - centre.y(), points.back().x() - centre.x());
  EXPECT_NEAR(vf::wrapAngle(angle - 9.95), 0.0, 1e-4);

  // From outside the streamline converges to the circle.
  std::vector<Vec2> inbound(400);
  vf::streamline(circle_field, centre + Vec2(2.0, 0.0), 0.05, vf::Span<Vec2>(inbound));
  EXPECT_LT(std::abs((inbound.back() - centre).norm() - 1.0), 0.02);
}

TEST(Field, StreamlineStopsOnZeroOrNonFiniteFields)
{
  std::vector<Vec2> points(10);
  const auto zero = [](const Vec2 &) { return Vec2(Vec2::Zero()); };
  EXPECT_EQ(vf::streamline(zero, Vec2(1.0, 2.0), 0.1, vf::Span<Vec2>(points)), 1U);
  EXPECT_EQ(points[0], Vec2(1.0, 2.0));
  const auto nan = [](const Vec2 &) { return Vec2(vf::kNaN, 0.0); };
  EXPECT_EQ(vf::streamline(nan, Vec2::Zero(), 0.1, vf::Span<Vec2>(points)), 1U);
  // The field ends beyond x = 0.27: the step from x = 0.2 has a stage at 0.3.
  const auto wall = [](const Vec2 & x) {
    return x.x() < 0.27 ? Vec2(Vec2::UnitX()) : Vec2(Vec2::Zero());
  };
  EXPECT_EQ(vf::streamline(wall, Vec2::Zero(), 0.1, vf::Span<Vec2>(points)), 3U);
  const auto unit = [](const Vec2 &) { return Vec2(3.0, 4.0); };
  EXPECT_EQ(vf::streamline(unit, Vec2::Zero(), 0.1, vf::Span<Vec2>(points)), 10U);
  EXPECT_NEAR((points[9] - 0.9 * Vec2(0.6, 0.8)).norm(), 0.0, 1e-14);
  EXPECT_EQ(vf::streamline(unit, Vec2(vf::kNaN, 0.0), 0.1, vf::Span<Vec2>(points)), 0U);
  EXPECT_EQ(vf::streamline(unit, Vec2::Zero(), 0.1, vf::Span<Vec2>()), 0U);
  EXPECT_EQ(vf::streamline(unit, Vec2::Zero(), 0.0, vf::Span<Vec2>(points)), 1U);
}
