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

// 3D field oracles (math_spec §14.3, §14.6): O3D-1 ‖Φ‖ = v_r, O3D-4 Ḋ = −v_r G on an inclined
// circle, O3D-5 the feedforward of a translating circle, O3D-6 the η bounds; the terminal field,
// the law's box and the limiter.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "vectorfield_core/d3.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace d3 = vectorfield_core::d3;
using d3::Vec3;

constexpr double kVr = 0.5;

d3::DistanceField3 field(int direction = 1)
{
  return d3::DistanceField3(vf::GainFunction(vf::GainType::kAtan, 2.0), kVr, direction, 0.1);
}

d3::CurveSpec3 builtin(
  const std::string & name, std::vector<std::string> names = {}, std::vector<double> values = {})
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kBuiltin;
  spec.builtin_shape = name;
  spec.parameter_names = std::move(names);
  spec.parameter_values = std::move(values);
  return spec;
}

d3::CurveSpec3 expression(const std::string & x, const std::string & y, const std::string & z)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kExpression;
  spec.x_expression = x;
  spec.y_expression = y;
  spec.z_expression = z;
  spec.s_max = 2.0 * vf::kPi;
  spec.closed = true;
  return spec;
}

TEST(D3Field, UnitSpeedEverywhereStatic)
{
  std::vector<std::shared_ptr<const d3::Curve3>> curves;
  for (const vf::BuiltinShapeInfo & info : d3::builtinShapes3()) {
    curves.push_back(d3::makeCurve3(builtin(info.name), {}));
  }
  d3::CurveSpec3 poly;
  poly.points = {Vec3(0, 0, 0), Vec3(1, 0.3, 0.2), Vec3(1.5, 1.2, -0.4), Vec3(2.7, 1.0, 0.5)};
  curves.push_back(d3::makeCurve3(poly, {}));
  curves.push_back(d3::makeCurve3(expression("2*cos(s)", "sin(s)", "0.3*sin(3*s)"), {}));
  for (const int direction : {1, -1}) {
    const auto f = field(direction);
    std::mt19937_64 rng(5);
    std::uniform_real_distribution<double> box(-3.0, 3.0);
    double worst = 0.0;
    for (const auto & curve : curves) {
      for (int i = 0; i < 2000; ++i) {
        const Vec3 x(box(rng), box(rng), box(rng));
        const auto cp = curve->closestPoint(x, d3::EvalContext3{0.0, direction, {}}, {});
        d3::FieldSample3 s;
        f.evaluate(x, cp, Vec3::Zero(), &s);
        ASSERT_TRUE(s.valid);
        worst = std::max(worst, std::abs(s.Phi.norm() - kVr));
        EXPECT_NEAR(s.n.dot(s.T), 0.0, 1e-12);
      }
    }
    EXPECT_LT(worst, 1e-12);
  }
}

TEST(D3Field, InclinedCircleDistanceDecreasesAtVrG)
{
  const auto circle = d3::makeCurve3(
    builtin("circle", {"radius", "cz", "yaw", "pitch", "roll"}, {1.2, 0.5, 0.3, 0.6, -0.4}), {});
  const auto f = field();
  std::mt19937_64 rng(9);
  std::uniform_real_distribution<double> s_dist(0.0, 7.5);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (int i = 0; i < 3000; ++i) {
    // Points inside the reach (D < 0.5 / kappa = 0.6).
    const Vec3 foot = circle->pointAt(s_dist(rng), 0.0);
    Vec3 offset(unit(rng), unit(rng), unit(rng));
    offset = 0.5 * offset.normalized() * std::abs(unit(rng));
    const Vec3 x = foot + offset;
    const auto cp = circle->closestPoint(x, {}, {});
    d3::FieldSample3 s;
    f.evaluate(x, cp, Vec3::Zero(), &s);
    ASSERT_TRUE(s.valid);
    // Ḋ = nᵀΦ = −v_r G(D) (O3D-4), and the finite difference of D along Φ agrees.
    EXPECT_NEAR(s.n.dot(s.Phi), -kVr * s.G, 1e-12);
    if (cp.D > 1e-3) {
      const double h = 1e-5;
      const double D_plus = circle->closestPoint(x + h * s.Phi, {}, {}).D;
      const double D_minus = circle->closestPoint(x - h * s.Phi, {}, {}).D;
      EXPECT_NEAR((D_plus - D_minus) / (2.0 * h), -kVr * s.G, 1e-6);
    }
  }
}

TEST(D3Field, TranslatingCircleFeedforward)
{
  const Vec3 u(0.12, -0.05, 0.08);
  const auto curve =
    d3::makeCurve3(expression("cos(s) + 0.12*t", "sin(s) - 0.05*t", "0.3*sin(s) + 0.08*t"), {});
  ASSERT_TRUE(curve->isTimeVarying());
  const auto f = field();
  std::mt19937_64 rng(13);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> time(0.0, 20.0);
  for (int i = 0; i < 500; ++i) {
    const double t = time(rng);
    const Vec3 x = curve->pointAt(2.0 * vf::kPi * 0.5 * (unit(rng) + 1.0), t) +
                   0.4 * Vec3(unit(rng), unit(rng), unit(rng));
    d3::EvalContext3 context;
    context.t = t;
    const auto cp = curve->closestPoint(x, context, {});
    const Vec3 v_c = curve->velocity(cp, t);
    EXPECT_LT((v_c - u).norm(), 1e-9);
    d3::FieldSample3 s;
    f.evaluate(x, cp, v_c, &s);
    ASSERT_TRUE(s.valid);
    const Vec3 expected = (d3::Mat3::Identity() - cp.T * cp.T.transpose()) * u;
    EXPECT_LT((s.Phi_T - expected).norm(), 1e-9);  // O3D-5
    EXPECT_NEAR(s.Phi.norm(), kVr, 1e-12);
    // Ḋ = nᵀ(Φ − v_c) = −η G (Proposition 2).
    EXPECT_NEAR(s.n.dot(s.Phi - v_c), -s.eta * s.G, 1e-9);
  }
}

TEST(D3Field, EtaBoundsAndClamp)
{
  const auto f = field();
  std::mt19937_64 rng(17);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (int i = 0; i < 10000; ++i) {
    d3::ClosestPoint3 cp;
    cp.valid = true;
    cp.T = Vec3(unit(rng), unit(rng), unit(rng)).normalized();
    cp.n = d3::rejectFrom(Vec3(unit(rng), unit(rng), unit(rng)), cp.T).normalized();
    cp.D = std::abs(unit(rng));
    cp.x_star = Vec3::Zero();
    const Vec3 x = cp.D * cp.n;
    const Vec3 v_c = 0.6 * kVr * Vec3(unit(rng), unit(rng), unit(rng));
    d3::FieldSample3 s;
    f.evaluate(x, cp, v_c, &s);
    ASSERT_TRUE(s.valid);
    const double p_T = s.Phi_T.norm();
    EXPECT_LE(p_T, 0.9 * kVr + 1e-15);
    EXPECT_GE(s.eta, kVr - p_T - 1e-12);  // O3D-6
    EXPECT_LE(s.eta, kVr + p_T + 1e-12);
    EXPECT_NEAR(s.Phi.norm(), kVr, 1e-12);
  }
  // The Assumption-3 clamp.
  d3::ClosestPoint3 cp;
  cp.valid = true;
  cp.D = 0.2;
  cp.n = Vec3::UnitY();
  d3::FieldSample3 s;
  f.evaluate(Vec3(0, 0.2, 0), cp, Vec3(0, 0, 2.0), &s);
  EXPECT_TRUE(s.status & vf::Status::kAssumption3Clamped);
  EXPECT_NEAR(s.Phi_T.norm(), 0.9 * kVr, 1e-15);
  // Invalid input.
  f.evaluate(Vec3(0, 0, 0), d3::ClosestPoint3{}, Vec3::Zero(), &s);
  EXPECT_FALSE(s.valid);
  EXPECT_TRUE(s.status & vf::Status::kNonFinite);
  EXPECT_EQ(s.Phi, Vec3::Zero());
}

TEST(D3Field, TerminalFieldStopsAtTheGoal)
{
  const auto curve = d3::makeCurve3(builtin("helix"), {});
  const auto f = field();
  const vf::TerminalProfile profile(kVr, 0.05, 0.5, 0.05);
  const double L = curve->length(0.0);
  for (const double s : {0.5 * L, L - 0.3, L - 0.01, L, L + 0.2}) {
    const Vec3 x = curve->pointAt(s, 0.0) + Vec3(0, 0, 0.05);
    const auto cp = curve->closestPoint(x, {}, {});
    d3::FieldSample3 sample;
    f.evaluateTerminal(x, cp, L - cp.s_star, profile, &sample);
    ASSERT_TRUE(sample.valid);
    EXPECT_NEAR(sample.phi, profile(L - cp.s_star), 1e-15);
    EXPECT_NEAR(sample.Phi.norm(), kVr * std::hypot(sample.G, sample.H * sample.phi), 1e-12);
    if (s > L) {
      EXPECT_LT(sample.Phi.dot(cp.T), 0.0);  // drives back toward the goal
    }
  }
}

TEST(D3Law, BoxKeepsDirectionAndEtaBoxKeepsFeedforward)
{
  d3::Limits3 limits;
  limits.max_speed = 0.4;
  limits.max_axis_speed = Vec3(1.0, 1.0, 0.1);
  const d3::VelocityLaw3 law(limits, d3::YawConfig3{});
  d3::FieldSample3 s;
  s.valid = true;
  s.T_d = Vec3(1, 0, 1).normalized();
  s.Phi_S = s.T_d;
  s.Phi = kVr * s.Phi_S;
  auto out = law.command(s, 1.0, kVr, 0.1, 0.0, vf::kNaN);
  EXPECT_NEAR(std::abs(out.cmd.v.z()), 0.1, 1e-15);
  EXPECT_LT((out.cmd.v.normalized() - s.Phi.normalized()).norm(), 1e-12);
  EXPECT_NEAR(out.psi_d, 0.0, 1e-15);
  // Time-varying: the eta-box keeps Phi_T and lowers eta only.
  s.Phi_T = Vec3(0.0, 0.05, 0.0);
  s.Phi_S = Vec3(1, 0, 0);
  s.eta = d3::solveEta(s.Phi_S, s.Phi_T, kVr);
  s.Phi = s.eta * s.Phi_S + s.Phi_T;
  out = law.command(s, 1.0, kVr, 0.1, 0.0, vf::kNaN);
  ASSERT_TRUE(out.eta_box);
  EXPECT_NEAR(out.cmd.v.norm(), 0.4, 1e-12);
  EXPECT_NEAR(out.cmd.v.y(), 0.05, 1e-15);
  // Vertical source: the yaw is held.
  d3::FieldSample3 up;
  up.valid = true;
  up.T_d = Vec3::UnitZ();
  up.Phi_S = up.T_d;
  up.Phi = kVr * up.T_d;
  out = law.command(up, 1.0, kVr, 0.1, 0.3, 1.0);
  EXPECT_DOUBLE_EQ(out.psi_d, 1.0);
  EXPECT_NEAR(out.cmd.wz, std::min(1.5, 2.0 * (1.0 - 0.3)), 1e-15);
}

TEST(D3Law, LimiterUniformAlphaAndGuard)
{
  const d3::CommandLimiter3 limiter;
  d3::Limits3 limits;
  d3::LimiterState3 state;
  state.valid = true;
  d3::Twist3 target;
  target.v = Vec3(0.5, -0.5, 0.2);
  target.wz = 1.0;
  std::uint32_t status = 0;
  double alpha = 0.0;
  const d3::Twist3 u =
    limiter.limit(target, state, &state, 0.05, limits, false, false, &status, &alpha);
  // One alpha: the command moves along the straight line toward the target.
  EXPECT_NEAR(alpha, 0.1, 1e-15);
  EXPECT_LT((u.v - 0.1 * target.v).norm(), 1e-15);
  EXPECT_NEAR(u.wz, 0.1, 1e-15);
  target.v.x() = vf::kNaN;
  limiter.limit(target, state, &state, 0.05, limits, false, false, &status, nullptr);
  EXPECT_TRUE(status & vf::Status::kNonFinite);
  EXPECT_EQ(state.u_prev.v, Vec3::Zero());
}

}  // namespace
