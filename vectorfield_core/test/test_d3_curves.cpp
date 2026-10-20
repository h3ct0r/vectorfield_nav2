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

// 3D core geometry (math_spec §14.1–§14.2, oracles O3D-2 and O3D-3): polylines with wedges and
// hairpins, the planar reduction of the wedge blend, the helix's analytic foot, built-in shapes at
// their defaults, and the expression hardening shared with the planar core.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/d3.hpp"
#include "vectorfield_core/polyline_curve.hpp"

namespace
{

namespace vf = vectorfield_core;
namespace d3 = vectorfield_core::d3;
using d3::Vec3;

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

d3::CurveSpec3 expression(
  const std::string & x, const std::string & y, const std::string & z, bool closed,
  double s_max = 2.0 * vf::kPi)
{
  d3::CurveSpec3 spec;
  spec.type = d3::CurveSpec3::Type::kExpression;
  spec.x_expression = x;
  spec.y_expression = y;
  spec.z_expression = z;
  spec.s_min = 0.0;
  spec.s_max = s_max;
  spec.closed = closed;
  return spec;
}

void expectFrame(const d3::ClosestPoint3 & cp, const char * where)
{
  ASSERT_TRUE(cp.valid) << where;
  EXPECT_NEAR(cp.T.norm(), 1.0, 1e-12) << where;
  EXPECT_NEAR(cp.n.dot(cp.T), 0.0, 1e-12) << where;
  if (cp.D >= vf::kEpsD) {
    EXPECT_NEAR(cp.n.norm(), 1.0, 1e-12) << where;
  }
}

TEST(D3Polyline, SegmentsRaysAndStations)
{
  const d3::PolylineCurve3 curve({Vec3(0, 0, 0), Vec3(2, 0, 0), Vec3(2, 0, 3)}, false);
  EXPECT_DOUBLE_EQ(curve.length(0.0), 5.0);
  const d3::CurveHint3 none;
  const d3::EvalContext3 context;
  // Segment interior.
  auto cp = curve.closestPoint(Vec3(1, 0.5, 0.2), context, none);
  expectFrame(cp, "segment");
  EXPECT_NEAR(cp.s_star, 1.0, 1e-15);
  EXPECT_NEAR(cp.D, std::hypot(0.5, 0.2), 1e-15);
  EXPECT_EQ(cp.piece, vf::PieceKind::kSegment);
  // Backward and forward rays.
  cp = curve.closestPoint(Vec3(-1, 0.3, 0), context, none);
  expectFrame(cp, "backward ray");
  EXPECT_TRUE(cp.on_ray);
  EXPECT_NEAR(cp.s_star, -1.0, 1e-15);
  cp = curve.closestPoint(Vec3(2.1, 0, 4), context, none);
  expectFrame(cp, "forward ray");
  EXPECT_EQ(cp.piece, vf::PieceKind::kForwardRay);
  EXPECT_NEAR(cp.s_star, 6.0, 1e-15);
  // pointAt / tangentAt extrapolate along the rays.
  EXPECT_TRUE(curve.pointAt(-1.0, 0.0).isApprox(Vec3(-1, 0, 0)));
  EXPECT_TRUE(curve.pointAt(6.0, 0.0).isApprox(Vec3(2, 0, 4)));
  EXPECT_TRUE(curve.tangentAt(3.0, 0.0).isApprox(Vec3(0, 0, 1)));
  // On the curve: n = 0; at the vertex point T = unit(t_in + t_out).
  cp = curve.closestPoint(Vec3(2, 0, 0), context, none);
  EXPECT_EQ(cp.n, Vec3::Zero());
  EXPECT_TRUE(cp.T.isApprox(Vec3(1, 0, 1).normalized(), 1e-12));
}

TEST(D3Polyline, WedgeBlendIsOrthogonalAndContinuous)
{
  // A non-planar zigzag: every interior vertex has a wedge.
  const std::vector<Vec3> points{
    Vec3(0, 0, 0), Vec3(1, 0.3, 0.2), Vec3(1.5, 1.2, -0.4), Vec3(2.7, 1.0, 0.5), Vec3(3, 2, 1)};
  const d3::PolylineCurve3 curve(points, false);
  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> box(-1.5, 4.5);
  int wedges = 0;
  for (int i = 0; i < 20000; ++i) {
    const Vec3 x(box(rng), box(rng) * 0.8, box(rng) * 0.6);
    const auto cp = curve.closestPoint(x, d3::EvalContext3{}, d3::CurveHint3{});
    expectFrame(cp, "random");
    wedges += cp.in_wedge ? 1 : 0;
  }
  EXPECT_GT(wedges, 500);
  // Continuity at the wedge boundary a1 = 0: T tends to t_in.
  const Vec3 t_in = (points[2] - points[1]).normalized();
  const Vec3 t_out = (points[3] - points[2]).normalized();
  Vec3 side = t_in.cross(t_out).cross(t_in).normalized();  // ⊥ t_in, in the turning plane
  if (side.dot(t_out) > 0.0) {
    side = -side;  // the outer side of the corner
  }
  const Vec3 x = points[2] + 0.5 * side + 1e-9 * t_in;
  const auto cp = curve.closestPoint(x, d3::EvalContext3{}, d3::CurveHint3{});
  ASSERT_TRUE(cp.in_wedge);
  EXPECT_LT((cp.T - t_in).norm(), 1e-6);
}

TEST(D3Polyline, PlanarWedgeMatchesThe2DRule)
{
  const std::vector<vf::Vec2> planar{
    vf::Vec2(0, 0), vf::Vec2(1, 0), vf::Vec2(1.5, 1), vf::Vec2(0.5, 1.8), vf::Vec2(2.5, 2.2)};
  std::vector<Vec3> spatial;
  for (const auto & p : planar) {
    spatial.emplace_back(p.x(), p.y(), 0.0);
  }
  vf::PolylineCurve curve2(vf::SearchPolicy::windowScan(-100.0, 100.0));
  ASSERT_EQ(curve2.assign(vf::Span<const vf::Vec2>(planar), false), vf::AssignStatus::kOk);
  const d3::PolylineCurve3 curve3(spatial, false);
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> box(-1.0, 3.5);
  int wedges = 0;
  for (int i = 0; i < 5000; ++i) {
    const vf::Vec2 x(box(rng), box(rng));
    const auto a = curve2.closestPoint(x, vf::EvalContext{}, vf::CurveHint{});
    const auto b =
      curve3.closestPoint(Vec3(x.x(), x.y(), 0.0), d3::EvalContext3{}, d3::CurveHint3{});
    ASSERT_TRUE(a.valid && b.valid);
    if (std::abs(a.D - b.D) > 1e-9 || a.hairpin) {
      continue;  // ties resolved differently (equidistant feet)
    }
    EXPECT_NEAR(a.T.x(), b.T.x(), 1e-9);
    EXPECT_NEAR(a.T.y(), b.T.y(), 1e-9);
    EXPECT_NEAR(b.T.z(), 0.0, 1e-12);
    wedges += b.in_wedge ? 1 : 0;
  }
  EXPECT_GT(wedges, 100);
}

TEST(D3Polyline, HairpinFallbackChain)
{
  const d3::PolylineCurve3 curve({Vec3(0, 0, 0), Vec3(2, 0, 0), Vec3(0, 0, 0.0005)}, false);
  d3::EvalContext3 context;
  context.last_tangent = Vec3(0, 1, 1);
  auto cp = curve.closestPoint(Vec3(2.5, 0.3, 0.2), context, d3::CurveHint3{});
  expectFrame(cp, "hairpin with T_last");
  EXPECT_TRUE(cp.hairpin);
  EXPECT_TRUE(cp.status_bits & vf::Status::kHairpin);
  // On the axis beyond the tip with T_last along n and t_in along n: the e_k fallback.
  context.last_tangent = Vec3(1, 0, 0);
  cp = curve.closestPoint(Vec3(2.5, 0, 0), context, d3::CurveHint3{});
  expectFrame(cp, "hairpin on the axis");
  EXPECT_TRUE(cp.hairpin);
}

TEST(D3Polyline, HintedSearchKeepsTheBranchAndRejectsBadInput)
{
  // A closed square loop at two heights: the hint keeps the local branch.
  const d3::PolylineCurve3 curve(
    {Vec3(0, 0, 0), Vec3(4, 0, 0), Vec3(4, 4, 0), Vec3(0, 4, 0)}, true);
  EXPECT_DOUBLE_EQ(curve.length(0.0), 16.0);
  auto cp = curve.closestPoint(Vec3(1, 0.2, 0), d3::EvalContext3{}, d3::CurveHint3{});
  for (int i = 0; i < 200; ++i) {
    const double s = 1.0 + 0.1 * i;
    const Vec3 x = curve.pointAt(s, 0.0) + Vec3(0, 0, 0.1);
    cp = curve.closestPoint(x, d3::EvalContext3{}, d3::makeHint(cp));
    expectFrame(cp, "loop");
    EXPECT_NEAR(curve.wrapStationDifference(cp.s_star - s, 0.0), 0.0, 1e-9);
  }
  EXPECT_FALSE(curve.closestPoint(Vec3(vf::kNaN, 0, 0), {}, {}).valid);
  EXPECT_THROW(d3::PolylineCurve3({Vec3(0, 0, 0)}, false), vf::CurveError);
  EXPECT_THROW(d3::PolylineCurve3({Vec3(0, 0, 0), Vec3(0, 0, 0.0005)}, false), vf::CurveError);
  EXPECT_THROW(d3::PolylineCurve3({Vec3(0, 0, 0), Vec3(vf::kInf, 0, 0)}, false), vf::CurveError);
  EXPECT_THROW(d3::PolylineCurve3({Vec3(0, 0, 0), Vec3(1, 0, 0)}, true), vf::CurveError);
}

TEST(D3Helix, AnalyticClosestPoint)
{
  const double a = 1.0;
  const double lead = 0.5;
  const double c = lead / (2.0 * vf::kPi);
  const auto curve =
    d3::makeCurve3(builtin("helix", {"radius", "lead", "turns"}, {a, lead, 3.0}), {});
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> u_dist(1.0, 17.0);
  std::uniform_real_distribution<double> offset(-0.3, 0.3);
  for (int i = 0; i < 2000; ++i) {
    const double u0 = u_dist(rng);
    const double rho = a + offset(rng);
    const double zeta = 0.3 * offset(rng);
    const Vec3 x(rho * std::cos(u0), rho * std::sin(u0), c * u0 + zeta);
    // Root of g(δ) = aρ sin δ + c²δ − cζ (math_spec §14.6, O3D-3).
    double delta = 0.0;
    for (int k = 0; k < 50; ++k) {
      delta -= (a * rho * std::sin(delta) + c * c * delta - c * zeta) /
               (a * rho * std::cos(delta) + c * c);
    }
    const double u = u0 + delta;
    const Vec3 foot(a * std::cos(u), a * std::sin(u), c * u);
    const auto cp = curve->closestPoint(x, d3::EvalContext3{}, d3::CurveHint3{});
    expectFrame(cp, "helix");
    EXPECT_LT((cp.x_star - foot).norm(), 1e-9) << "u0 = " << u0;
    EXPECT_NEAR(cp.u_star, u, 1e-9);
    EXPECT_NEAR(cp.s_star, u * std::sqrt(a * a + c * c), 1e-8);
  }
  EXPECT_FALSE(curve->isClosed());
  EXPECT_NEAR(curve->length(0.0), 6.0 * vf::kPi * std::sqrt(a * a + c * c), 1e-9);
}

TEST(D3Builtins, RegularAtDefaultsAndPosed)
{
  for (const vf::BuiltinShapeInfo & info : d3::builtinShapes3()) {
    SCOPED_TRACE(info.name);
    std::shared_ptr<const d3::Curve3> curve;
    ASSERT_NO_THROW(curve = d3::makeCurve3(builtin(info.name), {}));
    EXPECT_EQ(curve->isClosed(), std::string(info.name) != "helix");
    EXPECT_FALSE(curve->isTimeVarying());
    const double L = curve->length(0.0);
    EXPECT_GT(L, 1.0);
    ASSERT_NE(curve->spec(), nullptr);
    EXPECT_EQ(curve->spec()->builtin_shape, info.name);
    std::vector<Vec3> samples(400);
    const std::size_t count =
      curve->sampleStations(0.0, L, L / 300.0, 0.0, vf::Span<Vec3>(samples));
    EXPECT_GE(count, 300u);
    for (std::size_t k = 0; k + 1 < count; ++k) {
      EXPECT_LT((samples[k + 1] - samples[k]).norm(), 1.01 * L / 300.0);
    }
    // The points sampled at s are their own feet.
    for (std::size_t k = 0; k < count; k += 37) {
      const auto cp = curve->closestPoint(samples[k], d3::EvalContext3{}, d3::CurveHint3{});
      EXPECT_LT(cp.D, 1e-7);
    }
  }
  // Pose: an inclined circle centred at c lies in the rotated plane.
  const auto circle = d3::makeCurve3(
    builtin("circle", {"radius", "cx", "cz", "pitch", "roll"}, {1.5, 1.0, 2.0, 0.4, -0.7}), {});
  const d3::Mat3 R = d3::rotationZYX(0.0, 0.4, -0.7);
  const Vec3 normal = R.col(2);
  for (double s = 0.0; s < 9.0; s += 0.7) {
    const Vec3 p = circle->pointAt(s, 0.0) - Vec3(1, 0, 2);
    EXPECT_NEAR(p.norm(), 1.5, 1e-9);
    EXPECT_NEAR(p.dot(normal), 0.0, 1e-12);
  }
}

TEST(D3Builtins, ParameterRules)
{
  EXPECT_THROW(d3::makeCurve3(builtin("trefoil"), {}), vf::CurveError);
  EXPECT_THROW(d3::makeCurve3(builtin("circle", {"a"}, {1.0}), {}), vf::CurveError);
  EXPECT_THROW(d3::makeCurve3(builtin("circle", {"radius"}, {-1.0}), {}), vf::CurveError);
  EXPECT_THROW(d3::makeCurve3(builtin("helix", {"lead"}, {0.0}), {}), vf::CurveError);
  EXPECT_THROW(d3::makeCurve3(builtin("torus_knot", {"p", "q"}, {2.0, 4.0}), {}), vf::CurveError);
  EXPECT_THROW(d3::makeCurve3(builtin("torus_knot", {"p"}, {1.5}), {}), vf::CurveError);
  EXPECT_THROW(d3::makeCurve3(builtin("torus_knot", {"major_radius"}, {0.4}), {}), vf::CurveError);
  auto spec = builtin("circle");
  spec.s_max = 1.0;
  EXPECT_THROW(d3::makeCurve3(spec, {}), vf::CurveError);
  EXPECT_NO_THROW(d3::makeCurve3(builtin("torus_knot", {"p", "q"}, {3.0, 5.0}), {}));
}

TEST(D3Expression, CurveAndHardening)
{
  auto spec = expression("2*cos(s)", "sin(s)", "0.3*sin(2*s)", true);
  const auto curve = d3::makeCurve3(spec, {});
  EXPECT_TRUE(curve->isClosed());
  EXPECT_FALSE(curve->isTimeVarying());
  const auto cp = curve->closestPoint(Vec3(2.3, 0.1, 0.0), {}, {});
  expectFrame(cp, "expression");
  EXPECT_LT(cp.D, 0.31);

  const auto rejects = [](const d3::CurveSpec3 & s, vf::CurveError::Code code) {
    try {
      d3::makeCurve3(s, {});
      ADD_FAILURE() << "accepted " << s.x_expression << "; " << s.y_expression << "; "
                    << s.z_expression;
    } catch (const vf::CurveError & e) {
      EXPECT_EQ(e.code(), code) << e.what();
    }
  };
  const auto invalid = vf::CurveError::Code::kInvalidCurve;
  rejects(expression("cos(s)", "sin(s)", "s > 1 && s < 2", true), invalid);
  rejects(expression("cos(s)", "sin(s)", "s < 1 || s > 2", true), invalid);
  rejects(expression("cos(s)", "sin(s)", "rnd()", true), invalid);
  rejects(expression("cos(s)", "sin(s)", "u", true), invalid);
  rejects(expression("cos(s)", "sin(s)", "", true), invalid);
  rejects(expression("cos(s)", "sin(s)", "1, 2", true), invalid);
  rejects(expression("cos(s)", "sin(s)", std::string(1025, '1'), true), invalid);
  rejects(expression("cos(s)", "sin(s)", "0", true, 3.0), invalid);  // does not close
  rejects(expression("s", "0", "0", true), invalid);                 // does not close
  rejects(expression("0", "0", "0", false), invalid);                // irregular
  rejects(expression("cos(s)", "sin(s)", "t", false), vf::CurveError::Code::kUnsupported);
  auto with_param = expression("k*cos(s)", "sin(s)", "0", true);
  with_param.parameter_names = {"k"};
  with_param.parameter_values = {2.0};
  EXPECT_NO_THROW(d3::makeCurve3(with_param, {}));
  with_param.z_expression = "k = 3";
  rejects(with_param, invalid);
  with_param.z_expression = "0";
  with_param.parameter_names = {"sin"};
  rejects(with_param, invalid);
  // A closed moving curve is accepted and reports dr/dt.
  const auto moving = d3::makeCurve3(expression("cos(s) + 0.2*t", "sin(s)", "0.1*t", true), {});
  EXPECT_TRUE(moving->isTimeVarying());
  const auto cpm = moving->closestPoint(Vec3(1.5, 0, 0), d3::EvalContext3{1.0, 1, {}}, {});
  EXPECT_LT((moving->velocity(cpm, 1.0) - Vec3(0.2, 0.0, 0.1)).norm(), 1e-9);
}

}  // namespace
