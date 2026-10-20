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

// Gains and the path field: GainFunction, TerminalProfile, DistanceField (static, time-varying
// and terminal), FieldSample and streamlines (gains.hpp, field.hpp).

#include <cmath>
#include <cstddef>
#include <vector>

#include "bindings.hpp"

namespace vectorfield_py
{

namespace
{

using vf::GainFunction;

// A scalar method and its elementwise array overload.
template <typename Class, typename F>
void defElementwise(Class & cls, const char * name, F f, const char * arg, const char * doc)
{
  using T = typename Class::type;
  cls.def(name, [f](const T & self, double x) { return f(self, x); }, py::arg(arg), doc);
  cls.def(
    name,
    [f](const T & self, const DoubleArray & x) {
      return mapValues(x, [&self, &f](double v) { return f(self, v); });
    },
    py::arg(arg));
}

void bindGain(py::module_ & m)
{
  py::class_<GainFunction> cls(
    m, "GainFunction",
    "A G-type gain: ATAN G = (2/pi) atan(k D) or RATIONAL G = k D / sqrt(1 + (k D)^2), with "
    "H = sqrt(1 - G^2). Methods accept scalars or arrays.");
  cls
    .def(
      py::init([](vf::GainType type, double k) { return GainFunction(type, k); }),
      py::arg("type") = vf::GainType::kAtan, py::arg("k") = 2.0,
      "Raises ConfigError unless k is finite and positive.")
    .def_static(
      "from_config", &GainFunction::fromConfig, py::arg("field"), "The gain of a FieldConfig.")
    .def_property_readonly("type", &GainFunction::type, "GainType.")
    .def_property_readonly("k", &GainFunction::k, "Convergence gain k_f (1/m).")
    .def("dG0", &GainFunction::dG0, "G'(0).")
    .def("cG", &GainFunction::cG, "The constant c_G (0.18557 atan, 0.38490 rational).")
    .def(
      "max_integral_curvature", &GainFunction::maxIntegralCurvature,
      "kappa_F = c_G k_f, the maximum integral-curve curvature near a straight path.")
    .def("__repr__", [](const GainFunction & g) {
      return "GainFunction(type=" + std::string(vf::toString(g.type())) +
             ", k=" + py::repr(py::float_(g.k())).cast<std::string>() + ")";
    });
  defElementwise(cls, "G", [](const GainFunction & g, double D) { return g.G(D); }, "D", "G(D).");
  defElementwise(cls, "H", [](const GainFunction & g, double D) { return g.H(D); }, "D", "H(D).");
  defElementwise(
    cls, "dG", [](const GainFunction & g, double D) { return g.dG(D); }, "D", "G'(D) (1/m).");
  defElementwise(
    cls, "Ginv", [](const GainFunction & g, double y) { return g.Ginv(y); }, "y",
    "G^-1(y); inf for y >= 1.");
  defElementwise(
    cls, "integral_curve_curvature",
    [](const GainFunction & g, double D) { return g.integralCurveCurvature(D); }, "D",
    "G G' / H: the integral-curve curvature at height D above a straight path.");
}

void bindTerminal(py::module_ & m)
{
  py::class_<vf::TerminalProfile> cls(
    m, "TerminalProfile",
    "Terminal speed profile phi(sigma) = sgn(sigma) min(1, max(|sigma|/sigma_b, (v_min/v_r) "
    "min(1, |sigma|/sigma_stop))) (math_spec section 5.1).");
  cls
    .def(
      py::init<double, double, double, double>(), py::arg("v_r"), py::arg("v_min"),
      py::arg("sigma_b"), py::arg("sigma_stop"))
    .def_property_readonly("lipschitz", &vf::TerminalProfile::lipschitz, "L_phi.")
    .def_property_readonly("sigma_stop", &vf::TerminalProfile::sigmaStop, "sigma_stop.")
    .def_property_readonly("sigma_braking", &vf::TerminalProfile::sigmaBraking, "sigma_b.");
  defElementwise(
    cls, "__call__", [](const vf::TerminalProfile & p, double s) { return p(s); }, "sigma",
    "phi(sigma) (scalar or array).");

  m.def(
    "feedforward", &vf::feedforward, py::arg("T"), py::arg("v_c"),
    "Phi_T = v_c - (T . v_c) T (math_spec section 4.2).");
  m.def(
    "clamp_feedforward",
    [](const vf::Vec2 & Phi_T, double v_r, double margin) {
      const vf::ClampedFeedforward r = vf::clampFeedforward(Phi_T, v_r, margin);
      return py::make_tuple(r.Phi_T, r.clamped);
    },
    py::arg("Phi_T"), py::arg("v_r"), py::arg("assumption3_margin"),
    "Assumption-3 clamp: (Phi_T, clamped).");
  m.def(
    "solve_eta", &vf::solveEta, py::arg("Phi_S"), py::arg("Phi_T"), py::arg("speed"),
    "eta, the larger root of eta^2 + 2 b eta + |Phi_T|^2 - v^2 = 0; NaN when infeasible.");
}

void bindSamples(py::module_ & m)
{
  using vf::ContourSample;
  StructBinder<ContourSample>(m, "ContourSample", "Obstacle-contour quantities of one cycle.")
    .field("gate_open", &ContourSample::gate_open, "The path_blocked gate is open.")
    .field("active", &ContourSample::active, "A contour episode is active.")
    .field("sigma_c", &ContourSample::sigma_c, "Circulation side -1, 0 or +1.")
    .field("goal_veto", &ContourSample::goal_veto, "The goal veto applies.")
    .field("x_o", &ContourSample::x_o, "Nearest obstacle point.")
    .field("D_o", &ContourSample::D_o, "Obstacle distance (m); inf without obstacle.")
    .field("n_o", &ContourSample::n_o, "Obstacle normal.")
    .field("theta", &ContourSample::theta, "Blend weight (1 = path field only).")
    .field("travelled_m", &ContourSample::travelled_m, "Episode path length (m).")
    .field("elapsed_s", &ContourSample::elapsed_s, "Episode time (s).")
    .field(
      "first_blocked_sample", &ContourSample::first_blocked_sample, "First blocked gate sample.")
    .field("has_blocked_sample", &ContourSample::has_blocked_sample, "A gate sample is blocked.")
    .finish();

  using vf::CbfSample;
  StructBinder<CbfSample>(
    m, "CbfSample", "CBF-filter quantities of one evaluation (math_spec section 6.7).")
    .field("on", &CbfSample::on, "The filter ran (obstacle_mode cbf, a map, XY not reached).")
    .field("active", &CbfSample::active, "The filter changed the field.")
    .field("hold", &CbfSample::hold, "Active with ||F*|| < 0.05 ||F|| (kBlocked when tracking).")
    .field("relaxed", &CbfSample::relaxed, "Conflicting push-outs were dropped.")
    .field("count", &CbfSample::count, "Constraints used (0-3).")
    .field("delta", &CbfSample::delta, "delta_safe = r_e + cbf_margin (m).")
    .field("h", &CbfSample::h, "Smallest barrier value h_j (m); inf without constraints.")
    .field("speed_ratio", &CbfSample::speed_ratio, "||F*|| / ||F||.")
    .field("F_in", &CbfSample::F_in, "The field before the filter.")
    .field("x_o", &CbfSample::x_o, "Nearest point of each site (first `count` entries).")
    .field("n", &CbfSample::n, "Unit normal of each constraint.")
    .field("h_j", &CbfSample::h_j, "Barrier value of each constraint (m).")
    .field("binds", &CbfSample::binds, "Whether each constraint binds.")
    .finish();

  using vf::FieldSample;
  StructBinder<FieldSample>(
    m, "FieldSample",
    "Every field quantity of one evaluation (math_spec Appendix A), in the evaluation frame.")
    .field("x_eval", &FieldSample::x_eval, "Evaluation point of the active law.")
    .field("x_star", &FieldSample::x_star, "Closest point.")
    .field("T", &FieldSample::T, "Unit tangent (parameter direction).")
    .field("T_d", &FieldSample::T_d, "Field tangent dir T.")
    .field("n", &FieldSample::n, "Unit normal (zero on the curve).")
    .field("D", &FieldSample::D, "Distance (m).")
    .field("s_star", &FieldSample::s_star, "Arc-length station.")
    .field("sigma", &FieldSample::sigma, "Remaining arc length to the goal (NaN closed).")
    .field("sigma_end", &FieldSample::sigma_end, "Remaining arc length to the curve end.")
    .field("G", &FieldSample::G, "G(D).")
    .field("H", &FieldSample::H, "H(D).")
    .field("eta", &FieldSample::eta, "Scale used (gamma v_r, or eta for time-varying fields).")
    .field("phi", &FieldSample::phi, "Terminal profile phi(sigma); 1 outside it.")
    .field("gamma", &FieldSample::gamma, "Speed-shaping scale.")
    .field("uniform_shaping", &FieldSample::uniform_shaping, "Uniform shaping branch taken.")
    .field("Phi_S", &FieldSample::Phi_S, "Unit field -G n + H T_d.")
    .field("Phi_T", &FieldSample::Phi_T, "Feedforward.")
    .field("v_c", &FieldSample::v_c, "Curve-point velocity dr/dt.")
    .field("Phi", &FieldSample::Phi, "Nominal path field eta Phi_S + Phi_T.")
    .field("Phi_gamma", &FieldSample::Phi_gamma, "Shaped path field.")
    .field("F", &FieldSample::F, "Field the law tracks (post-contour).")
    .field("conv", &FieldSample::conv, "Convergence arrow -eta G n.")
    .field("circ", &FieldSample::circ, "Circulation arrow eta H T_d.")
    .field("has_goal", &FieldSample::has_goal, "A goal is known.")
    .field("terminal", &FieldSample::terminal, "The terminal field is in use.")
    .field("g", &FieldSample::g, "Goal.")
    .field("g_ref", &FieldSample::g_ref, "Shifted goal.")
    .field("s_goal", &FieldSample::s_goal, "Goal station.")
    .field("x_ref", &FieldSample::x_ref, "Reference-point offset of the active law.")
    .field("contour", &FieldSample::contour, "ContourSample.")
    .field("cbf", &FieldSample::cbf, "CbfSample.")
    .field("psi_d", &FieldSample::psi_d, "Desired heading (heading tracking).")
    .field("psi_d_dot", &FieldSample::psi_d_dot, "Feedforward psi_d rate.")
    .field("heading_error", &FieldSample::heading_error, "Heading error of the active law.")
    .field("target_heading", &FieldSample::target_heading, "Phase target heading.")
    .field("has_target_heading", &FieldSample::has_target_heading, "target_heading is set.")
    .field("status", &FieldSample::status, "Status bits raised while evaluating.")
    .field("valid", &FieldSample::valid, "The closest point was valid.")
    .finish();
}

void bindDistanceField(py::module_ & m)
{
  using vf::DistanceField;
  using vf::FieldSample;
  py::class_<DistanceField>(
    m, "DistanceField",
    "The path field Phi = eta Phi_S + Phi_T of math_spec sections 4-5 (static: v_r Phi_S, so "
    "|Phi| = v_r), and the terminal field of an open curve.")
    .def(
      py::init<const GainFunction &, double, int, double>(), py::arg("gain"), py::arg("v_r"),
      py::arg("direction") = 1, py::arg("assumption3_margin") = 0.1)
    .def_static(
      "from_config",
      [](const vf::CoreConfig & c) {
        return DistanceField(
          GainFunction::fromConfig(c.field), c.field.desired_speed, c.field.direction,
          c.field.assumption3_margin);
      },
      py::arg("config"), "The field of a CoreConfig's Field group.")
    .def(
      "evaluate",
      [](
        const DistanceField & f, const vf::Vec2 & x, const vf::ClosestPoint & cp,
        const vf::Vec2 & v_c) {
        FieldSample s;
        f.evaluate(x, cp, v_c, &s);
        return s;
      },
      py::arg("x_eval"), py::arg("closest_point"), py::arg("v_c") = vf::Vec2(vf::Vec2::Zero()),
      "Static or time-varying field at a closest point (evaluation frame).")
    .def(
      "evaluate_terminal",
      [](
        const DistanceField & f, const vf::Vec2 & x, const vf::ClosestPoint & cp, double sigma,
        const vf::TerminalProfile & profile) {
        FieldSample s;
        f.evaluateTerminal(x, cp, sigma, profile, &s);
        return s;
      },
      py::arg("x_eval"), py::arg("closest_point"), py::arg("sigma"), py::arg("profile"),
      "Terminal field v_r (-G n + H phi(sigma) T_d) of a static open curve.")
    .def(
      "shape",
      [](
        const DistanceField & f, const FieldSample & sample, double gamma, double gamma_lim,
        bool force_uniform) {
        FieldSample s = sample;
        f.shape(gamma, gamma_lim, force_uniform, &s);
        return s;
      },
      py::arg("sample"), py::arg("gamma"), py::arg("gamma_lim") = 1.0,
      py::arg("force_uniform") = false, "A speed-shaped copy of an evaluated sample.")
    .def(
      "sample",
      [](const DistanceField & f, const vf::Curve & curve, const vf::Vec2 & x, double t) {
        vf::EvalContext context;
        context.t = t;
        context.direction = f.direction();
        const vf::ClosestPoint cp = curve.closestPoint(x, context, vf::CurveHint{});
        FieldSample s;
        f.evaluate(x, cp, curve.velocity(cp, t), &s);
        return s;
      },
      py::arg("curve"), py::arg("x"), py::arg("t") = 0.0,
      "Closest point (global search) and field at x, with the curve frame as evaluation frame.")
    .def(
      "field_vectors",
      [](const DistanceField & f, const vf::Curve & curve, const DoubleArray & points, double t) {
        std::vector<vf::Vec2> xs = toPoints(points, "points");
        {
          const py::gil_scoped_release release;
          vf::EvalContext context;
          context.t = t;
          context.direction = f.direction();
          FieldSample s;
          for (vf::Vec2 & x : xs) {
            const vf::ClosestPoint cp = curve.closestPoint(x, context, vf::CurveHint{});
            f.evaluate(x, cp, curve.velocity(cp, t), &s);
            x = s.valid ? s.Phi : vf::Vec2(vf::kNaN, vf::kNaN);
          }
        }
        return fromPoints(xs);
      },
      py::arg("curve"), py::arg("points"), py::arg("t") = 0.0,
      "Nominal field Phi (N, 2) at each row of an (N, 2) array (independent global searches; "
      "NaN where invalid). Releases the GIL.")
    .def_property_readonly("gain", &DistanceField::gain, "GainFunction.")
    .def_property_readonly("reference_speed", &DistanceField::referenceSpeed, "v_r (m/s).")
    .def_property_readonly("direction", &DistanceField::direction, "dir.")
    .def_property_readonly("assumption3_margin", &DistanceField::assumption3Margin, "epsilon_v.");

  m.def(
    "streamline",
    [](const py::function & field, const vf::Vec2 & start, double step, std::size_t max_points) {
      CallbackGuard guard;
      const auto f = [&field, &guard](const vf::Vec2 & x) {
        return guard.call(
          [&]() { return field(vf::Vec2(x)).cast<vf::Vec2>(); }, vf::Vec2(vf::kNaN, vf::kNaN));
      };
      std::vector<vf::Vec2> out(max_points);
      out.resize(vf::streamline(f, start, step, out));
      guard.rethrow();
      return fromPoints(out);
    },
    py::arg("field"), py::arg("start"), py::arg("step"), py::arg("max_points") = 1000,
    "RK4 streamline of dx/dl = F(x)/|F(x)| for a callable field(x) -> (2,), from start with "
    "arc-length step `step`; stops at max_points or where |F| is zero or not finite.");
}

}  // namespace

void bindField(py::module_ & m)
{
  bindGain(m);
  bindTerminal(m);
  bindSamples(m);
  bindDistanceField(m);
}

}  // namespace vectorfield_py
