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

// The experimental 3D core as `vectorfield.d3` (curves, field, Tracker, sim), NumPy in and out.
// The C++ names drop their `3` suffix here: vectorfield_core::d3::Tracker3 is d3.Tracker.

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "bindings.hpp"
#include "vectorfield_core/d3.hpp"

namespace vectorfield_py
{

namespace
{

namespace d3 = vectorfield_core::d3;
using d3::Vec3;

std::vector<Vec3> toPoints3(const DoubleArray & array, const char * what)
{
  if (array.size() == 0) {
    return {};
  }
  if (array.ndim() != 2 || array.shape(1) != 3) {
    throw py::value_error(std::string(what) + " must be an (N, 3) array of points");
  }
  const auto view = array.unchecked<2>();
  std::vector<Vec3> points(static_cast<std::size_t>(view.shape(0)));
  for (py::ssize_t i = 0; i < view.shape(0); ++i) {
    points[static_cast<std::size_t>(i)] = Vec3(view(i, 0), view(i, 1), view(i, 2));
  }
  return points;
}

py::array_t<double> fromPoints3(const std::vector<Vec3> & points)
{
  const std::vector<py::ssize_t> shape{static_cast<py::ssize_t>(points.size()), 3};
  py::array_t<double> out(shape);
  auto view = out.mutable_unchecked<2>();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto row = static_cast<py::ssize_t>(i);
    for (py::ssize_t j = 0; j < 3; ++j) {
      view(row, j) = points[i][j];
    }
  }
  return out;
}

Vec3 toVec3(const DoubleArray & array, const char * what)
{
  if (array.ndim() != 1 || array.shape(0) != 3) {
    throw py::value_error(std::string(what) + " must be a point of shape (3,)");
  }
  return Vec3(array.at(0), array.at(1), array.at(2));
}

// Applies f to one point (3,) or to every row of (N, 3).
py::array_t<double> mapPoints3(
  const DoubleArray & points, const std::function<Vec3(const Vec3 &)> & f)
{
  if (points.ndim() == 1) {
    const Vec3 r = f(toVec3(points, "point"));
    py::array_t<double> out(3);
    for (py::ssize_t j = 0; j < 3; ++j) {
      out.mutable_at(j) = r[j];
    }
    return out;
  }
  std::vector<Vec3> pts = toPoints3(points, "points");
  for (Vec3 & p : pts) {
    p = f(p);
  }
  return fromPoints3(pts);
}

void setParameters(d3::CurveSpec3 & spec, const py::dict & parameters)
{
  spec.parameter_names.clear();
  spec.parameter_values.clear();
  for (const auto & item : parameters) {
    spec.parameter_names.push_back(item.first.cast<std::string>());
    spec.parameter_values.push_back(item.second.cast<double>());
  }
}

struct Curve3Box
{
  std::shared_ptr<const d3::Curve3> curve;
};

struct Simulator3Box
{
  d3::sim::ClosedLoopSimulator3 simulator;
};

py::dict recordsToDict(const std::vector<d3::sim::SimulationRecord3> & records)
{
  const auto n = static_cast<py::ssize_t>(records.size());
  py::array_t<double> t(n);
  py::array_t<double> position(std::vector<py::ssize_t>{n, 3});
  py::array_t<double> yaw(n);
  py::array_t<double> v(std::vector<py::ssize_t>{n, 3});
  py::array_t<double> wz(n);
  py::array_t<double> D(n);
  py::array_t<double> sigma(n);
  py::array_t<double> s_star(n);
  py::array_t<double> F(std::vector<py::ssize_t>{n, 3});
  py::array_t<std::int32_t> phase(n);
  py::array_t<std::uint32_t> status(n);
  py::array_t<bool> settled(n);
  auto p = position.mutable_unchecked<2>();
  auto vv = v.mutable_unchecked<2>();
  auto ff = F.mutable_unchecked<2>();
  for (py::ssize_t i = 0; i < n; ++i) {
    const auto & r = records[static_cast<std::size_t>(i)];
    t.mutable_at(i) = r.t;
    yaw.mutable_at(i) = r.pose.yaw;
    wz.mutable_at(i) = r.cmd.wz;
    D.mutable_at(i) = r.D;
    sigma.mutable_at(i) = r.sigma;
    s_star.mutable_at(i) = r.s_star;
    phase.mutable_at(i) = static_cast<std::int32_t>(r.phase);
    status.mutable_at(i) = r.status;
    settled.mutable_at(i) = r.settled;
    for (py::ssize_t j = 0; j < 3; ++j) {
      p(i, j) = r.pose.p[j];
      vv(i, j) = r.cmd.v[j];
      ff(i, j) = r.F[j];
    }
  }
  py::dict out;
  out["t"] = t;
  out["position"] = position;
  out["yaw"] = yaw;
  out["v"] = v;
  out["wz"] = wz;
  out["D"] = D;
  out["sigma"] = sigma;
  out["s_star"] = s_star;
  out["F"] = F;
  out["phase"] = phase;
  out["status"] = status;
  out["settled"] = settled;
  return out;
}

void bindCurves3(py::module_ & m)
{
  py::class_<d3::CurveSpec3> spec_class(
    m, "CurveSpec",
    "A 3D curve description (vectorfield_core::d3::CurveSpec3): POLYLINE points (N, 3), "
    "EXPRESSION x/y/z_expression over [s_min, s_max], or a BUILTIN shape (builtin_shapes()).");
  spec_class.def(py::init<>())
    .def_readwrite("type", &d3::CurveSpec3::type, "vectorfield.CurveSpec.Type.")
    .def_readwrite("closed", &d3::CurveSpec3::closed, "Closed curve (POLYLINE, EXPRESSION).")
    .def_property(
      "points", [](const d3::CurveSpec3 & s) { return fromPoints3(s.points); },
      [](d3::CurveSpec3 & s, const DoubleArray & a) { s.points = toPoints3(a, "points"); },
      "POLYLINE vertices, (N, 3) (a copy).")
    .def_readwrite("x_expression", &d3::CurveSpec3::x_expression, "x(s, t).")
    .def_readwrite("y_expression", &d3::CurveSpec3::y_expression, "y(s, t).")
    .def_readwrite("z_expression", &d3::CurveSpec3::z_expression, "z(s, t).")
    .def_readwrite("s_min", &d3::CurveSpec3::s_min, "EXPRESSION domain start.")
    .def_readwrite("s_max", &d3::CurveSpec3::s_max, "EXPRESSION domain end.")
    .def_readwrite("builtin_shape", &d3::CurveSpec3::builtin_shape, "BUILTIN shape name.")
    .def_readwrite("t0", &d3::CurveSpec3::t0, "Time origin of r(s, t) (s).")
    .def_property(
      "parameters",
      [](const d3::CurveSpec3 & s) {
        py::dict out;
        for (std::size_t i = 0; i < s.parameter_names.size(); ++i) {
          out[py::str(s.parameter_names[i])] = s.parameter_values[i];
        }
        return out;
      },
      &setParameters, "Parameters as a {name: value} dict (a copy).")
    .def_static(
      "polyline",
      [](const DoubleArray & points, bool closed) {
        d3::CurveSpec3 s;
        s.points = toPoints3(points, "points");
        s.closed = closed;
        return s;
      },
      py::arg("points"), py::arg("closed") = false, "A POLYLINE spec from (N, 3) points.")
    .def_static(
      "expression",
      [](
        const std::string & x, const std::string & y, const std::string & z, double s_min,
        double s_max, bool closed, const py::dict & parameters, double t0) {
        d3::CurveSpec3 s;
        s.type = d3::CurveSpec3::Type::kExpression;
        s.x_expression = x;
        s.y_expression = y;
        s.z_expression = z;
        s.s_min = s_min;
        s.s_max = s_max;
        s.closed = closed;
        s.t0 = t0;
        setParameters(s, parameters);
        return s;
      },
      py::arg("x"), py::arg("y"), py::arg("z"), py::arg("s_min") = 0.0,
      py::arg("s_max") = 2.0 * vf::kPi, py::arg("closed") = true,
      py::arg("parameters") = py::dict(), py::arg("t0") = 0.0,
      "An EXPRESSION spec: x(s, t), y(s, t), z(s, t).")
    .def_static(
      "builtin",
      [](const std::string & name, const py::kwargs & parameters) {
        d3::CurveSpec3 s;
        s.type = d3::CurveSpec3::Type::kBuiltin;
        s.builtin_shape = name;
        setParameters(s, parameters);
        return s;
      },
      py::arg("name"), "A BUILTIN spec; keyword arguments are the shape's parameters.")
    .def("__eq__", [](const d3::CurveSpec3 & a, const d3::CurveSpec3 & b) { return a == b; })
    .def("__repr__", [](const d3::CurveSpec3 & s) {
      return std::string("d3.CurveSpec(type=") + py::repr(py::cast(s.type)).cast<std::string>() +
             ", closed=" + (s.closed ? "True" : "False") + ")";
    });

  py::class_<d3::ClosestPoint3>(m, "ClosestPoint", "A 3D closest-point result.")
    .def_readonly("x_star", &d3::ClosestPoint3::x_star, "Foot (3,).")
    .def_readonly("T", &d3::ClosestPoint3::T, "Unit tangent (3,).")
    .def_readonly("n", &d3::ClosestPoint3::n, "Unit normal (3,), zero on the curve.")
    .def_readonly("D", &d3::ClosestPoint3::D, "Distance.")
    .def_readonly("s_star", &d3::ClosestPoint3::s_star, "Station of the foot.")
    .def_readonly("u_star", &d3::ClosestPoint3::u_star, "Curve parameter at the foot.")
    .def_readonly("on_ray", &d3::ClosestPoint3::on_ray, "The foot lies on a ray.")
    .def_readonly("in_wedge", &d3::ClosestPoint3::in_wedge, "A polyline vertex owns the point.")
    .def_readonly("hairpin", &d3::ClosestPoint3::hairpin, "The hairpin rule produced T.")
    .def_readonly("valid", &d3::ClosestPoint3::valid, "False for invalid input.")
    .def_readonly("status_bits", &d3::ClosestPoint3::status_bits, "Status bits.");

  py::class_<Curve3Box>(m, "Curve", "An immutable 3D curve built by make_curve().")
    .def_property_readonly(
      "is_closed", [](const Curve3Box & c) { return c.curve->isClosed(); }, "Closed curve.")
    .def_property_readonly(
      "is_time_varying", [](const Curve3Box & c) { return c.curve->isTimeVarying(); },
      "The shape depends on t.")
    .def_property_readonly(
      "spec",
      [](const Curve3Box & c) {
        return c.curve->spec() != nullptr ? py::cast(*c.curve->spec()) : py::none();
      },
      "The spec it was built from (a copy).")
    .def(
      "length", [](const Curve3Box & c, double t) { return c.curve->length(t); },
      py::arg("t") = 0.0, "Length L(t) (open: rays excluded).")
    .def(
      "point_at",
      [](const Curve3Box & c, const DoubleArray & s, double t) -> py::object {
        if (s.ndim() == 0) {
          return py::cast(Vec3(c.curve->pointAt(s.at(), t)));
        }
        std::vector<Vec3> out;
        for (py::ssize_t i = 0; i < s.size(); ++i) {
          out.push_back(c.curve->pointAt(s.data()[i], t));
        }
        return fromPoints3(out);
      },
      py::arg("s"), py::arg("t") = 0.0, "r(s, t) at station(s) s: (3,) or (N, 3).")
    .def(
      "tangent_at",
      [](const Curve3Box & c, double s, double t) { return c.curve->tangentAt(s, t); },
      py::arg("s"), py::arg("t") = 0.0, "Unit tangent at station s.")
    .def(
      "sample",
      [](const Curve3Box & c, double ds, double t) {
        const double L = c.curve->length(t);
        if (!(ds > 0.0)) {
          ds = std::min(0.05, L / 400.0);
        }
        const auto count = static_cast<std::size_t>(std::ceil(L / ds)) + 2;
        if (count > 10000000) {
          throw py::value_error("sample: ds is too small for this curve");
        }
        std::vector<Vec3> points(count);
        const std::size_t written = c.curve->sampleStations(0.0, L, ds, t, vf::Span<Vec3>(points));
        points.resize(written);
        if (c.curve->isClosed() && !points.empty()) {
          points.push_back(points.front());
        }
        return fromPoints3(points);
      },
      py::arg("ds") = 0.0, py::arg("t") = 0.0,
      "(N, 3) samples every ds metres (default: about 400 samples, at most 0.05 m); closed "
      "curves repeat the first point at the end.")
    .def(
      "closest_point",
      [](const Curve3Box & c, const DoubleArray & x, double t, int direction) {
        d3::EvalContext3 context;
        context.t = t;
        context.direction = direction;
        return c.curve->closestPoint(toVec3(x, "x"), context, d3::CurveHint3{});
      },
      py::arg("x"), py::arg("t") = 0.0, py::arg("direction") = 1,
      "Closest point of x (global search, no hint).")
    .def(
      "distance",
      [](const Curve3Box & c, const DoubleArray & points, double t) {
        const std::vector<Vec3> pts = toPoints3(points, "points");
        py::array_t<double> out(static_cast<py::ssize_t>(pts.size()));
        d3::EvalContext3 context;
        context.t = t;
        for (std::size_t i = 0; i < pts.size(); ++i) {
          out.mutable_at(static_cast<py::ssize_t>(i)) =
            c.curve->closestPoint(pts[i], context, d3::CurveHint3{}).D;
        }
        return out;
      },
      py::arg("points"), py::arg("t") = 0.0, "Distances D of (N, 3) points.");

  m.def(
    "make_curve",
    [](const d3::CurveSpec3 & spec, const vf::CurveOptions & options) {
      return Curve3Box{d3::makeCurve3(spec, options)};
    },
    py::arg("spec"), py::arg("options") = vf::CurveOptions{},
    "Builds and validates a 3D curve (raises CurveError).");
  m.def(
    "builtin_shapes",
    []() {
      py::list shapes;
      for (const vf::BuiltinShapeInfo & info : d3::builtinShapes3()) {
        py::dict parameters;
        for (const vf::BuiltinParameterInfo & p : info.parameters) {
          parameters[py::str(p.name)] = p.default_value;
        }
        py::dict shape;
        shape["name"] = py::str(info.name);
        shape["parameters"] = parameters;
        shape["formula"] = py::str(info.formula);
        shapes.append(shape);
      }
      return shapes;
    },
    "The 3D built-in shapes as dicts: name, parameters (defaults), formula.");
}

void bindField3(py::module_ & m)
{
  py::class_<d3::DistanceField3>(m, "DistanceField", "The 3D field Phi = eta Phi_S + Phi_T.")
    .def(
      py::init<const vf::GainFunction &, double, int, double>(),
      py::arg("gain") = vf::GainFunction(), py::arg("v_r") = 0.5, py::arg("direction") = 1,
      py::arg("assumption3_margin") = 0.1)
    .def(
      "__call__",
      [](const d3::DistanceField3 & f, const Curve3Box & c, const DoubleArray & x, double t) {
        return mapPoints3(x, [&](const Vec3 & p) { return f.at(*c.curve, p, t); });
      },
      py::arg("curve"), py::arg("x"), py::arg("t") = 0.0,
      "Phi at a point (3,) or at (N, 3) points (global closest point per point).")
    .def(
      "streamline",
      [](
        const d3::DistanceField3 & f, const Curve3Box & c, const DoubleArray & start, double step,
        int count, double t) {
        if (count < 1) {
          throw py::value_error("count must be >= 1");
        }
        std::vector<Vec3> out(static_cast<std::size_t>(count));
        const auto evaluate = [&](const Vec3 & p) { return f.at(*c.curve, p, t); };
        const std::size_t n =
          d3::streamline(evaluate, toVec3(start, "start"), step, vf::Span<Vec3>(out));
        out.resize(n);
        return fromPoints3(out);
      },
      py::arg("curve"), py::arg("start"), py::arg("step") = 0.05, py::arg("count") = 200,
      py::arg("t") = 0.0, "RK4 streamline of the normalised field, (M, 3).")
    .def_property_readonly("v_r", &d3::DistanceField3::referenceSpeed, "Reference speed.")
    .def_property_readonly("direction", &d3::DistanceField3::direction, "Direction.");
}

void bindTracker3(py::module_ & m, py::module_ & sim)
{
  py::class_<d3::Limits3>(m, "Limits", "Velocity, acceleration and yaw limits.")
    .def(py::init<>())
    .def_readwrite("max_speed", &d3::Limits3::max_speed, "||v|| bound (m/s).")
    .def_property(
      "max_axis_speed", [](const d3::Limits3 & l) { return l.max_axis_speed; },
      [](d3::Limits3 & l, const DoubleArray & value) {
        // Converted by hand: GCC 13 reports a false -Wmaybe-uninitialized inside pybind11's
        // fixed-size Eigen caster when it loads a Vector3d.
        l.max_axis_speed = toVec3(value, "max_axis_speed");
      },
      "Per-axis bounds (3,).")
    .def_readwrite("max_accel", &d3::Limits3::max_accel, "a_acc (m/s^2).")
    .def_readwrite("max_decel", &d3::Limits3::max_decel, "a_dec (m/s^2).")
    .def_readwrite("cancel_decel", &d3::Limits3::cancel_decel, "a_can (m/s^2).")
    .def_readwrite("max_yaw_rate", &d3::Limits3::max_yaw_rate, "Yaw rate bound (rad/s).")
    .def_readwrite("max_yaw_accel", &d3::Limits3::max_yaw_accel, "Yaw acceleration (rad/s^2).");
  py::class_<d3::YawConfig3>(m, "YawConfig", "Yaw policy (path tangent, field, fixed).")
    .def(py::init<>())
    .def_readwrite("policy", &d3::YawConfig3::policy, "vectorfield.OmniHeadingPolicy.")
    .def_readwrite("gain", &d3::YawConfig3::gain, "k_psi (1/s).")
    .def_readwrite("fixed_yaw", &d3::YawConfig3::fixed_yaw, "psi_d for FIXED (rad).");
  py::class_<d3::Tracker3Config>(m, "TrackerConfig", "Tracker configuration (Tracker3Config).")
    .def(py::init<>())
    .def(
      py::init([](const py::kwargs & kwargs) {
        d3::Tracker3Config c;
        py::object self = py::cast(c);
        for (const auto & item : kwargs) {
          const auto key = item.first.cast<std::string>();
          if (!py::hasattr(self, key.c_str())) {
            throw py::type_error("unexpected keyword argument '" + key + "'");
          }
          self.attr(key.c_str()) = item.second;
        }
        return self.cast<d3::Tracker3Config>();
      }),
      "Defaults, then the given attributes (keyword arguments).")
    .def_readwrite("desired_speed", &d3::Tracker3Config::desired_speed, "v_r (m/s).")
    .def_readwrite("convergence_gain", &d3::Tracker3Config::convergence_gain, "k_f (1/m).")
    .def_readwrite("gain_function", &d3::Tracker3Config::gain_function, "GainType.")
    .def_readwrite("direction", &d3::Tracker3Config::direction, "+1 or -1.")
    .def_readwrite(
      "assumption3_margin", &d3::Tracker3Config::assumption3_margin, "eps_v in (0, 1).")
    .def_readwrite("goal_tolerance", &d3::Tracker3Config::goal_tolerance, "Goal radius (m).")
    .def_readwrite(
      "terminal_braking_distance", &d3::Tracker3Config::terminal_braking_distance, "sigma_b (m).")
    .def_readwrite("min_approach_speed", &d3::Tracker3Config::min_approach_speed, "v_min (m/s).")
    .def_readwrite(
      "final_approach_distance", &d3::Tracker3Config::final_approach_distance,
      "FINAL_APPROACH entry (m); negative = auto.")
    .def_readwrite(
      "control_frequency", &d3::Tracker3Config::control_frequency, "Control rate (Hz).")
    .def_readwrite("limits", &d3::Tracker3Config::limits, "Limits.")
    .def_readwrite("yaw", &d3::Tracker3Config::yaw, "YawConfig.");

  py::class_<d3::Tracker3State>(m, "TrackerState", "Cross-cycle state (copyable).")
    .def(py::init<>())
    .def_readonly("phase", &d3::Tracker3State::phase, "Phase.")
    .def_readonly("xy_latched", &d3::Tracker3State::xy_latched, "Goal latch.")
    .def_readonly("holding", &d3::Tracker3State::holding, "Goal hold.")
    .def("copy", [](const d3::Tracker3State & s) { return s; });

  py::class_<d3::Tracker3Output>(m, "TrackerOutput", "Outputs of one cycle.")
    .def_property_readonly(
      "v", [](const d3::Tracker3Output & o) { return o.cmd.v; }, "v (3,).")
    .def_property_readonly(
      "wz", [](const d3::Tracker3Output & o) { return o.cmd.wz; }, "Yaw rate.")
    .def_property_readonly(
      "Phi", [](const d3::Tracker3Output & o) { return o.field.Phi; }, "Field at the body.")
    .def_property_readonly(
      "D", [](const d3::Tracker3Output & o) { return o.field.D; }, "D.")
    .def_property_readonly(
      "sigma", [](const d3::Tracker3Output & o) { return o.field.sigma; }, "Terminal sigma.")
    .def_property_readonly(
      "x_star", [](const d3::Tracker3Output & o) { return o.field.x_star; }, "Foot (3,).")
    .def_readonly("phase", &d3::Tracker3Output::phase, "Phase.")
    .def_readonly("status", &d3::Tracker3Output::status, "Status bits.")
    .def_readonly("settled", &d3::Tracker3Output::settled, "Settled at the goal.")
    .def_readonly("cancel_complete", &d3::Tracker3Output::cancel_complete, "Cancel done.")
    .def_readonly("goal", &d3::Tracker3Output::goal, "Goal of an open curve (3,).")
    .def_readonly("has_goal", &d3::Tracker3Output::has_goal, "Open curve.")
    .def_readonly("psi_d", &d3::Tracker3Output::psi_d, "Desired yaw.");

  py::class_<d3::Tracker3>(m, "Tracker", "The minimal 3D tracker (experimental).")
    .def(py::init<const d3::Tracker3Config &>(), py::arg("config") = d3::Tracker3Config{})
    .def_property_readonly("config", &d3::Tracker3::config, "The configuration (a copy).")
    .def_static("initial_state", &d3::Tracker3::initialState, "A fresh TrackerState.")
    .def(
      "step",
      [](
        const d3::Tracker3 & tracker, const Curve3Box & curve, const DoubleArray & position,
        double yaw, const d3::Tracker3State & state, double t, double dt,
        const py::object & measured_v, double measured_wz, std::uint32_t goal_seq,
        double speed_limit_scale, bool cancel) {
        d3::Tracker3Input input;
        input.curve = curve.curve.get();
        input.pose.p = toVec3(position, "position");
        input.pose.yaw = yaw;
        input.t = t;
        input.dt = dt;
        input.measured.v = measured_v.is_none()
                             ? Vec3::Zero()
                             : toVec3(measured_v.cast<DoubleArray>(), "measured_v");
        input.measured.wz = measured_wz;
        input.goal_seq = goal_seq;
        input.speed_limit_scale = speed_limit_scale;
        input.cancel = cancel;
        d3::Tracker3State next;
        d3::Tracker3Output out = tracker.step(input, state, &next);
        return py::make_tuple(out, next);
      },
      py::arg("curve"), py::arg("position"), py::arg("yaw") = 0.0,
      py::arg("state") = d3::Tracker3State{}, py::arg("t") = 0.0, py::arg("dt") = 0.05,
      py::arg("measured_v") = py::none(), py::arg("measured_wz") = 0.0, py::arg("goal_seq") = 0U,
      py::arg("speed_limit_scale") = 1.0, py::arg("cancel") = false,
      "One control cycle; returns (TrackerOutput, next TrackerState).");

  sim.def(
    "integrate",
    [](
      const DoubleArray & position, double yaw, const DoubleArray & v, double wz,
      const DoubleArray & cmd_v, double cmd_wz, double dt, double velocity_time_constant) {
      d3::sim::PlantState3 state;
      state.pose.p = toVec3(position, "position");
      state.pose.yaw = yaw;
      state.twist.v = toVec3(v, "v");
      state.twist.wz = wz;
      d3::Twist3 cmd;
      cmd.v = toVec3(cmd_v, "cmd_v");
      cmd.wz = cmd_wz;
      d3::sim::PlantParams3 params;
      params.velocity_time_constant = velocity_time_constant;
      const auto next = d3::sim::integrate(state, cmd, dt, params);
      return py::make_tuple(next.pose.p, next.pose.yaw, next.twist.v, next.twist.wz);
    },
    py::arg("position"), py::arg("yaw"), py::arg("v"), py::arg("wz"), py::arg("cmd_v"),
    py::arg("cmd_wz"), py::arg("dt"), py::arg("velocity_time_constant") = 0.0,
    "Exact ZOH step of the point / first-order-lag plant; returns (position, yaw, v, wz).");

  py::class_<Simulator3Box>(
    sim, "ClosedLoopSimulator", "Closed loop of a d3.Tracker and the plant.")
    .def(
      py::init([](
                 const d3::Tracker3 & tracker, double period, double velocity_time_constant,
                 bool stop_when_settled, bool stop_when_stopped) {
        d3::sim::SimulationOptions3 options;
        options.period = period;
        options.plant.velocity_time_constant = velocity_time_constant;
        options.stop_when_settled = stop_when_settled;
        options.stop_when_stopped = stop_when_stopped;
        return Simulator3Box{d3::sim::ClosedLoopSimulator3(tracker, options)};
      }),
      py::arg("tracker"), py::arg("period") = 0.05, py::arg("velocity_time_constant") = 0.0,
      py::arg("stop_when_settled") = true, py::arg("stop_when_stopped") = true)
    .def(
      "reset",
      [](Simulator3Box & s, const DoubleArray & position, double yaw, double t0) {
        d3::Pose3 pose;
        pose.p = toVec3(position, "position");
        pose.yaw = yaw;
        s.simulator.reset(pose, t0);
      },
      py::arg("position"), py::arg("yaw") = 0.0, py::arg("t0") = 0.0,
      "Resets pose, time, tracker state and records.")
    .def(
      "run",
      [](
        Simulator3Box & s, const Curve3Box & curve, double duration, double speed_limit_scale,
        double cancel_at) {
        d3::Tracker3Input input;
        input.curve = curve.curve.get();
        input.speed_limit_scale = speed_limit_scale;
        {
          const py::gil_scoped_release release;
          const double t_cancel = s.simulator.time() + cancel_at;
          const bool cancels = std::isfinite(cancel_at);
          s.simulator.run(input, duration, [&](double t, d3::Tracker3Input * in) {
            in->cancel = cancels && t >= t_cancel - 1e-9;
          });
        }
        return recordsToDict(s.simulator.records());
      },
      py::arg("curve"), py::arg("duration"), py::arg("speed_limit_scale") = 1.0,
      py::arg("cancel_at") = vf::kInf,
      "Steps for `duration` seconds (or until settled / a cancel completes; cancel_at is "
      "relative to the current time) and returns every record since reset as a dict of "
      "arrays: t, position (N, 3), yaw, v (N, 3), wz, D, sigma, s_star, F (N, 3), phase, "
      "status, settled.")
    .def_property_readonly(
      "position", [](const Simulator3Box & s) { return s.simulator.pose().p; }, "Position (3,).")
    .def_property_readonly(
      "time", [](const Simulator3Box & s) { return s.simulator.time(); }, "Time (s).")
    .def_property_readonly(
      "state", [](const Simulator3Box & s) { return s.simulator.state(); }, "TrackerState.")
    .def_property_readonly(
      "last_output", [](const Simulator3Box & s) { return s.simulator.lastOutput(); },
      "TrackerOutput of the last step.");
}

}  // namespace

void bindD3(py::module_ & m)
{
  py::module_ d3m = m.def_submodule(
    "d3",
    "The experimental 3D core (vectorfield_core::d3): curves in R^3, the 3D field, a holonomic "
    "multirotor tracker and its simulator. Points are float64 (3,) or (N, 3). Experimental in "
    "0.x: the API may change in a minor release.");
  py::module_ sim3 = d3m.def_submodule("sim", "3D plant integration and closed-loop simulator.");
  bindCurves3(d3m);
  bindField3(d3m);
  bindTracker3(d3m, sim3);
}

}  // namespace vectorfield_py
