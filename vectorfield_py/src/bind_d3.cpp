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

#include <algorithm>
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

Vec3 optionalVec3(const py::object & value, const char * what)
{
  return value.is_none() ? Vec3::Zero() : toVec3(value.cast<DoubleArray>(), what);
}

d3::Isometry3 toIsometry3(const DoubleArray & array, const char * what)
{
  if (array.ndim() != 2 || array.shape(0) != 4 || array.shape(1) != 4) {
    throw py::value_error(std::string(what) + " must be a homogeneous transform of shape (4, 4)");
  }
  const auto view = array.unchecked<2>();
  Eigen::Matrix4d m;
  for (py::ssize_t i = 0; i < 4; ++i) {
    for (py::ssize_t j = 0; j < 4; ++j) {
      m(i, j) = view(i, j);
    }
  }
  if (m.row(3) != Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)) {
    throw py::value_error(std::string(what) + ": the last row must be (0, 0, 0, 1)");
  }
  d3::Isometry3 T = d3::Isometry3::Identity();
  T.matrix() = m;
  return T;
}

Eigen::Matrix4d fromIsometry3(const d3::Isometry3 & T) { return T.matrix(); }

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

// A Python frame_motion(t) -> CurveFrame as a core FrameMotion3 that takes the GIL.
d3::sim::FrameMotion3 frameMotion(const py::object & callable)
{
  if (callable.is_none()) {
    return nullptr;
  }
  // The Python object is released with the GIL held, whichever thread drops the function.
  const std::shared_ptr<py::object> fn(new py::object(callable), [](py::object * o) {
    const py::gil_scoped_acquire gil;
    delete o;
  });
  return [fn](double t) {
    const py::gil_scoped_acquire gil;
    return (*fn)(t).cast<d3::CurveFrame3>();
  };
}

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
  py::array_t<double> frame(std::vector<py::ssize_t>{n, 4, 4});
  py::array_t<double> h(n);
  py::array_t<bool> cbf_active(n);
  auto fr = frame.mutable_unchecked<3>();
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
    h.mutable_at(i) = r.h;
    cbf_active.mutable_at(i) = r.cbf_active;
    for (py::ssize_t j = 0; j < 3; ++j) {
      p(i, j) = r.pose.p[j];
      vv(i, j) = r.cmd.v[j];
      ff(i, j) = r.F[j];
    }
    for (py::ssize_t j = 0; j < 4; ++j) {
      for (py::ssize_t k = 0; k < 4; ++k) {
        fr(i, j, k) = r.T_eval_curve.matrix()(j, k);
      }
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
  out["frame"] = frame;
  out["h"] = h;
  out["cbf_active"] = cbf_active;
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

void bindFrames3(py::module_ & m)
{
  py::class_<d3::CurveFrame3>(
    m, "CurveFrame",
    "The frame a curve lives in (CurveFrame3): T_eval_curve (4, 4) maps curve-frame points to "
    "the evaluation frame; the body twist v, w (3,) is expressed in the curve frame; moving "
    "enables the frame terms (math_spec 14.7).")
    .def(
      py::init([](
                 const py::object & T_eval_curve, const py::object & v, const py::object & w,
                 const py::object & moving) {
        d3::CurveFrame3 frame;
        if (!T_eval_curve.is_none()) {
          frame.T_eval_curve = toIsometry3(T_eval_curve.cast<DoubleArray>(), "T_eval_curve");
        }
        frame.twist.v = optionalVec3(v, "v");
        frame.twist.w = optionalVec3(w, "w");
        frame.moving = moving.is_none()
                         ? (frame.twist.v.squaredNorm() > 0.0 || frame.twist.w.squaredNorm() > 0.0)
                         : moving.cast<bool>();
        return frame;
      }),
      py::arg("T_eval_curve") = py::none(), py::arg("v") = py::none(), py::arg("w") = py::none(),
      py::arg("moving") = py::none(),
      "Identity and static by default; `moving` defaults to a non-zero twist.")
    .def_property(
      "T_eval_curve", [](const d3::CurveFrame3 & f) { return fromIsometry3(f.T_eval_curve); },
      [](d3::CurveFrame3 & f, const DoubleArray & T) {
        f.T_eval_curve = toIsometry3(T, "T_eval_curve");
      },
      "Curve frame to evaluation frame (4, 4).")
    .def_property(
      "v", [](const d3::CurveFrame3 & f) { return f.twist.v; },
      [](d3::CurveFrame3 & f, const DoubleArray & v) { f.twist.v = toVec3(v, "v"); },
      "Linear velocity of the frame, in the frame (3,).")
    .def_property(
      "w", [](const d3::CurveFrame3 & f) { return f.twist.w; },
      [](d3::CurveFrame3 & f, const DoubleArray & w) { f.twist.w = toVec3(w, "w"); },
      "Angular velocity of the frame, in the frame (3,).")
    .def_readwrite("moving", &d3::CurveFrame3::moving, "The frame moves.")
    .def(
      "at",
      [](const d3::CurveFrame3 & f, double tau) {
        d3::CurveFrame3 out = f;
        out.T_eval_curve = d3::extrapolate(f, tau);
        return out;
      },
      py::arg("tau"), "The frame tau seconds later under its constant twist (exact).")
    .def(
      "velocity",
      [](const d3::CurveFrame3 & f, const DoubleArray & x_curve) {
        return mapPoints3(x_curve, [&](const Vec3 & x) { return d3::frameVelocity(f, x); });
      },
      py::arg("x_curve"),
      "Velocity R (v + w x x_F) of frame-fixed points x_curve (3,) or (N, 3), evaluation frame.")
    .def("__repr__", [](const d3::CurveFrame3 & f) {
      return "CurveFrame(moving=" + std::string(f.moving ? "True" : "False") + ")";
    });

  m.def(
    "exp_se3",
    [](const DoubleArray & v, const DoubleArray & w, double tau) {
      d3::FrameTwist3 xi;
      xi.v = toVec3(v, "v");
      xi.w = toVec3(w, "w");
      return fromIsometry3(d3::expSE3(xi, tau));
    },
    py::arg("v"), py::arg("w"), py::arg("tau") = 1.0,
    "exp(tau xi^) of the body twist (v, w) as a (4, 4) transform.");
  m.def(
    "log_se3",
    [](const DoubleArray & T) {
      const d3::FrameTwist3 xi = d3::logSE3(toIsometry3(T, "T"));
      return py::make_tuple(xi.v, xi.w);
    },
    py::arg("T"), "log of a (4, 4) rigid transform: (v, w) with |w| <= pi.");
  m.def(
    "body_twist",
    [](const DoubleArray & a, const DoubleArray & b, double dt) {
      const d3::FrameTwist3 xi = d3::bodyTwist(toIsometry3(a, "a"), toIsometry3(b, "b"), dt);
      return py::make_tuple(xi.v, xi.w);
    },
    py::arg("a"), py::arg("b"), py::arg("dt"),
    "log(a^-1 b)/dt: the constant body twist (v, w) moving a to b in dt.");

  py::enum_<d3::FrameSampleResult3>(m, "FrameSampleResult", "What a sample did to the estimator.")
    .value("ACCEPTED", d3::FrameSampleResult3::kAccepted)
    .value("STARTED", d3::FrameSampleResult3::kStarted)
    .value("DUPLICATE", d3::FrameSampleResult3::kDuplicate)
    .value("TOO_CLOSE", d3::FrameSampleResult3::kTooClose)
    .value("OUTLIER", d3::FrameSampleResult3::kOutlier)
    .value("INVALID", d3::FrameSampleResult3::kInvalid);

  StructBinder<d3::FrameTwistConfig3>(m, "FrameTwistConfig", "FrameTwistEstimator settings.")
    .field("min_dt", &d3::FrameTwistConfig3::min_dt, "Minimum pair dt (s).")
    .field("max_dt", &d3::FrameTwistConfig3::max_dt, "Maximum pair dt (s); longer restarts.")
    .field("cutoff_hz", &d3::FrameTwistConfig3::cutoff_hz, "LPF cutoff (Hz); <= 0 disables it.")
    .field("max_speed", &d3::FrameTwistConfig3::max_speed, "Outlier speed (m/s).")
    .field("max_angular_rate", &d3::FrameTwistConfig3::max_angular_rate, "Outlier rate (rad/s).")
    .field("input_timeout", &d3::FrameTwistConfig3::input_timeout, "Staleness age (s).")
    .finish();

  py::class_<d3::FrameEstimate3>(m, "FrameEstimate", "The frame at a query time.")
    .def_readonly("ready", &d3::FrameEstimate3::ready, "A pose and a twist are known.")
    .def_readonly("stale", &d3::FrameEstimate3::stale, "Not ready or older than input_timeout.")
    .def_property_readonly(
      "pose", [](const d3::FrameEstimate3 & e) { return fromIsometry3(e.pose); },
      "Frame pose (4, 4), extrapolated or held.")
    .def_property_readonly(
      "v", [](const d3::FrameEstimate3 & e) { return e.twist.v; }, "Linear velocity (3,).")
    .def_property_readonly(
      "w", [](const d3::FrameEstimate3 & e) { return e.twist.w; }, "Angular velocity (3,).")
    .def_readonly("age", &d3::FrameEstimate3::age, "Age of the latest sample (s).")
    .def("curve_frame", &d3::FrameEstimate3::curveFrame, "The CurveFrame (moving iff not stale).");

  py::class_<d3::FrameTwistEstimator3>(
    m, "FrameTwistEstimator",
    "Pose and body twist of a moving frame from sampled poses (log_SE3 differencing, outlier "
    "rejection, low-pass) or from twist samples.")
    .def(py::init<const d3::FrameTwistConfig3 &>(), py::arg("config") = d3::FrameTwistConfig3{})
    .def_property(
      "config", [](const d3::FrameTwistEstimator3 & e) { return e.config(); },
      &d3::FrameTwistEstimator3::setConfig, "Settings (a copy).")
    .def("reset", &d3::FrameTwistEstimator3::reset, "Forgets every sample.")
    .def(
      "add_pose",
      [](d3::FrameTwistEstimator3 & e, double stamp, const DoubleArray & pose) {
        return e.addPose(stamp, toIsometry3(pose, "pose"));
      },
      py::arg("stamp"), py::arg("pose"), "A (4, 4) pose sample of the frame.")
    .def(
      "add_twist",
      [](
        d3::FrameTwistEstimator3 & e, double stamp, const DoubleArray & pose, const DoubleArray & v,
        const DoubleArray & w) {
        d3::FrameTwist3 xi;
        xi.v = toVec3(v, "v");
        xi.w = toVec3(w, "w");
        return e.addTwist(stamp, toIsometry3(pose, "pose"), xi);
      },
      py::arg("stamp"), py::arg("pose"), py::arg("v"), py::arg("w"),
      "A pose and the body twist in the frame (Odometry).")
    .def("estimate", &d3::FrameTwistEstimator3::estimate, py::arg("now"), "The frame at `now`.")
    .def_property_readonly(
      "outliers", &d3::FrameTwistEstimator3::outliers, "Samples dropped as outliers.");
}

// Signed distances of one point (3,) or of (N, 3) points: (D, n) as a float or (N,), and (3,) or
// (N, 3).
py::tuple distances3(
  const DoubleArray & points, const std::function<d3::ObstacleDistance3(const Vec3 &)> & f)
{
  if (points.ndim() == 1) {
    const d3::ObstacleDistance3 d = f(toVec3(points, "point"));
    return py::make_tuple(d.D, d.n);
  }
  const std::vector<Vec3> pts = toPoints3(points, "points");
  py::array_t<double> D(static_cast<py::ssize_t>(pts.size()));
  std::vector<Vec3> normals(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const d3::ObstacleDistance3 d = f(pts[i]);
    D.mutable_at(static_cast<py::ssize_t>(i)) = d.D;
    normals[i] = d.n;
  }
  return py::make_tuple(D, fromPoints3(normals));
}

// A (nz, ny, nx) mask (C order, x fastest, as the planar (height, width) masks) into a grid.
void setVoxelMask(
  d3::VoxelGrid3 & grid, const MaskArray & mask, double resolution, const DoubleArray & origin)
{
  if (mask.ndim() != 3) {
    throw py::value_error(
      "mask must be a (nz, ny, nx) array (mask[iz, iy, ix], nonzero = occupied)");
  }
  const Vec3 o = toVec3(origin, "origin");
  const bool ok = grid.setFromMask(
    vf::Span<const std::uint8_t>(mask.data(), static_cast<std::size_t>(mask.size())),
    static_cast<int>(mask.shape(2)), static_cast<int>(mask.shape(1)),
    static_cast<int>(mask.shape(0)), resolution, o);
  if (!ok) {
    throw py::value_error(
      "invalid voxel grid: each side in [1, 2^20], at most 2^31 - 1 voxels, a finite "
      "resolution > 0 and a finite origin");
  }
}

py::dict siteToDict(const d3::ObstacleSite3 & site)
{
  py::dict d;
  d["x_o"] = site.x_o;
  d["n"] = site.n;
  d["D"] = site.D;
  d["source"] = site.source;
  return d;
}

void bindObstacles3(py::module_ & m)
{
  py::enum_<d3::PrimitiveType3>(m, "PrimitiveType", "Kind of an analytic obstacle.")
    .value("SPHERE", d3::PrimitiveType3::kSphere)
    .value("BOX", d3::PrimitiveType3::kBox)
    .value("CYLINDER", d3::PrimitiveType3::kCylinder)
    .value("PLANE", d3::PrimitiveType3::kPlane);

  py::class_<d3::Primitive3>(
    m, "Primitive",
    "An analytic obstacle (a convex solid) with an exact signed distance: build it with "
    "Primitive.sphere / box / cylinder / plane (math_spec 14.8).")
    .def(py::init<>())
    .def_static(
      "sphere",
      [](const DoubleArray & center, double radius) {
        return d3::Primitive3::sphere(toVec3(center, "center"), radius);
      },
      py::arg("center"), py::arg("radius"), "A ball.")
    .def_static(
      "box",
      [](
        const DoubleArray & center, const DoubleArray & half_extents, double yaw, double pitch,
        double roll) {
        return d3::Primitive3::box(
          toVec3(center, "center"), toVec3(half_extents, "half_extents"),
          d3::rotationZYX(yaw, pitch, roll));
      },
      py::arg("center"), py::arg("half_extents"), py::arg("yaw") = 0.0, py::arg("pitch") = 0.0,
      py::arg("roll") = 0.0,
      "A box with half-extents along its axes, oriented by yaw, pitch, roll (Z-Y'-X''; all "
      "zero: axis-aligned).")
    .def_static(
      "cylinder",
      [](const DoubleArray & center, double radius, double half_height) {
        return d3::Primitive3::cylinder(toVec3(center, "center"), radius, half_height);
      },
      py::arg("center"), py::arg("radius"), py::arg("half_height") = vf::kInf,
      "A vertical cylinder; the default half-height (inf) makes an infinite pole.")
    .def_static(
      "plane",
      [](const DoubleArray & point, const DoubleArray & normal) {
        return d3::Primitive3::plane(toVec3(point, "point"), toVec3(normal, "normal"));
      },
      py::arg("point"), py::arg("normal"),
      "The solid half-space behind the plane through `point`; `normal` points into free space.")
    .def_readwrite("type", &d3::Primitive3::type, "PrimitiveType.")
    // Vec3 and Mat3 members are converted by hand: GCC 13 reports a false
    // -Wmaybe-uninitialized inside pybind11's fixed-size Eigen caster (see Limits.max_axis_speed).
    .def_property(
      "center", [](const d3::Primitive3 & p) { return p.center; },
      [](d3::Primitive3 & p, const DoubleArray & v) { p.center = toVec3(v, "center"); },
      "Centre, or a point on the plane (3,).")
    .def_readwrite("radius", &d3::Primitive3::radius, "Sphere or cylinder radius (m).")
    .def_property(
      "half_extents", [](const d3::Primitive3 & p) { return p.half_extents; },
      [](d3::Primitive3 & p, const DoubleArray & v) { p.half_extents = toVec3(v, "half_extents"); },
      "Box half-extents (3,).")
    .def_property(
      "rotation", [](const d3::Primitive3 & p) { return p.rotation; },
      [](d3::Primitive3 & p, const DoubleArray & v) {
        if (v.ndim() != 2 || v.shape(0) != 3 || v.shape(1) != 3) {
          throw py::value_error("rotation must be a (3, 3) matrix");
        }
        const auto view = v.unchecked<2>();
        for (py::ssize_t i = 0; i < 3; ++i) {
          for (py::ssize_t j = 0; j < 3; ++j) {
            p.rotation(i, j) = view(i, j);
          }
        }
      },
      "Box axes as columns (3, 3).")
    .def_readwrite("half_height", &d3::Primitive3::half_height, "Cylinder half-height (m).")
    .def_property(
      "normal", [](const d3::Primitive3 & p) { return p.normal; },
      [](d3::Primitive3 & p, const DoubleArray & v) { p.normal = toVec3(v, "normal"); },
      "Plane normal into free space (3,).")
    .def(
      "validate", [](const d3::Primitive3 & p) { d3::validatePrimitive3(p); },
      "Raises ConfigError unless the primitive is well formed.")
    .def(
      "signed_distance",
      [](const d3::Primitive3 & p, const DoubleArray & points) {
        return distances3(points, [&p](const Vec3 & x) { return d3::signedDistance(p, x); });
      },
      py::arg("points"),
      "(D, n): the signed distance (negative inside) and the outward normal at a point (3,) or "
      "at (N, 3) points.")
    .def("__repr__", [](const d3::Primitive3 & p) {
      return "<vectorfield.d3.Primitive " + py::repr(py::cast(p.type)).cast<std::string>() + ">";
    });

  py::class_<d3::VoxelGrid3>(
    m, "VoxelGrid",
    "A voxel occupancy snapshot: a (nz, ny, nx) mask (mask[iz, iy, ix], nonzero = occupied), "
    "a voxel edge (m) and the position of the corner of voxel (0, 0, 0).")
    .def(py::init<>(), "An empty grid.")
    .def(
      py::init([](const MaskArray & mask, double resolution, const DoubleArray & origin) {
        d3::VoxelGrid3 grid;
        setVoxelMask(grid, mask, resolution, origin);
        return grid;
      }),
      py::arg("mask"), py::arg("resolution"), py::arg("origin"),
      "A grid from a (nz, ny, nx) mask; raises ValueError on inconsistent arguments.")
    .def(
      "set_from_mask", &setVoxelMask, py::arg("mask"), py::arg("resolution"), py::arg("origin"),
      "Replaces the content; version changes iff the geometry or a voxel changed.")
    .def_property_readonly(
      "shape", [](const d3::VoxelGrid3 & g) { return py::make_tuple(g.nz(), g.ny(), g.nx()); },
      "(nz, ny, nx).")
    .def_property_readonly("resolution", &d3::VoxelGrid3::resolution, "Voxel edge (m).")
    .def_property_readonly("origin", &d3::VoxelGrid3::origin, "Corner of voxel (0, 0, 0).")
    .def_property_readonly("version", &d3::VoxelGrid3::version, "Content stamp (0 when empty).")
    .def_property_readonly(
      "mask",
      [](const d3::VoxelGrid3 & g) {
        py::array_t<std::uint8_t> out(std::vector<py::ssize_t>{g.nz(), g.ny(), g.nx()});
        std::copy(g.mask().begin(), g.mask().end(), out.mutable_data());
        return out;
      },
      "A copy of the mask, (nz, ny, nx).")
    .def(
      "cell_center",
      [](const d3::VoxelGrid3 & g, int ix, int iy, int iz) { return g.cellCenter(ix, iy, iz); },
      py::arg("ix"), py::arg("iy"), py::arg("iz"), "Centre of a voxel (3,).");

  py::class_<d3::DistanceTransform3>(
    m, "DistanceTransform",
    "Exact 3D Euclidean distance and feature transform (Felzenszwalb-Huttenlocher, separable) "
    "between voxel centres.")
    .def(py::init<>(), "An empty transform.")
    .def(
      py::init([](const d3::VoxelGrid3 & grid) {
        d3::DistanceTransform3 edt;
        edt.update(grid);
        return edt;
      }),
      py::arg("grid"), "The transform of `grid`.")
    .def(
      "update", &d3::DistanceTransform3::update, py::arg("grid"),
      py::call_guard<py::gil_scoped_release>(),
      "Rebuilds iff the grid's version changed; True iff it rebuilt.")
    .def(
      "build", &d3::DistanceTransform3::build, py::arg("grid"),
      py::call_guard<py::gil_scoped_release>(), "Rebuilds unconditionally.")
    .def_property_readonly("has_sites", &d3::DistanceTransform3::hasSites)
    .def(
      "distances",
      [](const d3::DistanceTransform3 & e) {
        py::array_t<double> out(std::vector<py::ssize_t>{e.nz(), e.ny(), e.nx()});
        auto view = out.mutable_unchecked<3>();
        for (int iz = 0; iz < e.nz(); ++iz) {
          for (int iy = 0; iy < e.ny(); ++iy) {
            for (int ix = 0; ix < e.nx(); ++ix) {
              view(iz, iy, ix) = e.distance(ix, iy, iz);
            }
          }
        }
        return out;
      },
      "Distances (m) between voxel centres and the nearest occupied voxel centre, (nz, ny, nx).")
    .def(
      "squared_distance_cells",
      [](const d3::DistanceTransform3 & e, int ix, int iy, int iz) {
        if (ix < 0 || iy < 0 || iz < 0 || ix >= e.nx() || iy >= e.ny() || iz >= e.nz()) {
          throw py::index_error("voxel out of bounds");
        }
        return e.squaredDistanceCells(ix, iy, iz);
      },
      py::arg("ix"), py::arg("iy"), py::arg("iz"), "Exact squared distance (voxel units^2).")
    .def(
      "feature",
      [](const d3::DistanceTransform3 & e, int ix, int iy, int iz) -> py::object {
        if (ix < 0 || iy < 0 || iz < 0 || ix >= e.nx() || iy >= e.ny() || iz >= e.nz()) {
          throw py::index_error("voxel out of bounds");
        }
        int fx = 0;
        int fy = 0;
        int fz = 0;
        if (!e.feature(ix, iy, iz, &fx, &fy, &fz)) {
          return py::none();
        }
        return py::make_tuple(fx, fy, fz);
      },
      py::arg("ix"), py::arg("iy"), py::arg("iz"),
      "The nearest occupied voxel (ix, iy, iz), or None without sites.")
    .def(
      "query",
      [](const d3::DistanceTransform3 & e, const DoubleArray & points) {
        return distances3(points, [&e](const Vec3 & x) { return e.query(x); });
      },
      py::arg("points"),
      "(D, n) of the continuous query (trilinear blend of feature vectors); D = inf outside "
      "the grid.");

  py::class_<d3::ObstacleSet3>(
    m, "ObstacleSet",
    "The static obstacles of a scene in the evaluation frame: primitives and an optional voxel "
    "grid (its distance transform is rebuilt only when the grid changes).")
    .def(py::init<>())
    .def(
      py::init([](const std::vector<d3::Primitive3> & primitives) {
        d3::ObstacleSet3 set;
        for (const auto & p : primitives) {
          set.add(p);
        }
        return set;
      }),
      py::arg("primitives"), "A set of primitives (each validated).")
    .def("add", &d3::ObstacleSet3::add, py::arg("primitive"), "Adds a validated primitive.")
    .def("clear_primitives", &d3::ObstacleSet3::clearPrimitives)
    .def_property_readonly(
      "primitives", [](const d3::ObstacleSet3 & s) { return s.primitives(); },
      "The primitives (a copy).")
    .def(
      "set_voxels", &d3::ObstacleSet3::setVoxels, py::arg("grid"),
      py::call_guard<py::gil_scoped_release>(),
      "Copies a VoxelGrid; True iff its distance transform was rebuilt.")
    .def("clear_voxels", &d3::ObstacleSet3::clearVoxels)
    .def_property_readonly("has_voxels", &d3::ObstacleSet3::hasVoxels)
    .def_property_readonly("voxels", &d3::ObstacleSet3::voxels, "The VoxelGrid (a copy).")
    .def_property_readonly("empty", &d3::ObstacleSet3::empty)
    .def(
      "nearest",
      [](const d3::ObstacleSet3 & s, const DoubleArray & points) {
        return distances3(points, [&s](const Vec3 & x) { return s.nearest(x); });
      },
      py::arg("points"), "(D, n) of the nearest obstacle at a point (3,) or (N, 3) points.")
    .def(
      "sites",
      [](const d3::ObstacleSet3 & s, const DoubleArray & point, int count) {
        d3::ObstacleSite3 sites[d3::kCbf3MaxSites];
        const int n = s.sites(toVec3(point, "point"), count, sites);
        py::list out;
        for (int j = 0; j < n; ++j) {
          out.append(siteToDict(sites[j]));
        }
        return out;
      },
      py::arg("point"), py::arg("count") = d3::kCbf3MaxSites,
      "The CBF sites at a point, nearest first: dicts with x_o, n, D and source (primitive "
      "index, -1 for voxels).");

  py::class_<d3::Obstacles3Config>(
    m, "ObstaclesConfig", "Obstacle handling of the 3D tracker (TrackerConfig.obstacles).")
    .def(py::init<>())
    .def_readwrite(
      "obstacle_mode", &d3::Obstacles3Config::obstacle_mode,
      "vectorfield.ObstacleMode: NONE or CBF (CONTOUR is planar only).")
    .def_readwrite("body_radius", &d3::Obstacles3Config::body_radius, "Body sphere radius (m).")
    .def_readwrite("cbf_margin", &d3::Obstacles3Config::cbf_margin, "Margin m (m).")
    .def_readwrite("cbf_gain", &d3::Obstacles3Config::cbf_gain, "gamma_b (1/s).")
    .def_readwrite("cbf_sites", &d3::Obstacles3Config::cbf_sites, "Sites k in [1, 4].");

  // Getters by value: GCC 15 reports a false -Wmaybe-uninitialized in def_readonly here.
  py::class_<d3::Cbf3Sample>(m, "CbfSample", "The CBF filter of one cycle (TrackerOutput.cbf).")
    .def_property_readonly(
      "on", [](const d3::Cbf3Sample & c) { return c.on; }, "Mode CBF with obstacles.")
    .def_property_readonly(
      "active", [](const d3::Cbf3Sample & c) { return c.active; }, "The filter changed the target.")
    .def_property_readonly(
      "relaxed", [](const d3::Cbf3Sample & c) { return c.relaxed; },
      "Conflicting push-outs were dropped.")
    .def_property_readonly(
      "hold", [](const d3::Cbf3Sample & c) { return c.hold; }, "The hold condition (kBlocked).")
    .def_property_readonly(
      "count", [](const d3::Cbf3Sample & c) { return c.count; }, "Constrained sites.")
    .def_property_readonly(
      "delta", [](const d3::Cbf3Sample & c) { return c.delta; },
      "delta_safe = body_radius + cbf_margin.")
    .def_property_readonly(
      "h", [](const d3::Cbf3Sample & c) { return c.h; }, "Least barrier min_j D_j - delta_safe.")
    .def_property_readonly(
      "speed_ratio", [](const d3::Cbf3Sample & c) { return c.speed_ratio; }, "|F*| / |F|.")
    .def_property_readonly(
      "F_in", [](const d3::Cbf3Sample & c) { return c.F_in; }, "Target before the filter (3,).")
    .def_property_readonly(
      "F", [](const d3::Cbf3Sample & c) { return c.F; }, "F* (3,).")
    .def_property_readonly(
      "binds", [](const d3::Cbf3Sample & c) { return c.binds; }, "Bit j: site j binds.")
    .def_property_readonly(
      "sites",
      [](const d3::Cbf3Sample & c) {
        py::list out;
        for (int j = 0; j < c.count; ++j) {
          out.append(siteToDict(c.sites[j]));
        }
        return out;
      },
      "The sites (dicts with x_o, n, D, source), nearest first.");

  m.def(
    "solve_cbf",
    [](const DoubleArray & F, const DoubleArray & normals, const DoubleArray & h, double gain) {
      const std::vector<Vec3> n = toPoints3(normals, "normals");
      if (h.ndim() != 1 || static_cast<std::size_t>(h.shape(0)) != n.size()) {
        throw py::value_error("h must be a (k,) array matching the (k, 3) normals");
      }
      if (n.size() > static_cast<std::size_t>(d3::kCbf3MaxSites)) {
        throw py::value_error("at most 4 constraints");
      }
      d3::Cbf3Constraint c[d3::kCbf3MaxSites];
      for (std::size_t j = 0; j < n.size(); ++j) {
        c[j].n = n[j];
        c[j].h = h.at(static_cast<py::ssize_t>(j));
      }
      const d3::Cbf3Solution s = d3::solveCbf3(toVec3(F, "F"), c, static_cast<int>(n.size()), gain);
      return py::make_tuple(s.F, s.active, s.relaxed, static_cast<int>(s.binds));
    },
    py::arg("F"), py::arg("normals"), py::arg("h"), py::arg("gain"),
    "The 3D CBF filter: argmin |F' - F| s.t. n_j . F' >= -gain h_j (k <= 4 unit normals); "
    "returns (F*, active, relaxed, binds).");
}

void bindTracker3(py::module_ & m, py::module_ & sim)
{
  bindFrames3(m);

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
    .def_readwrite("yaw", &d3::Tracker3Config::yaw, "YawConfig.")
    .def_readwrite(
      "obstacles", &d3::Tracker3Config::obstacles, "ObstaclesConfig (math_spec 14.8).");

  py::class_<d3::Tracker3State>(m, "TrackerState", "Cross-cycle state (copyable).")
    .def(py::init<>())
    .def_readonly("phase", &d3::Tracker3State::phase, "Phase.")
    .def_readonly("xy_latched", &d3::Tracker3State::xy_latched, "Goal latch.")
    .def_readonly("holding", &d3::Tracker3State::holding, "Goal hold.")
    .def_readonly("blocked", &d3::Tracker3State::blocked, "CBF hold of the last step.")
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
    .def_readonly("psi_d", &d3::Tracker3Output::psi_d, "Desired yaw.")
    .def_property_readonly(
      "Phi_T", [](const d3::Tracker3Output & o) { return o.field.Phi_T; }, "Feedforward (3,).")
    .def_property_readonly(
      "v_c", [](const d3::Tracker3Output & o) { return o.field.v_c; },
      "Curve velocity at the foot (transport velocity for an open curve in a moving frame).")
    .def_readonly(
      "frame_velocity", &d3::Tracker3Output::frame_velocity, "Frame velocity at the body (3,).")
    .def_readonly("x_star_curve", &d3::Tracker3Output::x_star_curve, "Foot in the curve frame.")
    .def_property_readonly(
      "cmd_target_v", [](const d3::Tracker3Output & o) { return o.cmd_target.v; },
      "Target velocity before the limiter (3,).")
    .def_readonly("cbf", &d3::Tracker3Output::cbf, "CbfSample of this cycle.");

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
        double speed_limit_scale, bool cancel, const py::object & frame,
        const py::object & obstacles) {
        d3::Tracker3Input input;
        input.curve = curve.curve.get();
        if (!obstacles.is_none()) {
          input.obstacles = obstacles.cast<const d3::ObstacleSet3 *>();
        }
        if (!frame.is_none()) {
          input.frame = frame.cast<d3::CurveFrame3>();
        }
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
      py::arg("speed_limit_scale") = 1.0, py::arg("cancel") = false, py::arg("frame") = py::none(),
      py::arg("obstacles") = py::none(),
      "One control cycle; returns (TrackerOutput, next TrackerState). `frame`: the CurveFrame "
      "of this cycle (default identity, static); `obstacles`: an ObstacleSet in the evaluation "
      "frame (used with obstacles.obstacle_mode CBF).");

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
                 bool stop_when_settled, bool stop_when_stopped, const py::object & frame,
                 const py::object & frame_motion, const py::object & obstacles) {
        d3::sim::SimulationOptions3 options;
        options.period = period;
        options.plant.velocity_time_constant = velocity_time_constant;
        options.stop_when_settled = stop_when_settled;
        options.stop_when_stopped = stop_when_stopped;
        if (!frame.is_none()) {
          options.frame = frame.cast<d3::CurveFrame3>();
        }
        options.frame_motion = frameMotion(frame_motion);
        if (!obstacles.is_none()) {
          options.obstacles = obstacles.cast<d3::ObstacleSet3>();
        }
        return Simulator3Box{d3::sim::ClosedLoopSimulator3(tracker, options)};
      }),
      py::arg("tracker"), py::arg("period") = 0.05, py::arg("velocity_time_constant") = 0.0,
      py::arg("stop_when_settled") = true, py::arg("stop_when_stopped") = true,
      py::arg("frame") = py::none(), py::arg("frame_motion") = py::none(),
      py::arg("obstacles") = py::none(),
      "`frame`: the CurveFrame at the reset time, moved with its constant twist; "
      "`frame_motion(t) -> CurveFrame` overrides it (math_spec 14.7). `obstacles`: an "
      "ObstacleSet (copied) for the tracker's CBF mode (math_spec 14.8).")
    .def(
      "frame_at", [](const Simulator3Box & s, double t) { return s.simulator.frameAt(t); },
      py::arg("t"), "The curve frame at time t.")
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
      "status, settled, frame (N, 4, 4), h (the nearest obstacle's barrier, inf without the "
      "filter), cbf_active.")
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
  bindObstacles3(d3m);
  bindTracker3(d3m, sim3);
}

}  // namespace vectorfield_py
