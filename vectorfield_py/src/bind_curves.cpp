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

// Curves: CurveSpec and make_curve(), the Curve interface, PolylineCurve and its SearchPolicy,
// the closest-point types and the polyline conditioning helpers (curve*.hpp, geometry.hpp,
// polyline_curve.hpp, conditioning.hpp).
//
// make_curve() returns an immutable curve shared by its Python handles (the C++
// shared_ptr<const Curve>). Expression curves are non-copyable and serialise their muParser
// evaluations internally; for parallel use build one curve per thread with make_curve(spec).
// The batch queries release the GIL: the curve must not be modified (PolylineCurve.assign)
// from another thread meanwhile.

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "bindings.hpp"

namespace vectorfield_py
{

namespace
{

using vf::Curve;
using vf::CurveSpec;
using vf::PolylineCurve;

void setParameters(CurveSpec & spec, const py::dict & parameters)
{
  spec.parameter_names.clear();
  spec.parameter_values.clear();
  for (const auto & item : parameters) {
    spec.parameter_names.push_back(item.first.cast<std::string>());
    spec.parameter_values.push_back(item.second.cast<double>());
  }
}

void bindSpec(py::module_ & m)
{
  StructBinder<CurveSpec> spec(
    m, "CurveSpec",
    "A curve description, field for field vectorfield_msgs/msg/Curve.msg (without the header). "
    "Build curves from it with make_curve(); the factories polyline(), expression() and "
    "builtin() fill the relevant fields.");
  py::enum_<CurveSpec::Type>(spec.cls(), "Type", "Curve type; values equal Curve.msg.")
    .value("POLYLINE", CurveSpec::Type::kPolyline)
    .value("EXPRESSION", CurveSpec::Type::kExpression)
    .value("BUILTIN", CurveSpec::Type::kBuiltin);
  spec.field("type", &CurveSpec::type, "CurveSpec.Type.")
    .field("closed", &CurveSpec::closed, "Closed curve (ignored for BUILTIN).")
    .field("points", &CurveSpec::points, "POLYLINE vertices (N, 2), curve frame.")
    .field("x_expression", &CurveSpec::x_expression, "EXPRESSION x(s, t, params).")
    .field("y_expression", &CurveSpec::y_expression, "EXPRESSION y(s, t, params).")
    .field("s_min", &CurveSpec::s_min, "Domain start (BUILTIN: 0 and 0 = natural domain).")
    .field("s_max", &CurveSpec::s_max, "Domain end.")
    .field("builtin_shape", &CurveSpec::builtin_shape, "BUILTIN shape name (builtin_shapes()).")
    .field(
      "parameter_names", &CurveSpec::parameter_names,
      "Parameter names (a copy: assign a whole list, or use `parameters`).")
    .field("parameter_values", &CurveSpec::parameter_values, "Parameter values (a copy).")
    .field("sample_spacing", &CurveSpec::sample_spacing, "POLYLINE resampling (m); <= 0: default.")
    .field(
      "max_corner_deviation", &CurveSpec::max_corner_deviation,
      "POLYLINE corner fillet deviation (m); <= 0: none.")
    .field("t0", &CurveSpec::t0, "Time origin t0 of r(s, t) (s).");
  spec.cls()
    .def_property(
      "parameters",
      [](const CurveSpec & s) {
        py::dict out;
        const std::size_t n = std::min(s.parameter_names.size(), s.parameter_values.size());
        for (std::size_t i = 0; i < n; ++i) {
          out[py::str(s.parameter_names[i])] = s.parameter_values[i];
        }
        return out;
      },
      &setParameters, "Parameters as a {name: value} dict (a copy).")
    .def_static(
      "polyline",
      [](const DoubleArray & points, bool closed, double sample_spacing, double deviation) {
        CurveSpec s;
        s.type = CurveSpec::Type::kPolyline;
        s.points = toPoints(points, "points");
        s.closed = closed;
        s.sample_spacing = sample_spacing;
        s.max_corner_deviation = deviation;
        return s;
      },
      py::arg("points"), py::arg("closed") = false, py::arg("sample_spacing") = 0.0,
      py::arg("max_corner_deviation") = 0.0, "A POLYLINE spec.")
    .def_static(
      "expression",
      [](
        const std::string & x, const std::string & y, double s_min, double s_max, bool closed,
        const std::optional<py::dict> & parameters, double t0) {
        CurveSpec s;
        s.type = CurveSpec::Type::kExpression;
        s.x_expression = x;
        s.y_expression = y;
        s.s_min = s_min;
        s.s_max = s_max;
        s.closed = closed;
        s.t0 = t0;
        if (parameters) {
          setParameters(s, *parameters);
        }
        return s;
      },
      py::arg("x"), py::arg("y"), py::arg("s_min"), py::arg("s_max"), py::arg("closed") = false,
      py::arg("parameters") = py::none(), py::arg("t0") = 0.0,
      "An EXPRESSION spec: muParser expressions in s, t and the named parameters.")
    .def_static(
      "builtin",
      [](const std::string & shape, const py::kwargs & parameters) {
        CurveSpec s;
        s.type = CurveSpec::Type::kBuiltin;
        s.builtin_shape = shape;
        setParameters(s, parameters);
        return s;
      },
      py::arg("shape"),
      "A BUILTIN spec, parameters as keywords: CurveSpec.builtin('ellipse', a=2.0, b=1.0).")
    .def(
      "__eq__", [](const CurveSpec & a, const CurveSpec & b) { return a == b; }, py::is_operator())
    .def(
      "__ne__", [](const CurveSpec & a, const CurveSpec & b) { return a != b; }, py::is_operator());
  spec.finish();
}

void bindClosestPointTypes(py::module_ & m)
{
  StructBinder<vf::CurveHint>(
    m, "CurveHint",
    "Search state carried across cycles: previous foot x_star, curve parameter s (arc length or "
    "u), segment, and valid (False before the first search).")
    .field("x_star", &vf::CurveHint::x_star, "Previous foot point.")
    .field("s", &vf::CurveHint::s, "Previous curve parameter.")
    .field("segment", &vf::CurveHint::segment, "Previous segment, -1 if unknown.")
    .field("valid", &vf::CurveHint::valid, "False before the first search.")
    .finish();

  StructBinder<vf::EvalContext>(m, "EvalContext", "Per-call context of a closest-point search.")
    .field("t", &vf::EvalContext::t, "Evaluation time (s); ignored by static curves.")
    .field("direction", &vf::EvalContext::direction, "Traversal direction +1 or -1.")
    .field(
      "last_tangent", &vf::EvalContext::last_tangent,
      "T_last for the hairpin rule (curve frame); zero = none.")
    .finish();

  py::enum_<vf::PieceKind>(m, "PieceKind", "Which kind of piece owns a closest point.")
    .value("NONE", vf::PieceKind::kNone)
    .value("SEGMENT", vf::PieceKind::kSegment)
    .value("VERTEX", vf::PieceKind::kVertex)
    .value("BACKWARD_RAY", vf::PieceKind::kBackwardRay)
    .value("FORWARD_RAY", vf::PieceKind::kForwardRay)
    .value("PARAMETRIC", vf::PieceKind::kParametric);

  using vf::ClosestPoint;
  StructBinder<ClosestPoint> cp(
    m, "ClosestPoint",
    "Result of a closest-point search: foot x_star, unit tangent T (parameter direction), "
    "normal n (zero on the curve), distance D, station s_star, parameter u_star and flags.");
  cp.field("x_star", &ClosestPoint::x_star, "Foot point.")
    .field("T", &ClosestPoint::T, "Unit tangent in the curve's parameter direction.")
    .field("n", &ClosestPoint::n, "Unit normal (x - x_star)/D, or zero on the curve.")
    .field("D", &ClosestPoint::D, "Distance (m).")
    .field("s_star", &ClosestPoint::s_star, "Arc-length station (negative on the backward ray).")
    .field("u_star", &ClosestPoint::u_star, "Curve parameter (polylines: s_star).")
    .field("segment", &ClosestPoint::segment, "Owning segment or vertex index, -1 if none.")
    .field("piece", &ClosestPoint::piece, "PieceKind.")
    .field("on_ray", &ClosestPoint::on_ray, "The foot lies on an end ray.")
    .field("in_wedge", &ClosestPoint::in_wedge, "A vertex owns x (wedge tangent).")
    .field("hairpin", &ClosestPoint::hairpin, "The hairpin rule produced T.")
    .field("valid", &ClosestPoint::valid, "False when no closest point exists.")
    .field("status_bits", &ClosestPoint::status_bits, "Status bits raised by the search.");
  cp.cls().def(
    "transformed", &vf::transformed, py::arg("T_eval_curve"),
    "This result mapped from the curve frame into the evaluation frame.");
  cp.finish();

  m.def(
    "make_hint", &vf::makeHint, py::arg("closest_point"),
    "The hint for the next search from a result (in the evaluation frame).");

  using vf::SearchPolicy;
  StructBinder<SearchPolicy> policy(
    m, "SearchPolicy",
    "Closest-point search policy of a PolylineCurve: WINDOW_SCAN (Nav2 plugin) or HINTED "
    "(curve follower, the default).");
  py::enum_<SearchPolicy::Kind>(policy.cls(), "Kind", "Policy kind.")
    .value("WINDOW_SCAN", SearchPolicy::Kind::kWindowScan)
    .value("HINTED", SearchPolicy::Kind::kHinted);
  policy.field("kind", &SearchPolicy::kind, "SearchPolicy.Kind.")
    .field("s_min", &SearchPolicy::s_min, "WINDOW_SCAN lower station bound (m).")
    .field("s_max", &SearchPolicy::s_max, "WINDOW_SCAN upper station bound (m).")
    .field("window", &SearchPolicy::window, "HINTED window w (m).")
    .field("global_trigger", &SearchPolicy::global_trigger, "HINTED D_trig (m).")
    .field("global_hysteresis", &SearchPolicy::global_hysteresis, "HINTED h_g (m).");
  policy.cls()
    .def_static(
      "window_scan", &SearchPolicy::windowScan, py::arg("s_min"), py::arg("s_max"),
      "A WINDOW_SCAN policy over [s_min, s_max].")
    .def_static(
      "hinted",
      [](double window, double trigger, double hysteresis) {
        return SearchPolicy::hinted(window, trigger, hysteresis);
      },
      py::arg("window") = 1.0, py::arg("trigger") = 0.5, py::arg("hysteresis") = 0.10,
      "A HINTED policy.")
    .def_static(
      "from_options", [](const vf::CurveOptions & o) { return SearchPolicy::hinted(o); },
      py::arg("options"), "The HINTED policy of CurveOptions.");
  policy.finish();
}

// Upper bound of the stations sample_stations() writes over [s0, s1] with step ds.
std::size_t stationCount(double s0, double s1, double ds, std::size_t max_points)
{
  if (!(ds > 0.0) || !std::isfinite(s0) || !std::isfinite(s1) || !std::isfinite(ds) || s1 < s0) {
    return 0;
  }
  const double n = std::floor((s1 - s0) / ds) + 2.0;
  return n < static_cast<double>(max_points) ? static_cast<std::size_t>(n) : max_points;
}

py::array_t<double> sampleStations(
  const Curve & c, double s0, double s1, double ds, double t, std::size_t max_points)
{
  std::vector<vf::Vec2> out(stationCount(s0, s1, ds, max_points));
  const std::size_t n = c.sampleStations(s0, s1, ds, t, out);
  out.resize(n);
  return fromPoints(out);
}

void bindCurve(py::module_ & m)
{
  py::class_<Curve, std::shared_ptr<Curve>> curve(
    m, "Curve",
    "A planar curve r(s, t) in its own frame, open (extended by end rays) or closed, possibly "
    "time-varying. Built by make_curve(), or a PolylineCurve.");
  curve
    .def(
      "closest_point",
      [](
        const Curve & c, const vf::Vec2 & x, const std::optional<vf::EvalContext> & context,
        const std::optional<vf::CurveHint> & hint) {
        return c.closestPoint(
          x, context.value_or(vf::EvalContext{}), hint.value_or(vf::CurveHint{}));
      },
      py::arg("x"), py::arg("context") = py::none(), py::arg("hint") = py::none(),
      "Closest point of x (curve frame). Without a hint the search is global (initialisation).")
    .def(
      "closest_points",
      [](const Curve & c, const DoubleArray & points, const std::optional<vf::EvalContext> & ctx) {
        const std::vector<vf::Vec2> xs = toPoints(points, "points");
        const vf::EvalContext context = ctx.value_or(vf::EvalContext{});
        std::vector<vf::ClosestPoint> cps(xs.size());
        {
          const py::gil_scoped_release release;
          for (std::size_t i = 0; i < xs.size(); ++i) {
            cps[i] = c.closestPoint(xs[i], context, vf::CurveHint{});
          }
        }
        std::vector<vf::Vec2> x_star(cps.size());
        std::vector<vf::Vec2> T(cps.size());
        std::vector<vf::Vec2> n(cps.size());
        const auto count = static_cast<py::ssize_t>(cps.size());
        py::array_t<double> D(count);
        py::array_t<double> s_star(count);
        py::array_t<bool> valid(count);
        for (std::size_t i = 0; i < cps.size(); ++i) {
          const auto k = static_cast<py::ssize_t>(i);
          x_star[i] = cps[i].x_star;
          T[i] = cps[i].T;
          n[i] = cps[i].n;
          D.mutable_at(k) = cps[i].D;
          s_star.mutable_at(k) = cps[i].s_star;
          valid.mutable_at(k) = cps[i].valid;
        }
        py::dict out;
        out["x_star"] = fromPoints(x_star);
        out["T"] = fromPoints(T);
        out["n"] = fromPoints(n);
        out["D"] = D;
        out["s_star"] = s_star;
        out["valid"] = valid;
        return out;
      },
      py::arg("points"), py::arg("context") = py::none(),
      "Independent (global) closest points of each row of an (N, 2) array: a dict of arrays "
      "x_star, T, n (N, 2) and D, s_star, valid (N,). Releases the GIL.")
    .def_property_readonly("is_closed", &Curve::isClosed, "True for closed curves.")
    .def_property_readonly(
      "is_time_varying", &Curve::isTimeVarying, "True iff the shape depends on t.")
    .def("length", &Curve::length, py::arg("t") = 0.0, "Length L(t) (rays excluded; one lap).")
    .def(
      "velocity", &Curve::velocity, py::arg("closest_point"), py::arg("t") = 0.0,
      "Shape velocity dr/dt at the foot of a closest_point() result (zero when static).")
    .def(
      "point_at", [](const Curve & c, double s, double t) -> vf::Vec2 { return c.pointAt(s, t); },
      py::arg("s"), py::arg("t") = 0.0,
      "r(s, t) at arc-length station s (scalar: (2,); array: (N, 2)); rays outside [0, L] "
      "for open curves, wrapped for closed curves.")
    .def(
      "point_at",
      [](const Curve & c, const DoubleArray & s, double t) {
        std::vector<vf::Vec2> out(static_cast<std::size_t>(s.size()));
        for (std::size_t i = 0; i < out.size(); ++i) {
          out[i] = c.pointAt(s.data()[i], t);
        }
        return fromPoints(out);
      },
      py::arg("s"), py::arg("t") = 0.0)
    .def(
      "tangent_at",
      [](const Curve & c, double s, double t) -> vf::Vec2 { return c.tangentAt(s, t); },
      py::arg("s"), py::arg("t") = 0.0, "Unit tangent T(s, t) (scalar or array of stations).")
    .def(
      "tangent_at",
      [](const Curve & c, const DoubleArray & s, double t) {
        std::vector<vf::Vec2> out(static_cast<std::size_t>(s.size()));
        for (std::size_t i = 0; i < out.size(); ++i) {
          out[i] = c.tangentAt(s.data()[i], t);
        }
        return fromPoints(out);
      },
      py::arg("s"), py::arg("t") = 0.0)
    .def(
      "sample_stations", &sampleStations, py::arg("s0"), py::arg("s1"), py::arg("ds"),
      py::arg("t") = 0.0, py::arg("max_points") = 1000000,
      "r(s_k, t) for s_k = s0 + k ds up to s1, plus r(s1) (N, 2); open curves clamp to [0, L].")
    .def(
      "sample",
      [](const Curve & c, double ds, double t, std::size_t max_points) {
        return sampleStations(c, 0.0, c.length(t), ds, t, max_points);
      },
      py::arg("ds") = 0.01, py::arg("t") = 0.0, py::arg("max_points") = 1000000,
      "The whole curve from s = 0 to L every ds (closed curves end where they start).")
    .def_property_readonly(
      "spec",
      [](const Curve & c) -> std::optional<CurveSpec> {
        if (c.spec() == nullptr) {
          return std::nullopt;
        }
        return *c.spec();
      },
      "A copy of the CurveSpec this curve was built from, or None.")
    .def_property_readonly(
      "vertex_spacing", &Curve::vertexSpacing,
      "Mean vertex spacing (m) of a polyline; 0 for parametric curves.")
    .def("end_station", &Curve::endStation, py::arg("t") = 0.0, "Station of the end, L(t).")
    .def(
      "wrap_station_difference", &Curve::wrapStationDifference, py::arg("ds"), py::arg("t") = 0.0,
      "Arc-length difference wrapped into (-L/2, L/2] for closed curves.")
    .def(
      "remaining_to_end",
      [](const Curve & c, const vf::ClosestPoint & cp, double t) {
        return vf::remainingToEnd(c, cp, t);
      },
      py::arg("closest_point"), py::arg("t") = 0.0,
      "sigma_end = max(0, L - s_star) for open curves; NaN for closed curves.");
}

void requireMutable(const PolylineCurve & c)
{
  if (c.spec() != nullptr) {
    throw py::value_error(
      "curves built by make_curve() are immutable; build a PolylineCurve() to assign vertices");
  }
}

void bindPolyline(py::module_ & m)
{
  py::enum_<vf::AssignStatus>(m, "AssignStatus", "Result of PolylineCurve.assign().")
    .value("OK", vf::AssignStatus::kOk)
    .value("TOO_FEW", vf::AssignStatus::kTooFew)
    .value("NON_FINITE", vf::AssignStatus::kNonFinite);

  py::class_<PolylineCurve, Curve, std::shared_ptr<PolylineCurve>>(
    m, "PolylineCurve",
    "A polyline with semi-infinite end rays (open) or closed, with wedge tangents and the "
    "WINDOW_SCAN / HINTED search policies. Assigned directly, without make_curve()'s "
    "conditioning (dedupe, fillet, resample).")
    .def(
      py::init([](
                 const std::optional<DoubleArray> & points, bool closed,
                 const std::optional<vf::SearchPolicy> & policy) {
        auto curve = std::make_shared<PolylineCurve>(policy.value_or(vf::SearchPolicy{}));
        if (points) {
          const std::vector<vf::Vec2> pts = toPoints(*points, "points");
          const vf::AssignStatus status = curve->assign(pts, closed);
          if (status != vf::AssignStatus::kOk) {
            throw vf::CurveError(
              vf::CurveError::Code::kInvalidCurve,
              status == vf::AssignStatus::kTooFew
                ? "PolylineCurve: fewer than 2 distinct vertices (closed: 3)"
                : "PolylineCurve: a vertex is not finite");
          }
        }
        return curve;
      }),
      py::arg("points") = py::none(), py::arg("closed") = false, py::arg("policy") = py::none(),
      "An empty curve, or one assigned from (N, 2) points (raises CurveError on failure).")
    .def(
      "assign",
      [](PolylineCurve & c, const DoubleArray & points, bool closed) {
        requireMutable(c);
        const std::vector<vf::Vec2> pts = toPoints(points, "points");
        return c.assign(pts, closed);
      },
      py::arg("points"), py::arg("closed") = false,
      "Replaces the vertices; returns AssignStatus (the curve is empty on failure).")
    .def(
      "reserve",
      [](PolylineCurve & c, std::size_t n) {
        requireMutable(c);
        c.reserve(n);
      },
      py::arg("vertex_count"), "Reserves capacity, so later assign() calls do not allocate.")
    .def(
      "clear",
      [](PolylineCurve & c) {
        requireMutable(c);
        c.clear();
      },
      "Empties the curve.")
    .def_property(
      "search_policy", &PolylineCurve::searchPolicy,
      [](PolylineCurve & c, const vf::SearchPolicy & p) {
        requireMutable(c);
        c.setSearchPolicy(p);
      },
      py::return_value_policy::copy, "The SearchPolicy (a copy; assign to change it).")
    .def("__len__", &PolylineCurve::size)
    .def_property_readonly("size", &PolylineCurve::size, "Vertex count N.")
    .def_property_readonly("empty", &PolylineCurve::empty, "True iff no polyline is assigned.")
    .def_property_readonly(
      "segment_count", &PolylineCurve::segmentCount, "N - 1 (open) or N (closed).")
    .def_property_readonly(
      "vertices", [](const PolylineCurve & c) { return fromPoints(c.vertices()); },
      "Vertices (N, 2).")
    .def_property_readonly(
      "stations",
      [](const PolylineCurve & c) {
        const std::size_t n = c.size() + (c.isClosed() && !c.empty() ? 1 : 0);
        py::array_t<double> out(static_cast<py::ssize_t>(n));
        for (std::size_t k = 0; k < n; ++k) {
          out.mutable_at(static_cast<py::ssize_t>(k)) = c.station(k);
        }
        return out;
      },
      "Vertex stations S_k (closed curves add S_N = L).")
    .def(
      "segment_tangent",
      [](const PolylineCurve & c, std::size_t k) -> vf::Vec2 {
        if (k >= c.segmentCount()) {
          throw py::index_error("segment index out of range");
        }
        return c.segmentTangent(k);
      },
      py::arg("k"), "Unit tangent of segment k.")
    .def(
      "segment_length",
      [](const PolylineCurve & c, std::size_t k) {
        if (k >= c.segmentCount()) {
          throw py::index_error("segment index out of range");
        }
        return c.segmentLength(k);
      },
      py::arg("k"), "Length of segment k (m).");
}

void bindFactory(py::module_ & m)
{
  m.def(
    "make_curve",
    [](const CurveSpec & spec, const vf::CurveOptions & options) {
      return std::const_pointer_cast<Curve>(vf::makeCurve(spec, options));
    },
    py::arg("spec"), py::arg("options") = vf::CurveOptions{},
    "Builds and validates a curve (the single validation point): POLYLINE conditioning, "
    "EXPRESSION hardening, BUILTIN parameter rules and the regularity and closure checks. "
    "Raises CurveError (code 'INVALID_CURVE' or 'UNSUPPORTED').");
  m.def(
    "builtin_shapes",
    []() {
      py::list shapes;
      for (const vf::BuiltinShapeInfo & info : vf::builtinShapes()) {
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
    "The built-in shapes: a list of {'name', 'parameters' ({name: default}), 'formula'}.");
}

void bindConditioning(py::module_ & m)
{
  m.def(
    "dedupe",
    [](const DoubleArray & points, bool closed, double min_spacing) {
      const std::vector<vf::Vec2> in = toPoints(points, "points");
      std::vector<vf::Vec2> out;
      vf::dedupe(in, closed, &out, min_spacing);
      return fromPoints(out);
    },
    py::arg("points"), py::arg("closed") = false, py::arg("min_spacing") = vf::kDedupeDistance,
    "Drops vertices closer than min_spacing to the last kept one (endpoints kept).");
  m.def(
    "fillet_corners",
    [](const DoubleArray & points, bool closed, double deviation, double arc_spacing) {
      if (!(arc_spacing > 0.0) || !std::isfinite(arc_spacing)) {
        throw py::value_error("arc_spacing must be positive and finite");
      }
      const std::vector<vf::Vec2> in = toPoints(points, "points");
      std::vector<vf::Vec2> out;
      vf::filletCorners(in, closed, deviation, arc_spacing, &out);
      return fromPoints(out);
    },
    py::arg("points"), py::arg("closed"), py::arg("max_corner_deviation"), py::arg("arc_spacing"),
    "Replaces corners by arcs deviating at most max_corner_deviation.");
  m.def(
    "resample",
    [](const DoubleArray & points, bool closed, double spacing) {
      if (!(spacing > 0.0) || !std::isfinite(spacing)) {
        throw py::value_error("spacing must be positive and finite");
      }
      const std::vector<vf::Vec2> in = toPoints(points, "points");
      std::vector<vf::Vec2> out;
      vf::resample(in, closed, spacing, &out);
      return fromPoints(out);
    },
    py::arg("points"), py::arg("closed"), py::arg("spacing"),
    "Splits each segment into equal pieces at most `spacing` long (vertices kept).");
  m.def(
    "kappa_path",
    [](const DoubleArray & points, bool closed) {
      const std::vector<vf::Vec2> in = toPoints(points, "points");
      return vf::kappaPath(in, closed);
    },
    py::arg("points"), py::arg("closed") = false,
    "kappa_path: max turning angle over mean adjacent segment length (1/m).");
}

}  // namespace

void bindCurves(py::module_ & m)
{
  bindSpec(m);
  bindClosestPointTypes(m);
  bindCurve(m);
  bindPolyline(m);
  bindFactory(m);
  bindConditioning(m);
}

}  // namespace vectorfield_py
