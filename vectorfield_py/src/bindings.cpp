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

// Compiled extension module `vectorfield._vectorfield`: the module, the exception types and the
// NumPy helpers shared by the bind_*.cpp files (plan section 8).
//
// Only the pybind11 2.9 API subset is used (jammy ships 2.9.1; Homebrew and resolute ship 3.x).

#include <cstdint>
#include <string>
#include <vector>

#include "bindings.hpp"

namespace vectorfield_py
{

std::vector<vf::Vec2> toPoints(const DoubleArray & array, const char * what)
{
  if (array.size() == 0) {
    return {};
  }
  if (array.ndim() != 2 || array.shape(1) != 2) {
    throw py::value_error(std::string(what) + " must be an (N, 2) array of points");
  }
  const auto view = array.unchecked<2>();
  std::vector<vf::Vec2> points(static_cast<std::size_t>(view.shape(0)));
  for (py::ssize_t i = 0; i < view.shape(0); ++i) {
    points[static_cast<std::size_t>(i)] = vf::Vec2(view(i, 0), view(i, 1));
  }
  return points;
}

py::array_t<double> fromPoints(vf::Span<const vf::Vec2> points)
{
  const std::vector<py::ssize_t> shape{static_cast<py::ssize_t>(points.size()), 2};
  py::array_t<double> out(shape);
  auto view = out.mutable_unchecked<2>();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto row = static_cast<py::ssize_t>(i);
    view(row, 0) = points[i].x();
    view(row, 1) = points[i].y();
  }
  return out;
}

py::array_t<double> fromPoses(vf::Span<const vf::Pose2> poses)
{
  const std::vector<py::ssize_t> shape{static_cast<py::ssize_t>(poses.size()), 3};
  py::array_t<double> out(shape);
  auto view = out.mutable_unchecked<2>();
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto row = static_cast<py::ssize_t>(i);
    view(row, 0) = poses[i].p.x();
    view(row, 1) = poses[i].p.y();
    view(row, 2) = poses[i].yaw;
  }
  return out;
}

py::array_t<std::uint8_t> fromMask(vf::Span<const std::uint8_t> mask, int width, int height)
{
  const std::vector<py::ssize_t> shape{height, width};
  py::array_t<std::uint8_t> out(shape);
  std::copy(mask.begin(), mask.end(), out.mutable_data());
  return out;
}

py::array_t<double> mapPoints(
  const DoubleArray & points, const std::function<vf::Vec2(const vf::Vec2 &)> & f)
{
  if (points.ndim() == 1 && points.shape(0) == 2) {
    const vf::Vec2 r = f(vf::Vec2(points.at(0), points.at(1)));
    py::array_t<double> out(2);
    out.mutable_at(0) = r.x();
    out.mutable_at(1) = r.y();
    return out;
  }
  std::vector<vf::Vec2> pts = toPoints(points, "points");
  for (vf::Vec2 & p : pts) {
    p = f(p);
  }
  return fromPoints(pts);
}

py::array_t<double> mapValues(const DoubleArray & values, const std::function<double(double)> & f)
{
  const std::vector<py::ssize_t> shape(values.shape(), values.shape() + values.ndim());
  py::array_t<double> out(shape);
  const double * in = values.data();
  double * res = out.mutable_data();
  for (py::ssize_t i = 0; i < values.size(); ++i) {
    res[i] = f(in[i]);
  }
  return out;
}

namespace
{

// Exception types, created once and kept for the life of the process (no static py::object, so
// nothing is destroyed after the interpreter has finalised).
PyObject * g_config_error = nullptr;
PyObject * g_curve_error = nullptr;

void registerExceptions(py::module_ & m)
{
  g_config_error = PyErr_NewExceptionWithDoc(
    "vectorfield.ConfigError",
    "Invalid configuration or input detected while constructing a core object (the C++ "
    "vectorfield_core::ConfigError). A ValueError.",
    PyExc_ValueError, nullptr);
  if (g_config_error == nullptr) {
    throw py::error_already_set();
  }
  g_curve_error = PyErr_NewExceptionWithDoc(
    "vectorfield.CurveError",
    "A curve rejected by make_curve() (vectorfield_core::CurveError). `code` is the FollowCurve "
    "result-code name: 'INVALID_CURVE' or 'UNSUPPORTED' (an open time-varying curve).",
    g_config_error, nullptr);
  if (g_curve_error == nullptr) {
    throw py::error_already_set();
  }
  m.attr("ConfigError") = py::handle(g_config_error);
  m.attr("CurveError") = py::handle(g_curve_error);

  // Translators run most recent first; this one handles both types, the derived one first.
  // pybind11's translator type takes the exception_ptr by value.
  py::register_exception_translator([](std::exception_ptr pointer) {  // NOLINT(performance-*)
    try {
      if (pointer) {
        std::rethrow_exception(pointer);
      }
    } catch (const vf::CurveError & e) {
      py::object error = py::reinterpret_borrow<py::object>(g_curve_error)(e.what());
      error.attr("code") = py::str(vf::toString(e.code()));
      PyErr_SetObject(g_curve_error, error.ptr());
    } catch (const vf::ConfigError & e) {
      PyErr_SetString(g_config_error, e.what());
    }
  });
}

}  // namespace

}  // namespace vectorfield_py

PYBIND11_MODULE(_vectorfield, m)
{
  namespace vp = vectorfield_py;
  m.doc() =
    "Compiled bindings of vectorfield_core (pybind11, NumPy I/O). Import the vectorfield "
    "package instead.";
  m.attr("__version__") = vp::py::str(vectorfield_core::version());
  vp::registerExceptions(m);
  vp::py::module_ sim = m.def_submodule(
    "sim",
    "Kinematic plant integration (exact arcs, slip) and the closed-loop simulator of a Tracker.");
  vp::bindTypes(m);
  vp::bindConfig(m);
  vp::bindCurves(m);
  vp::bindField(m);
  vp::bindMotion(m, sim);
  vp::bindObstacles(m);
  vp::bindTracker(m, sim);
}
