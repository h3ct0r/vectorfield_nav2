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

// Planar types (Pose2, Twist2, Isometry2), angle helpers, Phase and the status bits.

#include <cmath>
#include <cstdint>
#include <string>

#include "bindings.hpp"

namespace vectorfield_py
{

namespace
{

std::string number(double value) { return py::repr(py::float_(value)).cast<std::string>(); }

// A sequence of exactly `n` numbers.
std::vector<double> numbers(const py::sequence & values, std::size_t n, const char * what)
{
  if (values.size() != n || py::isinstance<py::str>(values)) {
    throw py::value_error(std::string(what) + " needs " + std::to_string(n) + " numbers");
  }
  std::vector<double> out;
  for (const auto & v : values) {
    out.push_back(v.cast<double>());
  }
  return out;
}

void bindPose(py::module_ & m)
{
  py::class_<vf::Pose2> cls(
    m, "Pose2",
    "Pose of base_link in a planar frame: position p (m) and yaw (rad, not required to be "
    "wrapped). Tuples, lists and arrays [x, y, yaw] convert implicitly.");
  cls
    .def(
      py::init([](double x, double y, double yaw) { return vf::Pose2{vf::Vec2(x, y), yaw}; }),
      py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("yaw") = 0.0)
    .def(
      py::init([](const py::sequence & xyyaw) {
        const auto v = numbers(xyyaw, 3, "Pose2");
        return vf::Pose2{vf::Vec2(v[0], v[1]), v[2]};
      }),
      py::arg("xyyaw"))
    .def_property(
      "p", [](const vf::Pose2 & s) -> vf::Vec2 { return s.p; },
      [](vf::Pose2 & s, const vf::Vec2 & p) { s.p = p; }, "Position (2,) (m).")
    .def_property(
      "x", [](const vf::Pose2 & s) { return s.p.x(); },
      [](vf::Pose2 & s, double v) { s.p.x() = v; }, "Position x (m).")
    .def_property(
      "y", [](const vf::Pose2 & s) { return s.p.y(); },
      [](vf::Pose2 & s, double v) { s.p.y() = v; }, "Position y (m).")
    .def_readwrite("yaw", &vf::Pose2::yaw, "Yaw (rad).")
    .def(
      "to_array",
      [](const vf::Pose2 & s) {
        py::array_t<double> out(3);
        out.mutable_at(0) = s.p.x();
        out.mutable_at(1) = s.p.y();
        out.mutable_at(2) = s.yaw;
        return out;
      },
      "[x, y, yaw] as a (3,) array.")
    .def(
      "__iter__",
      [](const vf::Pose2 & s) { return py::iter(py::make_tuple(s.p.x(), s.p.y(), s.yaw)); })
    .def(
      "__repr__",
      [](const vf::Pose2 & s) {
        return "Pose2(x=" + number(s.p.x()) + ", y=" + number(s.p.y()) + ", yaw=" + number(s.yaw) +
               ")";
      })
    .def("copy", [](const vf::Pose2 & s) { return s; })
    .def("__copy__", [](const vf::Pose2 & s) { return s; })
    .def("__deepcopy__", [](const vf::Pose2 & s, const py::dict &) { return s; });
  py::implicitly_convertible<py::tuple, vf::Pose2>();
  py::implicitly_convertible<py::list, vf::Pose2>();
  py::implicitly_convertible<py::array, vf::Pose2>();
}

void bindTwist(py::module_ & m)
{
  py::class_<vf::Twist2> cls(
    m, "Twist2",
    "Planar body twist at base_link: vx, vy (m/s) and wz (rad/s). Tuples, lists and arrays "
    "[vx, vy, wz] convert implicitly.");
  cls
    .def(
      py::init([](double vx, double vy, double wz) { return vf::Twist2{vx, vy, wz}; }),
      py::arg("vx") = 0.0, py::arg("vy") = 0.0, py::arg("wz") = 0.0)
    .def(
      py::init([](const py::sequence & values) {
        const auto v = numbers(values, 3, "Twist2");
        return vf::Twist2{v[0], v[1], v[2]};
      }),
      py::arg("vxvywz"))
    .def_readwrite("vx", &vf::Twist2::vx, "Longitudinal velocity (m/s).")
    .def_readwrite("vy", &vf::Twist2::vy, "Lateral velocity (m/s).")
    .def_readwrite("wz", &vf::Twist2::wz, "Yaw rate (rad/s).")
    .def(
      "to_array",
      [](const vf::Twist2 & s) {
        py::array_t<double> out(3);
        out.mutable_at(0) = s.vx;
        out.mutable_at(1) = s.vy;
        out.mutable_at(2) = s.wz;
        return out;
      },
      "[vx, vy, wz] as a (3,) array.")
    .def(
      "__iter__", [](const vf::Twist2 & s) { return py::iter(py::make_tuple(s.vx, s.vy, s.wz)); })
    .def(
      "__repr__",
      [](const vf::Twist2 & s) {
        return "Twist2(vx=" + number(s.vx) + ", vy=" + number(s.vy) + ", wz=" + number(s.wz) + ")";
      })
    .def("copy", [](const vf::Twist2 & s) { return s; })
    .def("__copy__", [](const vf::Twist2 & s) { return s; })
    .def("__deepcopy__", [](const vf::Twist2 & s, const py::dict &) { return s; });
  py::implicitly_convertible<py::tuple, vf::Twist2>();
  py::implicitly_convertible<py::list, vf::Twist2>();
  py::implicitly_convertible<py::array, vf::Twist2>();
}

void bindIsometry(py::module_ & m)
{
  using vf::Isometry2;
  py::class_<Isometry2> cls(
    m, "Isometry2",
    "Rigid planar transform: rotation by yaw, then translation. T_a_b maps frame-b coordinates "
    "into frame a (x_a = R x_b + t).");
  cls
    .def(
      py::init([](double yaw, const vf::Vec2 & t) { return Isometry2(yaw, t); }),
      py::arg("yaw") = 0.0, py::arg("translation") = vf::Vec2(vf::Vec2::Zero()))
    .def_static("identity", &Isometry2::identity, "The identity transform.")
    .def_static(
      "from_pose", &Isometry2::fromPose, py::arg("pose"),
      "The transform whose child frame is the given pose (body coordinates to parent).")
    .def("to_pose", &Isometry2::toPose, "The pose of the child frame in the parent frame.")
    .def_property_readonly("yaw", &Isometry2::yaw, "Rotation angle in (-pi, pi].")
    .def_property_readonly(
      "translation", [](const Isometry2 & s) -> vf::Vec2 { return s.translation(); },
      "Translation (2,).")
    .def_property_readonly("rotation", &Isometry2::rotation, "Rotation matrix (2, 2).")
    .def(
      "apply",
      [](const Isometry2 & s, const DoubleArray & x) {
        return mapPoints(x, [&s](const vf::Vec2 & p) { return s.apply(p); });
      },
      py::arg("points"), "R x + t for one point (2,) or each row of an (N, 2) array.")
    .def(
      "apply_inverse",
      [](const Isometry2 & s, const DoubleArray & x) {
        return mapPoints(x, [&s](const vf::Vec2 & p) { return s.applyInverse(p); });
      },
      py::arg("points"), "R^T (x - t) for one point or each row of an (N, 2) array.")
    .def(
      "rotate",
      [](const Isometry2 & s, const DoubleArray & v) {
        return mapPoints(v, [&s](const vf::Vec2 & p) { return s.rotate(p); });
      },
      py::arg("vectors"), "R v (free vectors: tangents, normals, velocities).")
    .def(
      "rotate_inverse",
      [](const Isometry2 & s, const DoubleArray & v) {
        return mapPoints(v, [&s](const vf::Vec2 & p) { return s.rotateInverse(p); });
      },
      py::arg("vectors"), "R^T v.")
    .def(
      "apply_pose", [](const Isometry2 & s, const vf::Pose2 & p) { return s.apply(p); },
      py::arg("pose"), "Maps a pose (yaw wrapped).")
    .def(
      "apply_inverse_pose",
      [](const Isometry2 & s, const vf::Pose2 & p) { return s.applyInverse(p); }, py::arg("pose"),
      "Maps a pose back.")
    .def("inverse", &Isometry2::inverse, "The inverse transform.")
    .def(
      "__mul__", [](const Isometry2 & a, const Isometry2 & b) { return a * b; },
      "Composition: (a * b).apply(x) == a.apply(b.apply(x)).")
    .def(
      "__repr__",
      [](const Isometry2 & s) {
        return "Isometry2(yaw=" + number(s.yaw()) + ", translation=[" +
               number(s.translation().x()) + ", " + number(s.translation().y()) + "])";
      })
    .def("copy", [](const Isometry2 & s) { return s; })
    .def("__copy__", [](const Isometry2 & s) { return s; })
    .def("__deepcopy__", [](const Isometry2 & s, const py::dict &) { return s; });
}

}  // namespace

void bindTypes(py::module_ & m)
{
  bindPose(m);
  bindTwist(m);
  bindIsometry(m);

  m.def(
    "wrap_angle", [](double a) { return vf::wrapAngle(a); }, py::arg("angle"),
    "Wraps angles into (-pi, pi] (scalar or array).");
  m.def(
    "wrap_angle", [](const DoubleArray & a) { return mapValues(a, &vf::wrapAngle); },
    py::arg("angle"));

  py::enum_<vf::Phase>(
    m, "Phase", "Tracker phase; values equal FieldState.msg PHASE_* (math_spec section 9.2).")
    .value("ROTATE_TO_FIELD", vf::Phase::kRotateToField, "Rotate in place toward the field.")
    .value("FOLLOW", vf::Phase::kFollow, "Follow the field (the default phase).")
    .value("FINAL_APPROACH", vf::Phase::kFinalApproach, "Terminal approach of an open curve.")
    .value("ROTATE_TO_HEADING", vf::Phase::kRotateToHeading, "Rotate to the goal yaw.")
    .value("STOPPED", vf::Phase::kStopped, "Decelerate to zero and stay there.")
    .def_property_readonly(
      "is_rotation", &vf::isRotationPhase, "ROTATE_TO_FIELD or ROTATE_TO_HEADING.")
    .def_property_readonly("is_translation", &vf::isTranslationPhase, "FOLLOW or FINAL_APPROACH.");

  // Status bits as (name, value) pairs; vectorfield/__init__.py builds the Status IntFlag.
  py::list bits;
  for (std::uint32_t bit = 1U; bit <= vf::Status::kAll; bit <<= 1U) {
    bits.append(py::make_tuple(vf::statusFlagName(bit), bit));
  }
  m.attr("_STATUS_BITS") = bits;
}

}  // namespace vectorfield_py
