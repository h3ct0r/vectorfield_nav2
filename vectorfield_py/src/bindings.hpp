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

// Shared helpers of the `vectorfield._vectorfield` extension (not installed).
//
// Conventions of the Python API:
// - snake_case names; enums use the upper-case value names of the parameter strings or of
//   FieldState.msg; parameter enums also convert from their parameter strings ("rational").
// - Vec2 is a float64 array of shape (2,); point lists are (N, 2) arrays; pose lists (N, 3)
//   arrays of [x, y, yaw]; grids (height, width) arrays indexed [iy, ix]. Every array returned is
//   a copy, so it never aliases C++ memory.
// - Plain structs (configs, samples, states) are mutable attribute objects with keyword
//   constructors, a generated __repr__, copy() and a `_fields` tuple listing their attributes.
//   A nested struct attribute is a live reference into its parent (`config.field.desired_speed =
//   0.3` edits `config`); an attribute that holds a list (`CurveSpec.parameter_names`) is a copy.
// - Objects that the core holds by non-owning pointer (curves, grids, distance transforms) are
//   kept alive by shared ownership in the input wrappers declared here.
// - Only the pybind11 2.9 API subset is used (jammy ships 2.9.1).

#ifndef VECTORFIELD_PY__BINDINGS_HPP_
#define VECTORFIELD_PY__BINDINGS_HPP_

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "vectorfield_core/vectorfield_core.hpp"

namespace vectorfield_py
{

namespace py = pybind11;
namespace vf = vectorfield_core;

/// Any array-like converted to a C-contiguous float64 array.
using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
/// Any array-like converted to a C-contiguous uint8 array (nonzero = set).
using MaskArray = py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast>;

/// @name Registration, one function per source file, called in this order by bindings.cpp
/// (pybind11 needs a type registered before it appears as a default argument).
/// @{
void bindTypes(py::module_ & m);
void bindConfig(py::module_ & m);
void bindCurves(py::module_ & m);
void bindField(py::module_ & m);
void bindMotion(py::module_ & m, py::module_ & sim);
void bindObstacles(py::module_ & m);
void bindTracker(py::module_ & m, py::module_ & sim);
/// @}

/// @name NumPy conversions
/// @{
/// (N, 2) points from an array; an empty array gives no points. Throws ValueError otherwise.
std::vector<vf::Vec2> toPoints(const DoubleArray & array, const char * what);
/// (N, 2) float64 array.
py::array_t<double> fromPoints(vf::Span<const vf::Vec2> points);
/// (N, 3) float64 array of [x, y, yaw].
py::array_t<double> fromPoses(vf::Span<const vf::Pose2> poses);
/// (height, width) uint8 array of a row-major mask.
py::array_t<std::uint8_t> fromMask(vf::Span<const std::uint8_t> mask, int width, int height);
/// Applies `f` to one point (shape (2,)) or to every row of an (N, 2) array.
py::array_t<double> mapPoints(
  const DoubleArray & points, const std::function<vf::Vec2(const vf::Vec2 &)> & f);
/// Applies `f` to every element of an array, keeping its shape.
py::array_t<double> mapValues(const DoubleArray & values, const std::function<double(double)> & f);
/// @}

/// @brief Calls Python from inside a `noexcept` core function (probes, rollout policies).
///
/// The first exception is kept, later calls return the fallback without calling Python, and
/// rethrow() raises it once the core function has returned, so nothing unwinds through C++ code
/// that cannot throw.
class CallbackGuard
{
public:
  template <typename R, typename F>
  R call(F && f, R fallback) noexcept
  {
    if (error_) {
      return fallback;
    }
    try {
      return f();
    } catch (...) {
      error_ = std::current_exception();
      return fallback;
    }
  }

  void rethrow() const
  {
    if (error_) {
      std::rethrow_exception(error_);
    }
  }

private:
  std::exception_ptr error_;
};

/// @brief Binds a plain struct: attributes, keyword constructor, __repr__, copy and `_fields`.
///
/// Vec2 members become (2,) array properties (copies); std::vector<Vec2> members (N, 2) array
/// properties; every other member uses def_readwrite (nested structs by reference).
template <typename T, typename... Options>
class StructBinder
{
public:
  using Class = py::class_<T, Options...>;
  using Setter = std::function<void(T &, py::handle)>;

  StructBinder(py::handle scope, const char * name, const char * doc)
  : cls_(scope, name, doc),
    setters_(std::make_shared<std::vector<std::pair<std::string, Setter>>>())
  {
  }

  /// Adds the attribute `name` for `member`.
  template <typename D>
  StructBinder & field(const char * name, D T::* member, const char * doc)
  {
    if constexpr (std::is_same_v<D, vf::Vec2>) {
      cls_.def_property(
        name, [member](const T & self) -> vf::Vec2 { return self.*member; },
        [member](T & self, const vf::Vec2 & value) { self.*member = value; }, doc);
      setters_->emplace_back(
        name, [member](T & self, py::handle value) { self.*member = value.cast<vf::Vec2>(); });
    } else if constexpr (std::is_same_v<D, std::vector<vf::Vec2>>) {
      cls_.def_property(
        name, [member](const T & self) { return fromPoints(self.*member); },
        [member, name](T & self, const DoubleArray & value) {
          self.*member = toPoints(value, name);
        },
        doc);
      setters_->emplace_back(name, [member, name](T & self, py::handle value) {
        self.*member = toPoints(value.cast<DoubleArray>(), name);
      });
    } else {
      cls_.def_readwrite(name, member, doc);
      setters_->emplace_back(
        name, [member](T & self, py::handle value) { self.*member = value.cast<D>(); });
    }
    names_.emplace_back(name);
    return *this;
  }

  /// The class, for methods that are not attributes.
  Class & cls() { return cls_; }

  /// Adds the keyword constructor, `_fields`, __repr__, __copy__, __deepcopy__ and copy().
  Class & finish()
  {
    auto setters = setters_;
    cls_.def(
      py::init([setters](const py::kwargs & kwargs) {
        T value{};
        for (const auto & item : kwargs) {
          const auto key = item.first.cast<std::string>();
          const auto it = std::find_if(
            setters->begin(), setters->end(), [&key](const auto & s) { return s.first == key; });
          if (it == setters->end()) {
            throw py::type_error("unexpected keyword argument '" + key + "'");
          }
          it->second(value, item.second);
        }
        return value;
      }),
      "Default values, then the given attributes (keyword arguments only).");
    py::tuple fields(names_.size());
    for (std::size_t i = 0; i < names_.size(); ++i) {
      fields[i] = py::str(names_[i]);
    }
    cls_.attr("_fields") = fields;
    cls_.def("__repr__", [names = names_](py::handle self) {
      std::string text = py::str(self.attr("__class__").attr("__name__"));
      text += "(";
      for (std::size_t i = 0; i < names.size(); ++i) {
        text += (i == 0 ? "" : ", ") + names[i] + "=";
        text += py::repr(self.attr(names[i].c_str())).template cast<std::string>();
      }
      return text + ")";
    });
    cls_.def("copy", [](const T & self) { return T(self); }, "A deep copy.");
    cls_.def("__copy__", [](const T & self) { return T(self); });
    cls_.def("__deepcopy__", [](const T & self, const py::dict &) { return T(self); });
    return cls_;
  }

private:
  Class cls_;
  std::vector<std::string> names_;
  std::shared_ptr<std::vector<std::pair<std::string, Setter>>> setters_;
};

/// @brief Python view of ObstacleMap: owns its grid and distance transform.
///
/// map() brings the transform up to date with the grid first (EuclideanDistanceTransform::update
/// rebuilds only when the grid's version changed), so the core always sees EDT_all of the grid.
struct ObstacleMapBox
{
  std::shared_ptr<vf::OccupancyGrid> grid;
  std::shared_ptr<vf::EuclideanDistanceTransform> edt;
  vf::Isometry2 T_grid_eval;

  vf::ObstacleMap map() const
  {
    edt->update(*grid);
    return vf::ObstacleMap{grid.get(), edt.get(), T_grid_eval};
  }
};

/// @brief Python view of TrackerInput: owns the curve and the obstacles it points to.
struct TrackerInputBox
{
  /// Every field except `curve` and `obstacles`, which resolved() fills.
  vf::TrackerInput input;
  std::shared_ptr<vf::Curve> curve;
  std::optional<ObstacleMapBox> obstacles;

  /// The core input, valid while this object lives and is not modified.
  const vf::TrackerInput & resolved()
  {
    input.curve = curve.get();
    input.obstacles.reset();
    if (obstacles) {
      input.obstacles = obstacles->map();
    }
    return input;
  }
};

/// @brief Python view of TrackerOutput: `predicted` copied out of the StepScratch.
struct TrackerOutputBox
{
  vf::TrackerOutput output;
  std::vector<vf::Pose2> predicted;

  TrackerOutputBox() = default;
  explicit TrackerOutputBox(const vf::TrackerOutput & out)
  : output(out), predicted(out.predicted.begin(), out.predicted.end())
  {
    output.predicted = {};
  }
};

/// @brief Python Tracker: the core Tracker plus the StepScratch used when none is passed.
struct TrackerBox
{
  vf::Tracker tracker;
  vf::StepScratch scratch;
};

}  // namespace vectorfield_py

#endif  // VECTORFIELD_PY__BINDINGS_HPP_
