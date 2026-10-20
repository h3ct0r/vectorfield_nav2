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

// makeCurve3(), CurveSpec3 equality and the 3D built-in shapes (math_spec §14.1).

#include "vectorfield_core/d3/curve_spec.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "expression_curve.hpp"
#include "vectorfield_core/d3/parametric_curve.hpp"
#include "vectorfield_core/d3/polyline_curve.hpp"

namespace vectorfield_core::d3
{

bool CurveSpec3::operator==(const CurveSpec3 & other) const
{
  return type == other.type && closed == other.closed && points == other.points &&
         x_expression == other.x_expression && y_expression == other.y_expression &&
         z_expression == other.z_expression && s_min == other.s_min && s_max == other.s_max &&
         builtin_shape == other.builtin_shape && parameter_names == other.parameter_names &&
         parameter_values == other.parameter_values && t0 == other.t0;
}

namespace
{

std::string fmt(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(10) << value;
  return stream.str();
}

[[noreturn]] void reject(const std::string & message)
{
  throw CurveError(CurveError::Code::kInvalidCurve, message);
}

enum class Shape : std::uint8_t
{
  kHelix,
  kCircle,
  kEllipse,
  kViviani,
  kLemniscate,
  kTorusKnot,
};

// Pose parameters shared by every shape.
#define VF_D3_POSE \
  {"cx", 0.0}, {"cy", 0.0}, {"cz", 0.0}, {"yaw", 0.0}, {"pitch", 0.0}, {"roll", 0.0}

constexpr BuiltinParameterInfo kHelixParams[] = {
  {"radius", 1.0}, {"lead", 0.5}, {"turns", 3.0}, VF_D3_POSE};
constexpr BuiltinParameterInfo kCircleParams[] = {{"radius", 1.0}, VF_D3_POSE};
constexpr BuiltinParameterInfo kEllipseParams[] = {{"a", 2.0}, {"b", 1.0}, VF_D3_POSE};
constexpr BuiltinParameterInfo kVivianiParams[] = {{"radius", 2.0}, VF_D3_POSE};
constexpr BuiltinParameterInfo kLemniscateParams[] = {{"a", 2.0}, {"h", 0.5}, VF_D3_POSE};
constexpr BuiltinParameterInfo kTorusKnotParams[] = {
  {"p", 2.0}, {"q", 3.0}, {"major_radius", 2.0}, {"minor_radius", 0.5}, VF_D3_POSE};

#undef VF_D3_POSE

const BuiltinShapeInfo kShapes[] = {
  {"helix", Span<const BuiltinParameterInfo>(kHelixParams),
   "(radius cos s, radius sin s, lead s / 2pi), s in [0, 2pi turns] (open)"},
  {"circle", Span<const BuiltinParameterInfo>(kCircleParams),
   "(radius cos s, radius sin s, 0), s in [0, 2pi)"},
  {"ellipse", Span<const BuiltinParameterInfo>(kEllipseParams),
   "(a cos s, b sin s, 0), s in [0, 2pi)"},
  {"viviani", Span<const BuiltinParameterInfo>(kVivianiParams),
   "(radius/2 (1 + cos s), radius/2 sin s, radius sin(s/2)), s in [0, 4pi)"},
  {"lemniscate3d", Span<const BuiltinParameterInfo>(kLemniscateParams),
   "(a cos s, a sin s cos s, h sin s), s in [0, 2pi)"},
  {"torus_knot", Span<const BuiltinParameterInfo>(kTorusKnotParams),
   "((R + r cos qs) cos ps, (R + r cos qs) sin ps, r sin qs), s in [0, 2pi), gcd(p, q) = 1"},
};

class BuiltinCurve3 final : public ParametricCurve3
{
public:
  BuiltinCurve3(
    Shape shape, double u_max, bool closed, const double * params, const Placement3 & placement,
    const CurveOptions & options)
  : ParametricCurve3(0.0, u_max, closed, false, options), shape_(shape), placement_(placement)
  {
    for (int i = 0; i < 4; ++i) {
      p_[i] = params[i];
    }
    initialize(0.0);
  }

  Vec3 r(double u, double /*t*/) const noexcept override { return placement_.apply(local(u)); }
  Vec3 drdu(double u, double /*t*/) const noexcept override
  {
    return placement_.rotate(localDerivative(u));
  }

private:
  Vec3 local(double u) const noexcept
  {
    switch (shape_) {
      case Shape::kHelix:
        return Vec3(p_[0] * std::cos(u), p_[0] * std::sin(u), p_[1] * u / kTwoPi);
      case Shape::kCircle:
        return Vec3(p_[0] * std::cos(u), p_[0] * std::sin(u), 0.0);
      case Shape::kEllipse:
        return Vec3(p_[0] * std::cos(u), p_[1] * std::sin(u), 0.0);
      case Shape::kViviani:
        return Vec3(
          0.5 * p_[0] * (1.0 + std::cos(u)), 0.5 * p_[0] * std::sin(u), p_[0] * std::sin(0.5 * u));
      case Shape::kLemniscate:
        return Vec3(p_[0] * std::cos(u), p_[0] * std::sin(u) * std::cos(u), p_[1] * std::sin(u));
      case Shape::kTorusKnot: {
        const double rho = p_[2] + p_[3] * std::cos(p_[1] * u);
        return Vec3(
          rho * std::cos(p_[0] * u), rho * std::sin(p_[0] * u), p_[3] * std::sin(p_[1] * u));
      }
    }
    return Vec3::Constant(kNaN);
  }

  Vec3 localDerivative(double u) const noexcept
  {
    switch (shape_) {
      case Shape::kHelix:
        return Vec3(-p_[0] * std::sin(u), p_[0] * std::cos(u), p_[1] / kTwoPi);
      case Shape::kCircle:
        return Vec3(-p_[0] * std::sin(u), p_[0] * std::cos(u), 0.0);
      case Shape::kEllipse:
        return Vec3(-p_[0] * std::sin(u), p_[1] * std::cos(u), 0.0);
      case Shape::kViviani:
        return Vec3(
          -0.5 * p_[0] * std::sin(u), 0.5 * p_[0] * std::cos(u), 0.5 * p_[0] * std::cos(0.5 * u));
      case Shape::kLemniscate:
        return Vec3(-p_[0] * std::sin(u), p_[0] * std::cos(2.0 * u), p_[1] * std::cos(u));
      case Shape::kTorusKnot: {
        const double p = p_[0];
        const double q = p_[1];
        const double rho = p_[2] + p_[3] * std::cos(q * u);
        const double drho = -p_[3] * q * std::sin(q * u);
        return Vec3(
          drho * std::cos(p * u) - rho * p * std::sin(p * u),
          drho * std::sin(p * u) + rho * p * std::cos(p * u), p_[3] * q * std::cos(q * u));
      }
    }
    return Vec3::Constant(kNaN);
  }

  Shape shape_;
  double p_[4]{};
  Placement3 placement_;
};

bool isPositiveInteger(double value) noexcept
{
  return value >= 1.0 && value <= 1e6 && std::floor(value) == value;
}

}  // namespace

Span<const BuiltinShapeInfo> builtinShapes3() noexcept
{
  return Span<const BuiltinShapeInfo>(kShapes);
}

namespace detail
{

std::unique_ptr<ParametricCurve3> makeBuiltin3(
  const CurveSpec3 & spec, const CurveOptions & options)
{
  const Span<const BuiltinShapeInfo> shapes = builtinShapes3();
  std::size_t index = shapes.size();
  for (std::size_t i = 0; i < shapes.size(); ++i) {
    if (spec.builtin_shape == shapes[i].name) {
      index = i;
    }
  }
  if (index == shapes.size()) {
    std::string names;
    for (const BuiltinShapeInfo & info : shapes) {
      names += names.empty() ? info.name : std::string(", ") + info.name;
    }
    reject("unknown 3D built-in shape '" + spec.builtin_shape + "' (known: " + names + ")");
  }
  const BuiltinShapeInfo & info = shapes[index];
  if (spec.parameter_names.size() != spec.parameter_values.size()) {
    reject("parameter_names and parameter_values differ in length");
  }
  if (spec.s_min != 0.0 || spec.s_max != 0.0) {
    reject(
      std::string("built-in '") + info.name +
      "' uses its natural domain: leave s_min and s_max at 0 (helix: set turns)");
  }
  std::vector<double> values;
  values.reserve(info.parameters.size());
  for (const BuiltinParameterInfo & parameter : info.parameters) {
    values.push_back(parameter.default_value);
  }
  for (std::size_t i = 0; i < spec.parameter_names.size(); ++i) {
    std::size_t j = 0;
    while (j < info.parameters.size() && spec.parameter_names[i] != info.parameters[j].name) {
      ++j;
    }
    if (j == info.parameters.size()) {
      reject(
        std::string("unknown parameter '") + spec.parameter_names[i] + "' of '" + info.name + "'");
    }
    if (!std::isfinite(spec.parameter_values[i])) {
      reject("parameter '" + spec.parameter_names[i] + "' is not finite");
    }
    values[j] = spec.parameter_values[i];
  }
  // The six pose parameters are the last ones of every shape.
  const std::size_t pose = values.size() - 6;
  Placement3 placement;
  placement.c = Vec3(values[pose], values[pose + 1], values[pose + 2]);
  placement.R = rotationZYX(values[pose + 3], values[pose + 4], values[pose + 5]);
  const auto positive = [&](std::size_t k) {
    if (!(values[k] > 0.0)) {
      reject(
        std::string(info.name) + ": " + info.parameters[k].name + " = " + fmt(values[k]) +
        " must be positive");
    }
  };
  double u_max = kTwoPi;
  bool closed = true;
  Shape shape = Shape::kCircle;
  const std::string name = info.name;
  if (name == "helix") {
    shape = Shape::kHelix;
    positive(0);
    positive(2);
    if (!(std::abs(values[1]) > 0.0)) {
      reject("helix: lead must be non-zero (a zero lead retraces a circle)");
    }
    u_max = kTwoPi * values[2];
    closed = false;
  } else if (name == "circle") {
    positive(0);
  } else if (name == "ellipse") {
    shape = Shape::kEllipse;
    positive(0);
    positive(1);
  } else if (name == "viviani") {
    shape = Shape::kViviani;
    positive(0);
    u_max = 2.0 * kTwoPi;
  } else if (name == "lemniscate3d") {
    shape = Shape::kLemniscate;
    positive(0);
    if (!(values[1] >= 0.0)) {
      reject("lemniscate3d: h = " + fmt(values[1]) + " must be >= 0");
    }
  } else {
    shape = Shape::kTorusKnot;
    if (!isPositiveInteger(values[0]) || !isPositiveInteger(values[1])) {
      reject("torus_knot: p and q must be positive integers");
    }
    if (std::gcd(static_cast<long>(values[0]), static_cast<long>(values[1])) != 1) {
      reject("torus_knot: gcd(p, q) must be 1 (otherwise the curve retraces itself)");
    }
    positive(3);
    if (!(values[2] > values[3])) {
      reject("torus_knot: major_radius must exceed minor_radius");
    }
  }
  return std::make_unique<BuiltinCurve3>(shape, u_max, closed, values.data(), placement, options);
}

}  // namespace detail

std::shared_ptr<const Curve3> makeCurve3(const CurveSpec3 & spec, const CurveOptions & options)
{
  const ValidationReport report = validateCurveOptions(options);
  if (!report.ok()) {
    reject("invalid CurveOptions: " + report.summary());
  }
  auto stored = std::make_shared<const CurveSpec3>(spec);
  switch (spec.type) {
    case CurveSpec3::Type::kPolyline: {
      auto curve = std::make_shared<PolylineCurve3>(spec.points, spec.closed, options);
      curve->setSpec(std::move(stored));
      return curve;
    }
    case CurveSpec3::Type::kExpression: {
      auto curve = std::make_shared<detail::ExpressionCurve3>(spec, options);
      curve->setSpec(std::move(stored));
      return curve;
    }
    case CurveSpec3::Type::kBuiltin: {
      std::shared_ptr<ParametricCurve3> curve = detail::makeBuiltin3(spec, options);
      curve->setSpec(std::move(stored));
      return curve;
    }
  }
  reject("unknown curve type");
}

}  // namespace vectorfield_core::d3
