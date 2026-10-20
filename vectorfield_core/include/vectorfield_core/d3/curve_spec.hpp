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

/// @file
/// @brief 3D core (experimental): curve specifications, makeCurve3() and the 3D built-in shapes
/// (math_spec §14.1).

#ifndef VECTORFIELD_CORE__D3__CURVE_SPEC_HPP_
#define VECTORFIELD_CORE__D3__CURVE_SPEC_HPP_

#include <memory>
#include <string>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/d3/curve.hpp"
#include "vectorfield_core/d3/types.hpp"
#include "vectorfield_core/span.hpp"

namespace vectorfield_core::d3
{

/// @brief A 3D curve description: the planar CurveSpec with z added.
///
/// There is no ROS message for it (Curve.msg is planar); the Python tools read and write it as
/// YAML or CSV.
struct CurveSpec3
{
  /// Curve type (the planar enum: POLYLINE, EXPRESSION, BUILTIN).
  using Type = CurveSpec::Type;
  /// Curve type.
  Type type{Type::kPolyline};
  /// Closed curve (POLYLINE and EXPRESSION; built-ins decide themselves: helix is open).
  bool closed{false};
  /// POLYLINE vertices.
  std::vector<Vec3> points;
  /// EXPRESSION: x(s, t, params).
  std::string x_expression;
  /// EXPRESSION: y(s, t, params).
  std::string y_expression;
  /// EXPRESSION: z(s, t, params).
  std::string z_expression;
  /// EXPRESSION: domain start; BUILTIN: both 0 = the shape's natural domain.
  double s_min{0.0};
  /// EXPRESSION: domain end; BUILTIN: both 0 = the shape's natural domain.
  double s_max{0.0};
  /// BUILTIN shape name (builtinShapes3()).
  std::string builtin_shape;
  /// EXPRESSION/BUILTIN parameter names.
  std::vector<std::string> parameter_names;
  /// EXPRESSION/BUILTIN parameter values.
  std::vector<double> parameter_values;
  /// Time origin \f$t_0\f$ (s): expressions see \f$t - t_0\f$; regularity is checked at \f$t_0\f$.
  double t0{0.0};

  /// @brief Field-wise equality.
  bool operator==(const CurveSpec3 & other) const;
  /// @brief Negation of operator==.
  bool operator!=(const CurveSpec3 & other) const { return !(*this == other); }
};

/// @brief Builds and validates a 3D curve, the single validation point of 3D curves.
///
/// - POLYLINE: PolylineCurve3 (dedupe, at most `max_curve_points` vertices).
/// - EXPRESSION: three expressions with the hardening of the planar makeCurve() (shared code):
///   parameters as constants, `s` and `t` variables, default functions without `rnd`, one result,
///   at most 1024 characters, no `&&` or `||`, at most 32 identifier-validated parameters.
/// - BUILTIN: builtinShapes3() with their parameter rules.
/// - Parametric curves pass the regularity and closure checks at t0; open and time-varying →
///   CurveError::Code::kUnsupported.
/// @throws CurveError with kInvalidCurve or kUnsupported.
std::shared_ptr<const Curve3> makeCurve3(const CurveSpec3 & spec, const CurveOptions & options);

/// @brief The 3D built-in shapes: helix, circle, ellipse, viviani, lemniscate3d, torus_knot.
///
/// Every shape takes the pose parameters `cx`, `cy`, `cz`, `yaw`, `pitch`, `roll`
/// (\f$x \mapsto R_z R_y R_x\,x + c\f$, rotationZYX()). The formula strings are in the shape's
/// own frame.
Span<const BuiltinShapeInfo> builtinShapes3() noexcept;

}  // namespace vectorfield_core::d3

#endif  // VECTORFIELD_CORE__D3__CURVE_SPEC_HPP_
