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
/// @brief Curve specifications (the core mirror of `vectorfield_msgs/msg/Curve.msg`) and the
/// `makeCurve` factory, the single validation point of curves (math_spec §2.6–§2.8).
///
/// Implemented in src/curve_spec.cpp. Internal, non-installed
/// pieces, in namespace `vectorfield_core::detail`:
/// - src/builtin_curves.{hpp,cpp}: the seven built-in shapes of math_spec §2.7;
/// - src/expression_curve.{hpp,cpp}: ExpressionCurve on the vendored muParser (PIMPL,
///   non-copyable, `evalNoThrow()`), never exposed in public headers.

#ifndef VECTORFIELD_CORE__CURVE_SPEC_HPP_
#define VECTORFIELD_CORE__CURVE_SPEC_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/errors.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

/// @brief Maximum length of each expression string.
inline constexpr std::size_t kMaxExpressionLength = 1024;
/// @brief Maximum number of declared expression parameters.
inline constexpr std::size_t kMaxExpressionParameters = 32;

/// @brief A curve description, field for field `vectorfield_msgs/msg/Curve.msg` (without the
/// header: the frame is the caller's, and `t0` carries the stamp as seconds).
struct CurveSpec
{
  /// Curve type; values equal Curve.msg `POLYLINE`, `EXPRESSION`, `BUILTIN`.
  enum class Type : std::uint8_t
  {
    /// Vertex list.
    kPolyline = 0,
    /// muParser expressions in s, t and the named parameters.
    kExpression = 1,
    /// One of the built-in shapes (builtinShapes()).
    kBuiltin = 2,
  };

  /// Curve type.
  Type type{Type::kPolyline};
  /// Closed curve (ignored for BUILTIN: all built-ins are closed).
  bool closed{false};
  /// POLYLINE vertices (curve frame; Curve.msg `points` with z dropped).
  std::vector<Vec2> points;
  /// EXPRESSION: x(s, t, params).
  std::string x_expression;
  /// EXPRESSION: y(s, t, params).
  std::string y_expression;
  /// EXPRESSION: required domain start; BUILTIN: both 0 = natural domain \f$[0, 2\pi)\f$.
  double s_min{0.0};
  /// EXPRESSION: required domain end; BUILTIN: both 0 = natural domain.
  double s_max{0.0};
  /// BUILTIN shape name (builtinShapes()).
  std::string builtin_shape;
  /// EXPRESSION/BUILTIN parameter names (same length as parameter_values).
  std::vector<std::string> parameter_names;
  /// EXPRESSION/BUILTIN parameter values.
  std::vector<double> parameter_values;
  /// POLYLINE resampling spacing (m); <= 0: CurveOptions::default_sample_spacing.
  double sample_spacing{0.0};
  /// POLYLINE corner fillet deviation \f$\delta\f$ (m); <= 0: no corner smoothing.
  double max_corner_deviation{0.0};
  /// Time origin \f$t_0\f$ of \f$r(s, t)\f$ (s); regularity is checked at \f$t_0\f$.
  double t0{0.0};

  /// @brief Field-wise equality (the follower keeps TrackerState across a preemption only for an
  /// identical spec).
  bool operator==(const CurveSpec & other) const;
  /// @brief Negation of operator==.
  bool operator!=(const CurveSpec & other) const { return !(*this == other); }
};

/// @brief A rejected curve, with the follower result code it maps to (FollowCurve.action).
class CurveError : public ConfigError
{
public:
  /// Error class.
  enum class Code : std::uint8_t
  {
    /// INVALID_CURVE: fewer than 2 points, non-finite values, s_min >= s_max, parse error, size
    /// limits, unknown built-in or parameter, irregular, declared closed but not closing,
    /// invalid CurveOptions.
    kInvalidCurve = 0,
    /// UNSUPPORTED: an open time-varying curve.
    kUnsupported = 1,
  };

  /// @brief An error of the given class with a human-readable message.
  CurveError(Code code, const std::string & what) : ConfigError(what), code_(code) {}

  /// @brief The error class.
  Code code() const noexcept { return code_; }

private:
  Code code_;
};

/// @brief "INVALID_CURVE" or "UNSUPPORTED" (the FollowCurve.action result-code names).
const char * toString(CurveError::Code code) noexcept;

/// @brief Builds and validates a curve. The single validation point of curves.
///
/// - POLYLINE: conditioning of math_spec §2.8 in this order: dedupe (1 mm), corner fillet when
///   `max_corner_deviation` > 0, resample at `sample_spacing` (or the default), then
///   N <= `max_curve_points`; a PolylineCurve with SearchPolicy::hinted(options).
/// - EXPRESSION: muParser hardening (parameters bound as constants, s and t as
///   variables, the default functions except `rnd`, one result per expression, <= 1024 chars,
///   <= 32 identifier-validated names that are not `s`, `t` or a function name); time-varying
///   iff an expression uses `t`.
/// - BUILTIN: the shapes of math_spec §2.7 with their parameter rules (lissajous needs
///   \f$\gcd(f_x, f_y) = 1\f$; regular_polygon needs corner_radius <= radius·cos(π/sides);
///   superellipse needs n >= 2).
/// - Parametric curves pass the regularity and closure checks of math_spec §2.6 on the M
///   global-stage samples at t0.
/// - `!closed && time-varying` → CurveError::Code::kUnsupported.
/// @param spec The curve description.
/// @param options Curve-intake options (validated with validateCurveOptions()).
/// @return A shared, immutable curve whose spec() returns a copy of `spec`.
/// @throws CurveError with kInvalidCurve or kUnsupported.
std::shared_ptr<const Curve> makeCurve(const CurveSpec & spec, const CurveOptions & options);

/// @brief One parameter of a built-in shape and its default.
struct BuiltinParameterInfo
{
  /// Parameter name, e.g. "a".
  const char * name;
  /// Default value.
  double default_value;
};

/// @brief Description of a built-in shape, for documentation tables and the Python tools.
struct BuiltinShapeInfo
{
  /// Shape name used in `builtin_shape`, e.g. "lemniscate_gerono".
  const char * name;
  /// Parameters with defaults (pose parameters `cx`, `cy`, `yaw` included).
  Span<const BuiltinParameterInfo> parameters;
  /// One-line formula in the curve frame (math_spec §2.7).
  const char * formula;
};

/// @brief The built-in shapes of math_spec §2.7, in the order of Curve.msg's comment.
Span<const BuiltinShapeInfo> builtinShapes() noexcept;

}  // namespace vectorfield_core

#endif  // VECTORFIELD_CORE__CURVE_SPEC_HPP_
