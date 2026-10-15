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
/// @brief Internal: the built-in shapes of math_spec §2.7 (not installed).

#ifndef VECTORFIELD_CORE__DETAIL__BUILTIN_CURVES_HPP_
#define VECTORFIELD_CORE__DETAIL__BUILTIN_CURVES_HPP_

#include <memory>
#include <string>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/parametric_curve.hpp"

namespace vectorfield_core::detail
{

/// @brief Relative tolerance on a BUILTIN domain that is not the natural one: s_max − s_min must
/// equal \f$2\pi\f$ within this fraction (one period; only the start may move).
inline constexpr double kBuiltinPeriodTolerance = 1e-6;

/// @brief Largest accepted integer parameter (lissajous `fx`, `fy`; regular_polygon `sides`).
inline constexpr double kBuiltinMaxInteger = 1e6;

/// @brief The shape named `name` in builtinShapes(), or null.
const BuiltinShapeInfo * findBuiltinShape(const std::string & name) noexcept;

/// @brief Builds a BUILTIN curve (math_spec §2.7): the shape, its parameters (defaults from
/// builtinShapes(); unknown, duplicate, non-finite or out-of-range values are rejected), the
/// domain (s_min = s_max = 0: \f$[0, 2\pi)\f$; otherwise one period starting at s_min) and the
/// regularity checks of math_spec §2.6 at `spec.t0`. All built-ins are closed and static;
/// `spec.closed` is ignored.
/// @throws CurveError (kInvalidCurve).
std::unique_ptr<ParametricCurve> makeBuiltinCurve(
  const CurveSpec & spec, const CurveOptions & options);

}  // namespace vectorfield_core::detail

#endif  // VECTORFIELD_CORE__DETAIL__BUILTIN_CURVES_HPP_
