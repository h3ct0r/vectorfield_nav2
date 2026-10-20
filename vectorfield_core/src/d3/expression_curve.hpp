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
/// @brief Internal: ExpressionCurve3 and the 3D built-in shapes (not installed).

#ifndef VECTORFIELD_CORE__DETAIL__D3_EXPRESSION_CURVE_HPP_
#define VECTORFIELD_CORE__DETAIL__D3_EXPRESSION_CURVE_HPP_

#include <memory>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/d3/curve_spec.hpp"
#include "vectorfield_core/d3/parametric_curve.hpp"
#include "vectorfield_core/d3/types.hpp"

namespace vectorfield_core::d3::detail
{

/// @brief \f$r(s, t) = (x, y, z)(s, t)\f$ from three hardened muParser expressions (the planar
/// ExpressionCurve's rules, shared code in src/expression_parser.hpp). Non-copyable; every
/// evaluation is serialised by a mutex and returns NaN on failure.
class ExpressionCurve3 final : public ParametricCurve3
{
public:
  /// @throws CurveError (kInvalidCurve; kUnsupported for an open time-varying curve).
  ExpressionCurve3(const CurveSpec3 & spec, const CurveOptions & options);
  ~ExpressionCurve3() override;
  ExpressionCurve3(const ExpressionCurve3 &) = delete;
  ExpressionCurve3 & operator=(const ExpressionCurve3 &) = delete;
  ExpressionCurve3(ExpressionCurve3 &&) = delete;
  ExpressionCurve3 & operator=(ExpressionCurve3 &&) = delete;

  Vec3 r(double u, double t) const noexcept override;
  Vec3 drdt(double u, double t) const noexcept override;

private:
  struct Impl;
  ExpressionCurve3(
    std::unique_ptr<Impl> impl, const CurveSpec3 & spec, const CurveOptions & options);
  static std::unique_ptr<Impl> parse(const CurveSpec3 & spec);
  double domainParameter(double u) const noexcept;
  Vec3 evalNoThrow(double s, double t) const noexcept;
  std::unique_ptr<Impl> impl_;
};

/// @brief Builds a 3D built-in shape (validated parameters, natural domain, pose applied).
/// @throws CurveError (kInvalidCurve).
std::unique_ptr<ParametricCurve3> makeBuiltin3(
  const CurveSpec3 & spec, const CurveOptions & options);

}  // namespace vectorfield_core::d3::detail

#endif  // VECTORFIELD_CORE__DETAIL__D3_EXPRESSION_CURVE_HPP_
