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

// CurveSpec and makeCurve, the single validation point of curves
// (math_spec §2.6–§2.8).

#include "vectorfield_core/curve_spec.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "builtin_curves.hpp"
#include "expression_curve.hpp"
#include "vectorfield_core/conditioning.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/polyline_curve.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

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

// Total length of a polyline (closed: with the closing segment).
double polylineLength(const std::vector<Vec2> & vertices, bool closed) noexcept
{
  double total = 0.0;
  for (std::size_t k = 1; k < vertices.size(); ++k) {
    total += (vertices[k] - vertices[k - 1]).norm();
  }
  if (closed && vertices.size() > 2) {
    total += (vertices.front() - vertices.back()).norm();
  }
  return total;
}

// POLYLINE: dedupe (1 mm) → corner fillet (δ > 0) → resample, then N <= max_curve_points
// (math_spec §2.8), on a PolylineCurve with the follower's Hinted policy.
std::shared_ptr<PolylineCurve> makePolyline(const CurveSpec & spec, const CurveOptions & options)
{
  if (spec.points.size() < 2) {
    reject("POLYLINE needs at least 2 points, got " + std::to_string(spec.points.size()));
  }
  for (std::size_t k = 0; k < spec.points.size(); ++k) {
    if (!isFinite(spec.points[k])) {
      reject("POLYLINE point " + std::to_string(k) + " is not finite");
    }
  }
  if (!std::isfinite(spec.sample_spacing) || !std::isfinite(spec.max_corner_deviation)) {
    reject(
      "POLYLINE sample_spacing = " + fmt(spec.sample_spacing) +
      " and max_corner_deviation = " + fmt(spec.max_corner_deviation) + " must be finite");
  }
  const double spacing =
    spec.sample_spacing > 0.0 ? spec.sample_spacing : options.default_sample_spacing;
  const auto max_points = static_cast<std::size_t>(options.max_curve_points);

  std::vector<Vec2> deduped;
  dedupe(Span<const Vec2>(spec.points), spec.closed, &deduped);
  const std::size_t minimum = spec.closed ? 3 : 2;
  if (deduped.size() < minimum) {
    reject(
      "POLYLINE needs at least " + std::to_string(minimum) + " distinct points" +
      (spec.closed ? " (closed)" : "") + " after dedupe, got " + std::to_string(deduped.size()));
  }
  // Bound the work before resampling: every segment is split to at most `spacing`, so the
  // result has at least length/spacing points.
  const double total = polylineLength(deduped, spec.closed);
  if (!std::isfinite(total) || total / spacing > static_cast<double>(max_points)) {
    reject(
      "POLYLINE of length " + fmt(total) + " m at sample spacing " + fmt(spacing) +
      " m needs more than max_curve_points = " + std::to_string(max_points) + " points");
  }

  std::vector<Vec2> filleted;
  if (spec.max_corner_deviation > 0.0) {
    filletCorners(
      Span<const Vec2>(deduped), spec.closed, spec.max_corner_deviation, spacing, &filleted);
  } else {
    filleted = deduped;
  }
  std::vector<Vec2> resampled;
  resample(Span<const Vec2>(filleted), spec.closed, spacing, &resampled);
  if (resampled.size() > max_points) {
    reject(
      "POLYLINE has " + std::to_string(resampled.size()) +
      " points after conditioning; max_curve_points = " + std::to_string(max_points));
  }

  auto curve = std::make_shared<PolylineCurve>(SearchPolicy::hinted(options));
  switch (curve->assign(Span<const Vec2>(resampled), spec.closed)) {
    case AssignStatus::kOk:
      break;
    case AssignStatus::kTooFew:
      reject("POLYLINE has too few distinct points after conditioning");
    case AssignStatus::kNonFinite:
      reject("POLYLINE has a non-finite point after conditioning");
  }
  return curve;
}

}  // namespace

bool CurveSpec::operator==(const CurveSpec & other) const
{
  return type == other.type && closed == other.closed && points == other.points &&
         x_expression == other.x_expression && y_expression == other.y_expression &&
         s_min == other.s_min && s_max == other.s_max && builtin_shape == other.builtin_shape &&
         parameter_names == other.parameter_names && parameter_values == other.parameter_values &&
         sample_spacing == other.sample_spacing &&
         max_corner_deviation == other.max_corner_deviation && t0 == other.t0;
}

const char * toString(CurveError::Code code) noexcept
{
  switch (code) {
    case CurveError::Code::kInvalidCurve:
      return "INVALID_CURVE";
    case CurveError::Code::kUnsupported:
      return "UNSUPPORTED";
  }
  return "INVALID_CURVE";
}

std::shared_ptr<const Curve> makeCurve(const CurveSpec & spec, const CurveOptions & options)
{
  const ValidationReport report = validateCurveOptions(options);
  if (!report.ok()) {
    std::string message = "invalid curve options:";
    for (const ValidationIssue & issue : report.errors) {
      message += " " + issue.message + ";";
    }
    reject(message);
  }
  auto stored = std::make_shared<const CurveSpec>(spec);
  try {
    switch (spec.type) {
      case CurveSpec::Type::kPolyline: {
        std::shared_ptr<PolylineCurve> curve = makePolyline(spec, options);
        curve->setSpec(stored);
        return curve;
      }
      case CurveSpec::Type::kExpression: {
        auto curve = std::make_shared<detail::ExpressionCurve>(spec, options);
        curve->setSpec(stored);
        return curve;
      }
      case CurveSpec::Type::kBuiltin: {
        std::shared_ptr<ParametricCurve> curve = detail::makeBuiltinCurve(spec, options);
        curve->setSpec(stored);
        return curve;
      }
    }
  } catch (const CurveError &) {
    throw;
  } catch (const ConfigError & error) {
    reject(error.what());
  }
  reject("unknown curve type " + std::to_string(static_cast<int>(spec.type)));
}

}  // namespace vectorfield_core
