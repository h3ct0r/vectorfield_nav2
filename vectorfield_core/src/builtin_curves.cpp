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

// The built-in shapes of math_spec §2.7, with analytic derivatives. Each shape is written in the
// curve frame and mapped by [cx, cy] + R(yaw)(·).

#include "builtin_curves.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/parametric_curve.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_core/types.hpp"

namespace vectorfield_core
{

namespace
{

constexpr BuiltinParameterInfo kEllipseParameters[] = {
  {"a", 1.0}, {"b", 1.0}, {"cx", 0.0}, {"cy", 0.0}, {"yaw", 0.0}};
constexpr BuiltinParameterInfo kBernoulliParameters[] = {
  {"a", 1.0}, {"cx", 0.0}, {"cy", 0.0}, {"yaw", 0.0}};
constexpr BuiltinParameterInfo kGeronoParameters[] = {
  {"a", 1.0}, {"b", 0.5}, {"cx", 0.0}, {"cy", 0.0}, {"yaw", 0.0}};
constexpr BuiltinParameterInfo kSuperellipseParameters[] = {{"a", 1.0},  {"b", 1.0},  {"n", 4.0},
                                                            {"cx", 0.0}, {"cy", 0.0}, {"yaw", 0.0}};
constexpr BuiltinParameterInfo kRoundedRectangleParameters[] = {
  {"width", 2.0}, {"height", 1.0}, {"radius", 0.2}, {"cx", 0.0}, {"cy", 0.0}, {"yaw", 0.0}};
constexpr BuiltinParameterInfo kLissajousParameters[] = {
  {"ax", 1.0},        {"ay", 1.0}, {"fx", 1.0}, {"fy", 2.0},
  {"phase", kHalfPi}, {"cx", 0.0}, {"cy", 0.0}, {"yaw", 0.0}};
constexpr BuiltinParameterInfo kRegularPolygonParameters[] = {
  {"radius", 1.0}, {"sides", 6.0}, {"corner_radius", 0.1}, {"cx", 0.0}, {"cy", 0.0}, {"yaw", 0.0}};

// In the order of Curve.msg's builtin_shape comment.
constexpr BuiltinShapeInfo kShapes[] = {
  {"ellipse", Span<const BuiltinParameterInfo>(kEllipseParameters), "(a cos s, b sin s)"},
  {"lemniscate_bernoulli", Span<const BuiltinParameterInfo>(kBernoulliParameters),
   "a (cos s, sin s cos s) / (1 + sin^2 s)"},
  {"lemniscate_gerono", Span<const BuiltinParameterInfo>(kGeronoParameters), "(a cos s, b sin 2s)"},
  {"superellipse", Span<const BuiltinParameterInfo>(kSuperellipseParameters),
   "rho(s) (cos s, sin s), rho = (|cos s / a|^n + |sin s / b|^n)^(-1/n), n >= 2"},
  {"rounded_rectangle", Span<const BuiltinParameterInfo>(kRoundedRectangleParameters),
   "width x height rectangle with corner radius (0 < radius <= min(width, height)/2), by arc "
   "length s L/(2 pi), counter-clockwise from (width/2, 0)"},
  {"lissajous", Span<const BuiltinParameterInfo>(kLissajousParameters),
   "(ax sin(fx s + phase), ay sin(fy s)), fx and fy coprime positive integers"},
  {"regular_polygon", Span<const BuiltinParameterInfo>(kRegularPolygonParameters),
   "sides-gon of circumradius radius, vertices at pi/n + 2 pi k/n, corners filleted with "
   "corner_radius (0 < corner_radius <= radius cos(pi/n)), by arc length s L/(2 pi), "
   "counter-clockwise from the +x side midpoint"},
};

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

// Pose of a shape in the curve frame.
struct Pose
{
  double cx{0.0};
  double cy{0.0};
  double yaw{0.0};
};

// Parameter values of one shape: the defaults, overridden by the spec.
class ShapeParameters
{
public:
  ShapeParameters(const BuiltinShapeInfo & shape, const CurveSpec & spec) : shape_(shape)
  {
    const std::size_t count = shape.parameters.size();
    for (std::size_t j = 0; j < count; ++j) {
      values_[j] = shape.parameters[j].default_value;
    }
    if (spec.parameter_names.size() != spec.parameter_values.size()) {
      reject(
        "builtin '" + std::string(shape.name) +
        "': " + std::to_string(spec.parameter_names.size()) + " parameter names but " +
        std::to_string(spec.parameter_values.size()) + " values");
    }
    std::array<bool, kMaxParameters> seen{};
    for (std::size_t i = 0; i < spec.parameter_names.size(); ++i) {
      const std::string & name = spec.parameter_names[i];
      const std::size_t j = indexOf(name);
      if (j == count) {
        std::string known;
        for (std::size_t m = 0; m < count; ++m) {
          known += (m == 0 ? "" : ", ") + std::string(shape.parameters[m].name);
        }
        reject(
          "builtin '" + std::string(shape.name) + "': unknown parameter '" + name +
          "' (known: " + known + ")");
      }
      if (seen[j]) {
        reject("builtin '" + std::string(shape.name) + "': duplicate parameter '" + name + "'");
      }
      seen[j] = true;
      const double value = spec.parameter_values[i];
      if (!std::isfinite(value)) {
        reject(
          "builtin '" + std::string(shape.name) + "': parameter '" + name + "' = " + fmt(value) +
          " is not finite");
      }
      values_[j] = value;
    }
  }

  double get(const char * name) const noexcept { return values_[indexOf(name)]; }

  double positive(const char * name) const
  {
    const double value = get(name);
    if (!(value > 0.0)) {
      reject(
        "builtin '" + std::string(shape_.name) + "': parameter '" + name + "' = " + fmt(value) +
        " must be > 0");
    }
    return value;
  }

  // A positive integer parameter in [minimum, kBuiltinMaxInteger].
  long long integer(const char * name, long long minimum) const
  {
    const double value = get(name);
    const double rounded = std::round(value);
    if (
      std::abs(value - rounded) > 1e-9 || rounded < static_cast<double>(minimum) ||
      rounded > detail::kBuiltinMaxInteger) {
      reject(
        "builtin '" + std::string(shape_.name) + "': parameter '" + name + "' = " + fmt(value) +
        " must be an integer in [" + std::to_string(minimum) + ", " +
        fmt(detail::kBuiltinMaxInteger) + "]");
    }
    return static_cast<long long>(rounded);
  }

  Pose pose() const noexcept { return Pose{get("cx"), get("cy"), get("yaw")}; }

private:
  static constexpr std::size_t kMaxParameters = 8;

  std::size_t indexOf(const std::string & name) const noexcept
  {
    const std::size_t count = shape_.parameters.size();
    for (std::size_t j = 0; j < count; ++j) {
      if (name == shape_.parameters[j].name) {
        return j;
      }
    }
    return count;
  }

  const BuiltinShapeInfo & shape_;
  std::array<double, kMaxParameters> values_{};
};

// s mapped to [0, 2π) (all built-ins have period 2π).
double phase2Pi(double u) noexcept
{
  double v = std::fmod(u, kTwoPi);
  if (v < 0.0) {
    v += kTwoPi;
  }
  return v < kTwoPi ? v : 0.0;
}

// A closed, static built-in: r(u) = c + R(yaw)·local(u), with analytic derivatives.
class BuiltinCurve : public ParametricCurve
{
public:
  Vec2 r(double u, double /*t*/) const noexcept final
  {
    const Vec2 p = local(u);
    return Vec2(
      center_x_ + cos_yaw_ * p.x() - sin_yaw_ * p.y(),
      center_y_ + sin_yaw_ * p.x() + cos_yaw_ * p.y());
  }

  Vec2 drdu(double u, double /*t*/) const noexcept final { return rotate(localDerivative(u)); }

  Vec2 drdt(double /*u*/, double /*t*/) const noexcept final { return Vec2::Zero(); }

protected:
  BuiltinCurve(std::pair<double, double> domain, const Pose & pose, const CurveOptions & options)
  : ParametricCurve(domain.first, domain.second, true, false, options),
    center_x_(pose.cx),
    center_y_(pose.cy),
    cos_yaw_(std::cos(pose.yaw)),
    sin_yaw_(std::sin(pose.yaw))
  {
  }

  // The shape in the curve frame before the pose.
  virtual Vec2 local(double u) const noexcept = 0;
  virtual Vec2 localDerivative(double u) const noexcept = 0;

private:
  Vec2 rotate(const Vec2 & v) const noexcept
  {
    return Vec2(cos_yaw_ * v.x() - sin_yaw_ * v.y(), sin_yaw_ * v.x() + cos_yaw_ * v.y());
  }

  double center_x_;
  double center_y_;
  double cos_yaw_;
  double sin_yaw_;
};

// (a cos s, b sin s).
class EllipseCurve final : public BuiltinCurve
{
public:
  EllipseCurve(
    double a, double b, std::pair<double, double> domain, const Pose & pose,
    const CurveOptions & options, double t0)
  : BuiltinCurve(domain, pose, options), a_(a), b_(b)
  {
    initialize(t0);
  }

private:
  Vec2 local(double u) const noexcept override { return Vec2(a_ * std::cos(u), b_ * std::sin(u)); }
  Vec2 localDerivative(double u) const noexcept override
  {
    return Vec2(-a_ * std::sin(u), b_ * std::cos(u));
  }

  double a_;
  double b_;
};

// a (cos s, sin s cos s) / (1 + sin² s); speed a / sqrt(1 + sin² s).
class LemniscateBernoulliCurve final : public BuiltinCurve
{
public:
  LemniscateBernoulliCurve(
    double a, std::pair<double, double> domain, const Pose & pose, const CurveOptions & options,
    double t0)
  : BuiltinCurve(domain, pose, options), a_(a)
  {
    initialize(t0);
  }

private:
  Vec2 local(double u) const noexcept override
  {
    const double s = std::sin(u);
    const double c = std::cos(u);
    const double d = 1.0 + s * s;
    return Vec2(a_ * c / d, a_ * s * c / d);
  }
  Vec2 localDerivative(double u) const noexcept override
  {
    const double s = std::sin(u);
    const double c = std::cos(u);
    const double d = 1.0 + s * s;
    const double d2 = d * d;
    return Vec2(
      -a_ * s * (3.0 - s * s) / d2, a_ * ((c * c - s * s) * d - 2.0 * s * s * c * c) / d2);
  }

  double a_;
};

// (a cos s, b sin 2s).
class LemniscateGeronoCurve final : public BuiltinCurve
{
public:
  LemniscateGeronoCurve(
    double a, double b, std::pair<double, double> domain, const Pose & pose,
    const CurveOptions & options, double t0)
  : BuiltinCurve(domain, pose, options), a_(a), b_(b)
  {
    initialize(t0);
  }

private:
  Vec2 local(double u) const noexcept override
  {
    return Vec2(a_ * std::cos(u), b_ * std::sin(2.0 * u));
  }
  Vec2 localDerivative(double u) const noexcept override
  {
    return Vec2(-a_ * std::sin(u), 2.0 * b_ * std::cos(2.0 * u));
  }

  double a_;
  double b_;
};

// ρ(s)(cos s, sin s), ρ = (|cos s/a|ⁿ + |sin s/b|ⁿ)^(−1/n). Evaluated with the larger of
// |cos s|/a and |sin s|/b factored out, so no power underflows or overflows.
class SuperellipseCurve final : public BuiltinCurve
{
public:
  SuperellipseCurve(
    double a, double b, double n, std::pair<double, double> domain, const Pose & pose,
    const CurveOptions & options, double t0)
  : BuiltinCurve(domain, pose, options), a_(a), b_(b), n_(n)
  {
    initialize(t0);
  }

private:
  struct Radius
  {
    double rho;
    double drho;
  };

  Radius radius(double u) const noexcept
  {
    const double c = std::cos(u);
    const double s = std::sin(u);
    const double p = std::abs(c) / a_;
    const double q = std::abs(s) / b_;
    const double m = std::max(p, q);
    const double pn = std::pow(p / m, n_);
    const double qn = std::pow(q / m, n_);
    const double sum = pn + qn;
    const double rho = 1.0 / (m * std::pow(sum, 1.0 / n_));
    // ρ' = −m⁻² S^(−1/n−1) [ (p/m)^(n−1) sgn(c)(−sin s)/a + (q/m)^(n−1) sgn(s) cos s / b ].
    const double pn1 = std::pow(p / m, n_ - 1.0);
    const double qn1 = std::pow(q / m, n_ - 1.0);
    const double bracket = pn1 * sgn(c) * (-s) / a_ + qn1 * sgn(s) * c / b_;
    const double drho = -bracket / (m * m) * std::pow(sum, -1.0 / n_ - 1.0);
    return Radius{rho, drho};
  }

  Vec2 local(double u) const noexcept override
  {
    const double rho = radius(u).rho;
    return Vec2(rho * std::cos(u), rho * std::sin(u));
  }
  Vec2 localDerivative(double u) const noexcept override
  {
    const Radius value = radius(u);
    const double c = std::cos(u);
    const double s = std::sin(u);
    return Vec2(value.drho * c - value.rho * s, value.drho * s + value.rho * c);
  }

  double a_;
  double b_;
  double n_;
};

// (ax sin(fx s + phase), ay sin(fy s)).
class LissajousCurve final : public BuiltinCurve
{
public:
  LissajousCurve(
    double ax, double ay, double fx, double fy, double phase, std::pair<double, double> domain,
    const Pose & pose, const CurveOptions & options, double t0)
  : BuiltinCurve(domain, pose, options), ax_(ax), ay_(ay), fx_(fx), fy_(fy), phase_(phase)
  {
    initialize(t0);
  }

private:
  Vec2 local(double u) const noexcept override
  {
    return Vec2(ax_ * std::sin(fx_ * u + phase_), ay_ * std::sin(fy_ * u));
  }
  Vec2 localDerivative(double u) const noexcept override
  {
    return Vec2(ax_ * fx_ * std::cos(fx_ * u + phase_), ay_ * fy_ * std::cos(fy_ * u));
  }

  double ax_;
  double ay_;
  double fx_;
  double fy_;
  double phase_;
};

// A piece of an arc-length shape: a straight segment or a counter-clockwise circular arc.
struct ShapePiece
{
  // Arc-length station of the piece's start.
  double start{0.0};
  double length{0.0};
  bool arc{false};
  // Line: start point. Arc: centre.
  Vec2 origin{Vec2::Zero()};
  // Line: unit direction.
  Vec2 direction{Vec2::UnitX()};
  // Arc: polar angle of the start point about the centre.
  double theta0{0.0};
};

// Rounded rectangle, parametrised by arc length ς = s·L/(2π) from (W/2, 0), counter-clockwise.
class RoundedRectangleCurve final : public BuiltinCurve
{
public:
  RoundedRectangleCurve(
    double width, double height, double radius, std::pair<double, double> domain, const Pose & pose,
    const CurveOptions & options, double t0)
  : BuiltinCurve(domain, pose, options), radius_(radius)
  {
    const double w = width;
    const double h = height;
    const double rho = radius;
    const double half_side = std::max(0.0, 0.5 * h - rho);
    const double top = std::max(0.0, w - 2.0 * rho);
    const double side = std::max(0.0, h - 2.0 * rho);
    const double quarter = kHalfPi * rho;
    // From (W/2, 0): up the right side, the four corner arcs and sides, and back up.
    const std::array<ShapePiece, 9> pieces{{
      {0.0, half_side, false, Vec2(0.5 * w, 0.0), Vec2(0.0, 1.0), 0.0},
      {0.0, quarter, true, Vec2(0.5 * w - rho, 0.5 * h - rho), Vec2::UnitX(), 0.0},
      {0.0, top, false, Vec2(0.5 * w - rho, 0.5 * h), Vec2(-1.0, 0.0), 0.0},
      {0.0, quarter, true, Vec2(-0.5 * w + rho, 0.5 * h - rho), Vec2::UnitX(), kHalfPi},
      {0.0, side, false, Vec2(-0.5 * w, 0.5 * h - rho), Vec2(0.0, -1.0), 0.0},
      {0.0, quarter, true, Vec2(-0.5 * w + rho, -0.5 * h + rho), Vec2::UnitX(), kPi},
      {0.0, top, false, Vec2(-0.5 * w + rho, -0.5 * h), Vec2(1.0, 0.0), 0.0},
      {0.0, quarter, true, Vec2(0.5 * w - rho, -0.5 * h + rho), Vec2::UnitX(), 1.5 * kPi},
      {0.0, half_side, false, Vec2(0.5 * w, -0.5 * h + rho), Vec2(0.0, 1.0), 0.0},
    }};
    pieces_ = pieces;
    double station = 0.0;
    for (ShapePiece & piece : pieces_) {
      piece.start = station;
      station += piece.length;
    }
    perimeter_ = station;
    scale_ = perimeter_ / kTwoPi;
    initialize(t0);
  }

private:
  const ShapePiece & pieceAt(double sigma) const noexcept
  {
    const ShapePiece * last = &pieces_[0];
    for (const ShapePiece & piece : pieces_) {
      if (piece.length > 0.0) {
        last = &piece;
        if (sigma < piece.start + piece.length) {
          return piece;
        }
      }
    }
    return *last;
  }

  double sigmaOf(double u) const noexcept
  {
    const double sigma = phase2Pi(u) * scale_;
    return sigma < perimeter_ ? sigma : 0.0;
  }

  Vec2 local(double u) const noexcept override
  {
    const double sigma = sigmaOf(u);
    const ShapePiece & piece = pieceAt(sigma);
    const double d = sigma - piece.start;
    if (piece.arc) {
      const double theta = piece.theta0 + d / radius_;
      return piece.origin + radius_ * Vec2(std::cos(theta), std::sin(theta));
    }
    return piece.origin + d * piece.direction;
  }

  Vec2 localDerivative(double u) const noexcept override
  {
    const double sigma = sigmaOf(u);
    const ShapePiece & piece = pieceAt(sigma);
    if (piece.arc) {
      const double theta = piece.theta0 + (sigma - piece.start) / radius_;
      return scale_ * Vec2(-std::sin(theta), std::cos(theta));
    }
    return scale_ * piece.direction;
  }

  double radius_;
  std::array<ShapePiece, 9> pieces_{};
  double perimeter_{0.0};
  double scale_{1.0};
};

// Regular polygon with filleted corners, parametrised by arc length from the midpoint of the side
// crossing +x, counter-clockwise (math_spec §2.7).
class RegularPolygonCurve final : public BuiltinCurve
{
public:
  RegularPolygonCurve(
    double circumradius, long long sides, double corner_radius, std::pair<double, double> domain,
    const Pose & pose, const CurveOptions & options, double t0)
  : BuiltinCurve(domain, pose, options),
    sides_(static_cast<double>(sides)),
    corner_radius_(corner_radius)
  {
    const double alpha = kPi / sides_;
    apothem_ = circumradius * std::cos(alpha);
    const double side = 2.0 * circumradius * std::sin(alpha);
    const double tangent_length = corner_radius * std::tan(alpha);
    straight_ = std::max(0.0, side - 2.0 * tangent_length);
    half_straight_ = 0.5 * straight_;
    arc_ = corner_radius * 2.0 * alpha;
    unit_ = straight_ + arc_;
    perimeter_ = sides_ * unit_;
    centre_radius_ = circumradius - corner_radius / std::cos(alpha);
    alpha_ = alpha;
    scale_ = perimeter_ / kTwoPi;
    initialize(t0);
  }

private:
  struct Location
  {
    double phi;
    double d;
  };

  // Side index k (normal angle φ_k = 2πk/n) and distance d from the start of its straight part.
  Location locate(double u) const noexcept
  {
    double sigma = phase2Pi(u) * scale_;
    if (!(sigma < perimeter_)) {
      sigma = 0.0;
    }
    double q = sigma + half_straight_;
    if (q >= perimeter_) {
      q -= perimeter_;
    }
    const double k = std::min(sides_ - 1.0, std::floor(q / unit_));
    return Location{kTwoPi * k / sides_, q - k * unit_};
  }

  Vec2 local(double u) const noexcept override
  {
    const Location at = locate(u);
    if (at.d < straight_) {
      const Vec2 midpoint = apothem_ * Vec2(std::cos(at.phi), std::sin(at.phi));
      const Vec2 direction(-std::sin(at.phi), std::cos(at.phi));
      return midpoint + (at.d - half_straight_) * direction;
    }
    const double theta = at.phi + (at.d - straight_) / corner_radius_;
    const Vec2 centre = centre_radius_ * Vec2(std::cos(at.phi + alpha_), std::sin(at.phi + alpha_));
    return centre + corner_radius_ * Vec2(std::cos(theta), std::sin(theta));
  }

  Vec2 localDerivative(double u) const noexcept override
  {
    const Location at = locate(u);
    if (at.d < straight_) {
      return scale_ * Vec2(-std::sin(at.phi), std::cos(at.phi));
    }
    const double theta = at.phi + (at.d - straight_) / corner_radius_;
    return scale_ * Vec2(-std::sin(theta), std::cos(theta));
  }

  double sides_;
  double corner_radius_;
  double apothem_{0.0};
  double straight_{0.0};
  double half_straight_{0.0};
  double arc_{0.0};
  double unit_{0.0};
  double perimeter_{0.0};
  double centre_radius_{0.0};
  double alpha_{0.0};
  double scale_{1.0};
};

// s_min = s_max = 0: [0, 2π); otherwise exactly one period starting at s_min.
std::pair<double, double> builtinDomain(const CurveSpec & spec)
{
  if (spec.s_min == 0.0 && spec.s_max == 0.0) {
    return {0.0, kTwoPi};
  }
  if (!std::isfinite(spec.s_min) || !std::isfinite(spec.s_max) || !(spec.s_max > spec.s_min)) {
    reject(
      "builtin '" + spec.builtin_shape + "': domain [" + fmt(spec.s_min) + ", " + fmt(spec.s_max) +
      "] must be [0, 0] (natural domain [0, 2 pi)) or finite with s_min < s_max");
  }
  const double span = spec.s_max - spec.s_min;
  if (std::abs(span - kTwoPi) > detail::kBuiltinPeriodTolerance * kTwoPi) {
    reject(
      "builtin '" + spec.builtin_shape + "': domain [" + fmt(spec.s_min) + ", " + fmt(spec.s_max) +
      "] must span exactly one period (2 pi); use [0, 0] for [0, 2 pi)");
  }
  return {spec.s_min, spec.s_max};
}

}  // namespace

Span<const BuiltinShapeInfo> builtinShapes() noexcept
{
  return Span<const BuiltinShapeInfo>(kShapes);
}

namespace detail
{

const BuiltinShapeInfo * findBuiltinShape(const std::string & name) noexcept
{
  for (const BuiltinShapeInfo & shape : kShapes) {
    if (name == shape.name) {
      return &shape;
    }
  }
  return nullptr;
}

std::unique_ptr<ParametricCurve> makeBuiltinCurve(
  const CurveSpec & spec, const CurveOptions & options)
{
  const BuiltinShapeInfo * shape = findBuiltinShape(spec.builtin_shape);
  if (shape == nullptr) {
    std::string known;
    for (const BuiltinShapeInfo & candidate : kShapes) {
      known += (known.empty() ? "" : ", ") + std::string(candidate.name);
    }
    reject("unknown builtin_shape '" + spec.builtin_shape + "' (known: " + known + ")");
  }
  if (!std::isfinite(spec.t0)) {
    reject("t0 = " + fmt(spec.t0) + " must be finite");
  }
  const ShapeParameters parameters(*shape, spec);
  const Pose pose = parameters.pose();
  const std::pair<double, double> domain = builtinDomain(spec);
  const std::string name = shape->name;
  const double t0 = spec.t0;

  if (name == "ellipse") {
    return std::make_unique<EllipseCurve>(
      parameters.positive("a"), parameters.positive("b"), domain, pose, options, t0);
  }
  if (name == "lemniscate_bernoulli") {
    return std::make_unique<LemniscateBernoulliCurve>(
      parameters.positive("a"), domain, pose, options, t0);
  }
  if (name == "lemniscate_gerono") {
    return std::make_unique<LemniscateGeronoCurve>(
      parameters.positive("a"), parameters.positive("b"), domain, pose, options, t0);
  }
  if (name == "superellipse") {
    const double n = parameters.get("n");
    if (!(n >= 2.0)) {
      reject(
        "builtin 'superellipse': parameter 'n' = " + fmt(n) +
        " must be >= 2 (cornered shapes: regular_polygon or a closed POLYLINE)");
    }
    return std::make_unique<SuperellipseCurve>(
      parameters.positive("a"), parameters.positive("b"), n, domain, pose, options, t0);
  }
  if (name == "rounded_rectangle") {
    const double width = parameters.positive("width");
    const double height = parameters.positive("height");
    const double radius = parameters.positive("radius");
    const double limit = 0.5 * std::min(width, height);
    if (radius > limit * (1.0 + 1e-12)) {
      reject(
        "builtin 'rounded_rectangle': radius = " + fmt(radius) +
        " must be <= min(width, height)/2 = " + fmt(limit));
    }
    return std::make_unique<RoundedRectangleCurve>(
      width, height, std::min(radius, limit), domain, pose, options, t0);
  }
  if (name == "lissajous") {
    const long long fx = parameters.integer("fx", 1);
    const long long fy = parameters.integer("fy", 1);
    const long long divisor = std::gcd(fx, fy);
    if (divisor != 1) {
      reject(
        "builtin 'lissajous': gcd(fx, fy) = " + std::to_string(divisor) +
        " must be 1 (non-coprime frequencies trace the curve gcd times per period)");
    }
    return std::make_unique<LissajousCurve>(
      parameters.positive("ax"), parameters.positive("ay"), static_cast<double>(fx),
      static_cast<double>(fy), parameters.get("phase"), domain, pose, options, t0);
  }
  // regular_polygon
  const double circumradius = parameters.positive("radius");
  const long long sides = parameters.integer("sides", 3);
  const double corner_radius = parameters.positive("corner_radius");
  const double apothem = circumradius * std::cos(kPi / static_cast<double>(sides));
  if (corner_radius > apothem * (1.0 + 1e-12)) {
    reject(
      "builtin 'regular_polygon': corner_radius = " + fmt(corner_radius) +
      " must be <= radius cos(pi/sides) = " + fmt(apothem));
  }
  return std::make_unique<RegularPolygonCurve>(
    circumradius, sides, std::min(corner_radius, apothem), domain, pose, options, t0);
}

}  // namespace detail

}  // namespace vectorfield_core
