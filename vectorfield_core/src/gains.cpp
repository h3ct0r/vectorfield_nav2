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

#include "vectorfield_core/gains.hpp"

#include <cmath>
#include <string>

namespace vectorfield_core
{

namespace
{

constexpr double kTwoOverPi = 2.0 / kPi;

// sqrt(1 + u^2) without overflow for large |u|.
double hypot1(double u) noexcept { return std::hypot(1.0, u); }

}  // namespace

GainFunction::GainFunction(GainType type, double k_f) : type_(type), k_(k_f)
{
  if (!std::isfinite(k_f) || !(k_f > 0.0)) {
    throw ConfigError("convergence_gain must be finite and positive, got " + std::to_string(k_f));
  }
}

GainFunction GainFunction::fromConfig(const FieldConfig & field)
{
  return GainFunction(field.gain_function, field.convergence_gain);
}

double GainFunction::G(double D) const noexcept
{
  const double u = k_ * D;
  if (type_ == GainType::kAtan) {
    return kTwoOverPi * std::atan(u);
  }
  if (std::isinf(u)) {
    return sgn(u);
  }
  return u / hypot1(u);
}

double GainFunction::H(double D) const noexcept
{
  if (type_ == GainType::kAtan) {
    const double g = G(D);
    // math_spec §3: (1 - G)(1 + G) avoids the cancellation of 1 - G^2 near G = 1.
    return std::sqrt((1.0 - g) * (1.0 + g));
  }
  return 1.0 / hypot1(k_ * D);
}

double GainFunction::dG(double D) const noexcept
{
  const double u = k_ * D;
  if (type_ == GainType::kAtan) {
    return kTwoOverPi * k_ / (1.0 + u * u);
  }
  const double h = hypot1(u);
  return k_ / (h * h * h);
}

double GainFunction::Ginv(double y) const noexcept
{
  if (std::isnan(y)) {
    return y;
  }
  const double a = std::abs(y);
  if (a >= 1.0) {
    return std::copysign(kInf, y);
  }
  double x = 0.0;
  if (type_ == GainType::kAtan) {
    x = std::tan(kHalfPi * a) / k_;
  } else {
    x = a / (k_ * std::sqrt((1.0 - a) * (1.0 + a)));
  }
  return std::copysign(x, y);
}

double GainFunction::dG0() const noexcept
{
  return type_ == GainType::kAtan ? kTwoOverPi * k_ : k_;
}

double GainFunction::cG() const noexcept
{
  return type_ == GainType::kAtan ? kCgAtan : kCgRational;
}

double GainFunction::integralCurveCurvature(double D) const noexcept { return G(D) * dG(D) / H(D); }

}  // namespace vectorfield_core
