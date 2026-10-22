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

// VelocityLaw3 (holonomic single integrator, kinematic box, yaw policy) and CommandLimiter3
// (math_spec §8.3–§8.4, §14.4).

#include "vectorfield_core/d3/law.hpp"

#include <algorithm>
#include <cmath>

namespace vectorfield_core::d3
{

Interval boxNormInterval(const Vec3 & a, const Vec3 & b, double limit) noexcept
{
  const double B = b.squaredNorm();
  const double A = a.dot(b);
  const double C = a.squaredNorm() - limit * limit;
  if (!(B > 0.0)) {
    return C <= 0.0 ? Interval{} : Interval{kInf, -kInf};
  }
  const double disc = A * A - B * C;
  if (disc < 0.0) {
    return Interval{kInf, -kInf};
  }
  const double root = std::sqrt(disc);
  return Interval{(-A - root) / B, (-A + root) / B};
}

double uniformBoxScale(const Vec3 & v, const Limits3 & limits) noexcept
{
  double c = 1.0;
  const double norm = v.norm();
  if (norm > 0.0) {
    c = std::min(c, limits.max_speed / norm);
  }
  for (int i = 0; i < 3; ++i) {
    const double magnitude = std::abs(v[i]);
    if (magnitude > 0.0) {
      c = std::min(c, limits.max_axis_speed[i] / magnitude);
    }
  }
  return std::max(0.0, c);
}

VelocityLaw3::VelocityLaw3(const Limits3 & limits, const YawConfig3 & yaw) noexcept
: limits_(limits), yaw_(yaw)
{
}

double VelocityLaw3::desiredYaw(const Vec3 & w, double yaw, double last_psi_d) const noexcept
{
  if (yaw_.policy == OmniHeadingPolicy::kFixed) {
    return yaw_.fixed_yaw;
  }
  const double horizontal = std::hypot(w.x(), w.y());
  if (!(horizontal >= kYawHoldNorm) || !std::isfinite(horizontal)) {
    return std::isfinite(last_psi_d) ? last_psi_d : yaw;
  }
  return std::atan2(w.y(), w.x());
}

LawOutput3 VelocityLaw3::command(
  const FieldSample3 & sample, double gamma, double v_r, double assumption3_margin, double yaw,
  double last_psi_d) const noexcept
{
  LawOutput3 out;
  const double g = std::isnan(gamma) ? 0.0 : clamp(gamma, 0.0, 1.0);
  const double p_T = sample.Phi_T.norm();
  // A static terminal field has Phi_T = 0; a terminal field transported by a moving frame keeps
  // eta1 = g v_r (the relative speed), math_spec §14.7.
  const bool time_varying = p_T > 0.0;
  if (time_varying && p_T < (1.0 - assumption3_margin) * g * v_r) {
    // eta-box: the largest eta in [0, eta1] inside every bound, Phi_T kept (math_spec §8.3).
    const double eta1 = sample.terminal ? g * v_r : solveEta(sample.Phi_S, sample.Phi_T, g * v_r);
    out.F = eta1 * sample.Phi_S + sample.Phi_T;
    Interval set{0.0, eta1};
    for (int i = 0; i < 3; ++i) {
      set =
        set.intersect(boxAxisInterval(sample.Phi_T[i], sample.Phi_S[i], limits_.max_axis_speed[i]));
    }
    set = set.intersect(boxNormInterval(sample.Phi_T, sample.Phi_S, limits_.max_speed));
    if (!set.empty() && std::isfinite(set.hi)) {
      out.eta = set.hi;
      out.eta_box = true;
      out.cmd.v = sample.Phi_T + set.hi * sample.Phi_S;
    } else {
      out.c = uniformBoxScale(out.F, limits_);
      out.cmd.v = out.c * out.F;
      out.status |= Status::kAssumption3Clamped;
    }
  } else {
    out.F = g * sample.Phi;
    if (time_varying) {
      out.status |= Status::kAssumption3Clamped;
    }
    out.c = uniformBoxScale(out.F, limits_);
    out.cmd.v = out.c * out.F;
  }
  const Vec3 & source = yaw_.policy == OmniHeadingPolicy::kField ? out.F : sample.T_d;
  out.psi_d = desiredYaw(source, yaw, last_psi_d);
  out.cmd.wz =
    clamp(yaw_.gain * wrapAngle(out.psi_d - yaw), -limits_.max_yaw_rate, limits_.max_yaw_rate);
  return out;
}

Twist3 CommandLimiter3::limit(
  const Twist3 & target, const LimiterState3 & in, LimiterState3 * out, double dt,
  const Limits3 & limits, bool stopped_cancel, bool brake_translation, std::uint32_t * status,
  double * alpha) const noexcept
{
  const Twist3 prev = in.u_prev;
  Twist3 u = prev;
  double a_out = 1.0;
  if (dt > 0.0 && std::isfinite(dt)) {
    const Vec3 dv = target.v - prev.v;
    const double dw = target.wz - prev.wz;
    double alpha_lin = 1.0;
    for (int i = 0; i < 3; ++i) {
      const double magnitude = std::abs(dv[i]);
      if (magnitude > 0.0) {
        double bound = prev.v[i] * dv[i] < 0.0 ? limits.max_decel : limits.max_accel;
        if (stopped_cancel) {
          bound = limits.cancel_decel;
        }
        alpha_lin = std::min(alpha_lin, bound * dt / magnitude);
      }
    }
    double alpha_w = 1.0;
    if (std::abs(dw) > 0.0) {
      alpha_w = std::min(1.0, limits.max_yaw_accel * dt / std::abs(dw));
    }
    if (brake_translation) {
      u.v = prev.v + alpha_lin * dv;
      u.wz = prev.wz + alpha_w * dw;
      a_out = std::min(alpha_lin, alpha_w);
    } else {
      a_out = std::min(alpha_lin, alpha_w);
      u.v = prev.v + a_out * dv;
      u.wz = prev.wz + a_out * dw;
    }
  }
  if (!isFinite(u)) {
    u = Twist3{};
    if (status != nullptr) {
      *status |= Status::kNonFinite;
    }
  }
  if (alpha != nullptr) {
    *alpha = a_out;
  }
  if (out != nullptr) {
    out->u_prev = u;
    out->valid = true;
  }
  return u;
}

}  // namespace vectorfield_core::d3
