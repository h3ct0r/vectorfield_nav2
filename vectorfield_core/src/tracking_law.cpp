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

// Tracking laws (math_spec §7).

#include "vectorfield_core/tracking_law.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "vectorfield_core/errors.hpp"

namespace vectorfield_core
{

namespace
{

/// @brief The drive-direction yaw \f$\psi_{\mathrm{dr}} = \psi + \pi[q = -1]\f$ (math_spec §7.4).
double driveYaw(double psi, double q) noexcept { return q < 0.0 ? psi + kPi : psi; }

/// @brief True iff a probe is usable for \f$\dot\psi_d\f$: a finite \f$\psi_d\f$ (a probe below the
/// hold speed returns NaN) and a finite station; \f$D\f$ may be \f$+\infty\f$ but not NaN.
bool probeUsable(const ProbeSample & probe) noexcept
{
  return std::isfinite(probe.psi_d) && std::isfinite(probe.s_star) && !std::isnan(probe.D);
}

/// @brief True iff two probes differ in their contour state (math_spec §7.4).
bool contourChanged(const ProbeSample & a, const ProbeSample & b) noexcept
{
  return a.contour_active != b.contour_active || a.sigma_c != b.sigma_c;
}

/// @brief Jump test between the spatial probes (math_spec §7.4 [Design]): a closest-point jump
/// \f$\lvert s^\star_+ - s^\star_-\rvert > 1.05\,(2\delta + 2\max(D_+, D_-))\f$, with the arc
/// difference wrapped on closed curves, or a contour change. A change of vertex or segment is not
/// a jump. The 5% slack covers the polyline arc between feet on different segments, which exceeds
/// their chord (at most \f$2\delta\f$ on the path): with \f$\delta = \bar\ell\f$ on a 126-gon the
/// feet are 1.0003 x \f$2\delta\f$ apart along the curve.
///
/// The threshold carries the closest-point tie tolerance (1e-9 m, Appendix C item 1) as a rounding
/// allowance: on the path of a straight piece (D = 0, heading along T) the feet are exactly
/// \f$2\delta\f$ apart, and rounding of the stations must not report that as a jump. Real jumps
/// are at least \f$2D\tan(\theta/2)\f$ beyond the threshold's geometry, never 1e-9 m.
bool probesJump(
  const ProbeSample & plus, const ProbeSample & minus, double delta, double curve_length) noexcept
{
  if (contourChanged(plus, minus)) {
    return true;
  }
  const double arc = wrapPeriodic(plus.s_star - minus.s_star, curve_length);
  return std::abs(arc) >
         kProbeJumpSlack * (2.0 * delta + 2.0 * std::max(plus.D, minus.D)) + kTieTolerance;
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Evaluation points and FL matrices (math_spec §7.1, §7.3).

EvaluationPoint evaluationPoint(
  const Pose2 & base, Phase phase, TrackingLaw law, const Kinematics & kinematics) noexcept
{
  EvaluationPoint point;
  point.law = resolveTrackingLaw(kinematics.type, law);
  point.x = base.p;
  point.x_ref = 0.0;
  if (kinematics.isAckermann()) {
    // Heading tracking at the rear axle p_r = p + x_r h(psi), in every phase; the reference
    // shift is the offset along the drive direction q h (math_spec §5.2 [Design]).
    point.x = base.p + kinematics.reference_point_offset * heading(base.yaw);
    point.x_ref = kinematics.driveSign() * kinematics.reference_point_offset;
    return point;
  }
  if (point.law == TrackingLaw::kFeedbackLinearization) {
    if (phase == Phase::kFinalApproach) {
      // FL hands over to heading tracking at base_link on the unshifted field.
      point.law = TrackingLaw::kHeadingTracking;
      return point;
    }
    // p_c = p + q d h(psi); the reference shift is d along the drive direction (math_spec §5.2).
    const double d = kinematics.control_point_offset;
    point.x = base.p + (kinematics.driveSign() * d) * heading(base.yaw);
    point.x_ref = d;
  }
  return point;
}

Mat2 flMatrix(double psi, double d, double q) noexcept
{
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  Mat2 m;
  m << c, s, -q * s / d, q * c / d;
  return m;
}

Mat2 flInverse(double psi, double d, double q) noexcept
{
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  Mat2 m;
  m << c, -q * d * s, s, q * d * c;
  return m;
}

// -----------------------------------------------------------------------------------------------
// Laws.

LawOutput holonomicLaw(const LawInput & input, OmniHeadingPolicy policy, double k_psi) noexcept
{
  LawOutput out;
  const double psi = input.pose.yaw;
  // (v_x, v_y) = R(psi)^T F: the base moves with F whatever omega is.
  const Vec2 body = rotate(-psi, input.F);
  out.cmd.vx = body.x();
  out.cmd.vy = body.y();

  double target = kNaN;
  bool no_target = false;
  switch (policy) {
    case OmniHeadingPolicy::kPathTangent:
      target = angleOf(input.T_d);
      break;
    case OmniHeadingPolicy::kField:
      if (input.F.norm() < input.hold_speed) {
        out.held = true;
        target = input.last_psi_d;
        // Held with no previous target: nothing to face yet.
        no_target = std::isnan(target);
      } else {
        target = angleOf(input.F);
      }
      out.psi_d = target;
      break;
    case OmniHeadingPolicy::kFixed:
      no_target = true;
      break;
  }

  if (no_target) {
    out.cmd.wz = 0.0;
    out.heading_error = 0.0;
    return out;
  }
  // Feedback only, never scaled by gamma; bounded by omega_bar on its own (math_spec §7.2).
  const double e = wrapAngle(target - psi);
  out.heading_error = e;
  out.cmd.wz = clamp(k_psi * e, -input.omega_bar, input.omega_bar);
  return out;
}

LawOutput feedbackLinearizationLaw(const LawInput & input, double d, double q) noexcept
{
  LawOutput out;
  const double psi = input.pose.yaw;
  const Vec2 h = heading(psi);
  // (v, omega) = M(psi) F, thesis eq. 2.1 with the drive sign (math_spec §7.3).
  out.cmd.vx = h.dot(input.F);
  out.cmd.vy = 0.0;
  out.cmd.wz = q * J(h).dot(input.F) / d;
  const bool has_direction = input.F.x() != 0.0 || input.F.y() != 0.0;
  out.heading_error = has_direction ? wrapAngle(angleOf(input.F) - driveYaw(psi, q)) : 0.0;
  return out;
}

LawOutput headingTrackingLaw(
  const LawInput & input, FieldProbe probe, double k_psi, double q,
  double min_turning_radius) noexcept
{
  LawOutput out;
  const double psi = input.pose.yaw;
  const double drive = q < 0.0 ? -1.0 : 1.0;
  const double psi_dr = driveYaw(psi, drive);
  const double F_norm = input.F.norm();
  const bool ackermann = min_turning_radius > 0.0;

  // Desired heading, held while ||F|| < hold_speed (0.05 v_r gamma_lim, math_spec §7.4). With no
  // previous value the drive direction is held, i.e. no turn on a meaningless direction.
  const bool held = F_norm < input.hold_speed;
  double psi_d = angleOf(input.F);
  if (held) {
    psi_d = std::isfinite(input.last_psi_d) ? input.last_psi_d : psi_dr;
  }
  const double e = wrapAngle(psi_d - psi_dr);
  // Ackermann in a contour episode keeps moving while it turns at R_min (math_spec §6.6).
  const bool contour_turn = ackermann && input.contour_episode;
  const double speed_factor =
    contour_turn ? std::max(std::cos(e), kContourTurnSpeedFactor) : std::max(0.0, std::cos(e));
  const double v = drive * F_norm * speed_factor;
  const double omega_c = ackermann ? std::abs(v) / min_turning_radius : input.omega_bar;

  // Feedforward along the actual motion v h (math_spec §7.4 [Design]); zero while held.
  double psi_d_dot = 0.0;
  bool jump = false;
  if (!held && probe) {
    const Vec2 h = heading(psi);
    // A span 2 delta of whole vertex spacings keeps the vertex count between the probes constant,
    // so the finite difference does not alternate between the full turn and 0 (curve.hpp
    // Curve::vertexSpacing()). NaN or negative spacings are ignored by the max.
    const double spacing = std::min(input.vertex_spacing, kProbeSpacingCap);
    const double delta =
      std::max({kProbeMinOffset, std::abs(v) * input.dt, std::isnan(spacing) ? 0.0 : spacing});
    const ProbeSample plus = probe(input.x + delta * h, 0.0);
    const ProbeSample minus = probe(input.x - delta * h, 0.0);
    bool zero = !probeUsable(plus) || !probeUsable(minus);
    if (!zero && probesJump(plus, minus, delta, input.curve_length)) {
      jump = true;
      zero = true;
    }
    double rate = 0.0;
    if (!zero) {
      rate = v * wrapAngle(plus.psi_d - minus.psi_d) / (2.0 * delta);
      if (input.time_varying && input.dt > 0.0 && std::isfinite(input.dt)) {
        // d_t psi_d over t -+ dt/2 at fixed x [Spec]; the hint, K and gate stay frozen.
        const ProbeSample later = probe(input.x, 0.5 * input.dt);
        const ProbeSample earlier = probe(input.x, -0.5 * input.dt);
        if (!std::isfinite(later.psi_d) || !std::isfinite(earlier.psi_d)) {
          zero = true;
        } else if (contourChanged(later, earlier)) {
          jump = true;
          zero = true;
        } else {
          rate += wrapAngle(later.psi_d - earlier.psi_d) / input.dt;
        }
      }
    }
    psi_d_dot = zero ? 0.0 : rate;
  }

  out.cmd.vx = v;
  out.cmd.vy = 0.0;
  out.cmd.wz = clamp(psi_d_dot, -omega_c, omega_c) + k_psi * e;
  out.psi_d = psi_d;
  out.heading_error = e;
  out.psi_d_dot = psi_d_dot;
  out.held = held;
  out.jump = jump;
  if (ackermann && !contour_turn && std::abs(e) > kHalfPi) {
    out.status |= Status::kHeadingInfeasible;
  }
  return out;
}

double rotateLawOmega(double e_r, const RotateLawParams & params) noexcept
{
  if (std::isnan(e_r)) {
    return kNaN;
  }
  const double magnitude = std::abs(e_r);
  if (magnitude <= params.dead_band) {
    return 0.0;
  }
  // RPP-style: the stopping term sqrt(2 alpha_max |e_r|) brakes within the angular acceleration
  // bound; the floor keeps a skid-steer base turning (math_spec §7.6 [Spec]).
  const double stopping = std::sqrt(2.0 * params.alpha_max * magnitude);
  return sgn(e_r) * std::max(params.omega_floor, std::min(params.omega_rot, stopping));
}

Twist2 applyReversingBackstop(const Twist2 & cmd, double q) noexcept
{
  Twist2 out = cmd;
  if (q * cmd.vx < 0.0) {
    out.vx = 0.0;
  }
  return out;
}

double applyAckermannBand(double v, double v_ack) noexcept
{
  return v > 0.0 && v < v_ack ? v_ack : v;
}

// -----------------------------------------------------------------------------------------------
// ControlLaw.

ControlLaw::ControlLaw(const CoreConfig & config)
: kinematics_(Kinematics::fromConfig(config)),
  law_(resolveTrackingLaw(config.motion.motion_model, config.motion.tracking_law)),
  heading_policy_(config.motion.omni_heading_policy),
  k_psi_(config.motion.heading_gain),
  rotate_to_heading_vel_(config.phases.rotate_to_heading_angular_vel),
  min_in_place_vel_(config.motion.min_in_place_angular_vel)
{
  if (!isValidCombination(config.motion.motion_model, law_)) {
    throw ConfigError(
      std::string("tracking_law '") + toString(config.motion.tracking_law) +
      "' is not valid for motion_model '" + toString(config.motion.motion_model) + "'");
  }
  const double d = config.motion.control_point_offset;
  if (law_ == TrackingLaw::kFeedbackLinearization && (!std::isfinite(d) || d <= 0.0)) {
    throw ConfigError(
      "feedback_linearization needs control_point_offset > 0, got " + std::to_string(d));
  }
}

TrackingLaw ControlLaw::law() const noexcept { return law_; }

TrackingLaw ControlLaw::activeLaw(Phase phase) const noexcept
{
  if (law_ == TrackingLaw::kFeedbackLinearization && phase == Phase::kFinalApproach) {
    return TrackingLaw::kHeadingTracking;
  }
  return law_;
}

EvaluationPoint ControlLaw::evaluationPoint(const Pose2 & base, Phase phase) const noexcept
{
  return vectorfield_core::evaluationPoint(base, phase, law_, kinematics_);
}

const Kinematics & ControlLaw::kinematics() const noexcept { return kinematics_; }

ControlLaw ControlLaw::withDriveSign(double q) const noexcept
{
  ControlLaw law = *this;
  law.kinematics_.drive_backward = q < 0.0;
  return law;
}

LawOutput ControlLaw::track(Phase phase, const LawInput & input, FieldProbe probe) const noexcept
{
  switch (activeLaw(phase)) {
    case TrackingLaw::kHolonomic:
      return holonomicLaw(input, heading_policy_, k_psi_);
    case TrackingLaw::kHeadingTracking: {
      // The drive sign q (Ackermann: -1 only in a reverse plan segment, math_spec §7.5), also
      // when FL hands over in FINAL_APPROACH; Ackermann bounds omega_c = |v|/R_min.
      const bool ackermann = kinematics_.isAckermann();
      const double r_min = ackermann ? kinematics_.min_turning_radius : 0.0;
      return headingTrackingLaw(input, probe, k_psi_, kinematics_.driveSign(), r_min);
    }
    case TrackingLaw::kFeedbackLinearization:
    case TrackingLaw::kAuto:
      break;
  }
  return feedbackLinearizationLaw(input, kinematics_.control_point_offset, kinematics_.driveSign());
}

double ControlLaw::rotate(Phase phase, double e_r, double omega_bar, double yaw_tol) const noexcept
{
  if (phase == Phase::kStopped) {
    return 0.0;
  }
  RotateLawParams params;
  params.omega_floor = min_in_place_vel_;
  params.omega_rot = std::min(rotate_to_heading_vel_, omega_bar);
  params.alpha_max = kinematics_.max_angular_accel;
  // ROTATE_TO_HEADING: omega* = 0 inside yaw_tol/2, the floor only outside it (math_spec §7.6).
  params.dead_band = phase == Phase::kRotateToHeading ? 0.5 * yaw_tol : 0.0;
  return rotateLawOmega(e_r, params);
}

}  // namespace vectorfield_core
