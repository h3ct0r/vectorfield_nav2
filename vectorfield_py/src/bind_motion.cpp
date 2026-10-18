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

// Motion: Kinematics and plant integration (vectorfield.sim), the tracking laws and their
// helpers, speed shaping, the kinematic box and the CommandLimiter (kinematics.hpp,
// tracking_law.hpp, limiter.hpp).

#include <cstdint>
#include <optional>

#include "bindings.hpp"

namespace vectorfield_py
{

namespace
{

void bindKinematics(py::module_ & m, py::module_ & sim)
{
  using vf::Kinematics;
  StructBinder<Kinematics> k(
    m, "Kinematics",
    "Motion model and limits shared by the laws, the limiter, the predictor and sim.integrate.");
  k.field("type", &Kinematics::type, "MotionModel.")
    .field("max_linear_vel", &Kinematics::max_linear_vel, "v_max (m/s).")
    .field("max_lateral_vel", &Kinematics::max_lateral_vel, "vy_max (m/s).")
    .field("max_angular_vel", &Kinematics::max_angular_vel, "omega_max (rad/s).")
    .field("max_linear_accel", &Kinematics::max_linear_accel, "a_acc (m/s^2).")
    .field("max_linear_decel", &Kinematics::max_linear_decel, "a_dec (m/s^2).")
    .field("max_angular_accel", &Kinematics::max_angular_accel, "alpha_max (rad/s^2).")
    .field("cancel_deceleration", &Kinematics::cancel_deceleration, "a_can (m/s^2).")
    .field("control_point_offset", &Kinematics::control_point_offset, "FL offset d (m).")
    .field("reference_point_offset", &Kinematics::reference_point_offset, "Ackermann x_r (m).")
    .field("min_turning_radius", &Kinematics::min_turning_radius, "Ackermann R_min (m).")
    .field("ackermann_min_speed", &Kinematics::ackermann_min_speed, "v_ack (m/s).")
    .field("drive_backward", &Kinematics::drive_backward, "q = -1.")
    .field("allow_reversing", &Kinematics::allow_reversing, "Allow q v < 0.");
  k.cls()
    .def_static(
      "from_config", &Kinematics::fromConfig, py::arg("config"), "The kinematics of a CoreConfig.")
    .def_property_readonly("drive_sign", &Kinematics::driveSign, "q: -1 iff drive_backward.")
    .def_property_readonly("is_holonomic", &Kinematics::isHolonomic, "Omnidirectional.")
    .def_property_readonly("is_ackermann", &Kinematics::isAckermann, "Ackermann.")
    .def_property_readonly("is_unicycle", &Kinematics::isUnicycle, "Differential or skid steer.")
    .def(
      "angular_limit", &Kinematics::angularLimit, py::arg("speed_limit_scale") = 1.0,
      "omega_max times the speed-limit scale.");
  k.finish();

  using vf::sim::SlipParams;
  StructBinder<SlipParams>(
    sim, "SlipParams",
    "Plant slip: omega' = eta_psi omega, vy' = vy - x_icr omega' (identity by default).")
    .field("eta_psi", &SlipParams::eta_psi, "Yaw-rate efficiency.")
    .field("x_icr", &SlipParams::x_icr, "ICR longitudinal offset (m).")
    .finish();

  sim.def(
    "feasible_twist", &vf::sim::feasibleTwist, py::arg("kinematics"), py::arg("command"),
    "The body twist the model can execute (unicycles drop vy; Ackermann also bounds omega).");
  sim.def(
    "integrate_twist", &vf::sim::integrateTwist, py::arg("pose"), py::arg("twist"), py::arg("dt"),
    "Exact SE(2) integration of a constant body twist over dt.");
  sim.def(
    "integrate", &vf::sim::integrate, py::arg("kinematics"), py::arg("pose"), py::arg("command"),
    py::arg("dt"), py::arg("slip") = SlipParams{},
    "One zero-order-hold plant step: feasible twist, slip, exact arc (bicycle at the rear "
    "axle). The yaw is not wrapped.");
}

void bindLaws(py::module_ & m)
{
  using vf::ProbeSample;
  StructBinder<ProbeSample>(
    m, "ProbeSample", "What a field probe returns at a point (math_spec section 7.4).")
    .field("F", &ProbeSample::F, "Field the law would track there.")
    .field("psi_d", &ProbeSample::psi_d, "Angle of F; NaN below the hold speed.")
    .field("s_star", &ProbeSample::s_star, "Closest-point station.")
    .field("D", &ProbeSample::D, "Distance to the curve.")
    .field("contour_active", &ProbeSample::contour_active, "A contour episode would be active.")
    .field("sigma_c", &ProbeSample::sigma_c, "Its circulation side.")
    .finish();

  using vf::EvaluationPoint;
  StructBinder<EvaluationPoint>(
    m, "EvaluationPoint", "The law's evaluation point and reference offset (math_spec 7.1).")
    .field("x", &EvaluationPoint::x, "Evaluation point.")
    .field("x_ref", &EvaluationPoint::x_ref, "Body-x offset of the point from base_link.")
    .field("law", &EvaluationPoint::law, "Law that runs in this phase.")
    .finish();

  m.def(
    "evaluation_point", &vf::evaluationPoint, py::arg("base"), py::arg("phase"), py::arg("law"),
    py::arg("kinematics"), "Evaluation point per (law, phase) of math_spec section 7.1.");
  m.def(
    "fl_matrix", &vf::flMatrix, py::arg("psi"), py::arg("d"), py::arg("q") = 1.0,
    "M(psi) with (v, omega) = M F (feedback linearisation).");
  m.def(
    "fl_inverse", &vf::flInverse, py::arg("psi"), py::arg("d"), py::arg("q") = 1.0, "M(psi)^-1.");

  using vf::LawInput;
  StructBinder<LawInput>(m, "LawInput", "Per-cycle inputs of a translation law.")
    .field("pose", &LawInput::pose, "base_link pose.")
    .field("x", &LawInput::x, "Evaluation point.")
    .field("F", &LawInput::F, "Field to track at x.")
    .field("T_d", &LawInput::T_d, "Field tangent at the closest point.")
    .field("dt", &LawInput::dt, "Delta t (s).")
    .field("omega_bar", &LawInput::omega_bar, "omega_max times the speed-limit scale.")
    .field("hold_speed", &LawInput::hold_speed, "|F| below which psi_d is held.")
    .field("last_psi_d", &LawInput::last_psi_d, "Previous psi_d (NaN: none).")
    .field("curve_length", &LawInput::curve_length, "Closed: L; open: inf.")
    .field("vertex_spacing", &LawInput::vertex_spacing, "Mean vertex spacing (m).")
    .field("time_varying", &LawInput::time_varying, "The field is time-varying.")
    .finish();

  using vf::LawOutput;
  StructBinder<LawOutput>(m, "LawOutput", "Output of a translation law, before the box.")
    .field("cmd", &LawOutput::cmd, "Body twist at base_link.")
    .field("psi_d", &LawOutput::psi_d, "Desired heading used (NaN when not applicable).")
    .field("heading_error", &LawOutput::heading_error, "Heading error of the law.")
    .field("psi_d_dot", &LawOutput::psi_d_dot, "psi_d rate after jump zeroing.")
    .field("held", &LawOutput::held, "psi_d was held.")
    .field("jump", &LawOutput::jump, "Jump-only zeroing fired.")
    .field("status", &LawOutput::status, "Status bits.")
    .finish();

  m.def(
    "holonomic_law", &vf::holonomicLaw, py::arg("input"), py::arg("policy"), py::arg("k_psi"),
    "Holonomic law (math_spec section 7.2).");
  m.def(
    "feedback_linearization_law", &vf::feedbackLinearizationLaw, py::arg("input"), py::arg("d"),
    py::arg("q") = 1.0, "Feedback linearisation (v, omega) = M(psi) F (math_spec section 7.3).");

  // probe(x, time_offset) -> ProbeSample from Python; None gives NaN probes (no feedforward).
  const auto run_with_probe = [](const std::optional<py::function> & probe, const auto & law) {
    CallbackGuard guard;
    const auto call = [&probe, &guard](const vf::Vec2 & x, double time_offset) {
      if (!probe) {
        return ProbeSample{};
      }
      return guard.call(
        [&]() { return (*probe)(vf::Vec2(x), time_offset).template cast<ProbeSample>(); },
        ProbeSample{});
    };
    const LawOutput out = law(vf::FieldProbe(call));
    guard.rethrow();
    return out;
  };
  m.def(
    "heading_tracking_law",
    [run_with_probe](
      const LawInput & input, const std::optional<py::function> & probe, double k_psi, double q,
      double min_turning_radius) {
      return run_with_probe(probe, [&](vf::FieldProbe p) {
        return vf::headingTrackingLaw(input, p, k_psi, q, min_turning_radius);
      });
    },
    py::arg("input"), py::arg("probe"), py::arg("k_psi"), py::arg("q") = 1.0,
    py::arg("min_turning_radius") = 0.0,
    "Heading tracking (math_spec section 7.4); probe(x, time_offset) -> ProbeSample gives the "
    "psi_d feedforward (None: no feedforward).");

  using vf::RotateLawParams;
  StructBinder<RotateLawParams>(m, "RotateLawParams", "Parameters of the rotate law.")
    .field("omega_floor", &RotateLawParams::omega_floor, "Minimum in-place rate (rad/s).")
    .field("omega_rot", &RotateLawParams::omega_rot, "Maximum rotate rate (rad/s).")
    .field("alpha_max", &RotateLawParams::alpha_max, "alpha_max (rad/s^2).")
    .field("dead_band", &RotateLawParams::dead_band, "|e| at or below which omega = 0.")
    .finish();
  m.def(
    "rotate_law_omega", &vf::rotateLawOmega, py::arg("e_r"), py::arg("params"),
    "Rotate law sgn(e) max(omega_floor, min(omega_rot, sqrt(2 alpha_max |e|))).");
  m.def(
    "apply_reversing_backstop", &vf::applyReversingBackstop, py::arg("cmd"), py::arg("q"),
    "Clamps the drive-direction speed to q v >= 0.");
  m.def(
    "apply_ackermann_band", &vf::applyAckermannBand, py::arg("v"), py::arg("v_ack"),
    "Raises a positive speed below v_ack to v_ack.");

  using vf::ControlLaw;
  py::class_<ControlLaw>(
    m, "ControlLaw", "Runs the configured tracking law of a phase (math_spec section 7).")
    .def(py::init<const vf::CoreConfig &>(), py::arg("config"), "Raises ConfigError if invalid.")
    .def_property_readonly("law", &ControlLaw::law, "The resolved law of FOLLOW.")
    .def("active_law", &ControlLaw::activeLaw, py::arg("phase"), "The law that runs in a phase.")
    .def(
      "evaluation_point", &ControlLaw::evaluationPoint, py::arg("base"), py::arg("phase"),
      "evaluation_point() for this law.")
    .def_property_readonly(
      "kinematics", &ControlLaw::kinematics, py::return_value_policy::copy, "Kinematics.")
    .def(
      "track",
      [run_with_probe](
        const ControlLaw & law, vf::Phase phase, const LawInput & input,
        const std::optional<py::function> & probe) {
        return run_with_probe(probe, [&](vf::FieldProbe p) { return law.track(phase, input, p); });
      },
      py::arg("phase"), py::arg("input"), py::arg("probe") = py::none(),
      "Translation law of FOLLOW or FINAL_APPROACH.")
    .def(
      "rotate", &ControlLaw::rotate, py::arg("phase"), py::arg("e_r"), py::arg("omega_bar"),
      py::arg("yaw_tol") = 0.0, "Rotate law in a rotation phase.");
}

void bindLimiter(py::module_ & m)
{
  py::enum_<vf::ShapingTerm>(m, "ShapingTerm", "The term that binds gamma.")
    .value("NONE", vf::ShapingTerm::kNone)
    .value("SPEED_LIMIT", vf::ShapingTerm::kSpeedLimit)
    .value("APPROACH", vf::ShapingTerm::kApproach)
    .value("CLEARANCE", vf::ShapingTerm::kClearance)
    .value("COLLISION", vf::ShapingTerm::kCollision);

  using vf::SpeedShaping;
  StructBinder<SpeedShaping> shaping(
    m, "SpeedShaping", "Speed-shaping terms; gamma() is their minimum with 1.");
  shaping.field("gamma_lim", &SpeedShaping::gamma_lim, "External speed limit.")
    .field("gamma_cl", &SpeedShaping::gamma_cl, "Clearance slowdown.")
    .field("gamma_ap", &SpeedShaping::gamma_ap, "Window-end approach cap.")
    .field("gamma_col", &SpeedShaping::gamma_col, "Collision cap.");
  shaping.cls()
    .def("gamma", &SpeedShaping::gamma, "gamma.")
    .def("binding", &SpeedShaping::binding, "The binding ShapingTerm.")
    .def("cap_binds", &SpeedShaping::capBinds, "The collision cap binds strictly.")
    .def("collision_ahead", &SpeedShaping::collisionAhead, "STATUS_COLLISION_AHEAD condition.");
  shaping.finish();

  m.def(
    "clearance_scale", &vf::clearanceScale, py::arg("edt_at_base"), py::arg("w_lat"),
    py::arg("slowdown_distance"), py::arg("slowdown_gain"), "gamma_cl.");
  m.def(
    "approach_scale", &vf::approachScale, py::arg("sigma_end"), py::arg("v_min"), py::arg("a_dec"),
    py::arg("v_r"), "gamma_ap = min(1, sqrt(v_min^2 + 2 a_dec sigma_end) / v_r).");
  m.def("collision_scale", &vf::collisionScale, py::arg("v_cap"), py::arg("v_r"), "gamma_col.");

  using vf::Interval;
  StructBinder<Interval> interval(m, "Interval", "Closed interval [lo, hi]; empty when lo > hi.");
  interval.field("lo", &Interval::lo, "Lower bound.").field("hi", &Interval::hi, "Upper bound.");
  interval.cls()
    .def_property_readonly("empty", &Interval::empty, "True iff empty.")
    .def("intersect", &Interval::intersect, py::arg("other"), "Intersection.");
  interval.finish();
  m.def(
    "box_axis_interval", &vf::boxAxisInterval, py::arg("a"), py::arg("b"), py::arg("limit"),
    "{eta : |a + eta b| <= limit}.");
  m.def(
    "box_norm_interval", &vf::boxNormInterval, py::arg("a"), py::arg("b"), py::arg("limit"),
    "{eta : |a + eta b| <= limit} for vectors.");

  using vf::BoxResult;
  StructBinder<BoxResult>(m, "BoxResult", "Result of the kinematic box (math_spec 8.3).")
    .field("cmd", &BoxResult::cmd, "Command after the box.")
    .field("c", &BoxResult::c, "Uniform scale.")
    .field("eta", &BoxResult::eta, "eta* of the eta-box, else NaN.")
    .field("eta_box", &BoxResult::eta_box, "The eta-box produced the command.")
    .field("fallback", &BoxResult::fallback, "The uniform fallback was used.")
    .field("status", &BoxResult::status, "Status bits.")
    .finish();
  m.def(
    "uniform_box", &vf::uniformBox, py::arg("cmd"), py::arg("kinematics"), py::arg("omega_bar"),
    "Uniform kinematic box.");
  m.def(
    "eta_box", &vf::etaBox, py::arg("a"), py::arg("b"), py::arg("eta1"), py::arg("holonomic_omega"),
    py::arg("fallback_cmd"), py::arg("kinematics"), py::arg("omega_bar"),
    "eta-box of a time-varying field under FL or holonomic.");

  using vf::LimiterState;
  StructBinder<LimiterState>(m, "LimiterState", "Cross-cycle state of the CommandLimiter.")
    .field("u_prev", &LimiterState::u_prev, "Previous output.")
    .field("cancel_hard", &LimiterState::cancel_hard, "Hard cancel deceleration latched.")
    .field("valid", &LimiterState::valid, "False requests a re-seed from the measured twist.")
    .finish();

  py::class_<vf::CommandLimiter>(
    m, "CommandLimiter",
    "Acceleration stage, Ackermann curvature clamp and non-finite guard (stateless).")
    .def(py::init<>())
    .def(
      "limit",
      [](
        const vf::CommandLimiter & limiter, const vf::Twist2 & target, const LimiterState & state,
        double dt, const vf::Kinematics & kinematics, bool stopped_cancel) {
        LimiterState next;
        std::uint32_t status = vf::Status::kNone;
        double alpha = 1.0;
        const vf::Twist2 cmd =
          limiter.limit(target, state, &next, dt, kinematics, stopped_cancel, &status, &alpha);
        return py::make_tuple(cmd, next, status, alpha);
      },
      py::arg("target"), py::arg("state"), py::arg("dt"), py::arg("kinematics"),
      py::arg("stopped_cancel") = false,
      "Limits a target twist: (cmd, next_state, status_bits, alpha).");
}

}  // namespace

void bindMotion(py::module_ & m, py::module_ & sim)
{
  bindKinematics(m, sim);
  bindLaws(m);
  bindLimiter(m);
}

}  // namespace vectorfield_py
