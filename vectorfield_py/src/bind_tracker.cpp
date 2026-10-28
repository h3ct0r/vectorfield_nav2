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

// The Tracker and its inputs, outputs and state, and the closed-loop simulator
// (tracker.hpp, simulator.hpp).
//
// GIL: Tracker.step() and probe() keep it (a step is short, and the tracker's own scratch is
// shared by its calls). ClosedLoopSimulator.run() releases it unless a Python callback
// (disturbance or update hook) is involved; the simulator works on its own copies of the input,
// so the curve and obstacles are kept alive for the run, but they must not be modified from
// another thread meanwhile.

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "bindings.hpp"

namespace vectorfield_py
{

namespace
{

void bindState(py::module_ & m)
{
  using vf::ContourState;
  StructBinder<ContourState>(m, "ContourState", "Cross-cycle state of the obstacle contour.")
    .field("gate_open", &ContourState::gate_open, "The path_blocked gate is open.")
    .field("gate_clear_s", &ContourState::gate_clear_s, "Time the path stayed clear (s).")
    .field("active", &ContourState::active, "A contour episode is active.")
    .field("sigma_c", &ContourState::sigma_c, "Circulation side.")
    .field("travelled_m", &ContourState::travelled_m, "Episode path length (m).")
    .field("elapsed_s", &ContourState::elapsed_s, "Episode time (s).")
    .field("k_seed", &ContourState::k_seed, "Last obstacle point of the episode (grid frame).")
    .field("k_valid", &ContourState::k_valid, "k_seed is set.")
    .field(
      "b_site", &ContourState::b_site,
      "Ackermann: last blocking site of the episode (grid frame), the exit's fallback.")
    .field("b_valid", &ContourState::b_valid, "b_site is set.")
    .finish();

  using vf::TrackerState;
  StructBinder<TrackerState> state(
    m, "TrackerState",
    "The only cross-cycle state of the controller. Start from Tracker.initial_state().");
  state.field("phase", &TrackerState::phase, "Phase.")
    .field("hint", &TrackerState::hint, "CurveHint (x_star in the evaluation frame).")
    .field("last_tangent", &TrackerState::last_tangent, "Last tangent; zero = none.")
    .field("last_psi_d", &TrackerState::last_psi_d, "Last desired heading; NaN = none.")
    .field("contour", &TrackerState::contour, "ContourState.")
    .field("blocked", &TrackerState::blocked, "kBlocked hysteresis latch.")
    .field("terminal_mode", &TrackerState::terminal_mode, "Goal-in-curve-end hysteresis.")
    .field("xy_latched", &TrackerState::xy_latched, "Follower XY latch.")
    .field("holding", &TrackerState::holding, "Goal-hold latch.")
    .field("goal_seq", &TrackerState::goal_seq, "Goal identity of the last step.")
    .field("limiter", &TrackerState::limiter, "LimiterState.")
    .field("rotate_target", &TrackerState::rotate_target, "Kept rotation target; NaN = none.")
    .field("stopped_status", &TrackerState::stopped_status, "Bits that latched STOPPED.")
    .field(
      "ackermann_band_lifted", &TrackerState::ackermann_band_lifted,
      "The Ackermann v_ack band is lifted for this goal.")
    .field("drive_sign", &TrackerState::drive_sign, "Drive sign of the last step; 0 = none yet.");
  state.cls().def(
    "request_limiter_reseed", [](TrackerState & s) { vf::requestLimiterReseed(&s); },
    "Re-seed the limiter from the measured twist at the next step.");
  state.finish();
}

void bindInput(py::module_ & m)
{
  using vf::GoalInfo;
  StructBinder<GoalInfo>(m, "GoalInfo", "The goal of an open curve for one cycle.")
    .field("pose", &GoalInfo::pose, "Goal pose (evaluation frame).")
    .field("has_yaw", &GoalInfo::has_yaw, "The goal yaw is to be reached.")
    .field("xy_tol", &GoalInfo::xy_tol, "XY tolerance (m).")
    .field("yaw_tol", &GoalInfo::yaw_tol, "Yaw tolerance (rad); <= 1e-6 = unchecked.")
    .field(
      "xy_reached", &GoalInfo::xy_reached,
      "XY reached as Nav2 sees it, or None (the core latch decides).")
    .finish();

  using vf::CurveFrame;
  StructBinder<CurveFrame>(m, "CurveFrame", "The curve frame of one cycle.")
    .field("T_eval_curve", &CurveFrame::T_eval_curve, "Curve frame to evaluation frame.")
    .field("twist_in_curve", &CurveFrame::twist_in_curve, "Frame twist expressed in the frame.")
    .field("moving", &CurveFrame::moving, "The frame moves.")
    .finish();

  py::class_<TrackerInputBox> cls(
    m, "TrackerInput",
    "Inputs of one control cycle. Keeps the curve and the obstacles alive; `curve` is required "
    "by Tracker.step().");
  cls.def(
    py::init([](
               const std::shared_ptr<vf::Curve> & curve, const vf::Pose2 & pose,
               const vf::Twist2 & measured, double t, double dt, const CurveFrame & frame,
               std::uint32_t goal_seq, const std::optional<GoalInfo> & goal,
               const std::optional<ObstacleMapBox> & obstacles, double speed_limit_scale,
               bool cancel, bool cap_at_curve_end) {
      TrackerInputBox box;
      box.curve = curve;
      box.input.pose = pose;
      box.input.measured = measured;
      box.input.t = t;
      box.input.dt = dt;
      box.input.frame = frame;
      box.input.goal_seq = goal_seq;
      box.input.goal = goal;
      box.obstacles = obstacles;
      box.input.speed_limit_scale = speed_limit_scale;
      box.input.cancel = cancel;
      box.input.cap_at_curve_end = cap_at_curve_end;
      return box;
    }),
    py::arg("curve") = py::none(), py::kw_only(), py::arg("pose") = vf::Pose2{},
    py::arg("measured") = vf::Twist2{}, py::arg("t") = 0.0, py::arg("dt") = 0.05,
    py::arg("frame") = CurveFrame{}, py::arg("goal_seq") = 0U, py::arg("goal") = py::none(),
    py::arg("obstacles") = py::none(), py::arg("speed_limit_scale") = 1.0,
    py::arg("cancel") = false, py::arg("cap_at_curve_end") = false);
  cls
    .def_property(
      "curve", [](const TrackerInputBox & b) { return b.curve; },
      [](TrackerInputBox & b, const std::shared_ptr<vf::Curve> & c) { b.curve = c; },
      "The curve (shared), or None.")
    .def_property(
      "pose", [](TrackerInputBox & b) -> vf::Pose2 & { return b.input.pose; },
      [](TrackerInputBox & b, const vf::Pose2 & p) { b.input.pose = p; }, "base_link pose.")
    .def_property(
      "measured", [](TrackerInputBox & b) -> vf::Twist2 & { return b.input.measured; },
      [](TrackerInputBox & b, const vf::Twist2 & u) { b.input.measured = u; },
      "Measured body twist.")
    .def_property(
      "t", [](const TrackerInputBox & b) { return b.input.t; },
      [](TrackerInputBox & b, double v) { b.input.t = v; }, "Time (s).")
    .def_property(
      "dt", [](const TrackerInputBox & b) { return b.input.dt; },
      [](TrackerInputBox & b, double v) { b.input.dt = v; }, "Time since the previous step (s).")
    .def_property(
      "frame", [](TrackerInputBox & b) -> CurveFrame & { return b.input.frame; },
      [](TrackerInputBox & b, const CurveFrame & f) { b.input.frame = f; }, "CurveFrame.")
    .def_property(
      "goal_seq", [](const TrackerInputBox & b) { return b.input.goal_seq; },
      [](TrackerInputBox & b, std::uint32_t v) { b.input.goal_seq = v; }, "Goal identity.")
    .def_property(
      "goal",
      [](TrackerInputBox & b) -> GoalInfo * { return b.input.goal ? &*b.input.goal : nullptr; },
      [](TrackerInputBox & b, const std::optional<GoalInfo> & g) { b.input.goal = g; },
      "GoalInfo of an open curve, or None.")
    .def_property(
      "obstacles",
      [](TrackerInputBox & b) -> ObstacleMapBox * { return b.obstacles ? &*b.obstacles : nullptr; },
      [](TrackerInputBox & b, const std::optional<ObstacleMapBox> & o) { b.obstacles = o; },
      "ObstacleMap, or None (no contour and no collision prediction).")
    .def_property(
      "speed_limit_scale", [](const TrackerInputBox & b) { return b.input.speed_limit_scale; },
      [](TrackerInputBox & b, double v) { b.input.speed_limit_scale = v; },
      "External speed limit in (0, 1].")
    .def_property(
      "cancel", [](const TrackerInputBox & b) { return b.input.cancel; },
      [](TrackerInputBox & b, bool v) { b.input.cancel = v; }, "A cancel is in progress.")
    .def_property(
      "cap_at_curve_end", [](const TrackerInputBox & b) { return b.input.cap_at_curve_end; },
      [](TrackerInputBox & b, bool v) { b.input.cap_at_curve_end = v; },
      "Apply the window-end approach cap.")
    .def_property(
      "drive_sign", [](const TrackerInputBox & b) { return static_cast<int>(b.input.drive_sign); },
      [](TrackerInputBox & b, int v) {
        b.input.drive_sign = static_cast<std::int8_t>(v < 0 ? -1 : (v > 0 ? 1 : 0));
      },
      "Drive sign of the cycle: 0 configured, +1 body +x, -1 body -x (a reverse plan segment).")
    .def("copy", [](const TrackerInputBox & b) { return b; })
    .def("__copy__", [](const TrackerInputBox & b) { return b; })
    .def("__deepcopy__", [](const TrackerInputBox & b, const py::dict &) { return b; });
}

void bindOutput(py::module_ & m)
{
  using Box = TrackerOutputBox;
  py::class_<Box>(m, "TrackerOutput", "Outputs of one control cycle.")
    .def_property_readonly(
      "cmd", [](const Box & b) { return b.output.cmd; }, "Final command (body twist).")
    .def_property_readonly(
      "cmd_target", [](const Box & b) { return b.output.cmd_target; },
      "Target after the kinematic box, before the CommandLimiter.")
    .def_property_readonly(
      "field", [](const Box & b) -> const vf::FieldSample & { return b.output.field; },
      "FieldSample at the law's evaluation point.")
    .def_property_readonly(
      "phase", [](const Box & b) { return b.output.phase; }, "Phase after this cycle.")
    .def_property_readonly(
      "status", [](const Box & b) { return b.output.status; },
      "Status bits (decode with vectorfield.Status).")
    .def_property_readonly("settled", [](const Box & b) { return b.output.settled; })
    .def_property_readonly(
      "s_free", [](const Box & b) { return b.output.s_free; }, "Free arc length (m).")
    .def_property_readonly(
      "cap", [](const Box & b) { return b.output.cap; }, "Braking cap (m/s); inf if none.")
    .def_property_readonly(
      "shaping", [](const Box & b) { return b.output.shaping; }, "SpeedShaping of the step.")
    .def_property_readonly("box_scale", [](const Box & b) { return b.output.box_scale; })
    .def_property_readonly("limiter_alpha", [](const Box & b) { return b.output.limiter_alpha; })
    .def_property_readonly(
      "predicted", [](const Box & b) { return fromPoses(b.predicted); },
      "Checked rollout poses (N, 3).")
    .def_property_readonly(
      "first_collision", [](const Box & b) { return b.output.first_collision; },
      "Index of the first colliding pose in `predicted`, or -1.")
    .def_property_readonly(
      "cancel_complete", [](const Box & b) { return b.output.cancel_complete; })
    .def("__repr__", [](const Box & b) {
      return "TrackerOutput(phase=" + std::string(vf::phaseName(b.output.phase)) +
             ", status=" + std::to_string(b.output.status) +
             ", D=" + py::repr(py::float_(b.output.field.D)).cast<std::string>() + ")";
    });

  py::class_<vf::StepScratch>(
    m, "StepScratch",
    "Caller-owned work buffers of Tracker.step(); read it after a step (copies are returned).")
    .def(py::init<>())
    .def(
      "reserve", &vf::StepScratch::reserve, py::arg("grid_cells"), py::arg("rollout_poses"),
      py::arg("gate_samples"))
    .def_property_readonly(
      "rollout", [](const vf::StepScratch & s) { return fromPoses(s.rollout); },
      "Checked poses of the final rollout (N, 3).")
    .def_property_readonly(
      "side_rollout", [](const vf::StepScratch & s) { return fromPoses(s.side_rollout); },
      "Checked poses of side-probe rollouts (N, 3).")
    .def_property_readonly(
      "k_mask",
      [](const vf::StepScratch & s) {
        const std::size_t cells =
          static_cast<std::size_t>(s.k_width) * static_cast<std::size_t>(s.k_height);
        if (cells == 0 || s.k_mask.size() < cells) {
          return fromMask({}, 0, 0);
        }
        return fromMask(
          vf::Span<const std::uint8_t>(s.k_mask.data(), cells), s.k_width, s.k_height);
      },
      "K (height, width), 1 = cell in the blocking component.")
    .def_readonly("k_valid", &vf::StepScratch::k_valid, "k_mask holds this cycle's K.")
    .def_property_readonly(
      "gate_samples", [](const vf::StepScratch & s) { return fromPoints(s.gate_samples); },
      "Gate samples (N, 2), evaluation frame.")
    .def_property_readonly(
      "gate_samples_grid",
      [](const vf::StepScratch & s) { return fromPoints(s.gate_samples_grid); },
      "Gate samples (N, 2), grid frame.")
    .def_property_readonly(
      "gate_blocked",
      [](const vf::StepScratch & s) {
        py::array_t<bool> out(static_cast<py::ssize_t>(s.gate_blocked.size()));
        for (std::size_t i = 0; i < s.gate_blocked.size(); ++i) {
          out.mutable_at(static_cast<py::ssize_t>(i)) = s.gate_blocked[i] != 0;
        }
        return out;
      },
      "Blocked flag of each gate sample (N,).")
    .def_property_readonly(
      "blocking",
      [](const vf::StepScratch & s) {
        py::array_t<double> stations(static_cast<py::ssize_t>(s.blocking.size()));
        std::vector<vf::Vec2> sites(s.blocking.size());
        for (std::size_t i = 0; i < s.blocking.size(); ++i) {
          stations.mutable_at(static_cast<py::ssize_t>(i)) = s.blocking[i].s;
          sites[i] = s.blocking[i].site_grid;
        }
        return py::make_tuple(stations, fromPoints(sites));
      },
      "Blocking set B: (stations (N,), feature sites (N, 2) in the grid frame).");
}

vf::StepScratch & scratchOf(TrackerBox & t, vf::StepScratch * scratch)
{
  return scratch != nullptr ? *scratch : t.scratch;
}

void requireCurve(const TrackerInputBox & input)
{
  if (!input.curve) {
    throw py::value_error("TrackerInput.curve is None");
  }
}

void bindTrackerClass(py::module_ & m)
{
  py::class_<TrackerBox>(
    m, "Tracker",
    "One control cycle of the vector-field controller. Owns only "
    "configuration, plus the StepScratch used when step() gets none.")
    .def(
      py::init([](const vf::CoreConfig & config) { return TrackerBox{vf::Tracker(config), {}}; }),
      py::arg("config"),
      "Validates and resolves a configuration; raises ConfigError listing the errors.")
    .def_static(
      "initial_state", &vf::Tracker::initialState,
      "The state of a fresh start: FOLLOW, no hint, limiter re-seed requested.")
    .def(
      "step",
      [](
        TrackerBox & t, TrackerInputBox & input, const vf::TrackerState & state,
        vf::StepScratch * scratch) {
        requireCurve(input);
        vf::TrackerState next;
        vf::StepScratch & s = scratchOf(t, scratch);
        const vf::TrackerOutput out = t.tracker.step(input.resolved(), state, &next, s);
        return py::make_tuple(TrackerOutputBox(out), next);
      },
      py::arg("input"), py::arg("state"), py::arg("scratch") = py::none(),
      "One control cycle: (TrackerOutput, next TrackerState). The state passed in is unchanged.")
    .def(
      "probe",
      [](
        TrackerBox & t, const vf::Vec2 & x, TrackerInputBox & input, const vf::TrackerState & state,
        double time_offset, vf::StepScratch * scratch) {
        requireCurve(input);
        return t.tracker.probe(x, input.resolved(), state, scratchOf(t, scratch), time_offset);
      },
      py::arg("x"), py::arg("input"), py::arg("state"), py::arg("time_offset") = 0.0,
      py::arg("scratch") = py::none(),
      "Field probe at x with the state's hint and the scratch's frozen K and gate. Call after "
      "step() with the same input, the state step() returned and the same scratch.")
    .def(
      "probe_points",
      [](
        TrackerBox & t, const DoubleArray & points, TrackerInputBox & input,
        const vf::TrackerState & state, double time_offset, vf::StepScratch * scratch) {
        requireCurve(input);
        const std::vector<vf::Vec2> xs = toPoints(points, "points");
        const vf::TrackerInput & in = input.resolved();
        const vf::StepScratch & s = scratchOf(t, scratch);
        std::vector<vf::Vec2> F(xs.size());
        const auto n = static_cast<py::ssize_t>(xs.size());
        py::array_t<double> psi_d(n);
        py::array_t<double> s_star(n);
        py::array_t<double> D(n);
        py::array_t<bool> contour_active(n);
        py::array_t<std::int8_t> sigma_c(n);
        py::array_t<bool> cbf_active(n);
        for (std::size_t i = 0; i < xs.size(); ++i) {
          const vf::ProbeSample p = t.tracker.probe(xs[i], in, state, s, time_offset);
          const auto k = static_cast<py::ssize_t>(i);
          F[i] = p.F;
          psi_d.mutable_at(k) = p.psi_d;
          s_star.mutable_at(k) = p.s_star;
          D.mutable_at(k) = p.D;
          contour_active.mutable_at(k) = p.contour_active;
          sigma_c.mutable_at(k) = p.sigma_c;
          cbf_active.mutable_at(k) = p.cbf_active;
        }
        py::dict out;
        out["F"] = fromPoints(F);
        out["psi_d"] = psi_d;
        out["s_star"] = s_star;
        out["D"] = D;
        out["contour_active"] = contour_active;
        out["sigma_c"] = sigma_c;
        out["cbf_active"] = cbf_active;
        return out;
      },
      py::arg("points"), py::arg("input"), py::arg("state"), py::arg("time_offset") = 0.0,
      py::arg("scratch") = py::none(),
      "probe() at each row of an (N, 2) array: a dict of F (N, 2) and psi_d, s_star, D, "
      "contour_active, sigma_c, cbf_active (N,). For field plots.")
    .def(
      "streamline",
      [](
        TrackerBox & t, const vf::Vec2 & start, TrackerInputBox & input,
        const vf::TrackerState & state, double step, std::size_t max_points, double time_offset,
        vf::StepScratch * scratch) {
        requireCurve(input);
        const vf::TrackerInput & in = input.resolved();
        const vf::StepScratch & s = scratchOf(t, scratch);
        const auto field = [&](const vf::Vec2 & x) {
          return t.tracker.probe(x, in, state, s, time_offset).F;
        };
        std::vector<vf::Vec2> out(max_points);
        out.resize(vf::streamline(field, start, step, out));
        return fromPoints(out);
      },
      py::arg("start"), py::arg("input"), py::arg("state"), py::arg("step") = 0.05,
      py::arg("max_points") = 1000, py::arg("time_offset") = 0.0, py::arg("scratch") = py::none(),
      "RK4 streamline (N, 2) of the probed field F from start, as streamline().")
    .def_property_readonly(
      "config", [](const TrackerBox & t) { return t.tracker.config(); },
      "The resolved CoreConfig (a copy).")
    .def_property_readonly(
      "validation", [](const TrackerBox & t) { return t.tracker.validation(); },
      "Warnings of validate_config() at construction.")
    .def_property_readonly(
      "kinematics", [](const TrackerBox & t) { return t.tracker.kinematics(); }, "Kinematics.")
    .def_property_readonly(
      "gain", [](const TrackerBox & t) { return t.tracker.gain(); }, "GainFunction.")
    .def_property_readonly(
      "footprint", [](const TrackerBox & t) { return t.tracker.footprint(); }, "Footprint.")
    .def_property_readonly(
      "law", [](const TrackerBox & t) { return t.tracker.law(); }, "ControlLaw.")
    .def_property_readonly(
      "scratch", [](TrackerBox & t) -> vf::StepScratch & { return t.scratch; },
      py::return_value_policy::reference_internal, "The StepScratch used when none is passed.")
    .def("copy", [](const TrackerBox & t) { return TrackerBox{t.tracker, {}}; })
    .def("__copy__", [](const TrackerBox & t) { return TrackerBox{t.tracker, {}}; })
    .def("__deepcopy__", [](const TrackerBox & t, const py::dict &) {
      return TrackerBox{t.tracker, {}};
    });
}

// ---------------------------------------------------------------------------------------------
// Simulator

/// A native disturbance (no Python call per period).
struct DisturbanceBox
{
  vf::sim::Disturbance function;
};

/// SimulationOptions with the disturbance as a Python object: None, a Disturbance or a callable
/// disturbance(t, pose) -> (2,).
struct SimulationOptionsBox
{
  vf::sim::SimulationOptions options;
  py::object disturbance = py::none();

  /// The core options; `*python` is set when the disturbance calls Python.
  vf::sim::SimulationOptions toCore(bool * python) const
  {
    vf::sim::SimulationOptions out = options;
    out.disturbance = nullptr;
    *python = false;
    if (disturbance.is_none()) {
      return out;
    }
    if (py::isinstance<DisturbanceBox>(disturbance)) {
      out.disturbance = disturbance.cast<const DisturbanceBox &>().function;
      return out;
    }
    // The Python object is released with the GIL held, whichever thread drops the function.
    const std::shared_ptr<py::object> fn(new py::object(disturbance), [](py::object * o) {
      const py::gil_scoped_acquire gil;
      delete o;
    });
    out.disturbance = [fn](double t, const vf::Pose2 & pose) {
      const py::gil_scoped_acquire gil;
      return (*fn)(t, pose).cast<vf::Vec2>();
    };
    *python = true;
    return out;
  }
};

struct SimulatorBox
{
  vf::sim::ClosedLoopSimulator simulator;
  bool python_disturbance{false};
};

/// Columns of a record range [begin, records.size()).
struct SimulationResultBox
{
  py::array_t<double> t, D, sigma, s_star, s_free;
  py::array_t<double> pose, measured, cmd, x_eval, F;
  py::array_t<std::uint8_t> phase;
  py::array_t<std::uint32_t> status;
  py::array_t<bool> settled;

  SimulationResultBox(const std::vector<vf::sim::SimulationRecord> & records, std::size_t begin)
  {
    const std::size_t n = records.size() - std::min(begin, records.size());
    const auto count = static_cast<py::ssize_t>(n);
    t = py::array_t<double>(count);
    D = py::array_t<double>(count);
    sigma = py::array_t<double>(count);
    s_star = py::array_t<double>(count);
    s_free = py::array_t<double>(count);
    phase = py::array_t<std::uint8_t>(count);
    status = py::array_t<std::uint32_t>(count);
    settled = py::array_t<bool>(count);
    std::vector<vf::Pose2> poses(n);
    std::vector<vf::Pose2> measured_twists(n);
    std::vector<vf::Pose2> commands(n);
    std::vector<vf::Vec2> points(n);
    std::vector<vf::Vec2> fields(n);
    for (std::size_t i = 0; i < n; ++i) {
      const vf::sim::SimulationRecord & r = records[begin + i];
      const auto k = static_cast<py::ssize_t>(i);
      t.mutable_at(k) = r.t;
      D.mutable_at(k) = r.D;
      sigma.mutable_at(k) = r.sigma;
      s_star.mutable_at(k) = r.s_star;
      s_free.mutable_at(k) = r.s_free;
      phase.mutable_at(k) = vf::toUnderlying(r.phase);
      status.mutable_at(k) = r.status;
      settled.mutable_at(k) = r.settled;
      poses[i] = r.pose;
      // Twists as [vx, vy, wz] rows (same (N, 3) layout as poses).
      measured_twists[i] = vf::Pose2{vf::Vec2(r.measured.vx, r.measured.vy), r.measured.wz};
      commands[i] = vf::Pose2{vf::Vec2(r.cmd.vx, r.cmd.vy), r.cmd.wz};
      points[i] = r.x_eval;
      fields[i] = r.F;
    }
    pose = fromPoses(poses);
    measured = fromPoses(measured_twists);
    cmd = fromPoses(commands);
    x_eval = fromPoints(points);
    F = fromPoints(fields);
  }
};

void bindSimulator(py::module_ & sim)
{
  py::class_<DisturbanceBox>(sim, "Disturbance", "A native world-frame velocity disturbance (m/s).")
    .def(
      "__call__",
      [](const DisturbanceBox & d, double t, const vf::Pose2 & pose) -> vf::Vec2 {
        return d.function(t, pose);
      },
      py::arg("t"), py::arg("pose"));
  sim.def(
    "bounded_random_disturbance",
    [](double bound, std::uint64_t seed) {
      return DisturbanceBox{vf::sim::boundedRandomDisturbance(bound, seed)};
    },
    py::arg("bound"), py::arg("seed"),
    "Uniform in the disc of radius `bound` (m/s), redrawn each period; reproducible per seed.");

  py::class_<SimulationOptionsBox>(sim, "SimulationOptions", "Options of a closed-loop simulation.")
    .def(
      py::init([](
                 double period, const vf::sim::SlipParams & slip, const py::object & disturbance,
                 bool stop_when_settled, bool stop_when_stopped) {
        SimulationOptionsBox box;
        box.options.period = period;
        box.options.slip = slip;
        box.disturbance = disturbance;
        box.options.stop_when_settled = stop_when_settled;
        box.options.stop_when_stopped = stop_when_stopped;
        return box;
      }),
      py::kw_only(), py::arg("period") = 0.05, py::arg("slip") = vf::sim::SlipParams{},
      py::arg("disturbance") = py::none(), py::arg("stop_when_settled") = true,
      py::arg("stop_when_stopped") = true)
    .def_property(
      "period", [](const SimulationOptionsBox & b) { return b.options.period; },
      [](SimulationOptionsBox & b, double v) { b.options.period = v; },
      "Control period: the ZOH step and TrackerInput.dt (s).")
    .def_property(
      "slip", [](SimulationOptionsBox & b) -> vf::sim::SlipParams & { return b.options.slip; },
      [](SimulationOptionsBox & b, const vf::sim::SlipParams & s) { b.options.slip = s; },
      "SlipParams of the plant.")
    .def_readwrite(
      "disturbance", &SimulationOptionsBox::disturbance,
      "None, a Disturbance, or a callable disturbance(t, pose) -> (2,) (m/s, world frame).")
    .def_property(
      "stop_when_settled",
      [](const SimulationOptionsBox & b) { return b.options.stop_when_settled; },
      [](SimulationOptionsBox & b, bool v) { b.options.stop_when_settled = v; })
    .def_property(
      "stop_when_stopped",
      [](const SimulationOptionsBox & b) { return b.options.stop_when_stopped; },
      [](SimulationOptionsBox & b, bool v) { b.options.stop_when_stopped = v; });

  using vf::sim::SimulationRecord;
  StructBinder<SimulationRecord>(sim, "SimulationRecord", "One recorded control period.")
    .field("t", &SimulationRecord::t, "Time at the start of the period (s).")
    .field("pose", &SimulationRecord::pose, "base_link pose at the start of the period.")
    .field("measured", &SimulationRecord::measured, "Measured twist given to the Tracker.")
    .field("cmd", &SimulationRecord::cmd, "Command held over the period.")
    .field("phase", &SimulationRecord::phase, "Phase.")
    .field("status", &SimulationRecord::status, "Status bits.")
    .field("settled", &SimulationRecord::settled, "Settled.")
    .field("D", &SimulationRecord::D, "Distance to the curve at the law's point.")
    .field("sigma", &SimulationRecord::sigma, "Remaining arc length (NaN closed).")
    .field("s_star", &SimulationRecord::s_star, "Closest-point station.")
    .field("x_eval", &SimulationRecord::x_eval, "Law's evaluation point.")
    .field("F", &SimulationRecord::F, "Tracked field.")
    .field("s_free", &SimulationRecord::s_free, "Free arc length (m).")
    .finish();

  const char * const columns[] = {"t", "pose",  "measured", "cmd",    "phase", "status", "settled",
                                  "D", "sigma", "s_star",   "x_eval", "F",     "s_free"};
  py::class_<SimulationResultBox> result(
    sim, "SimulationResult",
    "Recorded periods as NumPy columns: t, D, sigma, s_star, s_free (N,); pose (N, 3) [x, y, "
    "yaw]; measured and cmd (N, 3) [vx, vy, wz]; x_eval and F (N, 2); phase (N,) uint8 (compare "
    "with int(Phase.X)); status (N,) uint32; settled (N,) bool.");
  result.def_readonly("t", &SimulationResultBox::t)
    .def_readonly("pose", &SimulationResultBox::pose)
    .def_readonly("measured", &SimulationResultBox::measured)
    .def_readonly("cmd", &SimulationResultBox::cmd)
    .def_readonly("phase", &SimulationResultBox::phase)
    .def_readonly("status", &SimulationResultBox::status)
    .def_readonly("settled", &SimulationResultBox::settled)
    .def_readonly("D", &SimulationResultBox::D)
    .def_readonly("sigma", &SimulationResultBox::sigma)
    .def_readonly("s_star", &SimulationResultBox::s_star)
    .def_readonly("x_eval", &SimulationResultBox::x_eval)
    .def_readonly("F", &SimulationResultBox::F)
    .def_readonly("s_free", &SimulationResultBox::s_free)
    .def("__len__", [](const SimulationResultBox & r) { return r.t.size(); })
    .def(
      "as_dict",
      [columns](py::handle self) {
        py::dict out;
        for (const char * name : columns) {
          out[name] = self.attr(name);
        }
        return out;
      },
      "The columns as a {name: array} dict.")
    .def("__repr__", [](const SimulationResultBox & r) {
      return "SimulationResult(periods=" + std::to_string(r.t.size()) + ")";
    });

  py::class_<SimulatorBox>(
    sim, "ClosedLoopSimulator",
    "Zero-order-hold loop TrackerInput -> Tracker.step -> integrate, repeated. Owns a copy of "
    "the Tracker, the state, a StepScratch and the plant pose.")
    .def(
      py::init([](
                 const TrackerBox & tracker, const SimulationOptionsBox & options,
                 const std::optional<vf::Kinematics> & plant) {
        bool python = false;
        const vf::sim::SimulationOptions core = options.toCore(&python);
        if (plant) {
          return std::make_unique<SimulatorBox>(
            SimulatorBox{vf::sim::ClosedLoopSimulator(tracker.tracker, *plant, core), python});
        }
        return std::make_unique<SimulatorBox>(
          SimulatorBox{vf::sim::ClosedLoopSimulator(tracker.tracker, core), python});
      }),
      py::arg("tracker"), py::arg("options") = SimulationOptionsBox{},
      py::arg("plant") = py::none(),
      "A simulator of a copy of `tracker`; the plant defaults to the tracker's kinematics.")
    .def(
      "reset",
      [](SimulatorBox & s, const vf::Pose2 & pose, double t0, const vf::Twist2 & twist) {
        s.simulator.reset(pose, t0, twist);
      },
      py::arg("pose") = vf::Pose2{}, py::arg("t0") = 0.0, py::arg("twist") = vf::Twist2{},
      "Resets the time, pose, twist, state and records.")
    .def(
      "step",
      [](SimulatorBox & s, const TrackerInputBox & input) {
        requireCurve(input);
        TrackerInputBox local = input;
        return s.simulator.step(local.resolved());
      },
      py::arg("input"), "One control period: its SimulationRecord.")
    .def(
      "run",
      [](
        SimulatorBox & s, const TrackerInputBox & input, double duration,
        const py::object & update) {
        requireCurve(input);
        const std::size_t begin = s.simulator.records().size();
        // A copy shares the curve and obstacles, so they live until the run ends.
        TrackerInputBox local = input;
        const vf::TrackerInput & core = local.resolved();
        if (update.is_none()) {
          if (s.python_disturbance) {
            s.simulator.run(core, duration);
          } else {
            const py::gil_scoped_release release;
            s.simulator.run(core, duration);
          }
          return SimulationResultBox(s.simulator.records(), begin);
        }
        // The hook edits a Python TrackerInput mirroring the input the C++ loop carries over.
        py::object hooked = py::cast(local, py::return_value_policy::copy);
        auto & box = hooked.cast<TrackerInputBox &>();
        s.simulator.run(core, duration, [&](double t, vf::TrackerInput * current) {
          box.input = *current;
          update(t, hooked);
          requireCurve(box);
          *current = box.resolved();
        });
        return SimulationResultBox(s.simulator.records(), begin);
      },
      py::arg("input"), py::arg("duration"), py::arg("update") = py::none(),
      "Runs until `duration` elapses or a stop condition holds; update(t, input) may edit the "
      "input before each period. Returns the SimulationResult of this run. Releases the GIL "
      "when no Python callback is involved.")
    .def_property_readonly(
      "pose", [](const SimulatorBox & s) { return s.simulator.pose(); }, "Current pose.")
    .def_property_readonly(
      "time", [](const SimulatorBox & s) { return s.simulator.time(); }, "Current time (s).")
    .def_property_readonly(
      "state", [](const SimulatorBox & s) { return s.simulator.state(); },
      "Current TrackerState (a copy).")
    .def_property_readonly(
      "scratch",
      [](const SimulatorBox & s) -> const vf::StepScratch & { return s.simulator.scratch(); },
      py::return_value_policy::reference_internal, "StepScratch of the last step.")
    .def_property_readonly(
      "last_output",
      [](const SimulatorBox & s) { return TrackerOutputBox(s.simulator.lastOutput()); },
      "TrackerOutput of the last step.")
    .def(
      "records",
      [](const SimulatorBox & s) { return SimulationResultBox(s.simulator.records(), 0); },
      "Every period recorded since reset().")
    .def_property_readonly(
      "tracker", [](const SimulatorBox & s) { return TrackerBox{s.simulator.tracker(), {}}; },
      "A copy of the simulated Tracker.");
}

}  // namespace

void bindTracker(py::module_ & m, py::module_ & sim)
{
  bindState(m);
  bindInput(m);
  bindOutput(m);
  bindTrackerClass(m);
  bindSimulator(sim);
}

}  // namespace vectorfield_py
