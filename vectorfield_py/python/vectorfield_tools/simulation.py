# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Closed-loop scenarios on vectorfield.sim.ClosedLoopSimulator, and their summaries."""

import math
from dataclasses import dataclass, field

import numpy as np
import vectorfield as vf

from .curves import bounds, sample
from .obstacles import DEFAULT_RESOLUTION, Obstacles

MODELS = ('differential', 'skid_steer', 'omnidirectional', 'ackermann')
LAWS = ('auto', 'feedback_linearization', 'heading_tracking', 'holonomic')

#: vectorfield_sim's skid-steer plant slip (eta_psi, x_icr).
SKID_STEER_SLIP = (0.75, 0.05)
#: The Ackermann overlay's min_turning_radius (m), used when the configuration leaves it unset.
ACKERMANN_TURNING_RADIUS = 0.5
#: Default |D| tolerance of the settle time (m).
DEFAULT_TOLERANCE = 0.05


def configure(config, model=None, law=None, speed=None, gain=None, explicit=()):
    """Apply the common simulation options to ``config``; returns notes on implied changes."""
    notes = []
    if model is not None:
        config.motion.motion_model = model
    if law is not None:
        config.motion.tracking_law = law
    if gain is not None:
        config.field.convergence_gain = gain
    if speed is not None:
        config.field.desired_speed = speed
        if speed > config.motion.max_linear_vel and 'max_linear_vel' not in explicit:
            config.motion.max_linear_vel = speed
            notes.append(f'max_linear_vel raised to the speed, {speed:g} m/s')
    is_ackermann = config.motion.motion_model == vf.MotionModel.ACKERMANN
    if is_ackermann and config.motion.min_turning_radius <= 0.0:
        if 'min_turning_radius' not in explicit:
            config.motion.min_turning_radius = ACKERMANN_TURNING_RADIUS
            notes.append(f'min_turning_radius {ACKERMANN_TURNING_RADIUS} m (Ackermann default)')
    return notes


def default_slip(config):
    """Return the plant slip vectorfield_sim uses for the configured model."""
    if config.motion.motion_model == vf.MotionModel.SKID_STEER:
        return SKID_STEER_SLIP
    return (1.0, 0.0)


def default_duration(curve, speed):
    """Two laps of a closed curve, or its length plus 15 s for an open one (s)."""
    travel = curve.length() / max(speed, 1e-3)
    return float(np.clip(2.0 * travel, 10.0, 300.0) if curve.is_closed else travel + 15.0)


@dataclass
class Scenario:
    """A closed-loop run: configuration, curve, start pose and plant."""

    config: vf.CoreConfig
    curve: object
    pose: tuple
    duration: float
    period: float = 0.0
    obstacles: Obstacles = field(default_factory=Obstacles)
    slip: tuple = (1.0, 0.0)
    xy_tolerance: float = 0.25
    yaw_tolerance: float = 0.25
    resolution: float = DEFAULT_RESOLUTION
    t0: float = 0.0

    def control_period(self):
        """Return the ZOH period: ``period``, or 1 / controller_frequency."""
        return self.period if self.period > 0.0 else 1.0 / self.config.node.controller_frequency

    def extent(self, margin=2.0):
        """(xmin, xmax, ymin, ymax) covering the curve, the start and the obstacles."""
        points = np.vstack(
            [sample(self.curve, t=self.t0), [self.pose[:2]], self.obstacles.outline_points()]
        )
        return bounds(points, margin)

    def goal(self):
        """Return the GoalInfo of an open curve (its end, along the tangent), or None."""
        if self.curve.is_closed:
            return None
        if self.config.field.direction < 0:
            p, tangent = self.curve.point_at(0.0), -np.asarray(self.curve.tangent_at(0.0))
        else:
            end = self.curve.end_station()
            p, tangent = self.curve.point_at(end), np.asarray(self.curve.tangent_at(end))
        yaw = math.atan2(tangent[1], tangent[0])
        return vf.GoalInfo(
            pose=(float(p[0]), float(p[1]), yaw),
            has_yaw=True,
            xy_tol=self.xy_tolerance,
            yaw_tol=self.yaw_tolerance,
        )


@dataclass
class Run:
    """The outcome of simulate()."""

    scenario: Scenario
    result: object
    tracker: object
    obstacle_map: object = None
    contour_active: np.ndarray = None
    contour_side: np.ndarray = None
    cbf_active: np.ndarray = None
    cbf_barrier: np.ndarray = None


def simulate(scenario):
    """Run a Scenario; with obstacles, the contour and CBF activity is recorded per period."""
    period = scenario.control_period()
    tracker = vf.Tracker(scenario.config)
    eta_psi, x_icr = scenario.slip
    options = vf.sim.SimulationOptions(
        period=period, slip=vf.sim.SlipParams(eta_psi=eta_psi, x_icr=x_icr)
    )
    simulator = vf.sim.ClosedLoopSimulator(tracker, options)
    simulator.reset(tuple(scenario.pose), scenario.t0)
    obstacle_map = None
    if scenario.obstacles:
        obstacle_map = scenario.obstacles.obstacle_map(scenario.extent(), scenario.resolution)
    tracker_input = vf.TrackerInput(
        scenario.curve, goal=scenario.goal(), obstacles=obstacle_map, dt=period
    )
    if obstacle_map is None:
        result = simulator.run(tracker_input, scenario.duration)
        return Run(scenario, result, simulator.tracker)
    active, side, cbf_active, cbf_barrier = [], [], [], []
    for _ in range(int(round(scenario.duration / period))):
        record = simulator.step(tracker_input)
        field_sample = simulator.last_output.field
        active.append(bool(field_sample.contour.active))
        side.append(int(field_sample.contour.sigma_c))
        cbf_active.append(bool(field_sample.cbf.active))
        cbf_barrier.append(float(field_sample.cbf.h) if field_sample.cbf.on else math.nan)
        stopped = options.stop_when_stopped and record.phase == vf.Phase.STOPPED
        if (options.stop_when_settled and record.settled) or stopped:
            break
    return Run(
        scenario,
        simulator.records(),
        simulator.tracker,
        obstacle_map,
        np.array(active, dtype=bool),
        np.array(side, dtype=np.int8),
        np.array(cbf_active, dtype=bool),
        np.array(cbf_barrier, dtype=float),
    )


def settle_time(t, D, tolerance=DEFAULT_TOLERANCE):
    """First time after which |D| stays within ``tolerance``; None if it ends outside."""
    outside = np.flatnonzero(~(np.abs(D) <= tolerance))
    if outside.size == 0:
        return float(t[0])
    if outside[-1] == len(t) - 1:
        return None
    return float(t[outside[-1] + 1])


@dataclass
class Summary:
    """Convergence figures of a Run."""

    duration: float
    final_D: float
    max_D: float
    transient: float
    settle_time: object
    tolerance: float
    phases: dict
    status: object
    settled: bool
    final_pose: tuple
    goal_distance: object = None
    min_obstacle_distance: object = None
    contour_time: float = 0.0
    cbf_time: float = 0.0
    min_cbf_barrier: object = None

    def lines(self):
        """Human-readable summary lines."""
        settle = 'never' if self.settle_time is None else f'{self.settle_time:.2f} s'
        phases = ', '.join(f'{name} {seconds:.2f} s' for name, seconds in self.phases.items())
        x, y, yaw = self.final_pose
        out = [
            f'simulated            {self.duration:.2f} s',
            f'final |D|            {format_metres(self.final_D)}',
            f'max |D| after {self.transient:.1f} s  {format_metres(self.max_D)}',
            f'settle time          {settle} (|D| <= {self.tolerance:g} m)',
            f'phases               {phases}',
            f'status bits          {_status_names(self.status)}',
            f'final pose           x {x:.3f} m, y {y:.3f} m, yaw {math.degrees(yaw):.1f} deg',
        ]
        if self.goal_distance is not None:
            settled = 'settled' if self.settled else 'not settled'
            out.append(f'goal distance        {self.goal_distance:.3f} m ({settled})')
        if self.min_obstacle_distance is not None:
            out.append(f'closest obstacle     {self.min_obstacle_distance:.3f} m from base_link')
            out.append(f'contour active       {self.contour_time:.2f} s')
        if self.min_cbf_barrier is not None:
            out.append(f'cbf active           {self.cbf_time:.2f} s')
            out.append(f'min cbf barrier h    {self.min_cbf_barrier:.3f} m')
        return out


def format_metres(value):
    """Format a distance in metres, in scientific notation below 1 mm."""
    return f'{value:.4f} m' if not 0.0 < abs(value) < 1e-3 else f'{value:.1e} m'


def _status_names(status):
    names = [flag.name for flag in vf.Status if flag.value and flag in status]
    return ', '.join(names) if names else 'none'


def summarize(run, transient=None, tolerance=DEFAULT_TOLERANCE):
    """Summarize a Run; ``transient`` (s) defaults to half the simulated time."""
    result = run.result
    t, D = result.t, np.abs(result.D)
    if len(t) == 0:
        raise ValueError('the simulation recorded no period')
    period = run.scenario.control_period()
    duration = float(t[-1] - t[0] + period)
    transient = 0.5 * duration if transient is None else float(transient)
    after = D[t - t[0] >= transient]
    max_D = float(np.nanmax(after)) if after.size else float('nan')
    counts = np.bincount(result.phase.astype(int), minlength=len(vf.Phase.__members__))
    phases = {
        phase.name: float(counts[int(phase)] * period)
        for phase in vf.Phase.__members__.values()
        if counts[int(phase)]
    }
    status = vf.Status(int(np.bitwise_or.reduce(result.status)) if len(result.status) else 0)
    goal = run.scenario.goal()
    goal_distance = None
    if goal is not None:
        goal_distance = float(np.hypot(*(result.pose[-1, :2] - [goal.pose.x, goal.pose.y])))
    min_obstacle = None
    contour_time = 0.0
    if run.obstacle_map is not None:
        query = run.obstacle_map.edt.query_points(result.pose[:, :2])
        min_obstacle = float(np.min(query['D_o']))
        contour_time = float(np.count_nonzero(run.contour_active) * period)
    cbf_time = 0.0
    min_cbf = None
    if run.cbf_barrier is not None and np.isfinite(run.cbf_barrier).any():
        cbf_time = float(np.count_nonzero(run.cbf_active) * period)
        min_cbf = float(np.nanmin(run.cbf_barrier))
    return Summary(
        duration=duration,
        final_D=float(D[-1]),
        max_D=max_D,
        transient=transient,
        settle_time=settle_time(t - t[0], D, tolerance),
        tolerance=tolerance,
        phases=phases,
        status=status,
        settled=bool(result.settled[-1]),
        final_pose=tuple(float(v) for v in result.pose[-1]),
        goal_distance=goal_distance,
        min_obstacle_distance=min_obstacle,
        contour_time=contour_time,
        cbf_time=cbf_time,
        min_cbf_barrier=min_cbf,
    )
