#!/usr/bin/env python3
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

"""Compare the vector-field controller with RPP, MPPI and Graceful on the M3 route.

Each run starts ``loopback_demo.launch.py`` (Jazzy and later) on ``open_10x10`` headless, with a
scenario overlay that replaces ``FollowPath`` by the controller under test, drives the fixed 3-goal
route of :data:`vectorfield_system_tests.checks.ROUTE` with NavigateToPose, and stops the launch.
Every controller gets the same limits (:data:`SHARED_LIMITS`), goal checker, progress checker,
velocity smoother and collision monitor; everything else is the controller's stock configuration.

Metrics, computed the same way for every controller:

- cross-track error: distance from the true position (map -> base_footprint; the loopback
  simulator localizes perfectly) to the first global plan published on ``/plan`` for the goal,
  RMS and maximum over the pose samples from goal send to the result;
- time to goal: simulated time from goal send to SUCCEEDED, summed over the route;
- RMS jerk: second finite differences of the commanded linear and angular velocity, one sample
  per control update (:data:`BURST_GAP`), on the controller output (``cmd_vel_nav``) and on the
  final command after the smoother and collision monitor (``cmd_vel``).

Example::

    ros2 run vectorfield_system_tests compare_controllers.py --runs 5 --out-md page.md

``--from-csv`` regenerates the page from a recorded CSV without running anything.
"""

import argparse
import collections
import copy
import csv
import datetime
import functools
import json
import math
import os
import platform
import re
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def _add_helpers_to_path():
    """Make the ``vectorfield_system_tests`` helpers importable.

    They sit next to the installed script, or one level up in the source tree (where a
    ``--symlink-install`` script resolves to).
    """
    here = Path(__file__).resolve().parent
    for candidate in (here, here.parent):
        if (candidate / 'vectorfield_system_tests' / 'harness.py').is_file():
            if str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
            return


_add_helpers_to_path()

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402
from rclpy.parameter import parameter_value_to_python  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from vectorfield_system_tests.harness import STATUS_NAMES, NavHarness  # noqa: E402

from vectorfield_bringup import demo, params  # noqa: E402
from vectorfield_system_tests import checks  # noqa: E402

#: Controllers, in table order: key -> (name, plugin).
CONTROLLERS = {
    'vf': ('Vector field', 'vectorfield_nav2_controller::VectorFieldController'),
    'rpp': (
        'Regulated Pure Pursuit',
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    ),
    'mppi': ('MPPI', 'nav2_mppi_controller::MPPIController'),
    'graceful': ('Graceful', 'nav2_graceful_controller::GracefulController'),
}

#: Limits shared by every controller: the differential overlay's speed, yaw rate and
#: accelerations, with the forward speed limited to its desired_speed.
SHARED_LIMITS = {
    'Forward speed (m/s)': 0.4,
    'Yaw rate (rad/s)': 1.5,
    'Linear acceleration and deceleration (m/s²)': 1.0,
    'Angular acceleration (rad/s²)': 3.2,
}
_SPEED, _YAW_RATE, _ACCEL, _YAW_ACCEL = SHARED_LIMITS.values()

#: FollowPath parameters set on top of each controller's base block: the shared limits, expressed
#: with each controller's own parameters. A limit a controller has no parameter for is left to the
#: shared velocity smoother.
OVERRIDES = {
    'vf': {
        'desired_speed': _SPEED,
        'max_linear_vel': _SPEED,
        'max_angular_vel': _YAW_RATE,
        'max_linear_accel': _ACCEL,
        'max_linear_decel': _ACCEL,
        'max_angular_accel': _YAW_ACCEL,
    },
    'rpp': {
        'desired_linear_vel': _SPEED,
        'rotate_to_heading_angular_vel': _YAW_RATE,
        'max_angular_accel': _YAW_ACCEL,
    },
    'mppi': {
        'vx_max': _SPEED,
        'wz_max': _YAW_RATE,
        'ax_max': _ACCEL,
        'ax_min': -_ACCEL,
        'az_max': _YAW_ACCEL,
    },
    'graceful': {'v_linear_max': _SPEED, 'v_angular_max': _YAW_RATE},
}

#: Where each controller's FollowPath block comes from, before OVERRIDES.
BASE_SOURCES = {
    'vf': 'FollowPath of vectorfield_bringup `config/controller/differential.yaml`',
    'rpp': 'the plugin defaults (`nav2_params.yaml` has no RPP block)',
    'mppi': 'FollowPath of nav2_bringup `params/nav2_params.yaml`',
    'graceful': 'the plugin defaults (`nav2_params.yaml` has no Graceful block)',
}

#: Velocity commands recorded: the controller output and the final command.
COMMAND_TOPICS = ('/cmd_vel_nav', '/cmd_vel')

#: The map and the motion model of every run.
MAP = 'open_10x10'
MOTION_MODEL = 'differential'

#: A /plan belongs to a goal when it ends within this distance of the goal (m).
PLAN_END_TOLERANCE = 0.25

#: Wall seconds given to the launch to exit on SIGINT, then on SIGTERM, before SIGKILL.
SIGINT_TIMEOUT = 20.0
SIGTERM_TIMEOUT = 10.0

#: Commands received less than this apart (wall s) count as one control update, the later of which
#: the robot keeps. The controller server publishes about every 50 ms, with occasional
#: back-to-back commands when its loop catches up.
BURST_GAP = 0.02

#: Metric columns: key -> (header, unit, format).
METRICS = {
    'time': ('Time to goal, route', 's', '.1f'),
    'xte_rms': ('Cross-track RMS', 'm', '.3f'),
    'xte_max': ('Cross-track max', 'm', '.3f'),
    'jerk_v_nav': ('RMS jerk, linear', 'm/s³', '.2f'),
    'jerk_w_nav': ('RMS jerk, angular', 'rad/s³', '.2f'),
    'jerk_v_out': ('RMS jerk, linear', 'm/s³', '.2f'),
    'jerk_w_out': ('RMS jerk, angular', 'rad/s³', '.2f'),
}

CSV_FIELDS = (
    'controller',
    'run',
    'attempt',
    'repeated',
    'scope',
    'status',
    'time',
    'xte_rms',
    'xte_max',
    'jerk_v_nav',
    'jerk_w_nav',
    'jerk_v_out',
    'jerk_w_out',
    'pose_samples',
    'period_nav',
    'period_out',
    'note',
)


# Recording -----------------------------------------------------------------------------------


class RecordingHarness(NavHarness):
    """A :class:`NavHarness` that also keeps the /plan contents and the velocity commands."""

    def __init__(self):
        self.plan_messages = []
        self.commands = {topic: [] for topic in COMMAND_TOPICS}
        super().__init__(ground_truth=False)

    def _on_plan(self, message):
        super()._on_plan(message)
        points = np.array(
            [(p.pose.position.x, p.pose.position.y) for p in message.poses], dtype=float
        ).reshape(-1, 2)
        with self._lock:
            self.plan_messages.append((self.now(), message.header.frame_id, points))

    def _on_command(self, topic, stamped, message):
        twist = message.twist if stamped else message
        sample = (self.now(), time.monotonic(), twist.linear.x, twist.angular.z)
        with self._lock:
            self.commands[topic].append(sample)

    def record_commands(self, timeout=30.0):
        """Subscribe to :data:`COMMAND_TOPICS` with the type each one is published with."""
        deadline = time.monotonic() + timeout
        for topic in COMMAND_TOPICS:
            types = []
            while not types and time.monotonic() < deadline:
                types = dict(self.node.get_topic_names_and_types()).get(topic, [])
                if not types:
                    time.sleep(0.2)
            if 'geometry_msgs/msg/TwistStamped' in types:
                message_type, stamped = TwistStamped, True
            elif 'geometry_msgs/msg/Twist' in types:
                message_type, stamped = Twist, False
            else:
                raise RuntimeError(f'{topic}: no Twist or TwistStamped endpoint ({types})')
            self.node.create_subscription(
                message_type,
                topic,
                functools.partial(self._on_command, topic, stamped),
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
            )

    def get_parameters(self, node_name, names, timeout=10.0):
        """Return ``{name: value}`` of a node's parameters (None for an unset one)."""
        client = self.node.create_client(GetParameters, f'/{node_name}/get_parameters')
        try:
            response = self._call(client, GetParameters.Request(names=list(names)), timeout)
        finally:
            self.node.destroy_client(client)
        if response is None:
            raise RuntimeError(f'/{node_name}/get_parameters did not answer')
        if len(response.values) != len(names):
            raise RuntimeError(f'/{node_name}: not every one of {names} is declared')
        return {
            name: parameter_value_to_python(value) for name, value in zip(names, response.values)
        }

    def plan_for(self, goal, start, end):
        """Return the first map-frame /plan received in [start, end] that ends at ``goal``."""
        with self._lock:
            messages = list(self.plan_messages)
        for t, frame, points in messages:
            if start <= t <= end and frame == 'map' and len(points):
                if math.hypot(points[-1, 0] - goal[0], points[-1, 1] - goal[1]) <= (
                    PLAN_END_TOLERANCE
                ):
                    return points
        return None

    def commands_between(self, topic, start, end):
        """Return the (sim t, wall t, v, w) samples of ``topic`` received in [start, end]."""
        with self._lock:
            return [s for s in self.commands[topic] if start <= s[0] <= end]


# Metrics -------------------------------------------------------------------------------------


def polyline_distance(points, polyline):
    """Return the distance of each point (N x 2) to a polyline (M x 2)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(polyline) == 1:
        return np.hypot(*(points - polyline[0]).T)
    a, b = polyline[:-1], polyline[1:]
    ab = b - a
    length2 = np.maximum((ab**2).sum(axis=1), 1e-12)
    ap = points[:, None, :] - a[None, :, :]
    s = np.clip((ap * ab[None]).sum(axis=2) / length2[None], 0.0, 1.0)
    closest = a[None] + s[..., None] * ab[None]
    return np.min(np.linalg.norm(points[:, None, :] - closest, axis=2), axis=1)


def command_updates(samples):
    """Return the last (wall t, v, w) of each burst of commands (see :data:`BURST_GAP`)."""
    updates = []
    for _, wall, v, w in samples:
        if updates and wall - updates[-1][0] < BURST_GAP:
            updates[-1] = (wall, v, w)
        else:
            updates.append((wall, v, w))
    return np.array(updates, dtype=float).reshape(-1, 3)


def second_differences(samples):
    """Return the jerks (K x 2: linear, angular) of command samples, and the update period.

    The commands are reduced to one per control update (:func:`command_updates`) and differenced
    twice with the median interval between updates as the step; the series is split where an
    interval exceeds 1.5 times that median (a late tick or a dropped command).
    """
    updates = command_updates(samples)
    if len(updates) < 3:
        return np.zeros((0, 2)), math.nan
    intervals = np.diff(updates[:, 0])
    period = float(np.median(intervals))
    breaks = np.flatnonzero(intervals > 1.5 * period) + 1
    jerks = [np.zeros((0, 2))]
    for piece in np.split(updates[:, 1:], breaks):
        if len(piece) >= 3:
            jerks.append((piece[2:] - 2.0 * piece[1:-1] + piece[:-2]) / period**2)
    return np.concatenate(jerks), period


def rms(values):
    """Return the root mean square (NaN for no values)."""
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values**2))) if values.size else math.nan


def goal_metrics(nav, episode, goal):
    """Return the raw arrays and the scalar metrics of one goal."""
    poses = np.array([(p.x, p.y) for p in episode.poses]).reshape(-1, 2)
    plan = nav.plan_for(goal, episode.start, episode.end)
    xte = polyline_distance(poses, plan) if plan is not None and len(poses) else np.zeros(0)
    commands = {t: nav.commands_between(t, episode.start, episode.end) for t in COMMAND_TOPICS}
    differences = {topic: second_differences(commands[topic]) for topic in COMMAND_TOPICS}
    note = '' if plan is not None else 'no /plan ending at the goal'
    return {
        'xte': xte,
        'jerk': {topic: jerk for topic, (jerk, _) in differences.items()},
        'time': episode.end - episode.start,
        'pose_samples': len(poses),
        'periods': [period for _, period in differences.values()],
        'note': note,
    }


def scalar_row(raw):
    """Return the CSV metric columns of raw goal (or pooled route) arrays."""
    nav_jerk, out_jerk = raw['jerk'][COMMAND_TOPICS[0]], raw['jerk'][COMMAND_TOPICS[1]]
    return {
        'time': raw['time'],
        'xte_rms': rms(raw['xte']),
        'xte_max': float(np.max(raw['xte'])) if raw['xte'].size else math.nan,
        'jerk_v_nav': rms(nav_jerk[:, 0]),
        'jerk_w_nav': rms(nav_jerk[:, 1]),
        'jerk_v_out': rms(out_jerk[:, 0]),
        'jerk_w_out': rms(out_jerk[:, 1]),
        'pose_samples': raw['pose_samples'],
        'period_nav': raw['periods'][0],
        'period_out': raw['periods'][1],
    }


def pool(raws):
    """Pool the raw arrays of the route's goals."""
    return {
        'xte': np.concatenate([r['xte'] for r in raws]),
        'jerk': {
            topic: np.concatenate([r['jerk'][topic] for r in raws]) for topic in COMMAND_TOPICS
        },
        'time': sum(r['time'] for r in raws),
        'pose_samples': sum(r['pose_samples'] for r in raws),
        'periods': [float(np.nanmedian([r['periods'][i] for r in raws])) for i in range(2)],
    }


# Running -------------------------------------------------------------------------------------


def follow_path_block(controller):
    """Return the FollowPath block of ``controller``: its base block with :data:`OVERRIDES`."""
    plugin = CONTROLLERS[controller][1]
    if controller == 'vf':
        base = params.load_yaml(demo.controller_overlay(MOTION_MODEL))
    elif controller == 'mppi':
        base = params.load_yaml(params.stock_params_file())
    else:
        base = None
    if base is None:
        block = {'plugin': plugin}
    else:
        block = copy.deepcopy(base['controller_server']['ros__parameters']['FollowPath'])
        if block.get('plugin') != plugin:
            raise RuntimeError(f'{controller}: base FollowPath is {block.get("plugin")}')
    block.update(OVERRIDES[controller])
    return block


def write_overlay(controller, directory):
    """Write the scenario overlay that selects ``controller`` and return its path."""
    overlay = {
        'controller_server': {'ros__parameters': {'FollowPath': follow_path_block(controller)}}
    }
    path = Path(directory) / f'compare_{controller}.yaml'
    path.write_text(yaml.safe_dump(overlay, default_flow_style=False, sort_keys=False))
    return str(path)


def merged_parameters(overlay):
    """Return the parameters loopback_demo merges for ``overlay``."""
    return params.merge(demo.overlay_paths(MOTION_MODEL, ['loopback', overlay]))


def start_launch(overlay, log_path):
    """Start loopback_demo headless in its own process group, logging to ``log_path``."""
    command = [
        'ros2',
        'launch',
        'vectorfield_bringup',
        'loopback_demo.launch.py',
        f'motion_model:={MOTION_MODEL}',
        f'map:={MAP}',
        f'scenarios:={overlay}',
        'use_rviz:=false',
    ]
    log = open(log_path, 'w', encoding='utf-8')  # closed by stop_launch
    process = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )
    process.log = log
    return process


def stop_launch(process):
    """Stop the launch and everything in its process group."""
    for sig, timeout in ((signal.SIGINT, SIGINT_TIMEOUT), (signal.SIGTERM, SIGTERM_TIMEOUT)):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
        except ProcessLookupError:
            break
        except subprocess.TimeoutExpired:
            continue
    # Whatever is left of the group (a node that outlived ros2 launch).
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    if process.poll() is None:
        process.wait(timeout=10.0)
    process.log.close()


#: A failed goal whose last controller command came this many seconds before its end had a
#: controller server that stopped responding.
SILENCE = 10

#: Warnings every Nav2 run logs (the loop-rate warnings of the 10 Hz loopback clock and the
#: first-goal notices of controller_server), left out of a failure's log messages.
ROUTINE_WARNINGS = ('missed its desired rate', 'This warning will appear once')

#: The parts of a :func:`failure_note`.
_FAILURE_NOTE = re.compile(
    r'goal (?P<goal>\d+) \(.*?\): (?P<status>.+?) after .*?closest approach to the goal '
    r'(?P<closest>[\d.]+) m; (?P<reversals>\d+) reversals of an in-place rotation command; '
    r'last controller command (?P<silence>\d+) s before the end; '
    r'log: (?P<message>.*?)(?: \(\d+×\))?(?:; |$)'
)

_LOG_LINE = re.compile(r'\[(?P<node>[\w./-]+)\]: (?P<message>.*)$')


def goal_messages(log_path, goal, count=2):
    """Return the first ``count`` distinct WARN or ERROR messages logged since ``goal`` was sent.

    The window starts at bt_navigator's last "Begin navigating ... to (x, y)" line for the goal;
    :data:`ROUTINE_WARNINGS` are left out. Each message carries its count in the window.
    """
    with open(log_path, encoding='utf-8', errors='replace') as stream:
        lines = stream.readlines()
    marker = f'to ({goal[0]:.2f}, {goal[1]:.2f})'
    starts = [i for i, line in enumerate(lines) if 'Begin navigating' in line and marker in line]
    counts = {}
    for line in lines[starts[-1] if starts else 0 :]:
        if ('[WARN]' in line or '[ERROR]' in line) and not any(
            routine in line for routine in ROUTINE_WARNINGS
        ):
            match = _LOG_LINE.search(line.strip())
            text = f'{match["node"]}: {match["message"]}' if match else line.strip()
            counts[text] = counts.get(text, 0) + 1
    return '; '.join(f'{text} ({n}×)' for text, n in list(counts.items())[:count])


def in_place_reversals(samples, still=0.01):
    """Return how often the commanded yaw rate changes sign while the linear speed is ~0."""
    signs = [math.copysign(1.0, w) for _, _, v, w in samples if abs(v) < still and abs(w) > still]
    return sum(a != b for a, b in zip(signs, signs[1:]))


def failure_note(nav, episode, goal, index):
    """Describe a goal that did not succeed: status, closest approach, dithering, silence.

    :func:`goal_messages` adds the log messages once the launch has stopped and flushed its
    output.
    """
    status = STATUS_NAMES.get(episode.status, str(episode.status))
    closest = min((math.hypot(p.x - goal[0], p.y - goal[1]) for p in episode.poses), default=0.0)
    commands = nav.commands_between(COMMAND_TOPICS[0], episode.start, episode.end)
    last = commands[-1][0] if commands else episode.start
    return (
        f'goal {index} ({goal[0]:g}, {goal[1]:g}): {status} after '
        f'{episode.end - episode.start:.0f} s; closest approach to the goal {closest:.3f} m; '
        f'{in_place_reversals(commands)} reversals of an in-place rotation command; '
        f'last controller command {episode.end - last:.0f} s before the end'
    )


def check_overrides(nav, controller):
    """Raise if the running FollowPath plugin or an override differs from what was asked."""
    expected = {'FollowPath.plugin': CONTROLLERS[controller][1]}
    expected.update({f'FollowPath.{k}': v for k, v in OVERRIDES[controller].items()})
    actual = nav.get_parameters('controller_server', list(expected))
    wrong = [
        f'{name}={actual[name]!r} (expected {value!r})'
        for name, value in expected.items()
        if not (
            actual[name] == value
            or (
                isinstance(value, float)
                and isinstance(actual[name], float)
                and math.isclose(actual[name], value)
            )
        )
    ]
    if wrong:
        raise RuntimeError('controller_server parameters: ' + ', '.join(wrong))


def run_once(controller, run, overlay, log_path, args):
    """Drive the route once; return the CSV rows (one per goal reached or failed, plus route)."""
    print(f'[{controller} run {run}] starting loopback_demo (log {log_path})', flush=True)
    process = start_launch(overlay, log_path)
    nav = None
    rows, raws = [], []
    failed = None
    base = {'controller': controller, 'run': run}
    try:
        nav = RecordingHarness()
        nav.wait_until_active(timeout=args.startup_timeout)
        check_overrides(nav, controller)
        nav.record_commands()
        nav.sleep(1.0)
        for index, goal in enumerate(checks.ROUTE, start=1):
            episode = nav.navigate(*goal, timeout=args.goal_timeout)
            status = STATUS_NAMES.get(episode.status, str(episode.status))
            row = dict(base, scope=f'goal{index}', status=status)
            if episode.status != GoalStatus.STATUS_SUCCEEDED:
                row.update(
                    time=episode.end - episode.start, note=failure_note(nav, episode, goal, index)
                )
                rows.append(row)
                failed = (row, goal)
                break
            raw = goal_metrics(nav, episode, goal)
            raws.append(raw)
            row.update(scalar_row(raw), note=raw['note'])
            rows.append(row)
            print(
                f'[{controller} run {run}] goal {index}: {raw["time"]:.1f} s, '
                f'xte rms {row["xte_rms"]:.3f} max {row["xte_max"]:.3f} m',
                flush=True,
            )
    except Exception as error:  # recorded as the run's outcome
        rows.append(
            dict(base, scope='route', status='ERROR', note=f'{type(error).__name__}: {error}')
        )
        print(f'[{controller} run {run}] {rows[-1]["note"]}', flush=True)
        return rows
    finally:
        if nav is not None:
            nav.close()
        stop_launch(process)
    if failed is not None:
        row, goal = failed
        row['note'] += f'; log: {goal_messages(log_path, goal) or "no warning or error logged"}'
        print(f'[{controller} run {run}] {row["note"]}', flush=True)
    completed = len(raws) == len(checks.ROUTE)
    route = dict(base, scope='route', status='SUCCEEDED' if completed else 'INCOMPLETE')
    if completed:
        route.update(scalar_row(pool(raws)), note='')
    else:
        route.update(note=rows[-1].get('note', ''))
    rows.append(route)
    return rows


def run_all(args):
    """Run every controller ``args.runs`` times; return the rows and the run context."""
    if params.ros_distro() == 'humble':
        sys.exit('compare_controllers.py needs the Nav2 loopback simulator (Jazzy and later)')
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    log_dir = Path(args.log_dir or tempfile.mkdtemp(prefix='compare_controllers_'))
    log_dir.mkdir(parents=True, exist_ok=True)
    overlays = {c: write_overlay(c, log_dir) for c in args.controllers}
    rows = []
    started = time.monotonic()
    for run in range(1, args.runs + 1):
        # Interleaved, so that a slow drift of the host affects every controller alike.
        for controller in args.controllers:
            for attempt in range(1, args.retries + 2):
                log_path = log_dir / f'{controller}_{run}_{attempt}.log'
                attempt_rows = run_once(controller, run, overlays[controller], log_path, args)
                repeat = attempt <= args.retries and infrastructure_failure(attempt_rows[-1])
                for row in attempt_rows:
                    row.update(attempt=attempt, repeated='yes' if repeat else '')
                rows.extend(attempt_rows)
                write_csv(rows, args.out_csv)  # kept up to date, should the comparison be stopped
                time.sleep(args.pause)
                if not repeat:
                    break
                print(f'[{controller} run {run}] Nav2 failure; repeating the run', flush=True)
    return rows, {
        'wall_minutes': (time.monotonic() - started) / 60.0,
        'retries': args.retries,
    }


def infrastructure_failure(route):
    """Return whether a run's route row records a Nav2 failure outside the controller plugin.

    That is a harness or startup error, bt_navigator missing a server's goal acknowledgement
    (``default_server_timeout``), or a controller server that stopped publishing commands for
    :data:`SILENCE` seconds or more while the goal was active.
    """
    if route['status'] == 'ERROR':
        return True
    match = _FAILURE_NOTE.search(route.get('note') or '')
    return bool(match) and (
        int(match['silence']) >= SILENCE or 'acknowledge goal request' in match['message']
    )


# Reporting -----------------------------------------------------------------------------------


def write_csv(rows, path):
    """Write the rows as CSV."""
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({k: _csv_value(row.get(k, '')) for k in CSV_FIELDS})


def _csv_value(value):
    return f'{value:.6g}' if isinstance(value, float) else value


def context_path(csv_path):
    """Return the path of the JSON file that records a CSV's run context."""
    return Path(csv_path).with_suffix('.json')


def read_csv(path):
    """Read rows written by :func:`write_csv`."""
    rows = []
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            for key in METRICS:
                row[key] = float(row[key]) if row.get(key) not in (None, '') else math.nan
            row['run'] = int(row['run'])
            row['attempt'] = int(row.get('attempt') or 1)
            row['repeated'] = row.get('repeated') or ''
            rows.append(row)
    return rows


def summarize(rows, controllers):
    """Return, per controller, the completed route count and each metric's values."""
    summary = {}
    for controller in controllers:
        routes = [
            r
            for r in rows
            if r['controller'] == controller and r['scope'] == 'route' and not r.get('repeated')
        ]
        done = [r for r in routes if r['status'] == 'SUCCEEDED']
        summary[controller] = {
            'runs': len(routes),
            'completed': len(done),
            'values': {
                k: [float(r[k]) for r in done if not math.isnan(float(r[k]))] for k in METRICS
            },
            'failures': [r for r in routes if r['status'] != 'SUCCEEDED'],
        }
    return summary


def mean_std(values, spec):
    """Return 'mean ± std' (sample std) of ``values`` with the format ``spec``."""
    if not values:
        return '–'
    mean = statistics.fmean(values)
    if len(values) < 2:
        return f'{mean:{spec}}'
    return f'{mean:{spec}} ± {statistics.stdev(values):{spec}}'


def table(summary, keys):
    """Return a Markdown table of the metrics ``keys``, mean ± std per controller."""
    header = ['Controller', 'Routes completed'] + [
        f'{METRICS[k][0]} ({METRICS[k][1]})' for k in keys
    ]
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    for controller, entry in summary.items():
        cells = [CONTROLLERS[controller][0], f'{entry["completed"]}/{entry["runs"]}']
        cells += [mean_std(entry['values'][k], METRICS[k][2]) for k in keys]
        lines.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


def ranking(summary, key):
    """Return the controllers that completed routes, by increasing mean of one metric."""
    means = [
        (statistics.fmean(e['values'][key]), CONTROLLERS[c][0])
        for c, e in summary.items()
        if e['values'][key]
    ]
    if len(means) < 2:
        return None
    means.sort()
    spec = METRICS[key][2]
    unit = METRICS[key][1]
    return ', '.join(f'{name} {value:{spec}} {unit}' for value, name in means)


def overrides_text(controller):
    """Return the Markdown list item of one controller's FollowPath source and overrides."""
    values = ', '.join(f'`{k}: {v}`' for k, v in OVERRIDES[controller].items())
    return f'- **{CONTROLLERS[controller][0]}**: {BASE_SOURCES[controller]}, with {values}.'


def package_version(package):
    """Return the version in a package's installed package.xml ('' if unknown)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        text = Path(get_package_share_directory(package), 'package.xml').read_text()
    except Exception:
        return ''
    match = re.search(r'<version>([^<]+)</version>', text)
    return match.group(1) if match else ''


def render(rows, controllers, context, args):
    """Return the Markdown page."""
    summary = summarize(rows, controllers)
    runs = max((r['run'] for r in rows), default=0)
    merged = merged_parameters(write_overlay(controllers[0], tempfile.mkdtemp()))
    server = merged['controller_server']['ros__parameters']
    checker = server['general_goal_checker']
    progress = server['progress_checker']
    smoother = merged['velocity_smoother']['ros__parameters']
    periods = {
        key: [
            float(r[key]) for r in rows if r['scope'] == 'route' and r.get(key) not in ('', None)
        ]
        for key in ('period_nav', 'period_out')
    }
    periods = {k: [v * 1e3 for v in values if not math.isnan(v)] for k, values in periods.items()}
    route = ', '.join(f'({x:g}, {y:g}, {math.degrees(yaw):g}°)' for x, y, yaw in checks.ROUTE)
    distro = params.ros_distro() or 'unknown'
    nav2 = package_version('nav2_controller')
    wall = context.get('wall_minutes')
    limits = '\n'.join(f'- {name}: {value}' for name, value in SHARED_LIMITS.items())
    lines = [
        '# Controller comparison',
        '',
        ':::{note}',
        'Generated by `vectorfield_system_tests/benchmarks/compare_controllers.py` from a '
        f'recorded run ({context.get("date", "")}). The numbers are informative and do not '
        'gate CI.',
        ':::',
        '',
        f"The M3 route on Nav2's loopback simulator, driven {runs} times by each controller with "
        'identical limits, goal checker, velocity smoother and collision monitor.',
        '',
        '## Setup',
        '',
        f'- ROS 2 {distro.capitalize()}, Nav2 {nav2 or "(version unknown)"}; '
        f'`loopback_demo.launch.py`, map `{MAP}`, motion model `{MOTION_MODEL}`, headless.',
        f'- Hardware: {context.get("hardware", args.hardware)}.',
        f'- Route: from (0, 0, 0°), NavigateToPose to {route} in turn (map x, y, yaw), each goal '
        f'within {args.goal_timeout:g} s of simulated time. A route is completed when all three '
        'goals succeed; a run stops at its first failed goal.',
        f'- One launch per controller and run; runs interleaved (each run cycles through '
        f'{", ".join(CONTROLLERS[c][0] for c in controllers)}).',
        f'- Controller: `controller_frequency` {server["controller_frequency"]:g} Hz; goal '
        f'checker `{checker["plugin"]}` (`xy_goal_tolerance` {checker["xy_goal_tolerance"]:g} m, '
        f'`yaw_goal_tolerance` {checker["yaw_goal_tolerance"]:g} rad, `stateful` '
        f'{checker.get("stateful")}); progress checker `{progress["plugin"]}` '
        f'(`required_movement_radius` {progress["required_movement_radius"]:g} m, '
        f'`movement_time_allowance` {progress["movement_time_allowance"]:g} s).',
        f'- Velocity smoother (`cmd_vel_nav` to `cmd_vel_smoothed`): `max_velocity` '
        f'{smoother["max_velocity"]}, `max_accel` {smoother["max_accel"]}, `max_decel` '
        f'{smoother["max_decel"]}, `scale_velocities` {smoother.get("scale_velocities")}; then '
        'the stock collision monitor (`cmd_vel_smoothed` to `cmd_vel`).',
        '',
        '### Shared limits',
        '',
        limits,
        '',
        '### Controller parameters',
        '',
        'Each run replaces `controller_server.FollowPath` with the block below; everything not '
        'listed keeps its stock value. RPP and Graceful have no linear acceleration parameter, '
        'and Graceful no angular acceleration parameter: for them those limits come only from '
        'the shared velocity smoother.',
        '',
    ]
    lines += [overrides_text(c) for c in controllers]
    lines += [
        '',
        '### Metrics',
        '',
        '- Cross-track error: distance from the true position (map → base_footprint; the '
        'loopback simulator localizes perfectly) to the first global plan published on `/plan` '
        'for the goal, from goal send to the result. RMS over the pose samples (uniform in '
        'time, so in-place rotations count as zero-error samples) and maximum, pooled over the '
        'route.',
        '- Time to goal: simulated time from sending the goal to SUCCEEDED, summed over the three '
        'goals (the loopback simulator publishes `/clock` at 10 Hz, so times are multiples of '
        '0.1 s).',
        '- RMS jerk: second finite differences of the commanded linear (x) and angular (z) '
        'velocity, pooled over the route, for the controller output (`cmd_vel_nav`) and for the '
        'final command after the shared smoother and collision monitor (`cmd_vel`). Commands '
        f'received less than {BURST_GAP * 1e3:g} ms apart count as one update (the last one, '
        'which the robot keeps; the controller server occasionally publishes two back to back '
        'when its loop catches up), and the finite-difference step is the median interval '
        'between updates.' + period_text(periods),
        '',
        '## Results',
        '',
        f'Mean ± sample standard deviation over the completed routes ({runs} runs per '
        'controller).',
        '',
        '### Path tracking and controller output',
        '',
        table(summary, ('time', 'xte_rms', 'xte_max', 'jerk_v_nav', 'jerk_w_nav')),
        '',
        '### Final command (`cmd_vel`)',
        '',
        table(summary, ('jerk_v_out', 'jerk_w_out')),
        '',
    ]
    repeated = [r for r in rows if r['scope'] == 'route' and r.get('repeated')]
    if repeated:
        lines += [
            '### Repeated runs',
            '',
            f'A run was repeated, up to {context.get("retries", args.retries)} times, when it '
            'failed for a Nav2 reason outside the controller plugin: a harness or startup error, '
            "bt_navigator not receiving a server's goal acknowledgement within "
            '`default_server_timeout`, or a controller server that stopped publishing commands '
            f'for {SILENCE} s or more while the goal was active. '
            'These attempts are excluded from the statistics:',
            '',
        ]
        lines += [
            f'- {CONTROLLERS[r["controller"]][0]}, run {r["run"]}, attempt {r["attempt"]}: '
            f'{r["status"]}; {r.get("note") or "no detail"}'
            for r in repeated
        ]
        lines.append('')
    failures = [f for entry in summary.values() for f in entry['failures']]
    if failures:
        lines += ['### Runs that did not complete the route', '']
        lines += [
            f'- {CONTROLLERS[f["controller"]][0]}, run {f["run"]}: {f["status"]}; '
            f'{f.get("note") or "no detail"}'
            for f in failures
        ]
        lines.append('')
    lines += ['## Interpretation', '']
    lines += interpretation(summary, checker['xy_goal_tolerance'], server['controller_frequency'])
    if context.get('notes'):
        lines += ['', context['notes'].strip()]
    lines += [
        '',
        '## Reproducing',
        '',
        'In the Jazzy development container (or any Jazzy or later workspace):',
        '',
        '```bash',
        'rosdep install --from-paths src --ignore-src -y',
        'colcon build --packages-up-to vectorfield_system_tests',
        'source install/setup.bash',
        f'ros2 run vectorfield_system_tests compare_controllers.py --runs {runs} \\',
        '  --out-md controller_comparison.md --out-csv controller_comparison.csv',
        '```',
        '',
        '`--controllers vf,rpp` restricts the set; `--from-csv` regenerates the page from a '
        "recorded CSV; `--log-dir` keeps each launch's output. A run that fails for a Nav2 reason "
        'outside the controller plugin is repeated up to `--retries` times (default 2) and listed '
        'separately. One run takes about one minute.',
    ]
    if wall:
        lines[-1] += f' The recorded run took {wall:.0f} minutes of wall time.'
    return '\n'.join(lines) + '\n'


def period_text(periods):
    """Return the sentence on the measured command intervals (ms), or ''."""
    parts = [
        f'`{topic}` {span(values, ".0f")} ms'
        for topic, values in zip(('cmd_vel_nav', 'cmd_vel'), periods.values())
        if values
    ]
    if not parts:
        return ''
    return ' Measured median interval between updates, per route: ' + ', '.join(parts) + '.'


def span(values, spec):
    """Return 'min to max' of ``values`` with the format ``spec`` (one value if they are equal)."""
    low, high = f'{min(values):{spec}}', f'{max(values):{spec}}'
    return low if low == high else f'{low} to {high}'


def failure_summary(controller, entry, tolerance):
    """Return a list item that summarizes a controller's failed runs from their notes."""
    text = (
        f'- {CONTROLLERS[controller][0]} completed {entry["completed"]} of {entry["runs"]} routes.'
    )
    parsed = [_FAILURE_NOTE.search(f.get('note') or '') for f in entry['failures']]
    parsed = [m for m in parsed if m]
    if not parsed:
        return text + ' See the runs that did not complete the route above.'
    goals = ', '.join(sorted({m['goal'] for m in parsed}))
    statuses = ', '.join(sorted({m['status'] for m in parsed}))
    closest = [float(m['closest']) for m in parsed]
    reversals = [int(m['reversals']) for m in parsed]
    message = collections.Counter(m['message'] for m in parsed).most_common(1)[0][0]
    runs = f'{len(parsed)} failed run' + ('s' if len(parsed) > 1 else '')
    text += (
        f' Its {runs} ended at goal {goals} with status {statuses}, a closest approach to the '
        f'goal of {span(closest, ".3f")} m (`xy_goal_tolerance` {tolerance:g} m), '
        f'{span(reversals, "d")} reversals of an in-place rotation command; the first warning '
        f'or error logged after the goal was sent was most often `{message}`.'
    )
    silent = [int(m['silence']) for m in parsed if int(m['silence']) >= SILENCE]
    if silent:
        text += (
            f' In {len(silent)} of them the controller server published no command during the '
            f'last {span(silent, "d")} s of the goal, and the goal neither failed nor succeeded '
            'before the goal timeout.'
        )
    if any('acknowledge goal request' in m['message'] for m in parsed):
        text += (
            " In a run that logged a goal acknowledgement time-out, bt_navigator's action client "
            "did not receive the controller server's acceptance within `default_server_timeout` "
            '(20 ms in the stock parameters) and NavigateToPose aborted: a Nav2 timing failure '
            'on a loaded host, not a control decision.'
        )
    if min(closest) > tolerance and min(reversals) > 0:
        text += (
            ' The robot stopped translating just outside the goal tolerance and only rotated, '
            'so the goal checker never accepted the pose; the progress checker aborted '
            "FollowPath and the behavior tree's recoveries retried until the goal timeout."
        )
    return text


def interpretation(summary, tolerance, server_frequency):
    """Return the interpretation paragraphs: data-driven orderings plus fixed caveats."""
    lines = []
    orders = [
        ('Cross-track RMS', 'xte_rms'),
        ('Cross-track max', 'xte_max'),
        ('Time to goal', 'time'),
        ('Linear jerk of the controller output', 'jerk_v_nav'),
        ('Angular jerk of the controller output', 'jerk_w_nav'),
    ]
    for label, key in orders:
        text = ranking(summary, key)
        if text:
            lines.append(f'- {label}, lowest to highest mean: {text}.')
    lines += [
        failure_summary(c, e, tolerance) for c, e in summary.items() if e['completed'] < e['runs']
    ]
    lines += [
        '',
        'Caveats. The loopback simulator applies the final command exactly, without dynamics, '
        'noise or localization error, on an empty map, so the numbers compare the control laws '
        'under ideal conditions and say little about a real robot. Each route segment is a '
        'straight plan; most of the cross-track error comes from how a controller turns onto a '
        "segment whose direction is 90° from the robot's heading (goals 2 and 3) and how it "
        'approaches the goal. The vector-field controller steers a control point 0.2 m ahead of '
        '`base_footprint` (differential model), while the error is measured at `base_footprint` '
        'for every controller. Jerk of the controller output reflects how each controller limits '
        'its own acceleration; RPP and Graceful rely on the shared smoother for the linear '
        'acceleration, which the `cmd_vel` table includes. MPPI samples random trajectories, so '
        'its runs are not deterministic.',
        '',
        "The loopback simulator's clock also shapes the controller outputs. It publishes "
        '`/clock` at 10 Hz while the controller server calls the controllers about every '
        f'{1e3 / server_frequency:g} ms, so every other call sees no elapsed simulated time. '
        'The vector-field controller then returns its previous command, so its acceleration '
        'limits hold in simulated time and its output changes once per 0.1 s tick. Finite '
        'differences of such a staircase raise the jerk of `cmd_vel_nav` compared with an '
        'output that also changes between ticks; the `cmd_vel` table, after the shared '
        'smoother, compares the commands the robot receives.',
    ]
    return lines


# Main ----------------------------------------------------------------------------------------


def parse_args(argv=None):
    """Parse the command line."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--runs', type=int, default=5, help='Runs per controller (default 5).')
    parser.add_argument(
        '--controllers',
        default=','.join(CONTROLLERS),
        help=f'Comma-separated subset of {", ".join(CONTROLLERS)} (default: all).',
    )
    parser.add_argument('--out-md', default='controller_comparison.md', help='Markdown page.')
    parser.add_argument('--out-csv', default='controller_comparison.csv', help='Per-goal CSV.')
    parser.add_argument(
        '--from-csv', help='Regenerate the page from this recorded CSV; run nothing.'
    )
    parser.add_argument('--log-dir', help='Directory for the launch logs and overlays.')
    parser.add_argument(
        '--goal-timeout',
        type=float,
        default=checks.GOAL_TIMEOUT,
        help='Simulated seconds per goal (default %(default)g).',
    )
    parser.add_argument(
        '--startup-timeout',
        type=float,
        default=180.0,
        help='Wall seconds for Nav2 to become active (default %(default)g).',
    )
    parser.add_argument(
        '--retries',
        type=int,
        default=2,
        help='Repeats of a run that fails for a Nav2 reason outside the controller plugin '
        '(default %(default)d).',
    )
    parser.add_argument(
        '--pause', type=float, default=3.0, help='Wall seconds between runs (default %(default)g).'
    )
    parser.add_argument(
        '--notes',
        help='Markdown file of observations from the recorded logs, added to the interpretation '
        '(kept in the JSON next to the CSV).',
    )
    parser.add_argument(
        '--hardware',
        default=f'{platform.system()} {platform.machine()}, {os.cpu_count()} CPUs',
        help='Hardware description for the page.',
    )
    args = parser.parse_args(argv)
    args.controllers = [c.strip() for c in args.controllers.split(',') if c.strip()]
    unknown = [c for c in args.controllers if c not in CONTROLLERS]
    if unknown or not args.controllers:
        parser.error(f'unknown controllers {unknown}; choose from {", ".join(CONTROLLERS)}')
    if args.runs < 1:
        parser.error('--runs must be at least 1')
    return args


def main(argv=None):
    """Run the comparison (or re-render a recorded one) and write the page."""
    args = parse_args(argv)
    if args.from_csv:
        rows = read_csv(args.from_csv)
        meta = context_path(args.from_csv)
        context = json.loads(meta.read_text()) if meta.is_file() else {}
        present = {r['controller'] for r in rows}
        args.controllers = [c for c in args.controllers if c in present]
        if args.notes:
            context['notes'] = Path(args.notes).read_text(encoding='utf-8')
            meta.write_text(json.dumps(context, indent=2) + '\n')
    else:
        rows, context = run_all(args)
        context.update(date=datetime.date.today().isoformat(), hardware=args.hardware)
        if args.notes:
            context['notes'] = Path(args.notes).read_text(encoding='utf-8')
        write_csv(rows, args.out_csv)
        context_path(args.out_csv).write_text(json.dumps(context, indent=2) + '\n')
        print(f'wrote {args.out_csv} and {context_path(args.out_csv)}', flush=True)
    Path(args.out_md).write_text(render(rows, args.controllers, context, args), encoding='utf-8')
    print(f'wrote {args.out_md}', flush=True)


if __name__ == '__main__':
    main()
