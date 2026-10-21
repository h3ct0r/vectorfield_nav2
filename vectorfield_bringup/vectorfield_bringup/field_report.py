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

"""The field-test report: episodes, metrics, Markdown and plots from a recorded session.

:func:`load_recording` reads a rosbag2 bag (``record_field_test.py``) into a :class:`Recording`
of plain arrays; :func:`analyse` splits it into episodes (one per NavigateToPose,
NavigateThroughPoses or FollowCurve goal, from the action status topics) and computes the metrics
of ``docs/tutorials/field_testing.md``; :func:`render_markdown` and :func:`plot_episodes` write the
report. Only :func:`load_recording` needs ROS and only :func:`plot_episodes` matplotlib, so the
analysis can be tested on synthetic recordings.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from vectorfield_bringup import field_metrics as fm

#: Action names (the last component of ``<name>/_action/status``) by episode kind.
NAV_ACTIONS = ('navigate_to_pose', 'navigate_through_poses')
FOLLOW_ACTIONS = ('follow_curve',)
CONTROLLER_ACTIONS = ('follow_path',)
RECOVERY_ACTIONS = ('spin', 'backup', 'wait', 'drive_on_heading', 'assisted_teleop')

#: GoalStatus values.
STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}
TERMINAL = (4, 5, 6)

#: Command topics, controller output first (the last component of the topic name).
COMMAND_CHAIN = ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel', 'cmd_vel_safe')

#: Seconds after a speed-limit change before the commands must respect it (deceleration).
SPEED_LIMIT_GRACE = 1.0

#: A /plan belongs to a goal when it ends within this distance of the goal's last plan (m).
PLAN_END_TOLERANCE = 0.25


@dataclass
class Options:
    """Thresholds of the checks, and topic choices (empty: automatic)."""

    converged: float = 0.05
    tracking_limit: float = 0.10
    lap_limit: float = 0.10
    xy_tolerance: float = 0.25
    yaw_tolerance: float = 0.25
    stop_slack: float = 0.05
    settle: float = 0.5
    max_stop_latency: float = 1.0
    max_rtf_per_corner: int = 1
    max_linear_vel: float = math.nan
    jump_window: float = 3.0
    controller_topic: str = ''
    follower_topic: str = ''


@dataclass
class Recording:
    """A recorded session as plain arrays, in the stamp clock (see ``bag.TimeMap``).

    Track dicts hold numpy arrays of equal length under the keys named in the comments.
    """

    start: float = 0.0
    end: float = 0.0
    global_frame: str = 'map'
    odom_frame: str = 'odom'
    #: Robot pose in the global frame: t, x, y, yaw.
    pose: dict = field(default_factory=dict)
    #: Robot pose in the odometry frame (the field_state frame): t, x, y, yaw.
    pose_odom: dict = field(default_factory=dict)
    #: Measured twist (odometry): t, v, w.
    odom_twist: dict = field(default_factory=dict)
    #: field_state topic -> t, phase, status, distance, obstacle_distance, contour, cp_x, cp_y,
    #: cl_x, cl_y, limiter.
    field_states: dict = field(default_factory=dict)
    #: Command topic -> t, recv (receive time), v, vy, w.
    commands: dict = field(default_factory=dict)
    #: (t, frame, N x 2 points, final yaw) per /plan message.
    plans: list = field(default_factory=list)
    #: Action name -> [(t, [(goal id, status, accept stamp)])] per status message.
    statuses: dict = field(default_factory=dict)
    #: Action name -> t, lap (FollowCurve feedback).
    laps: dict = field(default_factory=dict)
    #: (t, percentage, limit) per speed_limit message.
    speed_limits: list = field(default_factory=list)
    #: Minimum valid scan range: t, r.
    scan_min: dict = field(default_factory=dict)
    #: Distance from the robot to the nearest lethal costmap cell: t, d.
    clearance: dict = field(default_factory=dict)
    #: global -> odom transform as published: t, x, y, yaw.
    map_odom: dict = field(default_factory=dict)
    #: (t, node name, previous status, current status) per behaviour-tree event.
    bt_events: list = field(default_factory=list)
    #: field_state topic -> its frame.
    field_state_frames: dict = field(default_factory=dict)
    #: field_state topic -> t, bind, hold (CBF, from the status markers).
    cbf: dict = field(default_factory=dict)
    #: topic -> (type, messages, rate Hz).
    topics: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)


@dataclass
class Episode:
    """One goal of a navigation or curve-following action."""

    kind: str
    action: str
    goal_id: str
    start: float
    end: float
    status: str


# Episodes ------------------------------------------------------------------------------------


def goals_from_status(messages):
    """Return [(goal id, start, end or None, last status)] from GoalStatusArray contents.

    ``messages`` are (t, [(goal id, status, accept stamp)]). The start is the acceptance stamp
    (else the first message with the goal), the end the first message with a terminal status.
    """
    goals = {}
    order = []
    for t, entries in messages:
        for goal_id, status, stamp in entries:
            goal = goals.get(goal_id)
            if goal is None:
                goal = {'start': stamp if stamp > 0.0 else t, 'end': None, 'status': status}
                goals[goal_id] = goal
                order.append(goal_id)
            if goal['end'] is None:
                goal['status'] = status
                if status in TERMINAL:
                    goal['end'] = t
    return [(g, goals[g]['start'], goals[g]['end'], goals[g]['status']) for g in order]


def action_kind(action):
    """Return 'nav', 'follow', 'controller', 'recovery' or '' for an action name."""
    name = action.rstrip('/').split('/')[-1]
    if name in NAV_ACTIONS:
        return 'nav'
    if name in FOLLOW_ACTIONS:
        return 'follow'
    if name in CONTROLLER_ACTIONS:
        return 'controller'
    if name in RECOVERY_ACTIONS:
        return 'recovery'
    return ''


def episodes(rec):
    """Return the navigation and curve-following episodes, by start time."""
    result = []
    for action, messages in rec.statuses.items():
        kind = action_kind(action)
        if kind not in ('nav', 'follow'):
            continue
        for goal_id, start, end, status in goals_from_status(messages):
            done = end is not None
            result.append(
                Episode(
                    kind,
                    action,
                    goal_id,
                    start,
                    end if done else rec.end,
                    STATUS_NAMES.get(status, str(status)) if done else 'UNFINISHED',
                )
            )
    return sorted(result, key=lambda e: e.start)


# Helpers -------------------------------------------------------------------------------------


def _window(track, start, end, key='t'):
    """Return a boolean mask of the samples of ``track`` with ``start <= t <= end``."""
    t = np.asarray(track.get(key, ()), dtype=float)
    return (t >= start) & (t <= end)


def _select(track, mask):
    return {k: np.asarray(v)[mask] for k, v in track.items()}


def _finite_min(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.min(values)) if values.size else math.nan


def _pose_at(rec, t):
    """Return (x, y, yaw) of the robot in the global frame at ``t`` (None without poses)."""
    pose = rec.pose
    if not pose or len(pose['t']) == 0:
        return None
    x, y, yaw = fm.interpolate_pose(t, pose['t'], pose['x'], pose['y'], pose['yaw'])
    return float(x), float(y), float(yaw)


def choose_field_state(rec, kind, options):
    """Return the field_state topic for an episode kind ('' if none)."""
    wanted = options.follower_topic if kind == 'follow' else options.controller_topic
    if wanted:
        return wanted if wanted in rec.field_states else ''
    topics = sorted(rec.field_states)
    follower = [t for t in topics if 'curve_follower' in t or 'follow_curve' in t]
    others = [t for t in topics if t not in follower]
    if kind == 'follow':
        return follower[0] if follower else (others[0] if len(others) == 1 else '')
    return others[0] if others else ''


def command_topics(rec):
    """Return the recorded command topics in chain order (controller output first)."""

    def rank(topic):
        name = topic.rstrip('/').split('/')[-1]
        return COMMAND_CHAIN.index(name) if name in COMMAND_CHAIN else len(COMMAND_CHAIN)

    return sorted(rec.commands, key=lambda topic: (rank(topic), topic))


def _plan_in(rec, start, end, until=math.inf):
    """Return (first, last) /plan of a goal (None, None without one).

    ``last`` is the latest plan received in [start, end] and before the next goal (``until``);
    ``first`` the earliest one received from start - 0.5 s that ends within
    :data:`PLAN_END_TOLERANCE` of where ``last`` ends (the replans of a previous goal can arrive
    just after the next goal starts).
    """
    plans = [p for p in rec.plans if start - 0.5 <= p[0] <= end and p[0] < until and len(p[2])]
    if not plans:
        return None, None
    inside = [p for p in plans if p[0] >= start] or plans
    last = inside[-1]
    goal = last[2][-1]
    for plan in plans:
        if math.hypot(*(plan[2][-1] - goal)) <= PLAN_END_TOLERANCE:
            return plan, last
    return plans[0], last


def speed_limit_at(limits, t):
    """Return (percentage, limit) in force at ``t`` (None without one)."""
    current = None
    for ts, percentage, limit in limits:
        if ts > t:
            break
        current = (percentage, limit)
    return current


def speed_limit_value(percentage, limit, max_linear_vel):
    """Return the speed limit in m/s (inf for none, NaN for a percentage without the maximum)."""
    if limit <= 0.0 or (percentage and limit >= 100.0):
        return math.inf
    if percentage:
        return max_linear_vel * limit / 100.0 if math.isfinite(max_linear_vel) else math.nan
    return limit


# Per-episode metrics ---------------------------------------------------------------------------


def episode_metrics(rec, episode, index, options, jumps=(), until=None):
    """Return the metrics and checks of one episode as a dict (see the tutorial's tables).

    ``until`` is when the next episode starts: the stop latency is only measured before it.
    """
    until = rec.end if until is None else until
    # The samples of the next goal can arrive before this goal's result is recorded.
    start, end = episode.start, min(episode.end, until - 1e-3)
    m = {
        'index': index,
        'kind': episode.kind,
        'action': episode.action,
        'status': episode.status,
        'start': start - rec.start,
        'duration': episode.end - start,
    }
    topic = choose_field_state(rec, episode.kind, options)
    m['field_state_topic'] = topic
    fs = (
        _select(rec.field_states[topic], _window(rec.field_states[topic], start, end))
        if topic
        else {}
    )
    have_fs = bool(fs) and len(fs['t']) > 0

    # Tracking from field_state (the controller's own distance to the path or curve).
    if have_fs:
        tracking = np.isin(fs['phase'], fm.TRACKING_PHASES)
        d = np.abs(fs['distance'][tracking])
        m['d_rms'] = fm.rms(d)
        m['d_max'] = float(np.max(d)) if d.size else math.nan
        # The convergence criterion leaves out the recovery from each localisation jump (from the
        # jump until |D| is back within the band, at most jump_window s): reported separately.
        keep = np.ones(len(fs['t']), dtype=bool)
        for t_jump, _, _ in jumps:
            if start <= t_jump <= end:
                _, back = fm.recovery_after(
                    fs['t'], fs['distance'], t_jump, options.jump_window, options.converged
                )
                span = back if math.isfinite(back) else options.jump_window
                keep &= ~((fs['t'] >= t_jump) & (fs['t'] <= t_jump + span))
        m['tracking_max'] = fm.tracking_error(
            fs['t'][keep], fs['phase'][keep], fs['distance'][keep], options.converged
        )
        m['field_state_samples'] = int(len(fs['t']))
    else:
        m['d_rms'] = m['d_max'] = m['tracking_max'] = math.nan
        m['field_state_samples'] = 0

    # Pose track of the episode.
    poses = _select(rec.pose, _window(rec.pose, start, end)) if rec.pose else {}
    have_pose = bool(poses) and len(poses['t']) > 0
    travelled = fm.path_length(poses['x'], poses['y']) if have_pose else np.zeros(0)
    m['distance_travelled'] = float(travelled[-1]) if travelled.size else math.nan

    corners = []
    if episode.kind == 'nav':
        first, last = _plan_in(rec, start, end, until)
        m['plan'] = first is not None
        if first is not None and have_pose:
            xte = fm.polyline_distance(np.column_stack((poses['x'], poses['y'])), first[2])
            m['xte_rms'], m['xte_max'] = fm.rms(xte), float(np.max(xte))
            corners = fm.plan_corners(first[2])
        else:
            m['xte_rms'] = m['xte_max'] = math.nan
        m['corners'] = [{'x': c[0], 'y': c[1], 'turn_deg': math.degrees(c[2])} for c in corners]
        if last is not None:
            goal = (float(last[2][-1, 0]), float(last[2][-1, 1]), float(last[3]))
            m['goal'] = goal
            # 0.5 s after the result, unless the next goal starts earlier.
            settle_t = min(end + options.settle, max(until, end), rec.end)
            final = _pose_at(rec, settle_t)
            if final is not None:
                m['goal_miss'] = math.hypot(final[0] - goal[0], final[1] - goal[1])
                m['goal_yaw_error'] = abs(float(fm.wrap_angle(final[2] - goal[2])))
        m.setdefault('goal_miss', math.nan)
        m.setdefault('goal_yaw_error', math.nan)
    else:
        laps = rec.laps.get(episode.action)
        lap1 = math.nan
        m['laps'] = 0
        if laps is not None and len(laps['t']):
            inside = _window(laps, start, end)
            lap_values = np.asarray(laps['lap'])[inside]
            lap_t = np.asarray(laps['t'])[inside]
            if lap_values.size:
                m['laps'] = int(np.max(lap_values))
                done = np.flatnonzero(lap_values >= 1)
                lap1 = float(lap_t[done[0]]) if done.size else math.nan
        m['lap1_time'] = lap1 - start if math.isfinite(lap1) else math.nan
        if have_fs and math.isfinite(lap1):
            after = fs['t'] > lap1
            tracking = np.isin(fs['phase'], fm.TRACKING_PHASES) & after
            d = np.abs(fs['distance'][tracking])
            m['d_max_after_lap1'] = float(np.max(d)) if d.size else math.nan
            m['d_rms_after_lap1'] = fm.rms(d)
        else:
            m['d_max_after_lap1'] = m['d_rms_after_lap1'] = math.nan

    # Phases, rotate-in-place entries.
    if have_fs:
        segments = fm.phase_segments(fs['t'], fs['phase'])
        m['timeline'] = [
            (fm.PHASE_NAMES.get(p, str(p)), a - start, b - start) for p, a, b in segments
        ]
        m['phases'] = fm.phase_durations(segments)
        entries = []
        for phase, t0, _ in segments:
            if phase != fm.PHASE_ROTATE_TO_FIELD:
                continue
            at = _pose_at(rec, t0)
            if at is None:
                continue
            gone = float(np.interp(t0, poses['t'], travelled)) if have_pose else 0.0
            entries.append((at[0], at[1], gone))
        at_start, per_corner, other = fm.attribute_entries(entries, corners)
        m['rtf_entries'] = len(entries)
        m['rtf_start'], m['rtf_per_corner'], m['rtf_other'] = at_start, per_corner, other
        m['flags'] = fm.flag_counts(fs['status'], fs['contour'])
        m['no_valid_control'] = int(np.count_nonzero(fs['status'] & fm.NO_VALID_CONTROL))
        m['min_obstacle_distance'] = _finite_min(fs['obstacle_distance'])
        m['min_limiter_scale'] = _finite_min(fs['limiter'])
    else:
        m['timeline'], m['phases'], m['flags'] = [], {}, {}
        m['rtf_entries'] = m['rtf_start'] = m['rtf_other'] = 0
        m['rtf_per_corner'] = []
        m['no_valid_control'] = 0
        m['min_obstacle_distance'] = m['min_limiter_scale'] = math.nan

    # CBF from the status markers, when recorded.
    cbf = rec.cbf.get(topic.replace('/field_state', '/markers')) if topic else None
    if cbf is not None and len(cbf['t']):
        inside = _window(cbf, start, end)
        m['cbf_bind'] = (
            int(np.count_nonzero(cbf['bind'][inside])),
            fm.rising_edges(cbf['bind'][inside]),
        )
        m['cbf_hold'] = (
            int(np.count_nonzero(cbf['hold'][inside])),
            fm.rising_edges(cbf['hold'][inside]),
        )

    # Smoothness of the commands.
    m['jerk'] = {}
    m['command_period'] = {}
    m['max_speed'] = {}
    for topic_name in command_topics(rec):
        cmd = rec.commands[topic_name]
        inside = _window(cmd, start, end)
        samples = list(
            zip(cmd['t'][inside], cmd['recv'][inside], cmd['v'][inside], cmd['w'][inside])
        )
        jerk, period = fm.second_differences(samples)
        m['jerk'][topic_name] = (fm.rms(jerk[:, 0]), fm.rms(jerk[:, 1]))
        m['command_period'][topic_name] = period
        m['max_speed'][topic_name] = (
            float(np.max(np.abs(cmd['v'][inside]))) if np.any(inside) else math.nan
        )

    # Clearance.
    if rec.clearance:
        m['min_costmap_clearance'] = _finite_min(
            np.asarray(rec.clearance['d'])[_window(rec.clearance, start, end)]
        )
    else:
        m['min_costmap_clearance'] = math.nan
    if rec.scan_min:
        m['min_scan_range'] = _finite_min(
            np.asarray(rec.scan_min['r'])[_window(rec.scan_min, start, end)]
        )
    else:
        m['min_scan_range'] = math.nan

    # Stopping after the goal ended.
    # NaN when the next goal started before the robot stopped, inf when it never stopped.
    if rec.odom_twist and len(rec.odom_twist['t']):
        window = _window(rec.odom_twist, end, until)
        latency = fm.stop_latency(
            rec.odom_twist['t'][window],
            rec.odom_twist['v'][window],
            rec.odom_twist['w'][window],
            end,
        )
        if math.isnan(latency) and until >= rec.end:
            latency = math.inf
        m['stop_latency'] = latency
    else:
        m['stop_latency'] = math.nan

    # Controller failures and recoveries in the episode.
    m['controller_aborts'] = 0
    m['recoveries'] = {}
    for action, messages in rec.statuses.items():
        kind = action_kind(action)
        if kind not in ('controller', 'recovery'):
            continue
        for _, g_start, g_end, status in goals_from_status(messages):
            if not start <= g_start <= end:
                continue
            if kind == 'controller' and status == 6:
                m['controller_aborts'] += 1
            elif kind == 'recovery':
                name = action.rstrip('/').split('/')[-1]
                m['recoveries'][name] = m['recoveries'].get(name, 0) + 1
    failures = {}
    for t, node, _, current in rec.bt_events:
        if start <= t <= end and current == 'FAILURE':
            failures[node] = failures.get(node, 0) + 1
    m['bt_failures'] = failures

    # Localisation jumps in the episode and the controller's reaction.
    m['jumps'] = []
    for t, shift, turn in jumps:
        if not start <= t <= end:
            continue
        entry = {'t': t - start, 'shift': shift, 'yaw_deg': math.degrees(turn)}
        if have_fs:
            peak, back = fm.recovery_after(
                fs['t'], fs['distance'], t, options.jump_window, options.converged
            )
            near = (fs['t'] >= t) & (fs['t'] <= t + options.jump_window)
            entry['d_peak'], entry['recovery'] = peak, back
            entry['rtf'] = bool(np.any(fs['phase'][near] == fm.PHASE_ROTATE_TO_FIELD))
        m['jumps'].append(entry)

    m['checks'] = checks(m, options)
    return m


def checks(m, options):
    """Return [(check, value text, limit text, passed or None)] for one episode's metrics."""
    result = []

    def add(name, value, limit, passed, fmt='.3f'):
        if isinstance(value, float) and not math.isfinite(value):
            result.append((name, 'n/a', limit, None))
        else:
            text = format(value, fmt) if isinstance(value, float) else str(value)
            result.append((name, text, limit, bool(passed)))

    def add_measured(name, value, limit, passed, fmt='.3f'):
        # inf is a measured failure (never converged, never stopped); NaN means not measured.
        if isinstance(value, float) and math.isinf(value):
            result.append((name, 'never', limit, False))
        else:
            add(name, value, limit, passed, fmt)

    status = m['status']
    if status == 'CANCELED':
        result.append(('goal status', status, 'SUCCEEDED', None))
    else:
        add('goal status', status, 'SUCCEEDED', status == 'SUCCEEDED')
    d = m['tracking_max']
    add_measured(
        'max |D| after convergence (m)',
        d,
        f'< {options.tracking_limit}',
        d < options.tracking_limit,
    )
    if m['kind'] == 'nav':
        miss = m['goal_miss']
        limit = options.xy_tolerance + options.stop_slack
        if status == 'SUCCEEDED':
            add('goal miss (m)', miss, f'<= {limit:.2f}', miss <= limit)
            yaw = m['goal_yaw_error']
            add(
                'goal yaw error (rad)',
                yaw,
                f'<= {options.yaw_tolerance}',
                yaw <= options.yaw_tolerance,
            )
        worst = max(m['rtf_per_corner'], default=0)
        add(
            'ROTATE_TO_FIELD entries per corner',
            worst,
            f'<= {options.max_rtf_per_corner}',
            worst <= options.max_rtf_per_corner,
            'd',
        )
    else:
        after = m['d_max_after_lap1']
        add('max |D| after lap 1 (m)', after, f'< {options.lap_limit}', after < options.lap_limit)
    add('NoValidControl samples', m['no_valid_control'], '0', m['no_valid_control'] == 0, 'd')
    latency = m['stop_latency']
    add_measured(
        'stop latency (s)',
        latency,
        f'<= {options.max_stop_latency}',
        latency <= options.max_stop_latency,
    )
    return result


# Session metrics -----------------------------------------------------------------------------


def speed_limit_periods(rec, options):
    """Return one row per speed-limit period: the limit and the largest commanded speeds.

    The first :data:`SPEED_LIMIT_GRACE` s after each change are skipped (the robot decelerates).
    """
    rows = []
    # A limit published several times (`ros2 topic pub -t 3`) is one period.
    limits = []
    for entry in rec.speed_limits:
        if not limits or limits[-1][1:] != entry[1:]:
            limits.append(entry)
    for i, (t0, percentage, limit) in enumerate(limits):
        t1 = limits[i + 1][0] if i + 1 < len(limits) else rec.end
        value = speed_limit_value(percentage, limit, options.max_linear_vel)
        row = {
            'start': t0 - rec.start,
            'duration': t1 - t0,
            'limit': limit,
            'percentage': bool(percentage),
            'limit_mps': value,
            'max_speed': {},
        }
        for topic in command_topics(rec):
            cmd = rec.commands[topic]
            inside = _window(cmd, t0 + SPEED_LIMIT_GRACE, t1)
            row['max_speed'][topic] = (
                float(np.max(np.abs(cmd['v'][inside]))) if np.any(inside) else math.nan
            )
        speeds = [v for v in row['max_speed'].values() if math.isfinite(v)]
        if math.isfinite(value) and speeds:
            row['excess'] = max(speeds) - value
        else:
            row['excess'] = math.nan
        rows.append(row)
    return rows


def session_jumps(rec):
    """Return the session's localisation jumps (:func:`field_metrics.localisation_jumps`)."""
    if not rec.map_odom or not rec.pose_odom or len(rec.map_odom['t']) < 2:
        return []
    return fm.localisation_jumps(
        (rec.map_odom['t'], rec.map_odom['x'], rec.map_odom['y'], rec.map_odom['yaw']),
        (rec.pose_odom['t'], rec.pose_odom['x'], rec.pose_odom['y'], rec.pose_odom['yaw']),
    )


def analyse(rec, options=None):
    """Return the report as a dict: episodes, speed limits, jumps, topics and notes."""
    options = options or Options()
    jumps = session_jumps(rec)
    found = episodes(rec)
    report = {
        'duration': rec.end - rec.start,
        'global_frame': rec.global_frame,
        'episodes': [
            episode_metrics(
                rec, e, i + 1, options, jumps, found[i + 1].start if i + 1 < len(found) else None
            )
            for i, e in enumerate(found)
        ],
        'speed_limits': speed_limit_periods(rec, options),
        'jumps': [
            {'t': t - rec.start, 'shift': s, 'yaw_deg': math.degrees(y)} for t, s, y in jumps
        ],
        'topics': rec.topics,
        'notes': list(rec.notes),
        'options': options.__dict__.copy(),
    }
    if not found:
        report['notes'].append(
            'no navigation or curve-following goal found: record the action status topics '
            '(record_field_test.py does)'
        )
    return report


# Markdown ------------------------------------------------------------------------------------


def _f(value, fmt='.3f'):
    if value is None:
        return '–'
    if isinstance(value, float):
        if math.isinf(value):
            return '∞' if value > 0 else '-∞'
        return format(value, fmt) if math.isfinite(value) else '–'
    return str(value)


def _timeline_text(timeline, limit=12):
    short = {
        'ROTATE_TO_FIELD': 'RTF',
        'FOLLOW': 'FOLLOW',
        'FINAL_APPROACH': 'FA',
        'ROTATE_TO_HEADING': 'RTH',
        'STOPPED': 'STOP',
    }
    parts = [f'{short.get(name, name)} {end - start:.1f}s' for name, start, end in timeline]
    if len(parts) > limit:
        parts = parts[: limit // 2] + [f'… {len(parts) - limit} more …'] + parts[-limit // 2 :]
    return ' → '.join(parts) if parts else '–'


def _verdict(passed):
    return {True: 'pass', False: '**FAIL**', None: '–'}[passed]


def render_markdown(report, bag_name='', images=None):
    """Return the report page as Markdown; ``images`` maps episode index -> PNG file name."""
    images = images or {}
    lines = [f'# Field-test report{": " + bag_name if bag_name else ""}', '']
    lines.append(
        f'Session: {report["duration"]:.1f} s, {len(report["episodes"])} episodes, '
        f'global frame `{report["global_frame"]}`.'
    )
    lines.append('')
    for note in report['notes']:
        lines.append(f'- Note: {note}')
    if report['notes']:
        lines.append('')

    lines += ['## Episodes', '']
    lines.append(
        '| # | Kind | Status | Start (s) | Time (s) | XTE RMS (m) | XTE max (m) | '
        'max \\|D\\| conv. (m) | Goal miss (m) | Yaw err (rad) | RTF entries | Checks |'
    )
    lines.append('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for m in report['episodes']:
        failed = sum(1 for c in m['checks'] if c[3] is False)
        passed = sum(1 for c in m['checks'] if c[3] is True)
        lines.append(
            f'| {m["index"]} | {m["kind"]} | {m["status"]} | {m["start"]:.1f} | '
            f'{m["duration"]:.1f} | {_f(m.get("xte_rms"))} | {_f(m.get("xte_max"))} | '
            f'{_f(m["tracking_max"])} | {_f(m.get("goal_miss"))} | '
            f'{_f(m.get("goal_yaw_error"))} | {m["rtf_entries"]} | '
            f'{passed} pass, {failed} fail |'
        )
    lines.append('')

    for m in report['episodes']:
        lines += _episode_section(m, images.get(m['index']))

    lines += ['## Speed limits', '']
    if report['speed_limits']:
        topics = sorted({t for row in report['speed_limits'] for t in row['max_speed']})
        lines.append(
            '| Start (s) | Duration (s) | Limit | Limit (m/s) | '
            + ' | '.join(f'max \\|v\\| `{t}`' for t in topics)
            + ' | Excess (m/s) |'
        )
        lines.append('|' + '---|' * (5 + len(topics)))
        for row in report['speed_limits']:
            limit = f'{row["limit"]:g} %' if row['percentage'] else f'{row["limit"]:g} m/s'
            speeds = ' | '.join(_f(row['max_speed'].get(t)) for t in topics)
            lines.append(
                f'| {row["start"]:.1f} | {row["duration"]:.1f} | {limit} | '
                f'{_f(row["limit_mps"])} | {speeds} | {_f(row["excess"])} |'
            )
        lines.append('')
        lines.append(
            f'The first {SPEED_LIMIT_GRACE:g} s after each change are skipped. A limit of 0 (or '
            '100 %) means no limit; percentages need `--max-linear-vel` to be checked.'
        )
    else:
        lines.append('No `speed_limit` messages.')
    lines.append('')

    lines += ['## Localisation corrections', '']
    if report['jumps']:
        lines.append('| t (s) | Shift (m) | Yaw (deg) |')
        lines.append('|---|---|---|')
        for jump in report['jumps']:
            lines.append(f'| {jump["t"]:.1f} | {jump["shift"]:.3f} | {jump["yaw_deg"]:.1f} |')
        lines.append('')
        lines.append(
            'Jumps: global → odom corrections that moved the robot by at least 5 cm or 2°. The '
            "episode sections list the controller's reaction to those during a goal."
        )
    else:
        lines.append('No global → odom correction of 5 cm or 2° or more.')
    lines.append('')

    lines += ['## Topics', '', '| Topic | Type | Messages | Rate (Hz) |', '|---|---|---|---|']
    for topic, (type_name, count, rate) in sorted(report['topics'].items()):
        lines.append(f'| `{topic}` | `{type_name}` | {count} | {_f(rate, ".1f")} |')
    lines.append('')

    o = report['options']
    lines += [
        '## Thresholds',
        '',
        f'- max |D| after convergence (first |D| <= {o["converged"]} m in FOLLOW or '
        f'FINAL_APPROACH): < {o["tracking_limit"]} m;',
        f'- curve follower, max |D| after lap 1: < {o["lap_limit"]} m;',
        f'- goal miss: <= {o["xy_tolerance"]} + {o["stop_slack"]} m, '
        f'{o["settle"]} s after the result; yaw error <= {o["yaw_tolerance"]} rad;',
        f'- ROTATE_TO_FIELD entries per plan corner: <= {o["max_rtf_per_corner"]};',
        f'- stop latency (odometry below 0.02 m/s and 0.05 rad/s for 0.3 s): '
        f'<= {o["max_stop_latency"]} s.',
        '',
    ]
    return '\n'.join(lines)


def _episode_section(m, image):
    title = 'Nav2 goal' if m['kind'] == 'nav' else 'Curve following'
    lines = [f'## Episode {m["index"]}: {title} ({m["status"]})', '']
    lines.append(
        f'`{m["action"]}`, from {m["start"]:.1f} s for {m["duration"]:.1f} s; '
        f'{_f(m["distance_travelled"], ".2f")} m travelled; field_state from '
        f'`{m["field_state_topic"] or "none"}` ({m["field_state_samples"]} samples).'
    )
    lines.append('')
    lines += ['| Check | Value | Limit | Result |', '|---|---|---|---|']
    for name, value, limit, passed in m['checks']:
        name = name.replace('|', '\\|')
        lines.append(f'| {name} | {value} | {limit} | {_verdict(passed)} |')
    lines.append('')

    rows = [
        ('\\|D\\| RMS / max, tracking phases (m)', f'{_f(m["d_rms"])} / {_f(m["d_max"])}'),
    ]
    if m['kind'] == 'nav':
        rows.append(
            (
                'cross-track error to the first /plan, RMS / max (m)',
                f'{_f(m.get("xte_rms"))} / {_f(m.get("xte_max"))}',
            )
        )
        if 'goal' in m:
            gx, gy, gyaw = m['goal']
            rows.append(('goal (last /plan pose)', f'({gx:.2f}, {gy:.2f}, {gyaw:.2f} rad)'))
        corners = ', '.join(f'{c["turn_deg"]:+.0f}°' for c in m['corners']) or 'none'
        rows.append(('plan corners', corners))
    else:
        rows.append(
            ('laps completed (feedback; the last lap ends with the result)', str(m['laps']))
        )
        rows.append(('first lap done after (s)', _f(m['lap1_time'], '.1f')))
        rows.append(
            (
                '\\|D\\| after lap 1, RMS / max (m)',
                f'{_f(m["d_rms_after_lap1"])} / {_f(m["d_max_after_lap1"])}',
            )
        )
    per_corner = ', '.join(str(n) for n in m['rtf_per_corner']) or '–'
    rows.append(
        (
            'ROTATE_TO_FIELD entries: start / per corner / elsewhere',
            f'{m["rtf_start"]} / {per_corner} / {m["rtf_other"]}',
        )
    )
    rows.append(('phases', _timeline_text(m['timeline'])))
    if m['phases']:
        rows.append(
            (
                'time per phase',
                ', '.join(f'{k} {v[0]:.1f} s ({v[1]}×)' for k, v in m['phases'].items()),
            )
        )
    flags = ', '.join(
        f'{name} {samples} samples / {count}×'
        for name, (samples, count) in m['flags'].items()
        if samples
    )
    rows.append(('status flags (samples / occurrences)', flags or 'none'))
    if 'cbf_bind' in m:
        rows.append(
            (
                'CBF binds / hold (samples / occurrences)',
                f'{m["cbf_bind"][0]} / {m["cbf_bind"][1]}×, '
                f'{m["cbf_hold"][0]} / {m["cbf_hold"][1]}×',
            )
        )
    for topic, (jv, jw) in m['jerk'].items():
        rows.append(
            (
                f'RMS jerk `{topic}`, linear / angular',
                f'{_f(jv, ".2f")} m/s³ / {_f(jw, ".2f")} rad/s³ '
                f'(period {_f(m["command_period"][topic], ".3f")} s, max \\|v\\| '
                f'{_f(m["max_speed"][topic], ".2f")} m/s)',
            )
        )
    rows.append(('min clearance to lethal costmap cells (m)', _f(m['min_costmap_clearance'])))
    rows.append(('min scan range (m)', _f(m['min_scan_range'])))
    rows.append(('min field_state obstacle_distance (m)', _f(m['min_obstacle_distance'])))
    rows.append(('min limiter scale', _f(m['min_limiter_scale'], '.2f')))
    rows.append(('stop latency after the result (s)', _f(m['stop_latency'], '.2f')))
    rows.append(('FollowPath aborts', str(m['controller_aborts'])))
    if m['recoveries']:
        rows.append(('recoveries', ', '.join(f'{k} ×{v}' for k, v in m['recoveries'].items())))
    if m['bt_failures']:
        rows.append(
            (
                'behaviour-tree failures',
                ', '.join(f'{k} ×{v}' for k, v in m['bt_failures'].items()),
            )
        )
    for jump in m['jumps']:
        rows.append(
            (
                f'localisation jump at {jump["t"]:.1f} s',
                f'{jump["shift"]:.3f} m, {jump["yaw_deg"]:+.1f}°; max \\|D\\| '
                f'{_f(jump.get("d_peak"))} m in the next seconds, back within the convergence '
                'band '
                f'after {_f(jump.get("recovery"), ".2f")} s'
                + ('; entered ROTATE_TO_FIELD' if jump.get('rtf') else ''),
            )
        )
    lines += ['| Metric | Value |', '|---|---|']
    lines += [f'| {name} | {value} |' for name, value in rows]
    lines.append('')
    if image:
        lines += [f'![Episode {m["index"]}]({image})', '']
    return lines


# Plots ---------------------------------------------------------------------------------------

PHASE_COLOURS = {
    fm.PHASE_ROTATE_TO_FIELD: '#e69f00',
    fm.PHASE_FOLLOW: '#009e73',
    fm.PHASE_FINAL_APPROACH: '#56b4e9',
    fm.PHASE_ROTATE_TO_HEADING: '#0072b2',
    fm.PHASE_STOPPED: '#d55e00',
}


def plot_episodes(rec, report, out_dir, options=None):
    """Write one PNG per episode into ``out_dir``; return {episode index: file name}.

    Top: the path (global frame: /plan and the robot for Nav2 goals; field_state frame: the
    closest points and the evaluation point for the follower). Middle: |D| over the phase bands.
    Bottom: the commanded v and w of every recorded command topic.
    """
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    options = options or Options()
    found = episodes(rec)
    files = {}
    for i, (m, episode) in enumerate(zip(report['episodes'], found)):
        start, end = episode.start, episode.end
        until = found[i + 1].start if i + 1 < len(found) else math.inf
        figure, (ax_xy, ax_d, ax_cmd) = plt.subplots(
            3, 1, figsize=(8, 11), gridspec_kw={'height_ratios': [3, 1.2, 1.2]}
        )
        topic = m['field_state_topic']
        fs = (
            _select(rec.field_states[topic], _window(rec.field_states[topic], start, end))
            if topic
            else {}
        )
        if episode.kind == 'nav':
            first, last = _plan_in(rec, start, end, until)
            if first is not None:
                ax_xy.plot(first[2][:, 0], first[2][:, 1], '--', color='0.5', label='first /plan')
            if last is not None and last is not first:
                ax_xy.plot(last[2][:, 0], last[2][:, 1], ':', color='0.7', label='last /plan')
            if rec.pose:
                inside = _window(rec.pose, start, end)
                ax_xy.plot(
                    rec.pose['x'][inside], rec.pose['y'][inside], color='#0072b2', label='robot'
                )
            for corner in m.get('corners', []):
                ax_xy.plot(corner['x'], corner['y'], 'k+', markersize=12)
            ax_xy.set_title(
                f'Episode {m["index"]}: Nav2 goal ({m["status"]}), {rec.global_frame} frame'
            )
        elif fs:
            ax_xy.plot(fs['cl_x'], fs['cl_y'], '--', color='0.5', label='closest point')
            ax_xy.plot(fs['cp_x'], fs['cp_y'], color='#0072b2', label='evaluation point')
            frame = rec.field_state_frames.get(topic, rec.odom_frame)
            ax_xy.set_title(
                f'Episode {m["index"]}: curve following ({m["status"]}), {frame} frame'
            )
        if fs and episode.kind == 'nav':
            rtf = np.flatnonzero(np.diff(np.concatenate(([-1], fs['phase']))) != 0)
            for i in rtf:
                if fs['phase'][i] == fm.PHASE_ROTATE_TO_FIELD:
                    at = _pose_at(rec, fs['t'][i])
                    if at is not None:
                        ax_xy.plot(at[0], at[1], 'o', color=PHASE_COLOURS[0], markersize=7)
        ax_xy.set_aspect('equal', adjustable='datalim')
        ax_xy.grid(True, alpha=0.3)
        ax_xy.legend(loc='best', fontsize=8)
        ax_xy.set_xlabel('x (m)')
        ax_xy.set_ylabel('y (m)')

        if fs:
            t = fs['t'] - start
            for phase, a, b in fm.phase_segments(fs['t'], fs['phase']):
                ax_d.axvspan(
                    a - start, b - start, color=PHASE_COLOURS.get(phase, '0.8'), alpha=0.2, lw=0
                )
            ax_d.plot(t, np.abs(fs['distance']), color='k', lw=1)
            ax_d.axhline(options.tracking_limit, color='#d55e00', ls='--', lw=0.8)
        ax_d.set_ylabel('|D| (m)')
        ax_d.grid(True, alpha=0.3)
        for topic_name in command_topics(rec):
            cmd = rec.commands[topic_name]
            inside = _window(cmd, start, end)
            ax_cmd.plot(cmd['t'][inside] - start, cmd['v'][inside], lw=1, label=f'v {topic_name}')
            ax_cmd.plot(
                cmd['t'][inside] - start, cmd['w'][inside], lw=1, ls='--', label=f'w {topic_name}'
            )
        ax_cmd.set_xlabel('time since the goal (s)')
        ax_cmd.set_ylabel('v (m/s), w (rad/s)')
        ax_cmd.grid(True, alpha=0.3)
        if command_topics(rec):
            ax_cmd.legend(loc='best', fontsize=7, ncol=2)
        figure.tight_layout()
        name = f'episode_{m["index"]:02d}.png'
        figure.savefig(f'{out_dir}/{name}', dpi=90)
        plt.close(figure)
        files[m['index']] = name
    return files


# Loading -------------------------------------------------------------------------------------


def _twist_of(message):
    return message.twist if hasattr(message, 'twist') and hasattr(message, 'header') else message


def _cbf_flags(message):
    """Return (bind, hold) of a vector-field MarkerArray: the CBF flag and the hold label."""
    bind = hold = False
    for marker in message.markers:
        if marker.ns == 'status' and marker.text:
            bind = bind or 'CBF' in marker.text.split()
        elif marker.ns == 'contour' and marker.text.startswith('cbf'):
            hold = hold or '| hold' in marker.text
    return bind, hold


def load_recording(uri, odom_topic='', base_frame='', global_frame='', warn=print):
    """Read a bag into a :class:`Recording`.

    Args:
        uri: The bag directory (or file).
        odom_topic: The Odometry topic ('' = ``/odom`` if recorded, else the first one).
        base_frame: The robot frame ('' = base_footprint if in TF, else base_link).
        global_frame: The plan frame ('' = the frame of /plan, else map, else the odometry
            frame).
        warn: Called with warning text.

    """
    import rclpy.duration
    import rclpy.time
    import tf2_ros

    from vectorfield_bringup import bag

    reader = bag.open_reader(uri)
    types = bag.topic_types(reader)
    duration, counts = bag.summary(reader)
    del reader

    def of_type(*names):
        return [t for t, ty in types.items() if ty in names]

    rec = Recording()
    odometry = of_type('nav_msgs/msg/Odometry')
    if odom_topic:
        odometry = [odom_topic] if odom_topic in types else []
    elif '/odom' in odometry:
        odometry = ['/odom']
    else:
        odometry = odometry[:1]
    if not odometry:
        rec.notes.append(
            'no Odometry topic: no stop latencies, and the time map uses other stamps'
        )
    field_topics = of_type('vectorfield_msgs/msg/FieldState')
    marker_topics = [
        t
        for t in of_type('visualization_msgs/msg/MarkerArray')
        if t.replace('/markers', '/field_state') in field_topics
    ]
    command_topics_ = [
        t
        for t in of_type('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped')
        if t.rstrip('/').split('/')[-1] in COMMAND_CHAIN
    ]
    status_topics = [
        t for t in of_type('action_msgs/msg/GoalStatusArray') if t.endswith('/_action/status')
    ]
    feedback_topics = of_type('vectorfield_msgs/action/FollowCurve_FeedbackMessage')
    plans = [t for t in of_type('nav_msgs/msg/Path') if t.rstrip('/').split('/')[-1] == 'plan']
    costmaps = [t for t in of_type('nav_msgs/msg/OccupancyGrid') if 'local_costmap' in t]
    costmaps = costmaps or [t for t in of_type('nav_msgs/msg/OccupancyGrid') if 'costmap' in t]
    scans = of_type('sensor_msgs/msg/LaserScan')[:1]
    wanted = (
        ['/tf', '/tf_static']
        + odometry
        + field_topics
        + marker_topics
        + command_topics_
        + status_topics
        + feedback_topics
        + plans[:1]
        + costmaps[:1]
        + scans
        + of_type('nav2_msgs/msg/SpeedLimit')
        + of_type('nav2_msgs/msg/BehaviorTreeLog')
    )
    for topic in ('/tf', '/tf_static'):
        if topic not in types:
            rec.notes.append(f'{topic} not recorded: no robot poses')

    buffer = tf2_ros.Buffer(
        cache_time=rclpy.duration.Duration(seconds=max(duration + 120.0, 7200.0))
    )
    frames = set()
    raw = {t: [] for t in wanted}
    stamp_pairs = []
    first_recv = math.inf
    last_recv = -math.inf
    for topic, message, recv in bag.read_messages(uri, wanted, warn):
        first_recv = min(first_recv, recv)
        last_recv = max(last_recv, recv)
        if topic in ('/tf', '/tf_static'):
            for transform in message.transforms:
                frames.add(transform.header.frame_id)
                frames.add(transform.child_frame_id)
                if topic == '/tf_static':
                    raw['/tf_static'].append((recv, transform))
                else:
                    buffer.set_transform(transform, 'bag')
                    raw['/tf'].append((recv, transform))
            continue
        if topic in odometry:
            stamp_pairs.append((recv, bag.stamp_seconds(message.header.stamp)))
        raw[topic].append((recv, message))

    if not odometry:
        for topic in field_topics:
            stamp_pairs += [(r, bag.stamp_seconds(m.header.stamp)) for r, m in raw[topic]]
    time_map = bag.TimeMap([p[0] for p in stamp_pairs], [p[1] for p in stamp_pairs])
    if not math.isfinite(first_recv):
        raise RuntimeError(f'{uri}: none of the expected topics has messages')
    rec.start, rec.end = float(time_map(first_recv)), float(time_map(last_recv))
    steps = _static_transforms(buffer, raw['/tf_static'], time_map, rec.start, rec.end)

    # Frames.
    if not base_frame:
        base_frame = 'base_footprint' if 'base_footprint' in frames else 'base_link'
    # The odometry frame: the Odometry header's, else the controller's field_state frame (the
    # follower's control frame can be map).
    controller_topics = [t for t in field_topics if 'curve_follower' not in t]
    if odometry and raw[odometry[0]]:
        rec.odom_frame = raw[odometry[0]][0][1].header.frame_id or 'odom'
    elif controller_topics and raw[controller_topics[0]]:
        rec.odom_frame = raw[controller_topics[0]][0][1].header.frame_id or 'odom'
    for topic in field_topics:
        if raw[topic]:
            rec.field_state_frames[topic] = raw[topic][0][1].header.frame_id
    if not global_frame:
        if plans and raw[plans[0]]:
            global_frame = raw[plans[0]][0][1].header.frame_id or 'map'
        else:
            global_frame = 'map' if 'map' in frames else rec.odom_frame
    rec.global_frame = global_frame

    # Pose samples at the odometry stamps (at most 20 Hz), else on a 20 Hz grid.
    if odometry and raw[odometry[0]]:
        times = np.array([bag.stamp_seconds(m.header.stamp) for _, m in raw[odometry[0]]])
        twist = [(bag.stamp_seconds(m.header.stamp), m.twist.twist) for _, m in raw[odometry[0]]]
        rec.odom_twist = {
            't': np.array([t for t, _ in twist]),
            'v': np.array([math.hypot(w.linear.x, w.linear.y) for _, w in twist]),
            'w': np.array([w.angular.z for _, w in twist]),
        }
    else:
        times = np.arange(rec.start, rec.end, 0.05)
    kept, last = [], -math.inf
    for t in np.sort(times):
        if t - last >= 0.05:
            kept.append(t)
            last = t

    def track(target):
        rows = []
        for t in kept:
            try:
                tf = buffer.lookup_transform(target, base_frame, rclpy.time.Time(seconds=float(t)))
            except tf2_ros.TransformException:
                continue
            q = tf.transform.rotation
            rows.append((t, tf.transform.translation.x, tf.transform.translation.y, bag.yaw_of(q)))
        array = np.array(rows, dtype=float).reshape(-1, 4)
        return {'t': array[:, 0], 'x': array[:, 1], 'y': array[:, 2], 'yaw': array[:, 3]}

    rec.pose = track(global_frame)
    rec.pose_odom = track(rec.odom_frame) if rec.odom_frame != global_frame else rec.pose
    if len(rec.pose['t']) == 0:
        rec.notes.append(f'no TF from {global_frame} to {base_frame}: no robot poses')

    # global -> odom as published (localisation corrections).
    corrections = [
        (bag.stamp_seconds(tf.header.stamp), tf.transform)
        for tf in [t for _, t in raw.get('/tf', [])] + steps
        if tf.header.frame_id == global_frame and tf.child_frame_id == rec.odom_frame
    ]
    corrections.sort(key=lambda item: item[0])
    rec.map_odom = {
        't': np.array([t for t, _ in corrections]),
        'x': np.array([tr.translation.x for _, tr in corrections]),
        'y': np.array([tr.translation.y for _, tr in corrections]),
        'yaw': np.array([bag.yaw_of(tr.rotation) for _, tr in corrections]),
    }

    for topic in field_topics:
        messages = [m for _, m in raw[topic]]
        rec.field_states[topic] = {
            't': np.array([bag.stamp_seconds(m.header.stamp) for m in messages]),
            'phase': np.array([m.phase for m in messages], dtype=int),
            'status': np.array([m.status for m in messages], dtype=np.int64),
            'distance': np.array([m.distance for m in messages]),
            'obstacle_distance': np.array([m.obstacle_distance for m in messages]),
            'contour': np.array([m.contour_direction for m in messages], dtype=int),
            'limiter': np.array([m.limiter_scale for m in messages]),
            'cp_x': np.array([m.control_point.x for m in messages]),
            'cp_y': np.array([m.control_point.y for m in messages]),
            'cl_x': np.array([m.closest_point.x for m in messages]),
            'cl_y': np.array([m.closest_point.y for m in messages]),
        }
    for topic in marker_topics:
        flags = [(time_map(r), *_cbf_flags(m)) for r, m in raw[topic]]
        rec.cbf[topic] = {
            't': np.array([f[0] for f in flags]),
            'bind': np.array([f[1] for f in flags], dtype=bool),
            'hold': np.array([f[2] for f in flags], dtype=bool),
        }
    for topic in command_topics_:
        rows = [(time_map(r), r, _twist_of(m)) for r, m in raw[topic]]
        rec.commands[topic] = {
            't': np.array([t for t, _, _ in rows]),
            'recv': np.array([r for _, r, _ in rows]),
            'v': np.array([w.linear.x for _, _, w in rows]),
            'vy': np.array([w.linear.y for _, _, w in rows]),
            'w': np.array([w.angular.z for _, _, w in rows]),
        }
    for topic in status_topics:
        action = topic[: -len('/_action/status')]
        rec.statuses[action] = [
            (
                float(time_map(r)),
                [
                    (
                        bytes(s.goal_info.goal_id.uuid).hex(),
                        s.status,
                        bag.stamp_seconds(s.goal_info.stamp),
                    )
                    for s in m.status_list
                ],
            )
            for r, m in raw[topic]
        ]
    for topic in feedback_topics:
        action = topic[: -len('/_action/feedback')]
        rec.laps[action] = {
            't': np.array([time_map(r) for r, _ in raw[topic]]),
            'lap': np.array([m.feedback.lap for _, m in raw[topic]], dtype=int),
        }
    for topic in plans[:1]:
        for _, m in raw[topic]:
            points = np.array(
                [(p.pose.position.x, p.pose.position.y) for p in m.poses], dtype=float
            ).reshape(-1, 2)
            yaw = bag.yaw_of(m.poses[-1].pose.orientation) if m.poses else 0.0
            rec.plans.append((bag.stamp_seconds(m.header.stamp), m.header.frame_id, points, yaw))
    for topic in of_type('nav2_msgs/msg/SpeedLimit'):
        rec.speed_limits += [
            (float(time_map(r)), bool(m.percentage), float(m.speed_limit)) for r, m in raw[topic]
        ]
    rec.speed_limits.sort(key=lambda item: item[0])
    for topic in of_type('nav2_msgs/msg/BehaviorTreeLog'):
        for _, m in raw[topic]:
            rec.bt_events += [
                (bag.stamp_seconds(e.timestamp), e.node_name, e.previous_status, e.current_status)
                for e in m.event_log
            ]
    for topic in scans:
        rows = []
        for _, m in raw[topic]:
            ranges = np.asarray(m.ranges, dtype=float)
            valid = np.isfinite(ranges) & (ranges >= m.range_min) & (ranges <= m.range_max)
            rows.append(
                (
                    bag.stamp_seconds(m.header.stamp),
                    float(np.min(ranges[valid])) if np.any(valid) else math.inf,
                )
            )
        rec.scan_min = {'t': np.array([r[0] for r in rows]), 'r': np.array([r[1] for r in rows])}
    for topic in costmaps[:1]:
        grids = [(bag.stamp_seconds(m.header.stamp), m.header.frame_id, m) for _, m in raw[topic]]
        grids.sort(key=lambda item: item[0])
        if grids:
            rec.clearance = _clearance(rec, grids, buffer, base_frame)

    for topic, type_name in sorted(types.items()):
        count = counts.get(topic, 0)
        rec.topics[topic] = (type_name, count, count / duration if duration > 0 else math.nan)
    for topic in ('/plan',):
        if topic not in types:
            rec.notes.append(f'{topic} not recorded: no cross-track error to the plan')
    if not status_topics:
        rec.notes.append(
            'no action status topics: no episodes (record with --include-hidden-topics)'
        )
    return rec


def _transform_key(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return tuple(round(v, 6) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w))


def _static_transforms(buffer, transforms, time_map, start, end):
    """Insert the /tf_static transforms; return those of the pairs that changed, as steps.

    A static transform that is published again with another value (vectorfield_sim's map ->
    odom after ``step_map_to_odom``; a localisation that publishes statically) would otherwise
    replace the old value for all times. Such a pair becomes a step function of dynamic
    transforms: each value holds from its stamp until just before the next one.
    """
    import copy

    from vectorfield_bringup import bag

    groups = {}
    for recv, transform in transforms:
        key = (transform.header.frame_id, transform.child_frame_id)
        groups.setdefault(key, []).append((recv, transform))
    steps = []
    for entries in groups.values():
        if len({_transform_key(t) for _, t in entries}) == 1:
            buffer.set_transform_static(entries[-1][1], 'bag')
            continue
        timed = []
        for recv, transform in entries:
            stamp = bag.stamp_seconds(transform.header.stamp)
            timed.append((stamp if stamp > 0.0 else float(time_map(recv)), transform))
        timed.sort(key=lambda item: item[0])
        for i, (t, transform) in enumerate(timed):
            hold_from = min(t, start - 1.0) if i == 0 else t
            hold_to = timed[i + 1][0] - 1e-4 if i + 1 < len(timed) else end + 1.0
            for when in (hold_from, hold_to):
                stamped = copy.deepcopy(transform)
                when = max(when, 0.0)
                stamped.header.stamp.sec = int(math.floor(when))
                stamped.header.stamp.nanosec = int((when - math.floor(when)) * 1e9)
                buffer.set_transform(stamped, 'bag')
                steps.append(stamped)
    return steps


def _clearance(rec, grids, buffer, base_frame):
    """Return {t, d}: the robot's distance to the lethal cells of the latest costmap."""
    import rclpy.time
    import tf2_ros

    cells = {}
    rows = []
    stamps = [g[0] for g in grids]
    for t in rec.pose['t']:
        index = int(np.searchsorted(stamps, t, side='right')) - 1
        if index < 0:
            continue
        _, frame, grid = grids[index]
        if index not in cells:
            cells[index] = fm.OccupiedCells(grid, threshold=fm.LETHAL)
        try:
            tf = buffer.lookup_transform(frame, base_frame, rclpy.time.Time(seconds=float(t)))
        except tf2_ros.TransformException:
            continue
        position = tf.transform.translation
        rows.append((t, cells[index].distance(position.x, position.y)))
    return {'t': np.array([r[0] for r in rows]), 'd': np.array([r[1] for r in rows])}
