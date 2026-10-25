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

"""Tracking, smoothness and clearance metrics on plain arrays.

Shared by ``field_test_report.py`` (recorded field tests), the controller comparison of
``vectorfield_system_tests`` (``compare_controllers.py``) and its launch-test helpers. Only numpy:
no ROS imports, so the functions can be tested on synthetic data.

The ``FieldState`` constants are mirrored here (:data:`PHASE_NAMES`, :data:`STATUS_BITS`) so that
the module does not need ``vectorfield_msgs``.
"""

import math

import numpy as np

#: Commands received less than this apart (wall s) count as one control update, the later of which
#: the robot keeps. The controller server publishes about every 50 ms, with occasional
#: back-to-back commands when its loop catches up.
BURST_GAP = 0.02

#: ``FieldState.PHASE_*``.
PHASE_ROTATE_TO_FIELD = 0
PHASE_FOLLOW = 1
PHASE_FINAL_APPROACH = 2
PHASE_ROTATE_TO_HEADING = 3
PHASE_STOPPED = 4
PHASE_NAMES = {
    PHASE_ROTATE_TO_FIELD: 'ROTATE_TO_FIELD',
    PHASE_FOLLOW: 'FOLLOW',
    PHASE_FINAL_APPROACH: 'FINAL_APPROACH',
    PHASE_ROTATE_TO_HEADING: 'ROTATE_TO_HEADING',
    PHASE_STOPPED: 'STOPPED',
}
#: Phases in which the robot tracks the path (the system tests' tracking criterion).
TRACKING_PHASES = (PHASE_FOLLOW, PHASE_FINAL_APPROACH)

#: ``FieldState.STATUS_*`` bit -> name.
STATUS_BITS = {
    1: 'NON_FINITE',
    2: 'ASSUMPTION3_CLAMPED',
    4: 'AMBIGUOUS_CLOSEST',
    8: 'HAIRPIN',
    16: 'CURVATURE_SATURATED',
    32: 'CONTOUR_WATCHDOG',
    64: 'HEADING_INFEASIBLE',
    128: 'COLLISION_AHEAD',
    256: 'BLOCKED',
}
#: Status bits for which the controller raises NoValidControl.
NO_VALID_CONTROL = 1 | 32 | 64 | 256

#: OccupancyGrid value of a lethal costmap cell (Nav2 publishes cost 254 as 100, 253 as 99).
LETHAL = 100


def rms(values):
    """Return the root mean square (NaN for no values)."""
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values**2))) if values.size else math.nan


def wrap_angle(angle):
    """Return ``angle`` wrapped to [-pi, pi) (arrays element-wise)."""
    return (np.asarray(angle, dtype=float) + math.pi) % (2.0 * math.pi) - math.pi


def polyline_distance(points, polyline):
    """Return the distance of each point (N x 2) to a polyline (M x 2)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    polyline = np.asarray(polyline, dtype=float).reshape(-1, 2)
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
    """Return the last (wall t, v, w) of each burst of commands (see :data:`BURST_GAP`).

    ``samples`` are (t, wall t, v, w) rows; only the wall time is used.
    """
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


def tracking_error(t, phase, distance, converged=0.05, after=-math.inf):
    """Return max |D| over the tracking samples from the first one with |D| <= ``converged``.

    The tracking criterion of the launch tests: only samples in :data:`TRACKING_PHASES` and
    later than ``after`` count. Returns inf if |D| never got that small, NaN without tracking
    samples.
    """
    t, phase, distance = (np.asarray(a, dtype=float) for a in (t, phase, distance))
    tracking = np.isin(phase, TRACKING_PHASES) & (t > after)
    d = np.abs(distance[tracking])
    if d.size == 0:
        return math.nan
    hits = np.flatnonzero(d <= converged)
    return float(np.max(d[hits[0] :])) if hits.size else math.inf


def phase_segments(t, phase):
    """Return the runs of equal phase as (phase, t_start, t_end) tuples.

    A run ends where the next one starts (the last one at its last sample).
    """
    t = np.asarray(t, dtype=float)
    phase = np.asarray(phase, dtype=int)
    if t.size == 0:
        return []
    starts = np.concatenate(([0], np.flatnonzero(np.diff(phase)) + 1))
    ends = np.concatenate((starts[1:], [t.size - 1]))
    return [(int(phase[s]), float(t[s]), float(t[e])) for s, e in zip(starts, ends)]


def phase_durations(segments):
    """Return {phase name: (total duration s, entries)} of :func:`phase_segments` output."""
    result = {}
    for phase, start, end in segments:
        name = PHASE_NAMES.get(phase, str(phase))
        total, entries = result.get(name, (0.0, 0))
        result[name] = (total + end - start, entries + 1)
    return result


def rising_edges(mask):
    """Return how many times a boolean series becomes true (a true first sample counts)."""
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0:
        return 0
    return int(mask[0]) + int(np.count_nonzero(mask[1:] & ~mask[:-1]))


def flag_counts(status, contour_direction=None):
    """Return {flag: (samples, occurrences)} for the status bits and the contour episodes.

    ``occurrences`` counts the rising edges; the contour episode is ``contour_direction != 0``
    (``CONTOUR``). Flags that never occur are included with zeros.
    """
    status = np.asarray(status, dtype=np.int64)
    counts = {}
    for bit, name in STATUS_BITS.items():
        mask = (status & bit) != 0
        counts[name] = (int(np.count_nonzero(mask)), rising_edges(mask))
    if contour_direction is not None:
        mask = np.asarray(contour_direction) != 0
        counts['CONTOUR'] = (int(np.count_nonzero(mask)), rising_edges(mask))
    return counts


def resample_polyline(points, step=0.05):
    """Return the polyline resampled every ``step`` m of arc length, and the arc lengths."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < 2:
        return points, np.zeros(len(points))
    seg = np.hypot(*np.diff(points, axis=0).T)
    keep = np.concatenate(([True], seg > 1e-9))
    points = points[keep]
    s = np.concatenate(([0.0], np.cumsum(seg[seg > 1e-9])))
    if s[-1] <= 0.0:
        return points[:1], np.zeros(1)
    grid = np.linspace(0.0, s[-1], max(2, int(math.ceil(s[-1] / step)) + 1))
    return np.column_stack(
        (np.interp(grid, s, points[:, 0]), np.interp(grid, s, points[:, 1]))
    ), grid


def plan_corners(points, min_turn=math.radians(45.0), window=1.0, step=0.05):
    """Return the corners of a path as (x, y, turn rad, arc length) tuples.

    At every point (resampled every ``step`` m) the direction of the chord over the next
    ``window``/2 m is compared with that of the chord over the previous ``window``/2 m; each run
    where they differ by more than ``min_turn`` is one corner, at the point with the largest
    difference; ``turn`` is the signed heading change (positive to the left) between the chords
    just before and after that run. Chords average
    out the staircase steps of grid planners, and a corner chamfered over less than ``window`` / 2
    (a planner's path around a wall corner) still counts as one.
    """
    pts, s = resample_polyline(points, step)
    k = max(1, int(round(0.5 * window / step)))
    if len(pts) <= 2 * k:
        return []
    before = pts[k:-k] - pts[: -2 * k]
    after = pts[2 * k :] - pts[k:-k]
    change = wrap_angle(
        np.arctan2(after[:, 1], after[:, 0]) - np.arctan2(before[:, 1], before[:, 0])
    )
    mask = np.abs(change) >= min_turn
    corners = []
    i = 0
    while i < mask.size:
        if not mask[i]:
            i += 1
            continue
        end = i
        while end + 1 < mask.size and mask[end + 1]:
            end += 1
        peak = i + int(np.argmax(np.abs(change[i : end + 1])))
        centre = k + peak
        turn = float(change[peak])
        if centre - 2 * k >= 0 and centre + 2 * k < len(pts):
            # The turn between the legs, from the chords just outside the corner region (the
            # chords through a chamfered corner see only part of it).
            a = pts[centre - k] - pts[centre - 2 * k]
            b = pts[centre + 2 * k] - pts[centre + k]
            turn = float(wrap_angle(math.atan2(b[1], b[0]) - math.atan2(a[1], a[0])))
        x, y = pts[centre]
        corners.append((float(x), float(y), turn, float(s[centre])))
        i = end + 1
    return corners


def attribute_entries(entries, corners, radius=1.0):
    """Assign rotate-in-place entries to corners.

    Args:
        entries: (x, y, travelled) per entry: the position and the path length the robot had
            travelled in the episode when it entered the phase.
        corners: :func:`plan_corners` output.
        radius: An entry within this distance of a corner belongs to it (m).

    Returns:
        ``(start, per_corner, other)``: entries before the robot travelled 0.3 m (the turn
        towards the path), a list with the entry count of each corner, and the rest.

    """
    start, other = 0, 0
    per_corner = [0] * len(corners)
    for x, y, travelled in entries:
        if travelled < 0.3:
            start += 1
            continue
        best, best_d = None, radius
        for i, (cx, cy, _, _) in enumerate(corners):
            d = math.hypot(x - cx, y - cy)
            if d <= best_d:
                best, best_d = i, d
        if best is None:
            other += 1
        else:
            per_corner[best] += 1
    return start, per_corner, other


def path_length(x, y):
    """Return the cumulative path length of a trajectory (0 at the first sample)."""
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    if x.size == 0:
        return np.zeros(0)
    return np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))


def stop_latency(t, v, w, t0, v_still=0.02, w_still=0.05, hold=0.3):
    """Return the time from ``t0`` until the robot is still for ``hold`` s (NaN if never).

    ``v`` and ``w`` are the measured linear and angular speeds (odometry twist) at ``t``.
    """
    t, v, w = (np.asarray(a, dtype=float) for a in (t, v, w))
    select = t >= t0
    t, still = t[select], (np.abs(v[select]) <= v_still) & (np.abs(w[select]) <= w_still)
    run_start = None
    for ti, s in zip(t, still):
        if s:
            run_start = ti if run_start is None else run_start
            if ti - run_start >= hold:
                return float(max(run_start - t0, 0.0))
        else:
            run_start = None
    return math.nan


def interpolate_pose(t, pose_t, x, y, yaw):
    """Return (x, y, yaw) of a sampled trajectory at the times ``t`` (yaw unwrapped, then wrapped).

    Times outside the samples take the first or the last pose.
    """
    yaw_u = np.unwrap(np.asarray(yaw, dtype=float))
    return (
        np.interp(t, pose_t, x),
        np.interp(t, pose_t, y),
        wrap_angle(np.interp(t, pose_t, yaw_u)),
    )


def localisation_jumps(
    map_odom, robot_odom, min_shift=0.05, min_yaw=math.radians(2.0), max_gap=math.inf
):
    """Return the map -> odom corrections that moved the robot in the map frame.

    Args:
        map_odom: (t, x, y, yaw) arrays of the map -> odom transform, as published.
        robot_odom: (t, x, y, yaw) arrays of the robot pose in odom.
        min_shift: A correction moving the robot's map position by at least this much (m) ...
        min_yaw: ... or turning it by at least this much (rad) is a jump.
        max_gap: Consecutive transforms further apart than this (s) are not compared.

    Returns:
        A list of (t, shift m, yaw change rad) per jump.

    """
    mt, mx, my, myaw = (np.asarray(a, dtype=float) for a in map_odom)
    if mt.size < 2 or len(robot_odom[0]) == 0:
        return []
    px, py, _ = interpolate_pose(mt, *robot_odom)
    # The robot's map position under each transform.
    rx = mx + np.cos(myaw) * px - np.sin(myaw) * py
    ry = my + np.sin(myaw) * px + np.cos(myaw) * py
    jumps = []
    for i in range(1, mt.size):
        if mt[i] - mt[i - 1] > max_gap:
            continue
        # The same odom point under the previous and the new transform.
        prev_x = mx[i - 1] + math.cos(myaw[i - 1]) * px[i] - math.sin(myaw[i - 1]) * py[i]
        prev_y = my[i - 1] + math.sin(myaw[i - 1]) * px[i] + math.cos(myaw[i - 1]) * py[i]
        shift = math.hypot(rx[i] - prev_x, ry[i] - prev_y)
        turn = float(wrap_angle(myaw[i] - myaw[i - 1]))
        if shift >= min_shift or abs(turn) >= min_yaw:
            jumps.append((float(mt[i]), shift, turn))
    return jumps


def recovery_after(t, distance, t0, window=3.0, converged=0.05):
    """Return (max |D| in [t0, t0 + window], time from t0 until |D| <= converged again).

    The recovery time is 0 when |D| stays within ``converged``, NaN when it does not come back
    before the samples end.
    """
    t, d = np.asarray(t, dtype=float), np.abs(np.asarray(distance, dtype=float))
    near = (t >= t0) & (t <= t0 + window)
    peak = float(np.max(d[near])) if np.any(near) else math.nan
    later = t >= t0
    if not np.any(later):
        return peak, math.nan
    tl, dl = t[later], d[later]
    above = np.flatnonzero(dl > converged)
    if above.size == 0:
        return peak, 0.0
    back = np.flatnonzero((dl <= converged) & (np.arange(dl.size) > above[0]))
    return peak, float(tl[back[0]] - t0) if back.size else math.nan


class OccupiedCells:
    """The occupied cells of an ``OccupancyGrid`` (axis-aligned origin), as squares."""

    def __init__(self, grid, threshold=65):
        info = grid.info
        cells = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
        rows, columns = np.nonzero(cells >= threshold)
        self._half = 0.5 * info.resolution
        self._x = info.origin.position.x + (columns + 0.5) * info.resolution
        self._y = info.origin.position.y + (rows + 0.5) * info.resolution

    @property
    def count(self):
        """Return the number of occupied cells."""
        return int(self._x.size)

    def distance(self, x, y):
        """Return the distance from (x, y) to the nearest occupied cell (0 inside one)."""
        if self._x.size == 0:
            return math.inf
        gap_x = np.maximum(np.abs(self._x - x) - self._half, 0.0)
        gap_y = np.maximum(np.abs(self._y - y) - self._half, 0.0)
        return float(np.min(np.hypot(gap_x, gap_y)))
