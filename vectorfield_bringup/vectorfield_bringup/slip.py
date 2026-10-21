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

r"""Skid-steer slip identification from commanded twists and measured poses.

The slip model of ``docs/motion_models.md`` (skid-slip) and of ``vectorfield_sim``: the base
realises the yaw rate :math:`\omega = \eta_\psi\,\omega_{cmd}` and turns about a centre of rotation
at body :math:`x = x_{ICR}`, so ``base_link`` drifts sideways with :math:`v_y = -x_{ICR}\,\omega`.
The controller's ``skid_yaw_rate_efficiency`` and ``skid_icr_offset`` are these two numbers.

Given the commands and a pose track (odometry that sees the slip, a localisation pose or motion
capture) over segments of constant command, :func:`estimate` returns per segment

- :math:`\eta_\psi`: realised over commanded yaw rate, the realised one from the yaw change over
  the steady part of the segment;
- :math:`x_{ICR} = -\sum v_y\,\omega / \sum \omega^2` from the body-frame velocities of pose
  pairs (:func:`body_twists`);
- :math:`\eta_v`: realised over commanded forward speed (not a parameter; a diagnostic),

and the pooled values with their spread. Only numpy.
"""

import math
from dataclasses import dataclass, field

import numpy as np

#: Yaw rates below this (rad/s) do not identify anything.
MIN_YAW_RATE = 0.1


@dataclass
class Segment:
    """A segment of constant command and what was measured over its steady part."""

    t_start: float
    t_end: float
    v_cmd: float
    w_cmd: float
    w_real: float = math.nan
    vx_real: float = math.nan
    eta_psi: float = math.nan
    x_icr: float = math.nan
    eta_v: float = math.nan
    pairs: int = 0
    note: str = ''


@dataclass
class SlipEstimate:
    """The pooled estimate over all segments."""

    segments: list = field(default_factory=list)
    eta_psi: float = math.nan
    eta_psi_spread: float = math.nan
    x_icr: float = math.nan
    x_icr_spread: float = math.nan
    eta_v: float = math.nan


def unwrap_yaw(yaw):
    """Return a continuous yaw track."""
    return np.unwrap(np.asarray(yaw, dtype=float))


def body_twists(t, x, y, yaw, min_dt=0.1):
    r"""Return (t_mid, vx, vy, w) of pose pairs at least ``min_dt`` s apart.

    For a constant body twist :math:`(v_x, v_y, \omega)` over :math:`\Delta t`, the displacement
    expressed in the frame of the mid heading is :math:`\Delta t\,\mathrm{sinc}(\omega\Delta t/2)
    (v_x, v_y)`; the pairs are corrected by that factor, so arcs are exact.
    """
    t, x, y = (np.asarray(a, dtype=float) for a in (t, x, y))
    yaw = unwrap_yaw(yaw)
    out = []
    i = 0
    n = t.size
    while i < n - 1:
        j = i + 1
        while j < n and t[j] - t[i] < min_dt:
            j += 1
        if j >= n:
            break
        dt = t[j] - t[i]
        dpsi = yaw[j] - yaw[i]
        mid = yaw[i] + 0.5 * dpsi
        dx, dy = x[j] - x[i], y[j] - y[i]
        fwd = math.cos(mid) * dx + math.sin(mid) * dy
        lat = -math.sin(mid) * dx + math.cos(mid) * dy
        half = 0.5 * dpsi
        sinc = math.sin(half) / half if abs(half) > 1e-9 else 1.0
        scale = dt * sinc
        out.append((0.5 * (t[i] + t[j]), fwd / scale, lat / scale, dpsi / dt))
        i = j
    return np.array(out, dtype=float).reshape(-1, 4)


def command_at(t, cmd_t, cmd_values):
    """Return the command in force at the times ``t`` (the last one at or before each time)."""
    cmd_t = np.asarray(cmd_t, dtype=float)
    index = np.searchsorted(cmd_t, np.asarray(t, dtype=float), side='right') - 1
    return np.asarray(cmd_values, dtype=float)[np.clip(index, 0, max(cmd_t.size - 1, 0))]


def constant_segments(t, v, w, tol_v=0.02, tol_w=0.05, min_duration=2.0):
    """Return (t_start, t_end, v, w) of the runs of nearly constant, non-zero command.

    A run continues while each command stays within ``tol_v`` / ``tol_w`` of the run's first
    one; runs shorter than ``min_duration`` s and standstill runs are dropped. Used on recorded
    bags, where the segments are not known.
    """
    t, v, w = (np.asarray(a, dtype=float) for a in (t, v, w))
    runs = []
    start = 0
    for i in range(1, t.size + 1):
        if i < t.size and abs(v[i] - v[start]) <= tol_v and abs(w[i] - w[start]) <= tol_w:
            continue
        end = i - 1
        if t[end] - t[start] >= min_duration and (abs(v[start]) > tol_v or abs(w[start]) > tol_w):
            runs.append(
                (
                    float(t[start]),
                    float(t[end]),
                    float(np.mean(v[start : end + 1])),
                    float(np.mean(w[start : end + 1])),
                )
            )
        start = i
    return runs


def estimate_segment(segment, pose, cmd, settle=1.0, min_dt=0.1):
    """Fill one :class:`Segment` from the pose and command tracks.

    Args:
        segment: The segment; its steady part starts ``settle`` s after ``t_start``.
        pose: (t, x, y, yaw) arrays.
        cmd: (t, v, w) arrays of the command the base received.
        settle: Seconds skipped at the start of the segment (acceleration, smoother ramp).
        min_dt: Minimum spacing of the pose pairs (s).

    """
    pt, px, py, pyaw = (np.asarray(a, dtype=float) for a in pose)
    lo, hi = segment.t_start + settle, segment.t_end
    inside = (pt >= lo) & (pt <= hi)
    if np.count_nonzero(inside) < 3:
        segment.note = 'too few poses in the steady part'
        return segment
    t, x, y = pt[inside], px[inside], py[inside]
    yaw = unwrap_yaw(pyaw)[inside]
    span = t[-1] - t[0]
    if span <= 0.0:
        segment.note = 'no time span'
        return segment
    # The command in force from each pose to the next, weighted by that interval: the same
    # span as the yaw change below.
    weights = np.diff(t)
    segment.w_cmd = float(np.sum(command_at(t[:-1], cmd[0], cmd[2]) * weights) / span)
    segment.v_cmd = float(np.sum(command_at(t[:-1], cmd[0], cmd[1]) * weights) / span)
    segment.w_real = float((yaw[-1] - yaw[0]) / span)
    twists = body_twists(t, x, y, yaw, min_dt)
    segment.pairs = len(twists)
    if len(twists):
        segment.vx_real = float(np.mean(twists[:, 1]))
        if abs(segment.v_cmd) > 0.02:
            segment.eta_v = segment.vx_real / segment.v_cmd
    if abs(segment.w_cmd) >= MIN_YAW_RATE:
        segment.eta_psi = segment.w_real / segment.w_cmd
    if len(twists) and np.sum(twists[:, 3] ** 2) / len(twists) >= MIN_YAW_RATE**2:
        segment.x_icr = float(-np.sum(twists[:, 2] * twists[:, 3]) / np.sum(twists[:, 3] ** 2))
    return segment


def _spread(values):
    values = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    if values.size < 2:
        return 0.0 if values.size else math.nan
    return float(np.std(values, ddof=1))


def estimate(segments, pose, cmd, settle=1.0, min_dt=0.1):
    r"""Estimate the slip over all segments (:func:`estimate_segment` each).

    The pooled :math:`\eta_\psi` and :math:`\eta_v` are the segment values weighted by the
    steady duration; the pooled :math:`x_{ICR}` is the least-squares fit over the pose pairs of
    every turning segment. The spreads are the sample standard deviations over the segments.
    """
    result = SlipEstimate()
    pairs = []
    weights_eta, values_eta, weights_v, values_v = [], [], [], []
    for segment in segments:
        estimate_segment(segment, pose, cmd, settle, min_dt)
        result.segments.append(segment)
        duration = max(segment.t_end - segment.t_start - settle, 0.0)
        if math.isfinite(segment.eta_psi):
            weights_eta.append(duration)
            values_eta.append(segment.eta_psi)
        if math.isfinite(segment.eta_v):
            weights_v.append(duration)
            values_v.append(segment.eta_v)
        if math.isfinite(segment.x_icr):
            pt = np.asarray(pose[0], dtype=float)
            inside = (pt >= segment.t_start + settle) & (pt <= segment.t_end)
            yaw = unwrap_yaw(pose[3])[inside]
            pairs.append(
                body_twists(
                    pt[inside],
                    np.asarray(pose[1], dtype=float)[inside],
                    np.asarray(pose[2], dtype=float)[inside],
                    yaw,
                    min_dt,
                )
            )
    if values_eta and sum(weights_eta) > 0.0:
        result.eta_psi = float(np.average(values_eta, weights=weights_eta))
        result.eta_psi_spread = _spread(values_eta)
    if values_v and sum(weights_v) > 0.0:
        result.eta_v = float(np.average(values_v, weights=weights_v))
    if pairs:
        stacked = np.concatenate(pairs)
        result.x_icr = float(-np.sum(stacked[:, 2] * stacked[:, 3]) / np.sum(stacked[:, 3] ** 2))
        result.x_icr_spread = _spread([s.x_icr for s in result.segments])
    return result


def yaml_lines(result, control_point_offset=None):
    """Return the parameter lines to paste, for the controller and for the curve follower."""
    eta = result.eta_psi
    x_icr = result.x_icr
    lines = []
    if math.isfinite(eta):
        eta = min(max(eta, 1e-3), 1.0)
    if math.isfinite(eta) or math.isfinite(x_icr):
        lines.append('# controller_server.ros__parameters.FollowPath and the curve follower:')
    if math.isfinite(eta):
        lines.append(f'skid_yaw_rate_efficiency: {eta:.3f}')
    if math.isfinite(x_icr):
        lines.append(f'skid_icr_offset: {x_icr:.3f}')
    if (
        control_point_offset is not None
        and math.isfinite(x_icr)
        and abs(x_icr) >= 0.5 * control_point_offset
    ):
        lines.append(
            f'# |skid_icr_offset| must stay below control_point_offset / 2 '
            f'({0.5 * control_point_offset:.3f}); the controller rejects this value'
        )
    return lines
