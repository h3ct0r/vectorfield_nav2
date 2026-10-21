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

"""Unit tests of the field-test kit's metrics on synthetic data.

``vectorfield_bringup.field_metrics``, ``.slip``, the pure parts of ``.field_report`` (episodes,
metrics, checks, Markdown) and ``.bag.TimeMap``; no bag and no ROS graph.
"""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from vectorfield_bringup import bag, field_report, slip
from vectorfield_bringup import field_metrics as fm

# field_metrics ---------------------------------------------------------------------------------


def test_polyline_distance():
    """Distances to the segments and to a single point."""
    line = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])
    points = np.array([[1.0, 0.3], [2.5, 1.0], [-1.0, 0.0], [3.0, 3.0]])
    np.testing.assert_allclose(
        fm.polyline_distance(points, line), [0.3, 0.5, 1.0, math.sqrt(2.0)], atol=1e-12
    )
    np.testing.assert_allclose(fm.polyline_distance([[3.0, 4.0]], [[0.0, 0.0]]), [5.0])


def test_second_differences_of_a_ramp_and_a_parabola():
    """Jerk of known signals; bursts merge, gaps split the series."""
    # 20 Hz updates; v = 0.5 t^2 has jerk 1 everywhere, w = t has none.
    t = np.arange(0.0, 2.0, 0.05)
    samples = [(ti, ti, 0.5 * ti**2, ti) for ti in t]
    jerk, period = fm.second_differences(samples)
    assert period == pytest.approx(0.05)
    np.testing.assert_allclose(jerk[:, 0], 1.0, atol=1e-9)
    np.testing.assert_allclose(jerk[:, 1], 0.0, atol=1e-9)
    assert fm.rms(jerk[:, 0]) == pytest.approx(1.0)
    # A back-to-back duplicate counts as one update; a gap splits the series.
    burst = samples[:10] + [(samples[9][0], samples[9][1] + 0.005, 9.0, 9.0)] + samples[10:]
    assert len(fm.command_updates(burst)) == len(samples)
    gap = samples[:10] + [(ti + 1.0, ti + 1.0, 0.5 * ti**2, ti) for ti in t[10:]]
    jerk_gap, _ = fm.second_differences(gap)
    assert len(jerk_gap) == len(t) - 4
    assert math.isnan(fm.second_differences(samples[:2])[1])


def test_tracking_error_follows_the_launch_test_rule():
    """The max |D| after convergence, as in the launch tests."""
    t = np.arange(10.0)
    phase = [0, 0, 1, 1, 1, 1, 2, 2, 3, 4]
    distance = [0.5, 0.4, 0.3, 0.04, 0.08, -0.06, 0.02, 0.01, 0.9, 0.9]
    # Converged at t=3 (|D| 0.04), then the largest tracking |D| is 0.08; rotate phases ignored.
    assert fm.tracking_error(t, phase, distance) == pytest.approx(0.08)
    assert fm.tracking_error(t, phase, distance, after=5.5) == pytest.approx(0.02)
    assert fm.tracking_error(t, phase, np.full(10, 0.2)) == math.inf
    assert math.isnan(fm.tracking_error(t, np.zeros(10), distance))


def test_phases_flags_and_edges():
    """Phase runs, time per phase and status-flag counts."""
    t = np.arange(8.0)
    phase = [0, 0, 1, 1, 0, 1, 2, 3]
    segments = fm.phase_segments(t, phase)
    assert segments[0] == (0, 0.0, 2.0)
    assert [s[0] for s in segments] == [0, 1, 0, 1, 2, 3]
    durations = fm.phase_durations(segments)
    assert durations['ROTATE_TO_FIELD'] == (3.0, 2)
    assert fm.rising_edges([1, 1, 0, 1, 0, 0, 1]) == 3
    status = [0, 256, 256, 0, 256 | 16, 2, 0, 0]
    contour = [0, 0, 1, 1, 0, -1, -1, 0]
    counts = fm.flag_counts(status, contour)
    assert counts['BLOCKED'] == (3, 2)
    assert counts['CURVATURE_SATURATED'] == (1, 1)
    assert counts['ASSUMPTION3_CLAMPED'] == (1, 1)
    assert counts['CONTOUR'] == (4, 2)
    assert counts['HAIRPIN'] == (0, 0)


def test_plan_corners_and_attribution():
    """Corners of 90 and 150 degrees, none on a staircase, entries per corner."""
    # An L with a 90 degree left turn at (3, 0) and a 150 degree turn at (3, 3), on a 5 cm grid.
    leg1 = np.column_stack((np.arange(0.0, 3.0, 0.05), np.zeros(60)))
    leg2 = np.column_stack((np.full(60, 3.0), np.arange(0.0, 3.0, 0.05)))
    back = np.array([math.cos(math.radians(240.0)), math.sin(math.radians(240.0))])
    leg3 = np.array([3.0, 3.0]) + np.arange(0.0, 2.0, 0.05)[:, None] * back
    corners = fm.plan_corners(np.vstack((leg1, leg2, leg3)))
    assert len(corners) == 2
    assert corners[0][:2] == pytest.approx((3.0, 0.0), abs=0.15)
    assert math.degrees(corners[0][2]) == pytest.approx(90.0, abs=5.0)
    assert math.degrees(corners[1][2]) == pytest.approx(150.0, abs=5.0)
    # A staircase (grid planner) is not a corner.
    stairs = np.array([[0.05 * (i // 2 + i % 2), 0.05 * (i // 2)] for i in range(80)])
    assert fm.plan_corners(stairs) == []
    start, per_corner, other = fm.attribute_entries(
        [(0.0, 0.0, 0.0), (3.1, 0.1, 3.0), (2.9, -0.2, 3.1), (1.5, 0.0, 1.5)], corners
    )
    assert (start, per_corner, other) == (1, [2, 0], 1)


def test_stop_latency_and_recovery():
    """Time to standstill, |D| peak and recovery after an event."""
    t = np.arange(0.0, 3.0, 0.05)
    v = np.where(t < 1.2, 0.3, 0.0)
    w = np.zeros_like(t)
    assert fm.stop_latency(t, v, w, 1.0) == pytest.approx(0.2, abs=0.051)
    assert math.isnan(fm.stop_latency(t, np.full_like(t, 0.3), w, 1.0))
    d = np.where((t > 1.0) & (t < 1.6), 0.12, 0.01)
    peak, back = fm.recovery_after(t, d, 1.0)
    assert peak == pytest.approx(0.12)
    assert back == pytest.approx(0.6, abs=0.051)
    assert fm.recovery_after(t, np.full_like(t, 0.01), 1.0) == (pytest.approx(0.01), 0.0)


def test_localisation_jumps():
    """A shift and a turn of map -> odom are jumps; the robot shift is measured."""
    t = np.array([0.0, 1.0, 2.0, 3.0])
    # A 10 cm correction at t=2 and a 3 degree one at t=3 (the robot 2 m from the odom origin).
    map_odom = (t, np.array([0.0, 0.0, 0.1, 0.1]), np.zeros(4), np.array([0, 0, 0, 0.0524]))
    robot = (t, np.full(4, 2.0), np.zeros(4), np.zeros(4))
    jumps = fm.localisation_jumps(map_odom, robot)
    assert [j[0] for j in jumps] == [2.0, 3.0]
    assert jumps[0][1] == pytest.approx(0.1)
    assert jumps[1][1] == pytest.approx(2.0 * 0.0524, rel=0.01)


def test_occupied_cells():
    """Only lethal cells count with the LETHAL threshold."""
    data = np.zeros((10, 10), dtype=int)
    data[5, 7] = 100
    data[2, 2] = 99  # inscribed, not lethal
    grid = SimpleNamespace(
        data=data.ravel().tolist(),
        info=SimpleNamespace(
            width=10,
            height=10,
            resolution=0.1,
            origin=SimpleNamespace(position=SimpleNamespace(x=-0.5, y=-0.5)),
        ),
    )
    lethal = fm.OccupiedCells(grid, threshold=fm.LETHAL)
    assert lethal.count == 1
    # Cell (row 5, column 7) spans x in [0.2, 0.3], y in [0.0, 0.1].
    assert lethal.distance(0.0, 0.05) == pytest.approx(0.2)
    assert fm.OccupiedCells(grid).count == 2


# slip --------------------------------------------------------------------------------------------


def simulate(segments, eta=0.75, x_icr=0.05, eta_v=1.0, dt=0.02, pause=1.0):
    """Integrate the vectorfield_sim slip model; return pose and command arrays and segments."""
    t, x, y, yaw = 0.0, 0.0, 0.0, 0.3
    poses, commands, spans = [(t, x, y, yaw)], [], []
    for v, w, duration in segments:
        for command, length in (((v, w), duration), ((0.0, 0.0), pause)):
            start = t
            for _ in range(int(round(length / dt))):
                commands.append((t, *command))
                wz = eta * command[1]
                vx, vy = eta_v * command[0], -x_icr * wz
                # Exact arc of a constant body twist.
                half = 0.5 * wz * dt
                sinc = math.sin(half) / half if abs(half) > 1e-12 else 1.0
                mid = yaw + half
                x += dt * sinc * (math.cos(mid) * vx - math.sin(mid) * vy)
                y += dt * sinc * (math.sin(mid) * vx + math.cos(mid) * vy)
                yaw += wz * dt
                t += dt
                poses.append((t, x, y, yaw))
            if command != (0.0, 0.0):
                spans.append(slip.Segment(start, t, *command))
    poses = np.array(poses).T
    poses[3] = fm.wrap_angle(poses[3])
    return poses, np.array(commands).T, spans


def test_body_twists_are_exact_on_arcs():
    """Pose pairs on a constant-twist arc give the twist exactly."""
    poses, _, _ = simulate([(0.2, 0.6, 3.0)], pause=0.0)
    twists = slip.body_twists(*poses, min_dt=0.2)
    np.testing.assert_allclose(twists[:, 1], 0.2, atol=1e-9)
    np.testing.assert_allclose(twists[:, 2], -0.05 * 0.45, atol=1e-9)
    np.testing.assert_allclose(twists[:, 3], 0.45, atol=1e-9)


def test_slip_estimate_recovers_the_simulated_slip():
    """The vectorfield_sim slip, from known and from detected segments."""
    plan = [(0.0, 0.6, 4.0), (0.0, -0.6, 4.0), (0.2, 0.6, 4.0), (0.2, -0.6, 4.0)]
    poses, commands, spans = simulate(plan, eta_v=0.9)
    result = slip.estimate(spans, poses, commands, settle=0.5)
    assert result.eta_psi == pytest.approx(0.75, abs=1e-6)
    assert result.x_icr == pytest.approx(0.05, abs=1e-6)
    assert result.eta_v == pytest.approx(0.9, abs=1e-6)
    assert result.eta_psi_spread == pytest.approx(0.0, abs=1e-9)
    lines = slip.yaml_lines(result, control_point_offset=0.2)
    assert 'skid_yaw_rate_efficiency: 0.750' in lines
    assert 'skid_icr_offset: 0.050' in lines
    # The offline path: segments found from the commands alone, sparse noisy poses.
    found = slip.constant_segments(*commands, min_duration=2.0)
    assert [(round(v, 2), round(w, 2)) for _, _, v, w in found] == [p[:2] for p in plan]
    rng = np.random.default_rng(1)
    sparse = poses[:, ::5].copy()
    sparse[1:3] += rng.normal(0.0, 0.002, sparse[1:3].shape)
    noisy = slip.estimate(
        [slip.Segment(*f) for f in found], sparse, commands, settle=0.5, min_dt=0.2
    )
    assert noisy.eta_psi == pytest.approx(0.75, rel=0.03)
    assert noisy.x_icr == pytest.approx(0.05, rel=0.15)


def test_slip_yaml_warns_about_a_large_offset():
    """An eta above 1 is clamped and an ICR offset above d / 2 is flagged."""
    result = slip.SlipEstimate(eta_psi=1.2, x_icr=0.15)
    lines = slip.yaml_lines(result, control_point_offset=0.2)
    assert 'skid_yaw_rate_efficiency: 1.000' in lines
    assert any('must stay below' in line for line in lines)


# bag.TimeMap -------------------------------------------------------------------------------------


def test_time_map():
    """Receive times map to the stamp clock; storage ids from extensions."""
    receive = np.arange(1000.0, 1010.0, 0.1)
    stamp = 0.5 * (receive - 1000.0) + 20.0  # real-time factor 0.5
    time_map = bag.TimeMap(receive, stamp)
    assert time_map(1004.0) == pytest.approx(22.0, abs=1e-6)
    identity = bag.TimeMap([], [])
    assert identity(5.0) == 5.0
    assert bag.storage_id('x/y.mcap') == 'mcap'
    assert bag.storage_id('x/y.db3') == 'sqlite3'


# field_report ----------------------------------------------------------------------------------


def goal_status(goal_id, status, stamp=0.0):
    """Return one GoalStatusArray entry as the loader stores it."""
    return (goal_id, status, stamp)


def synthetic_recording():
    """Return a 3 m straight goal, then one follower lap, with a 4 cm wobble and a speed limit."""
    rec = field_report.Recording(start=0.0, end=40.0)
    t = np.arange(0.0, 40.0, 0.05)
    x = np.clip((t - 1.0) * 0.3, 0.0, 3.0)
    y = np.where(t < 10.0, 0.04 * np.sin(t), 0.0)
    rec.pose = {'t': t, 'x': x, 'y': y, 'yaw': np.zeros_like(t)}
    rec.pose_odom = rec.pose
    moving = (t > 1.0) & (t < 11.0)
    rec.odom_twist = {'t': t, 'v': np.where(moving, 0.3, 0.0), 'w': np.zeros_like(t)}
    phase = np.where(t < 1.0, 0, np.where(t < 9.0, 1, np.where(t < 11.0, 2, 4)))
    rec.field_states['/FollowPath/field_state'] = {
        't': t[t < 12.0],
        'phase': phase[t < 12.0],
        'status': np.where((t > 5.0) & (t < 5.5), 256, 0)[t < 12.0],
        'distance': y[t < 12.0],
        'obstacle_distance': np.full(np.count_nonzero(t < 12.0), np.inf),
        'contour': np.zeros(np.count_nonzero(t < 12.0), dtype=int),
        'limiter': np.ones(np.count_nonzero(t < 12.0)),
        'cp_x': x[t < 12.0],
        'cp_y': y[t < 12.0],
        'cl_x': x[t < 12.0],
        'cl_y': np.zeros(np.count_nonzero(t < 12.0)),
    }
    follow = t >= 15.0
    rec.field_states['/curve_follower/field_state'] = {
        'distance': np.where(t[follow] < 25.0, 0.08, 0.02),
        't': t[follow],
        'phase': np.ones(np.count_nonzero(follow), dtype=int),
        'status': np.zeros(np.count_nonzero(follow), dtype=int),
        'obstacle_distance': np.full(np.count_nonzero(follow), 0.8),
        'contour': np.zeros(np.count_nonzero(follow), dtype=int),
        'limiter': np.ones(np.count_nonzero(follow)),
        'cp_x': np.cos(t[follow]),
        'cp_y': np.sin(t[follow]),
        'cl_x': np.cos(t[follow]),
        'cl_y': np.sin(t[follow]),
    }
    v = np.where(moving, 0.3, 0.0)
    rec.commands['/cmd_vel_nav'] = {'t': t, 'recv': t + 100.0, 'v': v, 'vy': 0 * t, 'w': 0 * t}
    plan = np.column_stack((np.linspace(0.0, 3.0, 61), np.zeros(61)))
    rec.plans = [(0.1, 'map', plan, 0.0)]
    rec.statuses['/navigate_to_pose'] = [
        (0.2, [goal_status('a', 2, 0.05)]),
        (11.0, [goal_status('a', 4, 0.05)]),
    ]
    rec.statuses['/follow_curve'] = [
        (15.0, [goal_status('b', 2)]),
        (35.0, [goal_status('b', 4)]),
    ]
    rec.statuses['/follow_path'] = [(0.3, [goal_status('c', 2)]), (5.4, [goal_status('c', 6)])]
    rec.laps['/follow_curve'] = {'t': np.array([16.0, 25.0, 34.9]), 'lap': np.array([0, 1, 2])}
    rec.speed_limits = [(2.0, False, 0.2), (8.0, False, 0.0)]
    rec.clearance = {'t': t, 'd': np.full_like(t, 1.2)}
    return rec


def test_report_on_a_synthetic_session():
    """Episodes, metrics, checks, speed limits and the Markdown of a session."""
    rec = synthetic_recording()
    report = field_report.analyse(rec)
    nav, follow = report['episodes']
    assert (nav['kind'], nav['status'], follow['kind']) == ('nav', 'SUCCEEDED', 'follow')
    assert nav['duration'] == pytest.approx(10.95)
    assert nav['xte_max'] == pytest.approx(0.04, abs=2e-3)
    assert nav['xte_rms'] == pytest.approx(0.04 / math.sqrt(2.0), rel=0.15)
    assert nav['tracking_max'] == pytest.approx(0.04, abs=2e-3)
    assert nav['goal_miss'] == pytest.approx(0.0, abs=1e-6)
    assert nav['corners'] == []
    assert nav['rtf_start'] == 1 and nav['rtf_entries'] == 1
    assert nav['flags']['BLOCKED'][1] == 1
    assert nav['no_valid_control'] > 0
    assert nav['controller_aborts'] == 1
    assert nav['min_costmap_clearance'] == pytest.approx(1.2)
    assert nav['stop_latency'] == pytest.approx(0.0, abs=0.06)
    assert nav['jerk']['/cmd_vel_nav'][0] > 0.0
    assert follow['laps'] == 2
    assert follow['d_max_after_lap1'] == pytest.approx(0.02)
    assert follow['d_max'] == pytest.approx(0.08)
    checks = {name: passed for name, _, _, passed in nav['checks']}
    assert checks['goal status'] and checks['goal miss (m)']
    assert checks['NoValidControl samples'] is False
    # The 0.2 m/s limit from 2 s to 8 s is exceeded by the 0.3 m/s command.
    limit = report['speed_limits'][0]
    assert limit['limit_mps'] == pytest.approx(0.2)
    assert limit['excess'] == pytest.approx(0.1)
    assert math.isinf(report['speed_limits'][1]['limit_mps'])
    text = field_report.render_markdown(report, 'synthetic', {1: 'episode_01.png'})
    assert '## Episode 1: Nav2 goal (SUCCEEDED)' in text
    assert '## Episode 2: Curve following (SUCCEEDED)' in text
    assert '![Episode 1](episode_01.png)' in text
    assert '**FAIL**' in text


def test_goals_from_status_and_unfinished_goals():
    """Goal start, end and status; a goal without a result."""
    messages = [
        (1.0, [goal_status('a', 1, 0.9)]),
        (2.0, [goal_status('a', 2, 0.9), goal_status('b', 2)]),
        (3.0, [goal_status('a', 5, 0.9), goal_status('b', 2)]),
        (4.0, [goal_status('a', 5, 0.9), goal_status('b', 2)]),
    ]
    goals = field_report.goals_from_status(messages)
    assert goals == [('a', 0.9, 3.0, 5), ('b', 2.0, None, 2)]
    rec = field_report.Recording(start=0.0, end=10.0)
    rec.statuses['/robot1/navigate_to_pose'] = messages
    found = field_report.episodes(rec)
    assert [(e.status, e.end) for e in found] == [('CANCELED', 3.0), ('UNFINISHED', 10.0)]
    assert field_report.action_kind('/robot1/follow_curve') == 'follow'
    assert field_report.action_kind('/spin') == 'recovery'
