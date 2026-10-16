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

"""Assertions shared by several launch tests: the route and the final-command publishers."""

import math

from action_msgs.msg import GoalStatus

from vectorfield_bringup import params

#: The fixed route on open_10x10 from the start pose (0, 0, 0): map goals (x, y, yaw).
ROUTE = ((3.0, 0.0, 0.0), (3.0, 3.0, 0.5 * math.pi), (0.0, 3.0, math.pi))
#: Simulated seconds within which each goal must succeed.
GOAL_TIMEOUT = 120.0
#: xy_goal_tolerance of the stock SimpleGoalChecker and of the Ackermann PositionGoalChecker.
XY_GOAL_TOLERANCE = 0.25
#: Allowance for the motion after the goal checker fires: the smoother's deceleration, which on
#: the loopback simulator's 10 Hz clock takes a few of its steps.
STOP_SLACK = 0.05
#: Simulated seconds after the result before the final pose is read.
SETTLE_TIME = 0.5


def tracking_limit(motion_model):
    """Return the bound on max |D| after convergence (m)."""
    return 0.25 if motion_model == 'ackermann' else 0.10


def describe(samples, limit=5):
    """Return field_state samples as short text, for failure messages.

    A positive ``limit`` takes the first samples, a negative one the last.
    """
    chosen = samples[:limit] if limit > 0 else samples[limit:]
    return ', '.join(
        f't={s.t:.2f} phase={s.phase} status={s.status} D={s.distance:.3f} '
        f'contour={s.contour_direction}'
        for s in chosen
    )


def check_goal(test, nav, episode, goal, motion_model):
    """Assert that a goal on a free path succeeded cleanly (M3 exit criteria)."""
    x, y, _ = goal
    label = f'goal ({x:.2f}, {y:.2f})'
    duration = episode.end - episode.start
    test.assertEqual(
        episode.status,
        GoalStatus.STATUS_SUCCEEDED,
        f'{label}: {episode.status_name} after {duration:.1f} s',
    )
    test.assertGreater(len(episode.samples), 20, f'{label}: too few field_state messages')
    error = episode.tracking_error()
    test.assertLess(
        error, tracking_limit(motion_model), f'{label}: max |D| after convergence {error:.3f} m'
    )
    # Where the robot comes to rest (the loopback simulator's TF also lags by up to a clock tick).
    nav.sleep(SETTLE_TIME)
    final = nav.pose()
    miss = math.hypot(final.x - x, final.y - y)
    test.assertLessEqual(
        miss, XY_GOAL_TOLERANCE + STOP_SLACK, f'{label}: stopped {miss:.3f} m from the goal'
    )
    failures = episode.no_valid_control()
    test.assertEqual(failures, [], f'{label}: NoValidControl status: {describe(failures)}')
    failed = nav.follow_path_failures_since(episode.start)
    test.assertEqual(failed, [], f'{label}: FollowPath failed at {failed}')


def check_route(test, nav, motion_model):
    """Drive :data:`ROUTE` and assert every goal with :func:`check_goal`."""
    for goal in ROUTE:
        episode = nav.navigate(*goal, timeout=GOAL_TIMEOUT)
        check_goal(test, nav, episode, goal, motion_model)


def check_final_command_publishers(test, nav):
    """Assert who publishes the final velocity command.

    Jazzy and later: ``cmd_vel``; Humble: ``cmd_vel_safe`` (bringup's collision_monitor). Only
    collision_monitor and the allow-listed docking_server (Jazzy and later) and following_server
    (Lyrical and later) may publish it; nothing from this repository does.
    """
    distro = params.ros_distro()
    topic = '/cmd_vel_safe' if distro == 'humble' else '/cmd_vel'
    allowed = {'collision_monitor'}
    if distro != 'humble':
        allowed.add('docking_server')
    if distro not in ('humble', 'jazzy', 'kilted'):
        allowed.add('following_server')
    names = nav.final_command_publishers(topic, 'collision_monitor')
    test.assertIn('collision_monitor', names, f'{topic} publishers: {names}')
    test.assertLessEqual(set(names), allowed, f'{topic} publishers: {names}')
