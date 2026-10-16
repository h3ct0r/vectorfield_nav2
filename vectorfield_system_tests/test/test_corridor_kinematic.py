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

"""Corridor launch test on vectorfield_sim for one motion model that contours obstacles.

The model (differential, skid-steer or omnidirectional) comes from ``VF_TEST_MOTION_MODEL``.
kinematic_sim_demo on ``corridor_door`` (a 2 m corridor with a 0.8 m door at x = 5 m, narrower
than twice the contour distance), contour on, the global costmap's obstacle layer off, starting
at (3.5, 0). In order:

1. through the door: zero contour activations and a true footprint clearance of at least 0.05 m;
2. the same back through the door after a 0.05 m / 2° map -> odom step (a localization jump,
   undone afterwards);
3. a goal 0.4 m in front of the corridor's end wall: no contour activation, goal reached;
4. an unmapped wall blocking the corridor with the contour on (differential only): neither side
   probe gets past it (a dead end), so no contour episode starts there; the robot stops at least
   stop_margin − res short of it, the controller raises NoValidControl (bt_navigator's FollowPath
   fails) and Nav2 replans;
5. the same wall with obstacle_mode switched to none at run time: the robot stops at least
   stop_margin − res short of it, the controller raises NoValidControl (bt_navigator's FollowPath
   fails) and Nav2 replans.
"""

import math
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.harness import NO_VALID_CONTROL, NavHarness

from vectorfield_system_tests import checks, geometry, launch

MOTION_MODEL = launch.motion_model()

#: The start pose; the odom origin, about which the map -> odom step rotates, is 1.5 m from the
#: door.
START = (3.5, 0.0, 0.0)
#: The localization jump: map -> odom gains (x, y, yaw).
STEP = (0.05, 0.0, math.radians(2.0))
#: Required true footprint clearance in the corridor (m).
CORRIDOR_CLEARANCE = 0.05
#: An unmapped wall across the corridor (and into its side walls), 2.1 m east of the goal near
#: the end wall.
BLOCK = [1.5, 0.0, 0.2, 2.4, 0.0]
#: Simulated seconds allowed for the stop, NoValidControl, the FollowPath failure and a new plan.
BLOCKED_TIMEOUT = 45.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start kinematic_sim_demo on corridor_door without the global obstacle layer."""
    return launch.demo(
        'kinematic_sim_demo.launch.py',
        MOTION_MODEL,
        'corridor_door',
        [launch.overlay('global_obstacle_layer_off')],
        START,
    )


class TestCorridor(unittest.TestCase):
    """The door before and after a localization jump, a goal near a wall, a blocked corridor."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2."""
        cls.nav = NavHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def _wall_clearance(self, episode):
        return episode.min_distance(self.nav.map.distance) - geometry.ROBOT_RADIUS

    def _pass_cleanly(self, goal, label):
        episode = self.nav.navigate(*goal, timeout=checks.GOAL_TIMEOUT)
        self.assertEqual(
            episode.status, GoalStatus.STATUS_SUCCEEDED, f'{label}: {episode.status_name}'
        )
        self.assertEqual(episode.contour_activations(), 0, f'{label}: the contour engaged')
        failures = episode.no_valid_control()
        self.assertEqual(failures, [], f'{label}: {checks.describe(failures)}')
        clearance = self._wall_clearance(episode)
        self.assertGreaterEqual(clearance, CORRIDOR_CLEARANCE, f'{label}: wall clearance')
        return episode

    def test_1_door(self):
        """Pass the door without contouring."""
        self._pass_cleanly((7.5, 0.0, 0.0), 'through the door')

    def test_2_door_after_map_to_odom_step(self):
        """Pass the door back after a map -> odom step."""
        self.nav.step_map_to_odom(*STEP)
        try:
            self.nav.sleep(1.0)
            self._pass_cleanly((START[0], START[1], math.pi), 'back after the step')
        finally:
            self.nav.step_map_to_odom(*(-value for value in STEP))
            self.nav.sleep(1.0)

    def test_3_goal_near_wall(self):
        """Reach a goal 0.4 m from a wall without contouring."""
        # The end wall's face is at x = -1.0; the goal is 0.4 m from it, facing it.
        goal = (-0.6, 0.0, math.pi)
        self._pass_cleanly(goal, 'goal near the wall')
        final = self.nav.pose()
        miss = math.hypot(final.x - goal[0], final.y - goal[1])
        self.assertLessEqual(miss, checks.XY_GOAL_TOLERANCE + checks.STOP_SLACK)

    @unittest.skipUnless(
        MOTION_MODEL == 'differential', 'the dead end is the same for every model; run once'
    )
    def test_4_blocked_corridor_with_contour(self):
        """Blocked corridor with the contour on: stop, NoValidControl, replan."""
        self._check_blocked(self._drive_into_block())

    def test_5_blocked_corridor(self):
        """Blocked corridor with obstacle_mode none: stop, NoValidControl, replan."""
        self.nav.set_parameter('controller_server', 'FollowPath.obstacle_mode', 'none')
        self._check_blocked(self._drive_into_block())

    def _drive_into_block(self):
        """Drive towards START through BLOCK and return what happened.

        The wait ends at a new plan after a FollowPath failure after NoValidControl, or after
        BLOCKED_TIMEOUT.
        """
        nav = self.nav
        nav.set_boxes([BLOCK])
        nav.sleep(1.0)
        goal = nav.send_goal(START[0], START[1], 0.0)

        def stopped():
            return next(
                (s.t for s in nav.field_samples_since(goal.start) if s.status & NO_VALID_CONTROL),
                None,
            )

        def failed():
            stop = stopped()
            return None if stop is None else next(iter(nav.follow_path_failures_since(stop)), None)

        def replanned():
            failure = failed()
            return failure is not None and bool(nav.plans_since(failure + 1e-6))

        nav.wait_until(replanned, BLOCKED_TIMEOUT)
        result = {'stopped': stopped(), 'failed': failed(), 'replanned': replanned()}
        nav.cancel(goal)
        result['episode'] = nav.episode(goal.start, None)
        nav.set_boxes([])
        return result

    def _check_blocked(self, result):
        episode = result['episode']
        final = self.nav.pose()
        context = (
            f'robot at ({final.x:.2f}, {final.y:.2f}, {final.yaw:.2f}), '
            f'{episode.contour_activations()} contour activations; last field_state: '
            f'{checks.describe(episode.samples, -3)}'
        )
        clearance = (
            episode.min_distance(lambda x, y: geometry.box_distance(x, y, BLOCK))
            - geometry.ROBOT_RADIUS
        )
        self.assertGreaterEqual(clearance, geometry.STOP_MARGIN - geometry.RESOLUTION)
        self.assertIsNotNone(result['stopped'], f'no NoValidControl status; {context}')
        self.assertIsNotNone(result['failed'], f'no FollowPath failure after it; {context}')
        self.assertTrue(
            result['replanned'], f'no new plan after the FollowPath failure; {context}'
        )
        # The stop happened at the wall, not anywhere before it.
        self.assertLess(clearance, 3.0 * geometry.STOP_MARGIN, f'stopped early; {context}')
