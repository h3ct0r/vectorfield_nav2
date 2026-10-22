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

"""Reversing launch test (M7) on vectorfield_sim for one motion model.

The model (differential or Ackermann) comes from ``VF_TEST_MOTION_MODEL``. kinematic_sim_demo on
``corridor_door`` (a 2 m corridor) with the ``reversing`` scenario overlay: Smac Hybrid-A* with
Reeds-Shepp primitives, ``allow_reversing`` and ``max_reverse_vel`` 0.2 m/s; the global costmap's
obstacle layer is off, so plans run through unmapped boxes, and NavigateToPose replans only when
the plan becomes invalid (a 1 Hz replan from the pose a contour has displaced can graze the box,
which stops the robot for a recovery instead). Starting at (3.5, 0) facing +x:

1. a goal 1.5 m behind the robot, facing the same way: backing up is the cheapest plan (a car
   cannot turn in the corridor without a three-point turn, and Reeds-Shepp charges reversing
   only twice); the goal succeeds, the robot backs up most of the way, never turns around and
   never backs up faster than max_reverse_vel;
2. back to the start, forward: the goal succeeds with no NoValidControl, faster than
   max_reverse_vel;
3. (differential) a goal 2.7 m behind with an unmapped 0.3 m box 0.1 m beside the reverse leg:
   the contour engages while backing up (0.3.0), passes the box on the free side with the base
   at least lambda - d - 0.05 from it, rejoins the path and the goal succeeds.
"""

import math
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.harness import TRACKING_PHASES, NavHarness

from vectorfield_system_tests import checks, geometry, launch

MOTION_MODEL = launch.motion_model()

#: The start pose, in the corridor west of the door.
START = (3.5, 0.0, 0.0)
#: The goal behind the robot, facing the same way.
BEHIND = (2.0, 0.0, 0.0)
#: Backward travel required for the goal behind (m): most of the 1.5 m.
MIN_BACKWARD = 1.0
#: Largest yaw excursion while backing up (rad): no turn-around.
MAX_YAW = 0.5
#: max_reverse_vel of the reversing scenario overlay (m/s).
MAX_REVERSE_VEL = 0.2
#: Ground-truth speed slack over a SPEED_WINDOW s finite difference (m/s).
SPEED_SLACK = 0.03
SPEED_WINDOW = 0.5
#: The goal of case 3, 2.7 m behind the start, and its box: 0.3 m, 0.1 m left of the leg.
BOX_GOAL = (0.8, 0.0, 0.0)
BOX = [2.15, 0.25, 0.3, 0.3, 0.0]


def backward_travel(poses):
    """Return the path length travelled against the heading (m)."""
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        along = (b.x - a.x) * math.cos(a.yaw) + (b.y - a.y) * math.sin(a.yaw)
        total += max(0.0, -along)
    return total


def along_speeds(poses, window=SPEED_WINDOW):
    """Return the speeds along the heading (m/s, negative backward) over ``window`` s steps."""
    speeds = []
    j = 0
    for i, a in enumerate(poses):
        j = max(j, i + 1)
        while j < len(poses) and poses[j].t - a.t < window:
            j += 1
        if j == len(poses):
            break
        b = poses[j]
        along = (b.x - a.x) * math.cos(a.yaw) + (b.y - a.y) * math.sin(a.yaw)
        speeds.append(along / (b.t - a.t))
    return speeds


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start kinematic_sim_demo on corridor_door with the reversing overlay."""
    return launch.demo(
        'kinematic_sim_demo.launch.py',
        MOTION_MODEL,
        'corridor_door',
        [
            'reversing',
            launch.overlay('global_obstacle_layer_off'),
            launch.overlay('replan_only_if_path_invalid'),
        ],
        START,
    )


class TestReversing(unittest.TestCase):
    """A goal behind the robot, reached by backing up, then forward again."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2."""
        cls.nav = NavHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def _reach(self, goal, label):
        episode = self.nav.navigate(*goal, timeout=checks.GOAL_TIMEOUT)
        context = f'{label}: {episode.status_name}; last field_state: '
        context += checks.describe(episode.samples, -3)
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, context)
        failures = episode.no_valid_control()
        self.assertEqual(failures, [], f'{label}: {checks.describe(failures)}')
        final = self.nav.settled_pose(checks.SETTLE_TIME)
        miss = math.hypot(final.x - goal[0], final.y - goal[1])
        self.assertLessEqual(miss, checks.XY_GOAL_TOLERANCE + checks.STOP_SLACK, label)
        return episode

    def test_1_goal_behind(self):
        """Back up to a goal behind the robot."""
        episode = self._reach(BEHIND, 'goal behind')
        backward = backward_travel(episode.poses)
        self.assertGreaterEqual(backward, MIN_BACKWARD, 'the robot did not back up')
        yaw = max(abs(math.remainder(p.yaw, 2.0 * math.pi)) for p in episode.poses)
        self.assertLess(yaw, MAX_YAW, 'the robot turned around')
        fastest = -min(along_speeds(episode.poses), default=0.0)
        print(f'[goal behind] fastest reverse {fastest:.3f} m/s', flush=True)
        self.assertLessEqual(fastest, MAX_REVERSE_VEL + SPEED_SLACK, 'max_reverse_vel exceeded')
        self.assertGreater(fastest, 0.5 * MAX_REVERSE_VEL, 'backed up too slowly')

    def test_2_forward_again(self):
        """Drive forward back to the start."""
        episode = self._reach(START, 'forward again')
        fastest = max(along_speeds(episode.poses), default=0.0)
        self.assertGreater(fastest, MAX_REVERSE_VEL + SPEED_SLACK, 'the forward leg was limited')

    @unittest.skipIf(
        MOTION_MODEL == 'ackermann',
        'the 2 m corridor is narrower than the car circulation 2 (lambda + w_lat) needs',
    )
    def test_3_box_on_the_reverse_leg(self):
        """Back up past an unmapped box beside the reverse leg: contoured in reverse."""
        nav = self.nav
        nav.set_boxes([BOX])
        nav.sleep(1.0)
        try:
            episode = self._reach(BOX_GOAL, 'box on the reverse leg')
        finally:
            nav.set_boxes([])
            nav.clear_costmaps()
        self.assertGreaterEqual(backward_travel(episode.poses), 2.0, 'the robot did not back up')
        self.assertGreaterEqual(episode.contour_activations(), 1, 'the contour never engaged')
        lam = geometry.auto_contour_distance(MOTION_MODEL)
        bound = lam - geometry.evaluation_offset(MOTION_MODEL) - 0.05
        distance = episode.min_distance(lambda x, y: geometry.box_distance(x, y, BOX))
        self.assertGreaterEqual(distance, bound, f'closest approach to the box (λ = {lam:.3f} m)')
        beside = [p for p in episode.poses if abs(p.x - BOX[0]) < 0.05]
        self.assertTrue(beside and max(p.y for p in beside) < 0.0, 'not on the free side')
        last_contour = max(s.t for s in episode.samples if s.contour_direction != 0)
        after = [s for s in episode.samples if s.t > last_contour and s.phase in TRACKING_PHASES]
        closest = min((abs(s.distance) for s in after), default=math.inf)
        self.assertLess(closest, checks.tracking_limit(MOTION_MODEL), 'no rejoin')
        fastest = -min(along_speeds(episode.poses), default=0.0)
        print(
            f'[reverse box] closest {distance:.3f} m (bound {bound:.3f}), '
            f'fastest reverse {fastest:.3f} m/s',
            flush=True,
        )
        self.assertLessEqual(fastest, MAX_REVERSE_VEL + SPEED_SLACK, 'max_reverse_vel exceeded')
