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
Reeds-Shepp primitives and ``allow_reversing``. Starting at (3.5, 0) facing +x:

1. a goal 1.5 m behind the robot, facing the same way: backing up is the cheapest plan (a car
   cannot turn in the corridor without a three-point turn, and Reeds-Shepp charges reversing
   only twice); the goal succeeds, the robot backs up most of the way and never turns around;
2. back to the start, forward: the goal succeeds with no NoValidControl.
"""

import math
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.harness import NavHarness

from vectorfield_system_tests import checks, launch

MOTION_MODEL = launch.motion_model()

#: The start pose, in the corridor west of the door.
START = (3.5, 0.0, 0.0)
#: The goal behind the robot, facing the same way.
BEHIND = (2.0, 0.0, 0.0)
#: Backward travel required for the goal behind (m): most of the 1.5 m.
MIN_BACKWARD = 1.0
#: Largest yaw excursion while backing up (rad): no turn-around.
MAX_YAW = 0.5


def backward_travel(poses):
    """Return the path length travelled against the heading (m)."""
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        along = (b.x - a.x) * math.cos(a.yaw) + (b.y - a.y) * math.sin(a.yaw)
        total += max(0.0, -along)
    return total


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start kinematic_sim_demo on corridor_door with the reversing overlay."""
    return launch.demo(
        'kinematic_sim_demo.launch.py', MOTION_MODEL, 'corridor_door', ['reversing'], START
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

    def test_2_forward_again(self):
        """Drive forward back to the start."""
        self._reach(START, 'forward again')
