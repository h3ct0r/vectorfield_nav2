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

"""NavigateThroughPoses launch test on vectorfield_sim (differential).

kinematic_sim_demo on ``open_10x10`` (NavFn, ``allow_reversing: false``). From (0, 0) facing +x,
one NavigateThroughPoses goal through (2, 0) to (0.27, 1): the second leg leaves the waypoint at
150° from the first, so the plan has a corner sharper than 90°. That is a forward corner, not a
cusp (math_spec §2.8): the goal succeeds with no NoValidControl, the robot passes the waypoint,
never backs up, and stops within the goal tolerance.
"""

import math
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.harness import NavHarness

from vectorfield_system_tests import checks, launch

#: The waypoint, and the goal 2 m from it at 150° from the first leg (the field-testing route).
WAYPOINT = (2.0, 0.0, 0.0)
GOAL = (0.27, 1.0, 2.62)
#: How close the robot passes the waypoint (m): the stock trees drop a waypoint within 0.7 m.
WAYPOINT_REACH = 0.75
#: Largest path length travelled against the heading (m).
MAX_BACKWARD = 0.05


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
    """Start kinematic_sim_demo on open_10x10."""
    return launch.demo('kinematic_sim_demo.launch.py', 'differential', 'open_10x10')


class TestThroughPoses(unittest.TestCase):
    """A sharp forward corner at a NavigateThroughPoses waypoint."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2."""
        cls.nav = NavHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def test_sharp_corner(self):
        """Drive through a 150° waypoint corner."""
        nav = self.nav
        episode = nav.navigate_through([WAYPOINT, GOAL], timeout=checks.GOAL_TIMEOUT)
        context = (
            f'{episode.status_name}; last field_state: {checks.describe(episode.samples, -3)}'
        )
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, context)
        failures = episode.no_valid_control()
        self.assertEqual(failures, [], checks.describe(failures))
        reach = min(math.hypot(p.x - WAYPOINT[0], p.y - WAYPOINT[1]) for p in episode.poses)
        self.assertLessEqual(reach, WAYPOINT_REACH, 'the robot cut the waypoint')
        self.assertLessEqual(backward_travel(episode.poses), MAX_BACKWARD, 'the robot backed up')
        final = nav.settled_pose(checks.SETTLE_TIME)
        miss = math.hypot(final.x - GOAL[0], final.y - GOAL[1])
        self.assertLessEqual(miss, checks.XY_GOAL_TOLERANCE + checks.STOP_SLACK)
