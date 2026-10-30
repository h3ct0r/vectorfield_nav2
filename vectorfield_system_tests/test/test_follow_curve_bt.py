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

"""The FollowCurve behaviour-tree node end to end, for ``VF_TEST_MOTION_MODEL``.

curve_follower_demo on ``open_10x10`` with the ``follow_curve_bt`` overlay, so bt_navigator runs
bringup's ``behavior_trees/follow_curve.xml`` for NavigateToPose. The tree approaches (2, 0)
heading +y with ComputePathToPose and FollowPath, then FollowCurve follows the 2 m × 1 m ellipse
that starts there for 1 lap, through the real follower:

1. bt_runner ticks the tree from the start pose: exit status 0 (SUCCESS), and the one follower
   goal it sent SUCCEEDED (with laps = 1, after a lap) with feedback on the way;
2. a NavigateToPose goal to the same pose runs the tree in bt_navigator: SUCCEEDED, and so did
   the one new follower goal;
3. bt_runner creates ``follow_curve_patrol.xml`` without ticking it (``dry_run``): its nodes,
   ports and servers all exist.
"""

import math
import os
import subprocess
import threading
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from vectorfield_system_tests.follower import FollowerHarness
from vectorfield_system_tests.shutdown import TestCleanShutdown  # noqa: F401

from vectorfield_msgs.action import FollowCurve
from vectorfield_system_tests import launch

MOTION_MODEL = launch.motion_model()

APPROACH_POSE = (2.0, 0.0, 0.5 * math.pi)
# Wall seconds for a whole tree: the approach, one 9.7 m lap and the start-up of bt_runner.
TREE_TIMEOUT = 300.0


def _runner(tree, *parameters):
    """Run bt_runner on a bringup tree; return the CompletedProcess (stdout and stderr merged)."""
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    executable = os.path.join(
        get_package_prefix('vectorfield_behavior_tree'),
        'lib',
        'vectorfield_behavior_tree',
        'bt_runner',
    )
    tree_file = os.path.join(
        get_package_share_directory('vectorfield_bringup'), 'behavior_trees', tree
    )
    arguments = [executable, '--ros-args', '-p', 'use_sim_time:=true']
    arguments += ['-p', f'tree_file:={tree_file}']
    for parameter in parameters:
        arguments += ['-p', parameter]
    return subprocess.run(
        arguments,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=TREE_TIMEOUT,
        check=False,
    )


class TreeHarness(FollowerHarness):
    """FollowerHarness that also records the follower's goals sent by other clients."""

    def _start(self):
        self._goal_lock = threading.Lock()
        self._statuses = {}
        self._feedback = {}
        super()._start()
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            GoalStatusArray, '/follow_curve/_action/status', self._on_status, latched
        )
        self.node.create_subscription(
            FollowCurve.Impl.FeedbackMessage,
            '/follow_curve/_action/feedback',
            self._on_curve_feedback,
            100,
        )

    def _on_status(self, message):
        with self._goal_lock:
            for status in message.status_list:
                self._statuses[bytes(status.goal_info.goal_id.uuid)] = status.status

    def _on_curve_feedback(self, message):
        goal = bytes(message.goal_id.uuid)
        with self._goal_lock:
            self._feedback[goal] = self._feedback.get(goal, 0) + 1

    def curve_goals(self):
        """Return {goal id: (last status, feedback messages)} of every goal seen."""
        with self._goal_lock:
            return {goal: (s, self._feedback.get(goal, 0)) for goal, s in self._statuses.items()}

    def new_curve_goal(self, before, timeout=10.0):
        """Return (status, feedback count) of the one goal not in ``before`` once it ended."""
        terminal = (
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        )

        def ended():
            new = [v for g, v in self.curve_goals().items() if g not in before]
            return len(new) == 1 and new[0][0] in terminal

        self._wait_wall(ended, timeout)
        new = [v for g, v in self.curve_goals().items() if g not in before]
        if len(new) != 1:
            raise AssertionError(f'expected one new follow_curve goal, saw {len(new)}')
        return new[0]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start curve_follower_demo on open_10x10 with bt_navigator running follow_curve.xml."""
    return launch.demo(
        'curve_follower_demo.launch.py', MOTION_MODEL, 'open_10x10', ('follow_curve_bt',)
    )


class TestFollowCurveTree(unittest.TestCase):
    """The example trees, in name order on one robot."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2 and the follower."""
        cls.nav = TreeHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def test_1_bt_runner(self):
        """bt_runner: approach, then one lap of the ellipse; SUCCESS."""
        before = self.nav.curve_goals()
        x, y, yaw = APPROACH_POSE
        run = _runner('follow_curve.xml', f'goal_pose:=[{x}, {y}, {yaw}]')
        print(run.stdout[-4000:], flush=True)
        self.assertEqual(run.returncode, 0, 'bt_runner did not finish with SUCCESS')
        self.assertIn('FollowEllipse', run.stdout)
        status, feedback = self.nav.new_curve_goal(before)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertGreater(feedback, 0)

    def test_2_bt_navigator(self):
        """bt_navigator: NavigateToPose runs the same tree; SUCCEEDED."""
        before = self.nav.curve_goals()
        episode = self.nav.navigate(*APPROACH_POSE, timeout=TREE_TIMEOUT)
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.status_name)
        status, feedback = self.nav.new_curve_goal(before)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertGreater(feedback, 0)

    def test_3_patrol_tree_loads(self):
        """bt_runner creates the patrol tree against the running servers."""
        run = _runner('follow_curve_patrol.xml', 'dry_run:=true')
        print(run.stdout[-2000:], flush=True)
        self.assertEqual(run.returncode, 0, 'bt_runner could not create the patrol tree')
