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

"""Curves in moving frames on vectorfield_sim (M5 exit criteria), for ``VF_TEST_MOTION_MODEL``.

curve_follower_demo on ``open_10x10``; the frame comes from moving_frame_publisher.py (parent
``odom``), run by the test. The robot first drives to a point of the curve, facing along it; the
goal (FRAME_MOVING, laps 0) then runs for RUN_TIME s and is cancelled. Each case runs with the
Odometry source (``moving_frame/odom``) and with the TF source (``moving_frame_reference``):
max |D| after 10 s below 0.10 m, no STATUS_ASSUMPTION3_CLAMPED, the goal still running.

1. a circle of radius 1.5 m translating at 0.1 m/s;
2. an ellipse 2 m × 1 m rotating at 0.05 rad/s about a point 1 m from its centre;
3. moving_frame_publisher killed mid-run: the goal aborts with STALE_INPUT.
"""

import unittest
from collections import namedtuple

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.follower import FollowerHarness, MovingFrame, curve_goal

from vectorfield_bringup import curves
from vectorfield_msgs.action import FollowCurve
from vectorfield_msgs.msg import FieldState
from vectorfield_system_tests import launch

MOTION_MODEL = launch.motion_model()

FRAME = 'moving_frame'
ODOM_TOPIC = f'{FRAME}/odom'
RUN_TIME = 30.0
SETTLE_TIME = 10.0
MOVING_LIMIT = 0.10
KILL_AFTER = 8.0
ABORT_TIMEOUT = 15.0

# A curve in the moving frame, the frame's start pose in the map and its twist, and the robot's
# start pose in the map (on the curve, facing along it).
Case = namedtuple('Case', 'curve frame twist start')
# A circle of radius 1.5 m whose centre starts at (−1.5, 0) and translates along +x.
CIRCLE = Case(
    curves.builtin('ellipse', a=1.5, b=1.5), (-1.5, 0.0, 0.0), (0.1, 0.0, 0.0), (-1.5, -1.5, 0.0)
)
# An ellipse centred 1 m along x from the frame origin, which spins about the map origin.
ELLIPSE = Case(
    curves.builtin('ellipse', a=2.0, b=1.0, cx=1.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.05),
    (1.0, -1.0, 0.0),
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start curve_follower_demo on open_10x10."""
    return launch.demo('curve_follower_demo.launch.py', MOTION_MODEL, 'open_10x10')


class TestMovingFrames(unittest.TestCase):
    """The moving-frame cases, in name order on one robot."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2 and the follower."""
        cls.nav = FollowerHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def test_1_translating_circle_odometry(self):
        """A translating circle, frame twist from Odometry."""
        self._track('translating circle (Odometry)', CIRCLE, ODOM_TOPIC)

    def test_2_translating_circle_tf(self):
        """A translating circle, frame twist from TF."""
        self._track('translating circle (TF)', CIRCLE, '')

    def test_3_rotating_ellipse_odometry(self):
        """A rotating ellipse, frame twist from Odometry."""
        self._track('rotating ellipse (Odometry)', ELLIPSE, ODOM_TOPIC)

    def test_4_rotating_ellipse_tf(self):
        """A rotating ellipse, frame twist from TF."""
        self._track('rotating ellipse (TF)', ELLIPSE, '')

    def test_5_publisher_killed(self):
        """moving_frame_publisher killed mid-run: STALE_INPUT."""
        nav = self.nav
        frame, handle = self._start(CIRCLE, ODOM_TOPIC)
        try:
            ended = nav.wait_result(handle, KILL_AFTER)
            self.assertIsNone(ended, 'the goal ended before the publisher was killed')
        finally:
            frame.kill()
        killed = nav.now()
        status = nav.wait_result(handle, ABORT_TIMEOUT)
        if status is None:
            status = nav.cancel(handle)
        episode = nav.curve_episode(
            handle, GoalStatus.STATUS_UNKNOWN if status is None else status
        )
        print(
            f'publisher killed at +{killed - episode.start:.1f} s: {episode.describe()}',
            flush=True,
        )
        self.assertEqual(episode.status, GoalStatus.STATUS_ABORTED, episode.describe())
        self.assertEqual(episode.error_code, FollowCurve.Result.STALE_INPUT, episode.describe())

    def _start(self, case, odom_topic):
        """Start the case's frame, drive to its start, set the frame moving, send the goal."""
        nav = self.nav
        frame = MovingFrame(nav.odom_pose(*case.frame), child=FRAME)
        try:
            approach = nav.navigate(*case.start, timeout=90.0)
            self.assertEqual(approach.status, GoalStatus.STATUS_SUCCEEDED, approach.status_name)
            self.assertTrue(nav.wait_for_frame(frame), f'no {frame.parent} -> {FRAME} transform')
            frame.move(nav, case.twist)
            goal = curve_goal(case.curve, frame=FRAME, moving=True, frame_odom_topic=odom_topic)
            return frame, nav.send_curve(goal)
        except BaseException:
            frame.kill()
            raise

    def _track(self, label, case, odom_topic):
        nav = self.nav
        frame, handle = self._start(case, odom_topic)
        try:
            episode = nav.run_for(handle, RUN_TIME)
        finally:
            frame.kill()
        error = episode.max_distance(episode.start + SETTLE_TIME)
        clamped = episode.flagged(FieldState.STATUS_ASSUMPTION3_CLAMPED)
        print(
            f'{label}: {episode.describe()}; max |D| after {SETTLE_TIME:.0f} s {error:.4f} m; '
            f'{len(clamped)} clamped of {len(episode.samples)} samples',
            flush=True,
        )
        self.assertEqual(episode.status, GoalStatus.STATUS_CANCELED, episode.describe())
        self.assertGreater(episode.end - episode.start, RUN_TIME - 1.0, episode.describe())
        self.assertLess(error, MOVING_LIMIT, f'{label}: max |D| after {SETTLE_TIME:.0f} s')
        self.assertEqual(clamped, [], f'{label}: Assumption 3 clamped')
