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

"""Open curves and the Nav2 interlock of the curve follower, for ``VF_TEST_MOTION_MODEL``.

curve_follower_demo on ``open_10x10``. Open curves are approached as send_curve.py does
(NavigateToPose to their start, facing along them).

1. an open polyline at 0.4 m/s: SUCCEEDED within length/v_r + 10 s (M5 exit criterion);
2. an open curve at 0.04 m/s: SUCCEEDED, never BLOCKED (the progress watchdog scales with the
   speed);
3. a FollowCurve goal while a NavigateToPose executes: aborted with NAV2_ACTIVE, without a
   single follower cycle.
"""

import math
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.follower import FollowerHarness, curve_goal

from vectorfield_bringup import curves
from vectorfield_msgs.action import FollowCurve
from vectorfield_msgs.msg import FieldState
from vectorfield_system_tests import launch

MOTION_MODEL = launch.motion_model()

SPEED = 0.4
TIME_SLACK = 10.0
POLYLINE = curves.polyline([(-2.5, -1.0), (0.5, -1.0), (0.5, 1.0), (2.5, 1.0)])
SLOW_SPEED = 0.04
SLOW_CURVE = curves.expression('s', 'A*sin(k*s)', 0.0, 1.0, closed=False, A=0.1, k=3.0)
NAV2_ACTIVE_TIMEOUT = 10.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start curve_follower_demo on open_10x10."""
    return launch.demo('curve_follower_demo.launch.py', MOTION_MODEL, 'open_10x10')


class TestOpenCurves(unittest.TestCase):
    """The open curves, then the interlock, in name order on one robot."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2 and the follower."""
        cls.nav = FollowerHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def test_1_polyline(self):
        """An open polyline at 0.4 m/s ends within length/v_r + 10 s."""
        nav = self.nav
        length = sum(math.dist(a, b) for a, b in zip(POLYLINE.points[:-1], POLYLINE.points[1:]))
        budget = length / SPEED + TIME_SLACK
        nav.approach(POLYLINE)
        episode = nav.follow(curve_goal(POLYLINE, speed=SPEED), 2.0 * budget)
        elapsed = _seconds(episode.result.total_elapsed_time) if episode.result else math.inf
        print(
            f'polyline ({length:.2f} m): {episode.describe()}; budget {budget:.1f} s; '
            f'max |D| {episode.max_distance(episode.start):.3f} m',
            flush=True,
        )
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.describe())
        self.assertLessEqual(elapsed, budget, episode.describe())

    def test_2_slow_open_curve(self):
        """An open curve at 0.04 m/s succeeds and is never BLOCKED."""
        import vectorfield as vf

        nav = self.nav
        length = vf.make_curve(curves.curve_spec(SLOW_CURVE)).length()
        nav.approach(SLOW_CURVE)
        timeout = 2.0 * length / SLOW_SPEED + 30.0
        episode = nav.follow(curve_goal(SLOW_CURVE, speed=SLOW_SPEED), timeout)
        blocked = episode.flagged(FieldState.STATUS_BLOCKED)
        print(
            f'slow open curve ({length:.2f} m at {SLOW_SPEED} m/s): {episode.describe()}; '
            f'{len(blocked)} BLOCKED samples',
            flush=True,
        )
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.describe())
        self.assertEqual(episode.error_code, FollowCurve.Result.NONE, episode.describe())
        self.assertEqual(blocked, [], 'BLOCKED status at 0.04 m/s')

    def test_3_nav2_active(self):
        """A goal sent while NavigateToPose executes aborts with NAV2_ACTIVE and stays silent."""
        nav = self.nav
        start = nav.pose()
        # Towards the centre of the map, far enough to still be driving when the curve is sent.
        heading = math.atan2(-start.y, -start.x)
        navigation = nav.send_goal(
            start.x + 3.0 * math.cos(heading), start.y + 3.0 * math.sin(heading), heading
        )
        try:
            nav.sleep(1.0)
            ellipse = curve_goal(curves.EXAMPLES['ellipse'], laps=1)
            episode = nav.follow(ellipse, NAV2_ACTIVE_TIMEOUT)
            still_navigating = not navigation.result.done()
        finally:
            nav.cancel(navigation)
        print(f'NAV2_ACTIVE case: {episode.describe()}', flush=True)
        self.assertTrue(still_navigating, 'NavigateToPose ended before the curve was sent')
        self.assertEqual(episode.status, GoalStatus.STATUS_ABORTED, episode.describe())
        self.assertEqual(episode.error_code, FollowCurve.Result.NAV2_ACTIVE, episode.describe())
        self.assertEqual(episode.samples, [], 'the follower ran cycles')


def _seconds(duration):
    return duration.sec + duration.nanosec * 1e-9
