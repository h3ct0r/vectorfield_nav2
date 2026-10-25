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

"""Curve follower laps on vectorfield_sim, for ``VF_TEST_MOTION_MODEL``.

curve_follower_demo on ``open_10x10``. Each closed curve is approached as send_curve.py does
(NavigateToPose to its closest point, facing along the tangent), then followed for 3 laps at
0.4 m/s: the goal SUCCEEDED with 3 laps, and max |D| after lap 1 is below 0.05 m.

1. an ellipse 2 m × 1 m (semi-axes);
2. a Bernoulli lemniscate, a = 2 m.
"""

import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.follower import FollowerHarness, curve_goal

from vectorfield_bringup import curves
from vectorfield_system_tests import launch

MOTION_MODEL = launch.motion_model()

LAPS = 3
SPEED = 0.4
LAP_LIMIT = 0.05


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start curve_follower_demo on open_10x10."""
    return launch.demo('curve_follower_demo.launch.py', MOTION_MODEL, 'open_10x10')


class TestCurveLaps(unittest.TestCase):
    """The closed curves, in name order on one robot."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2 and the follower."""
        cls.nav = FollowerHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def test_1_ellipse(self):
        """Three laps of an ellipse 2 m × 1 m."""
        self._laps('ellipse', curves.builtin('ellipse', a=2.0, b=1.0))

    def test_2_lemniscate_bernoulli(self):
        """Three laps of a Bernoulli lemniscate, a = 2 m."""
        self._laps('lemniscate', curves.builtin('lemniscate_bernoulli', a=2.0))

    def _laps(self, label, description):
        import vectorfield as vf

        nav = self.nav
        length = vf.make_curve(curves.curve_spec(description)).length()
        nav.approach(description)
        # Twice the nominal time, plus the convergence.
        timeout = 2.0 * LAPS * length / SPEED + 30.0
        episode = nav.follow(curve_goal(description, laps=LAPS, speed=SPEED), timeout)
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.describe())
        self.assertEqual(episode.result.laps_completed, LAPS, episode.describe())
        lap_1 = episode.lap_end(1)
        error = episode.max_distance(lap_1)
        print(
            f'{label}: {episode.describe()}; nominal {LAPS * length / SPEED:.1f} s; lap 1 '
            f'ended at +{lap_1 - episode.start:.1f} s; max |D| after it {error:.4f} m '
            f'({len(episode.samples)} field_state samples)',
            flush=True,
        )
        self.assertLess(error, LAP_LIMIT, f'{label}: max |D| after lap 1')
