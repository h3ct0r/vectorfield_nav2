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

"""Open-area launch test on Nav2's loopback simulator for one motion model (Jazzy and later).

loopback_demo on ``open_10x10``: the fixed 3-goal route (each goal SUCCEEDED within 120 s, max |D|
after convergence below 0.10 m, 0.25 m Ackermann, the final position within the goal tolerance,
no NoValidControl status and no FollowPath failure) and the final-command publishers. The loopback
simulator localizes perfectly, so map -> base_footprint is the true pose. CMake registers this
test only from Jazzy on; run by hand on Humble, it skips.
"""

import unittest

import launch_testing.markers
import pytest
from vectorfield_system_tests.harness import NavHarness

from vectorfield_bringup import params
from vectorfield_system_tests import checks, launch

MOTION_MODEL = launch.motion_model()
HUMBLE = params.ros_distro() == 'humble'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start loopback_demo on open_10x10 (nothing on Humble, which has no loopback simulator)."""
    if HUMBLE:
        import launch_testing.actions
        from launch import LaunchDescription

        return LaunchDescription([launch_testing.actions.ReadyToTest()])
    return launch.demo('loopback_demo.launch.py', MOTION_MODEL, 'open_10x10')


@unittest.skipIf(HUMBLE, 'Nav2 has no loopback simulator on Humble')
class TestLoopbackRoute(unittest.TestCase):
    """The route, then the final-command publishers."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2."""
        cls.nav = NavHarness(ground_truth=False)
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def test_1_route(self):
        """Drive the fixed 3-goal route."""
        checks.check_route(self, self.nav, MOTION_MODEL)

    def test_2_final_command_publishers(self):
        """Only collision_monitor and the allow-listed servers publish the final command."""
        checks.check_final_command_publishers(self, self.nav)
