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

"""Skid-slip identification launch test on vectorfield_sim (skid_steer).

kinematic_sim_demo on ``open_10x10`` with ``motion_model: skid_steer``, whose plant slips with
``slip_eta_psi`` 0.75 and ``slip_x_icr`` 0.05 m (``vectorfield_bringup.demo.SIM_SKID_SLIP``).
``identify_skid_slip.py --yes`` drives its default sequence (turns in place and arcs, both ways)
through Nav2's velocity smoother and collision monitor, and must recover both values within
10 % (docs/tutorials/field_testing.md, stage 6).
"""

import json
import os
import subprocess
import tempfile
import unittest

import launch_testing.markers
import pytest
from vectorfield_system_tests.harness import NavHarness

from vectorfield_bringup import demo, params
from vectorfield_system_tests import launch

MOTION_MODEL = launch.motion_model()
RELATIVE_TOLERANCE = 0.10


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start kinematic_sim_demo on open_10x10."""
    return launch.demo('kinematic_sim_demo.launch.py', MOTION_MODEL, 'open_10x10')


class TestSkidSlipIdentification(unittest.TestCase):
    """identify_skid_slip.py recovers the simulated slip."""

    @classmethod
    def setUpClass(cls):
        """Wait for Nav2 (the command chain: velocity smoother, collision monitor)."""
        nav = NavHarness()
        try:
            nav.wait_until_active()
        finally:
            nav.close()

    def test_identification(self):
        """Run the default sequence and compare with the plant's slip."""
        # The command the simulator receives: after bringup's collision_monitor on Humble.
        final = 'cmd_vel_safe' if params.ros_distro() == 'humble' else 'cmd_vel'
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'slip.json')
            done = subprocess.run(
                [
                    'ros2',
                    'run',
                    'vectorfield_bringup',
                    'identify_skid_slip.py',
                    '--yes',
                    '--use-sim-time',
                    '--final-topic',
                    final,
                    '--json',
                    path,
                ],
                capture_output=True,
                text=True,
                timeout=180.0,
            )
            output = done.stdout[-3000:] + done.stderr[-2000:]
            self.assertEqual(done.returncode, 0, output)
            with open(path) as stream:
                result = json.load(stream)
        eta, x_icr = demo.SIM_SKID_SLIP
        self.assertAlmostEqual(result['eta_psi'], eta, delta=RELATIVE_TOLERANCE * eta, msg=output)
        self.assertAlmostEqual(
            result['x_icr'], x_icr, delta=RELATIVE_TOLERANCE * x_icr, msg=output
        )
