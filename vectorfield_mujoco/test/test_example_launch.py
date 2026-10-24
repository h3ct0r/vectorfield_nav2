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

"""Launch test of a MuJoCo example: its scenario succeeds, headless.

The example launch ``VF_MUJOCO_EXAMPLE`` (set per CTest test) starts with ``scenario:=false``
(its world, without the runner) and ``use_rviz:=false``; the test then runs ``run_scenario`` on
``VF_MUJOCO_SCENARIO`` (a scenario name or path; default the example's own), which exits with 0
only when every NavigateToPose goal (or the FollowCurve or FollowCurve3 goal) SUCCEEDED.
``VF_MUJOCO_TIMEOUT`` (s) bounds the scenario.
"""

import os
import subprocess
import unittest

import launch_testing.markers
import pytest

EXAMPLE = os.environ.get('VF_MUJOCO_EXAMPLE', 'skid_steer')
SCENARIO = os.environ.get('VF_MUJOCO_SCENARIO', EXAMPLE)
TIMEOUT = float(os.environ.get('VF_MUJOCO_TIMEOUT', '600'))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start the example without its runner."""
    import launch_testing.actions
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    path = os.path.join(
        get_package_share_directory('vectorfield_mujoco'), 'launch', f'{EXAMPLE}.launch.py'
    )
    arguments = {'scenario': 'false', 'use_rviz': 'false'}
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path), launch_arguments=arguments.items()
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestExample(unittest.TestCase):
    """The scenario of the example."""

    def test_scenario(self):
        """run_scenario exits with 0: every goal SUCCEEDED."""
        from ament_index_python.packages import get_package_prefix

        runner = os.path.join(
            get_package_prefix('vectorfield_mujoco'), 'lib', 'vectorfield_mujoco', 'run_scenario'
        )
        # The launch's start-up (Nav2's container, the physics) comes out of the same budget;
        # the runner's own limit leaves a minute for the shutdown.
        done = subprocess.run(
            [runner, SCENARIO, '--timeout', f'{max(60.0, TIMEOUT - 120.0):g}'],
            capture_output=True,
            text=True,
            timeout=max(90.0, TIMEOUT - 60.0),
        )
        output = done.stdout[-4000:] + done.stderr[-4000:]
        self.assertEqual(done.returncode, 0, output)
