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

"""The launch descriptions under test: bringup's demos, headless, with the test overlays."""

import os
from pathlib import Path

#: Test overlays, merged after each demo's own scenario overlay.
CONFIG = Path(__file__).resolve().parents[1] / 'test' / 'config'


def motion_model():
    """Return the motion model under test (``VF_TEST_MOTION_MODEL``, set per CTest test)."""
    return os.environ.get('VF_TEST_MOTION_MODEL', 'differential')


def overlay(name):
    """Return the path of the test overlay ``test/config/<name>.yaml``."""
    return str(CONFIG / f'{name}.yaml')


def demo(launch_file, motion_model, map_name, scenarios=(), pose=(0.0, 0.0, 0.0)):
    """Return a launch_testing description that includes a vectorfield_bringup demo.

    Args:
        launch_file: The demo, e.g. ``kinematic_sim_demo.launch.py``.
        motion_model: The ``motion_model`` argument.
        map_name: The ``map`` argument (a name in bringup's ``maps/``).
        scenarios: Extra overlay paths, merged after the demo's own scenario overlay.
        pose: The initial map pose (x, y, yaw).

    """
    import launch_testing.actions
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    path = os.path.join(get_package_share_directory('vectorfield_bringup'), 'launch', launch_file)
    arguments = {
        'motion_model': motion_model,
        'map': map_name,
        'scenarios': ','.join(scenarios),
        'use_rviz': 'false',
        'initial_pose_x': str(pose[0]),
        'initial_pose_y': str(pose[1]),
        'initial_pose_yaw': str(pose[2]),
    }
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path), launch_arguments=arguments.items()
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )
