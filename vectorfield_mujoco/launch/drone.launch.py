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

"""Drone: the Skydio X2 flies a helix with the 3D follower and its CBF filter in MuJoCo.

Everything is preset: robot ``x2``, no map, config/robots/x2.yaml and
config/scenarios/drone.yaml, whose curve run_scenario flies once the follower is up
(``scenario:=false`` keeps only its world; ``exit_on_done:=true`` ends the launch with it).
The drone takes off and climbs two turns of a helix; a pillar and a sphere stand within its
body radius of the curve, and the CBF filter (``obstacle_mode: cbf``, against the simulated
obstacle cloud) steers it around them.

Examples::

    ros2 launch vectorfield_mujoco drone.launch.py
    ros2 launch vectorfield_mujoco drone.launch.py use_rviz:=false camera_topic:=camera
    ros2 launch vectorfield_mujoco drone.launch.py viewer:=true record_path:=~/drone.mp4
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the drone example."""
    return examples.launch_description('follower_3d', example='drone', robot='x2', world='')
