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

"""Skid-steer: the AgileX Scout Mini with Nav2 and the vector-field controller in MuJoCo.

Everything is preset: robot ``scout_mini``, world ``corridor_door``, config/robots/scout_mini.yaml
and config/scenarios/skid_steer.yaml, whose route run_scenario drives once Nav2 is up
(``scenario:=false`` keeps only its world; ``exit_on_done:=true`` ends the launch with it). The
robot drives through the corridor's 0.8 m door and back; the controller compensates the identified
skid slip (``skid_yaw_rate_efficiency``).

Examples::

    ros2 launch vectorfield_mujoco skid_steer.launch.py
    ros2 launch vectorfield_mujoco skid_steer.launch.py use_rviz:=false camera_topic:=camera
    ros2 launch vectorfield_mujoco skid_steer.launch.py viewer:=true record_path:=~/skid_steer.mp4
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the skid_steer example."""
    return examples.launch_description(
        'nav2', example='skid_steer', robot='scout_mini', world='corridor_door'
    )
