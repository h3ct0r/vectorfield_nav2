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

"""Differential drive: PAL TIAGo with Nav2 and the vector-field controller in MuJoCo.

Everything is preset: robot ``tiago``, world ``open_10x10``, config/robots/tiago.yaml and
config/scenarios/differential.yaml, whose route run_scenario drives once Nav2 is up
(``scenario:=false`` keeps only its world; ``exit_on_done:=true`` ends the launch with it). An
unmapped box stands on the path: the controller contours around it (``obstacle_mode: contour``),
rejoins the path and rotates to the goal heading.

Examples::

    ros2 launch vectorfield_mujoco differential.launch.py
    ros2 launch vectorfield_mujoco differential.launch.py use_rviz:=false camera_topic:=camera
    ros2 launch vectorfield_mujoco differential.launch.py viewer:=true record_path:=~/diff.mp4
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the differential example."""
    return examples.launch_description(
        'nav2', example='differential', robot='tiago', world='open_10x10'
    )
