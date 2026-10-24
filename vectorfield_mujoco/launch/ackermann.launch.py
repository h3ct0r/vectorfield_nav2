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

"""Ackermann: the own car with Nav2 (Smac Hybrid-A*, Dubins) and the vector-field controller.

Everything is preset: robot ``ackermann_car``, world ``open_10x10``,
config/robots/ackermann_car.yaml and config/scenarios/ackermann.yaml, whose route run_scenario
drives once Nav2 is up (``scenario:=false`` keeps only its world; ``exit_on_done:=true`` ends the
launch with it). An unmapped box stands beside the path: the CBF safety filter
(``obstacle_mode: cbf``) keeps the car's turning margin, then the car U-turns.

Examples::

    ros2 launch vectorfield_mujoco ackermann.launch.py
    ros2 launch vectorfield_mujoco ackermann.launch.py use_rviz:=false camera_topic:=camera
    ros2 launch vectorfield_mujoco ackermann.launch.py viewer:=true record_path:=~/ackermann.mp4
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the ackermann example."""
    return examples.launch_description(
        'nav2', example='ackermann', robot='ackermann_car', world='open_10x10'
    )
