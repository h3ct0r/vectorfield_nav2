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

"""Omnidirectional: the Stanford TidyBot base with the curve follower in MuJoCo.

Everything is preset: robot ``tidybot``, world ``open_10x10``, config/robots/tidybot.yaml and
config/scenarios/omnidirectional.yaml, whose route run_scenario drives once Nav2 is up
(``scenario:=false`` keeps only its world; ``exit_on_done:=true`` ends the launch with it). The
base strafes around a rounded rectangle (a superellipse) at a fixed heading (``omni_heading_policy:
fixed``).

Examples::

    ros2 launch vectorfield_mujoco omnidirectional.launch.py
    ros2 launch vectorfield_mujoco omnidirectional.launch.py use_rviz:=false camera_topic:=camera
    ros2 launch vectorfield_mujoco omnidirectional.launch.py viewer:=true record_path:=~/omni.mp4
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the omnidirectional example."""
    return examples.launch_description(
        'curve_follower', example='omnidirectional', robot='tidybot', world='open_10x10'
    )
