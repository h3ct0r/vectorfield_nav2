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

"""Curve following: the AgileX Scout Mini laps a lemniscate with the curve follower in MuJoCo.

Everything is preset: robot ``scout_mini``, world ``open_10x10``, config/robots/scout_mini.yaml and
config/scenarios/curve_following.yaml, whose route run_scenario drives once Nav2 is up
(``scenario:=false`` keeps only its world; ``exit_on_done:=true`` ends the launch with it).
run_scenario approaches the curve with NavigateToPose, then sends the FollowCurve goal with its
laps (send_curve.py).

Examples::

    ros2 launch vectorfield_mujoco curve_following.launch.py
    ros2 launch vectorfield_mujoco curve_following.launch.py use_rviz:=false camera_topic:=camera
    ros2 launch vectorfield_mujoco curve_following.launch.py viewer:=true record_path:=~/curve.mp4
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the curve_following example."""
    return examples.launch_description(
        'curve_follower', example='curve_following', robot='scout_mini', world='open_10x10'
    )
