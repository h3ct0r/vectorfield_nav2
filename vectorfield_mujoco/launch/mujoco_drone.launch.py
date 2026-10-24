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

"""The 3D follower on a MuJoCo drone, on every supported distro.

Starts mujoco_sim with an aerial robot (the Skydio X2: point_sim_3d's interface, the flight
controller standing in for an autopilot's velocity mode) and the 3D follower with its lifecycle
manager and RViz (``vectorfield_bringup.demo.follower3d_actions``), as follower3d_demo.launch.py
does on point_sim_3d. There is no Nav2. The parameters are ``config/mujoco_sim.yaml``,
``config/robots/<robot>.yaml`` (the follower's limits, body radius and CBF) and the scenario's
``params`` (``vectorfield_mujoco.examples``). The follower's CBF filter runs against the
simulator's ``obstacle_cloud``, the surfaces of the world's obstacles. With a ``scenario``,
run_scenario flies its 3D curve (send_curve3d.py).

Examples::

    ros2 launch vectorfield_mujoco mujoco_drone.launch.py scenario:=drone
    ros2 launch vectorfield_mujoco mujoco_drone.launch.py initial_pose_z:=1.0 \
        obstacles:="[{type: sphere, pos: [0.6, 0.0, 1.3], size: [0.25]}]"
    ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the generic MuJoCo drone launch."""
    return examples.launch_description('follower_3d')
