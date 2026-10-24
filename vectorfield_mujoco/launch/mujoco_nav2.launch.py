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

"""Nav2 with the vector-field controller on a MuJoCo robot, on every supported distro.

Starts mujoco_sim (the physics and the time source, with vectorfield_sim's interface) in a world
built from the map that map_server serves, and the Nav2 side of vectorfield_bringup's
kinematic_sim_demo (``vectorfield_bringup.demo.nav2_demo_actions``): map_server, Nav2's
navigation_launch.py, Humble's collision_monitor, initial_pose_publisher and RViz. The
parameters are the robot's motion-model overlay, ``config/mujoco_sim.yaml``,
``config/robots/<robot>.yaml`` and the scenario's ``params`` (``vectorfield_mujoco.examples``).
With a ``scenario``, run_scenario drives its waypoints.

Examples::

    ros2 launch vectorfield_mujoco mujoco_nav2.launch.py robot:=tiago scenario:=differential
    ros2 launch vectorfield_mujoco mujoco_nav2.launch.py robot:=scout_mini world:=corridor_door
    ros2 launch vectorfield_mujoco mujoco_nav2.launch.py robot:=ackermann_car \
        obstacles:="[{type: box, pos: [2, 0], size: [0.2, 0.2, 0.25]}]" camera_topic:=camera
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the generic MuJoCo Nav2 launch."""
    return examples.launch_description('nav2')
