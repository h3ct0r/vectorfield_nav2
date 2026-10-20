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

"""Nav2 with the vector-field controller on the Gazebo TurtleBot4 simulation (Jazzy and later).

Includes nav2_bringup's tb4_simulation_launch.py (nav2_minimal_tb4_sim's depot world, AMCL and
Nav2) with the merged parameters, initial_pose_publisher at the spawn pose (the world's (-8, 0)
is the origin of the depot map) and RViz with vectorfield.rviz.

Example::

    ros2 launch vectorfield_bringup tb4_gazebo_demo.launch.py headless:=False
"""

from vectorfield_bringup import demo


def generate_launch_description():
    """Build the Gazebo TurtleBot4 demo."""
    # The depot map's origin is the spawn point: map = world + (8, 0).
    return demo.gazebo_demo('tb4', 'depot', x=-8.0, y=0.0, yaw=0.0, map_offset=(8.0, 0.0))
