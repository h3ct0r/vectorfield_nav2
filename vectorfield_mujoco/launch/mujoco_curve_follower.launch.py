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

"""The curve follower on a MuJoCo robot, on every supported distro.

mujoco_nav2.launch.py with bringup's ``curve_follower`` scenario overlay merged after the
motion-model overlay, plus the curve follower and its lifecycle manager
(``vectorfield_bringup.demo.curve_follower_actions``), as curve_follower_demo.launch.py does on
vectorfield_sim. Nav2 stays up for the approach, the local costmap and the command chain. With a
``scenario``, run_scenario drives to its curve and follows it for its laps (send_curve.py).

Examples::

    ros2 launch vectorfield_mujoco mujoco_curve_follower.launch.py scenario:=curve_following
    ros2 launch vectorfield_mujoco mujoco_curve_follower.launch.py robot:=tidybot
    ros2 run vectorfield_bringup send_curve.py --shape ellipse --laps 2
"""

from vectorfield_mujoco import examples


def generate_launch_description():
    """Build the generic MuJoCo curve follower launch."""
    return examples.launch_description('curve_follower')
