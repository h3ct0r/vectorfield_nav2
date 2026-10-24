#!/usr/bin/env python3
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

"""The MuJoCo simulator node: ``ros2 run vectorfield_mujoco mujoco_sim --ros-args -p ...``.

See :mod:`vectorfield_mujoco.sim_node` for its parameters and topics; the example launches
start it.
"""

import sys

from vectorfield_mujoco import sim_node

if __name__ == '__main__':
    sys.exit(sim_node.main())
