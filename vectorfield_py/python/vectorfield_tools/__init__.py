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

"""Command-line tools built on the vectorfield bindings.

The CLIs arrive in M4: ``vf-plot`` (fields and streamlines), ``vf-sim`` (closed-loop
simulation, with GIF output), ``vf-tune`` and ``vf-curve``.
They import matplotlib lazily. Install the ``tools`` extra
(``pip install './vectorfield_py[tools]'``) or, under ROS, the python3-matplotlib rosdep.
"""
