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

"""MuJoCo physics for the vectorfield_nav2 examples (no ROS dependency in these modules).

- :mod:`~vectorfield_mujoco.registry`: the example robots and how each is driven;
- :mod:`~vectorfield_mujoco.menagerie`: the pinned MuJoCo Menagerie fetcher;
- :mod:`~vectorfield_mujoco.scene`: worlds from Nav2 maps, obstacles and a robot;
- :mod:`~vectorfield_mujoco.adapters`: body twist to wheel, steering and planar commands;
- :mod:`~vectorfield_mujoco.flight`: the quadrotor velocity controller;
- :mod:`~vectorfield_mujoco.sensors`: 2D lidar and obstacle cloud;
- :mod:`~vectorfield_mujoco.overlay`: RViz-style markers drawn into a MuJoCo scene;
- :mod:`~vectorfield_mujoco.recorder` and :mod:`~vectorfield_mujoco.gl`: offscreen video.
"""
