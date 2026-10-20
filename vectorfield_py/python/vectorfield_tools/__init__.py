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

- ``vf-curve`` (:mod:`.vf_curve`): describe, sample, plot and export curves;
- ``vf-plot`` (:mod:`.vf_plot`): fields, streamlines, obstacles and the field decomposition;
- ``vf-sim`` (:mod:`.vf_sim`): closed-loop simulation with plots and GIF animations;
- ``vf-sim3d`` (:mod:`.vf_sim3d`): the same for the experimental 3D core (mplot3d, GIF);
- ``vf-tune`` (:mod:`.vf_tune`): configuration validation, suggestions and parameter sweeps.

The helpers behind them (:mod:`.curves`, :mod:`.curves3d`, :mod:`.config`, :mod:`.obstacles`,
:mod:`.simulation` and :mod:`.plotting`) are importable, for example by
``docs/gen_figures.py``. matplotlib, Pillow and PyYAML are imported only when needed: install
the ``tools`` extra (``pip install './vectorfield_py[tools]'``) or, under ROS, the
python3-matplotlib, python3-pil and python3-yaml rosdep keys.
"""
