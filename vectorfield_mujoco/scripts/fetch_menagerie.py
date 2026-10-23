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

"""Fetch the MuJoCo Menagerie robots of the examples ahead of the first launch.

``ros2 run vectorfield_mujoco fetch_menagerie [--dir DIR]`` sparse-clones pal_tiago,
stanford_tidybot and skydio_x2 at the pinned commit into ``$VF_MENAGERIE_DIR`` or
``~/.cache/vectorfield_mujoco/menagerie/<sha>`` and prints the directory. It does nothing when
the checkout is already complete.
"""

import sys

from vectorfield_mujoco import menagerie

if __name__ == '__main__':
    sys.exit(menagerie.main())
