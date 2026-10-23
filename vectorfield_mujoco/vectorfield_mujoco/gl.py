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

"""Pick MuJoCo's OpenGL backend before ``mujoco`` is imported.

MuJoCo reads ``MUJOCO_GL`` once, when the module is first imported, so :func:`configure` must
run before any ``import mujoco`` (the simulator node calls it first thing). An explicit
``MUJOCO_GL`` is always kept. Otherwise:

- a display (``DISPLAY`` or ``WAYLAND_DISPLAY``): ``glfw``, which also serves the viewer;
- no display (Docker, CI): ``osmesa``, Mesa's software renderer (``libosmesa6``). EGL is
  faster on a GPU host but fails in Docker on macOS; set ``MUJOCO_GL=egl`` explicitly there;
- macOS: ``cgl``.
"""

import os
import sys


def choose_backend(environ=None, platform=None):
    """Return the backend :func:`configure` would select for an environment."""
    environ = os.environ if environ is None else environ
    platform = sys.platform if platform is None else platform
    if environ.get('MUJOCO_GL'):
        return environ['MUJOCO_GL']
    if platform == 'darwin':
        return 'cgl'
    if environ.get('DISPLAY') or environ.get('WAYLAND_DISPLAY'):
        return 'glfw'
    return 'osmesa'


def configure():
    """Set ``MUJOCO_GL`` (and ``PYOPENGL_PLATFORM`` for OSMesa/EGL) if unset; return it.

    Returns the backend in effect. If ``mujoco`` is already imported the choice cannot take
    effect any more, and the variable is left untouched.
    """
    if 'mujoco' in sys.modules:
        return os.environ.get('MUJOCO_GL', '')
    backend = choose_backend()
    os.environ['MUJOCO_GL'] = backend
    if backend in ('osmesa', 'egl'):
        os.environ.setdefault('PYOPENGL_PLATFORM', backend)
    return backend
