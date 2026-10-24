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

- macOS: ``cgl``;
- a display (``DISPLAY`` or ``WAYLAND_DISPLAY``): ``glfw``, which also serves the viewer;
- no display (Docker, CI): ``osmesa``, Mesa's software renderer, if its library is installed
  (``libosmesa6``; rosdep key ``libosmesa6-dev``), else ``egl`` if ``libEGL`` is (it needs a
  GPU or Mesa's EGL drivers, and fails in Docker on macOS), else none.

A backend is chosen only when its library exists: with ``MUJOCO_GL=osmesa`` and no OSMesa,
PyOpenGL fails inside ``import mujoco`` (not with an ImportError), and the physics would go with
it. With none, ``MUJOCO_GL`` stays unset: MuJoCo imports and steps, and only rendering (the
viewer, offscreen cameras, recording) fails. :func:`probe` tells, in a separate process, whether
importing and rendering work.
"""

import ctypes.util
import os
import subprocess
import sys

#: The libraries of the headless backends, in order of preference (``ctypes.util`` names).
HEADLESS = (('osmesa', 'OSMesa'), ('egl', 'EGL'))

#: The probe: an 8 x 8 offscreen render of an empty model.
_PROBE = (
    'import mujoco\n'
    'if {render}:\n'
    '    model = mujoco.MjModel.from_xml_string("<mujoco/>")\n'
    '    renderer = mujoco.Renderer(model, 8, 8)\n'
    '    renderer.update_scene(mujoco.MjData(model))\n'
    '    assert renderer.render().shape == (8, 8, 3)\n'
    '    renderer.close()\n'
)


def choose_backend(environ=None, platform=None, find_library=ctypes.util.find_library):
    """Return the backend :func:`configure` would select, or None when none is usable.

    Args:
        environ: The environment (default ``os.environ``).
        platform: ``sys.platform`` (default the running one).
        find_library: ``ctypes.util.find_library`` (replaceable for tests).

    """
    environ = os.environ if environ is None else environ
    platform = sys.platform if platform is None else platform
    if environ.get('MUJOCO_GL'):
        return environ['MUJOCO_GL']
    if platform == 'darwin':
        return 'cgl'
    if environ.get('DISPLAY') or environ.get('WAYLAND_DISPLAY'):
        return 'glfw'
    for backend, library in HEADLESS:
        if find_library(library):
            return backend
    return None


def configure():
    """Set ``MUJOCO_GL`` (and ``PYOPENGL_PLATFORM`` for OSMesa/EGL) if unset; return it.

    Returns the backend in effect, or None when no backend is usable (``MUJOCO_GL`` is then
    left unset; see the module docstring). If ``mujoco`` is already imported the choice cannot
    take effect any more, and the variable is left untouched.
    """
    if 'mujoco' in sys.modules:
        return os.environ.get('MUJOCO_GL') or None
    backend = choose_backend()
    if backend is None:
        return None
    os.environ['MUJOCO_GL'] = backend
    if backend in ('osmesa', 'egl'):
        os.environ.setdefault('PYOPENGL_PLATFORM', backend)
    return backend


def unavailable_reason(backend):
    """Return why rendering is off with ``backend`` (from :func:`configure`), or None."""
    if backend:
        return None
    return (
        'no OpenGL backend for offscreen rendering: no display, and neither OSMesa '
        '(apt libosmesa6, rosdep key libosmesa6-dev) nor EGL is installed; set MUJOCO_GL to '
        'choose one'
    )


def probe(render=True, timeout=120.0):
    """Return None if ``import mujoco`` (and, with ``render``, a render) works, else why not.

    Runs in a separate Python process with this process's environment (call :func:`configure`
    first), because a broken backend can fail inside the import or abort the process.
    """
    code = _PROBE.format(render=bool(render))
    try:
        done = subprocess.run(
            [sys.executable, '-c', code], capture_output=True, text=True, timeout=timeout
        )
    except subprocess.TimeoutExpired:
        return f'the OpenGL probe did not finish within {timeout:g} s'
    if done.returncode == 0:
        return None
    lines = [line for line in done.stderr.strip().splitlines() if line.strip()]
    detail = lines[-1] if lines else f'exit code {done.returncode}'
    return f'MUJOCO_GL={os.environ.get("MUJOCO_GL", "")!r}: {detail}'
