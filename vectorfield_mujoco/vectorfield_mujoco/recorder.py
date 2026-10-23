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

"""Offscreen camera frames and MP4 recording in simulated time.

:class:`CameraRig` places a free ``mjvCamera``: ``chase`` (behind the base, following its
position and heading with first-order smoothing in sim time), ``top`` (above the base, looking
down, +x to the right) or ``fixed`` (``lookat``, ``distance``, ``azimuth``, ``elevation``).
:class:`Recorder` renders with ``mujoco.Renderer`` at ``fps`` frames per *simulated* second, lets
an overlay callback add geoms (:func:`vectorfield_mujoco.overlay.add_markers`) and pipes the
frames to the system ``ffmpeg`` (H.264, yuv420p, ``+faststart``). Slow software rendering then
only slows the run; the clip keeps its timing.

Call :func:`vectorfield_mujoco.gl.configure` before importing ``mujoco`` when rendering headless.
"""

import math
import shutil
import subprocess

import numpy as np


class CameraRig:
    """A free camera following a body.

    Args:
        mode: ``'chase'``, ``'top'`` or ``'fixed'``.
        body: Body id to follow (chase, top).
        distance: Camera distance (m).
        elevation: Elevation (deg, negative looks down) for chase and fixed.
        azimuth: Azimuth (deg) for fixed.
        lookat: Look-at point for fixed (and the initial point of the others).
        smoothing: Time constant (s, sim time) of the chase position and heading filters.
        height: Height above the body origin that the chase and top cameras look at (m).
    """

    def __init__(
        self,
        mode='chase',
        body=-1,
        distance=2.5,
        elevation=-25.0,
        azimuth=90.0,
        lookat=(0.0, 0.0, 0.0),
        smoothing=0.6,
        height=0.2,
    ):
        if mode not in ('chase', 'top', 'fixed'):
            raise ValueError(f'camera mode must be chase, top or fixed, not {mode!r}')
        import mujoco

        self.mode = mode
        self.body = body
        self.smoothing = smoothing
        self.height = height
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance = distance
        self.camera.elevation = -89.9 if mode == 'top' else elevation
        self.camera.azimuth = 90.0 if mode == 'top' else azimuth
        self.camera.lookat[:] = lookat
        self._time = None
        self._yaw = None

    def update(self, data):
        """Move the camera for the current state; return the ``mjvCamera``."""
        if self.mode == 'fixed' or self.body < 0:
            return self.camera
        target = data.xpos[self.body] + np.array([0.0, 0.0, self.height])
        rot = data.xmat[self.body].reshape(3, 3)
        yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
        if self._time is None:
            alpha = 1.0
            self.camera.lookat[:] = target
            self._yaw = yaw
        else:
            dt = max(0.0, data.time - self._time)
            alpha = 1.0 - math.exp(-dt / self.smoothing) if self.smoothing > 0 else 1.0
        self._time = data.time
        self.camera.lookat[:] += alpha * (target - self.camera.lookat)
        if self.mode == 'chase':
            self._yaw += alpha * ((yaw - self._yaw + 180.0) % 360.0 - 180.0)
            self.camera.azimuth = self._yaw
        return self.camera


def ffmpeg_command(path, width, height, fps, crf=23, ffmpeg='ffmpeg'):
    """Return the ffmpeg command line that encodes raw RGB frames from stdin into ``path``."""
    return [
        ffmpeg,
        '-hide_banner',
        '-loglevel',
        'error',
        '-y',
        '-f',
        'rawvideo',
        '-pix_fmt',
        'rgb24',
        '-s',
        f'{width}x{height}',
        '-r',
        f'{fps:g}',
        '-i',
        '-',
        '-an',
        '-c:v',
        'libx264',
        '-preset',
        'medium',
        '-crf',
        str(crf),
        '-pix_fmt',
        'yuv420p',
        '-movflags',
        '+faststart',
        str(path),
    ]


class Recorder:
    """Renders frames in simulated time and encodes them to an MP4 with ffmpeg.

    Args:
        model: The compiled model.
        path: Output file (``.mp4``).
        width, height: Frame size (even numbers; at most the model's offscreen buffer).
        fps: Frames per simulated second.
        camera: A :class:`CameraRig`.
        overlay: Optional callable ``overlay(scene)`` adding geoms to the ``mjvScene`` of each
            frame (e.g. ``lambda scn: overlay.add_markers(scn, markers)``).
        scene_option: ``mjvOption`` (default :func:`vectorfield_mujoco.scene.visual_option`).
        crf: x264 quality (lower is better; 23 is ffmpeg's default).
        ffmpeg: The ffmpeg executable.
    """

    def __init__(
        self,
        model,
        path,
        width=960,
        height=540,
        fps=30.0,
        camera=None,
        overlay=None,
        scene_option=None,
        crf=23,
        ffmpeg='ffmpeg',
    ):
        import mujoco

        from vectorfield_mujoco import scene

        if width % 2 or height % 2:
            raise ValueError('width and height must be even (yuv420p)')
        if shutil.which(ffmpeg) is None:
            raise RuntimeError(f'{ffmpeg} not found: install ffmpeg to record videos')
        self.path = path
        self.period = 1.0 / float(fps)
        self.camera = camera or CameraRig('fixed', distance=6.0, elevation=-35.0)
        self.overlay = overlay
        self.option = scene_option or scene.visual_option()
        self.renderer = mujoco.Renderer(model, height=height, width=width, max_geom=10000)
        self.frames = 0
        self._next = None
        self._proc = subprocess.Popen(
            ffmpeg_command(path, width, height, fps, crf, ffmpeg),
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def render(self, data):
        """Return the current RGB frame (H x W x 3, uint8) with the overlay."""
        self.renderer.update_scene(data, self.camera.update(data), self.option)
        if self.overlay is not None:
            self.overlay(self.renderer.scene)
        return self.renderer.render()

    def maybe_capture(self, data):
        """Capture a frame if one is due at ``data.time``; return whether one was captured."""
        if self._next is not None and data.time + 1e-9 < self._next:
            return False
        self.capture(data)
        self._next = (data.time if self._next is None else self._next) + self.period
        return True

    def capture(self, data):
        """Render and encode one frame now."""
        frame = self.render(data)
        try:
            self._proc.stdin.write(np.ascontiguousarray(frame).tobytes())
        except BrokenPipeError as e:
            raise RuntimeError(f'ffmpeg failed: {self._proc.stderr.read().decode()}') from e
        self.frames += 1

    def close(self):
        """Finish the file; raise if ffmpeg failed."""
        if self._proc is None:
            return
        proc, self._proc = self._proc, None
        proc.stdin.close()
        err = proc.stderr.read().decode()
        if proc.wait() != 0:
            raise RuntimeError(f'ffmpeg exited with {proc.returncode}: {err.strip()}')
        self.renderer.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
