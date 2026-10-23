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

"""Tests of vectorfield_mujoco.gl and vectorfield_mujoco.recorder.

The MP4 test needs ffmpeg and a working OpenGL backend (OSMesa when headless) and skips
without them.
"""

import json
import math
import shutil
import subprocess
import types

import numpy as np
import pytest

from vectorfield_mujoco import gl

# MuJoCo reads MUJOCO_GL when it is first imported.
BACKEND = gl.configure()

mujoco = pytest.importorskip('mujoco')

from vectorfield_mujoco import overlay, recorder, scene  # noqa: E402


@pytest.mark.parametrize(
    'environ,platform,expected',
    [
        ({'MUJOCO_GL': 'egl', 'DISPLAY': ':0'}, 'linux', 'egl'),
        ({'DISPLAY': ':1'}, 'linux', 'glfw'),
        ({'WAYLAND_DISPLAY': 'wayland-0'}, 'linux', 'glfw'),
        ({}, 'linux', 'osmesa'),
        ({}, 'darwin', 'cgl'),
    ],
)
def test_choose_backend(environ, platform, expected):
    """MUJOCO_GL is kept if set, else picked from the display and the platform."""
    assert gl.choose_backend(environ, platform) == expected


def test_ffmpeg_command():
    """The encoder writes H.264 yuv420p with faststart."""
    cmd = recorder.ffmpeg_command('out.mp4', 960, 540, 30)
    joined = ' '.join(cmd)
    assert '-s 960x540' in joined and '-r 30' in joined and '-pix_fmt yuv420p' in joined
    assert '-c:v libx264' in joined and '+faststart' in joined and cmd[-1] == 'out.mp4'


def test_chase_camera_follows_with_smoothing():
    """The chase camera follows position and heading with a first-order lag."""
    rig = recorder.CameraRig('chase', body=1, distance=2.0, smoothing=0.5, height=0.0)
    data = types.SimpleNamespace(time=0.0, xpos=np.zeros((2, 3)), xmat=np.zeros((2, 9)))
    data.xmat[1] = np.eye(3).ravel()
    cam = rig.update(data)
    assert cam.azimuth == pytest.approx(0.0) and cam.lookat == pytest.approx([0, 0, 0])
    # The body jumps 1 m ahead and turns 90°: after one time constant the camera covers 63 %.
    data.time = 0.5
    data.xpos[1] = [1.0, 0.0, 0.0]
    data.xmat[1] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float).ravel()
    cam = rig.update(data)
    alpha = 1 - math.exp(-1)
    assert cam.lookat[0] == pytest.approx(alpha) and cam.azimuth == pytest.approx(90 * alpha)
    top = recorder.CameraRig('top', body=1)
    assert top.update(data).elevation < -89
    with pytest.raises(ValueError):
        recorder.CameraRig('orbit')


def test_record_mp4(tmp_path):
    """A one-second drive records ten frames of H.264 video."""
    if shutil.which('ffmpeg') is None:
        pytest.skip('ffmpeg not installed')
    world = scene.make_world(
        'scout_mini', obstacles=[{'type': 'box', 'pos': [1.5, 0.0], 'size': [0.2, 0.2, 0.2]}]
    )
    try:
        probe = mujoco.Renderer(world.model, 64, 64)
        probe.close()
    except Exception as e:  # any GL failure means "no backend here"
        pytest.skip(f'no OpenGL backend ({BACKEND}): {e}')
    path = tmp_path / 'clip.mp4'
    markers = [
        {
            'type': 'LINE_STRIP',
            'points': [(0, 0, 0.02), (2, 0, 0.02), (2, 1, 0.02)],
            'scale': (0.03, 0, 0),
            'rgba': (0.1, 0.8, 0.2, 1),
        }
    ]
    camera = recorder.CameraRig('chase', body=world.base_id)
    with recorder.Recorder(
        world.model,
        path,
        width=320,
        height=180,
        fps=10,
        camera=camera,
        overlay=lambda scn: overlay.add_markers(scn, markers),
    ) as rec:
        while world.data.time < 1.0:
            world.drive(0.4, 0.0, 0.0)
            mujoco.mj_step(world.model, world.data)
            rec.maybe_capture(world.data)
        frame = rec.render(world.data)
    # Frames are taken in simulated time: 10 fps over 1 s.
    assert rec.frames == 10
    assert frame.shape == (180, 320, 3) and frame.std() > 5
    assert path.stat().st_size > 1000
    if shutil.which('ffprobe'):
        info = json.loads(
            subprocess.run(
                [
                    'ffprobe',
                    '-v',
                    'error',
                    '-select_streams',
                    'v:0',
                    '-count_frames',
                    '-show_entries',
                    'stream=codec_name,pix_fmt,width,nb_read_frames',
                    '-of',
                    'json',
                    str(path),
                ],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )['streams'][0]
        assert info['codec_name'] == 'h264' and info['pix_fmt'] == 'yuv420p'
        assert int(info['nb_read_frames']) == 10 and info['width'] == 320
