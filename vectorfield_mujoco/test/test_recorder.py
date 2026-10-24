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

The camera and MP4 tests need MuJoCo; the MP4 test also needs ffmpeg and a working OpenGL
backend (OSMesa when headless). Whether MuJoCo imports and renders is probed in a separate
process first (vectorfield_mujoco.gl.probe), so a host without a backend skips them instead of
failing inside ``import mujoco``.
"""

import json
import math
import shutil
import subprocess
import types

import numpy as np
import pytest

from vectorfield_mujoco import gl, recorder

# MuJoCo reads MUJOCO_GL when it is first imported.
BACKEND = gl.configure()


@pytest.fixture(scope='module')
def mujoco():
    """Return the mujoco module, or skip if it cannot be imported with this backend."""
    problem = gl.probe(render=False)
    if problem:
        pytest.skip(f'mujoco does not import: {problem}')
    return pytest.importorskip('mujoco')


@pytest.mark.parametrize(
    'environ,platform,libraries,expected',
    [
        ({'MUJOCO_GL': 'egl', 'DISPLAY': ':0'}, 'linux', (), 'egl'),
        ({'MUJOCO_GL': 'osmesa'}, 'linux', (), 'osmesa'),
        ({'DISPLAY': ':1'}, 'linux', ('OSMesa',), 'glfw'),
        ({'WAYLAND_DISPLAY': 'wayland-0'}, 'linux', (), 'glfw'),
        ({}, 'linux', ('OSMesa', 'EGL'), 'osmesa'),
        ({}, 'linux', ('EGL',), 'egl'),
        ({}, 'linux', (), None),
        ({}, 'darwin', (), 'cgl'),
    ],
)
def test_choose_backend(environ, platform, libraries, expected):
    """MUJOCO_GL is kept; else the display, then a headless backend whose library exists."""
    found = {name: f'lib{name}.so.1' for name in libraries}
    assert gl.choose_backend(environ, platform, found.get) == expected


def test_no_backend_is_reported():
    """Without a backend, rendering is off with a reason that says what to install."""
    assert gl.unavailable_reason('osmesa') is None
    assert 'libosmesa6' in gl.unavailable_reason(None)


def test_ffmpeg_command():
    """The encoder writes H.264 yuv420p with faststart."""
    cmd = recorder.ffmpeg_command('out.mp4', 960, 540, 30)
    joined = ' '.join(cmd)
    assert '-s 960x540' in joined and '-r 30' in joined and '-pix_fmt yuv420p' in joined
    assert '-c:v libx264' in joined and '+faststart' in joined and cmd[-1] == 'out.mp4'


def test_parse_view():
    """A view is a YAML mapping of CameraRig settings, checked; empty is none."""
    assert recorder.parse_view('') == {} and recorder.parse_view(None) == {}
    view = recorder.parse_view('{lookat: [0.5, 0, 0.3], distance: 7, azimuth: 90, elevation: -50}')
    assert view == {
        'lookat': (0.5, 0.0, 0.3),
        'distance': 7.0,
        'azimuth': 90.0,
        'elevation': -50.0,
    }
    assert recorder.parse_view({'smoothing': 0, 'height': -0.1}) == {
        'smoothing': 0.0,
        'height': -0.1,
    }
    for bad in (
        '[1, 2]',
        '{zoom: 2}',
        '{lookat: [0, 0]}',
        '{lookat: 1}',
        '{distance: 0}',
        '{distance: far}',
        '{smoothing: -1}',
        '{azimuth: .nan}',
    ):
        with pytest.raises(ValueError):
            recorder.parse_view(bad)


def test_chase_camera_follows_with_smoothing(mujoco):
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
    # The follow camera keeps its own azimuth and elevation while it follows the body.
    follow = recorder.CameraRig('follow', body=1, azimuth=30.0, elevation=-40.0)
    cam = follow.update(data)
    assert cam.lookat == pytest.approx([1.0, 0.0, 0.2])
    assert (cam.azimuth, cam.elevation) == pytest.approx((30.0, -40.0))
    with pytest.raises(ValueError):
        recorder.CameraRig('orbit')


def test_record_mp4(tmp_path, mujoco):
    """A one-second drive records ten frames of H.264 video."""
    from vectorfield_mujoco import overlay, scene

    if shutil.which('ffmpeg') is None:
        pytest.skip('ffmpeg not installed')
    problem = gl.unavailable_reason(BACKEND) or gl.probe()
    if problem:
        pytest.skip(f'no OpenGL rendering: {problem}')
    world = scene.make_world(
        'scout_mini', obstacles=[{'type': 'box', 'pos': [1.5, 0.0], 'size': [0.2, 0.2, 0.2]}]
    )
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
