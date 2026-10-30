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

"""The experimental 3D follower on point_sim_3d (follower3d_demo, headless), in name order.

1. an open helix (2 turns, 0.5 m/s): SUCCEEDED, settled near its end;
2. a Viviani curve, 2 laps: SUCCEEDED with 2 laps, max |D| after lap 1 below 0.05 m;
3. a sphere beside a circle, seen only as point_sim_3d's point cloud (voxelised by the follower,
   0.1 m voxels): 1 lap SUCCEEDED, and the true barrier h = |p - c| - r - (body_radius +
   cbf_margin) stays above -resolution;
4. a cancel mid-run: CANCELED, the commands end with exactly one zero, then silence.
"""

import math
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.shutdown import TestCleanShutdown  # noqa: F401

from vectorfield_bringup import curves3d
from vectorfield_msgs.action import FollowCurve3

SPEED = 0.5
LAP_LIMIT = 0.05
HELIX = curves3d.builtin('helix', (0.0, 0.0, 0.5), radius=1.0, lead=0.5, turns=2.0)
HELIX_END = (1.0, 0.0, 1.5)
VIVIANI = curves3d.builtin('viviani', (-0.6, 0.0, 1.2), radius=1.2)
CIRCLE = curves3d.builtin('circle', (0.0, 0.0, 1.0), radius=1.2)
# Beside the circle, outside it at 90 degrees: its surface is 0.05 m from the curve.
SPHERE = (0.0, 1.55, 1.0, 0.3)
BODY_RADIUS = 0.2  # follower3d_demo
CBF_MARGIN = 0.05  # the default
RESOLUTION = 0.1  # follower3d_demo's voxel_resolution
# Simulated seconds between the follower's last command and the client seeing the result.
SILENCE_SLACK = 0.5


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start follower3d_demo headless, the robot near the helix start."""
    import os

    import launch_testing.actions
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    path = os.path.join(
        get_package_share_directory('vectorfield_bringup'), 'launch', 'follower3d_demo.launch.py'
    )
    arguments = {'use_rviz': 'false', 'initial_x': '1.2', 'initial_y': '0.0', 'initial_z': '0.5'}
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path), launch_arguments=arguments.items()
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def goal_of(description, laps=0, speed=SPEED):
    """Return the FollowCurve3 goal of a curves3d description in the map frame."""
    goal = FollowCurve3.Goal()
    goal.curve = curves3d.curve3_msg(description, 'map')
    goal.frame_motion = FollowCurve3.Goal.FRAME_STATIC
    goal.desired_speed = speed
    goal.laps = laps
    return goal


def stamp_of(message):
    """Return a message's header stamp (s)."""
    return message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec


def length_of(description):
    """Return the curve length (vectorfield_py)."""
    from vectorfield import d3

    return d3.make_curve(curves3d.curve_spec3(description)).length()


class TestFollower3D(unittest.TestCase):
    """The four episodes, in name order on one robot."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for the follower."""
        from vectorfield_system_tests.follower_3d import Follower3DHarness

        cls.client = Follower3DHarness()
        cls.client.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.client.close()

    def _expect_silence(self, label, end, duration=2.0):
        """Nothing is published after the goal ended at ``end``, which was a single zero.

        The commands are judged by their stamps (the follower's clock), not by when they arrive,
        so a delivery delay at the client cannot pass for a command after the end. A follower
        still publishing would stamp a 20 Hz stream past ``end`` within ``duration``.
        """
        client = self.client
        client.sleep(duration)
        client.drain()
        stamped = sorted(
            ((stamp_of(m), m) for _, m in client.commands()), key=lambda item: item[0]
        )
        self.assertTrue(stamped, f'{label}: no command at all')
        late = [t for t, _ in stamped if t > end + SILENCE_SLACK]
        self.assertFalse(late, f'{label}: {len(late)} commands stamped after the goal ended')
        zeros = 0
        for _, message in reversed(stamped):
            t = message.twist
            if (t.linear.x, t.linear.y, t.linear.z, t.angular.z) != (0.0, 0.0, 0.0, 0.0):
                break
            zeros += 1
        self.assertEqual(zeros, 1, f'{label}: the commands must end with exactly one zero')

    def test_1_helix(self):
        """An open helix ends SUCCEEDED near its end."""
        client = self.client
        timeout = 2.0 * length_of(HELIX) / SPEED + 30.0
        episode = client.follow(goal_of(HELIX), timeout)
        print(f'helix: {episode.describe()}', flush=True)
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.describe())
        last = max(episode.samples, key=stamp_of).position
        error = math.dist((last.x, last.y, last.z), HELIX_END)
        self.assertLess(error, 0.15, f'helix: ended {error:.3f} m from its end')
        self._expect_silence('helix', episode.end)

    def test_2_viviani_two_laps(self):
        """Two laps of a Viviani curve; max |D| after lap 1 below 0.05 m."""
        client = self.client
        laps = 2
        timeout = 2.0 * laps * length_of(VIVIANI) / SPEED + 30.0
        episode = client.follow(goal_of(VIVIANI, laps=laps), timeout)
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.describe())
        self.assertEqual(episode.result.laps_completed, laps, episode.describe())
        after = [s.distance for s in episode.samples if s.lap >= 1]
        self.assertTrue(after, 'no field_state sample after lap 1')
        error = max(after)
        print(
            f'viviani: {episode.describe()}; max |D| after lap 1 {error:.4f} m '
            f'({len(after)} samples)',
            flush=True,
        )
        self.assertLess(error, LAP_LIMIT)
        self._expect_silence('viviani', episode.end)

    def test_3_sphere_beside_circle(self):
        """A voxelised sphere beside a circle is passed with h >= -resolution."""
        client = self.client
        client.set_spheres(SPHERE)
        try:
            # The cloud (2 Hz) reaches the follower before the goal.
            client.sleep(1.5)
            timeout = 2.0 * length_of(CIRCLE) / SPEED + 30.0
            episode = client.follow(goal_of(CIRCLE, laps=1), timeout)
        finally:
            client.set_spheres([])
        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.describe())
        centre, radius = SPHERE[:3], SPHERE[3]
        barriers = [
            math.dist((s.position.x, s.position.y, s.position.z), centre)
            - radius
            - (BODY_RADIUS + CBF_MARGIN)
            for s in episode.samples
        ]
        active = sum(1 for s in episode.samples if s.cbf_active)
        least = min(barriers)
        print(
            f'sphere: {episode.describe()}; least true barrier {least:.4f} m, CBF active in '
            f'{active} cycles',
            flush=True,
        )
        self.assertGreater(active, 0, 'the CBF filter never acted: was the cloud seen?')
        self.assertGreaterEqual(least, -RESOLUTION)
        self._expect_silence('sphere', episode.end)

    def test_4_cancel(self):
        """A cancel mid-run ends CANCELED, with one zero and then silence."""
        client = self.client
        handle, result, start = client.send(goal_of(CIRCLE, laps=0))
        client.sleep(6.0)
        moving = client.samples_since(start)
        self.assertTrue(moving, 'no field_state while the goal ran')
        client.cancel(handle, result)
        self.assertTrue(result.done(), 'no result after the cancel')
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        self._expect_silence('cancel', client.now(), duration=3.0)
