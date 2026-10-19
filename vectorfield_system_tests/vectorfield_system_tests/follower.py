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

"""The curve follower's side of the launch tests: FollowCurve goals, episodes and moving frames.

:class:`FollowerHarness` extends :class:`NavHarness` with the ``follow_curve`` client and records
the follower's ``field_state`` instead of the controller's. Curves are built with
``vectorfield_bringup.curves``, and approached as send_curve.py does: NavigateToPose to the
joining pose of :func:`vectorfield_bringup.curves.approach_pose`.
"""

import math
import os
import subprocess
import time
from dataclasses import dataclass, field

import tf2_ros
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.time import Time

from vectorfield_bringup import curves
from vectorfield_msgs.action import FollowCurve
from vectorfield_system_tests.harness import Episode, Goal, NavHarness

FIELD_TOPIC = '/curve_follower/field_state'
FOLLOWER = 'curve_follower'
APPROACH_TIMEOUT = 90.0

ERROR_NAMES = {
    value: name
    for name, value in vars(FollowCurve.Result).items()
    if name.isupper() and isinstance(value, int)
}


@dataclass
class CurveGoal(Goal):
    """A FollowCurve goal in flight, with its feedback as (receive time, lap, D) tuples."""

    feedback: list = field(default_factory=list)


@dataclass
class CurveEpisode(Episode):
    """A FollowCurve goal from its start to its end, with its result and feedback."""

    result: object = None
    feedback: list = field(default_factory=list)

    @property
    def error_code(self):
        """Return the result's error code (None without a result)."""
        return None if self.result is None else self.result.error_code

    def describe(self):
        """Return the outcome as text, for failure messages."""
        if self.result is None:
            return f'{self.status_name}, no result after {self.end - self.start:.1f} s'
        return (
            f'{self.status_name} {ERROR_NAMES.get(self.error_code, self.error_code)} '
            f'"{self.result.error_msg}" after {self.end - self.start:.1f} s, '
            f'{self.result.laps_completed} laps, {self.result.distance_travelled:.2f} m'
        )

    def lap_end(self, lap):
        """Return when ``lap`` laps were complete (the first such feedback), inf if never."""
        return min((t for t, laps, _ in self.feedback if laps >= lap), default=math.inf)

    def max_distance(self, after):
        """Return max |D| over the field_state samples later than ``after`` (inf if none)."""
        return max((abs(s.distance) for s in self.samples if s.t > after), default=math.inf)

    def flagged(self, mask):
        """Return the field_state samples with any of the status bits of ``mask``."""
        return [s for s in self.samples if s.status & mask]


class MovingFrame:
    """moving_frame_publisher.py in its own process, on simulated time, still until :meth:`move`.

    A new DDS participant can take a few seconds to be discovered by every other process (Fast
    DDS announces it again every 3 s), so the frame is started before the robot's approach and
    set moving only once the follower can see it.

    Args:
        pose: The child's pose (x, y, yaw) in ``parent``.
        child: The moving frame (its Odometry is ``<child>/odom``).
        parent: Its parent frame.

    """

    NODE = 'moving_frame_publisher'
    DISCOVERY_TIME = 4.0

    def __init__(self, pose, child='moving_frame', parent='odom'):
        from ament_index_python.packages import get_package_prefix

        executable = os.path.join(
            get_package_prefix('vectorfield_bringup'),
            'lib',
            'vectorfield_bringup',
            'moving_frame_publisher.py',
        )
        values = dict(zip(('x', 'y', 'yaw'), pose))
        arguments = [executable, '--ros-args', '-p', 'use_sim_time:=true']
        arguments += ['-p', f'parent_frame:={parent}', '-p', f'child_frame:={child}']
        for name, value in values.items():
            arguments += ['-p', f'{name}:={float(value):.6f}']
        self.child = child
        self.parent = parent
        self.started = time.monotonic()
        self.process = subprocess.Popen(arguments)

    def move(self, nav, twist):
        """Set the constant twist (vx, vy, wz), at least DISCOVERY_TIME wall s after the start."""
        nav._wait_wall(lambda: time.monotonic() - self.started > self.DISCOVERY_TIME, 10.0)
        for name, value in zip(('vx', 'vy', 'wz'), twist):
            nav.set_parameter(self.NODE, name, value)

    def kill(self):
        """Kill the process (SIGKILL) and reap it."""
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=10.0)


class FollowerHarness(NavHarness):
    """The test client of the curve follower tests (curve_follower_demo)."""

    def __init__(self):
        super().__init__(field_topic=FIELD_TOPIC)

    def _start(self):
        super()._start()
        self._follow = ActionClient(self.node, FollowCurve, 'follow_curve')

    def wait_until_active(self, timeout=180.0):
        """Wait for Nav2 (:meth:`NavHarness.wait_until_active`), then for the active follower."""
        deadline = time.monotonic() + timeout
        super().wait_until_active(timeout)
        client = self.node.create_client(GetState, f'/{FOLLOWER}/get_state')
        active = False
        while not active and time.monotonic() < deadline:
            response = self._call(client, GetState.Request())
            active = response is not None and (
                response.current_state.id == State.PRIMARY_STATE_ACTIVE
            )
            if not active:
                time.sleep(0.5)
        self.node.destroy_client(client)
        if not active:
            raise AssertionError(f'{FOLLOWER} is not active after {timeout:.0f} s')
        if not self._follow.wait_for_server(timeout_sec=30.0):
            raise AssertionError('no follow_curve action server')

    def approach(self, description, reverse=False):
        """Drive to the joining pose of a map-frame curve with NavigateToPose; return it."""
        start = self.pose()
        x, y, yaw = curves.approach_pose(description, start.x, start.y, reverse)
        episode = self.navigate(x, y, yaw, timeout=APPROACH_TIMEOUT)
        if episode.status != GoalStatus.STATUS_SUCCEEDED:
            raise AssertionError(f'approach to ({x:.2f}, {y:.2f}): {episode.status_name}')
        # Let the follower's interlock see the terminal NavigateToPose and FollowPath statuses.
        self.sleep(1.0)
        return x, y, yaw

    def send_curve(self, goal):
        """Send a FollowCurve goal and wait until it is accepted (it always is)."""
        feedback = []

        def on_feedback(message):
            sample = (self.now(), message.feedback.lap, message.feedback.distance_to_curve)
            with self._lock:
                feedback.append(sample)

        start = self.now()
        future = self._follow.send_goal_async(goal, feedback_callback=on_feedback)
        if not self._wait_wall(future.done, 30.0):
            raise AssertionError('FollowCurve got no response')
        handle = future.result()
        if not handle.accepted:
            raise AssertionError('FollowCurve was rejected')
        return CurveGoal(handle, handle.get_result_async(), start, feedback)

    def follow(self, goal, timeout):
        """Send a goal and return its :class:`CurveEpisode`; cancelled after ``timeout`` sim s.

        A goal that has to be cancelled is reported as UNKNOWN (timed out).
        """
        handle = self.send_curve(goal)
        status = self.wait_result(handle, timeout)
        if status is None:
            self.cancel(handle)
            status = GoalStatus.STATUS_UNKNOWN
        return self.curve_episode(handle, status)

    def run_for(self, handle, duration):
        """Let a goal run ``duration`` sim s, cancel it if it has not ended; return its episode."""
        status = self.wait_result(handle, duration)
        if status is None:
            status = self.cancel(handle)
        return self.curve_episode(handle, GoalStatus.STATUS_UNKNOWN if status is None else status)

    def curve_episode(self, handle, status):
        """Return the :class:`CurveEpisode` of a goal from its start until now."""
        result = handle.result.result().result if handle.result.done() else None
        episode = self.episode(handle.start, status)
        with self._lock:
            feedback = list(handle.feedback)
        return CurveEpisode(
            episode.status,
            episode.start,
            episode.end,
            episode.samples,
            episode.poses,
            result,
            feedback,
        )

    def odom_pose(self, x, y, yaw):
        """Return the map pose (x, y, yaw) expressed in odom."""
        transform = self.tf_buffer.lookup_transform('odom', 'map', Time())
        t = transform.transform
        theta = 2.0 * math.atan2(t.rotation.z, t.rotation.w)
        c, s = math.cos(theta), math.sin(theta)
        return (
            t.translation.x + c * x - s * y,
            t.translation.y + s * x + c * y,
            theta + yaw,
        )

    def wait_for_frame(self, frame, timeout=10.0):
        """Wait for a :class:`MovingFrame`'s transforms; return whether they come.

        The TF buffer may still hold an earlier publisher's transforms of the same frame, so only
        a transform stamped after this call counts.
        """
        started = self.now()

        def fresh():
            try:
                transform = self.tf_buffer.lookup_transform(frame.parent, frame.child, Time())
            except tf2_ros.TransformException:
                return False
            stamp = transform.header.stamp
            return stamp.sec + stamp.nanosec * 1e-9 > started

        return self._wait_wall(fresh, timeout)


def curve_goal(description, laps=0, speed=0.0, frame='', moving=False, frame_odom_topic=''):
    """Return a FollowCurve goal for a curve of :mod:`vectorfield_bringup.curves`."""
    goal = FollowCurve.Goal()
    goal.curve = curves.curve_msg(description, frame)
    goal.frame_motion = goal.FRAME_MOVING if moving else goal.FRAME_STATIC
    goal.frame_odom_topic = frame_odom_topic
    goal.desired_speed = float(speed)
    goal.laps = laps
    return goal
