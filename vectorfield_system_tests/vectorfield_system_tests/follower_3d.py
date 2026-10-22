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

"""The client of the 3D follower's launch tests: follower3d_demo on point_sim_3d, no Nav2.

:class:`Follower3DHarness` spins one node in the background. It records ``follower_3d``'s
``~/field_state`` (FieldState3) and every ``cmd_vel`` message, sends FollowCurve3 goals and sets
point_sim_3d's ``obstacle_spheres``. Waits are measured in simulated time, bounded in wall time.
"""

import threading
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from vectorfield_msgs.action import FollowCurve3
from vectorfield_msgs.msg import FieldState3

#: Wall seconds allowed per simulated second, plus a grace, before a wait gives up.
WALL_PER_SIM = 2.0
WALL_GRACE = 15.0


@dataclass
class Episode3:
    """A finished FollowCurve3 goal and what was recorded while it ran."""

    status: int
    result: object
    start: float
    end: float
    samples: list

    def describe(self):
        """Return a one-line summary."""
        code = self.result.error_code if self.result is not None else None
        message = self.result.error_msg if self.result is not None else ''
        laps = self.result.laps_completed if self.result is not None else None
        return (
            f'status {self.status}, error {code} {message!r}, laps {laps}, '
            f'{self.end - self.start:.1f} s, {len(self.samples)} samples'
        )


class Follower3DHarness:
    """The test client (see the module docstring)."""

    def __init__(self, follower='/follower_3d', sim='/point_sim_3d'):
        self._follower = follower
        self._sim = sim
        self._lock = threading.Lock()
        self._samples = []
        self._commands = []
        self._context = rclpy.Context()
        rclpy.init(context=self._context)
        self.node = rclpy.create_node(
            'vf_follower_3d_test',
            context=self._context,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        history = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(
            FieldState3, f'{follower}/field_state', self._on_field, history
        )
        self.node.create_subscription(TwistStamped, '/cmd_vel', self._on_command, history)
        self.action = ActionClient(self.node, FollowCurve3, 'follow_curve_3d')
        self._get_state = self.node.create_client(GetState, f'{follower}/get_state')
        self._set_sim = self.node.create_client(SetParameters, f'{sim}/set_parameters')
        self._executor = MultiThreadedExecutor(num_threads=4, context=self._context)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def close(self):
        """Stop spinning and destroy the node."""
        self._executor.shutdown(timeout_sec=10.0)
        self._thread.join(timeout=10.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self._context)

    # Recording -----------------------------------------------------------------------------------

    def _on_field(self, message):
        with self._lock:
            self._samples.append(message)

    def _on_command(self, message):
        with self._lock:
            self._commands.append((self.now(), message))

    def samples_since(self, start):
        """Return the FieldState3 samples stamped at or after ``start`` (simulated s)."""
        with self._lock:
            return [
                s
                for s in self._samples
                if s.header.stamp.sec + 1e-9 * s.header.stamp.nanosec >= start
            ]

    def commands(self):
        """Return every (receive time, TwistStamped) so far."""
        with self._lock:
            return list(self._commands)

    # Time ----------------------------------------------------------------------------------------

    def now(self):
        """Return the simulated time (s)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def wait_until(self, predicate, timeout):
        """Wait until ``predicate()``, for at most ``timeout`` simulated (and bounded wall) s."""
        start = self.now()
        wall_deadline = time.monotonic() + WALL_PER_SIM * timeout + WALL_GRACE
        while not predicate():
            if self.now() - start > timeout or time.monotonic() > wall_deadline:
                return predicate()
            time.sleep(0.02)
        return True

    def sleep(self, duration):
        """Wait ``duration`` simulated seconds."""
        self.wait_until(lambda: False, duration)

    def drain(self, quiet=0.5, timeout=10.0):
        """Wait (wall time) until no FieldState3 or command has arrived for ``quiet`` s.

        The client can receive the streams later than an action result: call it after a goal
        ended, before judging what was recorded. Returns False if they never went quiet.
        """
        deadline = time.monotonic() + timeout
        last = None
        quiet_since = time.monotonic()
        while time.monotonic() < deadline:
            with self._lock:
                count = (len(self._samples), len(self._commands))
            if count != last:
                last = count
                quiet_since = time.monotonic()
            elif time.monotonic() - quiet_since >= quiet:
                return True
            time.sleep(0.05)
        return False

    def _wait_wall(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    # Graph ---------------------------------------------------------------------------------------

    def wait_until_active(self, timeout=120.0):
        """Wait (wall time) until the follower is active, its action server is up and time runs."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._get_state.wait_for_service(timeout_sec=1.0):
                future = self._get_state.call_async(GetState.Request())
                if self._wait_wall(future, 5.0) and future.result() is not None:
                    if future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                        break
            time.sleep(0.5)
        else:
            raise AssertionError('follower_3d did not become active')
        if not self.action.wait_for_server(timeout_sec=max(1.0, deadline - time.monotonic())):
            raise AssertionError('no follow_curve_3d action server')
        if not self._wait_wall_predicate(lambda: self.now() > 0.0, 30.0):
            raise AssertionError('no /clock')

    def _wait_wall_predicate(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def set_spheres(self, spheres):
        """Set point_sim_3d's ``obstacle_spheres`` (a flat x y z r list)."""
        if not self._set_sim.wait_for_service(timeout_sec=10.0):
            raise AssertionError('no point_sim_3d parameter service')
        request = SetParameters.Request()
        value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in spheres],
        )
        request.parameters = [ParameterMessage(name='obstacle_spheres', value=value)]
        future = self._set_sim.call_async(request)
        if not self._wait_wall(future, 10.0) or not future.result().results[0].successful:
            raise AssertionError('could not set obstacle_spheres')

    # Goals ---------------------------------------------------------------------------------------

    def send(self, goal):
        """Send a goal; return (handle, result future, start time) once it is accepted."""
        start = self.now()
        sent = self.action.send_goal_async(goal)
        if not self._wait_wall(sent, 30.0) or not sent.result().accepted:
            raise AssertionError('the FollowCurve3 goal was not accepted')
        handle = sent.result()
        return handle, handle.get_result_async(), start

    def finish(self, handle, result, start, timeout):
        """Wait for ``result`` (cancelled after ``timeout`` sim s); return an :class:`Episode3`."""
        if not self.wait_until(result.done, timeout):
            self.cancel(handle, result)
        end = self.now()
        self.drain()
        if not result.done():
            return Episode3(GoalStatus.STATUS_UNKNOWN, None, start, end, self.samples_since(start))
        wrapped = result.result()
        return Episode3(wrapped.status, wrapped.result, start, end, self.samples_since(start))

    def cancel(self, handle, result, timeout=20.0):
        """Cancel a goal and wait for its result."""
        self._wait_wall(handle.cancel_goal_async(), 10.0)
        self.wait_until(result.done, timeout)

    def follow(self, goal, timeout):
        """Send ``goal`` and return its :class:`Episode3`."""
        handle, result, start = self.send(goal)
        return self.finish(handle, result, start, timeout)
