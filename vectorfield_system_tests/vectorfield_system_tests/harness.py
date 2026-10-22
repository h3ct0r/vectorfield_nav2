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

"""An rclpy client for the launch tests: waits for Nav2, sends goals and records what they check.

The client spins its own node (``use_sim_time``) in a background executor. Every wait is bounded
in simulated time and, should the clock stall, in wall time too (:data:`WALL_PER_SIM`,
:data:`WALL_GRACE`). Nav2 readiness is checked with our own lifecycle ``get_state`` calls, each
with a timeout and retried, rather than ``BasicNavigator.waitUntilNav2Active``, which waits forever
for a dropped response.

Service clients are created once and kept until :meth:`NavHarness.close`: on Humble, destroying a
client while the executor holds it in a ready wait set raises ``InvalidHandle`` in the spin thread
(rclpy ``executors.py``, ``_take_client``; Jazzy and later catch it), which then stops spinning, so
every later wait times out. Should the spin thread stop anyway, the waits raise its exception.

/clock keeps only its latest message while the pose subscription queues them, so under load the
latest received pose can be older than :meth:`NavHarness.now`. Positions checked at a given time
are read by stamp (:meth:`NavHarness.pose_at`), not as the latest message.
"""

import faulthandler
import math
import os
import threading
import time
from dataclasses import dataclass

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
from nav2_msgs.srv import ClearEntireCostmap, SetInitialPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import UInt64

from vectorfield_msgs.msg import FieldState
from vectorfield_system_tests.geometry import OccupiedCells

#: FieldState status bits for which the controller raises NoValidControl.
NO_VALID_CONTROL = (
    FieldState.STATUS_NON_FINITE
    | FieldState.STATUS_CONTOUR_WATCHDOG
    | FieldState.STATUS_HEADING_INFEASIBLE
    | FieldState.STATUS_BLOCKED
)

#: Phases in which the robot tracks the path, so that the distance D is meaningful.
TRACKING_PHASES = (FieldState.PHASE_FOLLOW, FieldState.PHASE_FINAL_APPROACH)

#: Wall seconds allowed per simulated second, plus a fixed grace, before a wait gives up on a
#: stalled clock.
WALL_PER_SIM = 2.0
WALL_GRACE = 15.0

#: The lifecycle nodes that must be active before a test sends goals (every demo has them).
NAV2_NODES = (
    'map_server',
    'controller_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
)

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN (timed out)',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


@dataclass(frozen=True)
class FieldSample:
    """The parts of one ``field_state`` message the tests use."""

    t: float
    phase: int
    status: int
    distance: float
    contour_direction: int


@dataclass(frozen=True)
class PoseSample:
    """A true map pose of base_footprint."""

    t: float
    x: float
    y: float
    yaw: float


@dataclass
class Goal:
    """A NavigateToPose goal in flight."""

    handle: object
    result: object
    start: float


@dataclass
class Episode:
    """What happened between sending a goal and its end (or the end of the wait)."""

    status: int
    start: float
    end: float
    samples: list
    poses: list

    @property
    def status_name(self):
        """Return the goal status as text."""
        return STATUS_NAMES.get(self.status, str(self.status))

    def contour_activations(self):
        """Return how many times the contour engaged (contour_direction became non-zero)."""
        count, active = 0, False
        for sample in self.samples:
            if sample.contour_direction != 0 and not active:
                count += 1
            active = sample.contour_direction != 0
        return count

    def no_valid_control(self):
        """Return the samples with a status bit for which the controller raises NoValidControl."""
        return [sample for sample in self.samples if sample.status & NO_VALID_CONTROL]

    def tracking_error(self, converged=0.05, after=-math.inf):
        """Return max |D| over the tracking samples from the first one with |D| <= ``converged``.

        Only samples later than ``after`` count. Returns inf if |D| never got that small.
        """
        tracking = [s for s in self.samples if s.phase in TRACKING_PHASES and s.t > after]
        for index, sample in enumerate(tracking):
            if abs(sample.distance) <= converged:
                return max(abs(s.distance) for s in tracking[index:])
        return math.inf

    def min_distance(self, distance):
        """Return the minimum of ``distance(x, y)`` over the true poses (inf without poses)."""
        return min((distance(p.x, p.y) for p in self.poses), default=math.inf)


def yaw_of(quaternion):
    """Return the yaw of a planar quaternion."""
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


class NavHarness:
    """The test client: one node, spun in a background executor.

    Args:
        ground_truth: Read the true pose and the infeasible-command count from vectorfield_sim.
            Without it (Nav2's loopback simulator, which localizes perfectly), the true pose is
            map -> base_footprint from TF.
        field_topic: The FieldState topic recorded in the episodes (the controller's by default).

    """

    def __init__(self, ground_truth=True, field_topic='/FollowPath/field_state'):
        self._ground_truth = ground_truth
        self._field_topic = field_topic
        self._lock = threading.Lock()
        self._samples = []
        self._poses = []
        self._plans = []
        self._failures = []
        self._planner_requests = []
        self._infeasible = None
        self.map = None
        self._abandoned = []
        self._start()

    def _start(self):
        """Create the context, the node, its subscriptions and clients, and start spinning."""
        self._context = rclpy.Context()
        rclpy.init(context=self._context)
        self.node = rclpy.create_node(
            'vf_system_test',
            context=self._context,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        node = self.node
        history = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(FieldState, self._field_topic, self._on_field, history)
        node.create_subscription(Path, '/plan', self._on_plan, history)
        node.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._on_bt_log, history)
        node.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self._clients = {}
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        if self._ground_truth:
            node.create_subscription(
                Odometry, '/vectorfield_sim/ground_truth', self._on_ground_truth, history
            )
            node.create_subscription(
                UInt64, '/vectorfield_sim/infeasible_command_count', self._on_infeasible, latched
            )
        else:
            node.create_timer(0.05, self._sample_tf)
        self._navigate = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._navigate_through = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')

        self._executor = MultiThreadedExecutor(num_threads=4, context=self._context)
        self._executor.add_node(node)
        self._spin_error = None
        self._closed = False
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        """Spin the executor; keep the exception that stops it, for the waits to raise."""
        try:
            self._executor.spin()
        except Exception as error:
            self._spin_error = error
            raise

    def _check_spinning(self):
        if self._spin_error is not None:
            raise AssertionError(f'the test client stopped spinning: {self._spin_error!r}')

    def _client(self, service_type, name):
        """Return the client of service ``name``, created on first use and kept until close."""
        key = (service_type, name)
        if key not in self._clients:
            self._clients[key] = self.node.create_client(service_type, name)
        return self._clients[key]

    def _restart(self):
        """Start over with a new node and participant; the old ones are only stopped spinning.

        Destroying the node of a participant that discovers nothing can block for good, so it is
        kept, unused, until the process exits. The one known cause, Fast DDS 3's TypeLookup
        deadlock between this participant and launch_ros's (README, Known issues; the launch
        tests' Fast DDS profile prevents it), also freezes launch's shutdown, so starting over
        does not save that test; :meth:`close` then arms a stack dump that shows it.
        """
        self._executor.shutdown(timeout_sec=5.0)
        self._thread.join(timeout=5.0)
        self._abandoned.append((self._executor, self.node, self._context))
        self._start()

    def close(self):
        """Stop spinning and destroy the node (once)."""
        if self._closed:
            return
        self._closed = True
        self._arm_freeze_dump()
        self._executor.shutdown(timeout_sec=10.0)
        self._thread.join(timeout=10.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self._context)

    def _arm_freeze_dump(self):
        """After starting over, dump the thread stacks if the process still runs 60 s after close.

        Such processes have frozen in launch's shutdown (Lyrical; see :meth:`_restart`).
        """
        if self._closed and self._abandoned:
            faulthandler.dump_traceback_later(60.0)

    # Callbacks -----------------------------------------------------------------------------------

    def _on_field(self, message):
        sample = FieldSample(
            _seconds(message.header.stamp),
            message.phase,
            message.status,
            message.distance,
            message.contour_direction,
        )
        with self._lock:
            self._samples.append(sample)

    def _on_plan(self, _message):
        with self._lock:
            self._plans.append(self.now())

    def _on_bt_log(self, message):
        # The FollowPath action's status cannot tell a failure from a preemption (Nav2 aborts the
        # goal a new path replaces); bt_navigator's FollowPath node fails only on a real failure.
        # Event stamps are not simulated time on every distro: record the receive time.
        # ComputePathToPose going RUNNING is a planner request, whether or not a path is found.
        failed = any(
            event.node_name == 'FollowPath' and event.current_status == 'FAILURE'
            for event in message.event_log
        )
        planning = any(
            event.node_name == 'ComputePathToPose' and event.current_status == 'RUNNING'
            for event in message.event_log
        )
        if failed or planning:
            now = self.now()
            with self._lock:
                if failed:
                    self._failures.append(now)
                if planning:
                    self._planner_requests.append(now)

    def _on_map(self, message):
        self.map = OccupiedCells(message)

    def _on_ground_truth(self, message):
        pose = message.pose.pose
        sample = PoseSample(
            _seconds(message.header.stamp),
            pose.position.x,
            pose.position.y,
            yaw_of(pose.orientation),
        )
        with self._lock:
            self._poses.append(sample)

    def _on_infeasible(self, message):
        self._infeasible = message.data

    def _sample_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except tf2_ros.TransformException:
            return
        sample = PoseSample(
            _seconds(transform.header.stamp),
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw_of(transform.transform.rotation),
        )
        with self._lock:
            if not self._poses or self._poses[-1].t < sample.t:
                self._poses.append(sample)

    # Time and waiting ----------------------------------------------------------------------------

    def now(self):
        """Return the simulated time (s)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def wait_until(self, predicate, timeout):
        """Wait until ``predicate()`` is true, for at most ``timeout`` simulated seconds.

        Returns:
            Whether the predicate became true.

        """
        start = self.now()
        wall_deadline = time.monotonic() + WALL_PER_SIM * timeout + WALL_GRACE
        while not predicate():
            self._check_spinning()
            if self.now() - start > timeout or time.monotonic() > wall_deadline:
                return predicate()
            time.sleep(0.02)
        return True

    def sleep(self, duration):
        """Wait ``duration`` simulated seconds."""
        self.wait_until(lambda: False, duration)

    def _wait_wall(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            self._check_spinning()
            if time.monotonic() > deadline:
                return predicate()
            time.sleep(0.02)
        return True

    def _call(self, client, request, timeout=5.0):
        """Call a service; return the response, or None if none arrived in ``timeout`` wall s."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        if not self._wait_wall(future.done, timeout):
            client.remove_pending_request(future)
            return None
        return future.result()

    def wait_until_active(self, timeout=180.0):
        """Wait until /clock runs, every :data:`NAV2_NODES` node is active and the robot is placed.

        On failure the client is closed (``tearDownClass`` does not run after a failed
        ``setUpClass``).

        Raises:
            AssertionError: If something is not ready within ``timeout`` wall seconds.

        """
        timeout *= float(os.environ.get('VF_TEST_TIME_SCALE', '1'))
        # Should start-up hang anyway, the thread stacks show where (in the test log).
        faulthandler.dump_traceback_later(timeout + 60.0)
        try:
            self._wait_until_active(timeout)
        except BaseException:
            self.close()
            raise
        finally:
            faulthandler.cancel_dump_traceback_later()
            self._arm_freeze_dump()

    def _wait_until_active(self, timeout):
        deadline = time.monotonic() + timeout

        def remaining():
            return max(deadline - time.monotonic(), 0.0)

        # Rarely (Lyrical without the launch tests' Fast DDS profile: README, Known issues), the
        # new participant discovers nothing, or gets /clock but no service answer.
        first = NAV2_NODES[0]
        for attempt in range(3):
            clock = self._wait_wall(lambda: self.now() > 0.0, min(40.0, remaining()))
            if (
                clock
                and self._get_state(first, min(40.0, remaining()), lambda _: True) is not None
            ):
                break
            if not clock:
                problem = (
                    f'no /clock ({self.node.count_publishers("/clock")} publishers discovered)'
                )
            else:
                ready = self._client(GetState, f'/{first}/get_state').service_is_ready()
                problem = f'{first} never answered get_state (service ready: {ready})'
            if attempt == 2 or remaining() <= 0.0:
                raise AssertionError(problem)
            print(f'{problem}; new node', flush=True)
            self._restart()
        for name in NAV2_NODES:
            if not self.wait_lifecycle_active(name, remaining()):
                raise AssertionError(f'{name} is not active after {timeout:.0f} s')
        if not self._wait_wall(
            lambda: self.tf_buffer.can_transform('map', 'base_scan', Time()),
            min(30.0, remaining()),
        ):
            raise AssertionError('no map -> base_scan transform')
        if not self._navigate.wait_for_server(timeout_sec=max(remaining(), 1.0)):
            raise AssertionError('no navigate_to_pose action server')
        if not self._wait_wall(lambda: self.map is not None and self.pose() is not None, 30.0):
            raise AssertionError('no /map or no true pose')

    def wait_lifecycle_active(self, name, timeout):
        """Return whether lifecycle node ``name`` reports active within ``timeout`` wall s."""
        active = State.PRIMARY_STATE_ACTIVE
        return self._get_state(name, timeout, lambda state: state == active) == active

    def _get_state(self, name, timeout, accept):
        """Call ``name``'s get_state until ``accept(state id)``; return the last state id.

        None if it never answered within ``timeout`` wall seconds. Each call has its own timeout,
        so a dropped response costs one retry.
        """
        client = self._client(GetState, f'/{name}/get_state')
        deadline = time.monotonic() + timeout
        state = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return state
            response = self._call(client, GetState.Request(), min(5.0, remaining))
            if response is not None:
                state = response.current_state.id
                if accept(state):
                    return state
            time.sleep(0.5)

    # Navigation ----------------------------------------------------------------------------------

    def _map_pose(self, x, y, yaw):
        """Return a PoseStamped in the map frame, stamped now."""
        stamped = PoseStamped()
        stamped.header.frame_id = 'map'
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        stamped.pose.position.x = float(x)
        stamped.pose.position.y = float(y)
        stamped.pose.orientation.z = math.sin(0.5 * yaw)
        stamped.pose.orientation.w = math.cos(0.5 * yaw)
        return stamped

    def _send(self, client, goal, label):
        """Send ``goal`` with ``client`` and wait until it is accepted."""
        start = self.now()
        future = client.send_goal_async(goal)
        if not self._wait_wall(future.done, 30.0):
            raise AssertionError(f'{label} got no response')
        handle = future.result()
        if not handle.accepted:
            raise AssertionError(f'{label} was rejected')
        return Goal(handle, handle.get_result_async(), start)

    def send_goal(self, x, y, yaw):
        """Send a NavigateToPose goal in the map frame and wait until it is accepted."""
        goal = NavigateToPose.Goal()
        goal.pose = self._map_pose(x, y, yaw)
        return self._send(self._navigate, goal, f'NavigateToPose ({x}, {y})')

    def send_goal_through(self, poses):
        """Send a NavigateThroughPoses goal through ``poses`` ((x, y, yaw), map frame)."""
        goal = NavigateThroughPoses.Goal()
        stamped = [self._map_pose(*pose) for pose in poses]
        if hasattr(goal.poses, 'goals'):  # Kilted and later: nav_msgs/Goals
            goal.poses.header = stamped[-1].header
            goal.poses.goals = stamped
        else:
            goal.poses = stamped
        return self._send(self._navigate_through, goal, f'NavigateThroughPoses {poses}')

    def wait_result(self, goal, timeout):
        """Return the goal's final status, or None if it has none after ``timeout`` sim s."""
        if not self.wait_until(goal.result.done, timeout):
            return None
        return goal.result.result().status

    def cancel(self, goal, timeout=30.0):
        """Cancel a goal and return its final status (None if it never ended)."""
        future = goal.handle.cancel_goal_async()
        self._wait_wall(future.done, 10.0)
        return self.wait_result(goal, timeout)

    def navigate(self, x, y, yaw, timeout=120.0):
        """Send a goal and return its :class:`Episode`; it is cancelled after ``timeout`` sim s."""
        return self._finish(self.send_goal(x, y, yaw), timeout)

    def navigate_through(self, poses, timeout=120.0):
        """Like :meth:`navigate`, with one NavigateThroughPoses goal through ``poses``."""
        return self._finish(self.send_goal_through(poses), timeout)

    def _finish(self, goal, timeout):
        """Wait for ``goal`` (cancelled after ``timeout`` sim s); return its :class:`Episode`."""
        status = self.wait_result(goal, timeout)
        if status is None:
            self.cancel(goal)
            status = GoalStatus.STATUS_UNKNOWN
        return self.episode(goal.start, status)

    def episode(self, start, status, end=None):
        """Return the :class:`Episode` from ``start`` to ``end`` (default: now).

        Waits (at most 10 wall s) until the true poses up to ``end`` have arrived.
        """
        end = self.now() if end is None else end
        self._wait_wall(lambda: self._latest_pose_stamp() >= end, 10.0)
        with self._lock:
            samples = [s for s in self._samples if start <= s.t <= end]
            poses = [p for p in self._poses if start <= p.t <= end]
        return Episode(status, start, end, samples, poses)

    # Observations --------------------------------------------------------------------------------

    def pose(self):
        """Return the latest received true pose (:class:`PoseSample`), or None.

        Under load it can lag :meth:`now`; use :meth:`pose_at` for the pose at a given time.
        """
        with self._lock:
            return self._poses[-1] if self._poses else None

    def _latest_pose_stamp(self):
        with self._lock:
            return self._poses[-1].t if self._poses else -math.inf

    def pose_at(self, t, timeout=10.0):
        """Return the first true pose stamped at or after ``t`` (sim s), waiting for it to arrive.

        Raises:
            AssertionError: If no such pose arrives within ``timeout`` wall seconds.

        """
        lag = t - self._latest_pose_stamp()
        if not self._wait_wall(lambda: self._latest_pose_stamp() >= t, timeout):
            raise AssertionError(f'no true pose stamped at or after t = {t:.2f} s: {self.pose()}')
        if lag > 0.1:
            print(f'the true pose lagged t = {t:.2f} s by {lag:.2f} s', flush=True)
        with self._lock:
            index = len(self._poses) - 1
            while index > 0 and self._poses[index - 1].t >= t:
                index -= 1
            return self._poses[index]

    def settled_pose(self, settle):
        """Wait ``settle`` simulated seconds; return the true pose stamped at the end."""
        target = self.now() + settle
        self.sleep(settle)
        return self.pose_at(target)

    def field_samples_since(self, start):
        """Return the field_state samples stamped at or after ``start``."""
        with self._lock:
            return [s for s in self._samples if s.t >= start]

    def plans_since(self, start):
        """Return the receive times of the /plan messages since ``start``."""
        with self._lock:
            return [t for t in self._plans if t >= start]

    def follow_path_failures_since(self, start):
        """Return the times at which bt_navigator's FollowPath node failed, since ``start``."""
        with self._lock:
            return sorted(t for t in self._failures if t >= start)

    def planner_requests_since(self, start):
        """Return the times bt_navigator's ComputePathToPose node started, since ``start``."""
        with self._lock:
            return sorted(t for t in self._planner_requests if t >= start)

    def infeasible_command_count(self, timeout=10.0):
        """Return vectorfield_sim's infeasible-command count (None if it never arrived)."""
        self._wait_wall(lambda: self._infeasible is not None, timeout)
        return self._infeasible

    def final_command_publishers(self, topic, expected, timeout=15.0):
        """Return the nodes that publish ``topic``, waiting until ``expected`` is among them."""

        def names():
            return sorted(
                {info.node_name for info in self.node.get_publishers_info_by_topic(topic)}
            )

        self._wait_wall(lambda: expected in names(), timeout)
        return names()

    # vectorfield_sim and Nav2 services -----------------------------------------------------------

    def set_parameter(self, node_name, name, value):
        """Set a parameter of a node (a bool, string, float or list of floats)."""
        if isinstance(value, bool):
            message = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, str):
            message = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
        elif isinstance(value, (list, tuple)):
            message = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(v) for v in value],
            )
        else:
            message = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
            )
        request = SetParameters.Request(parameters=[ParameterMessage(name=name, value=message)])
        response = self._call(self._client(SetParameters, f'/{node_name}/set_parameters'), request)
        if response is None or not response.results[0].successful:
            raise AssertionError(f'{node_name} rejected {name} = {value}: {response}')

    def set_boxes(self, boxes):
        """Set vectorfield_sim's unmapped ``box_obstacles`` (a list of [x, y, sx, sy, yaw])."""
        self.set_parameter('vectorfield_sim', 'box_obstacles', [v for box in boxes for v in box])

    def step_map_to_odom(self, x, y, yaw):
        """Add (x, y, yaw) to vectorfield_sim's published map -> odom (a localization jump)."""
        request = SetInitialPose.Request()
        request.pose.header.frame_id = 'map'
        request.pose.pose.pose.position.x = float(x)
        request.pose.pose.pose.position.y = float(y)
        request.pose.pose.pose.orientation.z = math.sin(0.5 * yaw)
        request.pose.pose.pose.orientation.w = math.cos(0.5 * yaw)
        client = self._client(SetInitialPose, '/vectorfield_sim/step_map_to_odom')
        response = self._call(client, request)
        if response is None:
            raise AssertionError('vectorfield_sim did not answer step_map_to_odom')

    def clear_costmaps(self):
        """Clear both costmaps entirely."""
        for costmap in ('local_costmap', 'global_costmap'):
            client = self._client(ClearEntireCostmap, f'/{costmap}/clear_entirely_{costmap}')
            response = self._call(client, ClearEntireCostmap.Request())
            if response is None:
                raise AssertionError(f'{costmap} was not cleared')


def _seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
