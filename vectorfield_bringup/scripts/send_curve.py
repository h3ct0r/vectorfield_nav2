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

r"""Send an example curve to the curve follower's ``follow_curve`` action and wait for its end.

The examples (``--shape``) are written from their math in ``vectorfield_bringup.curves``:
``ellipse``, ``lemniscate_bernoulli``, ``lemniscate_gerono``, ``superellipse``, the open
``sine``, a time-varying ``expression`` (override it with ``--x``, ``--y``, ``--s-min``,
``--s-max`` and ``--open``) and ``moving``, a circle in a moving frame.

The approach is client-side: the script finds where to join the curve with
``vectorfield.make_curve`` and ``closest_point`` (an open curve: its start), drives there with
Nav2's NavigateToPose (``BasicNavigator.goToPose``) facing along the tangent, then sends the
goal. A moving curve is sent without an approach. It prints the feedback every two seconds and
exits with 0 when the goal succeeds (or after Ctrl-C has cancelled it), 1 otherwise.

Examples (with curve_follower_demo.launch.py running)::

    ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
    ros2 run vectorfield_bringup send_curve.py --shape sine --speed 0.2 --reverse
    ros2 run vectorfield_bringup moving_frame_publisher.py --ros-args -p use_sim_time:=true \
        -p vx:=0.1 -p y:=1.5
    ros2 run vectorfield_bringup send_curve.py --shape moving --laps 0      # Ctrl-C to stop
    ros2 run vectorfield_bringup send_curve.py --shape moving --tf --laps 0
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from vectorfield_bringup import curves
from vectorfield_msgs.action import FollowCurve
from vectorfield_msgs.msg import FieldState

STATUS = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
ERRORS = {
    value: name
    for name, value in vars(FollowCurve.Result).items()
    if name.isupper() and isinstance(value, int)
}
PHASES = {
    value: name[len('PHASE_') :]
    for name, value in vars(FieldState).items()
    if name.startswith('PHASE_') and isinstance(value, int)
}
FEEDBACK_PERIOD = 2.0


def parse_arguments(argv):
    """Parse the command line (ROS arguments removed)."""
    parser = argparse.ArgumentParser(
        description='Send an example curve to follow_curve and wait for the result.'
    )
    parser.add_argument('--shape', choices=sorted(curves.EXAMPLES), default='ellipse')
    parser.add_argument(
        '--laps',
        type=int,
        default=1,
        help='closed curves: laps to drive (0: until Ctrl-C); ignored for open curves',
    )
    parser.add_argument(
        '--speed', type=float, default=0.0, help='desired speed (m/s); 0: the parameter'
    )
    parser.add_argument('--reverse', action='store_true', help='traverse the curve backwards')
    parser.add_argument(
        '--frame', default='', help='curve frame (default: map; moving: moving_frame)'
    )
    parser.add_argument(
        '--tf',
        action='store_true',
        help='moving frame: TF differencing (moving_frame_reference) instead of <frame>/odom',
    )
    parser.add_argument('--x', help='expression: x(s, t) (the constants a, b, c, w are defined)')
    parser.add_argument('--y', help='expression: y(s, t)')
    parser.add_argument('--s-min', type=float, help='expression: start of the domain')
    parser.add_argument('--s-max', type=float, help='expression: end of the domain')
    parser.add_argument('--open', action='store_true', help='expression: an open curve')
    parser.add_argument('--no-approach', action='store_true', help='skip the NavigateToPose')
    parser.add_argument('--base-frame', default='base_link', help='robot base frame')
    parser.add_argument(
        '--max-duration', type=float, default=0.0, help='goal time limit (s); 0: none'
    )
    return parser.parse_args(argv)


def description_of(args):
    """Return the curve of ``--shape``, with the expression overrides applied."""
    description = curves.EXAMPLES[args.shape]
    if args.shape != 'expression':
        return description
    parameters = dict(description.parameters)
    return curves.expression(
        args.x or description.x,
        args.y or description.y,
        description.s_min if args.s_min is None else args.s_min,
        description.s_max if args.s_max is None else args.s_max,
        closed=not args.open,
        **parameters,
    )


class CurveClient:
    """A node, spun in the background, with the TF listener and the follow_curve client."""

    def __init__(self):
        self.node = rclpy.create_node('send_curve')
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self.node)
        self.action = ActionClient(self.node, FollowCurve, 'follow_curve')
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self.feedback = None

    def close(self):
        """Stop spinning and destroy the node."""
        self._executor.shutdown()
        self._thread.join(timeout=5.0)
        self.node.destroy_node()

    def pose(self, frame, base_frame, timeout=10.0):
        """Return the robot position (x, y) in ``frame``, or None after ``timeout`` s."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                t = self.buffer.lookup_transform(frame, base_frame, Time())
                return t.transform.translation.x, t.transform.translation.y
            except TransformException:
                time.sleep(0.1)
        return None

    def on_feedback(self, message):
        """Keep the latest feedback."""
        self.feedback = message.feedback


def wait(future, timeout=None):
    """Wait for ``future``; return whether it is done."""
    deadline = None if timeout is None else time.monotonic() + timeout
    while not future.done():
        if deadline is not None and time.monotonic() > deadline:
            return False
        time.sleep(0.05)
    return True


def approach(description, frame, args, client, log):
    """Drive to the curve with NavigateToPose; return whether the robot got there."""
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    position = client.pose(frame, args.base_frame)
    if position is None:
        log(f'no transform {frame} -> {args.base_frame}; skipping the approach')
        return True
    x, y, yaw = curves.approach_pose(description, *position, reverse=args.reverse)
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    log(f'approach: NavigateToPose to ({x:.2f}, {y:.2f}, {yaw:.2f}) in {frame}')
    navigator = BasicNavigator('send_curve_navigator')
    try:
        # bt_navigator rejects goals until it is active; controller_server stands in for the
        # localizer, which differs between setups.
        navigator.waitUntilNav2Active(localizer='controller_server')
        navigator.goToPose(pose)
        while not navigator.isTaskComplete():
            time.sleep(0.1)
        result = navigator.getResult()
    except KeyboardInterrupt:
        navigator.cancelTask()
        raise
    finally:
        navigator.destroyNode()
    if result != TaskResult.SUCCEEDED:
        log(f'approach failed: {result}')
        return False
    return True


def main():
    """Approach, send the goal, report its feedback and result."""
    args = parse_arguments(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    # Without rclpy's signal handlers, Ctrl-C only raises KeyboardInterrupt: the context stays
    # valid, so the goal (or the approach) can still be cancelled.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    client = CurveClient()

    def log(text):
        client.node.get_logger().info(text)

    description = description_of(args)
    moving = args.shape in curves.MOVING_EXAMPLES
    frame = args.frame or ('moving_frame' if moving else 'map')
    code = 1
    try:
        if not client.action.wait_for_server(timeout_sec=30.0):
            log('no follow_curve action server (is curve_follower_demo running?)')
            return 1
        if moving or args.no_approach or approach(description, frame, args, client, log):
            code = follow(description, frame, moving, args, client, log)
    except KeyboardInterrupt:
        code = 1
    finally:
        client.close()
        if rclpy.ok():
            rclpy.shutdown()
    return code


def follow(description, frame, moving, args, client, log):
    """Send the FollowCurve goal and wait for its result; return the exit code."""
    goal = FollowCurve.Goal()
    goal.curve = curves.curve_msg(description, frame)
    goal.frame_motion = goal.FRAME_MOVING if moving else goal.FRAME_STATIC
    if moving and not args.tf:
        goal.frame_odom_topic = f'{frame}/odom'
    goal.reverse_direction = args.reverse
    goal.desired_speed = args.speed
    # laps > 0 needs a closed curve; an open one runs to its end.
    goal.laps = max(args.laps, 0) if description.closed else 0
    goal.max_duration.sec = int(args.max_duration)
    goal.max_duration.nanosec = int((args.max_duration % 1.0) * 1e9)
    source = (' (TF)' if args.tf else f' ({goal.frame_odom_topic})') if moving else ''
    log(f'following {args.shape} in {frame}{source}, laps {goal.laps}')

    sent = client.action.send_goal_async(goal, feedback_callback=client.on_feedback)
    if not wait(sent, 30.0) or not sent.result().accepted:
        log('the goal was not accepted')
        return 1
    handle = sent.result()
    result = handle.get_result_async()
    cancelled = False
    try:
        last = 0.0
        while not result.done():
            time.sleep(0.05)
            feedback = client.feedback
            if feedback is not None and time.monotonic() - last > FEEDBACK_PERIOD:
                last = time.monotonic()
                log(
                    f'lap {feedback.lap}  D {feedback.distance_to_curve:.3f} m  '
                    f's {feedback.arc_length:.2f} m  remaining '
                    f'{feedback.remaining_arc_length:.2f} m  v {feedback.speed:.2f} m/s  '
                    f'{PHASES.get(feedback.phase, feedback.phase)}'
                )
    except KeyboardInterrupt:
        cancelled = True
        log('cancelling')
        wait(handle.cancel_goal_async(), 10.0)
        wait(result, 10.0)
    if not result.done():
        return 1
    status = result.result().status
    outcome = result.result().result
    log(
        f'{STATUS.get(status, status)}: {ERRORS.get(outcome.error_code, outcome.error_code)} '
        f'{outcome.error_msg} (laps {outcome.laps_completed}, '
        f'{outcome.distance_travelled:.2f} m)'
    )
    succeeded = status == GoalStatus.STATUS_SUCCEEDED
    return 0 if succeeded or (cancelled and status == GoalStatus.STATUS_CANCELED) else 1


if __name__ == '__main__':
    sys.exit(main())
