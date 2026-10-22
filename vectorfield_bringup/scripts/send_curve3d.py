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

r"""Send an example 3D curve to the 3D follower's ``follow_curve_3d`` action and wait for its end.

The examples (``--shape``) are in ``vectorfield_bringup.curves3d``: the open ``helix``,
``viviani``, ``torus_knot``, ``lemniscate3d``, a time-varying ``expression`` (override it with
``--x``, ``--y``, ``--z``, ``--s-min``, ``--s-max`` and ``--open``) and ``moving``, a circle in
point_sim_3d's moving frame (start the demo with ``moving_frame:=platform``).

There is no approach phase: a multirotor flies straight to the curve along the field. The script
prints the feedback every two seconds and exits with 0 when the goal succeeds (or after Ctrl-C
has cancelled it), 1 otherwise.

Examples (with follower3d_demo.launch.py running)::

    ros2 run vectorfield_bringup send_curve3d.py --shape helix
    ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2 --speed 0.6
    ros2 run vectorfield_bringup send_curve3d.py --shape torus_knot --laps 1
    ros2 run vectorfield_bringup send_curve3d.py --shape expression --laps 0     # Ctrl-C to stop
    ros2 run vectorfield_bringup send_curve3d.py --shape moving --laps 0
"""

import argparse
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from vectorfield_bringup import curves3d
from vectorfield_msgs.action import FollowCurve3
from vectorfield_msgs.msg import FieldState3

STATUS = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
ERRORS = {
    value: name
    for name, value in vars(FollowCurve3.Result).items()
    if name.isupper() and isinstance(value, int)
}
PHASES = {
    value: name[len('PHASE_') :]
    for name, value in vars(FieldState3).items()
    if name.startswith('PHASE_') and isinstance(value, int)
}
FEEDBACK_PERIOD = 2.0


def parse_arguments(argv):
    """Parse the command line (ROS arguments removed)."""
    parser = argparse.ArgumentParser(
        description='Send an example 3D curve to follow_curve_3d and wait for the result.'
    )
    parser.add_argument(
        '--shape', default='helix', choices=sorted(curves3d.EXAMPLES), help='example curve'
    )
    parser.add_argument('--x', help='expression: x(s, t)')
    parser.add_argument('--y', help='expression: y(s, t)')
    parser.add_argument('--z', help='expression: z(s, t)')
    parser.add_argument('--s-min', type=float, default=None, help='expression: domain start')
    parser.add_argument('--s-max', type=float, default=None, help='expression: domain end')
    parser.add_argument('--open', action='store_true', help='expression: an open curve')
    parser.add_argument('--laps', type=int, default=1, help='closed curves; 0 = until Ctrl-C')
    parser.add_argument('--speed', type=float, default=0.0, help='m/s; 0 = the parameter')
    parser.add_argument('--reverse', action='store_true', help='traverse backwards')
    parser.add_argument('--frame', default='', help='curve frame (default: map, or platform)')
    parser.add_argument('--tf', action='store_true', help='moving: TF differencing, no Odometry')
    parser.add_argument('--max-duration', type=float, default=0.0, help='s; 0 = unlimited')
    parser.add_argument(
        '--action', default='follow_curve_3d', help='action name (default follow_curve_3d)'
    )
    return parser.parse_args(argv)


def description_of(args):
    """Return the example, with the expression overrides applied."""
    description = curves3d.EXAMPLES[args.shape]
    if description.kind != 'expression':
        return description
    return curves3d.expression(
        args.x or description.x,
        args.y or description.y,
        args.z or description.z,
        description.s_min if args.s_min is None else args.s_min,
        description.s_max if args.s_max is None else args.s_max,
        closed=not args.open and description.closed,
        **dict(description.parameters),
    )


def wait(future, timeout=None):
    """Wait for ``future``; return whether it is done."""
    deadline = None if timeout is None else time.monotonic() + timeout
    while not future.done():
        if deadline is not None and time.monotonic() > deadline:
            return False
        time.sleep(0.05)
    return True


def main():
    """Send the goal, report its feedback and result."""
    args = parse_arguments(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    # Without rclpy's signal handlers, Ctrl-C only raises KeyboardInterrupt: the context stays
    # valid, so the goal can still be cancelled.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('send_curve3d')
    action = ActionClient(node, FollowCurve3, args.action)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    latest = {}

    def log(text):
        node.get_logger().info(text)

    code = 1
    try:
        if not action.wait_for_server(timeout_sec=30.0):
            log(f'no {args.action} action server (is follower3d_demo running?)')
            return 1
        code = follow(args, action, latest, log)
    except KeyboardInterrupt:
        code = 1
    finally:
        executor.shutdown()
        thread.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


def follow(args, action, latest, log):
    """Send the FollowCurve3 goal and wait for its result; return the exit code."""
    description = description_of(args)
    moving = args.shape in curves3d.MOVING_EXAMPLES
    frame = args.frame or ('platform' if moving else 'map')
    goal = FollowCurve3.Goal()
    goal.curve = curves3d.curve3_msg(description, frame)
    goal.frame_motion = goal.FRAME_MOVING if moving else goal.FRAME_STATIC
    if moving and not args.tf:
        goal.frame_odom_topic = f'{frame}/odom'
    goal.reverse_direction = args.reverse
    goal.desired_speed = args.speed
    # laps > 0 needs a closed curve; the helix (open) runs to its end.
    goal.laps = max(args.laps, 0) if args.shape != 'helix' and not args.open else 0
    goal.max_duration.sec = int(args.max_duration)
    goal.max_duration.nanosec = int((args.max_duration % 1.0) * 1e9)
    log(f'following {args.shape} in {frame}, laps {goal.laps}')

    def on_feedback(message):
        latest['feedback'] = message.feedback

    sent = action.send_goal_async(goal, feedback_callback=on_feedback)
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
            feedback = latest.get('feedback')
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
