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

r"""A minimal ``follow_curve`` action client: build a Curve, send it, print feedback and result.

It is the starting point for your own client; ``send_curve.py`` is the full-featured one. The
curve is either a built-in shape (``--shape``, with ``--param NAME=VALUE``) or a polyline
(``--points``). The script does not drive to the curve first: the follower converges from
anywhere, along the shortest way. The defaults (an ellipse centred at (1.5, 0) with semi-axes
1.5 m and 1 m) pass through the start pose of vectorfield_bringup's demos.

Examples (with curve_follower_demo.launch.py running)::

    ros2 run vectorfield_bringup follow_curve_client.py
    ros2 run vectorfield_bringup follow_curve_client.py --shape lemniscate_bernoulli \
        --param a=1.5 --param cx=1.5 --laps 2
    ros2 run vectorfield_bringup follow_curve_client.py --points "0,0 2,0 2,1.5" --speed 0.3

The exit status is 0 when the goal succeeds and 1 otherwise.
"""

import argparse
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions

from vectorfield_msgs.action import FollowCurve
from vectorfield_msgs.msg import Curve

# FollowCurve.Result error codes by value, e.g. 1005: 'BLOCKED'.
ERROR_CODES = {
    value: name
    for name, value in vars(FollowCurve.Result).items()
    if name.isupper() and isinstance(value, int)
}
STATUS = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def parse_arguments(argv):
    """Parse the command line (ROS arguments already removed)."""
    parser = argparse.ArgumentParser(
        description='Send one curve to the follow_curve action and print its feedback and result.',
        epilog='The exit status is 0 when the goal succeeds and 1 otherwise.',
    )
    parser.add_argument(
        '--shape',
        default='ellipse',
        help='built-in shape (ellipse, lemniscate_bernoulli, lemniscate_gerono, superellipse, '
        'lissajous, rounded_rectangle, regular_polygon); default: ellipse',
    )
    parser.add_argument(
        '--param',
        action='append',
        default=[],
        metavar='NAME=VALUE',
        help='shape parameter, repeatable (cx, cy and yaw place every shape); '
        'default for the ellipse: a=1.5 b=1.0 cx=1.5',
    )
    parser.add_argument(
        '--points',
        help='an open polyline instead of a shape: "x,y x,y ..." (m), at least 2 points',
    )
    parser.add_argument('--closed', action='store_true', help='--points: the polyline is closed')
    parser.add_argument('--frame', default='map', help='curve frame (default: map)')
    parser.add_argument(
        '--laps', type=int, default=1, help='closed curves: laps to drive; 0 runs until Ctrl-C'
    )
    parser.add_argument(
        '--speed', type=float, default=0.0, help='desired speed (m/s); 0: the parameter'
    )
    parser.add_argument('--reverse', action='store_true', help='traverse the curve backwards')
    parser.add_argument(
        '--action', default='follow_curve', help='action name (default: follow_curve)'
    )
    return parser.parse_args(argv)


def build_curve(args):
    """Return the vectorfield_msgs/Curve described by the arguments."""
    curve = Curve()
    curve.header.frame_id = args.frame
    if args.points:
        curve.type = Curve.POLYLINE
        curve.closed = args.closed
        for pair in args.points.split():
            x, y = (float(value) for value in pair.split(','))
            curve.points.append(Point(x=x, y=y))
        return curve
    curve.type = Curve.BUILTIN
    curve.builtin_shape = args.shape
    parameters = {'a': 1.5, 'b': 1.0, 'cx': 1.5} if args.shape == 'ellipse' else {}
    for item in args.param:
        name, value = item.split('=', 1)
        parameters[name.strip()] = float(value)
    curve.parameter_names = list(parameters)
    curve.parameter_values = list(parameters.values())
    return curve


def main():
    """Send the goal, print the feedback and the result; return the exit status."""
    args = parse_arguments(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    # Without rclpy's signal handlers, Ctrl-C only raises KeyboardInterrupt and the node can
    # still cancel the goal.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('follow_curve_client')
    client = ActionClient(node, FollowCurve, args.action)

    goal = FollowCurve.Goal()
    goal.curve = build_curve(args)
    goal.frame_motion = FollowCurve.Goal.FRAME_STATIC
    goal.reverse_direction = args.reverse
    goal.desired_speed = args.speed
    goal.laps = 0 if args.points and not args.closed else max(args.laps, 0)

    def on_feedback(message):
        feedback = message.feedback
        node.get_logger().info(
            f'lap {feedback.lap}  distance {feedback.distance_to_curve:.3f} m  '
            f'remaining {feedback.remaining_arc_length:.2f} m  speed {feedback.speed:.2f} m/s',
            throttle_duration_sec=2.0,
        )

    status = None
    try:
        if not client.wait_for_server(timeout_sec=30.0):
            node.get_logger().error(f'no {args.action} action server')
            return 1
        sent = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if not handle.accepted:
            node.get_logger().error('the goal was rejected')
            return 1
        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, result_future)
        except KeyboardInterrupt:
            node.get_logger().info('cancelling')
            rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=5.0)
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
        if not result_future.done():
            return 1
        status = result_future.result().status
        result = result_future.result().result
        node.get_logger().info(
            f'{STATUS.get(status, status)}: '
            f'{ERROR_CODES.get(result.error_code, result.error_code)} ({result.error_code}) '
            f'{result.error_msg}; laps {result.laps_completed}, '
            f'{result.distance_travelled:.2f} m'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if status == GoalStatus.STATUS_SUCCEEDED else 1


if __name__ == '__main__':
    sys.exit(main())
