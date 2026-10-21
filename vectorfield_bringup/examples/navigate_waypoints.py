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

"""Drive a route of poses with Nav2's simple commander, one NavigateToPose goal per pose.

Nav2 runs the route with whatever controller is configured as ``FollowPath``; with
vectorfield_bringup's demos that is the vector-field controller. Each pose is a separate
NavigateToPose goal, so the route also works with the Ackermann overlay, whose NavigateToPose
tree has no Spin (the stock NavigateThroughPoses trees use it). The default route is a loop in
the demos' ``open_10x10`` map.

Examples (with kinematic_sim_demo.launch.py running)::

    ros2 run vectorfield_bringup navigate_waypoints.py
    ros2 run vectorfield_bringup navigate_waypoints.py --poses "2,0,0 2,2,1.57 0,0,3.14"
    ros2 run vectorfield_bringup navigate_waypoints.py --relative --poses "3,0,0 0,0,3.14"

``--relative`` reads the poses in the robot's frame at the start (x ahead, y to the left, yaw from
its heading), as the field tests do (docs/tutorials/field_testing.md). ``--through`` sends the
route as one NavigateThroughPoses goal instead, so the plan has corners at the poses.

The exit status is 0 when every pose is reached and 1 otherwise.
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.signals import SignalHandlerOptions

DEFAULT_POSES = '2.5,0,0 2.5,2.5,1.57 -2.5,2.5,3.14 0,0,0'


def parse_arguments(argv):
    """Parse the command line (ROS arguments already removed)."""
    parser = argparse.ArgumentParser(
        description='Drive a route of poses with NavigateToPose, one goal per pose.',
        epilog='The exit status is 0 when every pose is reached and 1 otherwise.',
    )
    parser.add_argument(
        '--poses',
        default=DEFAULT_POSES,
        help=f'route as "x,y,yaw x,y,yaw ..." (m, rad); default: "{DEFAULT_POSES}"',
    )
    parser.add_argument('--frame', default='map', help='frame of the poses (default: map)')
    parser.add_argument(
        '--relative',
        action='store_true',
        help='poses relative to the robot pose at the start (x ahead, y left, yaw from heading)',
    )
    parser.add_argument(
        '--base-frame', default='base_link', help='robot frame for --relative (default: base_link)'
    )
    parser.add_argument(
        '--through',
        action='store_true',
        help='one NavigateThroughPoses goal through all poses instead of one goal per pose',
    )
    parser.add_argument(
        '--localizer',
        default='controller_server',
        help='node to wait for before the first goal: amcl with AMCL, controller_server '
        '(default) when the setup has no localizer node',
    )
    return parser.parse_args(argv)


def make_pose(navigator, frame, x, y, yaw):
    """Return a PoseStamped in ``frame`` at (x, y) with heading ``yaw``."""
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    return pose


def start_pose(navigator, frame, base_frame, timeout=10.0):
    """Return the robot's (x, y, yaw) in ``frame`` from TF, or None after ``timeout`` s."""
    import time

    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    buffer = Buffer()
    listener = TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform(frame, base_frame, Time())
            except TransformException:
                continue
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
            return transform.transform.translation.x, transform.transform.translation.y, yaw
    finally:
        del listener
    return None


def compose(origin, pose):
    """Return ``pose`` (x, y, yaw in the frame of ``origin``) in the frame ``origin`` is in."""
    ox, oy, oyaw = origin
    x, y, yaw = pose
    return (
        ox + math.cos(oyaw) * x - math.sin(oyaw) * y,
        oy + math.sin(oyaw) * x + math.cos(oyaw) * y,
        math.remainder(oyaw + yaw, 2.0 * math.pi),
    )


def main():
    """Drive the route; return the exit status."""
    args = parse_arguments(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    route = [tuple(float(v) for v in item.split(',')) for item in args.poses.split()]
    if not route or any(len(pose) != 3 for pose in route):
        print('--poses needs "x,y,yaw" triples', file=sys.stderr)
        return 1

    # Without rclpy's signal handlers, Ctrl-C only raises KeyboardInterrupt and the running goal
    # can still be cancelled.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    navigator = BasicNavigator('navigate_waypoints')
    code = 0
    try:
        navigator.waitUntilNav2Active(localizer=args.localizer)
        if args.relative:
            origin = start_pose(navigator, args.frame, args.base_frame)
            if origin is None:
                navigator.error(f'no transform {args.frame} -> {args.base_frame}')
                return 1
            navigator.info(f'start pose ({origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f})')
            route = [compose(origin, pose) for pose in route]
        if args.through:
            text = ' '.join(f'({x:.2f}, {y:.2f}, {yaw:.2f})' for x, y, yaw in route)
            navigator.info(f'through {len(route)} poses: {text}')
            navigator.goThroughPoses([make_pose(navigator, args.frame, *pose) for pose in route])
            route = [route[-1]]
        for index, (x, y, yaw) in enumerate(route, start=1):
            if not args.through:
                navigator.info(f'pose {index}/{len(route)}: ({x:.2f}, {y:.2f}, {yaw:.2f})')
                navigator.goToPose(make_pose(navigator, args.frame, x, y, yaw))
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback is not None:
                    navigator.get_logger().info(
                        f'  {feedback.distance_remaining:.2f} m to go',
                        throttle_duration_sec=2.0,
                    )
            result = navigator.getResult()
            navigator.info(f'pose {index}: {result.name}')
            if result != TaskResult.SUCCEEDED:
                code = 1
                break
    except KeyboardInterrupt:
        navigator.cancelTask()
        code = 1
    finally:
        navigator.destroyNode()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
