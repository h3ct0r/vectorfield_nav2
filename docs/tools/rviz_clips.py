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

"""Helpers of record_rviz_clips.sh: an RViz view, a NavigateToPose goal, a localization jump.

Subcommands (run in the dev container, the workspace sourced)::

    rviz_clips.py view OUT --scale S --x X --y Y [--angle A] [--width W] [--height H]
    rviz_clips.py goal X Y YAW_DEG [--wait SECONDS]
    rviz_clips.py step X_AT DX DY DYAW_DEG [--timeout SECONDS]

``view`` writes a copy of the shipped ``rviz/vectorfield.rviz`` with only the camera changed (a
top-down orthographic view), the docks hidden and the field-grid displays off. ``goal`` sends
one goal in the map frame and waits for its result. ``step`` waits until the control point
passes x >= X_AT in FOLLOW, then calls vectorfield_sim's ``~/step_map_to_odom``.
"""

import argparse
import math
import sys
import time


def write_view(args):
    """Write the view-only RViz config."""
    import yaml
    from ament_index_python.packages import get_package_share_directory

    source = get_package_share_directory('vectorfield_bringup') + '/rviz/vectorfield.rviz'
    with open(source) as f:
        config = yaml.safe_load(f)
    manager = config['Visualization Manager']

    def disable_field_grids(displays):
        for display in displays:
            if display.get('Name') == 'Field grid':
                display['Enabled'] = False
            disable_field_grids(display.get('Displays', []))

    disable_field_grids(manager['Displays'])
    manager['Views']['Current'] = {
        'Angle': args.angle,
        'Class': 'rviz_default_plugins/TopDownOrtho',
        'Enable Stereo Rendering': {
            'Stereo Eye Separation': 0.06,
            'Stereo Focal Distance': 1,
            'Swap Stereo Eyes': False,
            'Value': False,
        },
        'Invert Z Axis': False,
        'Name': 'Current View',
        'Near Clip Distance': 0.01,
        'Scale': args.scale,
        'Target Frame': '<Fixed Frame>',
        'Value': 'TopDownOrtho (rviz_default_plugins)',
        'X': args.x,
        'Y': args.y,
    }
    window = config['Window Geometry']
    window.update(
        {
            'Width': args.width,
            'Height': args.height,
            'X': 0,
            'Y': 0,
            'Hide Left Dock': True,
            'Hide Right Dock': True,
        }
    )
    with open(args.out, 'w') as f:
        yaml.safe_dump(config, f, sort_keys=False)


def _node(name):
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    return rclpy, Node(name)


def send_goal(args):
    """Send one NavigateToPose goal and wait for its result."""
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    rclpy, node = _node('rviz_clips_goal')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=30.0):
        sys.exit('rviz_clips: no navigate_to_pose server')
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(math.radians(args.yaw) / 2)
    goal.pose.pose.orientation.w = math.cos(math.radians(args.yaw) / 2)
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        sys.exit('rviz_clips: goal rejected')
    result = handle.get_result_async()
    end = time.monotonic() + args.wait
    while not result.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    status = result.result().status if result.done() else 'timeout'
    print(f'goal ({args.x}, {args.y}, {args.yaw} deg): {status}')
    node.destroy_node()
    rclpy.shutdown()


def step_map_to_odom(args):
    """Jump the published map -> odom once the control point passes x >= X_AT in FOLLOW."""
    from nav2_msgs.srv import SetInitialPose

    from vectorfield_msgs.msg import FieldState

    rclpy, node = _node('rviz_clips_step')
    passed = []

    def on_state(m):
        passed.append(m.phase == FieldState.PHASE_FOLLOW and m.control_point.x >= args.x_at)

    node.create_subscription(FieldState, '/FollowPath/field_state', on_state, 10)
    client = node.create_client(SetInitialPose, '/vectorfield_sim/step_map_to_odom')
    if not client.wait_for_service(timeout_sec=10.0):
        sys.exit('rviz_clips: no step_map_to_odom service')
    end = time.monotonic() + args.timeout
    while not any(passed) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if any(passed):
        request = SetInitialPose.Request()
        request.pose.header.frame_id = 'map'
        request.pose.pose.pose.position.x = args.dx
        request.pose.pose.pose.position.y = args.dy
        request.pose.pose.pose.orientation.z = math.sin(math.radians(args.dyaw) / 2)
        request.pose.pose.pose.orientation.w = math.cos(math.radians(args.dyaw) / 2)
        rclpy.spin_until_future_complete(node, client.call_async(request), timeout_sec=5.0)
        print('stepped map -> odom by', args.dx, args.dy, args.dyaw)
    else:
        print('rviz_clips: the robot never reached the step', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()


def main(argv=None):
    """Parse the subcommand and run it."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    view = sub.add_parser('view')
    view.add_argument('out')
    view.add_argument('--scale', type=float, required=True, help='pixels per metre')
    view.add_argument('--x', type=float, default=0.0)
    view.add_argument('--y', type=float, default=0.0)
    view.add_argument('--angle', type=float, default=0.0, help='view rotation (rad)')
    view.add_argument('--width', type=int, default=2400)
    view.add_argument('--height', type=int, default=1400)
    view.set_defaults(run=write_view)
    goal = sub.add_parser('goal')
    goal.add_argument('x', type=float)
    goal.add_argument('y', type=float)
    goal.add_argument('yaw', type=float, help='degrees')
    goal.add_argument('--wait', type=float, default=90.0)
    goal.set_defaults(run=send_goal)
    step = sub.add_parser('step')
    step.add_argument('x_at', type=float)
    step.add_argument('dx', type=float)
    step.add_argument('dy', type=float)
    step.add_argument('dyaw', type=float, help='degrees')
    step.add_argument('--timeout', type=float, default=60.0)
    step.set_defaults(run=step_map_to_odom)
    args = parser.parse_args(argv)
    args.run(args)


if __name__ == '__main__':
    main()
