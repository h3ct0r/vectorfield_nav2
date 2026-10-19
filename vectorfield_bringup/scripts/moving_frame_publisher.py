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

r"""Publish a frame that moves with a constant twist: TF ``parent -> child`` and its Odometry.

The curve follower's moving-frame sources: a ``FRAME_MOVING`` goal in ``child`` reads either the
Odometry (``frame_odom_topic``) or the TF (``moving_frame_reference``, the parent).

Its parameters:

- ``parent_frame`` (``odom``) and ``child_frame`` (``moving_frame``);
- ``x``, ``y``, ``yaw``: the child's pose in the parent when the clock starts;
- ``vx``, ``vy``, ``wz``: the constant twist, in the child frame (m/s, rad/s). The pose follows
  it exactly (the SE(2) exponential): a translation, a rotation about the child's origin, or a
  circle about the point (−vy/wz, vx/wz) of the child frame;
- ``odom_topic`` (empty: ``<child_frame>/odom``): ``nav_msgs/Odometry`` with ``frame_id`` the
  parent, ``child_frame_id`` the child and the twist in the child frame;
- ``rate`` (50.0 Hz).

``x``, ``y``, ``yaw``, ``vx``, ``vy`` and ``wz`` can be set while the node runs: a new pose moves
the frame there, a new twist applies from the current pose on. Time is the node clock (run it
with ``use_sim_time:=true`` next to a simulator); nothing is published before the clock starts.

Example (a frame translating along x at 0.1 m/s)::

    ros2 run vectorfield_bringup moving_frame_publisher.py --ros-args -p use_sim_time:=true \
        -p vx:=0.1
    ros2 param set /moving_frame_publisher wz 0.05
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

POSE = ('x', 'y', 'yaw')
TWIST = ('vx', 'vy', 'wz')


def integrate(pose, twist, duration):
    """Return ``pose`` (x, y, yaw) moved by the body ``twist`` (vx, vy, wz) for ``duration`` s."""
    x, y, yaw = pose
    vx, vy, wz = twist
    angle = wz * duration
    if abs(angle) < 1e-9:
        dx, dy = vx * duration, vy * duration
    else:
        sin, cos = math.sin(angle), math.cos(angle)
        dx = (sin * vx - (1.0 - cos) * vy) / wz
        dy = ((1.0 - cos) * vx + sin * vy) / wz
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, yaw + angle


class MovingFramePublisher(Node):
    """Broadcasts the moving frame and publishes its Odometry at ``rate``."""

    def __init__(self):
        super().__init__('moving_frame_publisher')
        self._parent = self.declare_parameter('parent_frame', 'odom').value
        self._child = self.declare_parameter('child_frame', 'moving_frame').value
        # The motion: the pose `_anchor` at the time `_start` (None until the clock starts),
        # then the constant twist.
        self._anchor = tuple(float(self.declare_parameter(k, 0.0).value) for k in POSE)
        self._twist = tuple(float(self.declare_parameter(k, 0.0).value) for k in TWIST)
        topic = self.declare_parameter('odom_topic', '').value or f'{self._child}/odom'
        rate = self.declare_parameter('rate', 50.0).value
        self._start = None
        self.add_on_set_parameters_callback(self._on_parameters)
        self._broadcaster = TransformBroadcaster(self)
        self._odom = self.create_publisher(Odometry, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'{self._parent} -> {self._child} from {self._anchor} with the twist {self._twist}; '
            f'Odometry on {self._odom.topic_name}'
        )

    def _pose(self, now):
        if self._start is None:
            return self._anchor
        return integrate(self._anchor, self._twist, (now - self._start).nanoseconds * 1e-9)

    def _on_parameters(self, parameters):
        values = {p.name: float(p.value) for p in parameters if p.name in POSE + TWIST}
        if values:
            now = self.get_clock().now()
            pose = self._pose(now)
            if any(name in POSE for name in values):
                current = dict(zip(POSE, self.get_parameters(POSE)))
                pose = tuple(values.get(k, float(current[k].value)) for k in POSE)
            self._twist = tuple(values.get(k, v) for k, v in zip(TWIST, self._twist))
            self._anchor = pose
            self._start = now if self._start is not None and now.nanoseconds else self._start
            self.get_logger().info(f'from {self._anchor} with the twist {self._twist}')
        return SetParametersResult(successful=True)

    def _tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        if self._start is None:
            self._start = now
        x, y, yaw = self._pose(now)
        stamp = now.to_msg()
        qz, qw = math.sin(0.5 * yaw), math.cos(0.5 * yaw)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._parent
        transform.child_frame_id = self._child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._parent
        odom.child_frame_id = self._child
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = self._twist[:2]
        odom.twist.twist.angular.z = self._twist[2]
        self._odom.publish(odom)


def main():
    """Publish until interrupted."""
    rclpy.init()
    node = MovingFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
