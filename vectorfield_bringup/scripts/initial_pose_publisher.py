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

"""Publish ``initialpose`` once a second until the map -> base frame transform exists.

Nav2's loopback simulator publishes no TF before it receives ``initialpose``, and AMCL needs one
to start; vectorfield_sim teleports to it. The node then exits.

Parameters: ``x``, ``y``, ``yaw`` (the pose in ``map_frame``), ``map_frame`` (``map``),
``base_frame`` (``base_footprint``) and ``rate`` (1.0 Hz). The rate is kept on the steady
clock, so the node works before ``/clock`` starts.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# RViz's SetInitialPose covariance: 0.5 m in x and y, about 15 degrees in yaw.
COVARIANCE_XY = 0.25
COVARIANCE_YAW = 0.06853891909122467


class InitialPosePublisher(Node):
    """Republishes the initial pose until the localization transform appears."""

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self._x = self.declare_parameter('x', 0.0).value
        self._y = self.declare_parameter('y', 0.0).value
        self._yaw = self.declare_parameter('yaw', 0.0).value
        self._map_frame = self.declare_parameter('map_frame', 'map').value
        self._base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = self.declare_parameter('rate', 1.0).value
        self.done = False
        self._sent = 0
        self._publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._timer = self.create_timer(
            1.0 / rate, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def _tick(self):
        if self._buffer.can_transform(self._map_frame, self._base_frame, Time()):
            self.get_logger().info(
                f'{self._map_frame} -> {self._base_frame} is available after {self._sent} '
                'initial pose message(s); exiting'
            )
            self.done = True
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self._x)
        msg.pose.pose.position.y = float(self._y)
        msg.pose.pose.orientation.z = math.sin(0.5 * self._yaw)
        msg.pose.pose.orientation.w = math.cos(0.5 * self._yaw)
        msg.pose.covariance[0] = COVARIANCE_XY
        msg.pose.covariance[7] = COVARIANCE_XY
        msg.pose.covariance[35] = COVARIANCE_YAW
        self._publisher.publish(msg)
        self._sent += 1
        if self._sent == 1:
            self.get_logger().info(
                f'publishing the initial pose ({self._x:.2f}, {self._y:.2f}, {self._yaw:.2f}) '
                f'in {self._map_frame} until {self._map_frame} -> {self._base_frame} exists'
            )


def main():
    """Run the node until the transform exists or the process is interrupted."""
    rclpy.init()
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
