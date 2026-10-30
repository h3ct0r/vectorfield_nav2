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

"""Exit once ``map`` carries a map: map_server is active.

map_server publishes ``map`` (transient local) when it activates. Until then its GetMap service
answers with an empty map, which Jazzy's loopback simulator keeps and later divides by; a demo
starts that simulator when this node exits 0.

Parameter: ``timeout`` (60.0 s, steady clock). Exit status: 0 once a map with a positive
resolution arrived, or at a shutdown signal (nothing to start then); 1 after the timeout.
"""

import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class WaitForMap(Node):
    """Remembers whether a non-empty map has arrived."""

    def __init__(self):
        super().__init__('wait_for_map')
        self.timeout = self.declare_parameter('timeout', 60.0).value
        self.received = False
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'map', self._on_map, qos)

    def _on_map(self, msg):
        if msg.info.resolution > 0.0 and msg.info.width > 0 and msg.info.height > 0:
            self.received = True


def main(args=None):
    """Wait for the map; return the exit status."""
    rclpy.init(args=args)
    node = WaitForMap()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and not node.received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    try:
        if node.received:
            node.get_logger().info('map_server serves the map')
        elif rclpy.ok():
            topic = node.resolve_topic_name('map')
            node.get_logger().error(f'no map on {topic} within {node.timeout} s')
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
