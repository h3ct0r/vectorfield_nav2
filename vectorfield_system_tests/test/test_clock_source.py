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

"""The simulators that publish /clock outlive the nodes that read it.

launch signals every process at once; a node shutting down can wait on simulated time, so
after SIGINT a simulator keeps publishing /clock while another node subscribes to it, and
exits once none does (vectorfield_ros/clock_source.hpp).
"""

import os
import signal
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

SIMULATORS = [('vectorfield_sim', 'kinematic_sim'), ('vectorfield_follower_3d', 'point_sim_3d')]
QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


@pytest.fixture
def reader():
    """Yield a node of the test; rclpy is shut down even when the test fails."""
    rclpy.init()
    try:
        node = rclpy.create_node('clock_reader')
        try:
            yield node
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def _start(package, executable, *args):
    return subprocess.Popen(
        [os.path.join(get_package_prefix(package), 'lib', package, executable)] + list(args)
    )


def _stop(process):
    if process.poll() is None:
        process.kill()
        process.wait()


def _spin_for(node, seconds, until=lambda: False):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not until():
        rclpy.spin_once(node, timeout_sec=0.02)


@pytest.mark.parametrize('package, executable', SIMULATORS, ids=[s[1] for s in SIMULATORS])
def test_the_clock_outlives_its_readers(reader, package, executable):
    """After SIGINT the clock runs while it has a reader, and stops soon after the last one."""
    stamps = []
    subscription = reader.create_subscription(Clock, '/clock', stamps.append, QOS)
    process = _start(package, executable)
    try:
        _spin_for(reader, 30.0, lambda: stamps)
        assert stamps, f'{executable} published no /clock'
        process.send_signal(signal.SIGINT)
        signalled = len(stamps)
        _spin_for(reader, 1.0)
        assert process.poll() is None, f'{executable} left while /clock had a reader'
        assert len(stamps) > signalled, f'{executable} stopped its clock while it was read'
        reader.destroy_subscription(subscription)
        left = time.monotonic()
        assert process.wait(timeout=10.0) == 0
        # The grace period (4 s) would end 3 s from here.
        assert time.monotonic() - left < 2.0, f'{executable} waited for its grace period'
    finally:
        _stop(process)


def test_no_clock_no_grace(reader):
    """point_sim_3d without publish_clock is no clock source: SIGINT stops it at once."""
    reader.create_subscription(Clock, '/clock', lambda _: None, QOS)
    odom = []
    reader.create_subscription(Odometry, 'odom', odom.append, 10)
    process = _start(
        'vectorfield_follower_3d', 'point_sim_3d', '--ros-args', '-p', 'publish_clock:=false'
    )
    try:
        _spin_for(reader, 30.0, lambda: odom)
        assert odom, 'point_sim_3d published no odom'
        process.send_signal(signal.SIGINT)
        signalled = time.monotonic()
        assert process.wait(timeout=10.0) == 0
        assert time.monotonic() - signalled < 2.0, 'point_sim_3d waited for the /clock reader'
    finally:
        _stop(process)
