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
exits once none does (vectorfield_ros/clock_source.hpp). A simulator that is not ours runs
under vectorfield_sim's shutdown_last, which holds the signal back from it until then.
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


# A simulator that is not ours: it publishes /clock at 10 Hz from 1000 s and dies of SIGINT, as
# Jazzy's Python loopback simulator does. It prints its pid first.
FAKE_SIMULATOR = """
import os, time
import rclpy
from rosgraph_msgs.msg import Clock
rclpy.init()
node = rclpy.create_node('fake_simulator')
publisher = node.create_publisher(Clock, '/clock', 10)
print(os.getpid(), flush=True)
stamp = 10_000
while True:
    msg = Clock()
    msg.clock.sec, msg.clock.nanosec = divmod(stamp * 100_000_000, 1_000_000_000)
    publisher.publish(msg)
    stamp += 1
    time.sleep(0.1)
"""


def _start_last(*command):
    """Start shutdown_last in its own session: killpg() on it then acts as a terminal's Ctrl+C."""
    return subprocess.Popen(
        [
            os.path.join(
                get_package_prefix('vectorfield_sim'), 'lib', 'vectorfield_sim', 'shutdown_last'
            ),
            *command,
        ],
        stdout=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )


def _start_fake(reader):
    """Start the fake simulator under shutdown_last; return the wrapper and the simulator's pid."""
    process = _start_last('python3', '-c', FAKE_SIMULATOR)
    pid = int(process.stdout.readline())
    _spin_for(reader, 30.0, lambda: reader.count_publishers('/clock') >= 1)
    assert reader.count_publishers('/clock') >= 1, 'the fake simulator published no /clock'
    return process, pid


def _alive(pid):
    """Whether process ``pid`` runs (a zombie does not)."""
    try:
        with open(f'/proc/{pid}/stat', encoding='utf-8') as stat:
            return stat.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def test_shutdown_last_stops_its_command_after_the_readers(reader):
    """A terminal's Ctrl+C reaches only shutdown_last; the clock runs until its reader leaves."""
    stamps = []
    subscription = reader.create_subscription(Clock, '/clock', stamps.append, QOS)
    process, pid = _start_fake(reader)
    try:
        _spin_for(reader, 30.0, lambda: stamps)
        os.killpg(process.pid, signal.SIGINT)
        signalled = len(stamps)
        _spin_for(reader, 1.0)
        assert process.poll() is None and _alive(pid), 'the simulator stopped while it was read'
        assert len(stamps) > signalled + 5, 'the clock stopped while it was read'
        reader.destroy_subscription(subscription)
        left = time.monotonic()
        # The simulator died of the forwarded SIGINT: a clean stop.
        assert process.wait(timeout=10.0) == 0
        assert time.monotonic() - left < 2.0, 'shutdown_last waited for its grace period'
        assert not _alive(pid)
    finally:
        _stop(process)


def test_shutdown_last_without_readers_stops_at_once(reader):
    """Nobody reads /clock: the signal goes on at once."""
    process, pid = _start_fake(reader)
    try:
        os.killpg(process.pid, signal.SIGINT)
        signalled = time.monotonic()
        assert process.wait(timeout=10.0) == 0
        assert time.monotonic() - signalled < 1.5
        assert not _alive(pid)
    finally:
        _stop(process)


def test_shutdown_last_forwards_sigterm_at_once(reader):
    """SIGTERM is not held back, even while the clock has a reader; it is no clean stop."""
    reader.create_subscription(Clock, '/clock', lambda _: None, QOS)
    process, pid = _start_fake(reader)
    try:
        process.send_signal(signal.SIGTERM)
        signalled = time.monotonic()
        # rclpy's handler shuts the simulator's context down, and its next publish fails.
        assert process.wait(timeout=10.0) != 0
        assert time.monotonic() - signalled < 1.0
        assert not _alive(pid)
    finally:
        _stop(process)


def test_shutdown_last_takes_its_command_along(reader):
    """Killed shutdown_last: its command gets SIGTERM and ends too."""
    process, pid = _start_fake(reader)
    try:
        process.kill()
        process.wait(timeout=10.0)
        end = time.monotonic() + 2.0
        while _alive(pid) and time.monotonic() < end:
            time.sleep(0.05)
        assert not _alive(pid), 'the simulator outlived shutdown_last'
    finally:
        _stop(process)
        if _alive(pid):
            os.kill(pid, signal.SIGKILL)


def test_shutdown_last_returns_the_command_status():
    """The command's own exit status, and 127 for a command that does not exist."""
    assert _start_last('sh', '-c', 'exit 3').wait(timeout=10.0) == 3
    assert _start_last('/nonexistent/simulator').wait(timeout=10.0) == 127
