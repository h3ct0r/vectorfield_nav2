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

"""Tests of the mujoco_sim node: message conversions, and the node stepped by hand.

The node runs with ``real_time_factor`` 0 (no step timer) and the vendored Scout Mini; the test
calls its tick and reads what it published from a second node.
"""

import math

import pytest

from vectorfield_mujoco import gl

# Chosen before mujoco is imported; a backend that breaks the import skips the module.
gl.configure()
PROBLEM = gl.probe(render=False)
if PROBLEM:
    pytest.skip(f'mujoco does not import: {PROBLEM}', allow_module_level=True)
pytest.importorskip('mujoco')
rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from vectorfield_mujoco import sim_node  # noqa: E402


def test_stamped_default_follows_the_distro():
    """TwistStamped by default from Kilted on, as Nav2 (compat::kStampedCmdVelDefault)."""
    assert not sim_node.stamped_cmd_vel_default('humble')
    assert not sim_node.stamped_cmd_vel_default('jazzy')
    assert sim_node.stamped_cmd_vel_default('kilted')
    assert sim_node.stamped_cmd_vel_default('lyrical')
    assert sim_node.stamped_cmd_vel_default('rolling')


def test_stamps():
    """Simulated seconds become a builtin_interfaces/Time."""
    stamp = sim_node.to_stamp(12.3456789)
    assert (stamp.sec, stamp.nanosec) == (12, 345678900)


def test_marker_dict_is_in_the_map_frame():
    """A marker's pose and points are placed through its frame's map pose."""
    marker = Marker(type=Marker.LINE_STRIP)
    marker.pose.position.x = 1.0
    marker.pose.orientation.w = 1.0
    marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=1.0, y=0.0, z=0.0)]
    marker.scale.x = 0.05
    marker.color.r, marker.color.a = 1.0, 0.5
    frame = (1.0, 2.0, 0.1, 0.5 * math.pi)
    d = sim_node.marker_dict(marker, frame)
    (x, y, z), quat = d['pose']
    assert (x, y, z) == pytest.approx((1.0, 3.0, 0.1))
    assert quat == pytest.approx((0.0, 0.0, math.sin(0.25 * math.pi), math.cos(0.25 * math.pi)))
    assert d['type'] == Marker.LINE_STRIP and d['rgba'] == (1.0, 0.0, 0.0, 0.5)
    from vectorfield_mujoco import overlay

    # The overlay applies the pose: the second point lands 1 m further along map +y.
    ((_, _, _, start, end, _),) = overlay.primitives(d)
    assert start == pytest.approx([1.0, 3.0, 0.1]) and end == pytest.approx([1.0, 4.0, 0.1])


def test_path_dict():
    """A Path becomes a LINE_STRIP of its positions in the map frame."""
    path = Path()
    for x in (0.0, 1.0, 2.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        path.poses.append(pose)
    d = sim_node.path_dict(path, (1.0, 0.0, 0.0, math.pi), (0.0, 0.0, 1.0, 1.0))
    assert d['type'] == 'LINE_STRIP'
    assert [p[0] for p in d['points']] == pytest.approx([1.0, 0.0, -1.0])


@pytest.fixture
def node():
    """Start the node at (1, -1, 0.5) in open_10x10 with an unmapped box, stepped by hand."""
    yield from _node()


def _node(extra=()):
    from ament_index_python.packages import get_package_share_directory

    world = get_package_share_directory('vectorfield_bringup') + '/maps/open_10x10.yaml'
    args = [
        '--ros-args',
        '-p', 'robot:=scout_mini',
        '-p', f'world_map:={world}',
        # A YAML string (quoted), as the launch passes it.
        '-p', "obstacles:='[{type: box, pos: [3, -1], size: [0.2, 0.2, 0.25]}]'",
        '-p', 'initial_pose_x:=1.0',
        '-p', 'initial_pose_y:=-1.0',
        '-p', 'initial_pose_yaw:=0.5',
        '-p', 'real_time_factor:=0.0',
        *extra,
    ]  # fmt: skip
    rclpy.init(args=args)
    sim = sim_node.MujocoSim()
    probe = rclpy.create_node('probe')
    yield sim, probe
    sim.close()
    sim.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def _spin(nodes, predicate, ticks, sim):
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    for _ in range(ticks):
        sim._on_tick()
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            break
    for _ in range(20):
        executor.spin_once(timeout_sec=0.01)
    return predicate()


def test_node_publishes_vectorfield_sims_interface(node):
    """/clock, odom, ~/ground_truth and the scan, in simulated time and the right frames."""
    from rclpy.qos import qos_profile_sensor_data

    sim, probe = node
    got = {'clock': [], 'odom': [], 'truth': [], 'scan': []}
    probe.create_subscription(Clock, '/clock', got['clock'].append, 10)
    probe.create_subscription(Odometry, 'odom', got['odom'].append, 10)
    probe.create_subscription(Odometry, '/mujoco_sim/ground_truth', got['truth'].append, 10)
    probe.create_subscription(LaserScan, 'scan', got['scan'].append, qos_profile_sensor_data)
    assert _spin([sim, probe], lambda: len(got['scan']) >= 2, 200, sim)
    assert got['clock'] and got['odom'] and got['truth']
    odom, truth, scan = got['odom'][-1], got['truth'][-1], got['scan'][-1]
    assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_footprint')
    assert abs(odom.pose.pose.position.x) < 0.01  # odometry starts at the identity
    assert truth.header.frame_id == 'map'
    assert truth.pose.pose.position.x == pytest.approx(1.0, abs=0.01)
    assert truth.pose.pose.position.y == pytest.approx(-1.0, abs=0.01)
    assert scan.header.frame_id == 'base_scan' and len(scan.ranges) == 360
    # The box 2 m ahead of the robot along map +x, seen at -0.5 rad in the robot's frame (the
    # box's near face is 2 - 0.2 m away along x).
    stamp = scan.header.stamp.sec + 1e-9 * scan.header.stamp.nanosec
    assert stamp == pytest.approx(round(stamp, 1), abs=1e-6)  # scans at 10 Hz of sim time
    beam = int(round((-0.5 - scan.angle_min) / scan.angle_increment))
    assert scan.ranges[beam] == pytest.approx(1.8, abs=0.06)


def test_initial_pose_teleports(node):
    """An initialpose message moves the robot and the published map -> odom with it."""
    sim, probe = node
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.pose.pose.position.x = -2.0
    message.pose.pose.position.y = 2.0
    message.pose.pose.orientation.z = math.sin(0.5)
    message.pose.pose.orientation.w = math.cos(0.5)
    sim.interface.on_initial_pose(message)
    assert sim.sim.truth() == pytest.approx((-2.0, 2.0, 1.0), abs=1e-6)
    assert sim.interface.frame_pose('base_footprint') == pytest.approx((-2.0, 2.0, 0.0, 1.0))
    x, y, z, yaw = sim.interface.frame_pose('base_link')
    assert z == pytest.approx(0.157, abs=0.005)
    assert sim.interface.frame_pose('unknown_frame') is None
    message.header.frame_id = 'odom'
    sim.interface.on_initial_pose(message)  # ignored: not the map frame
    assert sim.sim.truth()[:2] == pytest.approx((-2.0, 2.0), abs=1e-6)


def test_rendering_is_refused_without_a_backend(monkeypatch, tmp_path):
    """Without an OpenGL backend the views are off and the physics runs."""
    monkeypatch.delenv('MUJOCO_GL', raising=False)
    extra = ['-p', 'camera_topic:=camera', '-p', f'record_path:={tmp_path}/clip.mp4']
    extra += ['-p', 'viewer:=true']
    started = _node(extra)
    sim, _ = next(started)
    try:
        assert sim._camera is None and sim._recorder is None and sim._viewer is None
        for _ in range(10):
            sim._on_tick()
        assert sim.sim.time == pytest.approx(0.1)
        assert not (tmp_path / 'clip.mp4').exists()
    finally:
        next(started, None)
