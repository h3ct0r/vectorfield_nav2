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

The node runs with ``real_time_factor`` 0 (no step timer) and the vendored Scout Mini or, for
the aerial interface, the X2-like quadrotor of test/models (registered for the test as a package
robot, so no download); the test calls its tick and reads what it published from a second node.
"""

import dataclasses
import math
import pathlib

import numpy as np
import pytest

from vectorfield_mujoco import gl

# Chosen before mujoco is imported; a backend that breaks the import skips the module.
gl.configure()
PROBLEM = gl.probe(render=False)
if PROBLEM:
    pytest.skip(f'mujoco does not import: {PROBLEM}', allow_module_level=True)
pytest.importorskip('mujoco')
rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import (  # noqa: E402
    Point,
    PoseStamped,
    PoseWithCovarianceStamped,
    TwistStamped,
)
from nav_msgs.msg import Odometry, Path  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from vectorfield_mujoco import sim_node, simulation  # noqa: E402

QUADROTOR = pathlib.Path(__file__).resolve().parent / 'models' / 'quadrotor.xml'


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
    assert (d['ns'], d['id']) == ('', 0)
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
    yield from _started()


def _started():
    """Yield (sim, probe) and tear them down, also when the node fails to start or a test fails."""
    sim = probe = None
    try:
        sim = sim_node.MujocoSim()
        probe = rclpy.create_node('probe')
        yield sim, probe
    finally:
        if sim is not None:
            sim.close()
            sim.destroy_node()
        if probe is not None:
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
    assert z == pytest.approx(0.1645, abs=0.005)
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


def test_recording_overlay_hides_markers_and_draws_the_trail():
    """The recording leaves out record_hide and draws the robot's path on the floor."""
    import mujoco

    started = _node(['-p', 'record_trail:=1.0'])
    sim, _ = next(started)
    try:
        world = sim.sim.world
        sim.overlays._markers = {
            '/markers': {
                ('field', 0): {'ns': 'field', 'id': 0, 'type': 'SPHERE'},
                ('command', 0): {'ns': 'command', 'id': 0, 'type': 'SPHERE'},
            }
        }
        sim.overlays._dirty = True
        scn = mujoco.MjvScene(world.model, maxgeom=1000)
        assert sim._draw_recording(scn, ['field']) == 1  # one trail point: no line yet
        simulation.teleport(world, 2.0, -1.0, 0.5)
        sim._on_tick()
        scn.ngeom = 0
        assert sim._draw_recording(scn, ['field']) == 2  # the command and one trail segment
        segment = scn.geoms[1]
        assert segment.pos[2] == pytest.approx(sim_node.OVERLAY_LIFT, abs=1e-6)
        assert segment.rgba == pytest.approx(sim_node.TRAIL_COLOUR)
        # Points older than record_trail go, but the last two.
        for x in (2.5, 3.0):
            for _ in range(120):
                sim._on_tick()
            simulation.teleport(world, x, -1.0, 0.5)
            mujoco.mj_forward(world.model, world.data)
            sim._draw_recording(mujoco.MjvScene(world.model, maxgeom=1000), [])
        assert [round(p[0], 3) for _, p in sim._trail] == [2.5, 3.0]
    finally:
        next(started, None)


def test_point_cloud():
    """Points become point_sim_3d's PointCloud2: one row of FLOAT32 x, y, z."""
    points = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, 1.5]])
    cloud = sim_node.point_cloud(points, 'map', sim_node.to_stamp(2.5))
    assert (cloud.header.frame_id, cloud.height, cloud.width) == ('map', 1, 2)
    assert [(f.name, f.offset, f.datatype) for f in cloud.fields] == [
        ('x', 0, 7),
        ('y', 4, 7),
        ('z', 8, 7),
    ]
    assert (cloud.point_step, cloud.row_step, cloud.is_dense) == (12, 24, True)
    decoded = np.frombuffer(bytes(cloud.data), dtype='<f4').reshape(-1, 3)
    assert decoded == pytest.approx(points)
    assert sim_node.point_cloud(np.zeros((0, 3)), 'map', cloud.header.stamp).width == 0


SPHERE = {'type': 'sphere', 'pos': [2.0, 0.0, 1.0], 'size': [0.3]}
FRAME_ORIGIN, FRAME_LINEAR, FRAME_ANGULAR = (1.5, 0.0, 1.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1)


@pytest.fixture
def aerial(monkeypatch):
    """Start the node with the quadrotor at (1, -1, 1.2), yaw 0.5, a sphere and a moving frame."""
    from vectorfield_mujoco import registry

    robot = dataclasses.replace(
        registry.get('x2'), key='quadrotor', source='package', mjcf=str(QUADROTOR)
    )
    monkeypatch.setitem(registry.ROBOTS, 'quadrotor', robot)
    args = [
        '--ros-args',
        '-p', 'robot:=quadrotor',
        '-p', "obstacles:='[{type: sphere, pos: [2, 0, 1], size: [0.3]}]'",
        '-p', 'initial_pose_x:=1.0',
        '-p', 'initial_pose_y:=-1.0',
        '-p', 'initial_pose_z:=1.2',
        '-p', 'initial_pose_yaw:=0.5',
        '-p', 'real_time_factor:=0.0',
        '-p', 'moving_frame:=platform',
        '-p', 'moving_frame_topic:=platform/odom',
        '-p', f'moving_frame_origin:={list(FRAME_ORIGIN)}',
        '-p', f'moving_frame_linear:={list(FRAME_LINEAR)}',
        '-p', f'moving_frame_angular:={list(FRAME_ANGULAR)}',
    ]  # fmt: skip
    rclpy.init(args=args)
    yield from _started()


def _stamp(message):
    return message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec


def test_aerial_interface_publishes_point_sim_3ds_interface(aerial):
    """The odom (pose in map, twist in base_link), TF map -> base_link, cloud and moving frame."""
    sim, probe = aerial
    assert isinstance(sim.interface, sim_node.AerialInterface)
    got = {'clock': [], 'odom': [], 'cloud': [], 'tf': [], 'frame': []}
    probe.create_subscription(Clock, '/clock', got['clock'].append, 10)
    probe.create_subscription(Odometry, 'odom', got['odom'].append, 10)
    probe.create_subscription(PointCloud2, 'obstacle_cloud', got['cloud'].append, 10)
    probe.create_subscription(TFMessage, '/tf', got['tf'].append, 10)
    probe.create_subscription(Odometry, 'platform/odom', got['frame'].append, 10)
    assert _spin([sim, probe], lambda: got['cloud'] and len(got['frame']) >= 2, 200, sim)
    assert got['clock'] and got['odom'] and got['tf']
    odom = got['odom'][-1]
    assert (odom.header.frame_id, odom.child_frame_id) == ('map', 'base_link')
    p = odom.pose.pose.position
    assert (p.x, p.y, p.z) == pytest.approx((1.0, -1.0, 1.2), abs=0.02)  # hovering
    assert sim_node.yaw_of(odom.pose.pose.orientation) == pytest.approx(0.5, abs=0.01)
    children = {t.child_frame_id: t for m in got['tf'] for t in m.transforms}
    assert children['base_link'].header.frame_id == 'map' and 'platform' in children
    # The cloud: the sphere's surface in map, nothing else (no map, so no walls).
    cloud = got['cloud'][-1]
    assert cloud.header.frame_id == 'map' and cloud.width > 100
    points = np.frombuffer(bytes(cloud.data), dtype='<f4').reshape(-1, 3)
    radii = np.linalg.norm(points - np.array(SPHERE['pos']), axis=1)
    assert radii == pytest.approx(np.full(len(points), 0.3), abs=1e-5)
    # The moving frame: point_sim_3d's constant twist from its origin.
    frame = got['frame'][-1]
    assert (frame.header.frame_id, frame.child_frame_id) == ('map', 'platform')
    assert frame.twist.twist.linear.x == pytest.approx(0.1)
    assert frame.twist.twist.angular.z == pytest.approx(0.1)
    rotation, translation = simulation.exp_se3(FRAME_LINEAR, FRAME_ANGULAR, _stamp(frame))
    q = frame.pose.pose.position
    assert (q.x, q.y, q.z) == pytest.approx(np.asarray(FRAME_ORIGIN) + translation)
    assert sim_node.yaw_of(frame.pose.pose.orientation) == pytest.approx(
        math.atan2(rotation[1, 0], rotation[0, 0])
    )
    assert sim.interface.frame_pose('map') == (0.0, 0.0, 0.0, 0.0)
    assert sim.interface.frame_pose('base_link')[:3] == pytest.approx((1.0, -1.0, 1.2), abs=0.02)
    assert sim.interface.frame_pose('platform')[:3] == pytest.approx(
        sim.interface.moving_pose()[1]
    )
    assert sim.interface.frame_pose('odom') is None


def _command(frame_id, x, y, z, yaw_rate):
    message = TwistStamped()
    message.header.frame_id = frame_id
    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = x, y, z
    message.twist.angular.z = yaw_rate
    return message


def test_aerial_commands_follow_their_frame(aerial):
    """frame_id base_link: the heading frame; any other frame_id: the world (global) frame."""
    sim, probe = aerial
    interface = sim.interface
    interface.on_command(_command('base_link', 0.4, 0.0, 0.1, 0.0))
    linear, _ = sim.sim.velocity()
    assert linear == pytest.approx((0.4 * math.cos(0.5), 0.4 * math.sin(0.5), 0.1), abs=1e-3)
    interface.on_command(_command('map', 0.4, 0.0, 0.1, 0.2))
    assert sim.sim.velocity() == ((0.4, 0.0, 0.1), 0.2)
    interface.on_command(_command('map', math.nan, 0.0, 0.0, 0.0))  # ignored
    assert sim.sim.velocity() == ((0.4, 0.0, 0.1), 0.2)
    # Flown for 3 s: odom's base-frame twist, turned by its orientation as the 3D follower turns
    # it, is the commanded world velocity.
    got = []
    probe.create_subscription(Odometry, 'odom', got.append, 10)

    def command():
        interface.on_command(_command('map', 0.4, 0.0, 0.1, 0.0))
        return False

    _spin([sim, probe], command, 300, sim)
    odom = got[-1]
    q = odom.pose.pose.orientation
    rotation = np.array(
        [
            [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y)],
            [2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y * q.z - q.w * q.x)],
            [2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x**2 + q.y**2)],
        ]
    )
    v = odom.twist.twist.linear
    assert rotation @ np.array([v.x, v.y, v.z]) == pytest.approx([0.4, 0.0, 0.1], abs=0.03)


def test_the_clock_outlives_its_readers():
    """After the signal, spin_clock_source returns once no other node subscribes to /clock."""
    import time

    from rclpy.context import Context

    context = Context()
    rclpy.init(context=context)
    try:
        source = rclpy.create_node('clock_source', context=context)
        reader = rclpy.create_node('clock_reader', context=context)
        subscription = reader.create_subscription(Clock, '/clock', lambda _: None, 1)
        end = time.monotonic() + 5.0
        while source.count_subscribers('/clock') < 1 and time.monotonic() < end:
            time.sleep(0.01)
        signals = []
        start = time.monotonic()
        sim_node.spin_clock_source(
            source, lambda: True, grace=0.5, on_signal=lambda: signals.append(True)
        )
        assert time.monotonic() - start >= 0.5  # read until the grace ran out
        assert signals == [True]  # on_signal ran once
        reader.destroy_subscription(subscription)
        start = time.monotonic()
        sim_node.spin_clock_source(source, lambda: True, grace=5.0)
        assert time.monotonic() - start < 2.0  # nobody reads it any more
    finally:
        rclpy.shutdown(context=context)


def test_other_clock_sources_are_no_readers():
    """A node that publishes /clock and reads it (another simulator) keeps no source running."""
    import time

    from rclpy.context import Context

    context = Context()
    rclpy.init(context=context)
    try:
        source = rclpy.create_node('clock_source', context=context)
        other = rclpy.create_node('other_clock_source', context=context)
        other.create_publisher(Clock, '/clock', 1)
        other.create_subscription(Clock, '/clock', lambda _: None, 1)
        end = time.monotonic() + 5.0
        while source.count_subscribers('/clock') < 1 and time.monotonic() < end:
            time.sleep(0.01)
        assert source.count_subscribers('/clock') == 1
        assert sim_node.clock_readers(source) == 0
        start = time.monotonic()
        sim_node.spin_clock_source(source, lambda: True, grace=5.0)
        assert time.monotonic() - start < 2.0
    finally:
        rclpy.shutdown(context=context)
