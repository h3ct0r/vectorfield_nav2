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

"""``mujoco_sim``: MuJoCo physics behind vectorfield_sim's ROS interface.

The node is the time source. A steady timer advances the physics in ticks (``clock_rate``, a
whole number of ``physics_rate`` steps each) at ``real_time_factor`` times real time, and every
tick publishes ``/clock``; every stamp is simulated time. The world is built by
:func:`vectorfield_mujoco.scene.make_world` from ``robot`` (a key of
:data:`vectorfield_mujoco.registry.ROBOTS`; Menagerie robots are fetched on first use),
``world_map`` (the Nav2 map YAML that map_server serves: its occupied cells become walls) and
``obstacles`` (unmapped; a YAML list, see :func:`vectorfield_mujoco.simulation.parse_obstacles`).

**Ground robots** get vectorfield_sim's interface, so Nav2 and the followers run unchanged:

- input: ``cmd_vel_topic`` (``cmd_vel``; ``cmd_vel_safe`` after Humble's collision_monitor), a
  ``Twist`` or, with ``enable_stamped_cmd_vel`` (default: the distro's Nav2 default), a
  ``TwistStamped``; a command older than ``cmd_vel_timeout`` simulated seconds is replaced by
  zero; ``initialpose`` (map frame) teleports the robot; ``~/step_map_to_odom`` steps the
  published map → odom only;
- output: ``odom`` (ground-truth pose in the odom frame, so it carries the physics' slip, and the
  body twist), the TF ``odom → base_footprint`` and ``~/ground_truth`` at ``odom_rate``; a static
  ``map → odom``; the static ``base_footprint → base_link → base_scan`` of the model;
  ``scan`` in ``base_scan`` from the model's lidar site at ``scan_rate``, delivered
  ``scan_latency`` after its stamp, with ``odom`` and its TF also published at every scan stamp
  (as vectorfield_sim does, for Humble's tf2 message filter); Ackermann:
  ``~/infeasible_command_count``.

**Aerial robots** (the X2) get point_sim_3d's interface instead (:class:`AerialInterface`), so
the 3D follower runs unchanged:

- input: ``cmd_vel_topic`` (``cmd_vel``), a ``TwistStamped`` whose velocity is in the heading
  frame (``base_frame`` turned by its yaw only) when its ``frame_id`` is ``base_frame``, else in
  ``global_frame``, plus the yaw rate; a command older than ``cmd_vel_timeout`` is zero (hover).
  The flight controller (:mod:`vectorfield_mujoco.flight`) tracks it at the physics rate: it
  stands in for an autopilot's velocity mode (PX4 offboard, ArduPilot guided), so the clip shows
  real attitude dynamics, not a kinematic point;
- output: ``odom_topic`` (``odom``: the pose in ``global_frame`` with the full attitude, the
  twist in ``base_frame``) and the TF ``global_frame → base_frame`` at ``odom_rate``; on
  ``cloud_topic`` (``obstacle_cloud``, at ``cloud_rate``) a PointCloud2 of the world's obstacle
  and wall surfaces, ``cloud_spacing`` apart; with ``moving_frame``, point_sim_3d's frame moving
  with the constant twist ``moving_frame_linear``, ``moving_frame_angular`` from
  ``moving_frame_origin`` (TF and Odometry on ``moving_frame_topic``). ``initial_pose_z`` sets
  the start height (0 rests on the floor). point_sim_3d's ``obstacle_spheres`` has no
  counterpart: the cloud holds the obstacles built into the world (``obstacles``).

:class:`GroundInterface` and :class:`AerialInterface` hold everything specific to each kind.

**Viewing and recording** (all optional, all with the same overlays):

- ``viewer``: MuJoCo's passive viewer (needs a display and ``MUJOCO_GL=glfw``);
- ``camera_topic``: an offscreen ``sensor_msgs/Image`` (rgb8, ``camera_size``) at ``camera_rate``
  Hz of simulated time, for Foxglove or RViz on a host without a display;
- ``record_path``: an H.264 MP4 at ``record_fps`` frames per simulated second
  (:class:`vectorfield_mujoco.recorder.Recorder`), finished when the node shuts down; its
  camera is ``record_camera`` with ``record_view``'s settings, ``record_hide`` leaves marker
  namespaces (``ns`` or ``ns/id``) and Path topics out of it, to keep a clip legible, and
  ``record_trail`` draws the robot's path of its last seconds;
- ``overlay_topics``: ``visualization_msgs/MarkerArray`` and ``nav_msgs/Path`` topics (the type
  is looked up once the topic appears) drawn into those scenes
  (:mod:`vectorfield_mujoco.overlay`), so the clips show the plan and the controller's markers
  around the physical robot.

Headless rendering needs an OpenGL backend chosen before MuJoCo is imported: :func:`main` calls
:func:`vectorfield_mujoco.gl.configure` first. Without a usable one (no display, no OSMesa, no
EGL) the node refuses the viewer, the camera and the recording with an error and runs everything
else; a backend that fails while rendering stops only that feature.
"""

import array
import collections
import contextlib
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import UInt64
from visualization_msgs.msg import Marker, MarkerArray

from vectorfield_mujoco import gl, menagerie, overlay, registry, simulation

#: Overlay geoms are lifted this much (m) so that lines on the floor stay visible.
OVERLAY_LIFT = 0.02

#: Colour and width (m) of the robot's path drawn in a recording (``record_trail``).
TRAIL_COLOUR = (0.12, 0.12, 0.16, 0.9)
TRAIL_WIDTH = 0.025

#: Colours of the Path overlays, in the order of ``overlay_topics``.
PATH_COLOURS = ((0.1, 0.45, 1.0, 0.9), (0.1, 0.8, 0.3, 0.9), (0.9, 0.2, 0.7, 0.9))

#: Wall-clock seconds close() waits for the passive viewer's thread to stop.
VIEWER_CLOSE_TIMEOUT = 2.0

#: The wall-clock rate (Hz) at which the passive viewer is synchronised.
VIEWER_RATE = 30.0


def stamped_cmd_vel_default(distro=None):
    """Return the distro's Nav2 ``enable_stamped_cmd_vel`` default (vectorfield_sim's too).

    False on Humble and Jazzy, true on Kilted, Lyrical and later
    (``vectorfield_ros::compat::kStampedCmdVelDefault``).
    """
    distro = os.environ.get('ROS_DISTRO', '') if distro is None else distro
    return distro.strip().lower() not in ('humble', 'iron', 'jazzy')


def to_stamp(seconds):
    """Return a ``builtin_interfaces/Time`` of a simulated time (s)."""
    from builtin_interfaces.msg import Time

    nanoseconds = int(round(seconds * 1e9))
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def yaw_of(q):
    """Return the yaw of a ``geometry_msgs/Quaternion``."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform(parent, child, stamp, x, y, z, yaw):
    """Return a ``TransformStamped`` of a planar pose with a height."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.z = math.sin(0.5 * yaw)
    t.transform.rotation.w = math.cos(0.5 * yaw)
    return t


def odometry(parent, child, stamp, pose, twist):
    """Return an ``Odometry`` of a planar pose and a body twist (vx, vy, omega)."""
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.pose.pose.position.x = float(pose[0])
    msg.pose.pose.position.y = float(pose[1])
    msg.pose.pose.orientation.z = math.sin(0.5 * pose[2])
    msg.pose.pose.orientation.w = math.cos(0.5 * pose[2])
    msg.twist.twist.linear.x = float(twist[0])
    msg.twist.twist.linear.y = float(twist[1])
    msg.twist.twist.angular.z = float(twist[2])
    return msg


def transform3(parent, child, stamp, position, quat):
    """Return a ``TransformStamped`` of a position and an (x, y, z, w) quaternion."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
        float(v) for v in position
    )
    r = t.transform.rotation
    r.x, r.y, r.z, r.w = (float(v) for v in quat)
    return t


def odometry3(parent, child, stamp, position, quat, linear, angular):
    """Return an ``Odometry`` of a 3D pose ((x, y, z, w) quaternion) and a twist in ``child``."""
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    pose, twist = msg.pose.pose, msg.twist.twist
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    o = pose.orientation
    o.x, o.y, o.z, o.w = (float(v) for v in quat)
    twist.linear.x, twist.linear.y, twist.linear.z = (float(v) for v in linear)
    twist.angular.x, twist.angular.y, twist.angular.z = (float(v) for v in angular)
    return msg


def point_cloud(points, frame_id, stamp):
    """Return a ``sensor_msgs/PointCloud2`` of (N, 3) points: FLOAT32 x, y, z, as point_sim_3d."""
    xyz = np.ascontiguousarray(np.asarray(points, dtype='<f4').reshape(-1, 3))
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(len(xyz))
    msg.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate('xyz')
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    # array.array, not bytes: rosidl's uint8[] setter checks bytes element by element in Python
    # (Humble: ~35 ms for a 50k-point cloud), an array.array it takes as is.
    msg.data = array.array('B', xyz.tobytes())
    return msg


def matrix_quat(rotation):
    """Return the (x, y, z, w) quaternion of a rotation matrix."""
    import mujoco

    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rotation, dtype=float).ravel())
    return quat[1], quat[2], quat[3], quat[0]


def frame_quat(yaw, q):
    """Return the (x, y, z, w) quaternion ``Rz(yaw) * q`` of a ``geometry_msgs/Quaternion``."""
    cz, sz = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return (
        cz * q.x - sz * q.y,
        cz * q.y + sz * q.x,
        cz * q.z + sz * q.w,
        cz * q.w - sz * q.z,
    )


def marker_dict(marker, frame):
    """Return a ``visualization_msgs/Marker`` as an :mod:`vectorfield_mujoco.overlay` dict.

    Args:
        marker: The message.
        frame: (x, y, z, yaw) of the marker's frame in the map (world) frame.

    """
    fx, fy, fz, fyaw = frame
    c, s = math.cos(fyaw), math.sin(fyaw)
    p = marker.pose.position
    position = (fx + c * p.x - s * p.y, fy + s * p.x + c * p.y, fz + p.z)
    return {
        'ns': marker.ns,
        'id': int(marker.id),
        'type': int(marker.type),
        'points': [(q.x, q.y, q.z) for q in marker.points],
        'pose': (position, frame_quat(fyaw, marker.pose.orientation)),
        'scale': (marker.scale.x, marker.scale.y, marker.scale.z),
        'rgba': (marker.color.r, marker.color.g, marker.color.b, marker.color.a),
        'colors': [(k.r, k.g, k.b, k.a) for k in marker.colors],
    }


def path_dict(path, frame, rgba, width=0.03):
    """Return a ``nav_msgs/Path`` as a LINE_STRIP overlay dict (``frame`` as in marker_dict)."""
    fx, fy, fz, fyaw = frame
    c, s = math.cos(fyaw), math.sin(fyaw)
    points = []
    for pose in path.poses:
        p = pose.pose.position
        points.append((fx + c * p.x - s * p.y, fy + s * p.x + c * p.y, fz + p.z))
    return {'type': 'LINE_STRIP', 'points': points, 'scale': (width, width, width), 'rgba': rgba}


class Overlays:
    """The markers of ``overlay_topics``, kept per topic and (namespace, id), in the map frame.

    Args:
        node: The node (subscriptions, graph queries, logging).
        topics: The topic names.
        frame_pose: Callable ``frame_pose(frame_id)`` returning (x, y, z, yaw) in the map frame,
            or None for an unknown frame.

    """

    def __init__(self, node, topics, frame_pose):
        self._node = node
        self._frame_pose = frame_pose
        self._pending = [node.resolve_topic_name(t) for t in topics if t]
        self._order = list(self._pending)
        self._markers = {}
        self._snapshot = []
        self._dirty = False
        self._subscriptions = []
        # Best effort and volatile: compatible with every publisher.
        self._qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        if self._pending:
            self._timer = node.create_timer(
                1.0, self._subscribe_pending, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )

    def _subscribe_pending(self):
        types = dict(self._node.get_topic_names_and_types())
        for topic in list(self._pending):
            names = types.get(topic, [])
            if 'visualization_msgs/msg/MarkerArray' in names:
                callback = self._marker_callback(topic)
                self._subscriptions.append(
                    self._node.create_subscription(MarkerArray, topic, callback, self._qos)
                )
            elif 'nav_msgs/msg/Path' in names:
                # By the topic's place in the list: the same colour whichever appears first.
                index = self._order.index(topic) % len(PATH_COLOURS)
                callback = self._path_callback(topic, PATH_COLOURS[index])
                self._subscriptions.append(
                    self._node.create_subscription(Path, topic, callback, self._qos)
                )
            elif names:
                self._node.get_logger().warning(
                    f'overlay topic {topic} is a {names[0]}, not a MarkerArray or Path; ignored'
                )
            else:
                continue
            self._pending.remove(topic)
            self._node.get_logger().info(f'overlay: drawing {topic}')
        if not self._pending:
            self._timer.cancel()

    def _frame(self, frame_id):
        frame = self._frame_pose(frame_id)
        if frame is None:
            self._node.get_logger().warning(
                f'overlay: unknown frame {frame_id!r}; markers skipped', throttle_duration_sec=10.0
            )
            return None
        x, y, z, yaw = frame
        return x, y, z + OVERLAY_LIFT, yaw

    def _marker_callback(self, topic):
        def callback(message):
            markers = self._markers.setdefault(topic, {})
            for marker in message.markers:
                key = (marker.ns, marker.id)
                if marker.action == Marker.DELETEALL:
                    markers.clear()
                elif marker.action == Marker.DELETE:
                    markers.pop(key, None)
                else:
                    frame = self._frame(marker.header.frame_id)
                    if frame is not None:
                        markers[key] = marker_dict(marker, frame)
            self._dirty = True

        return callback

    def _path_callback(self, topic, rgba):
        def callback(message):
            frame = self._frame(message.header.frame_id)
            if frame is not None:
                # Its namespace is the topic, which a view can hide.
                path = dict(path_dict(message, frame, rgba), ns=topic)
                self._markers[topic] = {('', 0): path}
                self._dirty = True

        return callback

    def markers(self):
        """Return the current markers, a list of overlay dicts (rebuilt when they change)."""
        if self._dirty:
            self._snapshot = [m for markers in self._markers.values() for m in markers.values()]
            self._dirty = False
        return self._snapshot

    def draw(self, scene, cap=overlay.DEFAULT_CAP, hide=()):
        """Append the current markers to an ``mjvScene``, but those ``hide`` names.

        ``hide`` holds marker namespaces, single ``ns/id`` markers and Path topics
        (:func:`vectorfield_mujoco.overlay.visible`).
        """
        return overlay.add_markers(scene, overlay.visible(self.markers(), hide), cap)


class GroundInterface:
    """vectorfield_sim's topics, frames and timing for a ground robot.

    Args:
        node: The :class:`MujocoSim` node (parameters are declared there).
        sim: The :class:`vectorfield_mujoco.simulation.Simulation`.

    """

    def __init__(self, node, sim):
        import mujoco

        from vectorfield_mujoco import sensors

        self.node = node
        self.sim = sim
        world = sim.world
        p = node.read_only
        self.map_frame = p('map_frame', 'map', 'Map frame')
        self.odom_frame = p('odom_frame', 'odom', 'Odometry frame')
        self.base_frame = p('base_frame', 'base_footprint', 'Robot base frame (odom child)')
        self.robot_frame = p('robot_frame', 'base_link', 'The base body frame')
        self.scan_frame = p('scan_frame', 'base_scan', 'Frame of the scan')
        cmd_vel_topic = p('cmd_vel_topic', 'cmd_vel', 'Command topic')
        stamped = p(
            'enable_stamped_cmd_vel',
            stamped_cmd_vel_default(),
            'Subscribe to TwistStamped instead of Twist (default: the distro Nav2 default)',
        )
        odom_rate = p('odom_rate', 50.0, 'odom, its TF and ~/ground_truth (Hz, simulated)')
        scan_rate = p('scan_rate', 10.0, 'Scan rate (Hz of simulated time)')
        beams = p('scan_beams', 360, 'Beams over a full turn from -pi')
        self.range_min = p('scan_range_min', 0.05, 'LaserScan range_min (m)')
        range_max = p('scan_range_max', 30.0, 'LaserScan range_max (m)')
        self.use_inf = p('scan_use_inf', True, 'No return is +inf (else range_max - 0.1)')
        latency = p('scan_latency', 0.02, 'Delay from a scan stamp to its delivery (s)')
        publish_robot_tf = p(
            'publish_robot_tf', True, 'Publish base_footprint -> base_link -> base_scan'
        )
        if not (odom_rate > 0.0 and scan_rate > 0.0 and beams >= 1):
            raise ValueError('odom_rate and scan_rate must be positive and scan_beams at least 1')

        tick = sim.tick_period
        self.odom_every = simulation.steps_per_period(1.0 / odom_rate, tick)
        self.scan_every = simulation.steps_per_period(1.0 / scan_rate, tick)
        self.scan_delay = int(math.floor(latency / tick + 0.5))
        self.pending_scans = []

        # The model's frames at rest: base_link above base_footprint, the lidar in base_link.
        mujoco.mj_forward(world.model, world.data)
        self.base_height = max(0.0, world.base_pose()[2])
        self.lidar = None
        self.scan_offset = (0.0, 0.0, 0.0, 0.0)
        if world.scan_site >= 0:
            rot = world.data.xmat[world.base_id].reshape(3, 3)
            offset = rot.T @ (
                world.data.site_xpos[world.scan_site] - world.data.xpos[world.base_id]
            )
            site_rot = rot.T @ world.data.site_xmat[world.scan_site].reshape(3, 3)
            self.scan_offset = (
                float(offset[0]),
                float(offset[1]),
                float(offset[2]),
                math.atan2(site_rot[1, 0], site_rot[0, 0]),
            )
            self.lidar = sensors.Lidar2D(
                world.model, world.scan_site, beams, self.range_min, range_max
            )
            self.scan_height = float(world.data.site_xpos[world.scan_site][2])
        else:
            self.scan_height = None
            node.get_logger().warning(f'{world.robot.key} has no lidar site: no scan')

        import tf2_ros

        self.tf = tf2_ros.TransformBroadcaster(node)
        self.static_tf = tf2_ros.StaticTransformBroadcaster(node)
        self.publish_robot_tf = publish_robot_tf
        self.odom_pub = node.create_publisher(Odometry, 'odom', 10)
        self.truth_pub = node.create_publisher(Odometry, '~/ground_truth', 10)
        self.scan_pub = node.create_publisher(
            LaserScan, p('scan_topic', 'scan', 'LaserScan topic'), qos_profile_sensor_data
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.infeasible_pub = node.create_publisher(UInt64, '~/infeasible_command_count', latched)
        if stamped:
            self.cmd_sub = node.create_subscription(
                TwistStamped, cmd_vel_topic, lambda m: self.on_command(m.twist), 10
            )
        else:
            self.cmd_sub = node.create_subscription(Twist, cmd_vel_topic, self.on_command, 10)
        self.pose_sub = node.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose, 10
        )
        from nav2_msgs.srv import SetInitialPose

        self.step_srv = node.create_service(
            SetInitialPose, '~/step_map_to_odom', self.on_step_map_to_odom
        )
        self.publish_static(to_stamp(sim.time))
        self.publish_infeasible()
        self.cmd_vel_topic = cmd_vel_topic
        self.stamped = stamped

    def describe(self):
        """Return one line about the interface, for the start-up log."""
        kind = 'TwistStamped' if self.stamped else 'Twist'
        scan = f'scan at {self.scan_height:.3f} m' if self.scan_height is not None else 'no scan'
        return f'{kind} on {self.cmd_vel_topic!r}, {scan}'

    def publish_static(self, stamp):
        """Publish map → odom and the robot's fixed frames on /tf_static."""
        x, y, yaw = self.sim.map_to_odom()
        transforms = [transform(self.map_frame, self.odom_frame, stamp, x, y, 0.0, yaw)]
        if self.publish_robot_tf:
            dx, dy, dz, dyaw = self.scan_offset
            transforms += [
                transform(
                    self.base_frame, self.robot_frame, stamp, 0.0, 0.0, self.base_height, 0.0
                ),
                transform(self.robot_frame, self.scan_frame, stamp, dx, dy, dz, dyaw),
            ]
        self.static_tf.sendTransform(transforms)

    def publish_infeasible(self):
        """Publish the infeasible command count (Ackermann)."""
        self.infeasible_pub.publish(UInt64(data=self.sim.infeasible_count))

    def frame_pose(self, frame_id):
        """Return (x, y, z, yaw) of a known frame in the map frame (the estimate), or None."""
        frame = frame_id.lstrip('/')
        if frame in ('', self.map_frame):
            return 0.0, 0.0, 0.0, 0.0
        map_to_odom = self.sim.map_to_odom()
        if frame == self.odom_frame:
            return (*map_to_odom[:2], 0.0, map_to_odom[2])
        base = simulation.compose(map_to_odom, self.sim.odom_pose())
        if frame == self.base_frame:
            return (*base[:2], 0.0, base[2])
        if frame == self.robot_frame:
            return (*base[:2], self.base_height, base[2])
        if frame == self.scan_frame:
            dx, dy, dz, dyaw = self.scan_offset
            x, y, yaw = simulation.compose(base, (dx, dy, dyaw))
            return x, y, self.base_height + dz, yaw
        return None

    def on_command(self, twist):
        """Take a command (Twist)."""
        try:
            infeasible = self.sim.set_command(twist.linear.x, twist.linear.y, twist.angular.z)
        except ValueError:
            self.node.get_logger().warning(
                'Ignoring a non-finite command', throttle_duration_sec=1.0
            )
            return
        if infeasible:
            self.node.get_logger().warning(
                f'Infeasible Ackermann command v = 0, omega = {twist.angular.z:.3f} rad/s '
                f'({self.sim.infeasible_count} so far)',
                throttle_duration_sec=1.0,
            )
            self.publish_infeasible()

    def on_initial_pose(self, message):
        """Teleport to ``initialpose`` (map frame)."""
        frame = message.header.frame_id.lstrip('/')
        if frame and frame != self.map_frame:
            self.node.get_logger().warning(
                f"Ignoring an initial pose in frame '{frame}' (expected '{self.map_frame}')"
            )
            return
        pose = message.pose.pose
        x, y, yaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.node.get_logger().warning('Ignoring a non-finite initial pose')
            return
        moved = self.sim.set_initial_pose(x, y, yaw)
        self.publish_static(to_stamp(self.sim.time))
        verb = 'teleported to' if moved else 'anchored at'
        self.node.get_logger().info(f'Initial pose: {verb} ({x:.3f}, {y:.3f}, {yaw:.3f} rad)')

    def on_step_map_to_odom(self, request, response):
        """Step the published map → odom by the request pose (a localization jump)."""
        pose = request.pose.pose.pose
        delta = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        if not all(math.isfinite(v) for v in delta):
            self.node.get_logger().error('Ignoring a non-finite map -> odom step')
            return response
        self.sim.step_map_to_odom(*delta)
        self.publish_static(to_stamp(self.sim.time))
        self.node.get_logger().info(
            'Stepped map -> odom by ({:.3f} m, {:.3f} m, {:.4f} rad)'.format(*delta)
        )
        return response

    def on_tick(self, stamp):
        """Publish what is due after a tick."""
        ticks = self.sim.ticks
        scan_due = self.lidar is not None and ticks % self.scan_every == 0
        if scan_due or ticks % self.odom_every == 0:
            pose = self.sim.odom_pose()
            twist = self.sim.body_twist()
            self.odom_pub.publish(odometry(self.odom_frame, self.base_frame, stamp, pose, twist))
            self.tf.sendTransform(
                transform(self.odom_frame, self.base_frame, stamp, pose[0], pose[1], 0.0, pose[2])
            )
            self.truth_pub.publish(
                odometry(self.map_frame, self.base_frame, stamp, self.sim.truth(), twist)
            )
        if scan_due:
            self.pending_scans.append((ticks + self.scan_delay, self.scan(stamp)))
        while self.pending_scans and self.pending_scans[0][0] <= ticks:
            self.scan_pub.publish(self.pending_scans.pop(0)[1])

    def scan(self, stamp):
        """Return the LaserScan of the current state."""
        world = self.sim.world
        ranges = self.lidar.scan(world.model, world.data)
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.scan_frame
        msg.angle_min = float(self.lidar.angle_min)
        msg.angle_max = float(self.lidar.angle_max)
        msg.angle_increment = float(self.lidar.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = float(self.scan_every * self.sim.tick_period)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.lidar.range_max)
        if not self.use_inf:
            ranges = np.where(np.isfinite(ranges), ranges, self.lidar.range_max - 0.1)
        msg.ranges = ranges.astype(np.float32).tolist()
        return msg


def _vec3(values):
    """Return three values as an array, or zeros for any other length (point_sim_3d's rule)."""
    return np.asarray(values, dtype=float) if len(values) == 3 else np.zeros(3)


class AerialInterface:
    """point_sim_3d's topics and frames for an aerial robot (see the module docstring).

    Args:
        node: The :class:`MujocoSim` node (parameters are declared there).
        sim: The :class:`vectorfield_mujoco.simulation.Simulation` of a quadrotor.

    """

    #: No lidar: every obstacle reaches the follower through the cloud.
    scan_height = None

    def __init__(self, node, sim):
        import mujoco
        import tf2_ros

        from vectorfield_mujoco import sensors

        self.node = node
        self.sim = sim
        world = sim.world
        p = node.read_only
        self.global_frame = p('global_frame', 'map', 'World frame (ENU) of odom, TF and the cloud')
        self.base_frame = p(
            'base_frame', 'base_link', 'Robot frame; a cmd_vel in it is in the heading frame'
        )
        self.cmd_vel_topic = p('cmd_vel_topic', 'cmd_vel', 'TwistStamped command topic')
        odom_topic = p('odom_topic', 'odom', 'Odometry topic')
        odom_rate = p('odom_rate', 50.0, 'odom and its TF (Hz, simulated)')
        cloud_topic = p('cloud_topic', 'obstacle_cloud', 'PointCloud2 of the world obstacles')
        cloud_rate = p('cloud_rate', 2.0, 'Obstacle cloud rate (Hz, simulated)')
        spacing = p('cloud_spacing', 0.05, 'Spacing of the cloud points on the surfaces (m)')
        self.moving_frame = p(
            'moving_frame', '', 'A frame moving with a constant twist; empty: none'
        )
        frame_topic = p('moving_frame_topic', 'moving_frame/odom', 'Odometry of moving_frame')
        self.frame_origin = _vec3(
            p('moving_frame_origin', [0.0, 0.0, 0.0], 'Start of moving_frame in global_frame (m)')
        )
        self.frame_linear = _vec3(
            p('moving_frame_linear', [0.0, 0.0, 0.0], 'Linear velocity of moving_frame (own axes)')
        )
        self.frame_angular = _vec3(
            p('moving_frame_angular', [0.0, 0.0, 0.0], 'Angular velocity of moving_frame (own)')
        )
        if not (odom_rate > 0.0 and cloud_rate > 0.0 and spacing > 0.0):
            raise ValueError('odom_rate, cloud_rate and cloud_spacing must be positive')

        tick = sim.tick_period
        self.odom_every = simulation.steps_per_period(1.0 / odom_rate, tick)
        self.cloud_every = simulation.steps_per_period(1.0 / cloud_rate, tick)
        # The world's obstacles and walls do not move: their surface points are sampled once.
        mujoco.mj_forward(world.model, world.data)
        self.cloud = sensors.world_cloud(world.model, world.data, spacing)

        self.tf = tf2_ros.TransformBroadcaster(node)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = node.create_publisher(Odometry, odom_topic, reliable)
        self.cloud_pub = node.create_publisher(
            PointCloud2, cloud_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.frame_pub = (
            node.create_publisher(Odometry, frame_topic, reliable) if self.moving_frame else None
        )
        self.cmd_sub = node.create_subscription(
            TwistStamped, self.cmd_vel_topic, self.on_command, reliable
        )

    def describe(self):
        """Return one line about the interface, for the start-up log."""
        moving = f', moving frame {self.moving_frame!r}' if self.moving_frame else ''
        return (
            f'TwistStamped on {self.cmd_vel_topic!r} (heading frame when its frame_id is '
            f'{self.base_frame!r}, else {self.global_frame!r}), obstacle cloud of '
            f'{len(self.cloud)} points{moving}'
        )

    def moving_pose(self):
        """Return (rotation, translation) of the moving frame in the global frame now."""
        rotation, translation = simulation.exp_se3(
            self.frame_linear, self.frame_angular, self.sim.time
        )
        return rotation, self.frame_origin + translation

    def frame_pose(self, frame_id):
        """Return (x, y, z, yaw) of a known frame in the global frame, or None.

        Overlays are drawn turned by the yaw only (marker_dict), which suits markers in the
        global frame, as the 3D follower's are.
        """
        frame = frame_id.lstrip('/')
        if frame in ('', self.global_frame):
            return 0.0, 0.0, 0.0, 0.0
        if frame == self.base_frame:
            return self.sim.world.base_pose()
        if self.moving_frame and frame == self.moving_frame:
            rotation, (x, y, z) = self.moving_pose()
            return float(x), float(y), float(z), math.atan2(rotation[1, 0], rotation[0, 0])
        return None

    def on_command(self, message):
        """Take a command: heading frame when its frame_id is the base frame, else global."""
        linear = message.twist.linear
        try:
            self.sim.set_velocity(
                (linear.x, linear.y, linear.z),
                message.twist.angular.z,
                heading=message.header.frame_id == self.base_frame,
            )
        except ValueError:
            self.node.get_logger().warning(
                'Ignoring a non-finite command', throttle_duration_sec=1.0
            )

    def on_tick(self, stamp):
        """Publish what is due after a tick."""
        ticks = self.sim.ticks
        if ticks % self.odom_every == 0:
            self.publish_state(stamp)
        if ticks % self.cloud_every == 0:
            self.cloud_pub.publish(point_cloud(self.cloud, self.global_frame, stamp))

    def publish_state(self, stamp):
        """Publish odom, the TF of the base and, if any, the moving frame."""
        world = self.sim.world
        position = world.data.xpos[world.base_id]
        w, x, y, z = world.data.xquat[world.base_id]
        linear, angular = world.base_twist()
        transforms = [
            transform3(self.global_frame, self.base_frame, stamp, position, (x, y, z, w))
        ]
        self.odom_pub.publish(
            odometry3(
                self.global_frame, self.base_frame, stamp, position, (x, y, z, w), linear, angular
            )
        )
        if self.frame_pub is not None:
            rotation, translation = self.moving_pose()
            quat = matrix_quat(rotation)
            transforms.append(
                transform3(self.global_frame, self.moving_frame, stamp, translation, quat)
            )
            self.frame_pub.publish(
                odometry3(
                    self.global_frame,
                    self.moving_frame,
                    stamp,
                    translation,
                    quat,
                    self.frame_linear,
                    self.frame_angular,
                )
            )
        self.tf.sendTransform(transforms)


class MujocoSim(Node):
    """The ``mujoco_sim`` node (see the module docstring)."""

    def __init__(self):
        super().__init__('mujoco_sim')
        from vectorfield_mujoco import scene

        robot = registry.get(self.read_only('robot', 'scout_mini', 'Robot key (registry)'))
        world_map = self.read_only('world_map', '', 'Nav2 map YAML whose walls are extruded')
        obstacles = simulation.parse_obstacles(
            self.read_only('obstacles', '', 'Unmapped obstacles, a YAML list (map frame)')
        )
        pose = tuple(
            self.read_only(f'initial_pose_{axis}', 0.0, f'Initial map {axis}')
            for axis in ('x', 'y', 'yaw')
        )
        physics_rate = self.read_only('physics_rate', 500.0, 'Physics steps per simulated s')
        clock_rate = self.read_only('clock_rate', 100.0, '/clock and tick rate (Hz, simulated)')
        real_time_factor = self.read_only(
            'real_time_factor', 1.0, 'Simulated seconds per wall second; 0 stops the timer'
        )
        wall_height = self.read_only('wall_height', scene.WALL_HEIGHT, 'Map wall height (m)')
        cmd_timeout = self.read_only(
            'cmd_vel_timeout', 0.5, 'A command older than this (simulated s) is zero; 0: never'
        )
        if not cmd_timeout >= 0.0:
            raise ValueError('cmd_vel_timeout must be 0 (never) or positive')
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('initial_pose_x, initial_pose_y and initial_pose_yaw must be finite')
        aerial = robot.adapter == 'quadrotor'
        altitude = 0.0
        if aerial:
            # point_sim_3d's initial_z: the height of base_link (the MJCF's own clearance under
            # it, spawn_height, is part of it; 0 rests on the floor).
            height = self.read_only('initial_pose_z', 1.0, 'Initial base height (m), aerial')
            if not math.isfinite(height):
                raise ValueError('initial_pose_z must be finite')
            altitude = max(0.0, height - (robot.spawn_height or 0.0))

        menagerie_dir = None
        if robot.source == 'menagerie':
            if not menagerie.is_available():
                self.get_logger().info(
                    f'{robot.key} comes from the MuJoCo Menagerie, which is not downloaded yet: '
                    'fetching it once (about 20 MB, a sparse git clone)'
                )
            menagerie_dir = menagerie.ensure_menagerie(log=self.get_logger().info)
        world = scene.make_world(
            robot.key,
            map_yaml=world_map or None,
            obstacles=obstacles,
            pose=pose,
            altitude=altitude,
            wall_height=wall_height,
            menagerie_dir=menagerie_dir,
        )
        self.sim = simulation.Simulation(world, physics_rate, clock_rate, cmd_timeout)
        self.interface = (AerialInterface if aerial else GroundInterface)(self, self.sim)
        if self.interface.scan_height is not None:
            for i in simulation.unseen_obstacles(obstacles, self.interface.scan_height):
                self.get_logger().warning(
                    f'obstacle {i} {obstacles[i]} does not cross the lidar plane '
                    f'({self.interface.scan_height:.3f} m): Nav2 cannot see it'
                )
        self.clock_pub = self.create_publisher(
            ClockMsg, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.overlays = Overlays(
            self,
            self.read_only('overlay_topics', [''], 'MarkerArray and Path topics drawn in scenes'),
            self.interface.frame_pose,
        )
        self._setup_views(world)

        walls = sum(1 for g in range(world.model.ngeom) if world.model.geom(g).name[:5] == 'wall_')
        geometry = ', '.join(f'{k} {v:.4f}' for k, v in world.geometry.items())
        if aerial:
            geometry = f'{world.model.body_subtreemass[world.base_id]:.3f} kg'
        self.get_logger().info(
            f'{robot.description}; {geometry or "planar base"}; '
            f'{os.path.basename(world_map) or "no map"} ({walls} wall boxes), '
            f'{len(obstacles)} obstacle(s); {physics_rate:g} Hz physics in '
            f'{self.sim.substeps}-step ticks at {real_time_factor:.2f}x real time; '
            f'{self.interface.describe()}; GL {os.environ.get("MUJOCO_GL", "")}'
        )
        self._timer = None
        if real_time_factor > 0.0:
            self._timer = self.create_timer(
                self.sim.tick_period / real_time_factor,
                self._on_tick,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
        self._publish_clock()

    def read_only(self, name, default, description):
        """Declare a read-only parameter; return its value."""
        descriptor = ParameterDescriptor(description=description, read_only=True)
        return self.declare_parameter(name, default, descriptor).value

    def _setup_views(self, world):
        from vectorfield_mujoco import recorder

        camera_topic = self.read_only(
            'camera_topic', '', 'Offscreen camera Image topic; empty: off'
        )
        camera_rate = self.read_only('camera_rate', 10.0, 'Camera images per simulated s')
        if not camera_rate > 0.0:
            raise ValueError('camera_rate must be positive')
        camera_size = simulation.parse_size(self.read_only('camera_size', '640x360', 'WxH'))
        camera_mode = self.read_only('camera_mode', 'chase', 'chase, follow, top or fixed')
        record_path = self.read_only('record_path', '', 'MP4 to record; empty: off')
        record_fps = self.read_only('record_fps', 30.0, 'Recorded frames per simulated s')
        record_size = simulation.parse_size(self.read_only('record_size', '960x540', 'WxH'))
        record_camera = self.read_only('record_camera', 'chase', 'chase, follow, top or fixed')
        record_view = recorder.parse_view(
            self.read_only(
                'record_view',
                '',
                'YAML mapping overriding the recording camera (lookat, distance, azimuth, '
                'elevation, height, smoothing); empty: the camera defaults',
            )
        )
        record_hide = [
            name
            for name in self.read_only(
                'record_hide',
                [''],
                'Marker namespaces, ns/id markers and Path topics not recorded',
            )
            if name
        ]
        self._trail_span = self.read_only(
            'record_trail', 0.0, "Seconds of the robot's path drawn in the recording; 0: none"
        )
        self._trail = collections.deque()
        viewer = self.read_only('viewer', False, 'Open the MuJoCo passive viewer')

        self._camera = None
        self._recorder = None
        self._viewer = None
        # Whether a viewer was opened (main() then exits without the interpreter's teardown).
        self.viewer_used = False
        self._viewer_threads = []
        requested = [
            name
            for name, on in (
                ('viewer', viewer),
                ('camera_topic', camera_topic),
                ('record_path', record_path),
            )
            if on
        ]
        # Without an OpenGL backend only the rendering features are refused; the physics and
        # every topic of the interface run (vectorfield_mujoco.gl).
        reason = gl.unavailable_reason(os.environ.get('MUJOCO_GL'))
        if requested and reason:
            self.get_logger().error(f'{", ".join(requested)} off: {reason}')
            return
        if camera_topic:
            import mujoco

            width, height = camera_size
            try:
                renderer = mujoco.Renderer(world.model, height=height, width=width, max_geom=10000)
            except Exception as e:  # any GL failure: the backend cannot render here
                self.get_logger().error(f'camera_topic off: cannot render ({e})')
            else:
                self._camera = (
                    renderer,
                    self._camera_rig(world, camera_mode),
                    self.create_publisher(Image, camera_topic, qos_profile_sensor_data),
                )
                self._camera_every = simulation.steps_per_period(
                    1.0 / camera_rate, self.sim.tick_period
                )
        if record_path:
            path = os.path.abspath(os.path.expanduser(record_path))
            os.makedirs(os.path.dirname(path), exist_ok=True)
            width, height = record_size
            try:
                self._recorder = recorder.Recorder(
                    world.model,
                    path,
                    width,
                    height,
                    record_fps,
                    camera=self._camera_rig(world, record_camera, record_view),
                    overlay=lambda scn: self._draw_recording(scn, record_hide),
                )
            except Exception as e:  # no ffmpeg, or a GL failure
                self.get_logger().error(f'record_path off: {e}')
            else:
                self.get_logger().info(
                    f'recording {width}x{height} at {record_fps:g} fps (simulated time) to {path}'
                )
        if viewer:
            self._open_viewer(world)

    def _draw_recording(self, scn, hide):
        """Draw the overlays but ``hide`` and the robot's path of the last ``record_trail`` s."""
        added = self.overlays.draw(scn, hide=hide)
        if not self._trail_span > 0.0:
            return added
        world = self.sim.world
        position = world.data.xpos[world.base_id].copy()
        if world.robot.adapter != 'quadrotor':
            position[2] = OVERLAY_LIFT  # a ground robot's path, on the floor
        trail = self._trail
        if not trail or np.linalg.norm(position - trail[-1][1]) > 0.01:
            trail.append((world.data.time, position))
        while len(trail) > 2 and world.data.time - trail[0][0] > self._trail_span:
            trail.popleft()
        if len(trail) < 2:
            return added
        path = {
            'ns': 'trail',
            'type': 'LINE_STRIP',
            'points': [p for _, p in trail],
            'scale': (TRAIL_WIDTH, TRAIL_WIDTH, TRAIL_WIDTH),
            'rgba': TRAIL_COLOUR,
        }
        return added + overlay.add_markers(scn, [path])

    def _camera_rig(self, world, mode, view=None):
        """Return the ``mode`` camera, its defaults overridden by ``view`` (parse_view)."""
        from vectorfield_mujoco import recorder

        if world.robot.adapter == 'quadrotor':
            # A drone flies around its curve, a few metres across and up to 2 m high.
            defaults = {
                'fixed': {
                    'distance': 4.8,
                    'elevation': -20.0,
                    'azimuth': -60.0,
                    'lookat': (0.0, 0.0, 0.9),
                },
                'top': {'distance': 5.0, 'height': 0.0},
                'chase': {'distance': 3.5, 'elevation': -18.0, 'height': 0.0},
                'follow': {'distance': 3.5, 'elevation': -18.0, 'azimuth': 90.0, 'height': 0.0},
            }
        else:
            defaults = {
                'fixed': {'distance': 9.0, 'elevation': -50.0, 'azimuth': 90.0},
                'top': {'distance': 6.0},
                'chase': {'distance': 2.8, 'elevation': -28.0},
                'follow': {'distance': 3.0, 'elevation': -35.0, 'azimuth': 90.0},
            }
        settings = dict(defaults.get(mode, {}), **(view or {}))
        return recorder.CameraRig(mode, world.base_id, **settings)

    def _open_viewer(self, world):
        import mujoco

        if os.environ.get('MUJOCO_GL', 'glfw') != 'glfw':
            self.get_logger().error(
                f'viewer needs a display and MUJOCO_GL=glfw (it is {os.environ["MUJOCO_GL"]}); '
                'use camera_topic or record_path on a headless host'
            )
            return
        import mujoco.viewer

        from vectorfield_mujoco import scene

        before = set(threading.enumerate())
        try:
            self._viewer = mujoco.viewer.launch_passive(
                world.model, world.data, show_left_ui=False, show_right_ui=False
            )
        except Exception as e:  # no display after all
            self.get_logger().error(f'viewer off: {e}')
            return
        self.viewer_used = True
        # launch_passive renders in a thread of its own (except on macOS) and does not return
        # it; close() joins it. Handle.is_running() turns False as soon as close() asks the
        # viewer to exit, long before that thread has dropped its window and GL context.
        self._viewer_threads = [t for t in threading.enumerate() if t not in before]
        with self._viewer.lock():
            self._viewer.opt.geomgroup[:] = scene.visual_option().geomgroup
            self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self._viewer.cam.trackbodyid = world.base_id
            self._viewer.cam.distance = 3.0
            self._viewer.cam.elevation = -30.0
        self._viewer_period = 1.0 / VIEWER_RATE
        self._viewer_next = 0.0

    def _publish_clock(self):
        self.clock_pub.publish(ClockMsg(clock=to_stamp(self.sim.time)))

    def _capture(self, data):
        try:
            self._recorder.maybe_capture(data)
        except Exception as e:  # a GL or ffmpeg failure ends the recording, not the physics
            self.get_logger().error(f'recording stopped: {e}')
            recorder, self._recorder = self._recorder, None
            with contextlib.suppress(Exception):  # the failure is reported above
                recorder.close()

    def _on_tick(self):
        on_substep = self._capture if self._recorder is not None else None
        if self._viewer is not None and self._viewer.is_running():
            with self._viewer.lock():
                self.sim.step(on_substep)
        else:
            self.sim.step(on_substep)
        stamp = to_stamp(self.sim.time)
        self.clock_pub.publish(ClockMsg(clock=stamp))
        self.interface.on_tick(stamp)
        if self._camera is not None and self.sim.ticks % self._camera_every == 0:
            try:
                self._publish_camera(stamp)
            except Exception as e:  # a GL failure ends the camera, not the physics
                self.get_logger().error(f'camera_topic stopped: {e}')
                self._camera = None
        if self._viewer is not None:
            self._sync_viewer()

    def _publish_camera(self, stamp):
        from vectorfield_mujoco import scene

        renderer, rig, publisher = self._camera
        data = self.sim.world.data
        renderer.update_scene(data, rig.update(data), scene.visual_option())
        self.overlays.draw(renderer.scene)
        pixels = renderer.render()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'mujoco_camera'
        msg.height, msg.width = int(pixels.shape[0]), int(pixels.shape[1])
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.data = array.array('B', np.ascontiguousarray(pixels).tobytes())  # see point_cloud()
        publisher.publish(msg)

    def _sync_viewer(self):
        import time

        now = time.monotonic()
        if now < self._viewer_next:
            return
        self._viewer_next = now + self._viewer_period
        if not self._viewer.is_running():
            self.get_logger().info('viewer closed; the simulation goes on')
            self._viewer = None
            return
        with self._viewer.lock():
            self._viewer.user_scn.ngeom = 0
            self.overlays.draw(self._viewer.user_scn)
        self._viewer.sync()

    def close(self):
        """Finish the recording and close the views."""
        if self._timer is not None:
            self._timer.cancel()
        if self._recorder is not None:
            recorder, self._recorder = self._recorder, None
            try:
                recorder.close()
            except RuntimeError as e:
                self.get_logger().error(f'recording to {recorder.path} failed: {e}')
            else:
                self.get_logger().info(f'recorded {recorder.frames} frames to {recorder.path}')
        if self._camera is not None:
            self._camera[0].close()
            self._camera = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        # Wait until the viewer's thread has dropped its window and GL context (also after the
        # window was closed by hand), so that none of it runs while the process exits.
        deadline = time.monotonic() + VIEWER_CLOSE_TIMEOUT
        for thread in self._viewer_threads:
            thread.join(max(0.0, deadline - time.monotonic()))
        self._viewer_threads = []


def main(args=None):
    """Run the node until it is shut down; the recording is finished either way."""
    gl.configure()  # Before anything imports mujoco: MuJoCo reads MUJOCO_GL once.
    rclpy.init(args=args)
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:  # pragma: no cover - very old rclpy
        ExternalShutdownException = KeyboardInterrupt  # noqa: N806
    node = None
    try:
        node = MujocoSim()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A SIGINT can shut rclpy down while its executor takes a message, and take_message
        # then raises ("Unable to convert call argument"); only an error while running is one.
        if node is None or rclpy.ok():
            raise
    finally:
        viewer_used = False
        if node is not None:
            node.close()
            viewer_used = node.viewer_used
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if viewer_used:
        # Everything is closed and flushed; the viewer's GLFW thread can still crash the
        # interpreter's teardown (a segfault at the end of an example with viewer:=true), so
        # leave without it.
        os._exit(0)


if __name__ == '__main__':
    main()
