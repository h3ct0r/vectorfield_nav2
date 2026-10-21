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

r"""Identify the slip of a skid-steer base: ``skid_yaw_rate_efficiency`` and ``skid_icr_offset``.

**Online** (the default) the script drives the robot itself: a short sequence of constant
(v, w) segments (``--sequence``, magnitudes ``--v`` and ``--w``), each ``--segment-time`` s with
a ``--pause`` of zero command in between. It shows the plan and asks for ``yes`` first (``--yes``
skips the prompt), stops with a zero command on Ctrl-C, when the robot leaves a circle of
``--max-distance`` m around its start, after ``--max-time`` s, or when the pose stops arriving,
and scales the commands by Nav2's ``speed_limit`` (keeping the curvature). Commands go to
``--cmd-topic`` (``cmd_vel_nav``: through Nav2's velocity smoother and collision monitor); the
command the base received is read back from ``--final-topic`` when it is published.

**Offline** (``--bag``) the same estimate runs on a recorded bag (``record_field_test.py``):
segments of constant command are found in ``--final-topic`` (else ``--cmd-topic``).

The pose comes from ``--odom-topic`` or, better, from ``--pose-topic`` (a localisation pose,
motion capture or IMU-fused odometry: ``Odometry``, ``PoseStamped`` or
``PoseWithCovarianceStamped``). Plain wheel odometry computes the motion from the wheel speeds
and so never sees the slip: it would report :math:`\eta_\psi = 1`, :math:`x_{ICR} = 0`.

Per segment it prints the realised over commanded yaw rate :math:`\eta_\psi`, the ICR offset
:math:`x_{ICR} = -v_y/\omega` from the lateral drift of the pose, and the realised over
commanded forward speed (a diagnostic), then the pooled values with their spread and the
parameter lines to paste (``vectorfield_bringup.slip``, ``docs/motion_models.md``).

Examples::

    ros2 run vectorfield_bringup identify_skid_slip.py --v 0.2 --w 0.6
    ros2 run vectorfield_bringup identify_skid_slip.py --pose-topic /amcl_pose --repeats 2
    ros2 run vectorfield_bringup identify_skid_slip.py --bag field_test_20261021_101500

Exit status: 0 with an estimate, 1 without (too few data, aborted), 2 for invalid arguments,
130 after Ctrl-C.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

from vectorfield_bringup import slip

DEFAULT_SEQUENCE = '0:w,0:-w,v:w,v:-w'
#: The pose is stale (the robot stops) after this many seconds without a message.
POSE_TIMEOUT = 0.5


def parse_sequence(text, v, w):
    """Return [(v, w)] from ``V:W`` items; V and W are numbers or v, -v, w, -w (and +v, +w)."""
    symbols = {'v': v, '+v': v, '-v': -v, 'w': w, '+w': w, '-w': -w, '0': 0.0}
    segments = []
    for item in text.split(','):
        item = item.strip()
        if not item:
            continue
        parts = item.split(':')
        if len(parts) != 2:
            raise ValueError(f'"{item}": expected V:W')
        values = []
        for part in parts:
            part = part.strip()
            values.append(symbols[part] if part in symbols else float(part))
        segments.append(tuple(values))
    if not segments:
        raise ValueError('empty sequence')
    return segments


def excursion(v, w, duration):
    """Return the largest distance from its start of a constant (v, w) arc of ``duration`` s."""
    if abs(v) < 1e-9:
        return 0.0
    if abs(w) < 1e-9:
        return abs(v) * duration
    radius = abs(v / w)
    angle = min(abs(w) * duration, math.pi)
    return 2.0 * radius * math.sin(0.5 * angle)


def parse_arguments(argv):
    """Parse the command line (ROS arguments already removed)."""
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        epilog='See docs/tutorials/field_testing.md, stage 6.',
    )
    parser.add_argument('--v', type=float, default=0.2, help='linear speed magnitude (m/s)')
    parser.add_argument('--w', type=float, default=0.6, help='yaw rate magnitude (rad/s)')
    parser.add_argument(
        '--sequence',
        default=DEFAULT_SEQUENCE,
        help=f'segments V:W, comma separated; V and W are numbers or v, -v, w, -w '
        f'(default {DEFAULT_SEQUENCE}: turn in place both ways, then arcs both ways)',
    )
    parser.add_argument('--repeats', type=int, default=1, help='run the sequence this many times')
    parser.add_argument('--segment-time', type=float, default=4.0, help='seconds per segment')
    parser.add_argument(
        '--settle', type=float, default=1.0, help='seconds skipped at the start of each segment'
    )
    parser.add_argument(
        '--pause', type=float, default=1.0, help='zero command between segments (s)'
    )
    parser.add_argument('--cmd-topic', default='cmd_vel_nav', help='topic the commands go to')
    parser.add_argument(
        '--final-topic',
        default='cmd_vel',
        help='the command the base receives, used as the commanded value ("" to use ours)',
    )
    parser.add_argument('--odom-topic', default='odom', help='Odometry topic')
    parser.add_argument(
        '--pose-topic', default='', help='pose source that sees the slip ("" = --odom-topic)'
    )
    parser.add_argument(
        '--stamped',
        choices=('auto', 'on', 'off'),
        default='auto',
        help="publish TwistStamped (auto: as the topic's subscribers, else the distro default)",
    )
    parser.add_argument('--rate', type=float, default=20.0, help='command rate (Hz)')
    parser.add_argument('--max-distance', type=float, default=2.0, help='stop beyond this (m)')
    parser.add_argument('--max-time', type=float, default=90.0, help='stop after this (s)')
    parser.add_argument('--yes', action='store_true', help='do not ask for confirmation')
    parser.add_argument('--use-sim-time', action='store_true', help='use /clock (simulation)')
    parser.add_argument(
        '--control-point-offset',
        type=float,
        default=0.2,
        help="the controller's control_point_offset, to check |x_ICR| < d / 2",
    )
    parser.add_argument('--json', help='write the estimate and the segments to this JSON file')
    parser.add_argument('--bag', help='offline: estimate from this recorded bag instead')
    parser.add_argument(
        '--min-segment', type=float, default=2.0, help='offline: shortest constant segment (s)'
    )
    args = parser.parse_args(argv)
    if args.v < 0.0 or args.w < 0.0:
        parser.error('--v and --w are magnitudes (>= 0); signs go in --sequence')
    if args.v > 1.0 or args.w > 2.0:
        parser.error('--v above 1 m/s or --w above 2 rad/s is not a slip identification')
    if args.segment_time <= args.settle + 0.5:
        parser.error('--segment-time must exceed --settle by more than 0.5 s')
    if args.rate <= 0.0 or args.repeats < 1:
        parser.error('--rate must be positive and --repeats at least 1')
    try:
        args.segments = parse_sequence(args.sequence, args.v, args.w) * args.repeats
    except (KeyError, ValueError) as error:
        parser.error(f'--sequence: {error}')
    return args


def report(result, args):
    """Print the segments and the estimate; return the exit status."""
    print('\nsegment  v_cmd   w_cmd  w_real  eta_psi   x_icr   eta_v  pairs  note')
    for i, s in enumerate(result.segments, 1):
        print(
            f'{i:7d} {s.v_cmd:6.3f} {s.w_cmd:7.3f} {s.w_real:7.3f} {s.eta_psi:8.3f} '
            f'{s.x_icr:7.3f} {s.eta_v:7.3f} {s.pairs:6d}  {s.note}'
        )
    print(
        f'\neta_psi (skid_yaw_rate_efficiency) = {result.eta_psi:.3f} '
        f'(spread {result.eta_psi_spread:.3f})'
    )
    print(
        f'x_icr   (skid_icr_offset)          = {result.x_icr:.3f} m '
        f'(spread {result.x_icr_spread:.3f})'
    )
    print(f'eta_v   (forward speed, diagnostic) = {result.eta_v:.3f}')
    if math.isfinite(result.eta_psi) and result.eta_psi > 1.05:
        print(
            'warning: eta_psi > 1: the base turns faster than commanded. Check the pose source '
            'and the commanded reference (--final-topic).'
        )
    lines = slip.yaml_lines(result, args.control_point_offset)
    if lines:
        print('\nParameters (FollowPath and the curve follower):\n')
        print('\n'.join(f'  {line}' for line in lines))
    if args.json:
        data = {
            'eta_psi': result.eta_psi,
            'eta_psi_spread': result.eta_psi_spread,
            'x_icr': result.x_icr,
            'x_icr_spread': result.x_icr_spread,
            'eta_v': result.eta_v,
            'segments': [s.__dict__ for s in result.segments],
        }
        with open(args.json, 'w') as stream:
            json.dump(data, stream, indent=2, default=float)
        print(f'\nwrote {args.json}')
    usable = math.isfinite(result.eta_psi) or math.isfinite(result.x_icr)
    return 0 if usable else 1


# Offline -------------------------------------------------------------------------------------


def offline(args):
    """Estimate from a recorded bag."""
    import numpy as np

    from vectorfield_bringup import bag

    def absolute(topic):
        return topic if topic.startswith('/') else f'/{topic}'

    reader = bag.open_reader(args.bag)
    types = bag.topic_types(reader)
    del reader
    pose_topic = absolute(args.pose_topic or args.odom_topic)
    command_topic = ''
    for candidate in (args.final_topic, args.cmd_topic):
        if candidate and absolute(candidate) in types:
            command_topic = absolute(candidate)
            break
    if pose_topic not in types or not command_topic:
        print(
            f'{args.bag}: needs {pose_topic} and one of {args.final_topic}, {args.cmd_topic}; '
            f'it has {sorted(types)}',
            file=sys.stderr,
        )
        return 1
    print(f'pose from {pose_topic}, commands from {command_topic}')
    poses, commands, pairs = [], [], []
    for topic, message, recv in bag.read_messages(args.bag, [pose_topic, command_topic]):
        if topic == pose_topic:
            stamp = bag.stamp_seconds(message.header.stamp)
            pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
            poses.append((stamp, pose.position.x, pose.position.y, bag.yaw_of(pose.orientation)))
            pairs.append((recv, stamp))
        else:
            twist = message.twist if hasattr(message, 'header') else message
            commands.append((recv, twist.linear.x, twist.angular.z))
    time_map = bag.TimeMap([p[0] for p in pairs], [p[1] for p in pairs])
    poses = np.array(sorted(poses), dtype=float).reshape(-1, 4)
    commands = np.array(commands, dtype=float).reshape(-1, 3)
    if len(poses) < 10 or len(commands) < 10:
        print('too few poses or commands in the bag', file=sys.stderr)
        return 1
    commands[:, 0] = time_map(commands[:, 0])
    segments = [
        slip.Segment(t0, t1, v, w)
        for t0, t1, v, w in slip.constant_segments(
            commands[:, 0], commands[:, 1], commands[:, 2], min_duration=args.min_segment
        )
    ]
    if not segments:
        print('no segment of constant command found', file=sys.stderr)
        return 1
    print(f'{len(segments)} segments of constant command')
    result = slip.estimate(
        segments, poses.T, commands.T, settle=min(args.settle, 0.5 * args.min_segment)
    )
    return report(result, args)


# Online --------------------------------------------------------------------------------------


class Driver:
    """The node: publishes the commands, records the pose and the received commands."""

    def __init__(self, args):
        import rclpy
        from geometry_msgs.msg import Twist, TwistStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self.args = args
        self.node = rclpy.create_node(
            'identify_skid_slip',
            parameter_overrides=[Parameter('use_sim_time', value=args.use_sim_time)],
        )
        self._lock = threading.Lock()
        self.poses = []  # (receive t, stamp, x, y, yaw)
        self.received = []  # (receive t, v, w) of the final command
        self.sent = []  # (t, v, w)
        self.speed_limit = None
        self._twist_classes = (Twist, TwistStamped)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    def now(self):
        """Return the node clock in seconds."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def topic_types(self, topic, timeout=5.0):
        """Return the types of a topic, waiting up to ``timeout`` wall s for it to appear."""
        name = self.node.resolve_topic_name(topic)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            types = dict(self.node.get_topic_names_and_types()).get(name)
            if types:
                return types
            time.sleep(0.2)
        return []

    def setup(self):
        """Create the subscriptions and the publisher; return an error text or ''."""
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav2_msgs.msg import SpeedLimit
        from nav_msgs.msg import Odometry

        pose_topic = self.args.pose_topic or self.args.odom_topic
        types = self.topic_types(pose_topic)
        classes = {
            'nav_msgs/msg/Odometry': Odometry,
            'geometry_msgs/msg/PoseStamped': PoseStamped,
            'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped,
        }
        pose_class = next((classes[t] for t in types if t in classes), None)
        if pose_class is None:
            return f'{pose_topic}: no Odometry, PoseStamped or PoseWithCovarianceStamped ({types})'
        self.pose_topic = pose_topic
        self.node.create_subscription(pose_class, pose_topic, self._on_pose, self._sensor_qos)
        self.node.create_subscription(
            SpeedLimit, 'speed_limit', self._on_speed_limit, self._reliable_qos
        )
        if self.args.final_topic:
            final_types = self.topic_types(self.args.final_topic, timeout=1.0)
            twist_class = self._twist_class(final_types, default=None)
            if twist_class is not None:
                stamped = twist_class is self._twist_classes[1]
                self.node.create_subscription(
                    twist_class,
                    self.args.final_topic,
                    lambda m, s=stamped: self._on_final(m.twist if s else m),
                    self._reliable_qos,
                )
        self.stamped = self._choose_stamped()
        self.publisher = self.node.create_publisher(
            self._twist_classes[1] if self.stamped else self._twist_classes[0],
            self.args.cmd_topic,
            self._reliable_qos,
        )
        return ''

    def _twist_class(self, types, default):
        if 'geometry_msgs/msg/TwistStamped' in types:
            return self._twist_classes[1]
        if 'geometry_msgs/msg/Twist' in types:
            return self._twist_classes[0]
        return default

    def _choose_stamped(self):
        if self.args.stamped != 'auto':
            return self.args.stamped == 'on'
        name = self.node.resolve_topic_name(self.args.cmd_topic)
        infos = self.node.get_subscriptions_info_by_topic(name)
        types = {info.topic_type for info in infos}
        if 'geometry_msgs/msg/TwistStamped' in types:
            return True
        if 'geometry_msgs/msg/Twist' in types:
            return False
        return os.environ.get('ROS_DISTRO', '') not in ('humble', 'iron', 'jazzy')

    def _on_pose(self, message):
        from vectorfield_bringup import bag

        pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
        sample = (
            self.now(),
            bag.stamp_seconds(message.header.stamp),
            pose.position.x,
            pose.position.y,
            bag.yaw_of(pose.orientation),
        )
        with self._lock:
            self.poses.append(sample)

    def _on_final(self, twist):
        with self._lock:
            self.received.append((self.now(), twist.linear.x, twist.angular.z))

    def _on_speed_limit(self, message):
        self.speed_limit = (bool(message.percentage), float(message.speed_limit))

    def last_pose(self):
        """Return the latest (receive t, stamp, x, y, yaw), or None."""
        with self._lock:
            return self.poses[-1] if self.poses else None

    def limited(self, v, w):
        """Return (v, w) scaled by the speed limit in force, keeping the curvature."""
        if self.speed_limit is None:
            return v, w
        percentage, limit = self.speed_limit
        if limit <= 0.0:
            return v, w
        if percentage:
            scale = min(limit / 100.0, 1.0)
        else:
            scale = min(limit / abs(v), 1.0) if abs(v) > 1e-9 else 1.0
        return v * scale, w * scale

    def publish(self, v, w, record=True):
        """Publish one command (and remember it)."""
        twist = self._twist_classes[0]()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        if self.stamped:
            message = self._twist_classes[1]()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist = twist
        else:
            message = twist
        self.publisher.publish(message)
        if record:
            self.sent.append((self.now(), float(v), float(w)))

    def stop(self, seconds=0.5):
        """Publish zero commands for ``seconds`` wall s."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish(0.0, 0.0)
            time.sleep(1.0 / self.args.rate)

    def close(self):
        """Stop spinning and destroy the node."""
        self._executor.shutdown()
        self.node.destroy_node()


class Abort(Exception):
    """A safety bound stopped the run."""


def run_segments(driver, args):
    """Drive the segments; return their (t_start, t_end, v, w) in the node clock."""
    start = driver.last_pose()
    t_begin = driver.now()
    wall_limit = time.monotonic() + 3.0 * args.max_time + 30.0
    period = 1.0 / args.rate
    done = []

    def guard():
        pose = driver.last_pose()
        if pose is None or driver.now() - pose[0] > POSE_TIMEOUT:
            raise Abort(f'no pose for {POSE_TIMEOUT} s')
        if math.hypot(pose[2] - start[2], pose[3] - start[3]) > args.max_distance:
            raise Abort(f'the robot left the {args.max_distance} m circle around its start')
        if driver.now() - t_begin > args.max_time or time.monotonic() > wall_limit:
            raise Abort(f'--max-time {args.max_time} s reached')

    for i, (v, w) in enumerate(args.segments, 1):
        print(f'segment {i}/{len(args.segments)}: v {v:+.2f} m/s, w {w:+.2f} rad/s')
        t0 = driver.now()
        while driver.now() - t0 < args.segment_time:
            guard()
            driver.publish(*driver.limited(v, w))
            time.sleep(period)
        t1 = driver.now()
        done.append((t0, t1, v, w))
        t_pause = driver.now()
        while driver.now() - t_pause < args.pause:
            guard()
            driver.publish(0.0, 0.0)
            time.sleep(period)
    return done


def online(args):
    """Drive the sequence and estimate."""
    import numpy as np
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # Without rclpy's signal handlers Ctrl-C only raises KeyboardInterrupt: the context stays
    # valid, so the zero command can still be published.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    driver = Driver(args)
    status = 1
    try:
        error = driver.setup()
        if error:
            print(error, file=sys.stderr)
            return 1
        deadline = time.monotonic() + 5.0
        while driver.last_pose() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if driver.last_pose() is None:
            print(f'no message on {driver.pose_topic} within 5 s', file=sys.stderr)
            return 1
        bound = max(excursion(v, w, args.segment_time) for v, w in args.segments)
        x_icr_circle = 2.0 * 0.1  # a turn in place with |x_ICR| up to 0.1 m
        total = sum(args.segment_time + args.pause for _ in args.segments)
        print(
            f'Commands to {driver.node.resolve_topic_name(args.cmd_topic)} '
            f'({"TwistStamped" if driver.stamped else "Twist"}), pose from '
            f'{driver.node.resolve_topic_name(driver.pose_topic)}.'
        )
        for i, (v, w) in enumerate(args.segments, 1):
            radius = f'{abs(v / w):.2f} m' if abs(w) > 1e-9 else 'straight'
            print(
                f'  {i}: v {v:+.2f} m/s, w {w:+.2f} rad/s for {args.segment_time:.1f} s '
                f'(turn radius {radius})'
            )
        print(
            f'About {total:.0f} s; each segment stays within {max(bound, x_icr_circle):.2f} m '
            f'of where it starts; the run stops beyond {args.max_distance} m from here.'
        )
        if bound > args.max_distance:
            print(
                'the segments would leave --max-distance: lower --v or --segment-time',
                file=sys.stderr,
            )
            return 2
        if not args.yes:
            print(
                'Clear the area, keep the e-stop in hand. Type "yes" to start: ',
                end='',
                flush=True,
            )
            if sys.stdin.readline().strip().lower() != 'yes':
                print('not started')
                return 1
        try:
            segments = run_segments(driver, args)
        except Abort as reason:
            driver.stop()
            print(f'stopped: {reason}', file=sys.stderr)
            return 1
        driver.stop(1.0)

        with driver._lock:
            poses = np.array(driver.poses, dtype=float).reshape(-1, 5)
            received = np.array(driver.received, dtype=float).reshape(-1, 3)
        sent = np.array(driver.sent, dtype=float).reshape(-1, 3)
        # Everything in the pose stamp clock: offset = median(stamp - receive time).
        offset = float(np.median(poses[:, 1] - poses[:, 0]))
        reference = received if len(received) > 10 else sent
        if reference is received:
            print(f'commanded values from {args.final_topic} ({len(received)} messages)')
        else:
            print(
                f'commanded values: the published ones ({args.final_topic or "no"} '
                'final topic received)'
            )
        cmd = (reference[:, 0] + offset, reference[:, 1], reference[:, 2])
        pose = (poses[:, 1], poses[:, 2], poses[:, 3], poses[:, 4])
        order = np.argsort(pose[0])
        pose = tuple(a[order] for a in pose)
        result = slip.estimate(
            [slip.Segment(t0 + offset, t1 + offset, v, w) for t0, t1, v, w in segments],
            pose,
            cmd,
            settle=args.settle,
        )
        status = report(result, args)
    except KeyboardInterrupt:
        driver.stop()
        print('\ninterrupted: zero command sent', file=sys.stderr)
        status = 130
    finally:
        driver.close()
        if rclpy.ok():
            rclpy.shutdown()
    return status


def main():
    """Run the identification online or on a bag."""
    from rclpy.utilities import remove_ros_args

    args = parse_arguments(remove_ros_args(sys.argv)[1:])
    if args.bag:
        return offline(args)
    return online(args)


if __name__ == '__main__':
    sys.exit(main())
