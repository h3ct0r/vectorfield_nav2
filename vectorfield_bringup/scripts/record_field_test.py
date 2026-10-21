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

"""Record a field test of the vector-field controller and curve follower into a rosbag2 bag.

Records exactly the topics ``field_test_report.py`` and ``identify_skid_slip.py --bag`` need:
TF, odometry, ``/plan``, both ``field_state`` topics, the command chain (``cmd_vel_nav``,
``cmd_vel_smoothed``, ``cmd_vel``, ``cmd_vel_safe``), ``speed_limit``, the local costmap and its
footprint, the scan, the localisation pose, ``/behavior_tree_log`` and the status (and
FollowCurve feedback) of the navigation, controller, recovery and curve-following actions. Topics
that are not published are simply absent from the bag. ``--markers`` adds the marker topics
(needed to count CBF activity; about 0.5 MB/s more).

The bag is ``<output-dir>/field_test_<YYYYmmdd_HHMMSS>[_<label>]``. When the recording stops
(Ctrl-C or ``--duration``), the parameters of the controller server, the curve follower, the
velocity smoother, the collision monitor and the local costmap, dumped at the start, are written
into the bag directory with ``field_test.yaml`` (the command line, the distro, the topics).

Examples::

    ros2 run vectorfield_bringup record_field_test.py --label corners
    ros2 run vectorfield_bringup record_field_test.py --markers --duration 300
    ros2 run vectorfield_bringup record_field_test.py --namespace /robot1 --dry-run
"""

import argparse
import datetime
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

#: Topics, relative to --namespace except TF and /clock.
TOPICS = (
    'plan',
    'speed_limit',
    'cmd_vel_nav',
    'cmd_vel_smoothed',
    'cmd_vel',
    'cmd_vel_safe',
    'behavior_tree_log',
    'amcl_pose',
    'initialpose',
    'goal_pose',
    'local_costmap/published_footprint',
)
#: Actions whose status (and, for follow_curve, feedback) is recorded.
ACTIONS = (
    'navigate_to_pose',
    'navigate_through_poses',
    'follow_path',
    'compute_path_to_pose',
    'follow_curve',
    'spin',
    'backup',
    'wait',
    'drive_on_heading',
)
#: Nodes whose parameters are dumped, relative to --namespace.
PARAMETER_NODES = (
    'controller_server',
    'velocity_smoother',
    'collision_monitor',
    'local_costmap/local_costmap',
    'bt_navigator',
)
GLOBAL_TOPICS = ('/tf', '/tf_static', '/clock')


def absolute(namespace, name):
    """Return ``name`` under ``namespace`` (an absolute name)."""
    if name.startswith('/'):
        return name
    namespace = '/' + namespace.strip('/') if namespace.strip('/') else ''
    return f'{namespace}/{name}'


def topic_list(args):
    """Return the absolute topic names to record."""
    ns = args.namespace
    topics = list(GLOBAL_TOPICS)
    topics += [absolute(ns, t) for t in TOPICS]
    topics += [absolute(ns, args.odom_topic), absolute(ns, args.scan_topic)]
    topics.append(absolute(ns, args.costmap_topic))
    topics.append(absolute(ns, f'{args.controller}/field_state'))
    topics.append(absolute(ns, f'{args.follower}/field_state'))
    topics.append(absolute(ns, f'{args.follower}/curve'))
    if args.markers:
        topics.append(absolute(ns, f'{args.controller}/markers'))
        topics.append(absolute(ns, f'{args.follower}/markers'))
    for action in ACTIONS:
        topics.append(absolute(ns, f'{action}/_action/status'))
    topics.append(absolute(ns, 'follow_curve/_action/feedback'))
    topics += [absolute(ns, t) for t in args.extra]
    seen = []
    for topic in topics:
        if topic not in seen:
            seen.append(topic)
    return seen


def bag_name(args, now=None):
    """Return the bag directory: field_test_<timestamp>[_<label>] in --output-dir."""
    now = now or datetime.datetime.now()
    name = f'field_test_{now.strftime("%Y%m%d_%H%M%S")}'
    if args.label:
        name += '_' + re.sub(r'[^A-Za-z0-9_-]+', '-', args.label).strip('-')
    return str(Path(args.output_dir) / name)


def record_command(args, output, topics):
    """Return the ``ros2 bag record`` command line.

    One anchored regex selects the topics: ``-e`` exists on Humble and later, while the topic
    list moved from positional arguments to ``--topics`` in Jazzy. Hidden topics (the actions')
    need ``--include-hidden-topics``.
    """
    regex = '^(' + '|'.join(re.escape(t) for t in topics) + ')$'
    command = ['ros2', 'bag', 'record', '-o', output, '--include-hidden-topics', '-e', regex]
    if args.storage:
        command += ['-s', args.storage]
    return command


def dump_parameters(args, results):
    """Fill ``results`` with {node: YAML text} from ``ros2 param dump`` (run in a thread)."""
    nodes = [absolute(args.namespace, n) for n in PARAMETER_NODES]
    nodes.append(absolute(args.namespace, args.follower))
    for node in nodes:
        try:
            done = subprocess.run(
                ['ros2', 'param', 'dump', node],
                capture_output=True,
                text=True,
                timeout=20.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        if done.returncode == 0 and done.stdout.strip() and 'ros__parameters' in done.stdout:
            results[node] = done.stdout


def parse_arguments(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        epilog='See docs/tutorials/field_testing.md.',
    )
    parser.add_argument('-o', '--output-dir', default='.', help='where the bag directory goes')
    parser.add_argument('--label', default='', help='appended to the bag name')
    parser.add_argument('--namespace', default='', help='robot namespace of the Nav2 topics')
    parser.add_argument('--odom-topic', default='odom', help='Odometry topic')
    parser.add_argument('--scan-topic', default='scan', help='LaserScan topic')
    parser.add_argument(
        '--costmap-topic', default='local_costmap/costmap', help='OccupancyGrid of the costmap'
    )
    parser.add_argument('--controller', default='FollowPath', help='controller plugin name')
    parser.add_argument('--follower', default='curve_follower', help='curve follower node name')
    parser.add_argument(
        '--markers', action='store_true', help='also record the controller and follower markers'
    )
    parser.add_argument('--extra', nargs='*', default=[], help='more topics to record')
    parser.add_argument(
        '-s', '--storage', choices=('sqlite3', 'mcap'), help="storage plugin (default: rosbag2's)"
    )
    parser.add_argument('--duration', type=float, default=0.0, help='stop after this many s')
    parser.add_argument(
        '--no-param-dump', action='store_true', help='do not dump the node parameters'
    )
    parser.add_argument('--dry-run', action='store_true', help='print the command and exit')
    return parser.parse_args(argv)


def write_info(output, args, topics, parameters):
    """Write field_test.yaml and the parameter dumps into the bag directory."""
    import yaml

    directory = Path(output)
    if not directory.is_dir():
        return
    for node, text in parameters.items():
        (directory / f'params_{node.strip("/").replace("/", "_")}.yaml').write_text(text)
    info = {
        'command': ' '.join(shlex.quote(a) for a in sys.argv),
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'label': args.label,
        'topics': topics,
        'parameter_dumps': sorted(parameters),
    }
    (directory / 'field_test.yaml').write_text(yaml.safe_dump(info, sort_keys=False))


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    """Record until Ctrl-C, SIGTERM or --duration."""
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    # A shell without job control starts background commands with SIGINT ignored; restore it, so
    # `kill -INT` stops the recording cleanly too, and treat SIGTERM the same way.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _interrupt)
    topics = topic_list(args)
    output = bag_name(args)
    command = record_command(args, output, topics)
    if args.dry_run:
        print(' '.join(shlex.quote(a) for a in command))
        return 0
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    parameters = {}
    dumper = None
    if not args.no_param_dump:
        dumper = threading.Thread(target=dump_parameters, args=(args, parameters), daemon=True)
        dumper.start()
    print(f'recording {len(topics)} topics into {output} (Ctrl-C to stop)', flush=True)
    # Its own session: a terminal Ctrl-C reaches only this script, which forwards one SIGINT,
    # so the recorder closes the bag cleanly.
    recorder = subprocess.Popen(command, start_new_session=True)
    started = time.monotonic()
    status = 0
    try:
        while recorder.poll() is None:
            if args.duration > 0.0 and time.monotonic() - started >= args.duration:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    if recorder.poll() is None:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            recorder.terminate()
            recorder.wait(timeout=10.0)
    else:
        status = recorder.returncode
    if dumper is not None:
        dumper.join(timeout=5.0)
    write_info(output, args, topics, dict(parameters))
    print(f'bag: {output}')
    print(f'report: ros2 run vectorfield_bringup field_test_report.py {output}')
    return status


if __name__ == '__main__':
    sys.exit(main())
