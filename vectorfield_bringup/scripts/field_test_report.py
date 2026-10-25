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

"""Write the metrics report of a recorded field test (``record_field_test.py``).

Reads a rosbag2 bag (sqlite3 or mcap) and splits it into episodes, one per NavigateToPose,
NavigateThroughPoses or FollowCurve goal (from the action status topics). Per episode:

- cross-track error RMS and maximum to the first ``/plan`` (Nav2 goals), the controller's own
  max |D| after convergence and, for the curve follower, |D| after the first lap;
- time to goal, goal miss and yaw error, stop latency after the result;
- RMS jerk of every recorded command topic (``cmd_vel_nav`` ... ``cmd_vel``);
- the phase timeline and time per phase, ROTATE_TO_FIELD entries per plan corner;
- status-flag counts (BLOCKED, CONTOUR, ASSUMPTION3_CLAMPED, CURVATURE_SATURATED, ...; CBF binds
  and holds when the markers were recorded);
- minimum clearance to lethal costmap cells, minimum scan range, FollowPath aborts, recoveries
  and localisation jumps with the controller's reaction.

Session-wide: speed-limit periods and the commanded speeds in them, map -> odom corrections and
the recorded topics with their rates. Writes ``report.md``, ``metrics.json`` and one
``episode_NN.png`` per episode (matplotlib) into ``--output`` (default ``<bag>_report``). The
thresholds of the pass/fail checks are options; the defaults are the system tests' simulation
numbers relaxed for a real robot (docs/tutorials/field_testing.md).

Example::

    ros2 run vectorfield_bringup field_test_report.py field_test_20261021_101500 --no-plots

Exit status: 0 when every check passed, 1 when one failed, 2 when the bag cannot be read.
"""

import argparse
import json
import math
import sys
from pathlib import Path

from vectorfield_bringup import field_report


def parse_arguments(argv):
    """Parse the command line."""
    defaults = field_report.Options()
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0], epilog='See docs/tutorials/field_testing.md.'
    )
    parser.add_argument('bag', help='bag directory (or file)')
    parser.add_argument('-o', '--output', help='output directory (default <bag>_report)')
    parser.add_argument('--odom-topic', default='', help='Odometry topic (default /odom)')
    parser.add_argument('--base-frame', default='', help='robot frame (default base_footprint)')
    parser.add_argument('--global-frame', default='', help="plan frame (default: /plan's)")
    parser.add_argument('--controller-topic', default='', help='controller field_state topic')
    parser.add_argument('--follower-topic', default='', help='curve follower field_state topic')
    parser.add_argument('--no-plots', action='store_true', help='skip the PNG plots')
    group = parser.add_argument_group('thresholds')
    group.add_argument(
        '--tracking-limit',
        type=float,
        default=defaults.tracking_limit,
        help='max |D| after convergence (m)',
    )
    group.add_argument(
        '--lap-limit', type=float, default=defaults.lap_limit, help='follower max |D| after lap 1'
    )
    group.add_argument(
        '--xy-tolerance', type=float, default=defaults.xy_tolerance, help='goal checker xy (m)'
    )
    group.add_argument(
        '--yaw-tolerance', type=float, default=defaults.yaw_tolerance, help='goal checker yaw'
    )
    group.add_argument(
        '--max-stop-latency',
        type=float,
        default=defaults.max_stop_latency,
        help='time to standstill after a result (s)',
    )
    group.add_argument(
        '--max-linear-vel',
        type=float,
        default=math.nan,
        help='max_linear_vel, to check percentage speed limits (m/s)',
    )
    return parser.parse_args(argv)


def _json_safe(value):
    """Return ``value`` with NaN and inf replaced by None and numpy scalars by floats."""
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if hasattr(value, 'item') and not isinstance(value, (str, bytes)):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main(argv=None):
    """Read the bag, write the report."""
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    bag_path = Path(args.bag)
    output = Path(args.output) if args.output else Path(f'{str(bag_path).rstrip("/")}_report')
    options = field_report.Options(
        tracking_limit=args.tracking_limit,
        lap_limit=args.lap_limit,
        xy_tolerance=args.xy_tolerance,
        yaw_tolerance=args.yaw_tolerance,
        max_stop_latency=args.max_stop_latency,
        max_linear_vel=args.max_linear_vel,
        controller_topic=args.controller_topic,
        follower_topic=args.follower_topic,
    )

    def warn(text):
        print(f'warning: {text}', file=sys.stderr)

    try:
        recording = field_report.load_recording(
            bag_path, args.odom_topic, args.base_frame, args.global_frame, warn
        )
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 2
    report = field_report.analyse(recording, options)
    output.mkdir(parents=True, exist_ok=True)
    images = {}
    if not args.no_plots and report['episodes']:
        try:
            images = field_report.plot_episodes(recording, report, str(output), options)
        except ImportError as error:
            warn(f'no plots ({error}); install python3-matplotlib')
    (output / 'report.md').write_text(
        field_report.render_markdown(report, bag_path.name, images) + '\n'
    )
    with open(output / 'metrics.json', 'w') as stream:
        json.dump(_json_safe(report), stream, indent=1)
    failed = 0
    for m in report['episodes']:
        bad = [c[0] for c in m['checks'] if c[3] is False]
        failed += len(bad)
        print(
            f'episode {m["index"]:2d} {m["kind"]:6s} {m["status"]:10s} {m["duration"]:6.1f} s  '
            f'max|D| {m["tracking_max"]:.3f} m  '
            + (
                f'XTE rms {m["xte_rms"]:.3f} max {m["xte_max"]:.3f} m  '
                if m['kind'] == 'nav'
                else f'laps {m["laps"]} |D| after lap 1 {m["d_max_after_lap1"]:.3f} m  '
            )
            + (f'FAILED: {", ".join(bad)}' if bad else 'ok')
        )
    for note in report['notes']:
        warn(note)
    print(f'report: {output / "report.md"}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
