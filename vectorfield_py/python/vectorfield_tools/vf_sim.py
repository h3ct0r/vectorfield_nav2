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

"""vf-sim: closed-loop simulation of the controller on a curve."""

import csv
import os
import sys

import numpy as np
import vectorfield as vf

from . import _cli, curves, plotting, simulation
from .config import add_config_arguments, config_from_args
from .obstacles import add_obstacle_arguments, obstacles_from_args

PROG = 'vf-sim'
EPILOG = """examples:
  vf-sim --model differential --curve ellipse
  vf-sim --model ackermann --curve ellipse:a=2,b=1 --gif ellipse.gif
  vf-sim --model omnidirectional --curve '2*cos(s); sin(2*s)' -o run.png
  vf-sim --curve path.csv --circle 1.5,0.05,0.3 -o contour.svg

exit status: 0 ok, 1 missing optional dependency, 2 invalid input or configuration"""


def build_parser():
    """Build the vf-sim argument parser."""
    argp = _cli.parser(PROG, __doc__.split('\n', 1)[0].split(': ', 1)[1], EPILOG)
    run = argp.add_argument_group('simulation')
    run.add_argument('--model', choices=simulation.MODELS, help='motion model (default: config)')
    run.add_argument('--law', choices=simulation.LAWS, help='tracking law (default: auto)')
    run.add_argument('--speed', type=_cli.number, metavar='V', help='desired_speed v_r (m/s)')
    run.add_argument('--gain', type=_cli.number, metavar='K', help='convergence_gain k_f (1/m)')
    run.add_argument(
        '--pose',
        metavar='X,Y,YAW',
        help='initial pose (default: 1 m right of the curve start, along the tangent)',
    )
    run.add_argument(
        '--duration',
        type=_cli.number,
        metavar='T',
        help='simulated time (s; default two laps, or the length plus 15 s)',
    )
    run.add_argument(
        '--period', type=_cli.number, default=0.0, metavar='DT', help='control period (s)'
    )
    run.add_argument(
        '--slip',
        metavar='ETA_PSI,X_ICR',
        help='plant slip (default: 0.75,0.05 for skid_steer as in vectorfield_sim, else none)',
    )
    run.add_argument(
        '--goal-tolerance',
        type=_cli.number,
        default=0.25,
        metavar='M',
        help='xy goal tolerance of an open curve (m; default 0.25)',
    )
    run.add_argument(
        '--transient',
        type=_cli.number,
        metavar='T',
        help='max |D| is reported after this time (s; default half the run)',
    )
    run.add_argument(
        '--tolerance',
        type=_cli.number,
        default=simulation.DEFAULT_TOLERANCE,
        metavar='M',
        help='|D| tolerance of the settle time (m; default 0.05)',
    )
    curves.add_curve_arguments(argp)
    add_obstacle_arguments(argp)
    add_config_arguments(argp)
    out = argp.add_argument_group('output')
    _cli.add_output_arguments(out, 'the path, |D|(t), v(t) and omega(t) plot')
    out.add_argument('--gif', metavar='FILE', help='write an animated GIF of the run')
    out.add_argument('--fps', type=_cli.number, default=12.0, help='GIF frame rate (default 12)')
    out.add_argument(
        '--gif-seconds',
        type=_cli.number,
        default=10.0,
        metavar='S',
        help='GIF length; longer runs are time-lapsed (default 10)',
    )
    out.add_argument('--csv', metavar='FILE', help='write the recorded columns as CSV')
    return argp


def scenario_from_args(args):
    """(Scenario, CurveSpec, notes) from vf-sim arguments; raises InputError or ConfigError."""
    loaded = config_from_args(args)
    config = loaded.config
    if loaded.unknown:
        raise _cli.InputError(f'unknown parameters: {", ".join(sorted(loaded.unknown))}')
    notes = simulation.configure(
        config, args.model, args.law, args.speed, args.gain, explicit=set(loaded.applied)
    )
    obstacles = obstacles_from_args(args)
    if obstacles:
        mode = args.obstacle_mode
        if mode is None:
            mode = 'none' if config.motion.motion_model == vf.MotionModel.ACKERMANN else 'contour'
        config.obstacles.obstacle_mode = mode
    spec, curve = curves.curve_from_args(args)
    if args.pose:
        pose = tuple(_cli.numbers(args.pose, 3, '--pose'))
    else:
        pose = curves.default_pose(curve)
    if args.slip:
        slip = tuple(_cli.numbers(args.slip, 2, '--slip'))
    else:
        slip = simulation.default_slip(config)
    duration = args.duration
    if duration is None:
        duration = simulation.default_duration(curve, config.field.desired_speed)
    if duration <= 0.0 or args.period < 0.0:
        raise _cli.InputError('--duration must be positive and --period not negative')
    scenario = simulation.Scenario(
        config=config,
        curve=curve,
        pose=pose,
        duration=duration,
        period=args.period,
        obstacles=obstacles,
        slip=slip,
        xy_tolerance=args.goal_tolerance,
        yaw_tolerance=args.goal_tolerance,
        resolution=args.grid_resolution,
    )
    return scenario, spec, notes


def write_csv(path, result):
    """Write the SimulationResult columns as CSV."""
    columns = result.as_dict()
    header, data = [], []
    for name, values in columns.items():
        values = np.asarray(values)
        if values.ndim == 1:
            header.append(name)
            data.append(values.astype(float))
        else:
            suffixes = {3: ('x', 'y', 'yaw') if name == 'pose' else ('vx', 'vy', 'wz'), 2: 'xy'}
            for k, suffix in enumerate(suffixes[values.shape[1]]):
                header.append(f'{name}_{suffix}')
                data.append(values[:, k].astype(float))
    with open(path, 'w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(np.column_stack(data).tolist())


def describe(scenario, spec):
    """Return a one-line description of a scenario."""
    config = vf.resolve_auto_values(scenario.config)
    model = config.motion.motion_model.to_string()
    law = config.motion.tracking_law.to_string()
    rate = 1.0 / scenario.control_period()
    return (
        f'{model} ({law}) on {curves.describe_spec(spec)}, v_r {config.field.desired_speed:g} '
        f'm/s, k_f {config.field.convergence_gain:g} 1/m, {scenario.duration:.1f} s at {rate:g} Hz'
    )


def _main(argv):
    args = _cli.parse_args(build_parser(), argv)
    scenario, spec, notes = scenario_from_args(args)
    report = vf.validate_config(scenario.config)
    for kind, issues in (('error', report.errors), ('warning', report.warnings)):
        for issue in issues:
            print(f'{PROG}: {kind} [{issue.code}]: {issue.message}', file=sys.stderr)
    if not report.ok:
        raise _cli.InputError('invalid configuration (vf-tune suggests fixes)')
    print(describe(scenario, spec))
    for note in notes:
        print(f'  note: {note}')
    run = simulation.simulate(scenario)
    summary = simulation.summarize(run, args.transient, args.tolerance)
    for line in summary.lines():
        print(f'  {line}')
    if args.csv:
        write_csv(args.csv, run.result)
        print(f'wrote {args.csv}')
    if args.output or args.show or args.gif:
        plt = _cli.pyplot(interactive=args.show)
        plotting.apply_style(plt)
        if args.gif:
            frames = plotting.animate_simulation(
                plt, run, args.gif, fps=args.fps, seconds=args.gif_seconds
            )
            size = os.path.getsize(args.gif) / 1e6
            print(f'wrote {args.gif} ({frames} frames, {size:.2f} MB)')
        if args.output or args.show:
            fig = plotting.plot_simulation(plt, run, summary, title=describe_title(scenario))
            _cli.finish_figure(plt, fig, args)
    return _cli.EXIT_OK


def describe_title(scenario):
    """Return a short plot title."""
    return vf.resolve_auto_values(scenario.config).motion.motion_model.to_string()


def main(argv=None):
    """Entry point of vf-sim."""
    return _cli.run(PROG, _main, argv)


if __name__ == '__main__':
    sys.exit(main())
