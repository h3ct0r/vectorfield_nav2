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

"""vf-tune: validate a controller configuration and sweep a parameter in simulation.

The configuration goes through validate_config(), the validator the controller runs at
configure (math_spec section 11), and, with --curve, through the per-plan advisor for the curve's
kappa_path. Each finding is printed with a suggested change. --sweep simulates the curve
(default: the built-in ellipse) for each value of a parameter.
"""

import math
import sys

import vectorfield as vf

from . import _cli, curves, simulation
from .config import add_config_arguments, config_from_args, get_parameter, set_parameter

PROG = 'vf-tune'
EPILOG = """examples:
  vf-tune nav2_params.yaml
  vf-tune nav2_params.yaml --curve path.csv --costmap 3,3,0.05
  vf-tune --set desired_speed=0.6 --set max_linear_vel=0.7
  vf-tune --sweep convergence_gain=1:4:0.5 --curve ellipse:a=2,b=1

exit status: 0 valid (warnings allowed), 1 missing optional dependency,
2 invalid configuration or input"""


def build_parser():
    """Build the vf-tune argument parser."""
    argp = _cli.parser(PROG, __doc__.split('\n', 1)[0].split(': ', 1)[1], EPILOG)
    add_config_arguments(argp, positional=True)
    checks = argp.add_argument_group('checks')
    checks.add_argument(
        '--costmap',
        metavar='W,H,RES',
        help='rolling costmap width, height (m) and resolution for the costmap rules '
        '(default: from the file local_costmap, if any)',
    )
    curves.add_curve_arguments(argp, default=None)
    sweep = argp.add_argument_group('sweep (closed-loop simulation)')
    sweep.add_argument(
        '--sweep',
        action='append',
        default=[],
        metavar='NAME=START:STOP:STEP',
        help='simulate each value of a parameter (or NAME=V1,V2,...); repeatable',
    )
    sweep.add_argument('--pose', metavar='X,Y,YAW', help='initial pose (default as vf-sim)')
    sweep.add_argument('--duration', type=_cli.number, metavar='T', help='simulated time (s)')
    sweep.add_argument(
        '--transient', type=_cli.number, metavar='T', help='max |D| after this time (s)'
    )
    sweep.add_argument(
        '--tolerance',
        type=_cli.number,
        default=simulation.DEFAULT_TOLERANCE,
        metavar='M',
        help='|D| tolerance of the settle time (m; default 0.05)',
    )
    return argp


def _c_gain(config):
    """c_G = G'(0) / k_f of the configured gain function."""
    gain = vf.GainFunction(config.field.gain_function, config.field.convergence_gain)
    return gain.dG0() / config.field.convergence_gain


def suggestion(code, config, kappa=0.0):
    """Suggest a change for a validation issue code ('' when there is none)."""
    f, m, ph = config.field, config.motion, config.phases
    v_r, k_f, d = f.desired_speed, f.convergence_gain, m.control_point_offset
    dt = 1.0 / config.node.controller_frequency
    issue = vf.issue
    try:
        c_g = _c_gain(config)
    except vf.ConfigError:
        c_g = float('nan')
    texts = {
        issue.SPEED_ABOVE_MAX: (
            f'lower desired_speed to {m.max_linear_vel:g} or raise max_linear_vel to {v_r:g}'
        ),
        issue.IN_PLACE_ABOVE_ROTATE: (
            'lower min_in_place_angular_vel below rotate_to_heading_angular_vel '
            f'({ph.rotate_to_heading_angular_vel:g})'
        ),
        issue.ROTATE_ABOVE_MAX: (
            f'lower rotate_to_heading_angular_vel to max_angular_vel ({m.max_angular_vel:g})'
        ),
        issue.CONTOUR_ORDER: (
            'use obstacle_contour_distance < obstacle_full_switch_distance <= '
            'obstacle_blend_start_distance, or -1 (auto) for all three'
        ),
        issue.CONTOUR_BELOW_INSCRIBED: (
            'raise obstacle_contour_distance (or -1, auto) or lower control_point_offset'
        ),
        issue.CONTOUR_BELOW_CIRCUMSCRIBED: (
            'raise obstacle_contour_distance (or -1, auto) or lower control_point_offset'
        ),
        issue.INVALID_LAW: 'use tracking_law: auto, which picks the law of the motion model',
        issue.ACKERMANN_CONTOUR: 'use obstacle_mode: none with the ackermann model',
        issue.ACKERMANN_RADIUS: 'set min_turning_radius to the vehicle turning radius (> 0)',
        issue.FL_OFFSET: 'set control_point_offset > 0 (0.2 m on a TurtleBot-sized base)',
        issue.BRAKING_EXCEEDS_WINDOW: (
            'lower terminal_braking_distance or enlarge the local costmap (window_limit)'
        ),
        issue.BLEND_EXCEEDS_WINDOW: (
            'lower obstacle_blend_start_distance (or -1, auto) or enlarge the local costmap'
        ),
        issue.HORIZON_TOO_SHORT: (
            'raise collision_check_horizon (it must cover the stopping distance at max_linear_vel)'
        ),
        issue.HORIZON_EXCEEDS_WINDOW: 'lower collision_check_horizon or enlarge the local costmap',
        issue.DISCRETE_OVERSHOOT: (
            f'lower convergence_gain below {1.0 / (c_g * v_r * dt):.3g} '
            'or raise controller_frequency'
        ),
        issue.FL_GAIN_OFFSET: (
            f'lower convergence_gain below {0.5 / (c_g * d):.3g} or control_point_offset below '
            f'{0.5 / (c_g * k_f):.3g} m'
            if d > 0.0
            else ''
        ),
        issue.BRAKING_TOO_SHORT: (
            f'raise terminal_braking_distance to {v_r * v_r / m.max_linear_decel:.3g} m or '
            f'max_linear_decel to {v_r * v_r / max(ph.terminal_braking_distance, 1e-9):.3g} m/s^2'
        ),
        issue.YAW_RATE_FOR_CURVATURE: _yaw_rate(m.max_angular_vel, v_r, c_g, k_f, kappa),
        issue.ACKERMANN_CURVATURE: (
            f'lower convergence_gain below {(1.0 / m.min_turning_radius - kappa) / c_g:.3g}'
            if m.min_turning_radius > 0.0 and 1.0 / m.min_turning_radius > kappa
            else 'the path itself is tighter than min_turning_radius: smooth it'
        ),
        issue.FL_PATH_CURVATURE: (
            f'lower control_point_offset below {1.0 / kappa:.3g} m or smooth the path'
            if kappa > 0.0
            else ''
        ),
    }
    return texts.get(code, '')


def _yaw_rate(omega, v_r, c_g, k_f, kappa):
    speed = omega / (c_g * k_f + kappa)
    text = f'lower desired_speed below {speed:.3g} m/s'
    gain = (omega / v_r - kappa) / c_g
    if gain > 0.0:
        text += f' or convergence_gain below {gain:.3g}'
    return text + ', or raise max_angular_vel'


def print_report(title, report, config, kappa=0.0):
    """Print a ValidationReport with suggestions; returns (errors, warnings)."""
    print(f'{title}: {len(report.errors)} errors, {len(report.warnings)} warnings')
    for kind, issues in (('error', report.errors), ('warning', report.warnings)):
        for issue in issues:
            print(f'  {kind} [{issue.code}]: {issue.message}')
            hint = suggestion(issue.code, config, kappa)
            if hint:
                print(f'      suggestion: {hint}')
    return len(report.errors), len(report.warnings)


def sweep_values(text):
    """(name, values) of a --sweep NAME=START:STOP:STEP or NAME=V1,V2,..."""
    name, sep, spec = text.partition('=')
    if not sep or not name.strip():
        raise _cli.InputError(f'--sweep needs NAME=START:STOP:STEP, got {text!r}')
    if ':' in spec:
        parts = [_cli.number(p) for p in spec.split(':')]
        if len(parts) != 3 or parts[2] <= 0.0 or parts[1] < parts[0]:
            raise _cli.InputError(f'--sweep {text!r}: need START <= STOP and STEP > 0')
        start, stop, step = parts
        count = int(math.floor((stop - start) / step + 1e-9)) + 1
        values = [start + k * step for k in range(count)]
    else:
        values = _cli.numbers(spec, name='--sweep')
    if not values or len(values) > 200:
        raise _cli.InputError(f'--sweep {text!r}: between 1 and 200 values')
    return name.strip(), values


def run_sweep(args, config, curve, spec):
    """Simulate each --sweep value; prints a table per parameter."""
    for text in args.sweep:
        name, values = sweep_values(text)
        current = get_parameter(config, name)
        print(f'\nsweep {name} (now {current}) on {curves.describe_spec(spec)}')
        print(f'  {"value":>10}  {"max |D| (m)":>12}  {"final |D| (m)":>13}  {"settle (s)":>10}')
        for value in values:
            trial = config.copy()
            set_parameter(trial, name, int(value) if isinstance(current, int) else value)
            simulation.configure(trial)
            report = vf.validate_config(trial)
            if not report.ok:
                codes = ', '.join(issue.code for issue in report.errors)
                print(f'  {value:>10.4g}  invalid: {codes}')
                continue
            pose = tuple(_cli.numbers(args.pose, 3, '--pose')) if args.pose else None
            duration = args.duration or simulation.default_duration(
                curve, trial.field.desired_speed
            )
            scenario = simulation.Scenario(
                trial,
                curve,
                pose or curves.default_pose(curve),
                duration,
                slip=simulation.default_slip(trial),
            )
            summary = simulation.summarize(
                simulation.simulate(scenario), args.transient, args.tolerance
            )
            settle = '-' if summary.settle_time is None else f'{summary.settle_time:.2f}'
            print(
                f'  {value:>10.4g}  {summary.max_D:>12.4f}  {summary.final_D:>13.4f}  {settle:>10}'
            )


def _main(argv):
    args = _cli.parse_args(build_parser(), argv)
    loaded = config_from_args(args)
    config = loaded.config
    print(
        f'configuration: {args.config or "defaults"}'
        + (f' ({args.plugin})' if args.config else '')
    )
    if loaded.applied:
        print(f'  parameters set: {", ".join(sorted(loaded.applied))}')
    if loaded.ignored:
        print(f'  ROS-only, not checked here: {", ".join(sorted(loaded.ignored))}')
    for name in sorted(loaded.unknown):
        print(f'  warning: unknown parameter {name!r} (not a vectorfield parameter; typo?)')
    costmap = loaded.costmap
    if args.costmap:
        width, height, resolution = _cli.numbers(args.costmap, 3, '--costmap')
        costmap = vf.CostmapInfo(width_m=width, height_m=height, resolution=resolution)
    if costmap is not None:
        print(
            f'  costmap rules for a {costmap.width_m:g} x {costmap.height_m:g} m window '
            f'at {costmap.resolution:g} m'
        )
    errors, _ = print_report('validate_config', vf.validate_config(config, costmap), config)
    if args.curve:
        spec, curve = curves.curve_from_args(args)
        points = curves.sample(curve)
        closed = curve.is_closed and len(points) > 3
        kappa = vf.kappa_path(points[:-1] if closed else points, closed)
        print(f'\ncurve: {curves.describe_spec(spec)}, kappa_path {kappa:.3f} 1/m')
        if errors == 0:
            print_report('advise_for_plan', vf.advise_for_plan(config, kappa), config, kappa)
    else:
        spec = curves.parse_curve(curves.DEFAULT_CURVE)
        curve = curves.build_curve(spec)
    if args.sweep:
        run_sweep(args, config, curve, spec)
    return _cli.EXIT_BAD_INPUT if errors else _cli.EXIT_OK


def main(argv=None):
    """Entry point of vf-tune."""
    return _cli.run(PROG, _main, argv)


if __name__ == '__main__':
    sys.exit(main())
