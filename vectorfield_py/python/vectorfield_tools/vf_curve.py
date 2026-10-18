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

"""vf-curve: describe, sample, plot and export a curve.

The curve goes through the same validation as a FollowCurve goal (make_curve: polyline
conditioning, expression hardening, built-in parameter rules, regularity and closure). An
invalid curve prints the reason and exits with status 2.
"""

import sys
from pathlib import Path

import numpy as np
import vectorfield as vf

from . import _cli, curves, plotting

PROG = 'vf-curve'
EPILOG = """examples:
  vf-curve --list
  vf-curve --curve lemniscate_bernoulli:a=2 --plot -o lemniscate.svg
  vf-curve --curve '2*cos(s); sin(s)' --export ellipse.yaml --export ellipse.csv
  vf-curve --curve ellipse.csv --closed

--export writes FILE.csv as sampled x,y rows, and FILE.yaml as a vectorfield_msgs/msg/Curve
mapping of the spec (a polyline of the samples with --polyline), which --curve reads back.

exit status: 0 ok, 1 missing optional dependency, 2 invalid input or curve"""


def build_parser():
    """Build the vf-curve argument parser."""
    argp = _cli.parser(PROG, __doc__.split('\n', 1)[0].split(': ', 1)[1], EPILOG)
    argp.add_argument('--list', action='store_true', help='list the built-in shapes and exit')
    curves.add_curve_arguments(argp)
    group = argp.add_argument_group('sampling and export')
    group.add_argument(
        '--ds',
        type=_cli.number,
        metavar='M',
        help='sample spacing (m; default: about 400 samples, at most 0.05 m)',
    )
    group.add_argument('--time', type=_cli.number, default=0.0, help='time of a moving curve (s)')
    group.add_argument(
        '--export',
        action='append',
        default=[],
        metavar='FILE',
        help='write FILE.csv (samples) or FILE.yaml (Curve message); repeatable',
    )
    group.add_argument(
        '--polyline', action='store_true', help='export the YAML as a polyline of the samples'
    )
    group.add_argument('--frame', default='map', help='header.frame_id of the YAML (default map)')
    group.add_argument('--print-samples', action='store_true', help='print the samples as CSV')
    out = argp.add_argument_group('plot')
    out.add_argument('--plot', action='store_true', help='plot the curve (with -o and/or --show)')
    _cli.add_output_arguments(out, 'the curve plot')
    return argp


def list_shapes():
    """Return the built-in shapes as text lines."""
    lines = []
    for shape in vf.builtin_shapes():
        params = ', '.join(f'{k}={v:g}' for k, v in shape['parameters'].items())
        lines.append(f'{shape["name"]}: {shape["formula"]}')
        lines.append(f'    parameters: {params}')
    return lines


def description(spec, curve, points, t):
    """Text lines describing a validated curve."""
    xmin, xmax, ymin, ymax = curves.bounds(points)
    closed = len(points) > 3 and curve.is_closed
    kappa = vf.kappa_path(points[:-1] if closed else points, closed)
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    lines = [
        f'curve        {curves.describe_spec(spec)}',
        f'closed       {"yes" if curve.is_closed else "no (open: extended by rays)"}',
        f'moving       {"yes (depends on t)" if curve.is_time_varying else "no"}',
        f'length       {curve.length(t):.4f} m' + (f' at t = {t:g} s' if t else ''),
        f'extent       x [{xmin:.3f}, {xmax:.3f}], y [{ymin:.3f}, {ymax:.3f}] m',
        'regular      yes (make_curve validation passed)',
        f'kappa_path   {kappa:.3f} 1/m (max turning angle per mean segment length)',
        f'samples      {len(points)}, spacing {steps.min():.4f} to {steps.max():.4f} m',
    ]
    if spec.type == vf.CurveSpec.Type.POLYLINE:
        lines.append(
            f'vertices     {len(spec.points)} in, mean spacing {curve.vertex_spacing:.4f} m'
        )
    return lines


def export(path, spec, points, args):
    """Write one --export file (by extension)."""
    suffix = Path(path).suffix.lower()
    if suffix in ('.csv', '.txt'):
        curves.write_curve_csv(path, points)
    elif suffix in ('.yaml', '.yml'):
        if args.polyline:
            closed = bool(spec.closed or spec.type == vf.CurveSpec.Type.BUILTIN)
            spec = vf.CurveSpec.polyline(points[:-1] if closed else points, closed=closed)
        curves.write_curve_yaml(path, spec, args.frame)
    else:
        raise _cli.InputError(f'--export {path}: use a .csv or .yaml file name')
    print(f'wrote {path}')


def _main(argv):
    args = _cli.parse_args(build_parser(), argv)
    if args.list:
        print('\n'.join(list_shapes()))
        return _cli.EXIT_OK
    spec, curve = curves.curve_from_args(args)
    if curve.is_time_varying and args.export:
        print(f'{PROG}: note: samples of a moving curve are taken at t = {args.time:g} s')
    points = curves.sample(curve, args.ds, t=args.time)
    for line in description(curve.spec, curve, points, args.time):
        print(line)
    if args.print_samples:
        print('x,y')
        for x, y in points:
            print(f'{x:.6f},{y:.6f}')
    for path in args.export:
        export(path, curve.spec, points, args)
    if args.plot or args.output or args.show:
        if not (args.output or args.show):
            raise _cli.InputError('--plot needs -o FILE and/or --show')
        plt = _cli.pyplot(interactive=args.show)
        plotting.apply_style(plt)
        fig, ax = plt.subplots(figsize=(6.0, 4.5))
        plotting.draw_curve(ax, curve, t=args.time)
        ax.plot(
            points[:, 0], points[:, 1], ls='none', marker='.', ms=2.5, color=plotting.SERIES[0]
        )
        ax.plot(*points[0], marker='o', ms=6, color=plotting.SERIES[1], ls='none', label='s = 0')
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title(curves.describe_spec(spec))
        ax.legend(loc='best')
        _cli.finish_figure(plt, fig, args)
    return _cli.EXIT_OK


def main(argv=None):
    """Entry point of vf-curve."""
    return _cli.run(PROG, _main, argv)


if __name__ == '__main__':
    sys.exit(main())
