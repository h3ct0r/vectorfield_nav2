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

"""vf-plot: the vector field around a curve, with obstacles and the field decomposition.

A closed curve without obstacles shows the path field Phi (time-varying curves at --time). An
open curve, or any curve with obstacles, shows the field the controller commands from --pose:
the terminal field towards the goal and, near obstacles, the contour field (orange arrows).
"""

import sys

import numpy as np

from . import _cli, curves, plotting, simulation
from .config import add_config_arguments, config_from_args
from .obstacles import add_obstacle_arguments, obstacles_from_args

PROG = 'vf-plot'
EPILOG = """examples:
  vf-plot --curve ellipse:a=2,b=1 -o field.svg
  vf-plot --curve '1.5*cos(s) + 0.1*t; 1.5*sin(s)' --decompose 2.3,0.8 -o decomposition.png
  vf-plot --curve path.csv --circle 0,0,0.3 --pose -1.2,0,0 --style quiver --show

exit status: 0 ok, 1 missing optional dependency, 2 invalid input or configuration"""


def build_parser():
    """Build the vf-plot argument parser."""
    argp = _cli.parser(PROG, __doc__.split('\n', 1)[0].split(': ', 1)[1], EPILOG)
    view = argp.add_argument_group('field')
    view.add_argument(
        '--style',
        choices=['streamlines', 'quiver', 'both'],
        default='both',
        help='how to draw the field (default both)',
    )
    view.add_argument(
        '--extent',
        metavar='XMIN,XMAX,YMIN,YMAX',
        help='plotted region (default: the curve and obstacles plus a margin)',
    )
    view.add_argument('--grid', type=int, default=21, help='quiver columns (default 21)')
    view.add_argument('--seeds', type=int, default=28, help='streamline seeds (default 28)')
    view.add_argument('--speed', type=_cli.number, metavar='V', help='desired_speed v_r (m/s)')
    view.add_argument('--gain', type=_cli.number, metavar='K', help='convergence_gain k_f (1/m)')
    view.add_argument('--time', type=_cli.number, default=0.0, help='time of a moving curve (s)')
    view.add_argument(
        '--pose',
        metavar='X,Y,YAW',
        help='robot pose for the controller field (default: 1 m right of the curve start)',
    )
    view.add_argument(
        '--decompose',
        metavar='X,Y',
        help='draw and print -eta G n, eta H T, Phi_T and F of the path field at this point',
    )
    view.add_argument('--title', help='figure title')
    curves.add_curve_arguments(argp)
    add_obstacle_arguments(argp)
    add_config_arguments(argp)
    _cli.add_output_arguments(argp.add_argument_group('output'))
    return argp


def _vector(v):
    v = np.asarray(v, dtype=float)
    return f'({v[0]: .3f}, {v[1]: .3f})  |{np.linalg.norm(v):.3f}| m/s'


def decomposition_lines(sample, t):
    """Return the decomposition of a FieldSample as text lines."""
    x, x_star = np.asarray(sample.x_eval), np.asarray(sample.x_star)
    return [
        f'decomposition at ({x[0]:g}, {x[1]:g}), t = {t:g} s',
        f'  closest point ({x_star[0]:.3f}, {x_star[1]:.3f}), D {sample.D:.3f} m, '
        f'G {sample.G:.3f}, H {sample.H:.3f}, eta {sample.eta:.3f}',
        f'  -eta G n   {_vector(sample.conv)}',
        f'  eta H T    {_vector(sample.circ)}',
        f'  Phi_T      {_vector(sample.Phi_T)}',
        f'  F          {_vector(sample.Phi_gamma)}',
    ]


def _main(argv):
    args = _cli.parse_args(build_parser(), argv)
    if not (args.output or args.show or args.decompose):
        raise _cli.InputError('nothing to do: pass -o FILE, --show or --decompose X,Y')
    if args.grid < 2 or args.seeds < 0:
        raise _cli.InputError('--grid must be at least 2 and --seeds not negative')
    loaded = config_from_args(args)
    if loaded.unknown:
        raise _cli.InputError(f'unknown parameters: {", ".join(sorted(loaded.unknown))}')
    config = loaded.config
    simulation.configure(config, speed=args.speed, gain=args.gain, explicit=set(loaded.applied))
    obstacles = obstacles_from_args(args)
    if obstacles:
        config.obstacles.obstacle_mode = args.obstacle_mode or 'contour'
    _, curve = curves.curve_from_args(args)
    pose = tuple(_cli.numbers(args.pose, 3, '--pose')) if args.pose else curves.default_pose(curve)
    scenario = simulation.Scenario(
        config, curve, pose, 1.0, obstacles=obstacles, resolution=args.grid_resolution
    )
    nominal = plotting.NominalField(config, curve, args.time)
    point = None
    if args.decompose:
        point = _cli.numbers(args.decompose, 2, '--decompose')
        for line in decomposition_lines(nominal.sample(point), args.time):
            print(line)
    if not (args.output or args.show):
        return _cli.EXIT_OK

    if args.extent:
        extent = tuple(_cli.numbers(args.extent, 4, '--extent'))
        if extent[1] <= extent[0] or extent[3] <= extent[2]:
            raise _cli.InputError('--extent needs XMIN < XMAX and YMIN < YMAX')
    else:
        points = [curves.sample(curve, t=args.time), obstacles.outline_points()]
        if point is not None:
            points.append([point])
        lo = np.vstack(points)
        size = max(np.ptp(lo[:, 0]), np.ptp(lo[:, 1]), 1.0)
        extent = curves.bounds(lo, margin=0.25 * size)
    obstacle_map = (
        obstacles.obstacle_map(scenario.extent(), args.grid_resolution) if obstacles else None
    )
    controller = obstacles or not curve.is_closed
    if controller:
        field = plotting.TrackerField(config, curve, pose, scenario.goal(), obstacle_map)
    else:
        field = nominal

    plt = _cli.pyplot(interactive=args.show)
    plotting.apply_style(plt)
    fig, ax = plt.subplots(figsize=(6.4, 6.4 * (extent[3] - extent[2]) / (extent[1] - extent[0])))
    if args.style in ('quiver', 'both'):
        X, Y, grid = plotting.grid_points(extent, args.grid)
        F = field.vectors(grid)
        if controller and obstacles:
            active = field.probe(grid)['contour_active']
            plotting.draw_quiver(ax, X, Y, np.where(active[:, None], np.nan, F))
            plotting.draw_quiver(
                ax, X, Y, np.where(active[:, None], F, np.nan), color=plotting.CONTOUR
            )
        else:
            plotting.draw_quiver(ax, X, Y, F)
    if args.style in ('streamlines', 'both') and args.seeds:
        lines = plotting.streamlines(field, plotting.seed_points(extent, args.seeds), extent)
        plotting.draw_streamlines(ax, lines)
    plotting.draw_curve(ax, curve, t=args.time)
    if obstacles:
        obstacles.draw(ax, color=plotting.OBSTACLE)
        plotting.draw_offsets(ax, field.tracker.config, obstacle_map)
    if controller:
        outline = plotting.robot_outline(pose, field.tracker.footprint.r_circ)
        ax.plot(outline[:, 0], outline[:, 1], color=plotting.INK, lw=1.2, zorder=6)
    if point is not None:
        plotting.draw_decomposition(ax, nominal.sample(point))
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(args.title or curves.describe_spec(curve.spec))
    _cli.finish_figure(plt, fig, args)
    return _cli.EXIT_OK


def main(argv=None):
    """Entry point of vf-plot."""
    return _cli.run(PROG, _main, argv)


if __name__ == '__main__':
    sys.exit(main())
