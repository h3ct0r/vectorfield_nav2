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

"""vf-sim3d: closed-loop simulation of the experimental 3D core (holonomic multirotor).

Runs ``vectorfield.d3.Tracker`` against a point plant (optionally with a first-order velocity
lag) at a fixed control period, prints a summary, and optionally writes a 3D plot (mplot3d), an
animated GIF and the records as CSV. The 2D tools are unchanged; see ``vf-sim`` for robots in
the plane.
"""

import os
import sys

import numpy as np
import vectorfield as vf
from vectorfield import d3

from . import _cli, curves3d, plotting

PROG = 'vf-sim3d'
EPILOG = """examples:
  vf-sim3d --curve helix -o helix.png
  vf-sim3d --curve viviani:radius=2 --pose 2.5,0.5,0.6 --duration 60 --gif viviani.gif
  vf-sim3d --curve 'cos(s); sin(s); 0.3*sin(2*s)' --streamlines 10 -o loop.svg
  vf-sim3d --curve path.csv --open --speed 0.8 --csv run.csv

exit status: 0 ok, 1 missing optional dependency, 2 invalid input or curve"""


def build_parser():
    """Build the vf-sim3d argument parser."""
    argp = _cli.parser(PROG, __doc__.split('\n', 1)[0].split(': ', 1)[1], EPILOG)
    curves3d.add_curve_arguments(argp)
    group = argp.add_argument_group('simulation')
    group.add_argument(
        '--pose',
        metavar='X,Y,Z[,YAW]',
        help='start position and yaw (default: 0.6 m off the curve start)',
    )
    group.add_argument('--speed', type=_cli.number, default=0.5, help='v_r (m/s, default 0.5)')
    group.add_argument('--gain', type=_cli.number, default=2.0, help='k_f (1/m, default 2)')
    group.add_argument('--rate', type=_cli.number, default=20.0, help='control rate (Hz)')
    group.add_argument(
        '--duration', type=_cli.number, help='simulated time (s; default: two laps or the goal)'
    )
    group.add_argument(
        '--lag', type=_cli.number, default=0.0, help='velocity time constant tau (s, default 0)'
    )
    group.add_argument(
        '--yaw-policy',
        choices=('path_tangent', 'field', 'fixed'),
        default='path_tangent',
        help='yaw policy (default path_tangent)',
    )
    out = argp.add_argument_group('output')
    _cli.add_output_arguments(out, 'the 3D plot')
    out.add_argument(
        '--streamlines', type=int, default=0, metavar='N', help='draw N field streamlines'
    )
    out.add_argument('--gif', metavar='FILE', help='write an animated GIF of the run')
    out.add_argument('--fps', type=_cli.number, default=12.0, help='GIF frame rate (default 12)')
    out.add_argument(
        '--gif-seconds', type=_cli.number, default=8.0, help='GIF length (time-lapsed, default 8)'
    )
    out.add_argument('--csv', metavar='FILE', help='write the records as CSV')
    return argp


def _yaw_policy(name):
    return {
        'path_tangent': vf.OmniHeadingPolicy.PATH_TANGENT,
        'field': vf.OmniHeadingPolicy.FIELD,
        'fixed': vf.OmniHeadingPolicy.FIXED,
    }[name]


def default_start(curve):
    """0.6 m off the curve start, along a direction orthogonal to its tangent."""
    tangent = np.asarray(curve.tangent_at(0.0))
    helper = np.array([0.0, 0.0, 1.0]) if abs(tangent[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    normal = np.cross(tangent, helper)
    normal /= np.linalg.norm(normal)
    return curve.point_at(0.0) + 0.6 * normal


def simulate(
    curve,
    start,
    yaw=0.0,
    speed=0.5,
    gain=2.0,
    rate=20.0,
    duration=None,
    lag=0.0,
    yaw_policy='path_tangent',
):
    """Run the closed loop; returns the dict of arrays of ``d3.sim.ClosedLoopSimulator.run``."""
    config = d3.TrackerConfig(desired_speed=speed, convergence_gain=gain, control_frequency=rate)
    config.limits.max_speed = max(config.limits.max_speed, 1.2 * speed)
    yaw_config = config.yaw
    yaw_config.policy = _yaw_policy(yaw_policy)
    config.yaw = yaw_config
    tracker = d3.Tracker(config)
    simulator = d3.sim.ClosedLoopSimulator(tracker, period=1.0 / rate, velocity_time_constant=lag)
    simulator.reset(np.asarray(start, dtype=float), yaw=yaw)
    if duration is None:
        laps = 1.0 if not curve.is_closed else 2.0
        duration = laps * curve.length() / speed + 10.0
    return simulator.run(curve, duration)


def summary(run, curve):
    """Text lines describing a run."""
    t, D = run['t'], run['D']
    after = D[t >= 0.5 * t[-1]] if len(t) else D
    lines = [
        f'steps        {len(t)} ({t[-1] + (t[1] - t[0] if len(t) > 1 else 0.0):.2f} s)',
        f'D start/end  {D[0]:.4f} / {D[-1]:.4f} m',
        f'max D, 2nd half  {after.max():.4f} m',
    ]
    if not curve.is_closed:
        lines.append(f'settled      {"yes" if run["settled"][-1] else "no"}')
    return lines


def write_csv(path, run):
    """Write the records as CSV."""
    columns = ['t', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz', 'wz', 'D', 'sigma', 'phase', 'status']
    data = np.column_stack(
        [
            run['t'],
            run['position'],
            run['yaw'],
            run['v'],
            run['wz'],
            run['D'],
            run['sigma'],
            run['phase'],
            run['status'],
        ]
    )
    np.savetxt(path, data, delimiter=',', header=','.join(columns), comments='', fmt='%.6g')
    print(f'wrote {path}')


def draw(ax, curve, run, field=None, streamline_count=0):
    """Curve, streamlines, trajectory, start and goal on a 3D axis."""
    points = curve.sample()
    ax.plot(*points.T, color=plotting.INK, lw=1.4, label='curve')
    if field is not None and streamline_count > 0:
        for line in curves3d.streamlines(
            curve, field, curves3d.seeds_around(curve, streamline_count)
        ):
            ax.plot(*line.T, color=plotting.STREAM, lw=0.6, alpha=0.6)
    path = run['position']
    ax.plot(*path.T, color=plotting.SERIES[1], lw=1.2, label='trajectory')
    ax.scatter(*path[0], color=plotting.SERIES[1], s=18, label='start')
    curves3d.set_equal_aspect(ax, np.vstack([points, path]))


def _main(argv):
    args = _cli.parse_args(build_parser(), argv)
    spec, curve = curves3d.curve_from_args(args)
    if args.pose:
        values = _cli.numbers(args.pose, count=(3, 4), name='--pose')
        start, yaw = np.array(values[:3]), (values[3] if len(values) == 4 else 0.0)
    else:
        start, yaw = default_start(curve), 0.0
    if not (args.speed > 0 and args.rate > 0):
        raise _cli.InputError('--speed and --rate must be positive')
    run = simulate(
        curve,
        start,
        yaw,
        args.speed,
        args.gain,
        args.rate,
        args.duration,
        args.lag,
        args.yaw_policy,
    )
    print(f'curve        {curves3d.describe_spec(spec)}')
    for line in summary(run, curve):
        print(line)
    if args.csv:
        write_csv(args.csv, run)
    if args.output or args.show or args.gif:
        plt = _cli.pyplot(interactive=args.show)
        plotting.apply_style(plt)
        field = d3.DistanceField(vf.GainFunction(vf.GainType.ATAN, args.gain), args.speed)
        if args.output or args.show:
            fig, ax = curves3d.axes3d(plt)
            draw(ax, curve, run, field, args.streamlines)
            ax.set_title(curves3d.describe_spec(spec))
            ax.legend(loc='upper left')
            _cli.finish_figure(plt, fig, args)
        if args.gif:
            frames = animate(plt, curve, run, args.gif, fps=args.fps, seconds=args.gif_seconds)
            size = os.path.getsize(args.gif) / 1e6
            print(f'wrote {args.gif} ({frames} frames, {size:.2f} MB)')
    return _cli.EXIT_OK


def animate(plt, curve, run, path, fps=12.0, seconds=8.0, dpi=72):
    """Write an animated GIF of the trajectory (time-lapsed to ``seconds``); returns frames."""
    _cli.require('PIL')
    from matplotlib.animation import FuncAnimation, PillowWriter

    count = max(2, int(fps * seconds))
    indices = np.unique(np.linspace(0, len(run['t']) - 1, count).astype(int))
    fig, ax = curves3d.axes3d(plt, figsize=(5.0, 4.2))
    points = curve.sample()
    path_points = run['position']
    ax.plot(*points.T, color=plotting.INK, lw=1.2)
    (trail,) = ax.plot([], [], [], color=plotting.SERIES[1], lw=1.2)
    (body,) = ax.plot([], [], [], marker='o', ms=5, color=plotting.SERIES[1], ls='none')
    curves3d.set_equal_aspect(ax, np.vstack([points, path_points]))

    def update(frame):
        k = indices[frame]
        trail.set_data(path_points[: k + 1, 0], path_points[: k + 1, 1])
        trail.set_3d_properties(path_points[: k + 1, 2])
        body.set_data([path_points[k, 0]], [path_points[k, 1]])
        body.set_3d_properties([path_points[k, 2]])
        ax.view_init(elev=25.0, azim=-60.0 + 90.0 * frame / len(indices))
        ax.set_title(f't = {run["t"][k]:.1f} s, D = {run["D"][k]:.3f} m')
        return trail, body

    animation = FuncAnimation(fig, update, frames=len(indices), blit=False)
    animation.save(str(path), writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return len(indices)


def main(argv=None):
    """Entry point of vf-sim3d."""
    return _cli.run(PROG, _main, argv)


if __name__ == '__main__':
    sys.exit(main())
