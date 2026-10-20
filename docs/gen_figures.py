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

"""Generate the theory-page figures into docs/_static/figures/ (run manually, then commit).

The docs CI job does not build the C++ extension, so the SVGs are committed. Regenerate them
after a change to the field, the tracker or the plotting helpers:

    pip install './vectorfield_py[tools]'
    python docs/gen_figures.py

The figures use the vectorfield bindings and the vectorfield_tools plotting helpers, so they
show what the controller computes. The output is deterministic (no dates, fixed SVG ids).
"""

import argparse
import math
from pathlib import Path

import numpy as np
import vectorfield as vf
from vectorfield_tools import _cli, plotting, simulation
from vectorfield_tools.obstacles import Obstacles

OUTPUT = Path(__file__).resolve().parent / '_static' / 'figures'
MAX_BYTES = 200_000


def _axes(ax, extent):
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')


def field_ellipse(plt):
    """(1) The static field around an ellipse: unit arrows and streamlines."""
    curve = vf.make_curve(vf.CurveSpec.builtin('ellipse', a=2.0, b=1.0))
    field = plotting.NominalField(vf.CoreConfig(), curve)
    extent = (-3.2, 3.2, -2.2, 2.2)
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    X, Y, grid = plotting.grid_points(extent, 19)
    plotting.draw_quiver(ax, X, Y, field.vectors(grid))
    lines = plotting.streamlines(field, plotting.seed_points(extent, 24), extent)
    plotting.draw_streamlines(ax, lines, decimate=4)
    plotting.draw_curve(ax, curve)
    _axes(ax, extent)
    return fig


def field_decomposition(plt):
    """(2) -eta G n, eta H T, Phi_T and F at a point near a circle translating at 0.1 m/s."""
    spec = vf.CurveSpec.expression(
        'r*cos(s) + vx*t',
        'r*sin(s)',
        0.0,
        2.0 * math.pi,
        closed=True,
        parameters={'r': 1.5, 'vx': 0.1},
    )
    curve = vf.make_curve(spec)
    field = plotting.NominalField(vf.CoreConfig(), curve, t=0.0)
    extent = (-0.4, 3.4, -0.9, 2.3)
    fig, ax = plt.subplots(figsize=(5.6, 4.7))
    X, Y, grid = plotting.grid_points(extent, 13)
    plotting.draw_quiver(ax, X, Y, field.vectors(grid))
    later = curve.sample(ds=0.05, t=10.0)
    ax.plot(later[:, 0], later[:, 1], color=plotting.INK_SECONDARY, lw=0.9, ls='--', zorder=3)
    ax.annotate(
        'curve at t = 10 s',
        (later[0] + later[len(later) // 12]) / 2.0,
        xytext=(6, -2),
        textcoords='offset points',
        color=plotting.INK_SECONDARY,
        fontsize=8,
    )
    plotting.draw_curve(ax, curve, label='curve at t = 0')
    plotting.draw_decomposition(ax, field.sample([2.3, 0.8]), scale=2.0)
    _axes(ax, extent)
    return fig


def terminal_profile(plt):
    """(3) phi(sigma) and the braking zone; a closed-loop approach to the goal of a line."""
    config = vf.CoreConfig()
    v_r, v_min = config.field.desired_speed, config.phases.min_approach_speed
    sigma_b = config.phases.terminal_braking_distance
    fig, (ax_phi, ax_v) = plt.subplots(1, 2, figsize=(9.0, 3.4), layout='constrained')
    plotting.draw_terminal_profile(ax_phi, v_r, v_min, sigma_b, (0.25, 0.05))
    ax_phi.set_title('speed profile of the terminal field', fontsize=9)

    line = vf.make_curve(vf.CurveSpec.polyline([[0.0, 0.0], [4.0, 0.0]]))
    scenario = simulation.Scenario(config.copy(), line, (0.0, -0.4, 0.0), 30.0)
    result = simulation.simulate(scenario).result
    follow = result.phase != int(vf.Phase.ROTATE_TO_HEADING)
    sigma = result.sigma[follow]
    profile = vf.TerminalProfile(v_r, v_min, sigma_b, scenario.xy_tolerance / 2.0)
    grid = np.linspace(0.0, 1.5, 301)
    ax_v.axvspan(0.0, sigma_b, color=plotting.GRID, alpha=0.7, lw=0, zorder=0)
    ax_v.plot(grid, v_r * profile(grid), color=plotting.SERIES[1], lw=3.2, label=r'$v_r\,\varphi$')
    ax_v.plot(
        sigma, result.cmd[follow, 0], color=plotting.SERIES[0], lw=1.4, label='$v$ commanded'
    )
    ax_v.set_xlim(1.5, 0.0)
    ax_v.set_ylim(0.0, 0.45)
    ax_v.set_xlabel(r'remaining arc length $\sigma$ (m)')
    ax_v.set_ylabel('speed (m/s)')
    ax_v.set_title('closed loop, differential drive, xy_tol 0.25 m', fontsize=9)
    ax_v.legend(loc='lower left')
    return fig


def contour_disc(plt):
    """(4) Contouring a disc that blocks a straight path; the side is latched for the episode."""
    config = vf.CoreConfig()
    config.obstacles.obstacle_mode = 'contour'
    line = vf.make_curve(vf.CurveSpec.polyline([[-3.0, 0.0], [3.0, 0.0]]))
    obstacles = Obstacles(circles=[(0.0, 0.05, 0.3)])
    scenario = simulation.Scenario(config, line, (-3.0, 0.0, 0.0), 40.0, obstacles=obstacles)
    run = simulation.simulate(scenario)
    first = int(np.flatnonzero(run.contour_active)[0])
    field = plotting.TrackerField(
        config, line, tuple(run.result.pose[first]), scenario.goal(), run.obstacle_map
    )
    extent = (-2.4, 3.3, -1.5, 1.2)
    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    X, Y, grid = plotting.grid_points(extent, 30)
    probe = field.probe(grid)
    active = probe['contour_active'][:, None]
    plotting.draw_quiver(ax, X, Y, np.where(active, np.nan, probe['F']))
    plotting.draw_quiver(ax, X, Y, np.where(active, probe['F'], np.nan), color=plotting.CONTOUR)
    plotting.draw_path_panel(ax, run)
    outline = plotting.robot_outline(run.result.pose[first], run.tracker.footprint.r_circ)
    ax.plot(outline[:, 0], outline[:, 1], color=plotting.INK, lw=1.0, zorder=7)
    lam, d_in, d_in_blend = plotting.obstacle_offsets(run.tracker.config)
    labels = ((lam, r'$\lambda$'), (d_in, r'$D_{in}$'), (d_in_blend, r"$D'_{in}$"))
    for angle, (radius, text) in zip((40.0, 55.0, 70.0), labels):
        a = math.radians(angle)
        ax.annotate(
            text,
            (0.3 + radius) * np.array([math.cos(a), math.sin(a)]) + [0.0, 0.05],
            xytext=(2, 2),
            textcoords='offset points',
            color=plotting.INK_SECONDARY,
            fontsize=8,
        )
    _axes(ax, extent)
    ax.legend(loc='lower center', ncol=5)
    return fig


def cbf_disc(plt):
    """(6) The CBF filter around a disc 0.2 m off a straight path (math_spec section 6.7)."""
    config = vf.CoreConfig()
    config.obstacles.obstacle_mode = 'cbf'
    line = vf.make_curve(vf.CurveSpec.polyline([[-3.0, 0.0], [3.0, 0.0]]))
    obstacles = Obstacles(circles=[(0.0, 0.2, 0.3)])
    scenario = simulation.Scenario(config, line, (-3.0, 0.0, 0.0), 40.0, obstacles=obstacles)
    run = simulation.simulate(scenario)
    first = int(np.flatnonzero(run.cbf_active)[0])
    field = plotting.TrackerField(
        config, line, tuple(run.result.pose[first]), scenario.goal(), run.obstacle_map
    )
    extent = (-2.4, 3.3, -1.5, 1.2)
    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    X, Y, grid = plotting.grid_points(extent, 30)
    probe = field.probe(grid)
    active = probe['cbf_active'][:, None]
    plotting.draw_quiver(ax, X, Y, np.where(active, np.nan, probe['F']))
    plotting.draw_quiver(ax, X, Y, np.where(active, probe['F'], np.nan), color=plotting.CONTOUR)
    plotting.draw_path_panel(ax, run)
    outline = plotting.robot_outline(run.result.pose[first], run.tracker.footprint.r_circ)
    ax.plot(outline[:, 0], outline[:, 1], color=plotting.INK, lw=1.0, zorder=7)
    delta = plotting.cbf_delta(run.tracker.config, run.tracker.footprint)
    a = math.radians(50.0)
    ax.annotate(
        r'$\delta_{safe}$',
        np.array([0.0, 0.2]) + (0.3 + delta) * np.array([math.cos(a), math.sin(a)]),
        xytext=(2, 2),
        textcoords='offset points',
        color=plotting.INK_SECONDARY,
        fontsize=8,
    )
    _axes(ax, extent)
    ax.legend(loc='lower center', ncol=5)
    return fig


def models_convergence(plt):
    """(5) |D|(t) of the four motion models on a circle of radius 1.5 m, from 1 m outside."""
    circle = vf.make_curve(vf.CurveSpec.builtin('ellipse', a=1.5, b=1.5))
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    crossings = []
    for color, model in zip(plotting.SERIES, simulation.MODELS):
        config = vf.CoreConfig()
        simulation.configure(config, model=model)
        if model == 'skid_steer':
            # As kinematic_sim_demo: slip compensation set to the simulator's slip.
            motion = config.motion
            motion.skid_yaw_rate_efficiency, motion.skid_icr_offset = simulation.SKID_STEER_SLIP
        scenario = simulation.Scenario(
            config, circle, (2.5, 0.0, math.pi / 2), 40.0, slip=simulation.default_slip(config)
        )
        result = simulation.simulate(scenario).result
        D = np.maximum(np.abs(result.D), 1e-9)
        ax.semilogy(result.t, D, color=color, lw=1.6, label=model)
        below = np.flatnonzero(D < 1e-3)
        crossings.append((result.t[below[0]] if below.size else None, model, D[-1]))
    # Direct labels: at |D| = 1 mm, merged where the curves coincide, or at the end.
    placed = []
    for t, model, final in sorted(crossings, key=lambda c: (c[0] is None, c[0] or 0.0)):
        if t is None:
            ax.annotate(
                model, (40.0, final), xytext=(0, 5), textcoords='offset points', ha='right'
            )
        elif placed and abs(placed[-1][0] - t) < 0.6:
            placed[-1][1].append(model)
        else:
            placed.append((t, [model]))
    for k, (t, models) in enumerate(placed):
        # The earliest group is labelled on its left, the later ones on their right.
        side = (-6, 'right') if k == 0 else (6, 'left')
        ax.annotate(
            '\n'.join(models),
            (t, 1e-3),
            xytext=(side[0], 0),
            textcoords='offset points',
            ha=side[1],
            va='center',
        )
    ax.set_ylim(plotting.D_FLOOR, 2.0)
    ax.set_xlim(0.0, 40.0)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('|D| (m)')
    ax.legend(loc='center right')
    return fig


FIGURES = {
    'field_ellipse.svg': field_ellipse,
    'field_decomposition.svg': field_decomposition,
    'terminal_profile.svg': terminal_profile,
    'contour_disc.svg': contour_disc,
    'cbf_disc.svg': cbf_disc,
    'models_convergence.svg': models_convergence,
}


def main(argv=None):
    """Write every figure (or those named) and report the sizes."""
    argp = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    argp.add_argument(
        'names', nargs='*', help=f'figures to write (default all: {", ".join(FIGURES)})'
    )
    argp.add_argument('--output', type=Path, default=OUTPUT, help='output directory')
    args = argp.parse_args(argv)
    unknown = sorted(set(args.names) - set(FIGURES))
    if unknown:
        argp.error(f'unknown figures: {", ".join(unknown)}')
    plt = _cli.pyplot()
    plotting.apply_style(plt)
    args.output.mkdir(parents=True, exist_ok=True)
    status = 0
    for name in args.names or FIGURES:
        fig = FIGURES[name](plt)
        path = args.output / name
        fig.savefig(path, metadata=_cli.svg_metadata(path))
        plt.close(fig)
        size = path.stat().st_size
        print(f'{path}: {size / 1000:.0f} kB')
        if size > MAX_BYTES:
            print(f'  larger than {MAX_BYTES // 1000} kB: simplify the figure')
            status = 1
    return status


if __name__ == '__main__':
    raise SystemExit(main())
