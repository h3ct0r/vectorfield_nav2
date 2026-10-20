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

"""Plotting helpers of the vf-* CLIs and docs/gen_figures.py.

matplotlib is imported inside the functions that draw, never at module import. Colours follow
the RViz legend for the field decomposition (red convergence, green circulation, blue
feedforward) and a fixed categorical order for series.
"""

import math

import numpy as np
import vectorfield as vf

from ._cli import require
from .curves import sample
from .simulation import format_metres

#: Lower limit of the log-scale |D| axes (m).
D_FLOOR = 1e-4

INK = '#0b0b0b'
INK_SECONDARY = '#52514e'
GRID = '#e4e3df'
CURVE = INK
TRAJECTORY = '#2a78d6'
CONTOUR = '#eb6834'
OBSTACLE = '#a3a19b'
QUIVER = '#b9b7b0'
STREAM = '#2a78d6'
#: Categorical series, in this fixed order (validated for colour-vision deficiencies).
SERIES = ('#2a78d6', '#eb6834', '#1baf7a', '#eda100')
#: The paper's decomposition, with the RViz legend hues.
DECOMPOSITION = {
    'convergence': '#e34948',
    'circulation': '#008300',
    'feedforward': '#2a78d6',
    'field': INK,
}

STYLE = {
    'font.size': 9,
    'axes.titlesize': 10,
    'axes.labelsize': 9,
    'axes.edgecolor': INK_SECONDARY,
    'axes.labelcolor': INK,
    'axes.grid': True,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'grid.color': GRID,
    'grid.linewidth': 0.6,
    'xtick.color': INK_SECONDARY,
    'ytick.color': INK_SECONDARY,
    'legend.frameon': False,
    'legend.fontsize': 8,
    'lines.linewidth': 1.6,
    'figure.facecolor': 'white',
    'savefig.facecolor': 'white',
    'savefig.bbox': 'tight',
    'svg.fonttype': 'none',
    'svg.hashsalt': 'vectorfield',
}


def apply_style(plt):
    """Use the shared figure style."""
    plt.rcParams.update(STYLE)


# -- Fields -----------------------------------------------------------------------------------


class NominalField:
    """The path field Phi of a curve (DistanceField): static or time-varying, no obstacles."""

    def __init__(self, config, curve, t=0.0):
        """Field of ``curve`` at time ``t`` with the field parameters of ``config``."""
        self.field = vf.DistanceField.from_config(config)
        self.curve = curve
        self.t = t

    def vectors(self, points):
        """Phi (N, 2) at each point (NaN where undefined)."""
        return self.field.field_vectors(self.curve, np.asarray(points, dtype=float), self.t)

    def sample(self, x):
        """Return the FieldSample at x (the paper's decomposition)."""
        return self.field.sample(self.curve, np.asarray(x, dtype=float), self.t)

    def streamline(self, start, step, max_points):
        """RK4 streamline (N, 2) from start."""
        return vf.streamline(lambda x: self.sample(x).Phi, start, step, max_points)


class TrackerField:
    """The field the controller commands (terminal field, contour) as seen from a robot pose.

    The tracker steps ``settle_steps`` times at the fixed pose, so the obstacle gate and the
    contour side reach the state they would have there; probes then reuse that state.
    """

    def __init__(self, config, curve, pose, goal=None, obstacles=None, settle_steps=5):
        """Build and settle a Tracker at ``pose``."""
        self.tracker = vf.Tracker(config)
        period = 1.0 / config.node.controller_frequency
        self.input = vf.TrackerInput(curve, pose=pose, goal=goal, obstacles=obstacles, dt=period)
        self.scratch = vf.StepScratch()
        state = self.tracker.initial_state()
        for _ in range(max(1, settle_steps)):
            self.output, state = self.tracker.step(self.input, state, self.scratch)
        self.state = state

    def probe(self, points):
        """probe_points(): F, contour_active, sigma_c, D ... at each point."""
        return self.tracker.probe_points(
            np.asarray(points, dtype=float), self.input, self.state, scratch=self.scratch
        )

    def vectors(self, points):
        """F (N, 2) at each point."""
        return self.probe(points)['F']

    def streamline(self, start, step, max_points):
        """RK4 streamline (N, 2) of F from start."""
        return self.tracker.streamline(
            np.asarray(start, dtype=float),
            self.input,
            self.state,
            step,
            max_points,
            scratch=self.scratch,
        )


def grid_points(extent, count):
    """(X, Y, points): a grid with ``count`` columns over extent = (xmin, xmax, ymin, ymax)."""
    xmin, xmax, ymin, ymax = extent
    rows = max(2, int(round(count * (ymax - ymin) / max(xmax - xmin, 1e-9))))
    X, Y = np.meshgrid(np.linspace(xmin, xmax, count), np.linspace(ymin, ymax, rows))
    return X, Y, np.column_stack([X.ravel(), Y.ravel()])


def seed_points(extent, count):
    """About ``count`` streamline seeds: along the border and on an interior grid."""
    xmin, xmax, ymin, ymax = extent
    per_side = max(2, count // 6)
    xs = np.linspace(xmin, xmax, per_side + 2)[1:-1]
    ys = np.linspace(ymin, ymax, per_side + 2)[1:-1]
    border = [np.column_stack([xs, np.full_like(xs, y)]) for y in (ymin, ymax)]
    border += [np.column_stack([np.full_like(ys, x), ys]) for x in (xmin, xmax)]
    inner = max(2, int(round(math.sqrt(max(count - 4 * per_side, 4)))))
    _, _, interior = grid_points(
        (
            xmin + 0.15 * (xmax - xmin),
            xmax - 0.15 * (xmax - xmin),
            ymin + 0.15 * (ymax - ymin),
            ymax - 0.15 * (ymax - ymin),
        ),
        inner,
    )
    return np.vstack(border + [interior])


def streamlines(field, seeds, extent, step=None, max_points=None):
    """Streamlines of a field from each seed, clipped to ``extent``."""
    xmin, xmax, ymin, ymax = extent
    size = math.hypot(xmax - xmin, ymax - ymin)
    step = step or size / 300.0
    max_points = max_points or int(1.2 * size / step)
    lines = []
    for seed in seeds:
        line = np.asarray(field.streamline(seed, step, max_points))
        inside = (
            (line[:, 0] >= xmin)
            & (line[:, 0] <= xmax)
            & (line[:, 1] >= ymin)
            & (line[:, 1] <= ymax)
        )
        stop = np.flatnonzero(~inside)
        line = line[: stop[0]] if stop.size else line
        if len(line) >= 3:
            lines.append(line)
    return lines


def draw_quiver(ax, X, Y, F, color=QUIVER, length=None, zorder=1):
    """Draw unit arrows of a field on a grid (direction only; |F| is constant off the goal)."""
    U = np.asarray(F)[:, 0].reshape(X.shape)
    V = np.asarray(F)[:, 1].reshape(X.shape)
    norm = np.hypot(U, V)
    norm[norm == 0.0] = np.nan
    if length is None:
        length = 0.7 * min(abs(X[0, 1] - X[0, 0]), abs(Y[1, 0] - Y[0, 0]))
    ax.quiver(
        X,
        Y,
        U / norm * length,
        V / norm * length,
        angles='xy',
        scale_units='xy',
        scale=1.0,
        pivot='mid',
        color=color,
        width=0.0035,
        headwidth=4.0,
        headlength=4.5,
        headaxislength=4.0,
        zorder=zorder,
    )


def draw_streamlines(ax, lines, color=STREAM, linewidth=0.8, alpha=0.85, decimate=3):
    """Streamlines with one arrowhead each."""
    for line in lines:
        reduced = np.vstack([line[::decimate], line[-1:]])
        ax.plot(reduced[:, 0], reduced[:, 1], color=color, lw=linewidth, alpha=alpha, zorder=2)
        mid = len(line) // 3
        if mid + 1 < len(line):
            ax.annotate(
                '',
                xy=line[mid + 1],
                xytext=line[mid],
                arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': linewidth, 'alpha': alpha},
                zorder=2,
            )


def draw_curve(ax, curve, t=0.0, color=CURVE, linewidth=1.8, label='curve', zorder=4):
    """Draw the curve with its traversal direction, and the goal end of an open curve."""
    points = sample(curve, t=t)
    ax.plot(points[:, 0], points[:, 1], color=color, lw=linewidth, label=label, zorder=zorder)
    length = curve.length(t)
    s = 0.3 * length
    p, tangent = np.asarray(curve.point_at(s, t)), np.asarray(curve.tangent_at(s, t))
    ax.annotate(
        '',
        xy=p + 0.04 * length * tangent,
        xytext=p,
        arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': linewidth, 'mutation_scale': 14},
        zorder=zorder,
    )
    if not curve.is_closed:
        end = np.asarray(curve.point_at(curve.end_station(t), t))
        ax.plot(*end, marker='*', ms=11, color=color, zorder=zorder + 1, ls='none', label='goal')


def obstacle_offsets(config):
    """(lambda, D_in, D'_in) of the configuration, auto values resolved."""
    resolved = vf.resolve_auto_values(config).obstacles
    return (
        resolved.obstacle_contour_distance,
        resolved.obstacle_full_switch_distance,
        resolved.obstacle_blend_start_distance,
    )


def cbf_delta(config, footprint):
    """delta_safe of obstacle_mode cbf at the law's evaluation point (math_spec section 6.7)."""
    law = vf.resolve_tracking_law(config.motion.motion_model, config.motion.tracking_law)
    offset = 0.0
    if law == vf.TrackingLaw.FEEDBACK_LINEARIZATION:
        sign = -1.0 if config.motion.drive_backward else 1.0
        offset = sign * config.motion.control_point_offset
    return footprint.cover_radius((offset, 0.0)) + config.obstacles.cbf_margin


def draw_offsets(ax, config, obstacle_map, color=CONTOUR, footprint=None):
    """Draw the lambda (dashed), D_in and D'_in (dotted) level sets of the obstacle distance.

    With obstacle_mode cbf and a footprint, draw the delta_safe level set (dashed) instead.
    """
    grid = obstacle_map.grid
    distance = obstacle_map.edt.distance_grid()
    if not np.isfinite(distance).any():
        return
    ox, oy = grid.origin
    res = grid.resolution
    xs = ox + (np.arange(grid.width) + 0.5) * res
    ys = oy + (np.arange(grid.height) + 0.5) * res
    levels = sorted(set(obstacle_offsets(config)))
    if config.obstacles.obstacle_mode == vf.ObstacleMode.CBF and footprint is not None:
        levels = [cbf_delta(config, footprint)]
    ax.contour(
        xs,
        ys,
        distance,
        levels=levels,
        colors=color,
        linestyles=['--', ':', ':'][: len(levels)],
        linewidths=0.9,
        zorder=2,
    )


def draw_decomposition(ax, field_sample, scale=2.0, label=True):
    """Arrows -eta G n, eta H T, Phi_T and F at the sample's point, and its closest point."""
    x = np.asarray(field_sample.x_eval)
    x_star = np.asarray(field_sample.x_star)
    ax.plot(*np.column_stack([x, x_star]), color=INK_SECONDARY, lw=0.8, ls='--', zorder=5)
    ax.plot(*x_star, marker='o', ms=4, color=INK, zorder=6)
    ax.plot(*x, marker='o', ms=4, color=INK, zorder=6)
    if label:
        ax.annotate(
            r'$x^*$', x_star, xytext=(-14, -10), textcoords='offset points', color=INK, fontsize=9
        )
    arrows = [
        ('convergence', field_sample.conv, r'$-\eta G\,n$'),
        ('circulation', field_sample.circ, r'$\eta H\,T$'),
        ('feedforward', field_sample.Phi_T, r'$\Phi_T$'),
        ('field', field_sample.Phi_gamma, r'$F$'),
    ]
    for role, vector, text in arrows:
        vector = np.asarray(vector)
        if np.linalg.norm(vector) < 1e-9:
            continue
        tip = x + scale * vector
        width = 2.2 if role == 'field' else 1.4
        ax.annotate(
            '',
            xy=tip,
            xytext=x,
            arrowprops={
                'arrowstyle': '-|>',
                'color': DECOMPOSITION[role],
                'lw': width,
                'mutation_scale': 12,
                'shrinkA': 0,
                'shrinkB': 0,
            },
            zorder=7,
        )
        if label:
            direction = vector / np.linalg.norm(vector)
            anchor, offset = tip, 12.0 * direction
            if role == 'convergence':  # beside the arrow: its tip points at x*
                anchor, offset = (x + tip) / 2.0, 11.0 * np.array([direction[1], -direction[0]])
            ax.annotate(
                text,
                anchor,
                xytext=tuple(offset),
                textcoords='offset points',
                ha='center',
                va='center',
                color=INK,
                fontsize=9,
                zorder=8,
            )


def draw_terminal_profile(ax, v_r, v_min, sigma_b, xy_tolerances, sigma_range=(-0.3, 1.0)):
    """phi(sigma) for each goal tolerance, with the braking zone and the speed floor."""
    sigma = np.linspace(sigma_range[0], sigma_range[1], 801)
    ax.axvspan(0.0, sigma_b, color=GRID, alpha=0.7, lw=0, zorder=0)
    ax.text(
        0.5 * sigma_b,
        1.06,
        f'braking zone, $\\sigma_b$ = {sigma_b:g} m',
        ha='center',
        va='bottom',
        color=INK_SECONDARY,
        fontsize=8,
    )
    ax.axhline(v_min / v_r, color=INK_SECONDARY, lw=0.8, ls=':', zorder=1)
    ax.text(
        sigma_range[1],
        v_min / v_r + 0.03,
        r'floor $v_{\min}/v_r$',
        ha='right',
        va='bottom',
        color=INK_SECONDARY,
        fontsize=8,
    )
    widths = np.linspace(3.2, 1.4, len(xy_tolerances)) if len(xy_tolerances) > 1 else [1.8]
    for color, width, xy_tol in zip(SERIES, widths, xy_tolerances):
        profile = vf.TerminalProfile(v_r, v_min, sigma_b, xy_tol / 2.0)
        ax.plot(sigma, profile(sigma), color=color, lw=width, label=f'xy_tol {xy_tol:g} m')
    ax.axhline(0.0, color=INK_SECONDARY, lw=0.6)
    ax.axvline(0.0, color=INK_SECONDARY, lw=0.6)
    ax.set_xlim(*sigma_range)
    ax.set_ylim(-0.75, 1.2)
    ax.set_xlabel(r'remaining arc length $\sigma$ (m)')
    ax.set_ylabel(r'$\varphi(\sigma)$')
    ax.legend(loc='lower right')


# -- Closed-loop runs -------------------------------------------------------------------------


def _segments_where(points, mask):
    """Split ``points`` into runs of consecutive rows where ``mask`` holds."""
    if mask is None or not np.any(mask):
        return []
    edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(int), [0]])))
    return [points[a:b] for a, b in zip(edges[::2], edges[1::2]) if b - a >= 2]


def robot_outline(pose, radius):
    """Return a disc outline with a heading tick (closed polyline) for drawing a robot."""
    x, y, yaw = pose
    a = np.linspace(0.0, 2.0 * math.pi, 25)
    disc = np.column_stack([x + radius * np.cos(a + yaw), y + radius * np.sin(a + yaw)])
    return np.vstack([[x, y], disc])


def draw_path_panel(ax, run, title=None):
    """Curve, obstacles, contour offsets and the base_link trajectory of a Run."""
    scenario, result = run.scenario, run.result
    draw_curve(ax, scenario.curve, t=scenario.t0)
    if scenario.obstacles:
        scenario.obstacles.draw(ax, color=OBSTACLE)
        draw_offsets(ax, run.tracker.config, run.obstacle_map, footprint=run.tracker.footprint)
    pose = result.pose
    ax.plot(pose[:, 0], pose[:, 1], color=TRAJECTORY, lw=1.4, label='base_link', zorder=5)
    for k, segment in enumerate(_segments_where(pose[:, :2], run.contour_active)):
        ax.plot(
            segment[:, 0],
            segment[:, 1],
            color=CONTOUR,
            lw=2.2,
            zorder=6,
            label='contour active' if k == 0 else None,
        )
    for k, segment in enumerate(_segments_where(pose[:, :2], run.cbf_active)):
        ax.plot(
            segment[:, 0],
            segment[:, 1],
            color=CONTOUR,
            lw=2.2,
            ls=(0, (2, 1)),
            zorder=6,
            label='cbf active' if k == 0 else None,
        )
    ax.plot(*pose[0, :2], marker='o', ms=6, color=TRAJECTORY, ls='none', zorder=7, label='start')
    outline = robot_outline(pose[-1], run.tracker.footprint.r_circ)
    ax.plot(outline[:, 0], outline[:, 1], color=TRAJECTORY, lw=1.0, zorder=7)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    if title:
        ax.set_title(title)
    ax.legend(loc='best')


def plot_simulation(plt, run, summary, title=None):
    """Path, |D|(t), v(t) and omega(t) of a Run (one figure, one y-scale per panel)."""
    result = run.result
    fig = plt.figure(figsize=(10.0, 5.2), layout='constrained')
    grid = fig.add_gridspec(3, 2, width_ratios=[1.2, 1.0])
    draw_path_panel(fig.add_subplot(grid[:, 0]), run, title)
    ax_d = fig.add_subplot(grid[0, 1])
    ax_v = fig.add_subplot(grid[1, 1], sharex=ax_d)
    ax_w = fig.add_subplot(grid[2, 1], sharex=ax_d)
    t = result.t
    ax_d.semilogy(t, np.maximum(np.abs(result.D), 1e-9), color=TRAJECTORY, lw=1.4)
    ax_d.set_ylim(bottom=D_FLOOR)
    ax_d.axhline(summary.tolerance, color=INK_SECONDARY, lw=0.8, ls='--')
    ax_d.axvline(t[0] + summary.transient, color=INK_SECONDARY, lw=0.8, ls=':')
    ax_d.set_ylabel('|D| (m)')
    ax_d.set_title(
        f'final |D| {format_metres(summary.final_D)}, '
        f'max after transient {format_metres(summary.max_D)}',
        fontsize=9,
    )
    ax_v.plot(t, result.cmd[:, 0], color=SERIES[0], lw=1.2, label='$v_x$')
    if np.any(np.abs(result.cmd[:, 1]) > 1e-9):
        ax_v.plot(t, result.cmd[:, 1], color=SERIES[1], lw=1.2, label='$v_y$')
        ax_v.legend(loc='best')
    ax_v.set_ylabel('v (m/s)')
    ax_w.plot(t, result.cmd[:, 2], color=SERIES[0], lw=1.2)
    ax_w.set_ylabel(r'$\omega$ (rad/s)')
    ax_w.set_xlabel('t (s)')
    for ax in (ax_d, ax_v):
        ax.tick_params(labelbottom=False)
    return fig


def animate_simulation(plt, run, path, fps=12, seconds=10.0, dpi=72):
    """Write a GIF of a Run (time-lapse to at most ``seconds``); returns the frame count."""
    require('PIL')
    from matplotlib.animation import FuncAnimation, PillowWriter

    scenario, result = run.scenario, run.result
    n = len(result)
    period = scenario.control_period()
    length = min(seconds, n * period)
    frames = np.unique(np.linspace(0, n - 1, max(2, int(round(fps * length)))).round().astype(int))
    fig, (ax, ax_d) = plt.subplots(
        1, 2, figsize=(6.4, 3.0), gridspec_kw={'width_ratios': [1.25, 1.0]}
    )
    fig.subplots_adjust(left=0.11, right=0.98, bottom=0.16, top=0.88, wspace=0.3)
    draw_curve(ax, scenario.curve, t=scenario.t0, linewidth=1.4)
    if scenario.obstacles:
        scenario.obstacles.draw(ax, color=OBSTACLE)
    pose = result.pose
    points = np.vstack([pose[:, :2], sample(scenario.curve, t=scenario.t0)])
    lo, hi = points.min(axis=0) - 0.4, points.max(axis=0) + 0.4
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    (trail,) = ax.plot([], [], color=TRAJECTORY, lw=1.4, zorder=5)
    (robot,) = ax.plot([], [], color=TRAJECTORY, lw=1.2, zorder=6)
    radius = run.tracker.footprint.r_circ
    t = result.t
    D = np.maximum(np.abs(result.D), 1e-9)
    ax_d.semilogy(t, D, color=GRID, lw=1.0)
    (d_line,) = ax_d.semilogy([], [], color=TRAJECTORY, lw=1.4)
    ax_d.set_xlim(t[0], t[-1])
    ax_d.set_ylim(D_FLOOR, max(D.max() * 2.0, 10.0 * D_FLOOR))
    ax_d.set_xlabel('t (s)')
    ax_d.set_ylabel('|D| (m)')
    title = fig.suptitle('', fontsize=9, color=INK)

    def update(frame):
        k = frames[frame]
        trail.set_data(pose[: k + 1, 0], pose[: k + 1, 1])
        outline = robot_outline(pose[k], radius)
        robot.set_data(outline[:, 0], outline[:, 1])
        d_line.set_data(t[: k + 1], D[: k + 1])
        title.set_text(f't = {t[k]:5.1f} s    |D| = {abs(result.D[k]):.3f} m')
        return trail, robot, d_line, title

    animation = FuncAnimation(fig, update, frames=len(frames), blit=False)
    animation.save(str(path), writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return len(frames)
