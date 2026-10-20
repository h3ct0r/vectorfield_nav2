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

"""3D curves and 3D plots for the CLIs (the experimental ``vectorfield.d3`` core).

A 3D curve argument is one of

- a 3D built-in shape, optionally with parameters: ``helix``, ``helix:radius=1,lead=0.5``;
- an expression triple in ``s`` (and ``t``) separated by ``;``: ``'cos(s); sin(s); 0.1*s'``,
  with the domain from ``--s-range`` (default ``0:2pi``);
- a polyline file: CSV with one ``x,y,z`` row per vertex, or a YAML mapping as written by
  ``vf-curve --dim 3 --export FILE.yaml``.

The YAML mapping mirrors ``vectorfield_msgs/msg/Curve`` with ``z_expression`` and 3D points
added; it is not a ROS message (Curve.msg is planar).
"""

import csv
from pathlib import Path

import numpy as np
import vectorfield as vf
from vectorfield import d3

from ._cli import InputError, assignment, number, require

DEFAULT_CURVE = 'helix'
_TYPE_NAMES = {t.name.lower(): t for t in vf.CurveSpec.Type.__members__.values()}


def builtin_names():
    """Names of the 3D built-in shapes."""
    return [shape['name'] for shape in d3.builtin_shapes()]


def add_curve_arguments(argp, default=DEFAULT_CURVE):
    """Add the 3D curve options (``--curve``, ``-p``, ``--s-range``, ``--closed/--open``)."""
    group = argp.add_argument_group('curve (3D)')
    group.add_argument(
        '--curve',
        default=default,
        metavar='CURVE',
        help=(
            'built-in 3D shape with optional parameters (helix:radius=1,lead=0.5), expression '
            "triple 'x(s); y(s); z(s)', or polyline file (.csv x,y,z rows, .yaml)"
            + (f' (default {default})' if default else '')
        ),
    )
    group.add_argument(
        '-p',
        '--param',
        action='append',
        default=[],
        metavar='NAME=VALUE',
        help='built-in or expression parameter (repeatable)',
    )
    group.add_argument(
        '--s-range',
        default='0:2pi',
        metavar='MIN:MAX',
        help='domain of an expression triple (default 0:2pi)',
    )
    closure = group.add_mutually_exclusive_group()
    closure.add_argument(
        '--closed', dest='closed', action='store_const', const=True, help='closed curve'
    )
    closure.add_argument('--open', dest='closed', action='store_const', const=False)


def parse_curve(text, params=None, s_range='0:2pi', closed=None):
    """Parse a 3D curve argument into a ``d3.CurveSpec`` (see the module docstring)."""
    params = dict(params or {})
    text = str(text).strip()
    name, _, inline = text.partition(':')
    if name in builtin_names():
        for entry in filter(None, (part.strip() for part in inline.split(','))):
            key, value = assignment(entry, convert=number)
            params.setdefault(key, value)
        return d3.CurveSpec.builtin(name, **params)
    if text.count(';') == 2:
        x_expr, y_expr, z_expr = (part.strip() for part in text.split(';'))
        lo, sep, hi = str(s_range).partition(':')
        if not sep:
            raise InputError(f'--s-range needs MIN:MAX, got {s_range!r}')
        return d3.CurveSpec.expression(
            x_expr,
            y_expr,
            z_expr,
            number(lo),
            number(hi),
            closed=True if closed is None else closed,
            parameters=params,
        )
    path = Path(text)
    if path.suffix.lower() in ('.csv', '.txt', '.yaml', '.yml') or path.exists():
        return read_curve_file(path, closed=closed)
    raise InputError(
        f'unknown 3D curve {text!r}: not a built-in ({", ".join(builtin_names())}), '
        "an expression triple 'x(s); y(s); z(s)' or a .csv/.yaml file"
    )


def curve_from_args(args):
    """(spec, curve) from the 3D curve arguments; raises CurveError for an invalid curve."""
    params = {}
    for entry in args.param:
        key, value = assignment(entry, convert=number)
        params[key] = value
    spec = parse_curve(args.curve, params=params, s_range=args.s_range, closed=args.closed)
    return spec, d3.make_curve(spec)


def read_curve_file(path, closed=None):
    """Read a POLYLINE (CSV, YAML points) or any spec (YAML mapping) from a file."""
    path = Path(path)
    if not path.exists():
        raise InputError(f'{path}: no such file')
    if path.suffix.lower() in ('.yaml', '.yml'):
        yaml = require('yaml')
        data = yaml.safe_load(path.read_text())
        if isinstance(data, list):
            data = {'points': data}
        if not isinstance(data, dict):
            raise InputError(f'{path}: expected a mapping or a list of points')
        return spec_from_mapping(data, closed=closed)
    rows = []
    with path.open(newline='') as handle:
        for row in csv.reader(handle):
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                rows.append([float(value) for value in row[:3]])
            except ValueError:
                if rows:
                    raise InputError(f'{path}: not a number in row {row}') from None
    points = np.array(rows, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise InputError(f'{path}: expected x,y,z rows')
    if closed is None:
        closed = len(points) > 3 and np.linalg.norm(points[-1] - points[0]) < 1e-9
    if closed and np.linalg.norm(points[-1] - points[0]) < 1e-9:
        points = points[:-1]
    return d3.CurveSpec.polyline(points, closed=bool(closed))


def spec_from_mapping(data, closed=None):
    """Build a ``d3.CurveSpec`` from a mapping (the YAML of :func:`spec_to_mapping`)."""
    spec = d3.CurveSpec()
    kind = str(data.get('type', 'polyline')).lower()
    if kind not in _TYPE_NAMES:
        raise InputError(f'unknown curve type {kind!r}')
    spec.type = _TYPE_NAMES[kind]
    points = data.get('points') or []
    if points:
        spec.points = np.array(
            [
                [p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)] if isinstance(p, dict) else p
                for p in points
            ],
            dtype=float,
        )
    for key in ('x_expression', 'y_expression', 'z_expression', 'builtin_shape'):
        setattr(spec, key, str(data.get(key, '')))
    for key in ('s_min', 's_max', 't0'):
        setattr(spec, key, float(data.get(key, 0.0)))
    names = data.get('parameter_names') or []
    values = data.get('parameter_values') or []
    spec.parameters = dict(zip(names, (float(v) for v in values), strict=True))
    spec.closed = bool(data.get('closed', False)) if closed is None else bool(closed)
    return spec


def spec_to_mapping(spec, frame_id='map'):
    """Return a YAML-ready mapping of a ``d3.CurveSpec``."""
    params = spec.parameters
    return {
        'header': {'frame_id': frame_id},
        'type': spec.type.name.lower(),
        'closed': bool(spec.closed),
        'points': [{'x': float(x), 'y': float(y), 'z': float(z)} for x, y, z in spec.points],
        'x_expression': spec.x_expression,
        'y_expression': spec.y_expression,
        'z_expression': spec.z_expression,
        's_min': float(spec.s_min),
        's_max': float(spec.s_max),
        'builtin_shape': spec.builtin_shape,
        'parameter_names': list(params),
        'parameter_values': [float(v) for v in params.values()],
        't0': float(spec.t0),
    }


def write_curve_yaml(path, spec, frame_id='map'):
    """Write a 3D spec as YAML."""
    yaml = require('yaml')
    Path(path).write_text(yaml.safe_dump(spec_to_mapping(spec, frame_id), sort_keys=False))


def write_curve_csv(path, points):
    """Write (N, 3) samples as x,y,z rows."""
    with Path(path).open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['x', 'y', 'z'])
        for x, y, z in points:
            writer.writerow([f'{x:.6f}', f'{y:.6f}', f'{z:.6f}'])


def describe_spec(spec):
    """One-line description of a 3D spec."""
    kind = spec.type.name.lower()
    if kind == 'builtin':
        params = ', '.join(f'{k}={v:g}' for k, v in spec.parameters.items())
        return f'{spec.builtin_shape}' + (f' ({params})' if params else '')
    if kind == 'expression':
        return f'({spec.x_expression}; {spec.y_expression}; {spec.z_expression})'
    return f'polyline of {len(spec.points)} vertices'


def set_equal_aspect(ax, points, margin=0.1):
    """Equal scaling on a 3D axis around (N, 3) points."""
    lo = points.min(axis=0)
    hi = points.max(axis=0)
    center = 0.5 * (lo + hi)
    half = 0.5 * float(np.max(hi - lo)) * (1.0 + margin) + 1e-9
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    if hasattr(ax, 'set_box_aspect'):
        ax.set_box_aspect((1.0, 1.0, 1.0))


def streamlines(curve, field, seeds, step=0.05, count=160, t=0.0):
    """Streamlines of the 3D field from (N, 3) seeds: a list of (M, 3) arrays."""
    return [field.streamline(curve, seed, step=step, count=count, t=t) for seed in seeds]


def seeds_around(curve, count=12, offset=0.6, t=0.0, seed=1):
    """Seed points scattered about the curve (offsets of up to ``offset`` metres)."""
    rng = np.random.default_rng(seed)
    length = curve.length(t)
    stations = np.linspace(0.0, length, count, endpoint=not curve.is_closed)
    base = curve.point_at(stations, t)
    directions = rng.normal(size=(count, 3))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    return base + offset * directions


def axes3d(plt, figsize=(6.0, 5.0)):
    """Return a figure with one 3D axis (mplot3d)."""
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    return fig, ax
