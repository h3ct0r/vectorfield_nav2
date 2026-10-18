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

"""Curves from the command line: built-ins, expression pairs and polyline files.

A curve argument is one of

- a built-in shape, optionally with parameters: ``ellipse``, ``ellipse:a=2,b=1``;
- an expression pair in ``s`` (and ``t``) separated by ``;``: ``'2*cos(s); sin(s)'``, with the
  domain from ``--s-range`` (default ``0:2pi``);
- a polyline file: CSV with one ``x,y`` row per vertex, or YAML, either shaped like
  ``vectorfield_msgs/msg/Curve`` (what ``vf-curve --export`` writes) or a plain list of points.
"""

import csv
import math
from pathlib import Path

import numpy as np
import vectorfield as vf

from ._cli import InputError, assignment, number, require

DEFAULT_CURVE = 'ellipse'
_CLOSURE_TOLERANCE = 1e-9
_TYPE_NAMES = {t.name.lower(): t for t in vf.CurveSpec.Type.__members__.values()}


def builtin_names():
    """Names of the built-in shapes."""
    return [shape['name'] for shape in vf.builtin_shapes()]


def add_curve_arguments(argp, default=DEFAULT_CURVE):
    """Add the curve options shared by every CLI."""
    group = argp.add_argument_group('curve')
    group.add_argument(
        '--curve',
        default=default,
        metavar='CURVE',
        help=(
            'built-in shape with optional parameters (ellipse:a=2,b=1), expression pair '
            "'x(s); y(s)', or polyline file (.csv, .yaml)"
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
        help='domain of an expression pair (default 0:2pi)',
    )
    closure = group.add_mutually_exclusive_group()
    closure.add_argument(
        '--closed',
        dest='closed',
        action='store_const',
        const=True,
        help='the expression or polyline is closed (default: detected)',
    )
    closure.add_argument('--open', dest='closed', action='store_const', const=False)
    group.add_argument(
        '--sample-spacing',
        type=number,
        default=0.0,
        metavar='M',
        help='polyline resampling spacing (m; default 0: the curve options default)',
    )


def _parameters(entries):
    params = {}
    for entry in entries:
        name, value = assignment(entry, convert=number)
        params[name] = value
    return params


def spec_from_args(args):
    """Return the CurveSpec of the curve arguments (not validated yet)."""
    return parse_curve(
        args.curve,
        params=_parameters(args.param),
        s_range=args.s_range,
        closed=args.closed,
        sample_spacing=args.sample_spacing,
    )


def curve_from_args(args):
    """(spec, curve) from the curve arguments; raises CurveError for an invalid curve."""
    spec = spec_from_args(args)
    return spec, build_curve(spec, auto_closed=args.closed is None)


def parse_curve(text, params=None, s_range='0:2pi', closed=None, sample_spacing=0.0):
    """Parse a curve argument into a CurveSpec (see the module docstring)."""
    params = dict(params or {})
    text = str(text).strip()
    name, _, inline = text.partition(':')
    if name in builtin_names():
        for entry in filter(None, (part.strip() for part in inline.split(','))):
            key, value = assignment(entry, convert=number)
            params.setdefault(key, value)
        return vf.CurveSpec.builtin(name, **params)
    if ';' in text:
        x_expr, _, y_expr = text.partition(';')
        lo, sep, hi = str(s_range).partition(':')
        if not sep:
            raise InputError(f'--s-range needs MIN:MAX, got {s_range!r}')
        return vf.CurveSpec.expression(
            x_expr.strip(),
            y_expr.strip(),
            number(lo),
            number(hi),
            closed=True if closed is None else closed,
            parameters=params or None,
        )
    path = Path(text)
    if path.suffix.lower() in ('.csv', '.txt', '.yaml', '.yml') or path.exists():
        spec = read_curve_file(path, closed=closed)
        if sample_spacing > 0.0:
            spec.sample_spacing = sample_spacing
        return spec
    raise InputError(
        f'unknown curve {text!r}: not a built-in ({", ".join(builtin_names())}), '
        "an expression pair 'x(s); y(s)' or a .csv/.yaml file"
    )


def build_curve(spec, auto_closed=False, options=None):
    """make_curve(); with ``auto_closed`` an expression that does not close is retried open."""
    options = options or vf.CurveOptions()
    try:
        return vf.make_curve(spec, options)
    except vf.CurveError as closed_error:
        if not (auto_closed and spec.type == vf.CurveSpec.Type.EXPRESSION and spec.closed):
            raise
        open_spec = spec.copy()
        open_spec.closed = False
        try:
            return vf.make_curve(open_spec, options)
        except vf.CurveError as open_error:
            raise (closed_error if open_error.code == 'UNSUPPORTED' else open_error) from None


def _polyline_spec(points, closed):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2 or len(points) < 2:
        raise InputError('a polyline needs at least two [x, y] points')
    points = points[:, :2]
    joined = bool(np.linalg.norm(points[0] - points[-1]) <= _CLOSURE_TOLERANCE)
    if closed is None:
        closed = joined and len(points) > 3
    if closed and joined:
        points = points[:-1]
    return vf.CurveSpec.polyline(points, closed=bool(closed))


def read_curve_file(path, closed=None):
    """Read a CurveSpec from a CSV or YAML polyline file, or a Curve.msg-shaped YAML file."""
    path = Path(path)
    if path.suffix.lower() in ('.yaml', '.yml'):
        yaml = require('yaml')
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        return spec_from_mapping(data, closed=closed, source=path)
    rows = []
    with path.open(encoding='utf-8', newline='') as stream:
        for row in csv.reader(line for line in stream if not line.lstrip().startswith('#')):
            cells = [cell.strip() for cell in row if cell.strip()]
            if not cells:
                continue
            try:
                rows.append([float(cell) for cell in cells[:2]])
            except ValueError:
                if rows:
                    raise InputError(f'{path}: not a number in row {row}') from None
                continue  # a header row
    if not rows:
        raise InputError(f'{path}: no x,y rows')
    return _polyline_spec(rows, closed)


def _point(entry):
    if isinstance(entry, dict):
        return [float(entry.get('x', 0.0)), float(entry.get('y', 0.0))]
    return [float(value) for value in list(entry)[:2]]


def spec_from_mapping(data, closed=None, source='curve'):
    """Build a CurveSpec from parsed YAML: a Curve.msg mapping, ``{points: [...]}`` or points."""
    if isinstance(data, dict) and isinstance(data.get('curve'), dict):
        data = data['curve']  # a FollowCurve goal
    if isinstance(data, list):
        return _polyline_spec([_point(p) for p in data], closed)
    if not isinstance(data, dict):
        raise InputError(f'{source}: expected a Curve mapping or a list of points')
    kind = data.get('type')
    if kind is None:
        if data.get('builtin_shape'):
            kind = vf.CurveSpec.Type.BUILTIN
        elif data.get('x_expression'):
            kind = vf.CurveSpec.Type.EXPRESSION
        else:
            kind = vf.CurveSpec.Type.POLYLINE
    elif isinstance(kind, str):
        if kind.lower() not in _TYPE_NAMES:
            raise InputError(f'{source}: unknown curve type {kind!r}')
        kind = _TYPE_NAMES[kind.lower()]
    else:
        kind = vf.CurveSpec.Type(int(kind))
    names = list(data.get('parameter_names') or [])
    values = [float(v) for v in data.get('parameter_values') or []]
    if len(names) != len(values):
        raise InputError(f'{source}: parameter_names and parameter_values differ in length')
    if kind == vf.CurveSpec.Type.POLYLINE:
        flag = data.get('closed') if closed is None else closed
        spec = _polyline_spec([_point(p) for p in data.get('points') or []], flag)
    else:
        spec = vf.CurveSpec(
            type=kind,
            closed=bool(data.get('closed', False)) if closed is None else closed,
            x_expression=str(data.get('x_expression') or ''),
            y_expression=str(data.get('y_expression') or ''),
            s_min=float(data.get('s_min') or 0.0),
            s_max=float(data.get('s_max') or 0.0),
            builtin_shape=str(data.get('builtin_shape') or ''),
            parameter_names=names,
            parameter_values=values,
        )
    spec.sample_spacing = float(data.get('sample_spacing') or 0.0)
    spec.max_corner_deviation = float(data.get('max_corner_deviation') or 0.0)
    return spec


def spec_to_mapping(spec, frame_id='map'):
    """Return the ``vectorfield_msgs/msg/Curve`` mapping of a spec (for messages and goals)."""
    return {
        'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': frame_id},
        'type': int(spec.type),
        'closed': bool(spec.closed),  # ignored for BUILTIN (every built-in is closed)
        'points': [{'x': float(x), 'y': float(y), 'z': 0.0} for x, y in spec.points],
        'x_expression': spec.x_expression,
        'y_expression': spec.y_expression,
        's_min': float(spec.s_min),
        's_max': float(spec.s_max),
        'builtin_shape': spec.builtin_shape,
        'parameter_names': list(spec.parameter_names),
        'parameter_values': [float(v) for v in spec.parameter_values],
        'sample_spacing': float(spec.sample_spacing),
        'max_corner_deviation': float(spec.max_corner_deviation),
    }


def write_curve_yaml(path, spec, frame_id='map'):
    """Write a Curve.msg-shaped YAML file."""
    yaml = require('yaml')
    with Path(path).open('w', encoding='utf-8') as stream:
        stream.write(
            '# vectorfield_msgs/msg/Curve (vf-curve --export); type 0 POLYLINE, 1 EXPRESSION, '
            '2 BUILTIN\n'
        )
        yaml.safe_dump(spec_to_mapping(spec, frame_id), stream, sort_keys=False)


def write_curve_csv(path, points):
    """Write an ``x,y`` CSV polyline (closed curves repeat their first point at the end)."""
    with Path(path).open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['x', 'y'])
        writer.writerows([[repr(float(x)), repr(float(y))] for x, y in points])


def describe_spec(spec):
    """One-line description of a spec."""
    params = ', '.join(f'{k}={v:g}' for k, v in spec.parameters.items())
    if spec.type == vf.CurveSpec.Type.BUILTIN:
        return f'builtin {spec.builtin_shape}' + (f' ({params})' if params else '')
    if spec.type == vf.CurveSpec.Type.EXPRESSION:
        text = f'expression ({spec.x_expression}; {spec.y_expression}), s in [{spec.s_min:g}, '
        return text + f'{spec.s_max:g}]' + (f', {params}' if params else '')
    return f'polyline of {len(spec.points)} vertices'


def sample(curve, ds=None, t=0.0):
    """Points along the curve every ``ds`` (default: about 400 samples, at most 0.05 m)."""
    if ds is None or ds <= 0.0:
        ds = min(0.05, max(curve.length() / 400.0, 1e-3))
    return curve.sample(ds=ds, t=t)


def default_pose(curve, offset=1.0, t=0.0):
    """Return a start pose ``offset`` right of the curve start, facing along the tangent."""
    p, tangent = np.asarray(curve.point_at(0.0, t)), np.asarray(curve.tangent_at(0.0, t))
    right = np.array([tangent[1], -tangent[0]])
    x, y = p + offset * right
    return (float(x), float(y), float(math.atan2(tangent[1], tangent[0])))


def bounds(points, margin=0.0):
    """(xmin, xmax, ymin, ymax) of an (N, 2) array, grown by ``margin``."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    lo, hi = points.min(axis=0) - margin, points.max(axis=0) + margin
    return (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]))
