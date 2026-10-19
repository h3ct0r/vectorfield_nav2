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

"""Example curves for the curve follower and the client-side approach (send_curve.py).

A :class:`CurveDescription` is a curve as ``vectorfield_msgs/Curve`` carries it, without its
frame and time: a built-in shape with its parameters, muParser expressions x(s, t), y(s, t) or a
polyline. :data:`EXAMPLES` writes the demo curves from their math. :func:`curve_msg` turns a
description into the goal's ``Curve``, :func:`curve_spec` into a ``vectorfield.CurveSpec``
(vectorfield_py), and :func:`approach_pose` gives the pose to drive to before following it.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CurveDescription:
    """A curve of a FollowCurve goal, without frame and time.

    Attributes:
        kind: ``builtin``, ``expression`` or ``polyline``.
        shape: The built-in shape (``builtin``).
        parameters: Named constants of a built-in or an expression, as (name, value) pairs.
        x: The expression x(s, t).
        y: The expression y(s, t).
        s_min: The start of the expression domain.
        s_max: The end of the expression domain.
        points: The polyline vertices, as (x, y) pairs.
        closed: Closed curve (built-ins always are).

    """

    kind: str
    shape: str = ''
    parameters: tuple = ()
    x: str = ''
    y: str = ''
    s_min: float = 0.0
    s_max: float = 0.0
    points: tuple = ()
    closed: bool = True


def builtin(shape, **parameters):
    """Return a built-in shape, e.g. ``builtin('ellipse', a=2.0, b=1.0)``."""
    values = tuple((name, float(value)) for name, value in parameters.items())
    return CurveDescription('builtin', shape=shape, parameters=values)


def expression(x, y, s_min, s_max, closed=True, **parameters):
    """Return the curve (x(s, t), y(s, t)) for s in [s_min, s_max], with named constants."""
    values = tuple((name, float(value)) for name, value in parameters.items())
    return CurveDescription(
        'expression',
        x=x,
        y=y,
        s_min=float(s_min),
        s_max=float(s_max),
        closed=closed,
        parameters=values,
    )


def polyline(points, closed=False):
    """Return the polyline through ``points`` ((x, y) pairs)."""
    vertices = tuple((float(x), float(y)) for x, y in points)
    return CurveDescription('polyline', points=vertices, closed=closed)


# The demo curves, sized for the open_10x10 map and centred on the origin of their frame.
EXAMPLES = {
    # (a cos s, b sin s).
    'ellipse': builtin('ellipse', a=2.0, b=1.0),
    # a (cos s, sin s cos s) / (1 + sin² s): (x² + y²)² = a² (x² − y²), crossing at the origin.
    'lemniscate_bernoulli': builtin('lemniscate_bernoulli', a=2.0),
    # (a cos s, b sin 2s).
    'lemniscate_gerono': builtin('lemniscate_gerono', a=2.0, b=1.0),
    # |x/a|ⁿ + |y/b|ⁿ = 1.
    'superellipse': builtin('superellipse', a=2.0, b=1.5, n=4.0),
    # Open: y = A sin(k x) for x in [−3, 3].
    'sine': expression('s', 'A*sin(k*s)', -3.0, 3.0, closed=False, A=0.5, k=1.5),
    # Closed and time-varying: an ellipse whose semi-axes breathe by ±c at w rad/s.
    'expression': expression(
        '(a + c*sin(w*t))*cos(s)',
        '(b + c*sin(w*t))*sin(s)',
        0.0,
        2.0 * math.pi,
        a=2.0,
        b=1.0,
        c=0.3,
        w=0.2,
    ),
    # A circle of radius 1.5 m in a moving frame (moving_frame_publisher.py).
    'moving': builtin('ellipse', a=1.5, b=1.5),
}

# The examples whose frame moves (FRAME_MOVING).
MOVING_EXAMPLES = ('moving',)


def curve_msg(description, frame_id=''):
    """Return the ``vectorfield_msgs/Curve`` of ``description`` (stamp zero: goal acceptance)."""
    from geometry_msgs.msg import Point

    from vectorfield_msgs.msg import Curve

    msg = Curve()
    msg.header.frame_id = frame_id
    msg.type = {
        'polyline': Curve.POLYLINE,
        'expression': Curve.EXPRESSION,
        'builtin': Curve.BUILTIN,
    }[description.kind]
    msg.closed = description.closed
    msg.points = [Point(x=x, y=y) for x, y in description.points]
    msg.x_expression = description.x
    msg.y_expression = description.y
    msg.s_min = description.s_min
    msg.s_max = description.s_max
    msg.builtin_shape = description.shape
    msg.parameter_names = [name for name, _ in description.parameters]
    msg.parameter_values = [value for _, value in description.parameters]
    return msg


def curve_spec(description):
    """Return the ``vectorfield.CurveSpec`` of ``description`` (needs vectorfield_py)."""
    import vectorfield as vf

    parameters = dict(description.parameters)
    if description.kind == 'builtin':
        return vf.CurveSpec.builtin(description.shape, **parameters)
    if description.kind == 'expression':
        return vf.CurveSpec.expression(
            description.x,
            description.y,
            description.s_min,
            description.s_max,
            closed=description.closed,
            parameters=parameters or None,
        )
    return vf.CurveSpec.polyline(list(description.points), closed=description.closed)


def approach_pose(description, x, y, reverse=False):
    """Return the pose (x, y, yaw) in the curve frame from which to start following the curve.

    A closed curve is joined at its closest point to (x, y) (``vectorfield.closest_point`` at
    t = 0), an open one at its start (its end when ``reverse``); the yaw is the tangent in the
    direction of travel.
    """
    import vectorfield as vf

    curve = vf.make_curve(curve_spec(description))
    if curve.is_closed:
        station = curve.closest_point((x, y)).s_star
    else:
        station = curve.end_station() if reverse else 0.0
    px, py = curve.point_at(station)
    tx, ty = curve.tangent_at(station)
    if reverse:
        tx, ty = -tx, -ty
    return float(px), float(py), math.atan2(ty, tx)
