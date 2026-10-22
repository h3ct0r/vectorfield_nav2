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

"""Example 3D curves for the experimental 3D follower (send_curve3d.py, the system tests).

A :class:`CurveDescription3` is a curve as ``vectorfield_msgs/Curve3`` carries it, without its
frame and time: a built-in shape with its parameters (and an optional placement), expressions
x(s, t), y(s, t), z(s, t), or a 3D polyline. :data:`EXAMPLES` holds the demo curves.
:func:`curve3_msg` turns a description into the goal's ``Curve3``, and :func:`curve_spec3` into a
``vectorfield.d3.CurveSpec`` (vectorfield_py).
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CurveDescription3:
    """A curve of a FollowCurve3 goal, without frame and time.

    Attributes:
        kind: ``builtin``, ``expression`` or ``polyline``.
        shape: The built-in shape (``builtin``).
        parameters: Named constants of a built-in or an expression, as (name, value) pairs.
        position: The built-in's placement (x, y, z) in the curve frame (``Curve3.pose``).
        yaw: The built-in's rotation about z (rad), part of ``Curve3.pose``.
        x: The expression x(s, t).
        y: The expression y(s, t).
        z: The expression z(s, t).
        s_min: The start of the expression domain.
        s_max: The end of the expression domain.
        points: The polyline vertices, as (x, y, z) triples.
        closed: Closed curve (polylines and expressions; built-ins decide).

    """

    kind: str
    shape: str = ''
    parameters: tuple = ()
    position: tuple = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    x: str = ''
    y: str = ''
    z: str = ''
    s_min: float = 0.0
    s_max: float = 0.0
    points: tuple = ()
    closed: bool = False


def builtin(shape, position=(0.0, 0.0, 0.0), yaw=0.0, **parameters):
    """Return a built-in shape placed at ``position`` and turned by ``yaw``.

    For example ``builtin('helix', (0, 0, 0.5), radius=1.0, turns=2)``.
    """
    values = tuple((name, float(value)) for name, value in parameters.items())
    return CurveDescription3(
        'builtin',
        shape=shape,
        parameters=values,
        position=tuple(float(v) for v in position),
        yaw=float(yaw),
    )


def expression(x, y, z, s_min, s_max, closed=True, **parameters):
    """Return the curve (x(s, t), y(s, t), z(s, t)) for s in [s_min, s_max]."""
    values = tuple((name, float(value)) for name, value in parameters.items())
    return CurveDescription3(
        'expression',
        x=x,
        y=y,
        z=z,
        s_min=float(s_min),
        s_max=float(s_max),
        closed=closed,
        parameters=values,
    )


def polyline(points, closed=False):
    """Return the 3D polyline through ``points`` ((x, y, z) triples)."""
    vertices = tuple((float(x), float(y), float(z)) for x, y, z in points)
    return CurveDescription3('polyline', points=vertices, closed=closed)


# The demo curves, around the origin of their frame at flying height (z about 1 m).
EXAMPLES = {
    # Open: (r cos s, r sin s, lead s / 2pi), two turns from z = 0.5.
    'helix': builtin('helix', (0.0, 0.0, 0.5), radius=1.0, lead=0.5, turns=2.0),
    # Closed, crossing itself at (R, 0, 0): a sphere of radius R cut by a cylinder.
    'viviani': builtin('viviani', (-0.6, 0.0, 1.2), radius=1.2),
    # Closed: the (p, q) torus knot.
    'torus_knot': builtin(
        'torus_knot', (0.0, 0.0, 1.0), p=2.0, q=3.0, major_radius=1.2, minor_radius=0.4
    ),
    # Closed: a figure eight lifted by a sine.
    'lemniscate3d': builtin('lemniscate3d', (0.0, 0.0, 1.0), a=1.5, h=0.4),
    # Closed and time-varying: a circle whose height breathes by +-c at w rad/s.
    'expression': expression(
        'r*cos(s)',
        'r*sin(s)',
        '1 + 0.3*sin(2*s) + c*sin(w*t)',
        0.0,
        2.0 * math.pi,
        r=1.2,
        c=0.2,
        w=0.3,
    ),
    # A circle of radius 0.8 in point_sim_3d's moving frame (``moving_frame:=platform``).
    'moving': builtin('circle', radius=0.8),
}

# The examples whose frame moves (FRAME_MOVING).
MOVING_EXAMPLES = ('moving',)


def curve3_msg(description, frame_id=''):
    """Return the ``vectorfield_msgs/Curve3`` of ``description`` (stamp zero: goal acceptance)."""
    from geometry_msgs.msg import Point

    from vectorfield_msgs.msg import Curve3

    msg = Curve3()
    msg.header.frame_id = frame_id
    msg.type = {
        'polyline': Curve3.POLYLINE,
        'expression': Curve3.EXPRESSION,
        'builtin': Curve3.BUILTIN,
    }[description.kind]
    msg.closed = description.closed
    msg.points = [Point(x=x, y=y, z=z) for x, y, z in description.points]
    msg.x_expression = description.x
    msg.y_expression = description.y
    msg.z_expression = description.z
    msg.s_min = description.s_min
    msg.s_max = description.s_max
    msg.builtin_shape = description.shape
    msg.parameter_names = [name for name, _ in description.parameters]
    msg.parameter_values = [value for _, value in description.parameters]
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = description.position
    msg.pose.orientation.z = math.sin(0.5 * description.yaw)
    msg.pose.orientation.w = math.cos(0.5 * description.yaw)
    return msg


def curve_spec3(description):
    """Return the ``vectorfield.d3.CurveSpec`` of ``description`` (needs vectorfield_py).

    A built-in's placement becomes its ``cx cy cz yaw`` parameters, as the follower does.
    """
    from vectorfield import d3

    parameters = dict(description.parameters)
    if description.kind == 'builtin':
        cx, cy, cz = description.position
        parameters.update(cx=cx, cy=cy, cz=cz, yaw=description.yaw)
        return d3.CurveSpec.builtin(description.shape, **parameters)
    if description.kind == 'expression':
        return d3.CurveSpec.expression(
            description.x,
            description.y,
            description.z,
            description.s_min,
            description.s_max,
            closed=description.closed,
            parameters=parameters,
        )
    return d3.CurveSpec.polyline(list(description.points), closed=description.closed)
