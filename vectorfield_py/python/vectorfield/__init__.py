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

"""Python bindings for vectorfield_core, the ROS-free core of vectorfield_nav2.

The compiled module ``vectorfield._vectorfield`` (pybind11, NumPy I/O) is re-exported here:

- curves: ``CurveSpec``, ``make_curve``, ``Curve``, ``PolylineCurve``, ``SearchPolicy``,
  ``ClosestPoint``, ``builtin_shapes`` and the polyline conditioning helpers;
- fields and laws: ``GainFunction``, ``DistanceField``, ``TerminalProfile``, ``FieldSample``,
  ``Kinematics``, ``ControlLaw`` and the law helpers, ``CommandLimiter``;
- obstacles: ``OccupancyGrid``, ``EuclideanDistanceTransform``, ``ObstacleMap``,
  ``FootprintChecker``, ``CollisionPredictor``;
- configuration: ``CoreConfig`` and its groups, ``validate_config`` (codes in ``issue``);
- the controller: ``Tracker``, ``TrackerInput``, ``TrackerState``, ``Phase`` and ``Status``;
- ``sim``: ``integrate`` and the ``ClosedLoopSimulator``;
- ``d3``: the experimental 3D core (curves, field, ``Tracker``, ``d3.sim``).

Points are float64 arrays of shape (2,) or (N, 2), poses (N, 3) [x, y, yaw], grids
(height, width) indexed [iy, ix]. Invalid configurations raise ``ConfigError`` (a
``ValueError``); rejected curves raise its subclass ``CurveError``.
"""

import enum as _enum
import sys as _sys

from . import _vectorfield
from ._vectorfield import *  # noqa: F403
from ._vectorfield import __version__ as __version__
from ._vectorfield import d3, issue, sim

# `import vectorfield.sim` and `from vectorfield.sim import ...` work like for a package module.
_sys.modules[f'{__name__}.sim'] = sim
_sys.modules[f'{__name__}.issue'] = issue
_sys.modules[f'{__name__}.d3'] = d3
_sys.modules[f'{__name__}.d3.sim'] = d3.sim

Status = _enum.IntFlag('Status', [('NONE', 0), *_vectorfield._STATUS_BITS], module=__name__)
Status.__doc__ = (
    'Status bits of one control cycle; values equal FieldState.msg STATUS_*. '
    'Decode an output with Status(output.status).'
)

__all__ = sorted(
    {name for name in dir(_vectorfield) if not name.startswith('_')}
    | {'ConfigError', 'CurveError', 'Status', '__version__', 'd3', 'issue', 'sim'}
)
