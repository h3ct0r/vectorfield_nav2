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

"""Circle and box obstacles, rasterised into the occupancy grid of the core.

The layouts match vectorfield_sim's ``circle_obstacles`` ([x, y, radius]) and ``box_obstacles``
([x, y, size_x, size_y, yaw]) parameters.
"""

import math
from dataclasses import dataclass, field

import numpy as np
import vectorfield as vf

from ._cli import InputError, number, numbers

DEFAULT_RESOLUTION = 0.05


@dataclass
class Obstacles:
    """Circles (x, y, radius) and boxes (x, y, size_x, size_y, yaw) in the world frame."""

    circles: list = field(default_factory=list)
    boxes: list = field(default_factory=list)

    def __bool__(self):
        """Return True when there is at least one obstacle."""
        return bool(self.circles or self.boxes)

    def outline_points(self):
        """Points on every obstacle boundary (N, 2), for plot bounds."""
        points = [np.zeros((0, 2))]
        for x, y, r in self.circles:
            a = np.linspace(0.0, 2.0 * math.pi, 16)
            points.append(np.column_stack([x + r * np.cos(a), y + r * np.sin(a)]))
        points.extend(self.box_corners(box) for box in self.boxes)
        return np.vstack(points)

    @staticmethod
    def box_corners(box):
        """Return the four corners (4, 2) of a box."""
        x, y, sx, sy, yaw = box
        c, s = math.cos(yaw), math.sin(yaw)
        local = np.array([[-sx, -sy], [sx, -sy], [sx, sy], [-sx, sy]]) / 2.0
        return local @ np.array([[c, s], [-s, c]]) + [x, y]

    def mask(self, extent, resolution=DEFAULT_RESOLUTION):
        """(mask, origin) of a grid covering ``extent`` = (xmin, xmax, ymin, ymax)."""
        xmin, xmax, ymin, ymax = extent
        width = max(1, int(math.ceil((xmax - xmin) / resolution)))
        height = max(1, int(math.ceil((ymax - ymin) / resolution)))
        iy, ix = np.indices((height, width))
        cx = xmin + (ix + 0.5) * resolution
        cy = ymin + (iy + 0.5) * resolution
        occupied = np.zeros((height, width), dtype=bool)
        for x, y, r in self.circles:
            occupied |= (cx - x) ** 2 + (cy - y) ** 2 <= r * r
        for x, y, sx, sy, yaw in self.boxes:
            c, s = math.cos(yaw), math.sin(yaw)
            u = c * (cx - x) + s * (cy - y)
            v = -s * (cx - x) + c * (cy - y)
            occupied |= (np.abs(u) <= sx / 2.0) & (np.abs(v) <= sy / 2.0)
        return occupied.astype(np.uint8), (xmin, ymin)

    def obstacle_map(self, extent, resolution=DEFAULT_RESOLUTION):
        """Return an ObstacleMap (grid frame = world frame) over ``extent``."""
        mask, origin = self.mask(extent, resolution)
        return vf.ObstacleMap(vf.OccupancyGrid(mask, resolution, origin=origin))

    def draw(self, ax, color='0.45', **kwargs):
        """Draw the obstacles as filled patches on a matplotlib Axes."""
        from matplotlib.patches import Circle, Polygon

        for x, y, r in self.circles:
            ax.add_patch(Circle((x, y), r, color=color, zorder=3, **kwargs))
        for box in self.boxes:
            ax.add_patch(Polygon(self.box_corners(box), color=color, zorder=3, **kwargs))


def add_obstacle_arguments(argp):
    """``--circle``, ``--box``, ``--obstacle-mode`` and ``--grid-resolution``."""
    group = argp.add_argument_group('obstacles (rasterised into an occupancy grid)')
    group.add_argument(
        '--circle',
        action='append',
        default=[],
        metavar='X,Y,R',
        help='a disc obstacle (repeatable)',
    )
    group.add_argument(
        '--box',
        action='append',
        default=[],
        metavar='X,Y,SX,SY[,YAW]',
        help='a box obstacle: centre, sizes and yaw (repeatable)',
    )
    group.add_argument(
        '--obstacle-mode',
        choices=['contour', 'cbf', 'none'],
        help='with obstacles: contour them (default; none for ackermann), keep a margin with the '
        'CBF filter (cbf), or only stop before them',
    )
    group.add_argument(
        '--grid-resolution',
        type=number,
        default=DEFAULT_RESOLUTION,
        metavar='M',
        help=f'occupancy grid resolution (m; default {DEFAULT_RESOLUTION})',
    )


def obstacles_from_args(args):
    """Obstacles from the obstacle arguments."""
    circles = [tuple(numbers(c, 3, '--circle')) for c in args.circle]
    boxes = []
    for text in args.box:
        values = numbers(text, (4, 5), '--box')
        boxes.append(tuple(values + [0.0] * (5 - len(values))))
    if any(r <= 0.0 for _, _, r in circles) or any(b[2] <= 0.0 or b[3] <= 0.0 for b in boxes):
        raise InputError('obstacle sizes must be positive')
    if args.grid_resolution <= 0.0:
        raise InputError('--grid-resolution must be positive')
    return Obstacles(circles, boxes)
