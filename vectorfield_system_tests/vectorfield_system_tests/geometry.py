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

"""Clearance geometry for the obstacle assertions, and the controller values they depend on."""

import math

import numpy as np

#: nav2_bringup's TB3: the robot_radius of both stock costmaps.
ROBOT_RADIUS = 0.22
#: The controller's default control_point_offset d; no shipped overlay sets it.
CONTROL_POINT_OFFSET = 0.2
#: The controller's default stop_margin.
STOP_MARGIN = 0.10
#: The stock costmap resolution.
RESOLUTION = 0.05
#: The Ackermann overlay's min_turning_radius (config/controller/ackermann.yaml).
MIN_TURNING_RADIUS = 0.5


def evaluation_offset(motion_model):
    """Return where the field is evaluated, ahead of base_footprint (m).

    The feedback-linearization law of the differential and skid-steer models uses the control
    point d; the holonomic (omnidirectional) and heading-tracking (Ackermann, reference point 0)
    laws use the base itself.
    """
    return CONTROL_POINT_OFFSET if motion_model in ('differential', 'skid_steer') else 0.0


def auto_contour_distance(motion_model, radius=ROBOT_RADIUS):
    """Return the controller's auto contour distance λ for a circular footprint.

    vectorfield_core's ``autoContourDistance``: max(√((w_lat + 0.10)² + d²), r_circ + d + 0.05),
    with w_lat = r_circ = ``radius`` for a disc; for Ackermann ``ackermannAutoContourDistance``,
    at least min_turning_radius + w_lat + 0.05.
    """
    d = evaluation_offset(motion_model)
    lam = max(math.hypot(radius + 0.10, d), radius + d + 0.05)
    if motion_model == 'ackermann':
        lam = max(lam, MIN_TURNING_RADIUS + radius + 0.05)
    return lam


def ackermann_contour_bound(ahead, side, r_min=MIN_TURNING_RADIUS, radius=ROBOT_RADIUS):
    """Return the Ackermann counterpart of the contour bound λ − d − 0.05 (m).

    A box whose near face is ``ahead`` m in front of the rear axle, with its near corner on the
    passing side ``side`` m off the path, lies inside the car's turn-in band. The car starting at
    rest turns at ``r_min`` at once, so its rear axle passes that corner at
    √(ahead² + (r_min − side)²) − r_min; the bound is the smaller of this and λ, minus 0.05.
    """
    turn = math.hypot(ahead, r_min - side) - r_min
    return min(auto_contour_distance('ackermann', radius), turn) - 0.05


def box_distance(x, y, box):
    """Return the distance from (x, y) to a box ``[x, y, size_x, size_y, yaw]`` (0 inside)."""
    cx, cy, size_x, size_y, yaw = box
    dx, dy = x - cx, y - cy
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return math.hypot(max(abs(local_x) - 0.5 * size_x, 0.0), max(abs(local_y) - 0.5 * size_y, 0.0))


class OccupiedCells:
    """The occupied cells of an ``OccupancyGrid`` (axis-aligned origin), as squares."""

    def __init__(self, grid, threshold=65):
        info = grid.info
        cells = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
        rows, columns = np.nonzero(cells >= threshold)
        self._half = 0.5 * info.resolution
        self._x = info.origin.position.x + (columns + 0.5) * info.resolution
        self._y = info.origin.position.y + (rows + 0.5) * info.resolution

    def distance(self, x, y):
        """Return the distance from (x, y) to the nearest occupied cell (0 inside one)."""
        if self._x.size == 0:
            return math.inf
        gap_x = np.maximum(np.abs(self._x - x) - self._half, 0.0)
        gap_y = np.maximum(np.abs(self._y - y) - self._half, 0.0)
        return float(np.min(np.hypot(gap_x, gap_y)))
