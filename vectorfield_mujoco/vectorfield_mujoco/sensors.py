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

"""Simulated sensors: a 2D lidar for the ground robots and an obstacle cloud for the drone.

:class:`Lidar2D` casts its beams with ``mujoco.mj_multiRay`` from a site, in the site's
horizontal plane (so it tilts with the robot, as a mounted lidar does), with a geom-group mask
that hits only the world geoms (:data:`vectorfield_mujoco.scene.WORLD_GROUP`): never the robot,
nor the floor. The beam layout is that of a ``sensor_msgs/LaserScan`` covering 360°.

:func:`world_cloud` samples the surfaces of the world geoms about ``spacing`` apart, as
``point_sim_3d`` publishes its ``obstacle_cloud`` (spheres use the same Fibonacci sampling).
"""

import math

import numpy as np

from vectorfield_mujoco.scene import WORLD_GROUP, WORLD_GROUP_MASK

#: Golden angle of the Fibonacci sphere (rad), as in point_sim_3d.
GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))


class Lidar2D:
    """A planar lidar on a site.

    Args:
        model: The compiled model.
        site: Site id (or name) of the lidar origin; the scan plane is the site's x-y plane.
        beams: Number of beams over 360°; beam i points at ``angle_min + i * angle_increment``.
        range_min, range_max: Valid range (m); other returns read ``inf``.
        geomgroup: ``mj_multiRay`` geom-group mask (default: the world group only).
    """

    def __init__(
        self, model, site, beams=360, range_min=0.05, range_max=12.0, geomgroup=WORLD_GROUP_MASK
    ):
        self.site = site if isinstance(site, int) else model.site(site).id
        self.beams = int(beams)
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.geomgroup = np.asarray(geomgroup, dtype=np.uint8)
        self.angle_increment = 2.0 * math.pi / self.beams
        self.angle_min = -math.pi
        self.angle_max = self.angle_min + (self.beams - 1) * self.angle_increment
        angles = self.angle_min + self.angle_increment * np.arange(self.beams)
        self._local = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(self.beams)))
        self._geomid = np.zeros(self.beams, dtype=np.int32)
        self._dist = np.zeros(self.beams, dtype=np.float64)

    def scan(self, model, data):
        """Return the ranges (m, ``inf`` where nothing valid was hit), one per beam."""
        origin = data.site_xpos[self.site].copy()
        rot = data.site_xmat[self.site].reshape(3, 3)
        vec = (self._local @ rot.T).ravel()
        multi_ray(
            model,
            data,
            origin,
            vec,
            self.geomgroup,
            self._geomid,
            self._dist,
            self.beams,
            self.range_max,
        )
        ranges = self._dist.copy()
        ranges[(ranges < self.range_min) | (ranges > self.range_max) | (self._geomid < 0)] = np.inf
        return ranges


def multi_ray(model, data, origin, vec, geomgroup, geomid, dist, nray, cutoff):
    """Call ``mujoco.mj_multiRay`` (MuJoCo 3.3+ added a ``normal`` output argument)."""
    import mujoco

    try:
        mujoco.mj_multiRay(
            model, data, origin, vec, geomgroup, 1, -1, geomid, dist, None, nray, cutoff
        )
    except TypeError:  # pragma: no cover - MuJoCo < 3.3
        mujoco.mj_multiRay(model, data, origin, vec, geomgroup, 1, -1, geomid, dist, nray, cutoff)


def sphere_points(radius, spacing):
    """Points on a sphere about the origin, about ``spacing`` apart (Fibonacci sampling)."""
    n = int(math.ceil(4.0 * math.pi * radius * radius / (spacing * spacing))) + 1
    k = np.arange(n)
    z = 1.0 - 2.0 * (k + 0.5) / n
    rho = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    theta = GOLDEN_ANGLE * k
    return radius * np.column_stack((rho * np.cos(theta), rho * np.sin(theta), z))


def cylinder_points(radius, half_height, spacing):
    """Points on a z-axis cylinder (side and caps) about the origin, about ``spacing`` apart."""
    n_theta = max(3, int(math.ceil(2.0 * math.pi * radius / spacing)))
    n_z = max(2, int(math.ceil(2.0 * half_height / spacing)) + 1)
    theta = 2.0 * math.pi * np.arange(n_theta) / n_theta
    zs = np.linspace(-half_height, half_height, n_z)
    t, z = np.meshgrid(theta, zs)
    side = np.column_stack((radius * np.cos(t).ravel(), radius * np.sin(t).ravel(), z.ravel()))
    cap = [np.zeros((1, 2))]
    for r in np.arange(spacing, radius - 0.5 * spacing, spacing):
        m = max(3, int(math.ceil(2.0 * math.pi * r / spacing)))
        a = 2.0 * math.pi * np.arange(m) / m
        cap.append(np.column_stack((r * np.cos(a), r * np.sin(a))))
    cap = np.concatenate(cap)
    caps = [np.column_stack((cap, np.full(len(cap), s * half_height))) for s in (-1.0, 1.0)]
    return np.concatenate([side, *caps])


def box_points(half, spacing):
    """Points on the faces of a box with half-extents ``half`` about the origin."""
    half = np.asarray(half, dtype=float)
    axes = [np.linspace(-h, h, max(2, int(math.ceil(2.0 * h / spacing)) + 1)) for h in half]
    faces = []
    for k in range(3):
        i, j = [a for a in range(3) if a != k]
        u, v = np.meshgrid(axes[i], axes[j])
        for sign in (-1.0, 1.0):
            face = np.zeros((u.size, 3))
            face[:, i], face[:, j], face[:, k] = u.ravel(), v.ravel(), sign * half[k]
            faces.append(face)
    return np.unique(np.concatenate(faces).round(9), axis=0)


def geom_points(geom_type, size, spacing):
    """Surface points of a sphere, cylinder or box geom in its own frame (else empty)."""
    import mujoco

    if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        return sphere_points(size[0], spacing)
    if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return cylinder_points(size[0], size[1], spacing)
    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        return box_points(size[:3], spacing)
    return np.zeros((0, 3))


def world_cloud(model, data, spacing=0.05, group=WORLD_GROUP):
    """Return an (N, 3) array of points on the world geoms of ``group``, in the world frame."""
    clouds = []
    for g in range(model.ngeom):
        if model.geom_group[g] != group:
            continue
        local = geom_points(model.geom_type[g], model.geom_size[g], spacing)
        if len(local):
            rot = data.geom_xmat[g].reshape(3, 3)
            clouds.append(local @ rot.T + data.geom_xpos[g])
    return np.concatenate(clouds) if clouds else np.zeros((0, 3))
