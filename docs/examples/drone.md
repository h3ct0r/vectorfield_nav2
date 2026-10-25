# Drone: Skydio X2

```{clip} drone
:alt: Skydio X2 in MuJoCo takes off and climbs a helix, bending around a pillar and a sphere
:controls:
```

A Skydio X2 quadrotor takes off and climbs two turns of a helix with the 3D follower. A pillar
and a sphere stand within its body radius of the curve; the follower's CBF safety filter bends
the flight around them, seeing them only through a simulated point cloud, and the field brings
the drone back to the curve.

```bash
ros2 launch vectorfield_mujoco drone.launch.py
```

```{list-table}
:class: vf-facts

* - Mode
  - the 3D core: `follow_curve_3d`, `obstacle_mode: cbf` against a voxelised point cloud
* - Robot
  - `x2`: the Skydio X2 from the MuJoCo Menagerie (`skydio_x2/x2.xml`), four thrust motors under this package's flight controller
* - Driven by
  - `vectorfield_follower_3d` (no Nav2)
* - World
  - no map: a floor, a pillar and a sphere
* - Files
  - [`launch/drone.launch.py`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/launch/drone.launch.py), [`config/robots/x2.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/robots/x2.yaml), [`config/scenarios/drone.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/scenarios/drone.yaml)
```

:::{note}
The 3D follower sends velocity setpoints, as it would to PX4's or ArduPilot's velocity mode.
Here a cascaded flight controller of this package stands in for that autopilot: a velocity
loop, a tilt-limited attitude command, an SO(3) attitude controller and a mixer to the four
rotors, at the 500 Hz physics rate. The drone's attitude and velocity lag are simulated; the
autopilot itself is not.
:::

## What you see

The drone rests on the floor below the start of the helix, a 1 m radius curve that climbs from
0.5 m to 1.5 m in two counter-clockwise turns. The follower takes it to the curve and along it
at 0.5 m/s. The pillar inside the helix and the sphere outside the first turn are 0.2 m and
0.15 m from the curve, inside the drone's 0.4 m body radius: the CBF filter keeps the body
sphere plus a 0.1 m margin off their surfaces ({ref}`core-3d-obstacles`). The helix is open, so
the goal succeeds at its end.

- [Blue line]{.vf-key .vf-blue3d}: the curve.
- [Translucent green sphere]{.vf-key .vf-body}: the body sphere the CBF filter keeps clear (`body_radius`).
- [Small orange sphere and arrow on the curve]{.vf-key .vf-amber}: the closest point and the curve's tangent.
- [Green arrow]{.vf-key .vf-green3d}: the velocity command.
- [Magenta sphere]{.vf-key .vf-magenta}: the goal, the end of the open helix.
- [Dark line]{.vf-key .vf-trail}: the drone's path (recordings only).
- [Orange pillar and sphere]{.vf-key .vf-obstacle}: the obstacles, which the follower sees only through the
  simulator's `obstacle_cloud`.

## Configuration

`mujoco_sim` stands in for `point_sim_3d`: it takes the follower's `cmd_vel` (TwistStamped, in
the global frame or, with `cmd_frame:=body`, the heading frame) and publishes `odom`, the TF
`map → base_link`, `/clock` and `obstacle_cloud`, the surfaces of the world's obstacles. The
follower's limits (1 m/s, 0.5 m/s vertical, 1.5 m/s²) stay far inside the flight controller's,
so the CBF's single-integrator model holds; its margin is twice the default because the drone
reaches a commanded velocity only after its loop's 0.33 s time constant.

::::{tab-set}

:::{tab-item} Scenario

```{literalinclude} ../../vectorfield_mujoco/config/scenarios/drone.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/scenarios/drone.yaml
```

:::

:::{tab-item} Robot

```{literalinclude} ../../vectorfield_mujoco/config/robots/x2.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/robots/x2.yaml
```

:::
::::

## Things to try

- Other curves: start the drone in the air without the scenario and send them yourself.

  ```bash
  ros2 launch vectorfield_mujoco drone.launch.py scenario:=false initial_pose_z:=1.0
  ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2
  ```

- `cmd_frame:=body` sends the setpoints in the heading frame, as ArduPilot's DDS interface
  takes them.
- `obstacles:=none` flies the helix without obstacles;
  `obstacles:="[{type: sphere, pos: [0.6, 0.0, 1.3], size: [0.25]}]"` puts a sphere elsewhere.

## Adapt it to your vehicle

On a real drone the follower talks to the autopilot instead: MAVROS, PX4 over uXRCE-DDS or
ArduPilot DDS, with a supervisor that arms and takes off, and `hold_between_goals: true` so
that the vehicle keeps getting setpoints between goals ({doc}`../follower_3d`,
{doc}`../tutorials/aerial_3d`). Keep the follower's speed and
acceleration limits well inside what the vehicle's velocity mode tracks, and widen
`cbf_margin` by how far the vehicle travels during its velocity lag.

## Troubleshooting

- **The drone grazes an obstacle**: the CBF assumes the commanded velocity is reached at once.
  Increase `cbf_margin` or lower `desired_speed` and `max_accel`.
- **The goal aborts with `STALE_INPUT`**: the follower needs `odom` and, with `cbf`, an
  `obstacle_cloud` younger than `voxel_timeout` (the simulator publishes it at 2 Hz); check
  that `mujoco_sim` runs and keeps up.
- **The drone holds and the goal aborts with `BLOCKED`**: an obstacle sits right on the curve,
  where the filter stops instead of passing it ({doc}`../follower_3d`).
- **"Could not fetch the MuJoCo Menagerie robots"**: see {doc}`index` (first run).
