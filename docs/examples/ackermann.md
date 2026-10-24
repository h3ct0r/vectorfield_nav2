# Ackermann: car model

```{clip} ackermann
:alt: A front-steered car in MuJoCo swerves around a box next to its path and returns to it
:controls:
```

A front-steered, rear-driven car follows a Nav2 Smac Hybrid-A\* plan. A box stands right next
to the path, closer than the car's half width: the controller's control-barrier-function (CBF)
safety filter keeps the car's turning margin clear of it, so the car swerves around the box
and returns to the path.

```bash
ros2 launch vectorfield_mujoco ackermann.launch.py
```

```{list-table}
:class: vf-facts

* - Mode
  - `motion_model: ackermann`, `min_turning_radius: 0.5`, `obstacle_mode: cbf`
* - Robot
  - `ackermann_car`: a car built for this example from primitives (MuJoCo Menagerie has no car); steering hinges with position servos, rear wheels with velocity servos
* - Driven by
  - Nav2's `navigate_to_pose`: Smac Hybrid-A\* with the Dubins model (forward only) and `VectorFieldController` as `FollowPath`
* - World
  - `open_10x10` and an unmapped 0.3 m box whose face is 0.1 m from the path
* - Files
  - [`launch/ackermann.launch.py`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/launch/ackermann.launch.py), [`config/robots/ackermann_car.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/robots/ackermann_car.yaml), [`config/scenarios/ackermann.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/scenarios/ackermann.yaml)
```

## What you see

The car drives from (−2.5, 0) to (2.5, 0) on a straight plan. The global costmap has no
obstacle layer, so the plan passes the box; the car's 0.125 m half width would scrape it. As
the car approaches, the CBF filter changes the field just enough to keep a barrier disc about
the rear axle, widened by the car's turning margin, away from the box: the car steers out,
passes and steers back onto the path ({ref}`cbf-mode`).

- [Blue line]{.vf-key .vf-blue}: Nav2's plan (`/plan`).
- [Green rectangle]{.vf-key .vf-green}: the footprint; orange while the speed is capped, red when blocked.
- [Red sphere, dots and line]{.vf-key .vf-red}: while the filter is active, the nearest constrained obstacle
  site, the other constrained sites and the binding constraint's normal.
- [Green line ahead of the car]{.vf-key .vf-green}: the predicted motion, checked for collisions.
- [Purple sphere]{.vf-key .vf-purple}: the goal.
- [Dark line]{.vf-key .vf-trail}: the car's path (recordings only).

## Configuration

The car is sized for vectorfield_bringup's Ackermann overlay: `base_link` is the rear-axle
centre, and the 0.5 m turning radius needs 0.59 rad on the inner wheel, within the hinges'
0.65 rad. The scenario switches the obstacle mode from the overlay's `contour` to `cbf`.

::::{tab-set}

:::{tab-item} Scenario

```{literalinclude} ../../vectorfield_mujoco/config/scenarios/ackermann.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/scenarios/ackermann.yaml
```

:::

:::{tab-item} Robot

```{literalinclude} ../../vectorfield_mujoco/config/robots/ackermann_car.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/robots/ackermann_car.yaml
```

:::
::::

## Things to try

- `obstacles:=none` drives the straight path alone.
- With the example running, `ros2 param set /controller_server FollowPath.obstacle_mode contour`
  switches to Ackermann obstacle contouring for comparison ({ref}`ackermann-contour`);
  {doc}`../obstacle_avoidance` compares the two modes.
- `scenario:=false` and RViz's **2D Goal Pose**: Smac Hybrid-A\* plans with the car's turning
  radius and never reverses, so a goal behind the car gives a forward loop.

## Adapt it to your robot

On a real car-like robot, `min_turning_radius` must match the planner's
`minimum_turning_radius`, and `reference_point_offset` places the tracked point along the body
({doc}`../motion_models`, {doc}`../tutorials/your_robot`). The annotated template
`vectorfield_bringup/config/examples/` has an Ackermann `nav2_params` to start from.

## Troubleshooting

- **The car stops in front of an obstacle (`BLOCKED`, `cbf … | hold`)**: the obstacle is
  across the path, not beside it. The CBF filter stops the car there instead of circulating,
  and Nav2 replans once the global costmap sees the obstacle, which this scenario's global
  costmap does not ({ref}`cbf-mode`). The contour mode circulates around such obstacles.
- **Nav2 fails to plan**: Smac Hybrid-A\* needs room for the turning radius around the goal;
  pick goals at least a turning radius from walls.
- **Nothing happens for a while**: the scenario waits for Nav2 to become active.
