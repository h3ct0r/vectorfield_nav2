# Skid-steer: AgileX Scout Mini

```{clip} skid_steer
:alt: AgileX Scout Mini in MuJoCo drives along a corridor, through a 0.8 m door and back
:controls:
```

The AgileX Scout Mini, a four-wheel skid-steer base, drives a Nav2 route through the 0.8 m
door of a corridor and back. Its wheels skid when it turns, as a real skid-steer base's do;
the controller compensates the slip it was identified with.

```bash
ros2 launch vectorfield_mujoco skid_steer.launch.py
```

```{list-table}
:class: vf-facts

* - Mode
  - `motion_model: skid_steer`, feedback linearisation with slip compensation
* - Robot
  - `scout_mini`: AgileX Scout Mini converted from its Gazebo description and vendored in the package (no download); four in-wheel velocity servos, the Gazebo wheel friction
* - Driven by
  - Nav2's `navigate_to_pose` with `VectorFieldController` as `FollowPath`
* - World
  - `corridor_door`: a 2 m wide corridor along +x with a 0.8 m door at x = 5 m
* - Files
  - [`launch/skid_steer.launch.py`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/launch/skid_steer.launch.py), [`config/robots/scout_mini.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/robots/scout_mini.yaml), [`config/scenarios/skid_steer.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/scenarios/skid_steer.yaml)
```

## What you see

The 0.55 m wide robot leaves (0, 0), passes the door with 0.12 m to spare on each side, drives
to the end of the corridor at (9, 0), turns around in place and drives back. Its wheels realise
a little less than half the commanded yaw rate; without compensation it would run outside
every turn. The robot overlay sets the identified slip (`skid_yaw_rate_efficiency`,
`skid_icr_offset`), so the controller scales its yaw-rate command to match
({ref}`skid-slip`).

- [Blue line]{.vf-key .vf-blue}: Nav2's plan (`/plan`).
- [Green rectangle]{.vf-key .vf-green}: the footprint; orange while the speed is capped, red when blocked.
- [Purple sphere]{.vf-key .vf-purple}: the goal.
- [Dark line]{.vf-key .vf-trail}: the robot's path (recordings only).
- [Grey walls]{.vf-key .vf-wall}: the map's occupied cells, built into the MuJoCo world from the same image
  map_server serves.

## Configuration

The slip values come from `identify_skid_slip.py` run on this model, as on a real robot (the
comment in the robot overlay has the command and the result). The Scout Mini model itself is
documented in
[`models/scout_mini/README.md`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/models/scout_mini/README.md):
the wheel geometry and the 20 kg mass from the AgileX user manual, and every change from the
Gazebo description.

::::{tab-set}

:::{tab-item} Scenario

```{literalinclude} ../../vectorfield_mujoco/config/scenarios/skid_steer.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/scenarios/skid_steer.yaml
```

:::

:::{tab-item} Robot

```{literalinclude} ../../vectorfield_mujoco/config/robots/scout_mini.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/robots/scout_mini.yaml
```

:::
::::

## Things to try

- `world:=corridor_corner scenario:=false` puts the robot in the L-shaped corridor; send a goal
  at (6, 6) to take its 90° corner.
- The slip identification itself: start the generic launch away from the walls and run the
  identification script, as the robot overlay's comment shows:

  ```bash
  ros2 launch vectorfield_mujoco mujoco_nav2.launch.py robot:=scout_mini \
      initial_pose_x:=-2.0 initial_pose_y:=-2.0
  ros2 run vectorfield_bringup identify_skid_slip.py --yes --use-sim-time --repeats 2 --max-distance 4.0
  ```

## Adapt it to your robot

Skid-steer bases differ mostly in how much they slip. Identify your robot's slip on its own
floor before setting `skid_yaw_rate_efficiency` and `skid_icr_offset`
({doc}`../tutorials/field_testing`, stage 6): over-compensation is as bad as none.
{doc}`../tutorials/your_robot` covers the rest of the configuration: the footprint, the limits
and the velocity smoother.

## Troubleshooting

- **The robot touches the door frame**: the door leaves 0.12 m on each side. Check that the
  footprint in `scout_mini.yaml` still matches the model if you changed it.
- **It turns too little or overshoots in place**: the slip values no longer match the model,
  for example after a friction change; identify them again as above.
- **Nothing happens for a while**: the scenario waits for Nav2 to become active.
