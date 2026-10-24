# Omnidirectional: Stanford TidyBot base

```{clip} omnidirectional
:alt: Stanford TidyBot base in MuJoCo strafes around a rounded rectangle without turning
:controls:
```

The Stanford TidyBot base follows a rounded rectangle with the curve follower, strafing
sideways and backwards without ever turning: the holonomic law tracks the field with the
base's x and y velocity while its heading stays fixed.

```bash
ros2 launch vectorfield_mujoco omnidirectional.launch.py
```

```{list-table}
:class: vf-facts

* - Mode
  - `motion_model: omnidirectional`, holonomic, `omni_heading_policy: fixed`
* - Robot
  - `tidybot`: the Stanford TidyBot base from the MuJoCo Menagerie (`stanford_tidybot/base.xml`), its planar x, y and yaw joints driven by velocity servos
* - Driven by
  - the curve follower's `follow_curve` action; Nav2 runs for the approach, the local costmap and the command chain
* - World
  - `open_10x10`
* - Files
  - [`launch/omnidirectional.launch.py`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/launch/omnidirectional.launch.py), [`config/robots/tidybot.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/robots/tidybot.yaml), [`config/scenarios/omnidirectional.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/scenarios/omnidirectional.yaml)
```

## What you see

`run_scenario` first drives the base onto the curve with a NavigateToPose goal, then sends the
FollowCurve goal: two laps of the superellipse $|x/2|^4 + |y/1.5|^4 = 1$, a 4 m × 3 m rounded
rectangle. The field converges the base to the curve and circulates it along at 0.4 m/s; with
`omni_heading_policy: fixed` the yaw is held, so the base moves forwards, sideways and
backwards around the corners ({doc}`../motion_models`).

- [Teal line]{.vf-key .vf-teal}: the curve being followed.
- [Purple arrows and sphere]{.vf-key .vf-purple}: the direction of travel, every metre, and the curve's start.
- [Green square]{.vf-key .vf-green}: the footprint; orange while the speed is capped, red when blocked.
- [Green line ahead of the robot]{.vf-key .vf-green}: the predicted motion, checked for collisions.
- [Slate and cyan arrows]{.vf-key .vf-slate}: the field $F$ the robot tracks and the command sent.

## Configuration

The base is an ideal holonomic base: the Menagerie model moves on planar joints, not on its
casters, and this package swaps its position servos for velocity servos. The motion-model
overlay's limits (0.5 m/s on each axis, 1.5 rad/s) are within the servos' and are kept.

::::{tab-set}

:::{tab-item} Scenario

```{literalinclude} ../../vectorfield_mujoco/config/scenarios/omnidirectional.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/scenarios/omnidirectional.yaml
```

:::

:::{tab-item} Robot

```{literalinclude} ../../vectorfield_mujoco/config/robots/tidybot.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/robots/tidybot.yaml
```

:::
::::

## Things to try

- Other curves: start only the world and send your own goal; `send_curve.py` drives to the
  curve first, then follows it.

  ```bash
  ros2 launch vectorfield_mujoco omnidirectional.launch.py scenario:=false
  ros2 run vectorfield_bringup send_curve.py --shape lemniscate_gerono --laps 2
  ```

- Other heading policies: copy the scenario file, set `omni_heading_policy` to `path_tangent`
  (face along the curve) or `field` (face the field), and run the copy with the generic launch:
  `ros2 launch vectorfield_mujoco mujoco_curve_follower.launch.py robot:=tidybot scenario:=<file>`.

## Adapt it to your robot

A real omnidirectional base (mecanum or swerve) takes the same `motion_model: omnidirectional`
and the lateral limits `max_lateral_vel` and the velocity smoother's y axis, which must be
non-zero here, unlike on the other bases ({doc}`../tutorials/your_robot`). The curve follower's
own parameters and its action are in {doc}`../curve_follower` and
{doc}`../tutorials/curve_following`.

## Troubleshooting

- **The base only moves forwards**: the velocity smoother's y limits are zero, as on a
  differential base; set them as in `tidybot.yaml`.
- **The goal ends with `NAV2_ACTIVE`**: a Nav2 action (a NavigateToPose goal, a recovery) was
  running; the follower never commands the base while Nav2 does ({doc}`../curve_follower`).
- **"Could not fetch the MuJoCo Menagerie robots"**: see {doc}`index` (first run).
