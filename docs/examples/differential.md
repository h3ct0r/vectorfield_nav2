# Differential: PAL TIAGo

```{clip} differential
:alt: PAL TIAGo in MuJoCo drives along its Nav2 plan, contours around an unmapped box and rejoins the path
:controls:
```

PAL TIAGo drives a Nav2 route with the vector-field controller. An unmapped box stands on its
path: the controller contours around it, rejoins the path and turns to the goal heading, then
does the same on the way back.

```bash
ros2 launch vectorfield_mujoco differential.launch.py
```

```{list-table}
:class: vf-facts

* - Mode
  - `motion_model: differential`, feedback linearisation
* - Robot
  - `tiago`: PAL TIAGo from the MuJoCo Menagerie (`pal_tiago/tiago_velocity.xml`), upper body frozen with the arm tucked; two wheel velocity servos
* - Driven by
  - Nav2's `navigate_to_pose` with `VectorFieldController` as `FollowPath`, `obstacle_mode: contour`
* - World
  - `open_10x10` and an unmapped 0.4 m box at (0.5, 0)
* - Files
  - [`launch/differential.launch.py`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/launch/differential.launch.py), [`config/robots/tiago.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/robots/tiago.yaml), [`config/scenarios/differential.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/scenarios/differential.yaml)
```

## What you see

The robot starts at (−2, 0) and drives to (3, 0), then back. The global costmap of this
scenario has no obstacle layer, so Nav2 plans straight through the box; only the lidar and the
local costmap see it. When the path ahead is blocked, the contour gate opens and the robot
circulates around the box at the contour distance, on its free side, until the path is clear
again ({doc}`../obstacle_avoidance`). At each goal it stops and turns in place to the goal yaw.

- [Blue line]{.vf-key .vf-blue}: Nav2's plan (`/plan`).
- [Green ring]{.vf-key .vf-green}: the footprint; orange while the speed is capped, red when blocked.
- [Red dots and line]{.vf-key .vf-red}: during a contour episode, the path samples that opened the gate and
  the line to the nearest obstacle point.
- [Purple sphere and cyan arrow]{.vf-key .vf-purple}: the goal and its yaw.
- [Dark line]{.vf-key .vf-trail}: the robot's path (recordings only).
- [Orange box]{.vf-key .vf-obstacle}: the unmapped obstacle; walls from the map would be grey.

The clip leaves some markers out for clarity (the curve's extension rays, the control point,
the field's components, the command's arc, the braking zone); RViz shows them all, with the
status text ({doc}`../visualization`).

## Configuration

The robot overlay keeps the controller and the velocity smoother within what the wheels can do:
5 rad/s on 0.098 m wheels is 0.49 m/s straight or 2.4 rad/s in place. The scenario adds the box
and switches the global costmap's obstacle layer off.

::::{tab-set}

:::{tab-item} Scenario

```{literalinclude} ../../vectorfield_mujoco/config/scenarios/differential.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/scenarios/differential.yaml
```

:::

:::{tab-item} Robot

```{literalinclude} ../../vectorfield_mujoco/config/robots/tiago.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/robots/tiago.yaml
```

:::
::::

## Things to try

- `obstacles:=none` drives the same route without the box.
- With the example running, `ros2 param set /controller_server FollowPath.obstacle_mode none`
  makes the robot stop short of the box with `BLOCKED` instead of contouring it; Nav2's
  behaviour tree then replans or recovers.
- `scenario:=false` starts only the world; send goals with RViz's **2D Goal Pose**.

The {doc}`index` lists the arguments every example takes (viewing, recording, start pose).

## Adapt it to your robot

The controller keys in `tiago.yaml` are the ones a real differential base needs on top of the
motion-model overlay: the speed, the limits from the wheels' maximum speed, and the footprint.
{doc}`../tutorials/your_robot` puts them into your own `nav2_params.yaml`, step by step, with
the velocity smoother and collision monitor settings. To run another MuJoCo model:

1. Add it to `vectorfield_mujoco/registry.py`: its MJCF, base body and wheel actuators. The
   wheel radius and track are read from the model.
2. Add it to `ROBOT_MODELS` in `vectorfield_mujoco/examples.py` with its motion model (an
   aerial robot goes in `AERIAL_ROBOTS` instead); the launch files accept only the robots
   listed there.
3. Write its `config/robots/<robot>.yaml`.
4. For a MuJoCo Menagerie model, add its directory to `ROBOT_DIRS` in
   `vectorfield_mujoco/menagerie.py`, so that the fetch includes it.

Then start `mujoco_nav2.launch.py robot:=<robot>`.

## Troubleshooting

- **"Could not fetch the MuJoCo Menagerie robots"**: the first run needs network access and
  git. Run `ros2 run vectorfield_mujoco fetch_menagerie` once online, or set
  `VF_MENAGERIE_DIR` to a `mujoco_menagerie` checkout.
- **The robot never moves**: the scenario waits for Nav2 to become active; the first start can
  take a while on a slow machine. Look for `Managed nodes are active` in the log.
- **`viewer` is refused**: the viewer needs a display and `MUJOCO_GL=glfw`. Without a display,
  use `camera_topic:=camera` ({doc}`index`).
