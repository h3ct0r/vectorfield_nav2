# Quickstart

This page runs the controller and the curve follower in simulation, then the Python tools
without ROS. The {doc}`visualization` guide explains what RViz shows. The tutorials go through
the same steps in more detail ({doc}`tutorials/getting_started`), run every demo scenario
({doc}`tutorials/scenarios`) and put the controller on your own robot
({doc}`tutorials/your_robot`).

## Build

The packages build from source in a colcon workspace on Humble, Jazzy and Lyrical:

```bash
mkdir -p ~/vf_ws/src && cd ~/vf_ws/src
git clone https://github.com/h3ct0r/vectorfield_nav2.git
cd ~/vf_ws
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

`vectorfield_mujoco`, the package of the MuJoCo examples, needs MuJoCo from pip (rosdep key
`python3-mujoco-pip`). Ubuntu Noble and later (Jazzy, Lyrical) mark the system Python as
externally managed, so rosdep's pip install fails there without
`PIP_BREAK_SYSTEM_PACKAGES=1`, as the CI and the devcontainers set it. To leave the examples
out instead, skip the key and the package:

```bash
rosdep install --from-paths src --ignore-src -y --skip-keys python3-mujoco-pip
colcon build --symlink-install --packages-skip vectorfield_mujoco
```

The repository also has one devcontainer per distro under `.devcontainer/` (`humble`, `jazzy`,
`lyrical` and `rolling`). In one of them, run `colcon build --symlink-install`, then
`colcon test && colcon test-result --verbose`, from `/ws`. The Rolling container is for
compiling and unit tests only; run the demos in the Lyrical container.

## MuJoCo examples

One command per mode runs the controller on a physics model of a robot, with a scenario that
drives it by itself:

```bash
ros2 launch vectorfield_mujoco differential.launch.py
```

```{clip} differential
:alt: PAL TIAGo drives past an unmapped box on its free side with Nav2 and the vector-field controller, rejoins the path and turns to the goal heading.
```

The other examples are `skid_steer`, `omnidirectional`, `ackermann`, `drone` and
`curve_following`. {doc}`examples/index` shows each one with its clip and configuration, and
lists what they need; the first run downloads three robots from the MuJoCo Menagerie.

## Controller demo

`kinematic_sim_demo` runs on every supported distro. It starts `vectorfield_sim`, map_server,
Nav2's `navigation_launch.py` with the vector-field controller as `FollowPath`, and RViz with
`rviz/vectorfield.rviz`:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=omnidirectional
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py map:=corridor_door
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py map:=corridor_corner
```

On Jazzy and later, `loopback_demo` runs the same controller configuration on Nav2's loopback
simulator:

```bash
ros2 launch vectorfield_bringup loopback_demo.launch.py
```

:::{figure} _static/screenshots/demo.gif
:alt: The robot in RViz follows the plan, circulates around a box on it with the contour markers shown, rejoins the plan and stops on the goal.
:width: 100%

What RViz shows during a run of `kinematic_sim_demo` with `scenarios:=demo_obstacles` (see
{doc}`visualization` for every marker).
:::

`motion_model` is `differential` (default), `skid_steer`, `omnidirectional` or `ackermann`. The
robot starts at (0, 0) facing +x on every map; `open_10x10` is a 10 m × 10 m open area centred on
the origin, `corridor_door` a 2 m wide corridor along +x with a 0.8 m door at x = 5 m, and
`corridor_corner` a 1.2 m wide L-shaped corridor that runs along +x to a sharp 90° left corner at
(6, 0) and then along +y (send a goal at (6, 6) to take the corner). The
`vectorfield_bringup` README lists every launch argument.

Send goals with RViz's **2D Goal Pose** tool, or from a terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Goals worth trying, and what to expect:

- **Behind the robot** (as above). A differential or skid-steer robot first rotates in place
  (ROTATE_TO_FIELD, amber status text) until the field is within 45° of its heading, then
  follows the plan.
- **At the robot's own position with a new yaw.** The robot only turns in place
  (ROTATE_TO_HEADING, blue) and ends aligned with the goal yaw.
- **A few metres ahead.** The robot converges to the plan (FOLLOW, green), slows down over the last
  `terminal_braking_distance` (0.5 m) of the path (FINAL_APPROACH, cyan) and stops on the goal.
  When the goal yaw differs from the arrival heading, the rotation to it starts once the robot is
  inside half the XY tolerance and ends aligned; otherwise Nav2 ends the goal when the robot
  enters the XY tolerance.
- **0.4 m from a wall, or through the door** (`map:=corridor_door`). The robot reaches the goal
  without obstacle contouring: the door is free, so the contour gate stays closed.
- **Around a sharp corner** (`map:=corridor_corner`, goal (6, 6)). The robot follows the plan
  through the 90° corner of the 1.2 m corridor without touching its walls.

::::{grid} 1 2 2 2
:gutter: 2

:::{grid-item}

```{clip} viz_rotate_to_field
:poster: screenshots/rotate_to_field.png
:class: vf-natural
:alt: The robot turns in place toward a goal behind it, the amber heading arrow and arc showing the turn still to do.
```

ROTATE_TO_FIELD: a goal behind the robot.
:::

:::{grid-item}

```{clip} viz_rotate_to_heading
:poster: screenshots/rotate_to_heading.png
:class: vf-natural
:alt: The robot on the goal turns in place to the goal yaw, with the blue target arrow and arc.
```

ROTATE_TO_HEADING: a new yaw at the same spot.
:::
::::

```{clip} viz_final_approach
:poster: screenshots/final_approach.png
:class: vf-natural
:alt: The robot drives along the plan and slows down onto the goal inside its tolerance rings.
```

FINAL_APPROACH: the last part of a goal a few metres ahead, braking onto it.

In RViz, check:

- the status text above the robot, whose colour changes with the phase;
- the red and green field-component arrows and the slate resulting field at the control point;
- the cyan command arrow;
- the green prediction line;
- the goal rings (XY tolerance and $\sigma_{\mathrm{stop}}$) and the orange braking zone;
- the field grid (the **Field grid** display, on by default), which shows the whole field
  around the robot.

## Obstacles

`demo_obstacles` adds unmapped obstacles that only the simulated laser sees, among them a 0.4 m
box 2 m ahead of the start pose. The demo overlays set `obstacle_mode: contour` (except
Ackermann). The overlay also turns the global costmap's obstacle layer off (the local costmap
keeps its own), so the planner keeps the plan through the box and the robot meets it on its path
instead of planning around it:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py scenarios:=demo_obstacles
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

The robot passes the box on its free side at about the contour distance $\lambda$, rejoins the
path and stops at the goal. While it circulates, the status reads `CONTOUR +1` or `CONTOUR -1`,
and the `contour` markers show the path samples that opened the gate, the obstacle cells, the
$\lambda$, $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ rings, the chosen side and the watchdog
progress.

```{clip} viz_contour
:poster: screenshots/contour.png
:class: vf-natural
:alt: The robot circulates around a box on its plan, with the contour markers, then rejoins the plan.
```

The obstacles are run-time parameters of the simulator:

```bash
ros2 param set /vectorfield_sim box_obstacles "[2.0, 0.0, 0.4, 0.4, 0.0]"
```

With `ros2 param set /controller_server FollowPath.obstacle_mode none` the robot stops short of
the box instead and the status reads `BLOCKED`. The controller then raises `NoValidControl`, and
Nav2's behaviour tree replans or recovers; with the stock global obstacle layer, the new plan goes
around the box. {doc}`obstacle_avoidance` explains both modes.

```{clip} viz_blocked
:poster: screenshots/blocked.png
:class: vf-natural
:alt: The robot stops in front of a wall across its plan, its footprint red, a red collision cube ahead and BLOCKED in the status text.
```

## Curve follower

`curve_follower_demo` adds the `follow_curve` action server to the kinematic demo; Nav2 keeps
running, and its local costmap is the follower's obstacle input:

```bash
ros2 launch vectorfield_bringup curve_follower_demo.launch.py
ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
```

```{clip} curve_following
:alt: An AgileX Scout Mini drives laps of a lemniscate in MuJoCo with the curve follower.
```

The clip is the MuJoCo example of the same action ({doc}`examples/curve_following`).

`send_curve.py` first drives the robot to the curve with NavigateToPose, then sends the goal,
prints the feedback and exits with status 0 once the three laps are done. Other shapes are
`ellipse`, `lemniscate_gerono`, `superellipse`, the open `sine`, a time-varying `expression` and
`moving`; `--laps 0` runs until Ctrl-C, which cancels the goal. A curve attached to a moving
frame needs the frame publisher:

```bash
ros2 run vectorfield_bringup moving_frame_publisher.py --ros-args -p use_sim_time:=true \
  -p vx:=0.1 -p y:=1.5
ros2 run vectorfield_bringup send_curve.py --shape moving --laps 0
```

A NavigateToPose goal sent while the follower runs (for example with 2D Goal Pose) stops the
follower with `NAV2_ACTIVE`. {doc}`curve_follower` documents the action.

## Python tools (no ROS)

The Python bindings and tools build with pip from a checkout, for example on macOS. They need a
C++17 compiler, CMake 3.22 or newer and Eigen 3.4 or newer (`brew install cmake eigen`):

```bash
git clone https://github.com/h3ct0r/vectorfield_nav2.git && cd vectorfield_nav2
python3 -m venv .venv
. .venv/bin/activate
pip install './vectorfield_py[tools]'
vf-sim --model differential --curve ellipse --gif ellipse.gif
vf-plot --curve lemniscate_bernoulli:a=2 -o field.svg
```

:::{figure} _static/figures/field_ellipse.svg
:alt: Streamlines of the vector field around an ellipse, converging onto it and circulating along it.
:width: 70%

A field plotted with the Python tools: integral curves converge to the ellipse and then travel
along it ({doc}`theory`).
:::

`vf-tune` checks a parameter file and sweeps a parameter in simulation (see {doc}`tuning`).
`vectorfield_core` itself also builds and tests without ROS:

```bash
brew install cmake ninja eigen googletest
cd vectorfield_core && cmake --preset dev && cmake --build --preset dev && ctest --preset dev
```
