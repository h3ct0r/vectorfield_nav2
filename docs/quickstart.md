# Quickstart

This page runs the controller and the curve follower in simulation, then the Python tools
without ROS. The {doc}`visualization` guide explains what RViz shows.

## Build

The packages build from source in a colcon workspace on Humble, Jazzy and Lyrical:

```bash
mkdir -p ~/vf_ws/src && cd ~/vf_ws/src
git clone https://github.com/h3ct0r/vectorfield_nav2.git
cd ~/vf_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

The repository also has one devcontainer per distro under `.devcontainer/` (`humble`, `jazzy`,
`lyrical` and `rolling`). In one of them, run `colcon build --symlink-install`, then
`colcon test && colcon test-result --verbose`, from `/ws`. The Rolling container is for
compiling and unit tests only; run the demos in the Lyrical container.

## Controller demo

`kinematic_sim_demo` runs on every supported distro. It starts `vectorfield_sim`, map_server,
Nav2's `navigation_launch.py` with the vector-field controller as `FollowPath`, and RViz with
`rviz/vectorfield.rviz`:

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py motion_model:=omnidirectional
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py map:=corridor_door
```

On Jazzy and later, `loopback_demo` runs the same controller configuration on Nav2's loopback
simulator:

```bash
ros2 launch vectorfield_bringup loopback_demo.launch.py
```

`motion_model` is `differential` (default), `skid_steer`, `omnidirectional` or `ackermann`. The
robot starts at (0, 0) facing +x on both maps; `open_10x10` is a 10 m × 10 m open area centred on
the origin, `corridor_door` a 2 m wide corridor along +x with a 0.8 m door at x = 5 m. The
`vectorfield_bringup` README lists every launch argument.

Send goals with RViz's **2D Goal Pose** tool, or from a terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Goals worth trying, and what to expect:

- **Behind the robot** (as above). A differential or skid-steer robot first rotates in place
  (ROTATE_TO_FIELD, yellow status text) until the field is within 45° of its heading, then
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

In RViz, check:

- the status text above the robot, whose colour changes with the phase;
- the red, green and white field-component arrows at the control point;
- the cyan command arrow;
- the green prediction line;
- the goal rings (XY tolerance and $\sigma_{\mathrm{stop}}$) and the orange braking zone;
- the field grid (enable the **Field grid** display), which shows the whole field around the
  robot.

## Obstacles

`demo_obstacles` adds unmapped obstacles that only the simulated laser sees, among them a 0.4 m
box 2 m ahead of the start pose. The demo overlays set `obstacle_mode: contour` (except
Ackermann). To make the robot meet the box on its plan rather than plan around it, keep the box
out of the global costmap with a small overlay of your own, for example `no_global_obstacles.yaml`:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "inflation_layer"]
```

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py \
  scenarios:=demo_obstacles,$PWD/no_global_obstacles.yaml
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

The robot passes the box on its free side at about the contour distance $\lambda$, rejoins the
path and stops at the goal. While it circulates, the status reads `CONTOUR +1` or `CONTOUR -1`,
and the `contour` markers show the path samples that opened the gate, the obstacle cells, the
$\lambda$, $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ rings, the chosen side and the watchdog
progress. The obstacles are run-time parameters of the simulator:

```bash
ros2 param set /vectorfield_sim box_obstacles "[2.0, 0.0, 0.4, 0.4, 0.0]"
```

With `ros2 param set /controller_server FollowPath.obstacle_mode none` the robot stops short of
the box instead and the status reads `BLOCKED`. The controller then raises `NoValidControl`, and
Nav2's behaviour tree replans or recovers; with the stock global obstacle layer, the new plan goes
around the box. {doc}`obstacle_avoidance` explains both modes.

## Curve follower

`curve_follower_demo` adds the `follow_curve` action server to the kinematic demo; Nav2 keeps
running, and its local costmap is the follower's obstacle input:

```bash
ros2 launch vectorfield_bringup curve_follower_demo.launch.py
ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
```

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

`vf-tune` checks a parameter file and sweeps a parameter in simulation (see {doc}`tuning`).
`vectorfield_core` itself also builds and tests without ROS:

```bash
brew install cmake ninja eigen googletest
cd vectorfield_core && cmake --preset dev && cmake --build --preset dev && ctest --preset dev
```
