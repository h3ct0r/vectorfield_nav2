# Getting started

This tutorial goes from a machine with ROS 2 to a simulated robot that drives to goals with the
vector-field controller, and explains what to look at while it does. It takes about half an
hour, most of it the first build. The {doc}`../quickstart` is the condensed version of the same
steps.

:::{admonition} Build from source
:class: note

0.5.1 is released, but its binary packages (`ros-<distro>-vectorfield-nav2`) wait on the ROS
build farm ({doc}`../distro_matrix`). Until they are out, build the packages from source as
below.
:::

## 1. Prerequisites

| Distro | Ubuntu | ROS 2 installation guide |
|---|---|---|
| Humble | 22.04 (jammy) | <https://docs.ros.org/en/humble/Installation.html> |
| Jazzy | 24.04 (noble) | <https://docs.ros.org/en/jazzy/Installation.html> |
| Lyrical | 26.04 (resolute) | <https://docs.ros.org/en/lyrical/Installation.html> |

Install ROS 2 from the Debian packages of your distro (`ros-<distro>-desktop` includes RViz;
`ros-<distro>-ros-base` is enough for headless use). Everything else, Nav2 and the simulators
included, is installed by rosdep in the next step. You also need git, colcon and rosdep:

```bash
sudo apt update
```

```bash
sudo apt install -y git python3-colcon-common-extensions python3-rosdep
```

If rosdep has never been initialised on this machine:

```bash
sudo rosdep init
```

```bash
rosdep update
```

The Gazebo TurtleBot demos and Nav2's loopback simulator need Jazzy or later; everything else in
these tutorials runs on all three distros. On Humble rosdep skips the Gazebo and loopback
dependencies.

## 2. Build the workspace

Source your ROS 2 installation. Replace `jazzy` with `humble` or `lyrical`:

```bash
source /opt/ros/jazzy/setup.bash
```

Create a workspace and clone the repository into it:

```bash
mkdir -p ~/vf_ws/src
```

```bash
cd ~/vf_ws/src && git clone https://github.com/h3ct0r/vectorfield_nav2.git
```

Install the dependencies (Nav2, the simulators, RViz, pybind11, the Python tools' dependencies
and MuJoCo for the {doc}`MuJoCo examples <../examples/index>`):

```bash
cd ~/vf_ws && PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths src --ignore-src -y
```

MuJoCo comes from pip. `PIP_BREAK_SYSTEM_PACKAGES=1` lets rosdep's pip install it on Ubuntu
Noble and later, whose system Python refuses pip installs otherwise; Humble's Jammy does not
need it. The {doc}`../quickstart` shows how to leave the MuJoCo examples out instead.

Build. The first build compiles the C++ core, the controller plugin, the curve follower and the
Python bindings, and takes several minutes:

```bash
cd ~/vf_ws && colcon build --symlink-install
```

Source the workspace in every new terminal:

```bash
source ~/vf_ws/install/setup.bash
```

Check that ROS finds the plugin package:

```bash
ros2 pkg prefix vectorfield_nav2_controller
```

It prints `~/vf_ws/install/vectorfield_nav2_controller` (expanded). To run the tests as well (the
system tests start the demos and take about 15 minutes):

```bash
cd ~/vf_ws && colcon test --packages-select vectorfield_bringup vectorfield_system_tests && colcon test-result --verbose
```

### Alternative: the Docker devcontainers

The repository has one devcontainer per distro under `.devcontainer/` (`humble`, `jazzy`,
`lyrical`; `rolling` compiles and runs the unit tests only). They need Docker and either VS Code
with the Dev Containers extension or the devcontainer CLI, and run natively on Apple Silicon:

1. Open the repository folder in VS Code and run **Dev Containers: Reopen in Container**, then
   pick a distro. The container runs rosdep for you.
2. In the container's terminal, from `/ws`:

   ```bash
   colcon build --symlink-install
   ```

   ```bash
   source install/setup.bash
   ```

3. For RViz, start the noVNC desktop described in `.devcontainer/README.md` and
   `export DISPLAY=:1` before launching, or launch with `use_rviz:=false` and connect Foxglove to
   `foxglove_bridge` ({doc}`../visualization`).

The workspace lives in `/ws` (sources in `/ws/src/vectorfield_nav2`), and `build`, `install` and
`log` are Docker volumes named after the distro, so they survive container rebuilds.

## 3. Run the first demo

```bash
ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py
```

This starts:

- `vectorfield_sim`, a kinematic simulator of a TurtleBot-sized differential base with a laser,
  which is also the clock (`use_sim_time`);
- map_server with `open_10x10`, a 10 m × 10 m open area centred on the origin;
- Nav2's `navigation_launch.py` with the stock parameters of your distro, merged with the
  differential overlay that makes `FollowPath` the vector-field controller;
- on Humble only, a collision_monitor between Nav2 and the simulator (`cmd_vel` → `cmd_vel_safe`);
- RViz with `rviz/vectorfield.rviz`.

Nav2 is ready when the log prints `Managed nodes are active` for `lifecycle_manager_navigation`,
and the controller logs
`FollowPath: vector-field controller configured (…, differential, feedback_linearization)`. If
configuration fails, the controller's validation message is in the launch log instead (see
{doc}`your_robot`). Without a display, add `use_rviz:=false` and check from a second terminal:

```bash
ros2 lifecycle get /bt_navigator
```

It prints `active [3]` once Nav2 accepts goals; bt_navigator is activated after
controller_server, and a goal sent before that is rejected.

## 4. Set the initial pose and send goals in RViz

The robot starts at (0, 0) facing +x. The demo publishes that start pose itself, so you do not
have to set it. To move the robot elsewhere, use RViz's **2D Pose Estimate** tool: click and drag
on the map. In the kinematic simulator this teleports the robot (it moves `map → odom`); in the
Gazebo demos, which run AMCL, it initialises the localization instead. The same from a terminal:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: -1.0}, orientation: {w: 1.0}}}}"
```

Send a goal with the **2D Goal Pose** tool: click where the robot should go and drag toward the
heading it should end with. Nav2 plans a path (grey), the controller follows it and stops on the
goal, then turns to the goal heading. From a terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

The command ends with `Goal finished with status: SUCCEEDED`. A route of several poses:

```bash
ros2 run vectorfield_bringup navigate_waypoints.py
```

`navigate_waypoints.py` (installed from `vectorfield_bringup/examples/`) sends one
NavigateToPose goal per pose with Nav2's simple commander, prints the distance remaining, and
exits with 0 once every pose is reached; `--poses "x,y,yaw x,y,yaw"` sets your own route.

Goals that show the controller's phases:

| Goal | What happens |
|---|---|
| Behind the robot, for example (−3, 0) | The robot first turns in place (ROTATE_TO_FIELD, amber status) until the field is within 45° of its heading, then follows the plan |
| At the robot's position with a new heading | It only turns in place (ROTATE_TO_HEADING, blue) |
| A few metres ahead | It converges to the plan (FOLLOW, green), brakes over the last 0.5 m (FINAL_APPROACH, cyan) and stops on the goal |

## 5. What the markers mean

The vector-field markers are drawn at the robot and along the plan:

- the **status text** above the robot: the phase, coloured as in the table above (STOPPED is
  red), and flags such as `BLOCKED` or `CONTOUR +1`;
- the **field arrows** at the control point, 0.2 m ahead of the robot: convergence toward the
  path (red), circulation along it (green) and their sum, the field the robot tracks (slate);
- the **command** (cyan arrow) and the **predicted path** (green up to the first predicted
  collision);
- the **goal rings** (the XY tolerance and the settle radius) and the orange **braking zone**;
- the **field grid** (the *Field grid* display, on by default): the field around the robot.

{doc}`../visualization` has the full legend, a clip of every phase and the Foxglove and
PlotJuggler layouts.

## 6. Stop and clean up

Stop the demo with Ctrl-C in its terminal; it stops every node it started. If a later launch
reports nodes that are already running, or `ros2 node list` still shows demo nodes, stop the ROS
daemon so the CLI forgets stale entries:

```bash
ros2 daemon stop
```

To rebuild from scratch, delete the workspace's `build`, `install` and `log` directories, then
build again:

```bash
cd ~/vf_ws && rm -rf build install log
```

## Next steps

- {doc}`../examples/index`: the controller on physics models of real robots, one command per
  mode.
- {doc}`scenarios`: the other demos (corridors, obstacles, other motion models, speed limits,
  run-time tuning).
- {doc}`curve_following`: the `follow_curve` action, from the demo to your own client.
- {doc}`your_robot`: put the controller into your robot's Nav2 configuration.
- {doc}`offline_design`: check a configuration with the Python tools before deploying.
