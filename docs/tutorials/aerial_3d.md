# Following curves in 3D (experimental)

The `follow_curve_3d` action server (`vectorfield_follower_3d`) drives a robot that takes a 3D
velocity setpoint, such as a multirotor or an underwater vehicle, along curves in space. These
can be closed curves lap after lap, open curves to their end, curves given by equations, and
curves in a moving frame, all with obstacle avoidance. This tutorial runs the simulated demo,
sends each kind of curve and adds an obstacle. It ends with the settings for a real autopilot.
{doc}`../follower_3d` is the reference.

:::{warning}
The 3D follower is experimental in 0.x. Fly it first in simulation and then with a safety
pilot who can take over.
:::

## 1. Start the demo

```bash
ros2 launch vectorfield_bringup follower3d_demo.launch.py
```

It starts three things: `point_sim_3d` (a point robot at 1 m height that integrates
`/cmd_vel`), the `follower_3d` node under `lifecycle_manager_follower_3d`, and RViz with an
orbit view. There is no map and no Nav2. The follower is ready when

```bash
ros2 lifecycle get /follower_3d
```

prints `active [3]`. In RViz, the *3D follower* group shows the curve (blue), the closest point
and tangent (orange), the command (green), arrows of the field around the robot, and a status
text. The *Trajectory* display trails the robot's odometry.

## 2. An open helix

```bash
ros2 run vectorfield_bringup send_curve3d.py --shape helix
```

The helix (radius 1 m, two turns rising 0.5 m each) is open. The robot flies straight to it
along the field, follows it at `desired_speed` (0.5 m/s), slows down in FINAL_APPROACH and
settles at its end. The script prints the feedback every two seconds, then the result
(`SUCCEEDED: NONE`, with the distance travelled).

`--reverse` flies it from the end to the start. `--speed 0.8` overrides the speed. The vertical
speed is capped by `max_axis_speed_z` (0.5 m/s), and the command is scaled down uniformly to
respect it, so the robot stays on the curve.

## 3. Closed curves and laps

```bash
ros2 run vectorfield_bringup send_curve3d.py --shape viviani --laps 2
ros2 run vectorfield_bringup send_curve3d.py --shape torus_knot --laps 1 --speed 0.6
ros2 run vectorfield_bringup send_curve3d.py --shape lemniscate3d --laps 0     # Ctrl-C to stop
```

Viviani's curve (a sphere cut by a cylinder) crosses itself. The closest-point search keeps
the branch the robot is on, so the lap count is not fooled. With `--laps 0` the goal runs until
you cancel it with Ctrl-C. The robot then decelerates at `cancel_decel`, sends one zero and
falls silent.

## 4. Curves from expressions

```bash
ros2 run vectorfield_bringup send_curve3d.py --shape expression --laps 0
ros2 run vectorfield_bringup send_curve3d.py --shape expression --laps 1 \
  --x '1.5*cos(s)' --y 'sin(2*s)' --z '1 + 0.3*cos(3*s)'
```

The default expression is a circle whose height breathes with time (`t`). The follower adds the
curve's own velocity as a feedforward. An open curve must not depend on `t` (it is
UNSUPPORTED).

## 5. Obstacles

The demo runs the CBF filter against the simulator's point cloud: the surfaces of the spheres
in `point_sim_3d`'s `obstacle_spheres` (none at start). The follower voxelises the cloud at
0.1 m. Put a sphere on the next curve:

```bash
ros2 param set /point_sim_3d obstacle_spheres "[1.1, 1.05, 1.3, 0.25]"
ros2 run vectorfield_bringup send_curve3d.py --shape lemniscate3d --laps 1
```

The voxels appear in red. The robot (a sphere of radius 0.2 m, drawn in green) slows down near
the sphere, slides around it and rejoins the curve. A sphere centred on an open curve stops the
robot instead: the status reads BLOCKED, and after `blocked_timeout` (10 s) the goal aborts
BLOCKED. Static primitives work without a cloud:

```bash
ros2 param set /follower_3d obstacles "['sphere 1.0 0.0 1.2 0.3', 'plane 0 0 0.2 0 0 1']"
```

The `plane` entry is a floor at 0.2 m. Clear the cloud with `obstacle_spheres "[]"`.

## 6. A moving frame

```bash
ros2 launch vectorfield_bringup follower3d_demo.launch.py moving_frame:=platform
ros2 run vectorfield_bringup send_curve3d.py --shape moving --laps 0
```

The simulator moves the frame `platform` at 0.1 m/s while it turns at 0.1 rad/s, and publishes
its Odometry on `platform/odom`. The goal follows a circle in that frame. The follower
estimates the frame's twist and adds the frame velocity at the robot. `--tf` uses TF
differencing against `moving_frame_reference` (`map` in the demo) instead of the Odometry.

## 7. On a real vehicle

Start the follower alone with your parameters, then configure and activate it, either with a
lifecycle manager or with `ros2 lifecycle set`:

```bash
ros2 run vectorfield_follower_3d follower_3d --ros-args --params-file my_drone.yaml \
  -r odom:=/mavros/local_position/odom -r cmd_vel:=/mavros/setpoint_velocity/cmd_vel
```

```yaml
follower_3d:
  ros__parameters:
    global_frame: map           # the frame of the odometry's pose
    cmd_frame: global           # MAVROS setpoint_velocity/cmd_vel takes ENU
    desired_speed: 0.5
    max_speed: 1.0
    max_axis_speed_z: 0.4
    max_accel: 1.0
    max_decel: 1.0
    yaw_policy: path_tangent
    obstacle_mode: cbf
    body_radius: 0.4            # the vehicle plus the propeller guards
    obstacles: ['plane 0 0 0.3 0 0 1']   # a virtual floor at 0.3 m
```

Arm, take off and switch to offboard or guided mode outside the follower. The follower
publishes only while a goal runs, so your supervisor must keep a setpoint stream (a position
hold) between goals, or the autopilot's offboard failsafe will trigger. For PX4 over
uXRCE-DDS or ArduPilot's DDS interface, see {doc}`../follower_3d`, "Inputs, output and
autopilots".

## 8. Your own client

The goal is a `vectorfield_msgs/action/FollowCurve3`. In Python, with the curves module of
`vectorfield_bringup`:

```python
from vectorfield_bringup import curves3d
from vectorfield_msgs.action import FollowCurve3

goal = FollowCurve3.Goal()
goal.curve = curves3d.curve3_msg(
    curves3d.builtin('torus_knot', (0.0, 0.0, 1.5), p=3.0, q=2.0), 'map')
goal.laps = 2
goal.desired_speed = 0.6
# ActionClient(node, FollowCurve3, 'follow_curve_3d').send_goal_async(goal)
```

The result codes and the feedback are those of FollowCurve
({doc}`../follower_3d`, "Safety contract and failures").
