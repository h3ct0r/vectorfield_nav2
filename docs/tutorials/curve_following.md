# Following curves

Nav2's controller follows plans to a goal. The `follow_curve` action server
(`vectorfield_curve_follower`) follows curves instead: closed curves lap after lap, open curves to
their end, curves given by equations, and curves attached to a moving frame. It shares the
controller's core and limits, sends its commands through Nav2's velocity smoother and
collision_monitor, and stays out of Nav2's way. This tutorial runs the demo, sends every kind of
curve, and ends with your own action client. {doc}`../curve_follower` is the reference for the
action, the frames, the failures and the parameters.

## 1. Start the demo

```bash
ros2 launch vectorfield_bringup curve_follower_demo.launch.py
```

It is the kinematic demo of {doc}`getting_started` (same arguments: `motion_model`, `map`,
`use_rviz`, ...) plus the `curve_follower` node under its own lifecycle manager. Nav2 keeps
running: its local costmap is the follower's obstacle input. The follower is ready when

```bash
ros2 lifecycle get /curve_follower
```

prints `active [3]`. In RViz, the *Curve follower* group shows the curve, the follower's markers
and its predicted path.

## 2. Closed curves and laps

`send_curve.py` sends one of the example curves. It first drives the robot to the curve with
NavigateToPose (the follower converges from anywhere, but along the shortest way, through
whatever lies between), then sends the goal, prints the feedback every two seconds and exits
with 0 when the goal succeeds:

```bash
ros2 run vectorfield_bringup send_curve.py --shape ellipse --laps 2
```

```bash
ros2 run vectorfield_bringup send_curve.py --shape lemniscate_bernoulli --laps 3
```

The last line of the output is the action status and the result code, `SUCCEEDED: NONE`, followed
by the result message, the laps completed and the distance travelled. The other closed shapes are
`lemniscate_gerono` and `superellipse`. `--laps 0` runs until Ctrl-C, which cancels the goal;
`--speed 0.3` overrides `desired_speed` for this goal; `--reverse` drives the curve the other way.

## 3. Curves from expressions

An `EXPRESSION` curve is a pair of muParser expressions in the parameter `s` (and the time `t`
for time-varying curves). The `expression` example is an ellipse whose semi-axes breathe with
time; `--x` and `--y` replace its expressions, with the constants `a` = 2, `b` = 1, `c` = 0.3 and
`w` = 0.2 defined. A closed figure-eight on the default domain, $s \in [0, 2\pi]$:

```bash
ros2 run vectorfield_bringup send_curve.py --shape expression --x "a*cos(s)" --y "b*sin(2*s)" --laps 1
```

An open parabola, followed to its end:

```bash
ros2 run vectorfield_bringup send_curve.py --shape expression --x "s" --y "0.25*s^2" --s-min -2 --s-max 2 --open
```

Use `^` for powers and `_pi` for π. A curve declared closed must close, and an open curve may
not depend on `t`; the follower rejects either with `INVALID_CURVE` or `UNSUPPORTED` before the
robot moves. `vf-curve --curve 'x(s); y(s)'` checks an expression without ROS
({doc}`offline_design`).

## 4. Open curves and reverse

An open curve ends with the same terminal field as a Nav2 plan: the robot brakes over the last
`terminal_braking_distance` and stops on the end point. The `sine` example is open:

```bash
ros2 run vectorfield_bringup send_curve.py --shape sine --speed 0.2
```

With `--reverse` the robot starts at the curve's last point and ends at its first:

```bash
ros2 run vectorfield_bringup send_curve.py --shape sine --speed 0.2 --reverse
```

`laps` must be 0 for an open curve (`INVALID_CURVE` otherwise); `send_curve.py` ignores `--laps`
for open curves.

## 5. Moving frames

A curve can be attached to a frame that moves. The follower adds the frame's velocity to the
field (the feedforward term of the time-varying field, {doc}`../theory`), so the robot tracks the
curve while it moves. `moving_frame_publisher.py` publishes such a frame, `moving_frame` under
`odom`, with a constant twist (`vx`, `vy`, `wz`, in the frame itself) from the pose `x`, `y`,
`yaw`. The `moving` example is a 1.5 m circle in that frame.

A translating frame: the circle starts centred at (0, 1.5), so it passes through the robot, and
drifts along +x at 0.1 m/s. In one terminal:

```bash
ros2 run vectorfield_bringup moving_frame_publisher.py --ros-args -p use_sim_time:=true -p vx:=0.1 -p y:=1.5
```

In another (Ctrl-C cancels the goal):

```bash
ros2 run vectorfield_bringup send_curve.py --shape moving --laps 0
```

The frame keeps drifting while the publisher runs; in `open_10x10` the circle reaches the map's
edge at x = 5 m after about 35 s, and the robot then stops at the wall (the goal ends `BLOCKED`).
Restart the publisher to bring the frame back.

A rotating frame: with `wz` as well, the frame's origin moves on a circle of radius `vx/wz`
(2 m here), carrying the curve around it. Restart the publisher with:

```bash
ros2 run vectorfield_bringup moving_frame_publisher.py --ros-args -p use_sim_time:=true -p vx:=0.1 -p wz:=0.05 -p y:=1.5
```

The frame's velocity comes from one of two sources:

| Source | Goal field | `send_curve.py` | Needs |
|---|---|---|---|
| Odometry of the frame | `frame_odom_topic: moving_frame/odom` | default | an Odometry whose child frame is the curve frame (the publisher's `<child_frame>/odom`) |
| TF differencing | `frame_odom_topic` empty | `--tf` | the follower's `moving_frame_reference` (the demo sets `odom`); never a chain through `map` → `odom` |

```bash
ros2 run vectorfield_bringup send_curve.py --shape moving --tf --laps 0
```

The twist can be changed while the publisher runs, for example
`ros2 param set /moving_frame_publisher vx 0.2`. The curve must move slower than the robot
(Assumption 3 of the method): the feedforward is clamped, the status shows `A3-CLAMP`, and a
clamp that lasts ends the goal with `ASSUMPTION3_VIOLATED`.

## 6. Your own action client

`follow_curve_client.py` (installed from `vectorfield_bringup/examples/`) is a minimal rclpy
client to start from. It builds a `Curve`, sends the goal, prints the feedback and the result
code, and exits with 0 on success. Its default curve, an ellipse centred at (1.5, 0) with
semi-axes 1.5 m and 1 m, passes through the demo's start pose:

```bash
ros2 run vectorfield_bringup follow_curve_client.py
```

A built-in shape with parameters, or a polyline:

```bash
ros2 run vectorfield_bringup follow_curve_client.py --shape lemniscate_bernoulli --param a=1.5 --param cx=1.5 --laps 2
```

```bash
ros2 run vectorfield_bringup follow_curve_client.py --points "0,0 2,0 2,1.5" --speed 0.3
```

The goal is built from the message definitions:

```{literalinclude} ../../vectorfield_bringup/examples/follow_curve_client.py
:language: python
:pyobject: build_curve
```

```{literalinclude} ../../vectorfield_bringup/examples/follow_curve_client.py
:language: python
:start-at: "    goal = FollowCurve.Goal()"
:end-before: "    def on_feedback"
```

The same in C++ with `rclcpp_action` (link the node against `rclcpp`, `rclcpp_action` and
`vectorfield_msgs`):

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"

using FollowCurve = vectorfield_msgs::action::FollowCurve;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("follow_curve_cpp_client");
  auto client = rclcpp_action::create_client<FollowCurve>(node, "follow_curve");
  if (!client->wait_for_action_server(std::chrono::seconds(30))) {
    RCLCPP_ERROR(node->get_logger(), "no follow_curve action server");
    return 1;
  }

  FollowCurve::Goal goal;
  goal.curve.header.frame_id = "map";
  goal.curve.type = vectorfield_msgs::msg::Curve::BUILTIN;
  goal.curve.builtin_shape = "ellipse";
  goal.curve.parameter_names = {"a", "b", "cx"};
  goal.curve.parameter_values = {1.5, 1.0, 1.5};
  goal.frame_motion = FollowCurve::Goal::FRAME_STATIC;
  goal.laps = 1;

  rclcpp_action::Client<FollowCurve>::SendGoalOptions options;
  options.feedback_callback =
    [node](auto, const std::shared_ptr<const FollowCurve::Feedback> feedback) {
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000, "lap %u, distance %.3f m",
        feedback->lap, feedback->distance_to_curve);
    };
  auto sent = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, sent) != rclcpp::FutureReturnCode::SUCCESS ||
    !sent.get())
  {
    RCLCPP_ERROR(node->get_logger(), "the goal was not accepted");
    return 1;
  }
  auto result = client->async_get_result(sent.get());
  rclcpp::spin_until_future_complete(node, result);
  const auto wrapped = result.get();
  RCLCPP_INFO(
    node->get_logger(), "error_code %u: %s (laps %u)",
    static_cast<unsigned>(wrapped.result->error_code), wrapped.result->error_msg.c_str(),
    static_cast<unsigned>(wrapped.result->laps_completed));
  rclcpp::shutdown();
  return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ? 0 : 1;
}
```

## 7. Feedback and result codes

The feedback carries `distance_to_curve` (m), `arc_length` and `remaining_arc_length` (m), `lap`,
`speed` (m/s) and `phase` (a `FieldState.PHASE_*` value). The result carries `error_code`,
`error_msg`, `laps_completed`, `distance_travelled` and `total_elapsed_time`. A goal is never
rejected at the action level: it is accepted and then validated, so every failure ends with a
code and a message.

| Code | Name | Typical cause |
|---|---|---|
| 0 | `NONE` | success: the laps are done, or the robot settled at the end of an open curve |
| 1001 | `INVALID_CURVE` | a malformed or irregular curve, a closed curve that does not close, `laps` > 0 on an open curve |
| 1002 | `UNSUPPORTED` | an open time-varying curve, a moving goal without a twist source, the interlock cannot see Nav2 |
| 1003 | `TF_ERROR` | the curve frame cannot be transformed |
| 1004 | `STALE_INPUT` | pose, odometry, costmap or moving frame too old for too long |
| 1005 | `BLOCKED` | blocked or no progress for 10 s, or the contour watchdog |
| 1006 | `TIMEOUT` | `max_duration` reached |
| 1007 | `ASSUMPTION3_VIOLATED` | the curve moved almost as fast as the robot for too long |
| 1008 | `NAV2_ACTIVE` | a Nav2 motion action was active at the start or became active |
| 1009 | `INVALID_GOAL` | goal overrides that make the configuration invalid, e.g. `desired_speed` above `max_linear_vel` |
| 1000 | `UNKNOWN` | a non-finite command, deactivation during the goal, a failed parameter rebuild |

A cancelled goal ends with the action status CANCELED. {doc}`../curve_follower` ("Failures")
lists every condition.

## 8. The Nav2 interlock

The follower and Nav2's own motion actions must never command the base at the same time. The
follower watches the status of every Nav2 action that moves the robot (`follow_path`,
`navigate_to_pose`, `spin`, `backup`, ...): a goal does not start while one of them is active, and
a running goal stops with one zero command when one becomes active. To see it, start a curve
with `--laps 0` and, while it runs, send a NavigateToPose goal (RViz's **2D Goal Pose**, or):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

The follower ends its goal with `NAV2_ACTIVE` (1008) and Nav2 drives the robot to the pose.
`send_curve.py` itself uses NavigateToPose for its approach and sends the curve only once that
goal has finished. With `require_nav2_interlock` (the default), a goal is refused with
`UNSUPPORTED` when the follower cannot see Nav2's `follow_path` action, usually because of a
namespace or remapping mismatch.

## 9. Behaviour trees

The `FollowCurve` behaviour-tree node ({doc}`../behavior_tree`) sends these goals from a Nav2
behaviour tree, so a tree can do the approach that `send_curve.py` does and then follow the
curve. With the `follow_curve_bt` overlay, bt_navigator runs `behavior_trees/follow_curve.xml`
for every NavigateToPose goal:

```bash
ros2 launch vectorfield_bringup curve_follower_demo.launch.py scenarios:=follow_curve_bt
```

Send a NavigateToPose goal to (2, 0) heading +y (RViz's **2D Goal Pose**, or):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
```

The tree plans and drives to the pose, then follows the 2 m × 1 m ellipse that starts there for
one lap, and the NavigateToPose goal succeeds. The overlay also takes `navigate_to_pose` out of
the follower's interlock: the navigator's own action stays active while its tree runs the
follower. Without bt_navigator, `bt_runner` ticks the same tree from the file:

```bash
ros2 run vectorfield_behavior_tree bt_runner --ros-args -p use_sim_time:=true \
  -p tree_file:=$(ros2 pkg prefix --share vectorfield_bringup)/behavior_trees/follow_curve.xml \
  -p "goal_pose:=[2.0, 0.0, 1.5708]"
```

It prints each node's status changes and exits with 0 when the tree succeeds.
`follow_curve_patrol.xml` patrols a rounded rectangle until halted and recovers when the
follower aborts.
