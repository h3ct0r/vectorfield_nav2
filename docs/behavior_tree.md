# Behaviour trees

`vectorfield_behavior_tree` adds `FollowCurve`, a Nav2 behaviour-tree action node that sends
goals to the curve follower's `follow_curve` action ({doc}`curve_follower`). A tree can drive to
a curve with Nav2's planner and controller, follow the curve, and recover when the follower
gives up, all inside `bt_navigator`. The package also ships `bt_runner`, a small executor that
ticks a tree from a file without `bt_navigator`. {doc}`design/follow_curve_bt` records the
design and the per-distro differences.

## The node

```xml
<FollowCurve shape="ellipse" parameters="a=2.0;b=1.0" laps="1"
             error_code_id="{follow_curve_error_code}" error_msg="{follow_curve_error_msg}"/>
```

The plugin library is `vectorfield_follow_curve_action_bt_node`; the action is `follow_curve`
unless `server_name` says otherwise. Give the curve in exactly one of two ways:

- `curve`: a `vectorfield_msgs/msg/Curve` on the blackboard (`curve="{curve}"`), sent as is. Any
  curve type works: polylines, expressions and built-ins.
- `shape`, `parameters` and `frame_id`: a built-in closed shape ({doc}`curve_follower`), its
  parameters as `name=value` pairs separated by `;` or `,`, and its frame.

| Port | Type | Default | Meaning |
|---|---|---|---|
| `curve` | `vectorfield_msgs/msg/Curve` | none | The curve, usually a blackboard entry |
| `shape` | string | none | A built-in shape: `ellipse`, `lemniscate_bernoulli`, `rounded_rectangle`, ... |
| `parameters` | string | `""` | The built-in's parameters, for example `a=2.0;b=1.0`; unset ones keep their defaults |
| `frame_id` | string | `""` | The built-in's frame; empty is the follower's `global_frame` |
| `laps` | unsigned | 1 | Laps of a closed curve; 0 follows it until the node is halted |
| `desired_speed` | double | 0 | Cruise speed (m/s); ≤ 0 uses the follower's parameter |
| `reverse` | bool | false | Follow the curve against its direction |
| `frame_motion` | string | `static` | `moving` for a curve in a moving frame |
| `frame_odom_topic` | string | `""` | The moving frame's Odometry; empty uses TF differencing |
| `max_duration` | double | 0 | Time limit (s); 0 is unlimited |
| `server_name` | string | `follow_curve` | The action name |
| `server_timeout` | ms | the blackboard's | How long the follower may take to acknowledge a goal |
| `error_code_id` (out) | uint16 | | The result's `error_code` (`NONE` on success) |
| `error_msg` (out) | string | | The result's `error_msg` |
| `laps_completed` (out) | uint32 | | The result's `laps_completed` |
| `distance_travelled` (out) | double | | The result's `distance_travelled` (m) |

| Event | Node status | Outputs |
|---|---|---|
| The follower succeeds | SUCCESS | from the result |
| The follower aborts the goal | FAILURE | from the result; `UNKNOWN` when it carries no code (another client's goal preempted it) |
| The goal is canceled by the follower or another client | SUCCESS (Nav2's convention) | from the result |
| A parent halts the node | the goal is canceled | from the result, if it arrives within `cancel_timeout` (Humble: `server_timeout`) |
| The ports change while the goal runs (a new `{curve}`, say) | the new goal is sent; the follower preempts the old one | |
| Ports that make no goal: neither or both of `curve` and `shape`, malformed `parameters` or `frame_motion`, a negative `max_duration` | FAILURE, no goal sent | `INVALID_GOAL` and the reason |
| The follower rejects the goal (it is not active) | FAILURE | `GOAL_REJECTED` (1) |
| The goal cannot be sent (the client's executor was interrupted) | FAILURE | `SEND_GOAL_FAILURE` (2) |
| The follower does not acknowledge the goal within `server_timeout` | FAILURE; on Lyrical the node also cancels the client's goals | `TIMEOUT` (1006) |

The last three rows use the result codes that Nav2's `BtActionNode` reports when no result
arrives; Lyrical's sets them itself, and on Humble and Jazzy the node does, with the same
messages. The outputs are reset (`NONE`, empty, 0) when a goal starts. Everything else about a goal is
checked by the follower, which ends an invalid goal with its code
({doc}`tutorials/curve_following`, "Feedback and result codes"). Use the codes in the tree, for
example to retry only on `BLOCKED` (1005).

## Example trees

`vectorfield_bringup/behavior_trees/` holds two trees. Both work unchanged on Humble, Jazzy and
Lyrical: BT.CPP v3 ignores `BTCPP_format`, and the trees use only nodes and ports the three
distros share.

- `follow_curve.xml` approaches `{goal}` with ComputePathToPose and FollowPath (replanning at
  1 Hz, clearing the costmaps on failure), then follows the 2 m × 1 m ellipse for one lap. The
  ellipse starts at (2, 0) heading +y, so that is the pose to approach.
- `follow_curve_patrol.xml` approaches `{goal}` the same way, then follows a 4 m × 2 m rounded
  rectangle lap after lap. When the follower aborts (`BLOCKED`, `STALE_INPUT`, ...), the recovery
  clears the costmaps and waits 2 s, and the patrol resumes from wherever the robot is; after 5
  recoveries the tree fails.

In both, FollowCurve starts only after FollowPath has succeeded, so the follower's Nav2
interlock never sees both running. Copy a tree and edit the FollowCurve attributes for your
curve.

## In bt_navigator

The `follow_curve_bt` scenario overlay makes bt_navigator run `follow_curve.xml` for
NavigateToPose in the curve-follower demo:

```bash
ros2 launch vectorfield_bringup curve_follower_demo.launch.py scenarios:=follow_curve_bt
```

Send a NavigateToPose goal to (2, 0) heading +y, with RViz's **2D Goal Pose** or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
```

The robot drives there, follows the ellipse once, and the NavigateToPose goal succeeds. For
your own robot, the same settings go into your Nav2 parameter file:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: /path/to/your/follow_curve.xml
    # Humble: append to the full stock list. Jazzy and later: custom plugins only.
    plugin_lib_names: ["vectorfield_follow_curve_action_bt_node"]
    # The tree is created at activation; the follower may come up later than Nav2.
    wait_for_service_timeout: 10000

curve_follower:
  ros__parameters:
    # bt_navigator's navigate_to_pose stays active while the tree runs.
    exclusive_actions: ["follow_path", "spin", "backup", "drive_on_heading", "assisted_teleop",
      "dock_robot", "undock_robot", "follow_object"]
```

- **Plugins.** Humble's `plugin_lib_names` replaces Nav2's list, so it must keep every stock
  plugin; the bringup's `params.merge()` appends overlay entries to the stock list for you. From
  Jazzy on, the stock plugins are always loaded and the parameter lists only custom ones.
- **Interlock.** Inside bt_navigator, `navigate_to_pose` is active for the whole tree, so a
  follower that watches it refuses every goal with `NAV2_ACTIVE`. Leave the navigator actions
  out of `exclusive_actions` and let the tree keep FollowPath and FollowCurve apart.
- **Goal acknowledgement.** bt_navigator's `default_server_timeout` is 20 ms; the example trees
  give FollowCurve `server_timeout="1000"`.
- **Error codes in the NavigateToPose result.** On Jazzy, add `follow_curve_error_code` to
  `error_code_names`; on Lyrical, add `follow_curve` to `error_code_name_prefixes` (the trees
  write `{follow_curve_error_code}` and `{follow_curve_error_msg}`). Both parameters are full
  lists: keep the stock entries. Humble's result has no error code.

## bt_runner

`bt_runner` loads the plugins, puts on the blackboard what Nav2's nodes read, ticks the tree
every `bt_loop_duration` and exits with the tree's status. It needs Nav2's servers and the
follower, but not bt_navigator; the end-to-end test uses it. With the curve-follower demo
running:

```bash
ros2 run vectorfield_behavior_tree bt_runner --ros-args -p use_sim_time:=true \
  -p tree_file:=$(ros2 pkg prefix --share vectorfield_bringup)/behavior_trees/follow_curve.xml \
  -p "goal_pose:=[2.0, 0.0, 1.5708]"
```

| Parameter | Default | Meaning |
|---|---|---|
| `tree_file` | (required) | The tree's XML file |
| `plugin_lib_names` | FollowCurve and the Nav2 nodes of the example trees | Plugin libraries to load |
| `goal_pose` | `[]` | `[x, y, yaw]` (floats): put `goal`, a PoseStamped in `goal_frame`, on the blackboard |
| `goal_frame` | `map` | The frame of `goal_pose` |
| `bt_loop_duration` | 10 | Tick period (ms) |
| `default_server_timeout` | 1000 | Blackboard `server_timeout` (ms) |
| `default_cancel_timeout` | 1000 | Blackboard `cancel_timeout` (ms) |
| `wait_for_service_timeout` | 5000 | How long each action node waits for its server when the tree is created (ms) |
| `dry_run` | false | Create the tree (load the plugins, connect every node to its server) and exit |
| `log_transitions` | true | Print every node's status changes |

Exit status: 0 when the tree succeeds, 1 when it fails, 2 on an error (a missing plugin or
server, a malformed tree) or on SIGINT and SIGTERM, after halting the tree so that its running
goals are canceled.

## Distro notes

- Humble uses BT.CPP v3 and Jazzy and Lyrical BT.CPP v4; the node, the trees and `bt_runner` are
  the same on all three.
- On Lyrical, Nav2's action node logs once per FollowCurve node that the action defines no
  `TIMEOUT`, `GOAL_REJECTED` or `SEND_GOAL_FAILURE` code; on those events the node reports
  `UNKNOWN`. Humble and Jazzy have no hook for them, and a goal that is not acknowledged within
  `server_timeout` fails the node with the reset outputs.
