# The FollowCurve behaviour-tree node

A Nav2 behaviour-tree action node that sends `vectorfield_msgs/action/FollowCurve` goals, so a
tree can approach a curve with Nav2 and then follow it. Facts read from the installed headers of
`ros-<distro>-nav2-behavior-tree` 1.1.20 (Humble), 1.3.13 (Jazzy) and 1.5.2 (Lyrical).

**Package.** `vectorfield_behavior_tree` (new) depends on `vectorfield_msgs`, `nav2_behavior_tree`
and the BT.CPP that it uses, and on `vectorfield_ros` at build time only, for the CMake helpers. It
does not depend on the follower: the action is the interface. `vectorfield_bringup` exec-depends on
it, `vectorfield_system_tests` test-depends on it, and the `vectorfield_nav2` metapackage lists it.

**Node.** XML tag `FollowCurve`, class `vectorfield_behavior_tree::FollowCurveAction`
(`nav2_behavior_tree::BtActionNode<FollowCurve>`), action `follow_curve`, plugin library
`vectorfield_follow_curve_action_bt_node` (`BT_REGISTER_NODES`).

| Port | Dir | Type | Default | Goal field |
|---|---|---|---|---|
| `curve` | in | `vectorfield_msgs/msg/Curve` | none | `curve`, as is (usually `{curve}`) |
| `shape` | in | string | none | a `BUILTIN` curve: `builtin_shape` |
| `parameters` | in | string | `""` | `"a=2.0;b=1.0"`: `parameter_names`, `parameter_values` |
| `frame_id` | in | string | `""` | the built-in's `header.frame_id` (`""`: the follower's `global_frame`) |
| `laps` | in | unsigned | 1 | `laps`; 0 runs until the node is halted |
| `desired_speed` | in | double | 0 | `desired_speed` (≤ 0: the follower's parameter) |
| `reverse` | in | bool | false | `reverse_direction` |
| `frame_motion` | in | string | `static` | `frame_motion` (`static` or `moving`) |
| `frame_odom_topic` | in | string | `""` | `frame_odom_topic` |
| `max_duration` | in | double (s) | 0 | `max_duration` (0: unlimited) |
| `server_name`, `server_timeout` | in | | | `BtActionNode`'s basic ports |
| `error_code_id` | out | uint16 | | `error_code` (Nav2's port name for error codes) |
| `error_msg`, `laps_completed`, `distance_travelled` | out | string, uint32, double | | the result fields |

Exactly one of `curve` and `shape` is set. Ports that do not make a goal (neither or both, a
malformed `parameters` or `frame_motion`) fail the node without sending a goal, with
`INVALID_GOAL`. Everything else is validated by the follower, which accepts every goal and ends an
invalid one with its code.

**Behaviour.** The outputs are reset (`NONE`, `""`, 0) when a goal starts and set from every
result. *Succeeded*: SUCCESS. *Aborted*: FAILURE with the result's code; an abort with `NONE` (the
follower's `SimpleActionServer` aborts a goal preempted by another client with an empty result) is
reported as `UNKNOWN`. *Canceled* by the server or another client: SUCCESS, Nav2's convention.
*Halt* (a parent halts the node): the base class cancels the goal and waits for its result.
*Preemption*: while the goal runs, every tick rebuilds the goal from the ports; when it differs
(a new `{curve}`, say), the node sends it and the follower preempts the running goal. *Goal not
acknowledged within `server_timeout`, rejected, or not sent*: FAILURE with `TIMEOUT`,
`GOAL_REJECTED` or `SEND_GOAL_FAILURE` (the last two added to FollowCurve.action with Nav2's
values 1 and 2). Lyrical's base class sets them in `on_timeout()`, `on_goal_rejected()` and
`on_send_goal_failure()`, and warns at construction when an action lacks them; Humble and Jazzy
offer no hook, so the node's `tick()` sets them after a FAILURE without a result.

**Distro differences.** The only `#if`s are in `bt_compat.hpp`.

| | Humble | Jazzy | Lyrical |
|---|---|---|---|
| BT.CPP | v3 (`behaviortree_cpp_v3/`), `Tree::tickRoot()` | v4 (`behaviortree_cpp/`), `Tree::tickOnce()` | v4 |
| Blackboard `node` | `rclcpp::Node` | `rclcpp::Node` | `nav2::LifecycleNode` |
| Timeouts read | `server_timeout`, `bt_loop_duration`, `wait_for_service_timeout` | also `cancel_timeout` | as Jazzy |
| Halt waits for the result | `server_timeout` | `cancel_timeout` | `cancel_timeout` |
| Basic ports | `server_name`, `server_timeout` | as Humble | also `error_code_id`, `error_msg` |
| bt_navigator custom nodes | `plugin_lib_names` is the full list | `plugin_lib_names` lists custom nodes only | as Jazzy |
| Error codes in the navigator result | none | key in `error_code_names` | prefix in `error_code_name_prefixes` |

The BT.CPP version comes from CMake (the BT.CPP package that `nav2_behavior_tree` pulls in), not
`__has_include`, because a Humble machine can have both installed. `bt_compat.hpp` also
includes Nav2's `BT::convertFromString` specializations: without them, `bt_runner`'s blackboard
types instantiate BT.CPP's throwing generic under the same symbol, which then replaces Nav2's in
the plugins (a literal `server_timeout` stops parsing). Lyrical's `BtActionNode` warns
once per node that FollowCurve defines no `TIMEOUT`, `GOAL_REJECTED` or `SEND_GOAL_FAILURE` code;
the interface stays frozen. BT.CPP v3 ignores `BTCPP_format`, so one set of example trees, written
with `BTCPP_format="4"` and only the nodes and ports common to the three distros, serves all.

**Interlock.** Inside bt_navigator, `navigate_to_pose` stays active for the whole tree, so the
follower would refuse the goal with `NAV2_ACTIVE`. The `follow_curve_bt` scenario overlay removes
`navigate_to_pose` and `navigate_through_poses` from the follower's `exclusive_actions`; the tree
itself sequences FollowPath and FollowCurve so that they never overlap.

**Where it runs.** (1) bt_navigator: the `follow_curve_bt` overlay sets
`default_nav_to_pose_bt_xml` to bringup's `behavior_trees/follow_curve.xml` and lists the plugin
(`params.merge()` appends `plugin_lib_names` to the stock list); a NavigateToPose goal is the
approach pose. (2) `bt_runner`, a standalone executor in the package: it loads the plugins, puts
`node`, the timeouts and an optional `goal` pose on the blackboard, ticks the tree from a file at
`bt_loop_duration` and exits 0 on SUCCESS, 1 on FAILURE and 2 on an error or a signal (after
halting the tree). The end-to-end test uses both, on vectorfield_sim with the real follower.
