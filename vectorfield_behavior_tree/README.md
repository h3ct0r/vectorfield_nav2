# vectorfield_behavior_tree

`FollowCurve`, a Nav2 behaviour-tree action node that sends `vectorfield_msgs/action/FollowCurve`
goals to the curve follower ([`vectorfield_curve_follower`](../vectorfield_curve_follower/README.md)),
and `bt_runner`, a minimal executor that ticks a behaviour tree from a file. The full guide is
[docs/behavior_tree.md](../docs/behavior_tree.md); the design and the per-distro differences are
in [docs/design/m7_follow_curve_bt.md](../docs/design/m7_follow_curve_bt.md).

**Status:** M7. Tested in trees against a fake `follow_curve` server, and end to end on the
kinematic simulator with the real follower, in `bt_runner` and in bt_navigator
(`test_follow_curve_bt` in `vectorfield_system_tests`).

## The node

- Plugin library: `vectorfield_follow_curve_action_bt_node` (`BT_REGISTER_NODES`).
- XML tag: `FollowCurve`; class `vectorfield_behavior_tree::FollowCurveAction`, a
  `nav2_behavior_tree::BtActionNode<vectorfield_msgs::action::FollowCurve>`.
- Action: `follow_curve` (port `server_name`).

```xml
<FollowCurve shape="ellipse" parameters="a=2.0;b=1.0" laps="1"
             error_code_id="{follow_curve_error_code}" error_msg="{follow_curve_error_msg}"/>
<FollowCurve curve="{curve}" laps="0" desired_speed="0.3"/>
```

Inputs: `curve` (a blackboard `vectorfield_msgs/msg/Curve`) or `shape` with `parameters`
(`name=value;...`) and `frame_id`; `laps` (default 1; 0 until halted), `desired_speed`,
`reverse`, `frame_motion` (`static` or `moving`), `frame_odom_topic`, `max_duration` (s), and
Nav2's `server_name` and `server_timeout`. Outputs: `error_code_id`, `error_msg`,
`laps_completed`, `distance_travelled`.

SUCCESS when the follower succeeds, FAILURE when it aborts (with its error code), SUCCESS when
the goal is canceled by the server. Halting the node cancels the goal; changed ports while the
goal runs send a new goal, which the follower takes over. Ports that make no goal fail the node
with `INVALID_GOAL` without sending anything.

## Using it

- **bt_navigator:** add `vectorfield_follow_curve_action_bt_node` to `plugin_lib_names` (on
  Humble, to the full stock list), point `default_nav_to_pose_bt_xml` at your tree, and take
  `navigate_to_pose` and `navigate_through_poses` out of the follower's `exclusive_actions`.
  `vectorfield_bringup`'s `follow_curve_bt` scenario overlay does all three for the demo:

  ```bash
  ros2 launch vectorfield_bringup curve_follower_demo.launch.py scenarios:=follow_curve_bt
  ```

- **bt_runner:**

  ```bash
  ros2 run vectorfield_behavior_tree bt_runner --ros-args -p use_sim_time:=true \
    -p tree_file:=$(ros2 pkg prefix --share vectorfield_bringup)/behavior_trees/follow_curve.xml \
    -p "goal_pose:=[2.0, 0.0, 1.5708]"
  ```

  Parameters: `tree_file`, `plugin_lib_names`, `goal_pose` (`[x, y, yaw]`, the blackboard
  `goal`), `goal_frame`, `bt_loop_duration`, `default_server_timeout`, `default_cancel_timeout`,
  `wait_for_service_timeout`, `dry_run`, `log_transitions`. Exit status 0 (SUCCESS), 1 (FAILURE)
  or 2 (error or signal; the tree is halted first).

Example trees: `vectorfield_bringup/behavior_trees/follow_curve.xml` (approach with Nav2, then
one lap of an ellipse) and `follow_curve_patrol.xml` (a patrol with a recovery).

## Distros

Humble (BT.CPP v3), Jazzy and Lyrical (BT.CPP v4). The differences are confined to
`include/vectorfield_behavior_tree/bt_compat.hpp`: the BT.CPP headers and tick call, and the
type of the blackboard `node` (`nav2::LifecycleNode` on Lyrical).
