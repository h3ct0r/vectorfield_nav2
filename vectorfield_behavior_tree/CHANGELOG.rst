^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_behavior_tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* A goal that gets no result fails with GOAL_REJECTED, SEND_GOAL_FAILURE or TIMEOUT in
  error_code_id: Lyrical's BtActionNode sets them, and the node does on Humble and Jazzy.

0.2.0 (2026-10-20)
------------------
* New package: the Nav2 behaviour-tree node ``FollowCurve`` (plugin library
  ``vectorfield_follow_curve_action_bt_node``), a client of the curve follower's ``follow_curve``
  action, with a ``curve`` port for a blackboard ``vectorfield_msgs/Curve``, convenience ports
  for a built-in shape and the goal options, and ``error_code_id``, ``error_msg``,
  ``laps_completed`` and ``distance_travelled`` outputs. A changed curve preempts the running
  goal; halting the node cancels it.
* ``bt_runner``, a standalone executor that ticks a behaviour tree from a file and exits with its
  status.
* Tests of the node in trees built from XML against a fake ``follow_curve`` server.
