^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* SingleGoalActionServer (single_goal_action_server.hpp): the followers' action server, on
  rclcpp_action, with one worker thread for the server's lifetime. Nav2's
  SimpleActionServer, which it replaces, can accept a goal and never run it: a goal that
  arrives while the previous goal's worker thread exits waits as pending, and nothing
  executes it. The methods and their semantics are Nav2's, except is_cancel_requested(),
  which is the running goal's flag (Nav2 reports the pending goal's).
* compat::SimpleActionServer and compat::makeActionServer() are removed.

0.5.1 (2026-10-29)
------------------
* clock_source.hpp: runClockSource(), the main() of a simulator that publishes /clock. After
  SIGINT or SIGTERM a node that publishes /clock keeps running while another node subscribes to
  it (at most 4 s), so nodes whose shutdown waits on simulated time finish it. The executor
  spins as rclcpp::spin does; a timer on the node watches for the signal.

0.5.0 (2026-10-25)
------------------
* New parameter obstacle_gate_release_margin (0.10 m): the contour gate's hysteresis.

0.4.1 (2026-10-25)
------------------

0.4.0 (2026-10-24)
------------------
* viridis() is public (marker_builder.hpp): the field grid's colour by distance, shared with the
  3D follower's field arrows.

0.3.0 (2026-10-23)
------------------
* The obstacle_mode description covers cbf for Ackermann (rear axle, turning margin).
* Parameter max_reverse_vel (default -1: max_linear_vel), the speed bound while backing up.
* The status text puts the phase, the command, D and sigma, and the flags on their own lines,
  lifted above the robot, and every text marker sets its space width (scale.x): RViz 2 drew each
  space about 0.6 m wide.
* No white markers: the resulting field is slate, the goal and curve_info are purple, readable on
  RViz's white map and on a dark background.

0.2.0 (2026-10-20)
------------------
* Parameters cusp_tolerance, cbf_gain, cbf_margin and cbf_sites; CBF markers; REV in the status
  text and the active cusp as the goal marker.

0.1.1 (2026-10-20)
------------------
* Parameters skid_yaw_rate_efficiency and skid_icr_offset.

0.1.0 (2026-10-19)
------------------
* First release: the distro compatibility layer (Humble to Rolling), message conversions, the
  generated controller and follower parameters with their binding tables, cross-parameter
  validation and transactional rebuilds.
* RViz visualization: MarkerBuilder and FieldVisualizer with twelve marker namespaces, the field
  grid, predicted path, window, field state and obstacle-distance topics.
* Fix a lost executor cancel at cleanup in the action-server and TF-listener helpers.
