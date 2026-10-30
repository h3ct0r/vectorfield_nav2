^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-10-30)
------------------
* loopback_demo starts the simulator side of nav2_bringup's TB3 loopback wrapper itself
  (robot_state_publisher with the TB3 URDF, Nav2's loopback simulator) and Nav2 through
  nav2_demo_actions(), as kinematic_sim_demo does: its container preloads the lifecycle
  manager (no crash at exit), Lyrical's lifecycle_manager_navigation runs without bonds,
  and it takes the use_composition argument. loopback.yaml no longer sets
  lifecycle_manager_nav2.
* loopback_demo starts the loopback simulator once map_server serves the map
  (wait_for_map): Jazzy's simulator fetches the map once, and crashed at its first scan on
  the empty map of a map_server that was not active yet.
* The loopback and Gazebo demos stop the simulator after Nav2 (shutdown_last()): Ctrl+C no
  longer leaves Nav2 waiting on a stopped clock until launch kills it. The Gazebo demos
  preload the lifecycle manager into nav2_bringup's container (LD_PRELOAD for the included
  launch file) and apply the new gazebo scenario overlay (1 s default_server_timeout,
  Lyrical's lifecycle_manager_nav2 without bonds).

0.5.1 (2026-10-29)
------------------
* The kinematic-sim and loopback demos give bt_navigator a 1 s default_server_timeout instead
  of Nav2's 20 ms, so a loaded machine no longer aborts a goal before the planner accepts it.
* The demos' component containers preload Nav2's lifecycle manager library. The manager leaves
  a pre-shutdown callback in the global context, which the context destroyed at exit after the
  container had unloaded the library: the curve follower's container ended every run with a
  segmentation fault (Jazzy and later; on Humble the container hung before it got there).
* On Humble the demos' containers are this package's isolated_container: Humble's
  component_container_isolated waits forever after a shutdown signal for its executors to spin
  again (rclcpp_components 16.0), and launch had to kill every container after 15 s. It is
  built where rclcpp_components is older than 17, not by ROS_DISTRO, which a package build does
  not always set.
* Lyrical: the demos' lifecycle_manager_navigation runs without bonds. At a shutdown signal the
  manager reset the servers whose bonds had just broken, and the cleanup of bt_navigator or a
  costmap deadlocked with the context's shutdown, so launch killed Nav2's container. The
  loopback overlay does the same for nav2_bringup's lifecycle_manager_nav2 (Lyrical's name).

0.5.0 (2026-10-25)
------------------

0.4.1 (2026-10-25)
------------------
* params.merge() also takes already loaded dictionaries as overlays.

0.4.0 (2026-10-24)
------------------
* rviz/vectorfield.rviz and foxglove/vectorfield.json show the field grid by default and orbit
  the robot (RViz Orbit view on base_link; a perspective Foxglove camera following base_link).
* send_curve.py waits, after the approach, until NavigateToPose's status topic shows the goal
  done: its result arrives first, and the follower's Nav2 interlock read the still-active status
  and refused the curve (NAV2_ACTIVE).
* vectorfield_bringup.demo: nav2_demo_actions() (map_server and its manager, navigation_launch.py,
  Humble's collision_monitor, the initial pose and RViz around a simulator's own actions),
  curve_follower_actions() (the follower and its manager), sim_cmd_vel_topic() and the
  lifecycle helpers, moved out of kinematic_sim_demo and curve_follower_demo, which call them
  unchanged; vectorfield_mujoco's examples share them.
* vectorfield_bringup.demo.follower3d_actions(): the 3D follower, its lifecycle manager and RViz
  with follower_3d.rviz, moved out of follower3d_demo (unchanged) and shared with
  vectorfield_mujoco's drone; rviz_node() takes the RViz config.

0.3.0 (2026-10-23)
------------------
* follower3d_demo.launch.py (experimental): point_sim_3d, the 3D follower with its lifecycle
  manager and RViz with the new rviz/follower_3d.rviz; scripts/send_curve3d.py sends the 3D
  examples of the new vectorfield_bringup.curves3d module (helix, viviani, torus_knot,
  lemniscate3d, a time-varying expression and a circle in a moving frame).
* The reversing scenario overlay sets max_reverse_vel: 0.2; scenarios may set max_reverse_vel
  (at most the model's max_linear_vel).
* identify_skid_slip.py joins its spin thread before shutting down: a thread still in rclpy at
  exit aborted the process (std::terminate) after a good result, which failed
  test_skid_slip_kinematic on Jazzy.
* Field-test kit (docs/tutorials/field_testing.md): scripts/record_field_test.py records the
  topics the metrics need into a timestamped bag with the node parameters;
  scripts/field_test_report.py writes a Markdown report, metrics.json and plots per goal from a
  sqlite3 or mcap bag; scripts/identify_skid_slip.py identifies skid_yaw_rate_efficiency and
  skid_icr_offset online (guarded) or from a bag. New modules field_metrics (the metrics shared
  with vectorfield_system_tests), slip, bag and field_report, with unit tests.
* navigate_waypoints.py: --relative (poses in the robot's start frame) and --through (one
  NavigateThroughPoses goal).
* The laser scan (RViz, Foxglove) and PlotJuggler's phase curve are no longer white.
* config/examples: annotated nav2_params templates (differential, omnidirectional, Ackermann) and
  the rotation-shim overlays for both key layouts, validated like the controller overlays.
* examples/follow_curve_client.py and examples/navigate_waypoints.py, the tutorials' clients.
* send_curve.py sends laps 0 for open curves (laps > 0 made every open curve INVALID_CURVE).

0.2.0 (2026-10-20)
------------------
* Example behaviour trees follow_curve.xml and follow_curve_patrol.xml, the follow_curve_bt and
  reversing scenario overlays, and bt_runner.
* The Ackermann overlay and template enable contour with a 4 x 4 m local costmap; params.merge()
  merges a FollowPath without a plugin key key by key.

0.1.1 (2026-10-20)
------------------
* The kinematic-sim demos enable skid slip compensation with vectorfield_sim's own slip parameters;
  the shipped skid overlay stays neutral.
* kinematic_sim_demo composes collision_monitor and its lifecycle manager in nav2_container (a
  dropped change_state reply could stall Humble's manager); the curve follower and its manager
  share a container.
* TB4 Gazebo demo: the initial pose is offset into the depot map frame.

0.1.0 (2026-10-19)
------------------
* First release: the kinematic-sim, loopback, TB3/TB4 Gazebo and curve-follower demos; maps
  open_10x10, corridor_door and corridor_corner; controller overlays per motion model and scenario
  overlays; params.merge().
* Scripts send_curve.py, moving_frame_publisher.py and initial_pose_publisher; RViz, Foxglove and
  PlotJuggler configurations.
* Ready-to-use Nav2 parameter templates for differential, omnidirectional and Ackermann robots,
  rotation-shim overlays, and the example scripts follow_curve_client.py and
  navigate_waypoints.py.
