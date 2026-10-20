^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
