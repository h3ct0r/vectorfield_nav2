^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* config/examples: annotated nav2_params templates (differential, omnidirectional, Ackermann) and
  the rotation-shim overlays for both key layouts, validated like the controller overlays.
* examples/follow_curve_client.py and examples/navigate_waypoints.py, the tutorials' clients.
* send_curve.py sends laps 0 for open curves (laps > 0 made every open curve INVALID_CURVE).

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
