^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_nav2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-10-24)
------------------
* MuJoCo examples (vectorfield_mujoco, source-only): one launch per motion model, with physics,
  the controller, RViz and a scenario that drives the robot by itself: differential (PAL TIAGo,
  Nav2 + obstacle contour), skid-steer (AgileX Scout Mini with the AgileX manual's geometry,
  through a door), omnidirectional (Stanford TidyBot base, curve follower), Ackermann (car,
  Smac Hybrid + CBF), drone (Skydio X2, 3D follower + CBF) and curve following (Scout Mini,
  lemniscate laps); clips of each in the README and the docs.
* Documentation site revamped (Furo, light and dark): a landing page with the example gallery
  and an Examples section with one page per robot.
* RViz and Foxglove layouts show the field grid by default and orbit the robot; the 3D
  follower's field arrows have heads and are coloured by distance to the curve.
* send_curve.py no longer has its curve refused right after a successful approach.
* See each package's changelog for the details.

0.3.0 (2026-10-23)
------------------
* Adds vectorfield_follower_3d (experimental): the ``follow_curve_3d`` action for drones and
  other robots that take a 3D velocity setpoint, with moving frames, CBF obstacle avoidance
  (primitives and a voxelised PointCloud2), ``point_sim_3d`` and ``follower3d_demo.launch.py``.
* 3D core (experimental): moving curve frames and a 3D CBF safety filter, also in
  ``vectorfield.d3`` and ``vf-sim3d``.
* ``obstacle_mode: cbf`` for Ackermann robots (rear axle, turning margin).
* Reversing: ``max_reverse_vel`` bounds the speed while backing up, obstacles on reverse legs
  are contoured, and a forward corner sharper than 90° is no longer read as a cusp.
* Field-test kit and tutorials: recording script, annotated ``nav2_params`` templates and
  example clients in vectorfield_bringup.
* See each package's changelog for the details.

0.2.0 (2026-10-20)
------------------
* Adds vectorfield_behavior_tree.

0.1.1 (2026-10-20)
------------------

0.1.0 (2026-10-19)
------------------
* First release of the metapackage.
