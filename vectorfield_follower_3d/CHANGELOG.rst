^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_follower_3d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-10-25)
------------------
* New parameter hold_between_goals (false): while active and without a goal, publish a zero
  setpoint every control period, so that PX4's offboard mode and ArduPilot's guided mode keep
  receiving setpoints.
* ~/markers is built only while someone subscribes, and its lattice of field arrows is capped
  at 7 per axis (a finer field_arrows_step is widened).

0.4.1 (2026-10-25)
------------------

0.4.0 (2026-10-24)
------------------
* The field arrows in ~/markers have heads (three strokes 120° apart, readable from any
  viewpoint) and are coloured viridis by the distance to the curve, like the 2D field grid.

0.3.0 (2026-10-23)
------------------
* First version (experimental): the ``follower_3d`` lifecycle component with the
  ``follow_curve_3d`` action (FollowCurve3) around vectorfield_core's Tracker3, for drones and
  other robots that take a 3D velocity setpoint. Odometry in, TwistStamped out (global ENU or
  heading-frame body velocity), the planar follower's safety contract and failure table, moving
  frames from Odometry or TF, CBF obstacle avoidance against primitives from parameters and an
  optional voxelised PointCloud2, FieldState3 and RViz markers under ``~/``.
* ``point_sim_3d``: a kinematic point simulator (odometry, TF, /clock, an optional moving frame
  and a point cloud of spherical obstacles) for the tests and the demo.
