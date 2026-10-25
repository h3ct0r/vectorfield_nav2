^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_follower_3d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
