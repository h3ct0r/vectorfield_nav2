^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_nav2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
