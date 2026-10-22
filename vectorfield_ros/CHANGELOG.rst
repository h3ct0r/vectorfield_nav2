^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
