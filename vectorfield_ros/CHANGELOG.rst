^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
