^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-19)
------------------
* First release.
* Add the ``vf-curve``, ``vf-plot``, ``vf-sim`` and ``vf-tune`` command-line tools
  (``vectorfield_tools``, M4): curve description and Curve.msg/CSV export, field plots with
  streamlines, obstacles and the field decomposition, closed-loop simulation with plots and GIF
  animations, and configuration validation with suggestions and parameter sweeps. matplotlib,
  Pillow and PyYAML are imported lazily (the ``tools`` extra); the tools are pip entry points and
  ament programs (``ros2 run vectorfield_py vf-sim``).
* Bind the core API (M4): curves, gains, fields, kinematics, tracking laws, the command
  limiter, the occupancy grid and EDT, the footprint checker, the collision predictor, the
  Tracker, the closed-loop simulator and ``validate_config``, with NumPy I/O and the
  ``ConfigError`` / ``CurveError`` exceptions; pytest suite run by both build paths.
* Add the package skeleton: pybind11 extension ``vectorfield._vectorfield`` (exposes
  ``__version__``), the ``vectorfield`` and ``vectorfield_tools`` Python packages, a dual
  ament_cmake / scikit-build-core build with the version read from package.xml, and import
  tests.
