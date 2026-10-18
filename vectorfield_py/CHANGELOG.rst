^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Bind the core API (M4): curves, gains, fields, kinematics, tracking laws, the command
  limiter, the occupancy grid and EDT, the footprint checker, the collision predictor, the
  Tracker, the closed-loop simulator and ``validate_config``, with NumPy I/O and the
  ``ConfigError`` / ``CurveError`` exceptions; pytest suite run by both build paths.
* Add the package skeleton: pybind11 extension ``vectorfield._vectorfield`` (exposes
  ``__version__``), the ``vectorfield`` and ``vectorfield_tools`` Python packages, a dual
  ament_cmake / scikit-build-core build with the version read from package.xml, and import
  tests.
