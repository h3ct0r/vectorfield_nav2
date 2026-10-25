^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-10-25)
------------------
* ObstacleConfig.obstacle_gate_release_margin.

0.4.1 (2026-10-25)
------------------

0.4.0 (2026-10-24)
------------------

0.3.0 (2026-10-23)
------------------
* vectorfield.d3 obstacles (math_spec 14.8): Primitive (sphere, box, cylinder, plane;
  signed_distance(), validate()), PrimitiveType, VoxelGrid ((nz, ny, nx) masks),
  DistanceTransform (distances(), feature(), query()), ObstacleSet (add(), set_voxels(),
  nearest(), sites()), solve_cbf(), ObstaclesConfig as TrackerConfig.obstacles, CbfSample as
  TrackerOutput.cbf, TrackerOutput.cmd_target_v and TrackerState.blocked; Tracker.step(obstacles=...)
  and d3.sim.ClosedLoopSimulator(obstacles=...), whose records add h and cbf_active. vf-sim3d
  --sphere, --box, --cylinder, --plane, --body-radius, --cbf-margin and --cbf-gain: the summary
  reports the least barrier, the plot and the GIF draw the obstacles, and the CSV adds h.
* vectorfield.d3 moving frames: CurveFrame (T_eval_curve (4, 4), v, w, moving, at(), velocity()),
  exp_se3(), log_se3(), body_twist(), FrameTwistEstimator, FrameTwistConfig, FrameEstimate and
  FrameSampleResult; Tracker.step(frame=...), TrackerOutput.Phi_T, v_c, frame_velocity and
  x_star_curve; d3.sim.ClosedLoopSimulator(frame=..., frame_motion=...), frame_at() and the
  records' frame (N, 4, 4). vf-sim3d --frame-velocity and --frame-rate (the plot and the GIF
  draw the moving curve).
* CbfSample.turn_margin and the issue codes ACKERMANN_CBF_WINDOW and ACKERMANN_CBF_MARGIN;
  ACKERMANN_CBF_UNSUPPORTED is removed (obstacle_mode cbf now supports Ackermann). vf-tune hints
  for the new warnings.
* MotionConfig.max_reverse_vel, Kinematics.max_reverse_vel and linear_limit(),
  reverse_speed_limit(), reference_speed(), and the issue codes REVERSE_ABOVE_MAX and
  ACKERMANN_REVERSE_BELOW_BAND.

0.2.0 (2026-10-20)
------------------
* Bindings for the CBF and reversing parameters, and the 3D core as vectorfield.d3; new tools
  vf-sim3d and vf-curve --dim 3; vf-sim and vf-plot --obstacle-mode cbf.

0.1.1 (2026-10-20)
------------------
* CoreConfig exposes the skid slip compensation parameters.

0.1.0 (2026-10-19)
------------------
* First release.
* Add the ``vf-curve``, ``vf-plot``, ``vf-sim`` and ``vf-tune`` command-line tools
  (``vectorfield_tools``): curve description and Curve.msg/CSV export, field plots with
  streamlines, obstacles and the field decomposition, closed-loop simulation with plots and GIF
  animations, and configuration validation with suggestions and parameter sweeps. matplotlib,
  Pillow and PyYAML are imported lazily (the ``tools`` extra); the tools are pip entry points and
  ament programs (``ros2 run vectorfield_py vf-sim``).
* Bind the core API: curves, gains, fields, kinematics, tracking laws, the command
  limiter, the occupancy grid and EDT, the footprint checker, the collision predictor, the
  Tracker, the closed-loop simulator and ``validate_config``, with NumPy I/O and the
  ``ConfigError`` / ``CurveError`` exceptions; pytest suite run by both build paths.
* Add the package skeleton: pybind11 extension ``vectorfield._vectorfield`` (exposes
  ``__version__``), the ``vectorfield`` and ``vectorfield_tools`` Python packages, a dual
  ament_cmake / scikit-build-core build with the version read from package.xml, and import
  tests.
