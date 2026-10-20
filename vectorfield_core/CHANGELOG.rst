^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2026-10-20)
------------------
* In ROTATE_TO_FIELD, ROTATE_TO_HEADING and STOPPED the translational axes brake at their own
  deceleration bound instead of sharing the angular axis' acceleration scale, so a rotation that
  starts with residual speed no longer creeps forward (21 mm down to 2.7 mm after a 0.1 m/s
  approach).
* Skid-steer slip compensation: the new skid_yaw_rate_efficiency and skid_icr_offset (defaults 1
  and 0, off).
* Curve expressions with && or || are rejected: muParser folds them through an int cast that
  overflows for large operands (found by the fuzzer).

0.1.0 (2026-10-19)
------------------
* First release: a ROS-free C++17 library (Eigen, vendored muParser) implementing the constructive
  time-varying vector field of Rezende, Gonçalves and Pimenta (IEEE T-RO, 2022).
* Curves: polylines with windowed and hinted closest-point search, parametric and muParser
  expression curves (hardened parser with a libFuzzer target), seven closed built-in shapes,
  time-varying curves.
* Fields: static and time-varying distance fields with the eta normalisation and the Assumption-3
  clamp; a terminal field for open paths with a reference-point shift.
* Tracking laws for differential, skid-steer, omnidirectional and Ackermann bases (feedback
  linearization, heading tracking, holonomic).
* Tracker: rotate-to-field, follow, final-approach, rotate-to-heading and stopped phases, the
  settled and goal-hold rules, the cancel ramp, and a command limiter with one acceleration scale
  for all axes.
* Obstacles: occupancy grid with an exact distance and feature transform, footprint checker,
  collision rollout with a braking cap and kBlocked hysteresis, and obstacle contouring with a
  path-blocked gate, a blocking set, a side probe that must get past the blockage, and a watchdog.
* Kinematic simulator, configuration validation with stability advisories, no allocation in steady
  state, and benchmarks.
