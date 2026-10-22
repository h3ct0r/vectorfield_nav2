^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* max_reverse_vel (MotionConfig, auto = max_linear_vel): with q = -1 (a reverse plan segment or
  drive_backward; not omni) the field speed is min(desired_speed, max_reverse_vel) and the
  kinematic box bounds |v| by it (Kinematics::max_reverse_vel, linearLimit(); reverseSpeedLimit(),
  referenceSpeed()). Errors max_reverse_vel_above_max_linear_vel and, for Ackermann,
  ackermann_reverse_below_min_speed. Forward behaviour is unchanged.
* The obstacle contour runs on reverse plan segments: at the law's point on the drive side, with
  the side probe rolling out the reverse pipeline (math_spec §9.4 [Design, 0.3.0]). New oracles:
  a disc on a reverse leg (diff FL, diff HT, Ackermann), a dead end while reversing, the reverse
  speed bound, and the CBF filter in reverse.
* findCusps() and planHasHeadings(): a turn of the chord-decimated plan sharper than 90° is a
  cusp only where the plan's headings change driving direction or, for a plan without headings
  (NavFn, Theta*, NavigateThroughPoses waypoints), where it doubles back along itself for more than
  2 l_c (legs within 3° of opposite and a cell apart). Sharp forward corners are no longer cusps.
  segmentPlan() uses both; hasCusp() remains the chord test that finds the candidates.
* despike(): drops a polyline's turns of more than 90° next to a segment shorter than a given
  length (the step back to the next leg's start cell at a NavigateThroughPoses junction).

0.2.0 (2026-10-20)
------------------
* Reversing: TrackerInput::drive_sign for plans with cusps; FL control point, heading tracking and
  Ackermann follow it, a sign change resets the hint, headings, contour episode and blocked latch.
* Ackermann contour: auto lambda of at least R_min + w_lat + 0.05, a turn-in band before D_in,
  curvature-aware validation, and a speed factor that keeps the car turning at R_min during an
  episode.
* CBF obstacle mode (obstacle_mode: cbf): a minimal-change barrier filter on the shaped field over
  up to three obstacle sites; a field pointing straight at an obstacle holds the robot and raises
  kBlocked.
* Experimental 3D core (vectorfield_core::d3): 3D curves and built-ins, DistanceField3 with the
  time-varying feedforward and terminal field, VelocityLaw3, CommandLimiter3, Tracker3 and a 3D
  simulator.

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
