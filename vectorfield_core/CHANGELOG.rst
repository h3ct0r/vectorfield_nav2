^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-10-24)
------------------
* d3: DistanceField3::sampleAt(), the whole field sample (D, foot, G, H) at a point; at() is
  its field vector.

0.3.0 (2026-10-23)
------------------
* 3D core (experimental): obstacle avoidance by a 3D CBF safety filter (math_spec §14.8
  [Design, 0.3.0], m7_3d.md "Obstacles"). New d3/obstacles.hpp: analytic Primitive3 obstacles
  (sphere, oriented box, vertical cylinder or infinite pole, solid half-space) with exact
  signedDistance() and outward normals; VoxelGrid3 and DistanceTransform3, the exact separable
  Felzenszwalb-Huttenlocher distance and feature transform in 3D (rebuilt only when the grid
  changes) with a trilinear continuous query; ObstacleSet3 (primitives plus voxels, sites() for
  the filter); solveCbf3(), the exact active-set QP over up to four constraints. Tracker3Config::
  obstacles (Obstacles3Config: obstacle_mode none or cbf, body_radius, cbf_margin, cbf_gain,
  cbf_sites), Tracker3Input::obstacles, Tracker3State::blocked, Tracker3Output::cbf; the filter
  acts on every nonzero translational target, and its hold sets kBlocked with the planar 5 % /
  10 % band. VelocityLaw3::commandUniform(), SimulationOptions3::obstacles, SimulationRecord3::h
  and cbf_active. Without obstacles Tracker3 is bit-identical. Oracles O3D-15 to O3D-23
  (test_d3_obstacles, test_oracle_d3_cbf), no-allocation cases with obstacles and for the voxel
  EDT rebuild, and the benchmarks BM_D3CbfSolve_FourSites, BM_D3TrackerStep_Helix_Cbf* and
  BM_D3VoxelEdt_64.
* 3D core (experimental): moving curve frames (math_spec §14.7 [Design, 0.3.0], m7_3d.md "Moving
  frames"). New d3/frame.hpp: CurveFrame3 {T_eval_curve, FrameTwist3 twist, moving},
  expSE3()/logSE3()/bodyTwist()/extrapolate()/frameVelocity(), and FrameTwistEstimator3 (log_SE(3)
  differencing of sampled poses, outlier rejection, LPF, staleness). Tracker3Input::frame: the
  search runs in the curve frame with the hint kept there; closed curves get Phi_T = Pi_T v_c
  with v_c = R (v_F + w_F x r_F + dr_F/dt); open curves (static in the frame) follow the
  transported terminal field v_r Phi_S + v_tr(x) and the goal hold rides with the frame. New
  DistanceField3::evaluateTerminal() overload, Tracker3Output::frame_velocity and x_star_curve,
  SimulationOptions3::frame and frame_motion, SimulationRecord3::T_eval_curve,
  ClosedLoopSimulator3::frameAt(). A static identity frame is bit-identical to 0.2. Oracles
  O3D-10 to O3D-14 (test_d3_frame) and a moving-frame no-allocation case.
* obstacle_mode cbf for Ackermann (math_spec §6.7 [Design, 0.3.0], m7_cbf.md "Ackermann"): the
  error ackermann_cbf_unsupported is removed. The filter runs at the rear axle on
  max(0, h_j − μ_j), with the turning margin μ_j = R_min (1 − sqrt(1 − c_j²)) for
  c_j = −n_jᵀ q h > 0 (the distance a full-lock turn away from a face covers toward it), and
  without the push-out a car cannot follow. CbfSample::turn_margin carries μ_j. Diff, skid and
  omni are bit-identical. New warnings ackermann_cbf_window (x_r + δ_safe + R_min + v_r/γ_b > half
  the costmap) and ackermann_cbf_margin (cbf_margin < v_ack²/(2 a_dec)). Oracle O59
  (test_oracle_cbf_ackermann, and a disc on the reverse leg in test_reversing).
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
