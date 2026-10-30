^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_nav2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-10-30)
------------------
* Followers: goals run on a race-free action server (vectorfield_ros::SingleGoalActionServer).
  Nav2's SimpleActionServer could accept a goal sent right after the previous goal's result and
  never run it. Cancelling a pending goal no longer ends the running one;
  compat::SimpleActionServer and compat::makeActionServer() are removed.
* Shutdowns: the loopback demo runs Nav2 as the kinematic demo does (a preloaded container) and
  starts the loopback simulator once the map is served (no start-up crash on Jazzy). The loopback
  and Gazebo demos stop their simulator after Nav2 (vectorfield_sim's shutdown_last), and the
  Gazebo demos preload the lifecycle manager into nav2_bringup's container. Every lifecycle manager
  the demos start runs without bonds (on Lyrical, map_server's manager reset it during its shutdown
  cleanup and the container crashed). Lyrical's Gazebo demos can still meet Nav2's own shutdown
  races (see vectorfield_bringup's README).
* Tests: every demo launch test checks that each process exits 0 at the shutdown signal.

0.5.1 (2026-10-29)
------------------
* Ackermann: while a contour episode or the CBF filter bends the field more than 90 degrees from
  the car's heading, the car keeps turning toward it at its minimum radius (half speed) instead
  of stopping with kHeadingInfeasible (the CBF filter only with collision detection on).
* MuJoCo Ackermann example: Nav2 plans once and replans only when the path becomes invalid; the
  1 Hz replans sometimes bent the path toward the unmapped box and stopped the car beside it.
* Demos and MuJoCo examples: bt_navigator waits 1 s, not 20 ms, for a server to accept a goal.
* Shutdowns: the simulators keep /clock running until its readers have shut down; the
  containers the demos start keep Nav2's lifecycle manager library loaded (no segmentation fault
  at exit), Humble's use a container without component_container_isolated's shutdown hang, and
  Lyrical's navigation managers run without bonds (no shutdown deadlock). The loopback and
  Gazebo demos run nav2_bringup's own container and simulator: they keep the container's exit
  crash (Jazzy and later), and the container can still wait at shutdown once their simulator has
  stopped /clock.

0.5.0 (2026-10-25)
------------------
* Obstacle contouring: the gate stays open while a replanned path still grazes the obstacle
  (obstacle_gate_release_margin), so Nav2's replans near an unmapped obstacle no longer leave the
  robot blocked against it.
* 3D CBF: voxel sites count from their voxel cubes, half a voxel diagonal closer than the centre.
* 3D follower: hold_between_goals streams zero setpoints between goals for PX4 and ArduPilot.
* MuJoCo examples: review fixes (recording errors, map loading as map_server, the car's tyres,
  near-plane clipping, overlay budget, the gallery script's interrupts and failures); the
  gallery clips are recorded again, the drone's without its lattice of field arrows.
* Docs: the left sidebar keeps its scroll position between pages; animated RViz clips of every
  phase; clips and figures in the quickstart; a drone and Scout Mini clip in the overview; the
  roadmap page is gone and the design notes are renamed by topic (old URLs redirect).

0.4.1 (2026-10-25)
------------------
* Fixes from a review of 0.4.0:

  * The curve follower waits up to 0.5 s at goal start for the Nav2 action statuses to settle,
    so a curve started right after a Nav2 action (an approach, the FollowCurve BT node after
    FollowPath) is no longer refused with NAV2_ACTIVE.
  * MuJoCo examples: no ~40 ms per camera frame on Humble; the viewer's render thread is joined
    at shutdown; rates are range-checked; concurrent first Menagerie fetches; run_scenario
    (a lone negative-x waypoint, a bounded wait for Nav2, the client cancels its goal on
    timeout or interrupt); scenario params get Humble's plugin names and behaviour-tree paths.
  * Docs: release status, field grid default, rosdep lines that skip the MuJoCo pip key, bloom
    ignores vectorfield_mujoco.

0.4.0 (2026-10-24)
------------------
* MuJoCo examples (vectorfield_mujoco, source-only): one launch per motion model, with physics,
  the controller, RViz and a scenario that drives the robot by itself: differential (PAL TIAGo,
  Nav2 + obstacle contour), skid-steer (AgileX Scout Mini with the AgileX manual's geometry,
  through a door), omnidirectional (Stanford TidyBot base, curve follower), Ackermann (car,
  Smac Hybrid + CBF), drone (Skydio X2, 3D follower + CBF) and curve following (Scout Mini,
  lemniscate laps); clips of each in the README and the docs.
* Documentation site revamped (Furo, light and dark): a landing page with the example gallery
  and an Examples section with one page per robot.
* RViz and Foxglove layouts show the field grid by default and orbit the robot; the 3D
  follower's field arrows have heads and are coloured by distance to the curve.
* send_curve.py no longer has its curve refused right after a successful approach.
* See each package's changelog for the details.

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
