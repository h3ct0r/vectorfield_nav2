^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_system_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-10-30)
------------------
* test_clock_source: shutdown_last holds SIGINT back while /clock has a reader (also at a
  terminal's Ctrl+C), forwards it once the reader leaves or at once without one, forwards
  SIGTERM at once, takes its command along when it is killed, and returns the command's
  status.
* The loopback launch tests run in the coverage build again: bringup's loopback demo no
  longer starts Jazzy's loopback simulator before the map is served.
* Every launch test checks after the shutdown that each process of the demo exited 0
  (shutdown.TestCleanShutdown): a crash at exit, or a process launch had to kill, fails it.

0.5.1 (2026-10-29)
------------------
* test_clock_source: after SIGINT, kinematic_sim and point_sim_3d keep their clock running
  while it has a reader and exit soon after the last one; point_sim_3d without publish_clock
  exits at once.

0.5.0 (2026-10-25)
------------------

0.4.1 (2026-10-25)
------------------

0.4.0 (2026-10-24)
------------------

0.3.0 (2026-10-23)
------------------
* test_follower_3d (experimental 3D follower on point_sim_3d, every distro): an open helix, two
  laps of a Viviani curve (max |D| after lap 1 < 0.05 m), a sphere beside a circle seen through
  the simulator's point cloud (true barrier >= -resolution) and a cancel, each ending with one
  zero and then silence. Helpers in vectorfield_system_tests.follower_3d.
* test_cbf_kinematic runs for Ackermann (the box beside the path; no infeasible command at the
  simulator), and the shipped Ackermann overlay and template validate with obstacle_mode cbf,
  without warnings in the stock 3 m window.
* test_reversing_kinematic: the global obstacle layer is off and NavigateToPose replans only
  when the plan becomes invalid (test overlay replan_only_if_path_invalid); the reverse
  ground-truth speed stays within the overlay's max_reverse_vel, and (differential) Smac backs up
  2.7 m past an unmapped box on the reverse leg, which the contour passes in reverse.
* test_through_poses_kinematic (differential): NavigateThroughPoses through a 150° waypoint
  corner on NavFn succeeds, driving forward only. NavHarness.navigate_through() sends
  NavigateThroughPoses goals (Humble to Lyrical).
* test_skid_slip_kinematic (skid_steer): identify_skid_slip.py recovers vectorfield_sim's slip
  within 10 %.
* compare_controllers.py and geometry.OccupiedCells use vectorfield_bringup.field_metrics, shared
  with the field-test report.
* A test client that had to abandon its participant (Lyrical) dumps the thread stacks 60 s after
  it closes: such a process freezes in launch_ros's shutdown after its tests.
* Fast DDS 3 (Lyrical): the launch tests no longer freeze until the CTest timeout. Within a test
  process, the TypeLookup service of launch_ros's participant and that of the test client could
  deadlock each other (README, Known issues). The launch tests now keep builtin traffic out of
  intraprocess delivery (test/fastdds_no_builtin_intraprocess.xml).
* test_shipped_configs also validates vectorfield_bringup's config/examples templates and
  rotation-shim overlays.

0.2.0 (2026-10-20)
------------------
* End-to-end tests for the behaviour-tree node, reversing with Smac Reeds-Shepp (diff and
  Ackermann), Ackermann contour and the CBF mode.
* Coverage builds scale the wall-clock budgets (VF_TEST_TIME_SCALE).

0.1.1 (2026-10-20)
------------------
* The harness keeps its lifecycle clients alive (destroying them killed the spin thread on Humble)
  and reads poses by timestamp.
* Known issue documented: Jazzy's loopback simulator divides by the resolution of map_server's
  empty pre-activation map.

0.1.0 (2026-10-19)
------------------
* End-to-end launch tests per distro and motion model (route, obstacle contour, corridor, blocked
  path, final-command publishers, curve follower laps and moving frames), shipped-configuration
  validation, and the controller comparison script.
