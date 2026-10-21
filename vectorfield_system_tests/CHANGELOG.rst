^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_system_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* A test client that had to abandon its participant (Lyrical) dumps the thread stacks 60 s after
  it closes: such a process freezes in launch_ros's shutdown after its tests.
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
