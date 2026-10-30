^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-10-30)
------------------
* shutdown_last <command> (executable, a launch prefix): runs a simulator that is not ours
  so that it stops after the nodes that read its /clock, as kinematic_sim does. The command
  runs in its own process group (a terminal's Ctrl+C reaches only shutdown_last), gets
  SIGINT once no node reads /clock (at most 4 s) and SIGTERM at once, and ends with
  shutdown_last.

0.5.1 (2026-10-29)
------------------
* kinematic_sim keeps publishing /clock after SIGINT or SIGTERM while another node reads it
  (at most 4 s). Nav2's container no longer hangs at shutdown in a transform lookup on
  simulated time, which made launch kill it.

0.5.0 (2026-10-25)
------------------

0.4.1 (2026-10-25)
------------------

0.4.0 (2026-10-24)
------------------

0.3.0 (2026-10-23)
------------------

0.2.0 (2026-10-20)
------------------

0.1.1 (2026-10-20)
------------------

0.1.0 (2026-10-19)
------------------
* First release: a kinematic simulator node for the four motion models (with skid slip), a laser
  scan against the map and runtime obstacles, odom, TF and /clock, initial pose and a map->odom
  step service for localization jumps.
