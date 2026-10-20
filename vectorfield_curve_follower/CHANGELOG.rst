^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_curve_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2026-10-20)
------------------

0.1.0 (2026-10-19)
------------------
* First release: the FollowCurve action server for closed, parametric, expression and moving
  curves, with laps and progress, Odometry and TF frame-twist sources, the exclusive-actions
  interlock, cmd_vel_nav published only while a goal executes, and one result code per failure
  mode.
* Pure helpers FrameTwistEstimator, GoalSupervisor and LapCounter; in-process tests for every
  failure mode.
