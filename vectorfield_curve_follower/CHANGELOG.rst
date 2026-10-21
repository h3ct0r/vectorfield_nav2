^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_curve_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* result_code::kGoalRejected and kSendGoalFailure, the client-side codes of an inactive
  follower's rejection; the docs no longer claim that goals are never rejected.

0.2.0 (2026-10-20)
------------------
* CBF obstacle mode support.

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
