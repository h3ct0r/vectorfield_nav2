^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_curve_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-10-30)
------------------
* Goals run on vectorfield_ros::SingleGoalActionServer: a goal sent right after the previous
  goal's result always runs (Nav2's SimpleActionServer could accept it and never execute
  it). Cancelling a pending goal no longer ends the running one.
* The wait at goal start for an exclusive Nav2 action to end stops when the follower is
  deactivated.

0.5.1 (2026-10-29)
------------------

0.5.0 (2026-10-25)
------------------

0.4.1 (2026-10-25)
------------------
* At goal start the Nav2 interlock waits up to 0.5 s for the exclusive actions' statuses to
  settle: an action that has just ended publishes its result before its final status, so a
  client starting the curve right after it (an approach, the FollowCurve BT node after
  FollowPath) was refused with NAV2_ACTIVE.

0.4.0 (2026-10-24)
------------------

0.3.0 (2026-10-23)
------------------
* Progress supervision uses the reverse field speed min(desired_speed, max_reverse_vel) for a
  drive_backward robot.
* result_code::kGoalRejected and kSendGoalFailure, the client-side codes of an inactive
  follower's rejection; the docs no longer claim that goals are never rejected.
* SupervisorInput::position_z: the progress watchdog measures the 3D displacement for the 3D
  follower (vectorfield_follower_3d). Planar callers leave it at 0 and are unchanged.

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
