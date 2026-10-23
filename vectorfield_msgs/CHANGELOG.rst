^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-10-23)
------------------
* Curve3, FieldState3 and the FollowCurve3 action for the experimental 3D follower
  (vectorfield_follower_3d): a curve in R^3 (polyline, expressions x/y/z, built-in shapes with a
  6-DoF pose), FollowCurve's goal fields and result codes, and a per-cycle state for plots. Additive:
  no existing definition changes.
* FollowCurve result: GOAL_REJECTED (1) and SEND_GOAL_FAILURE (2), the client-side codes Nav2's
  BtActionNode looks for (with TIMEOUT), so Lyrical no longer warns when it creates a FollowCurve
  node. An additive interface change: no existing constant changes value.

0.2.0 (2026-10-20)
------------------

0.1.1 (2026-10-20)
------------------

0.1.0 (2026-10-19)
------------------
* First release: the Curve and FieldState messages and the FollowCurve action.
