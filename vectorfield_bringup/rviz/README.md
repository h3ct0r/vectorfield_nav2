# rviz

`vectorfield.rviz` shows the map, both costmaps, the global plan, the robot model and TF, the
laser scan, the local footprint and the controller's `FollowPath/markers` (every namespace),
`FollowPath/field_grid` (off by default), `FollowPath/window` and `FollowPath/predicted_path`,
and the same four topics of `curve_follower` (the "Curve follower" group), coloured with the
legend documented in `vectorfield_ros`. Every demo launch opens it.

`follower_3d.rviz` is the 3D view of `follower3d_demo.launch.py`: an orbit camera, the grid, TF,
the robot's `/odom` trail and the 3D follower's `/follower_3d/markers` (the curve, the closest
point and tangent, the command, the field arrows, the goal, the obstacles and voxels, the body
sphere and the status text).
