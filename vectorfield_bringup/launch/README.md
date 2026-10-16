# launch

- `kinematic_sim_demo.launch.py` (every distro): `vectorfield_sim`, map_server, Nav2's
  `navigation_launch.py`, the static identity TFs base_footprint → base_link → base_scan,
  `initial_pose_publisher` and, on Humble, collision_monitor from `cmd_vel` to `cmd_vel_safe`.
- `loopback_demo.launch.py` (Jazzy and later): nav2_bringup's TB3 loopback wrapper
  (`tb3_loopback_simulation_launch.py`, or `tb3_loopback_simulation.launch.py` on Jazzy).
- `tb3_gazebo_demo.launch.py`, `tb4_gazebo_demo.launch.py` (Jazzy and later): nav2_bringup's
  Gazebo TurtleBot simulations with AMCL.
- M5: `curve_follower_demo.launch.py`, which starts the curve follower and
  `lifecycle_manager_curve_follower`.

Every demo starts RViz with `rviz/vectorfield.rviz` unless `use_rviz:=false`, and exits with an
error naming the supported distros when it cannot run on the current one. The arguments are
listed in the package README.
