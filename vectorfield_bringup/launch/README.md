# launch

Launch files arrive in later milestones:

- M3: `loopback_demo.launch.py` (Jazzy and later, Nav2's TB3 loopback simulator),
  `kinematic_sim_demo.launch.py` (every distro, `vectorfield_sim`), and
  `tb3_gazebo_demo.launch.py` / `tb4_gazebo_demo.launch.py` (Jazzy and later).
- M5: `curve_follower_demo.launch.py`, which starts the curve follower and
  `lifecycle_manager_curve_follower`.

Every demo starts RViz with `rviz/vectorfield.rviz` unless `use_rviz:=false`.
