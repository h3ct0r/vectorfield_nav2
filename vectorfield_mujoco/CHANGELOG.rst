^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_mujoco
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-10-30)
------------------
* mujoco_sim counts its /clock readers as kinematic_sim does: another clock source is no
  reader (clock_readers()).
* test_example_launch checks that every process of the example exits 0 at the shutdown
  signal.

0.5.1 (2026-10-29)
------------------
* The Ackermann example plans once and replans only when the path becomes invalid
  (navigate_w_replanning_only_if_path_becomes_invalid.xml). With the overlay's 1 Hz
  replanning, a new path sometimes bent toward the unmapped box and led the car to its side,
  where the car stopped.
* mujoco_sim keeps publishing /clock after SIGINT or SIGTERM while another node reads it (at
  most 4 s): Ctrl+C no longer leaves Nav2's container waiting on simulated time until launch
  kills it. The signal finishes the recording at once, and while the node is being built (a
  first-run Menagerie download) SIGINT and SIGTERM still interrupt it.
* config/nav2.yaml, merged by the ground examples: bt_navigator waits 1 s, not Nav2's 20 ms, for
  a server to answer, as in vectorfield_bringup's kinematic demos.

0.5.0 (2026-10-25)
------------------
* The drone's recording hides the 3D follower's lattice of field arrows, as the ground examples
  hide theirs; the gallery clips are recorded again.
* mujoco_sim: a recording that fails stops quietly for the rest of its tick (the first error
  stays visible), and an uncreatable record_path directory disables recording instead of
  killing the node; MuJoCo's auto-reset of a diverged state is undone (the last pose at rest,
  time kept going forward, the command zeroed) and logged; overlay_topics are subscribed only
  with a viewer, a camera or a recording, so the controllers' markers stay unbuilt otherwise.
* Overlays fill the geom budget smallest marker first: a large lattice of field arrows is cut,
  never the robot's own markers.
* Worlds load Nav2 maps as map_server does: the mean of the colour channels (not the luma),
  the alpha in trinary mode, and the scale and raw modes.
* Small worlds widen vis.map.shadowclip instead of the statistic extent, which also pushed the
  near clipping plane out (close views of the drone were clipped).
* The Ackermann command counts as infeasible below vectorfield_sim's zero speed (1e-6 m/s), and
  the adapter keeps the car's wheels straight and still there.
* The drone's cbf_margin is 0.015 m (was 0.1 m): the 3D CBF now counts voxel sites from their
  cubes, which covers the voxel blur the larger margin allowed for; the barrier keeps the same
  0.5 m from the voxel centres.
* The Ackermann car's tyres get the Scout Mini's soft first touch: skidding tyres stay on the
  floor (four-tyre contact 99 % of a hard turn, from 36 %).
* record_gallery.sh: Ctrl-C and SIGTERM stop the running example before the script exits; a
  failed encode fails only that example; the work directory is kept when an example failed;
  the size summary prints when nothing matched.
* convert_scout_mini.py caches the upstream files per commit.
* The refusal on unsupported distros names mujoco_drone.launch.py; tf2_msgs is a test
  dependency; test_sim_node tears its nodes down when a test or the node's start fails.

0.4.1 (2026-10-25)
------------------
* mujoco_sim: the camera image and the obstacle cloud are assigned as array.array, which
  rosidl's uint8[] setter takes as is (on Humble, bytes cost ~40 ms per 640x360 frame inside
  the physics timer); close() joins the passive viewer's render thread (Handle.is_running()
  turns False at once); odom_rate, scan_rate, scan_beams, camera_rate and cmd_vel_timeout are
  range-checked; a concurrent first Menagerie fetch reuses the checkout another process finished.
* run_scenario: a lone waypoint with a negative x no longer reads as an option; the wait for
  Nav2 is bounded (300 s); the client gets one SIGINT (it cancels its goal) on the timeout,
  Ctrl-C or SIGTERM instead of being killed; scenario params go through params.merge's
  substitutions, behaviour-tree paths and Humble plugin names.
* test_sim_node runs isolated (a ROS domain of its own).

0.4.0 (2026-10-24)
------------------
* mujoco_sim waits for the passive viewer's thread when it closes and, after a viewer session,
  exits without the interpreter's teardown: with viewer:=true the node crashed (SIGSEGV) at the
  end of an example.
* The drone clip shows the 3D follower's coloured field arrows.
* New package: MuJoCo physics for the examples (source-only; MuJoCo comes from pip through the
  python3-mujoco-pip rosdep key). Pure-Python core without ROS imports:

  * registry: the robots tiago (MuJoCo Menagerie PAL TIAGo, upper body frozen, ellipsoid wheel
    contacts), scout_mini, ackermann_car, tidybot (Menagerie Stanford TidyBot base with velocity
    servos) and x2 (Menagerie Skydio X2); wheel radius, track, wheelbase and steering limit are
    read from the compiled model.
  * menagerie and ``fetch_menagerie``: sparse clone of the three Menagerie robot directories at
    a pinned commit into ``$VF_MENAGERIE_DIR`` or ``~/.cache/vectorfield_mujoco``.
  * scene: worlds whose walls are the occupied cells of a Nav2 map (greedily merged boxes),
    plus unmapped obstacles and the attached robot.
  * adapters (differential, skid-steer, Ackermann, holonomic), flight (cascaded quadrotor
    velocity controller with a priority mixer), sensors (2D lidar, obstacle cloud), overlay
    (markers drawn into a MuJoCo scene), recorder (MP4 in simulated time) and gl.

* models/scout_mini: the AgileX Scout Mini converted from agilexrobotics/ugv_gazebo_sim by
  tools/convert_scout_mini.py (BSD per its package.xml; changes in its README.md), with the
  wheel geometry and mass of the AgileX user manual (wheelbase 452 mm, track 450 mm, 175 mm
  wheels, 20 kg) and the chassis mesh turned to lie along x (front fence at +x). Its tyre
  contacts are soft at first touch (solimp 0.05 0.95 0.001), so the skidding wheels stay on the
  floor instead of hopping: the base turns in place at 0.49 of its no-slip yaw rate (isotropic
  Coulomb friction: 0.50; 0.15 to 0.25 before), and skid_steer and curve_following use the
  re-identified slip (skid_yaw_rate_efficiency 0.45).
* models/ackermann_car: a front-steered, rear-driven car built from primitives, sized for the
  Ackermann controller overlay.
* mujoco_sim (sim_node, simulation): the MuJoCo simulator node with vectorfield_sim's interface
  for ground robots (/clock, cmd_vel or cmd_vel_safe, Twist or TwistStamped by the distro
  default, odom with the ground-truth pose, odom -> base_footprint, static map -> odom and the
  model's base_link and base_scan, scan, initialpose, ~/ground_truth, ~/step_map_to_odom,
  ~/infeasible_command_count); worlds from the Nav2 map and unmapped obstacles; the passive
  viewer, an offscreen camera Image topic, MP4 recording in simulated time and MarkerArray and
  Path overlays in all of them. Menagerie robots are fetched on first use.
* Launch files: mujoco_nav2 and mujoco_curve_follower (robot, world, obstacles, scenario, views,
  recording, initial pose) and the examples differential (TIAGo, contour around an unmapped
  box), skid_steer (Scout Mini through corridor_door, identified slip compensation),
  omnidirectional (TidyBot, curve follower, rounded rectangle at a fixed heading), ackermann
  (own car, Smac Hybrid DUBIN, CBF swerve around a box next to the path) and curve_following
  (Scout Mini, lemniscate laps), on vectorfield_bringup's demo helpers.
* config/mujoco_sim.yaml, config/robots/<robot>.yaml (footprints, limits within the actuators,
  full velocity smoother lists, lidar) and config/scenarios/<example>.yaml (obstacles, initial
  pose, waypoints or curve and laps, Nav2 behaviour overlays); examples.merged_params() merges
  them onto vectorfield_bringup's motion-model overlays.
* run_scenario: drives a scenario with navigate_waypoints.py or send_curve.py and exits with 0
  when every goal succeeds; the example launches start it (exit_on_done ends the launch).
* gl chooses a headless backend only when its library is installed (OSMesa, then EGL): with
  MUJOCO_GL=osmesa and no OSMesa, ``import mujoco`` failed inside PyOpenGL (CI on Humble and
  Jazzy). With no backend MUJOCO_GL stays unset, the physics runs and the node refuses only the
  viewer, camera and recording; gl.probe() checks importing and rendering in a subprocess, and
  the rendering tests skip without a backend. New dependency libosmesa6-dev (OSMesa).
* recorder: ffmpeg runs in its own session, so a Ctrl-C (sent to the whole process group) no
  longer kills it before the MP4's index is written.
* Tests: unit tests of the node, the frames and the configs (shipped-config rules, actuator
  limits, validate_config); the per-PR launch test test_mujoco_smoke (label mujoco, the vendored
  Scout Mini) and, with -DVF_NIGHTLY_TESTS=ON, a launch test per ground example.
* Drone: mujoco_sim takes the aerial robot (x2) with point_sim_3d's interface
  (sim_node.AerialInterface): TwistStamped cmd_vel in the heading frame when its frame_id is the
  base frame, else in the global frame; odom (pose in map with the full attitude, twist in
  base_link), TF map -> base_link, /clock, obstacle_cloud (the world's obstacle and wall
  surfaces), point_sim_3d's optional moving frame (TF and Odometry) and initial_pose_z (0 rests
  on the floor). The flight controller tracks the command at the physics rate, standing in for
  an autopilot's velocity mode. Simulation.set_velocity() is the 3D command path; the ground
  path is unchanged. mujoco_sim also exits cleanly when a SIGINT shuts rclpy down while its
  executor takes a message (a RuntimeError from take_message, seen on Jazzy).
* Launch files mujoco_drone (generic: robot, world, obstacles, scenario, cmd_frame, views) and
  the example drone: the Skydio X2 takes off and flies the helix with the 3D follower, whose CBF
  filter steers it around a pillar and a sphere seen only through the obstacle cloud
  (vectorfield_bringup.demo.follower3d_actions()); config/robots/x2.yaml (the follower's limits
  within the flight controller's, a 0.4 m body radius covering the rotors, cbf_margin 0.1) and
  config/scenarios/drone.yaml (curve3d, initial_z). run_scenario flies a curve3d with
  send_curve3d.py once follower_3d is active.
* registry: the X2's own spotlight is removed (the world's sun casts its shadow). scene: a world
  smaller than open_10x10 gets its statistic extent (MIN_EXTENT, 20 m), which sizes the shadow
  map; small worlds had a speckled floor.
* Tests: the heading-frame command, the moving frame's motion, the drone's command path and the
  aerial interface on an X2-like quadrotor in test/models (per PR, no download); the drone
  config and scenario checks; with -DVF_NIGHTLY_TESTS=ON, the drone example's launch test.
* Gallery: scripts/record_gallery.sh records every example headless and writes the README and
  docs clips (docs/_static/media/<example>.mp4, .gif and .jpg, captioned, within size budgets).
  A scenario's ``recording`` sets its clip's camera, view, hidden markers and the robot's trail;
  mujoco_sim's ``record_view``, ``record_hide`` and ``record_trail`` parameters take them, and
  ``record_camera`` defaults to the scenario's. A ``follow`` camera follows the robot from a fixed
  azimuth and elevation. The scene's headlight is dimmer, so the floor's checker no longer
  saturates, and a Path overlay keeps its colour whichever topic appears first.
