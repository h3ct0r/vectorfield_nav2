^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_mujoco
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
  tools/convert_scout_mini.py (BSD per its package.xml; changes in its README.md).
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
  (own car, Smac Hybrid DUBIN, CBF beside a box, U-turn) and curve_following (Scout Mini,
  lemniscate laps), on vectorfield_bringup's demo helpers.
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
