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
