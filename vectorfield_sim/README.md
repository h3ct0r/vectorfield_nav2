# vectorfield_sim

A lightweight kinematic simulator for headless integration tests and demos. It runs on every
supported distro, including Humble, which has no Nav2 loopback simulator, and it adds what the
loopback simulator lacks: unmapped obstacles and skid-steer and Ackermann realism.

**Status:** skeleton — implemented in M3.

## Planned behaviour

- **Motion:** integrates the commanded twist with `vectorfield_core`'s exact-arc integrator for
  differential, skid-steer, omnidirectional and Ackermann bases. Skid-steer adds slip
  (η_ψ = 0.75, x_ICR = 0.05 m). Infeasible Ackermann commands (v = 0 with
  ω ≠ 0) are counted.
- **Input:** `cmd_vel`; the topic is a parameter (Humble demos use `cmd_vel_safe`, after
  collision_monitor). `Twist` or `TwistStamped` follows `enable_stamped_cmd_vel`, whose default
  matches the distro's Nav2 default.
- **Outputs:** `odom`, the TF `odom → base_footprint`, a static `map → odom` and `/clock`.
- **Laser scan:** `scan` in `base_scan` (360 beams, 10 Hz), ray-cast against `/map` plus
  unmapped `circle_obstacles` and `box_obstacles` (map frame, settable at runtime), so obstacle
  avoidance can be tested against obstacles the planner does not know about.
- **Also:** an `initialpose` subscription and a service that steps `map → odom`, used to test
  localization jumps.

Launch it through `kinematic_sim_demo.launch.py` in `vectorfield_bringup` (M3).
