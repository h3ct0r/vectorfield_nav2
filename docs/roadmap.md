# Roadmap

Development is split into milestones. A milestone is done when its exit criteria hold and CI is
green on Humble, Jazzy (amd64 and arm64) and Lyrical. Rolling builds nightly without gating
releases; its failures become issues.

Status: M0 to M5 are done; M6 is in progress; M7 is optional.

| Milestone | Delivers | Exit criteria |
|---|---|---|
| **M0** | Design documents (math specification, upstream audit, Nav2 API matrix); repository scaffolding, community files and changelogs; devcontainers, pre-commit and the CI skeleton; `vectorfield_msgs`; this documentation skeleton | Packages build on the CI matrix; the documentation deploys. |
| **M1** | `vectorfield_core`: curves, fields, tracking laws, limiter, obstacle contouring, collision prediction and the `Tracker` | Core unit and scenario tests pass, including AddressSanitizer, ThreadSanitizer and the no-allocation test; core line coverage ≥ 90 %; benchmarks run. |
| **M2** | `vectorfield_ros` (including the marker builder and visualizer) and `vectorfield_nav2_controller`, with an in-process test harness that mimics `controller_server` on each distro | Plugin tests pass, including the visualization cases; path-handler parity with Nav2 passes on Lyrical; the API-generation compile test passes; `FieldState` frozen. |
| **M3** | `vectorfield_sim`, controller bringup and system tests | On every distro and motion model, a fixed three-goal route succeeds within 120 s, with the cross-track error below 0.10 m after convergence (0.25 m for Ackermann) and no controller failures in free space. Obstacle-contour and corridor tests pass for differential, skid-steer and omnidirectional robots; Ackermann stops short of a blocked corridor. Shipped configurations validate. |
| **M4** | `vectorfield_py` bindings and command-line tools; theory pages with generated figures | pip install and pytest pass on macOS; the `vf-sim` animation converges. |
| **M5** | `vectorfield_curve_follower`, its demo, `send_curve.py` and `moving_frame_publisher.py`; `Curve` and `FollowCurve` frozen | Three laps of an ellipse and of a lemniscate at 0.4 m/s with distance below 0.05 m after the first lap; translating and rotating curves tracked within 0.10 m with both frame-velocity sources; an open polyline completes. |
| **M6** | Complete documentation (including the visualization guide with screenshots), benchmarks, controller comparison, migration guide; release 0.1.0 | `sphinx-build -W` passes; the comparison table comes from a recorded run; 0.1.0 tagged and bloom release requests opened for Humble, Jazzy and Lyrical. |
| **M7** (optional) | Control-barrier-function obstacle mode; reversing and path cusps, and Nav2 path constraints (done: {ref}`reversing`); Ackermann contouring (done: {ref}`ackermann-contour`); a `FollowCurve` behaviour-tree node (done: {doc}`behavior_tree`); global-costmap checks in the curve follower; a 3D core; moving polylines | — |
