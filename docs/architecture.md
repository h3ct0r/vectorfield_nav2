# Architecture

All control logic lives in `vectorfield_core`, a ROS-free C++17 library. The Nav2 plugin and the
curve follower are thin ROS layers around the same `Tracker`, so both run identical code and the
core can be tested, benchmarked and bound to Python without ROS.

## Packages

An arrow points from a package to a package it depends on. No package may depend, even only for
tests, on a package above it; tests that need several packages live in
`vectorfield_system_tests`. `vectorfield_behavior_tree` depends on `vectorfield_msgs` and Nav2's
`nav2_behavior_tree`, and on `vectorfield_ros` at build time only, for its CMake helpers (dotted);
it reaches the follower through the `follow_curve` action, not as a package.

```{graphviz}
:caption: Package dependencies. Dashed edges belong to the vectorfield_nav2 metapackage.

digraph packages {
  rankdir=BT;
  node [shape=box, style="rounded,filled", fillcolor="#f5f5f5", fontname="Helvetica", fontsize=11];
  edge [color="#555555"];

  core [label="vectorfield_core\n(ROS-free C++17)"];
  msgs [label="vectorfield_msgs"];
  ros [label="vectorfield_ros"];
  controller [label="vectorfield_nav2_controller"];
  follower [label="vectorfield_curve_follower"];
  bt [label="vectorfield_behavior_tree"];
  sim [label="vectorfield_sim"];
  py [label="vectorfield_py"];
  bringup [label="vectorfield_bringup"];
  system_tests [label="vectorfield_system_tests\n(not released)"];
  meta [label="vectorfield_nav2\n(metapackage)", fillcolor="#e8eef7"];

  ros -> core;
  ros -> msgs;
  controller -> ros;
  follower -> ros;
  bt -> msgs;
  bt -> ros [style=dotted];
  sim -> ros;
  sim -> core;
  py -> core;
  bringup -> controller;
  bringup -> follower;
  bringup -> bt;
  bringup -> sim;
  bringup -> py;
  system_tests -> bringup;

  edge [style=dashed, color="#8899bb"];
  meta -> {msgs core ros controller follower bt py sim};
}
```

Distro-specific code is confined to `vectorfield_ros`'s `compat.hpp`, the controller's two
signature blocks (Gen C on Lyrical and later, Gen A/B on Humble to Kilted) and the
`VF_CANCEL_RESET_OVERRIDE` macro. The plugin's test harness is the only other exception, besides
`vectorfield_behavior_tree`'s `bt_compat.hpp` (BT.CPP v3 or v4, and the blackboard node type).

## Control cycle

Each control cycle, the ROS layer converts its inputs, calls `Tracker::step` once and publishes
the result. Inside `step`, the stages run in this order:

```{graphviz}
:caption: One control cycle. When the predictor's braking cap binds, the nominal step runs again at the reduced speed.

digraph cycle {
  rankdir=TB;
  nodesep=0.4;
  ranksep=0.22;
  node [shape=box, style="rounded,filled", fillcolor="#f5f5f5", fontname="Helvetica",
        fontsize=12, width=3.2, height=0.36];
  edge [color="#555555", fontname="Helvetica", fontsize=11, arrowsize=0.7];

  window [label="window: plan window or Curve goal"];
  curve [label="curve: PolylineCurve / makeCurve"];
  closest [label="closest point: x*, T, n, D, s*"];
  field [label="field: speed shaping + DistanceField"];
  contour [label="contour: ObstacleContour"];
  law [label="law: TrackingLaw + phases"];
  box [label="box: kinematic limits"];
  limiter [label="limiter: CommandLimiter"];
  predictor [label="predictor: CollisionPredictor"];
  cmd [label="cmd: Twist + status", shape=oval, fillcolor="#e8f4e8"];
  grid [label="costmap snapshot:\nOccupancyGrid + EDT", fillcolor="#fbeee6", width=2.0, height=0.5];

  window -> curve -> closest -> field -> contour -> law -> box -> limiter -> predictor -> cmd;
  predictor -> field [style=dashed, label=" cap", constraint=false];
  grid -> contour [style=dotted];
  grid -> predictor [style=dotted];
  { rank=same; field; grid; }
}
```

window
: The plugin takes the server's pruned plan in the costmap frame and conditions its tail at the
  goal. The follower takes the `Curve` from the action goal.

curve
: `PolylineCurve::assign` for paths and polylines; `makeCurve` validates expressions and built-in
  shapes.

closest point
: A window scan around the robot in the plugin; a hinted local search with a guarded global stage
  in the follower.

field
: The speed shaping step, then the static, time-varying or terminal field
  (see {doc}`theory`).

contour
: The obstacle extension, when enabled and the path is blocked.

law
: Feedback linearisation, heading tracking or holonomic tracking, plus the phase logic.

box and limiter
: Velocity limits by scaling the field, then acceleration limits, the Ackermann curvature clamp
  and a non-finite guard.

predictor
: A rollout of the nominal step on the local costmap. It gives the free arc length $s_{free}$
  and the braking cap $\sqrt{2\, a_{dec} \max(0,\ s_{free} - m)}$, with $m$ = `stop_margin`.

cmd
: The command, the phase, the status bits and everything the visualizer draws.

Obstacle preparation runs once per call, before the rollout: the obstacle component is
flood-filled on the grid and its distance transform built. Both stay frozen for the rest of the
cycle, so every rollout step and every field probe sees the same obstacle.

## Tracker phases

`TrackerState` carries the phase from one cycle to the next. The transitions are evaluated in a
fixed order at the start of each `step`: a new goal resets the goal-scoped state; a cancel or a
STOPPED-entry status enters STOPPED; settled enters ROTATE_TO_HEADING or goal hold; the
ROTATE_TO_HEADING exit when XY is lost; the FINAL_APPROACH entry; the ROTATE_TO_FIELD entry and
exit. {doc}`motion_models` gives the entry and exit conditions in full.

```{graphviz}
:caption: Tracker phases. STOPPED is latched until the next goal or a reset; BLOCKED is a per-cycle status, not a phase.

digraph phases {
  rankdir=LR;
  nodesep=0.35;
  node [shape=box, style="rounded,filled", fontname="Helvetica", fontsize=11];
  edge [color="#555555", fontname="Helvetica", fontsize=10, arrowsize=0.7];

  rtf [label="ROTATE_TO_FIELD", fillcolor="#fff6c8"];
  follow [label="FOLLOW", fillcolor="#e3f4e3"];
  fa [label="FINAL_APPROACH", fillcolor="#e0f6f8"];
  rth [label="ROTATE_TO_HEADING", fillcolor="#e3e9f8"];
  hold [label="goal hold\n(FOLLOW or FINAL_APPROACH,\nzero command)", fillcolor="#f5f5f5"];
  stopped [label="STOPPED", fillcolor="#f8e3e3"];

  follow -> rtf [label="|e| > π/2"];
  rtf -> follow [label="|e| < π/4"];
  follow -> fa [label="σ < final_approach_distance"];
  fa -> rth [label="settled, yaw checked"];
  fa -> hold [label="settled, no rotation"];
  rth -> fa [label="XY lost"];
  hold -> fa [label="XY lost"];
  rtf -> fa [label="|e| < π/4, near the goal"];
  any [label="any phase", shape=plaintext, style=""];
  any -> stopped [style=dashed, label="cancel, contour watchdog,\nheading infeasible"];
}
```

## State, ownership and threads

- **The `Tracker` owns only configuration.** `Tracker::step` is `const noexcept` and takes the
  previous `TrackerState` and writes the next one; the output may alias the input.
- **`TrackerState` is the only cross-cycle state.** It holds fixed-size Eigen types and plain
  values only and is statically asserted to be trivially destructible. It never stores window
  arc lengths or vertex indices, so a re-pruned or re-transformed plan window cannot invalidate
  it.
- **`StepScratch` is caller-owned, one per thread.** It holds the rollout poses, the obstacle
  mask, distance-transform buffers and search queues. It grows on demand and never shrinks, so a
  steady-state cycle performs no heap allocation; a test counts allocations to enforce this.
- **Curves are non-owning inputs.** A curve is evaluated in its own frame through the
  `T_eval_curve` transform; obstacle queries go through `T_grid_eval`.
- **Commit before acting.** Callers store the new state and the command before they map a status
  to an exception or an action result.
- **Errors.** Constructors and factories throw `ConfigError`, which only `configure()` can see.
  `step`, field evaluation and the limiter never throw; problems come back as status bits.
- **Concurrency.** Const methods are safe to call concurrently. `ExpressionCurve` is
  non-copyable; each thread builds its own from the curve specification, and every muParser call
  is serialised and guarded.

In the ROS layer:

- The plugin runs on `controller_server`'s control thread. It copies the costmap under the
  costmap mutex once per cycle. Parameter changes rebuild the `Tracker` transactionally: a new
  one is built aside and swapped in only if construction succeeds, keeping the `TrackerState`.
- The follower's subscriptions swap immutable snapshots under a mutex; the action's execute loop
  runs at the follower's `controller_frequency`.
- The `FieldVisualizer` publishes from the control thread at the end of each cycle, with no
  timers. It does nothing when `visualize` is false or a topic has no subscribers.

## Further reading

- {doc}`design/index`: the mathematical specification, the upstream audit and the Nav2 API
  matrix.
- {doc}`visualization`: what each marker shows.
- {doc}`api`: the C++ API reference.
