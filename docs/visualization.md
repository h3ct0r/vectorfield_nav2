# Visualization

The controller and the curve follower publish markers that let anyone watching RViz or Foxglove
see, without reading logs:

- which **phase** the robot is in, and why;
- what the **field** commands, split into the paper's components;
- where the robot is **heading**;
- what it **predicts**;
- what is **blocking** it.

:::{note}
The markers ship with the controller in M2 and the curve follower in M5. Annotated RViz
screenshots of each phase and a demo GIF are planned for M6.
:::

## Topics

`<ns>` is the plugin name with `.` replaced by `/`, resolved in the `controller_server`
namespace: `FollowPath/markers` for the usual plugin name. The curve follower publishes the
same topics under its private namespace (`~/markers`, and so on), plus `~/curve`.

| Topic | Type | Content |
|---|---|---|
| `<ns>/markers` | `visualization_msgs/MarkerArray` | The marker namespaces in the legend below. |
| `<ns>/field_grid` | `visualization_msgs/MarkerArray` | The field on a grid around the robot; separate so its cost is opt-in. |
| `<ns>/predicted_path` | `nav_msgs/Path` | The collision-prediction rollout. |
| `<ns>/window` | `nav_msgs/Path` | The conditioned path window the field is built on. |
| `<ns>/field_state` | `vectorfield_msgs/FieldState` | Every per-cycle value, for plotting. |
| `<ns>/obstacle_distance` | `nav_msgs/OccupancyGrid` | The obstacle distance transform, scaled 0–100 up to $D'_{\mathrm{in}}$ + 0.5 m. Only with `publish_obstacle_distance`. |
| `~/curve` (follower only) | defined in M5 | The active curve, published transient local. |

Everything is gated by the `visualize` parameter (default `true`) **and** by each topic's
subscriber count, so nothing is computed when no viewer is connected. Markers are published from
the control thread at the end of each cycle, in the control frame (the costmap's global frame for
the plugin, `global_frame` for the follower) with the cycle's stamp. Marker ids are stable within
each namespace. A namespace that does not apply in a cycle is deleted, and all markers are
cleared when a goal ends, on reset and on deactivation, so stale markers never linger.

## Marker legend

Each namespace of `<ns>/markers` can be toggled in RViz. Colours are defined once and shared by
the code, the shipped RViz configuration and this table.

| Namespace | Marker(s) | Shows | Colour |
|---|---|---|---|
| `status` | TEXT_VIEW_FACING 0.6 m above `base_link`, e.g. `FOLLOW \| v 0.38 w 0.12 \| D 0.04 m \| sigma 1.20 m`, plus the active flags `BLOCKED`, `CONTOUR ±1`, `A3-CLAMP`, `CURV-SAT`, `CANCEL` | phase, command, distance to the curve, distance to the goal, active conditions | text colour = phase: FOLLOW green, ROTATE_TO_FIELD yellow, FINAL_APPROACH cyan, ROTATE_TO_HEADING blue, STOPPED red |
| `footprint` | LINE_STRIP at the current pose | robot outline | green normal / orange speed-capped / red blocked |
| `curve` | LINE_STRIP of the conditioned window (plugin) or the full curve (follower); LINE_LIST dashes for the extension rays; small SPHERE at the virtual tail vertex | exactly the geometry the field uses | white; rays grey |
| `closest_point` | SPHERE at $x^*$, ARROW of $T$, LINE from the evaluation point to $x^*$ (length $D$) | projection, tangent, distance | yellow |
| `control_point` | SPHERE at $p_c$ (feedback linearisation) or the reference point (Ackermann), LINE from `base_link` | where the field is evaluated | magenta |
| `field` | ARROWs at the evaluation point: convergence $-\eta G\, n$, circulation $\eta H\, T$, feedforward $\Phi_T$ (only when non-zero), resulting $F$ (thick) | the paper's decomposition of the field | red / green / blue / white |
| `command` | ARROW from `base_link` along the commanded body velocity, plus a LINE_STRIP arc of 1 s of motion under the current $(v, \omega)$ | what is actually sent | cyan |
| `heading` | in rotate phases: ARROW of the target heading and an arc from the current heading to it | the ROTATE_TO_FIELD / ROTATE_TO_HEADING target | yellow / blue |
| `prediction` | LINE_STRIP of the rollout poses (green up to the first collision), CUBE at the first colliding pose, TEXT `s_free 0.42 m` | collision prediction and braking cap | green → red |
| `goal` | SPHERE at the goal $g$, small SPHERE at the shifted goal $g_{ref}$ (when the reference point is offset), CYLINDER rings for the XY tolerance and $\sigma_{\mathrm{stop}}$, ARROW for the goal yaw, orange overlay of the last $\sigma_b$ of the curve | terminal field, braking zone, settle radius | white / orange / cyan |
| `contour` | only while the gate is open or an episode is active: CUBE_LIST of the obstacle component's cells (translucent red); red POINTS at the path samples that tripped the gate; SPHERE at $x^*_o$ with a LINE to the evaluation point; CYLINDER rings at $\lambda$ (orange) and at $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ (yellow); ARROW for the circulation side $\sigma_c$; TEXT `contour +1 \| 1.3/5.0 m \| 4/20 s` | why the contour engaged, around which obstacle, on which side, and the watchdog progress | red / orange / yellow |
| `curve_info` (follower) | ARROWs every 1 m along the curve (traversal direction), SPHERE at the start, TEXT `lap 2/3 \| rem 7.4 m`; for moving frames, an ARROW of the frame velocity | direction, laps, moving-curve motion | white / blue |

### Status flags

The flags in the `status` text correspond to the `FieldState` status bits:

| Flag | `FieldState` bit | Meaning |
|---|---|---|
| `BLOCKED` | `STATUS_BLOCKED` | The current pose collides, the free distance ahead is too short to keep moving, or the rotation in place would collide. The command decelerates to zero, and motion resumes by itself when the way clears. |
| `CONTOUR ±1` | — | An obstacle-contour episode is active, circulating on side +1 or −1. |
| `A3-CLAMP` | `STATUS_ASSUMPTION3_CLAMPED` | The curve moves too fast for the requested speed; the feedforward was clamped (Assumption 3, see {doc}`theory`). |
| `CURV-SAT` | `STATUS_CURVATURE_SATURATED` | The Ackermann turning-radius limit saturated the angular velocity. |
| `CANCEL` | — | A cancel is in progress; the robot is decelerating to a stop. |

`FieldState.status` also carries `STATUS_NON_FINITE`, `STATUS_AMBIGUOUS_CLOSEST`,
`STATUS_HAIRPIN`, `STATUS_CONTOUR_WATCHDOG`, `STATUS_HEADING_INFEASIBLE` and
`STATUS_COLLISION_AHEAD`; plot it from `<ns>/field_state`.

## Field grid

`<ns>/field_grid` shows the field on a `field_grid_size` square around the robot, at
`field_grid_resolution`, published at `field_grid_rate`:

- `field_grid_style: arrows` draws one ARROW per cell, with length proportional to $\lVert F\rVert$
  and colour by distance to the curve (viridis);
- `field_grid_style: streamlines` draws LINE_STRIPs integrated through the field.

The grid probes the same field the controller used in that cycle, with the same closest-point
hint, obstacle component and gate, so it includes any active contour.

## Shipped viewer configurations

`vectorfield_bringup` ships, with the M3 demos:

- `rviz/vectorfield.rviz`: the map, both costmaps, the global plan, the robot model and TF,
  every `markers` namespace, `field_grid`, `window` and `predicted_path`, coloured per the legend.
  Every demo launch opens it (`use_rviz:=true` by default).
- `foxglove/vectorfield.json`: a 3D panel with the same topics, plots of $D$, $\sigma$, $v$,
  $\omega$ and the limiter scale, and a state-transition panel for the phase and status from
  `field_state`.
- `plotjuggler/vectorfield.xml`: the same plots.

## What to look for

In a running demo:

- the status text, whose colour changes with the phase;
- the red, green and white field-component arrows at the control point;
- the cyan command arrow;
- the green prediction line, which turns red ahead of a collision;
- the goal rings and the orange braking zone as the robot approaches the goal;
- the `field_grid` arrows (toggle them on), which show the whole field around the robot.

During an obstacle-contour episode, the `contour` namespace shows the red gate samples, the
highlighted obstacle cells, the $\lambda$, $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ rings, the
circulation-side arrow and the watchdog text, and the status reads `CONTOUR +1` or `CONTOUR -1`.
