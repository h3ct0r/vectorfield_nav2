# Visualization

The controller and the curve follower publish markers that let anyone watching RViz or Foxglove
see, without reading logs:

- which **phase** the robot is in, and why;
- what the **field** commands, split into the paper's components;
- where the robot is **heading**;
- what it **predicts**;
- what is **blocking** it.

:::{figure} _static/screenshots/demo.gif
:alt: RViz top view of a differential robot that follows a straight plan, circulates around a box that blocks it, rejoins the plan and stops at the goal.
:width: 100%

`kinematic_sim_demo` on Jazzy with the shipped `rviz/vectorfield.rviz`, 14 s: the robot follows
the plan (green status), meets an unmapped box on it, contours it on side −1 (`CONTOUR -1`, red
`contour` text and markers), rejoins the plan, brakes in FINAL_APPROACH (cyan) and stops; the
markers are cleared when the goal ends. The scenario is the one of {ref}`viz-contour`.
:::

## Topics

`<ns>` is the plugin name with `.` replaced by `/`, resolved in the `controller_server`
namespace: `FollowPath/markers` for the usual plugin name. The curve follower publishes the
same topics under its private namespace (`~/markers`, and so on), plus `~/curve`.

| Topic | Type | Content |
|---|---|---|
| `<ns>/markers` | `visualization_msgs/MarkerArray` | The marker namespaces in the legend below. |
| `<ns>/field_grid` | `visualization_msgs/MarkerArray` | The field on a grid around the robot; separate so that only its subscribers pay its cost (the shipped RViz and Foxglove layouts show it). |
| `<ns>/predicted_path` | `nav_msgs/Path` | The collision-prediction rollout. |
| `<ns>/window` | `nav_msgs/Path` | The conditioned path window the field is built on. |
| `<ns>/field_state` | `vectorfield_msgs/FieldState` | Every per-cycle value, for plotting. |
| `<ns>/obstacle_distance` | `nav_msgs/OccupancyGrid` | The obstacle distance transform, scaled 0–100 up to $D'_{\mathrm{in}}$ + 0.5 m. Only with `publish_obstacle_distance`. |
| `~/curve` (follower only) | `vectorfield_msgs/Curve` | The active curve, published transient local. |

Everything is gated by the `visualize` parameter (default `true`) **and** by each topic's
subscriber count, so nothing is computed when no viewer is connected. Markers are published from
the control thread at the end of each cycle, in the control frame (the costmap's global frame for
the plugin, `global_frame` for the follower) with the cycle's stamp. Marker ids are stable within
each namespace. A namespace that does not apply in a cycle is deleted, and all markers are
cleared when a goal ends, on reset and on deactivation, so stale markers never linger.

## Marker legend

Each namespace of `<ns>/markers` can be toggled in RViz. Colours are defined once and shared by
the code, the shipped RViz configuration and this table. None is white, which would vanish on
RViz's white map: the curve (teal), the closest point (amber), the resulting field (slate) and
the goal and `curve_info` (purple) keep a contrast of at least 2.5:1 on the white map and on a
dark background alike.

| Namespace | Marker(s) | Shows | Colour |
|---|---|---|---|
| `status` | TEXT_VIEW_FACING 0.6 m above `base_link`, one group per line: the phase (`FOLLOW`), the command (`v 0.38 w 0.12`, plus `vy` for omnidirectional robots), `D 0.04 m sigma 1.20 m`, and the active flags `BLOCKED`, `CONTOUR ±1`, `CBF`, `A3-CLAMP`, `CURV-SAT`, `CANCEL`, `REV` | phase, command, distance to the curve, distance to the goal, active conditions | text colour = phase: FOLLOW green, ROTATE_TO_FIELD amber, FINAL_APPROACH cyan, ROTATE_TO_HEADING blue, STOPPED red |
| `footprint` | LINE_STRIP at the current pose | robot outline | green normal / orange speed-capped / red blocked |
| `curve` | LINE_STRIP of the conditioned window (plugin) or the full curve (follower); LINE_LIST dashes for the extension rays; small SPHERE at the virtual tail vertex | exactly the geometry the field uses | teal; rays grey |
| `closest_point` | SPHERE at $x^*$, ARROW of $T$, LINE from the evaluation point to $x^*$ (length $D$) | projection, tangent, distance | amber |
| `control_point` | SPHERE at $p_c$ (feedback linearisation) or the reference point (Ackermann), LINE from `base_link` | where the field is evaluated | magenta |
| `field` | ARROWs at the evaluation point: convergence $-\eta G\, n$, circulation $\eta H\, T$, feedforward $\Phi_T$ (only when non-zero), resulting $F$ (thick) | the paper's decomposition of the field | red / green / blue / slate |
| `command` | ARROW from `base_link` along the commanded body velocity, plus a LINE_STRIP arc of 1 s of motion under the current $(v, \omega)$ | what is actually sent | cyan |
| `heading` | in rotate phases: ARROW of the target heading and an arc from the current heading to it | the ROTATE_TO_FIELD / ROTATE_TO_HEADING target | amber / blue |
| `prediction` | LINE_STRIP of the rollout poses (green up to the first collision), CUBE at the first colliding pose, TEXT `s_free 0.42 m` | collision prediction and braking cap | green → red |
| `goal` | SPHERE at the goal $g$, small SPHERE at the shifted goal $g_{ref}$ (when the reference point is offset), CYLINDER rings for the XY tolerance and $\sigma_{\mathrm{stop}}$, ARROW for the goal yaw, orange overlay of the last $\sigma_b$ of the curve; $g_{ref}$ and the overlay only once the curve ends at the goal (terminal mode), so a window that stops short of the goal shows neither; with `allow_reversing` the active cusp of the plan is the goal until the robot settles there | terminal field, braking zone, settle radius | purple / orange / cyan |
| `contour` | only while the gate is open or an episode is active: CUBE_LIST of the obstacle component's cells (translucent red); red POINTS at the path samples that tripped the gate; SPHERE at $x^*_o$ with a LINE to the evaluation point; CYLINDER rings at $\lambda$ (orange) and at $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ (amber); ARROW for the circulation side $\sigma_c$; TEXT `contour +1 \| 1.3/5.0 m \| 4/20 s` | why the contour engaged, around which obstacle, on which side, and the watchdog progress | red / orange / amber |
| `contour` with `obstacle_mode: cbf` | while the CBF filter binds or the nearest site is within $2\delta_{\mathrm{safe}}$: red POINTS at the constrained sites and a SPHERE at the nearest; LINE_LIST of the binding constraint normals to the evaluation point; CYLINDER ring of radius $\delta_{\mathrm{safe}}$ about the evaluation point (orange, red while $h < 0$); ARROW of the correction $F^\star - F$; TEXT `cbf h 0.12 m \| 1/2 bind \| 85 %` (`\| hold` while the hold sets `STATUS_BLOCKED`) | which obstacles constrain the field, how much margin is left and how much speed the filter removed | red / orange |
| `curve_info` (follower) | ARROWs every 1 m along the curve (traversal direction), SPHERE at the start, TEXT `lap 2/3 \| rem 7.4 m`; for moving frames, an ARROW of the frame velocity | direction, laps, moving-curve motion | purple / blue |

### Status flags

The flags in the `status` text correspond to the `FieldState` status bits:

| Flag | `FieldState` bit | Meaning |
|---|---|---|
| `BLOCKED` | `STATUS_BLOCKED` | The current pose collides, the free distance ahead is too short to keep moving, or the rotation in place would collide. The command decelerates to zero, and motion resumes by itself when the way clears. |
| `CONTOUR ±1` | — | An obstacle-contour episode is active, circulating on side +1 or −1. |
| `CBF` | — | The CBF filter binds: it changed the field to keep the margin (`obstacle_mode: cbf`). |
| `A3-CLAMP` | `STATUS_ASSUMPTION3_CLAMPED` | The curve moves too fast for the requested speed; the feedforward was clamped (Assumption 3, see {doc}`theory`). |
| `CURV-SAT` | `STATUS_CURVATURE_SATURATED` | The Ackermann turning-radius limit saturated the angular velocity. |
| `CANCEL` | — | A cancel is in progress; the robot is decelerating to a stop. |
| `REV` | — | A reverse plan segment (`allow_reversing`): the robot backs up, drive direction $-x$ ({ref}`reversing`). |

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

`vectorfield_bringup` ships:

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
- the red and green field-component arrows and the slate resulting field at the control point;
- the cyan command arrow;
- the green prediction line, which turns red ahead of a collision;
- the goal rings and the orange braking zone as the robot approaches the goal;
- the `field_grid` arrows (on by default; `field_grid_style: streamlines` draws streamlines),
  which show the whole field around the robot. The view orbits the robot (RViz's Orbit view
  on `base_link`, a perspective camera following `base_link` in Foxglove); switch RViz to
  TopDownOrtho for a map view.

During an obstacle-contour episode, the `contour` namespace shows the red gate samples, the
highlighted obstacle cells, the $\lambda$, $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ rings, the
circulation-side arrow and the watchdog text, and the status reads `CONTOUR +1` or `CONTOUR -1`.

## Screenshots

These are RViz 2 captures of `kinematic_sim_demo` (differential drive, `open_10x10`) on Jazzy,
with the shipped `rviz/vectorfield.rviz`; only the camera (zoom, centre and angle) differs. In
some views the camera is turned by 135°, so the robot drives to the right and the map's grid
appears diagonal. The status text sits above the robot whatever the camera: one line per group,
from the phase down to the flags.

### FOLLOW

:::{figure} _static/screenshots/follow.png
:alt: Robot to the left of a straight plan, with red, green and slate field arrows at its control point, the green status text above it and the purple goal on the right.
:width: 100%

FOLLOW (green status: `FOLLOW`, `v 0.38 w -0.59`, `D 0.41 m`). Goal (4, 0) from the origin;
while the robot drove, a simulated localization jump of 0.45 m to the side
(`/vectorfield_sim/step_map_to_odom`) moved it off the plan, so the convergence term is large
enough to see. At the magenta control point: the red convergence arrow toward the path, the
green circulation arrow along it and the slate resulting field. The cyan arrow is the command;
the amber sphere, line and arrow are the closest point, the distance $D$ and the tangent; the
teal line over the grey global plan is the window the field uses. On the right, the purple goal
with its two cyan rings and the cyan goal-yaw arrow; the window does not reach the goal yet, so
neither the braking zone nor $\sigma$ is shown. The thin green circle is Nav2's footprint
display, a little behind the jump.
:::

### ROTATE_TO_FIELD

:::{figure} _static/screenshots/rotate_to_field.png
:alt: Robot turning in place at the upper right, with an amber heading arrow and arc; the purple goal is at the lower left.
:width: 100%

ROTATE_TO_FIELD (amber status: `v 0.00 w -1.50`, `D 0.19 m`). The goal (−2.5, −2.5) lies
behind the robot, which starts at the origin facing +x. The robot turns in place: the amber
`heading` arrow is the target heading (the direction of the field at the control point, along
the slate resulting field) and the amber arc runs from the current heading to it. There is no
command arrow, because the linear speed is zero. The phase ends once the heading error is below
45°.
:::

### FINAL_APPROACH

:::{figure} _static/screenshots/final_approach.png
:alt: Robot on a straight plan just short of the goal, with the goal rings around the purple goal point and the cyan status text above the robot.
:width: 100%

FINAL_APPROACH (cyan status: `v 0.26 w -0.04`, `D 0.00 m sigma 0.32 m`). Start (1.8, −1.8)
facing the origin, goal at the origin (view turned by 135°). With 0.32 m of path left, the robot
tracks the terminal field at `base_link` (the slate arrow toward the goal) and brakes toward
`min_approach_speed`. The goal shows its XY-tolerance and settle rings
($\sigma_{\mathrm{stop}}$) and the cyan goal-yaw arrow.
:::

### ROTATE_TO_HEADING

:::{figure} _static/screenshots/rotate_to_heading.png
:alt: Robot on the goal turning in place, with a blue target-heading arrow and arc pointing down.
:width: 100%

ROTATE_TO_HEADING (blue status: `v 0.00 w -1.50`, `D 0.00 m sigma 0.12 m`). After the previous
goal, a second goal at the same position with a yaw 90° to the right. The robot is settled, so
it only turns in place: the blue arrow is the goal yaw and the blue arc the rotation still to
do; the cyan arrow is the goal-yaw marker of the `goal` namespace.
:::

(viz-contour)=

### Contour episode

:::{figure} _static/screenshots/contour.png
:alt: Robot circulating around a box that sits on its plan, with the obstacle cells highlighted, translucent distance discs and red contour text.
:width: 100%

A contour episode (`obstacle_mode: contour`). Launched with `scenarios:=demo_obstacles`, which
turns the global obstacle layer off (see {doc}`quickstart`), with a 0.4 m box set at run time on
the plan between the start (1.6, −1.6) and the goal (−1.6, 1.6) (view turned by 135°). The
status reads `CONTOUR -1`, and the red text `contour -1 | …/5.0 m | 2/20 s` is the watchdog
progress (path length and time of the episode against their limits). The `contour` namespace
shows the blocking obstacle's cells, the path samples that opened the gate (red), the closest
obstacle point (red sphere) with its line to the evaluation point, the $\lambda$ (orange),
$D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$ (amber) rings, drawn as translucent discs, and the
orange circulation-side arrow. Which side the episode takes depends on the robot's exact pose
when the gate opens: the demo GIF above circulates on the same side.
:::

### Blocked

:::{figure} _static/screenshots/blocked.png
:alt: Robot stopped in front of a wall across its plan, with a red footprint, a red collision cube and the BLOCKED flag in the status text.
:width: 100%

`BLOCKED` with `obstacle_mode: none`. A wall across the map (an unmapped box 13 m long) blocks
the plan, and the global obstacle layer is off, so the planner keeps the plan through it. The
robot has stopped with 0.09 m of free path left, less than `stop_margin`: the status reads
`v 0.00 w 0.00`, `D 0.01 m` and `BLOCKED` (`STATUS_BLOCKED` and `STATUS_COLLISION_AHEAD`), the
footprint is red, and the `prediction` namespace shows the colliding pose (red cube) and
`s_free 0.09 m`. Each blocked cycle raises `NoValidControl`, so Nav2 replans or recovers, and
the markers are cleared between attempts. With `obstacle_mode: contour` the gate opens here but
no episode starts, because neither side gets past the wall (the dead-end rule of
{doc}`obstacle_avoidance`).
:::
