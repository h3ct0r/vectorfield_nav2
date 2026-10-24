# Curve following: AgileX Scout Mini

```{clip} curve_following
:alt: AgileX Scout Mini in MuJoCo laps a lemniscate with the curve follower
:controls:
```

The Scout Mini drives laps of a lemniscate, a figure eight that crosses itself, with the curve
follower. Nav2 cannot express a closed curve as a path; the `follow_curve` action follows it
for any number of laps.

```bash
ros2 launch vectorfield_mujoco curve_following.launch.py
```

```{list-table}
:class: vf-facts

* - Mode
  - `motion_model: skid_steer`, feedback linearisation with slip compensation
* - Robot
  - `scout_mini`: the vendored AgileX Scout Mini (no download), as in {doc}`skid_steer`
* - Driven by
  - the curve follower's `follow_curve` action; Nav2 runs for the approach, the local costmap and the command chain
* - World
  - `open_10x10`
* - Files
  - [`launch/curve_following.launch.py`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/launch/curve_following.launch.py), [`config/robots/scout_mini.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/robots/scout_mini.yaml), [`config/scenarios/curve_following.yaml`](https://github.com/h3ct0r/vectorfield_nav2/blob/main/vectorfield_mujoco/config/scenarios/curve_following.yaml)
```

## What you see

The robot starts at (0, −1.5). `run_scenario` first drives it onto the curve with a
NavigateToPose goal, then sends the FollowCurve goal: two laps of the lemniscate of Bernoulli
with $a$ = 2 m, which crosses itself at the origin. At the crossing two branches are equally
close; the follower keeps the branch it is on by searching for the closest point only near the
previous one ({doc}`../theory`). The goal succeeds when the second lap is done.

- [Teal line]{.vf-key .vf-teal}: the curve being followed.
- [Purple arrows and sphere]{.vf-key .vf-purple}: the direction of travel, every metre, and the curve's start.
- [Green rectangle]{.vf-key .vf-green}: the footprint; orange while the speed is capped, red when blocked.
- [Green line ahead of the robot]{.vf-key .vf-green}: the predicted motion, checked for collisions.
- [Slate and cyan arrows]{.vf-key .vf-slate}: the field $F$ the robot tracks and the command sent.

## Configuration

The robot overlay is the one of {doc}`skid_steer`, slip compensation included. The scenario
holds only the start pose, the curve and its laps.

::::{tab-set}

:::{tab-item} Scenario

```{literalinclude} ../../vectorfield_mujoco/config/scenarios/curve_following.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/scenarios/curve_following.yaml
```

:::

:::{tab-item} Robot

```{literalinclude} ../../vectorfield_mujoco/config/robots/scout_mini.yaml
:language: yaml
:lines: 15-
:caption: vectorfield_mujoco/config/robots/scout_mini.yaml
```

:::
::::

## Things to try

- Other shapes, or laps until Ctrl-C (`--laps 0`, which cancels the goal):

  ```bash
  ros2 launch vectorfield_mujoco curve_following.launch.py scenario:=false
  ros2 run vectorfield_bringup send_curve.py --shape ellipse --laps 0
  ```

- Another robot on the same curve, with the generic launch:
  `ros2 launch vectorfield_mujoco mujoco_curve_follower.launch.py robot:=tiago scenario:=curve_following`.
- A NavigateToPose goal (RViz's **2D Goal Pose**) while the robot laps: the follower stops and
  ends its goal with `NAV2_ACTIVE`, so the two never command the base together.

## Adapt it to your robot

The curve follower runs next to Nav2 on a real robot too: it uses the local costmap for
collision prediction and the same command chain. {doc}`../tutorials/curve_following` goes from
the demo to your own `FollowCurve` client, with polylines, expressions and moving frames;
{doc}`../curve_follower` documents the action and its parameters.

## Troubleshooting

- **The goal ends with `NAV2_ACTIVE`**: a Nav2 action was running when the goal started, or
  started during it ({doc}`../curve_follower`).
- **The robot runs outside the curve in the turns**: the slip compensation does not match the
  model; see {doc}`skid_steer`.
- **Nothing happens for a while**: the scenario waits for Nav2 and the follower to become
  active.
