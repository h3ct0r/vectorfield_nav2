# Security policy

## Supported versions

vectorfield_nav2 has not been released yet (milestone M0). Until 0.1.0, only the latest commit on `main` receives fixes.

| Version | ROS 2 distributions | Supported |
|---|---|---|
| `main` (0.0.x, unreleased) | Humble, Jazzy, Lyrical; Rolling best effort | Yes |
| Kilted builds | — | No (unsupported distribution) |

After 0.1.0, fixes go to the latest release on each supported distribution (Humble, Jazzy, Lyrical) until that distribution reaches end of life. See the support policy in [CONTRIBUTING.md](CONTRIBUTING.md#support-policy).

## Reporting a vulnerability

Please report vulnerabilities privately; do not open a public issue.

- **Preferred:** GitHub private vulnerability reporting, at [Security → Report a vulnerability](https://github.com/h3ct0r/vectorfield_nav2/security/advisories/new).
- **Email:** [hector@dcc.ufmg.br](mailto:hector@dcc.ufmg.br), with "vectorfield_nav2 security" in the subject.

Include the affected package and commit or version, the ROS 2 distribution, a description of the impact and, if possible, a minimal reproduction (for example the offending `FollowCurve` goal or parameter file). You will get an acknowledgement within 7 days. Fixes are coordinated with you and disclosed through a GitHub security advisory once a fix is available.

## Threat model and trust boundaries

vectorfield_nav2 is a navigation controller, not a safety-rated component. Keep it behind Nav2's velocity smoother and collision monitor (on Humble, the separately launched collision monitor), as the shipped bringup does, and rely on certified hardware for safety functions.

### The Nav2 controller plugin (planned for M2)

The plugin receives plans, costmaps and parameters through Nav2's `controller_server`. It trusts those inputs exactly as much as Nav2 does. Parameter updates are validated before they are applied; rejected updates leave the previous configuration in place.

### FollowCurve goals (planned for M5)

The `follow_curve` action server accepts curve definitions, including mathematical expressions, from any client on the ROS 2 graph. **Goal validation is the trust boundary.** Goals are accepted and then validated in full before any velocity command is published; an invalid goal ends with `INVALID_CURVE`, `INVALID_GOAL` or `UNSUPPORTED` and the robot does not move.

Expressions are evaluated with the vendored [muParser](https://github.com/beltoforion/muparser), never with a general-purpose interpreter (the ROS 1 predecessor evaluated expressions received over a topic with Python's `eval()`). The parser is restricted as follows (implemented in M1, `vectorfield_core` `ExpressionCurve`):

- `s` and `t` are the only variables, written immediately before each evaluation.
- Declared parameters are bound as constants, so an expression that assigns to them fails to parse.
- The function table is cleared and only muParser's deterministic built-in functions are defined again; `rnd` is not available.
- Undeclared names are errors (no variable factory).
- An expression must produce exactly one result, so comma-separated lists are rejected.
- Each expression is limited to 1024 characters, and a curve to 32 identifier-validated parameter names.
- Every evaluation is guarded (mutex, exception capture). A failing or non-finite evaluation produces NaN, which stops the robot and aborts the goal.
- The expression parser is fuzzed with a libFuzzer target in CI.

Size limits bound the CPU time and memory a single goal can consume: `max_curve_points` (default 20000) caps polyline and sampled curves, and the closest-point search works on a bounded window (`search_window`, `global_search_samples`). The curve is also checked for regularity and closure before it is used.

The follower refuses to drive while a Nav2 action (for example `follow_path` or `navigate_to_pose`) is active. This interlock prevents two controllers from commanding the robot; it is a safety measure, not an access control.

### Transport security

ROS 2 does not authenticate graph participants by default: anything that can reach the DDS domain can send `FollowCurve` goals, change parameters or publish velocity commands. On shared or untrusted networks:

- enable [SROS2](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html) (authentication, encryption and access control), and restrict which participants may call `follow_curve` and the parameter services;
- limit discovery to the host where possible (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on Jazzy and later, `ROS_LOCALHOST_ONLY=1` on Humble);
- do not expose foxglove_bridge or rosbridge ports beyond trusted networks.

## Third-party code

muParser is vendored unmodified at a pinned commit that includes the OSS-Fuzz fixes (planned for M1). Vulnerabilities in muParser itself should also be reported upstream; we will update the pinned copy.
