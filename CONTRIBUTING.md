# Contributing to vectorfield_nav2

Thank you for helping. This guide covers the development setup, style, tests, pull requests, releases and the support policy. By participating you agree to the [Code of Conduct](CODE_OF_CONDUCT.md). Security issues go through [SECURITY.md](SECURITY.md), not public issues.

The [documentation](https://h3ct0r.github.io/vectorfield_nav2/) describes the packages and their design. For anything larger than a small fix, please open an issue first so the design can be agreed before you write code.

## Development setup

### Devcontainers (ROS packages)

`.devcontainer/<distro>/` exists for `humble`, `jazzy`, `lyrical` and `rolling`. All use one shared Dockerfile, run natively on Apple Silicon (arm64) and on amd64, and keep `build/`, `install/`, `log/` and the ccache directory in named volumes so builds stay off the slow host bind mount.

1. Install Docker and VS Code with the Dev Containers extension (or the `devcontainer` CLI).
2. Run **Dev Containers: Reopen in Container** and pick a distribution.
3. In the container, from the workspace root (`/ws`):

   ```bash
   colcon build --symlink-install
   colcon test
   colcon test-result --verbose   # colcon test exits 0 even when tests fail
   ```

The Rolling container is based on the Nav2 nightly image and is for compiling and unit tests only; run demos in the Lyrical container. Do not build Nav2 from source under Docker Desktop.

To view RViz or Foxglove from macOS, use foxglove_bridge on port 8765 (forwarded by the devcontainers) or the optional noVNC service. Do not rely on DDS discovery between the macOS host and a container.

### Native core build (macOS or Linux, no ROS)

`vectorfield_core` is ROS-free and builds with plain CMake presets:

```bash
brew install cmake ninja eigen googletest google-benchmark   # macOS
cd vectorfield_core
cmake --preset dev && cmake --build --preset dev && ctest --preset dev
```

Presets: `dev`, `asan` (ASan + UBSan), `tsan`, `coverage` and `bench` (`-DVF_BUILD_BENCHMARKS=ON`). Preset build trees live under `vectorfield_core/build/` and are git-ignored.

The Python package builds the same way on the host:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install './vectorfield_py[tools]'
pytest vectorfield_py
```

### pre-commit

All formatting and most linting run through [pre-commit](https://pre-commit.com/):

```bash
pip install pre-commit       # or: brew install pre-commit
pre-commit install
pre-commit run --all-files
```

The last three hooks (`ament_copyright`, `ament_xmllint`, `ament_lint_cmake`) need a sourced ROS 2 environment and run in the devcontainers. On the host, skip them:

```bash
SKIP=ament_copyright,ament_xmllint,ament_lint_cmake pre-commit run --all-files
```

CI runs pre-commit with the same `SKIP`; `colcon test` runs those linters through `ament_lint_auto`. Files under `vectorfield_core/third_party/` are vendored and never reformatted.

## Coding style

### General

- Every source file starts with the Apache-2.0 header (`Copyright 2026 Héctor Azpúrua` for the original files; add your own copyright line to files you create). `ament_copyright` checks it.
- Namespaces equal package names, and macros use the `VF_` prefix.
- Distro-specific code lives only in `vectorfield_ros/compat.hpp`, the controller's API signature blocks, behind `VF_CANCEL_RESET_OVERRIDE`, and in the test server harness. `compat.hpp` detects the Nav2 API generation with `__has_include` probes, not version numbers.
- Logging: throttle messages inside loops to 1 s; use INFO only for phase transitions, limiter re-seeds and Assumption-3 on/off.

### C++

- C++17, and the code must also compile as C++20 (Lyrical and Rolling build with C++20).
- Format with clang-format using the ament/ROS 2 style in [`.clang-format`](.clang-format): Google base, 100 columns, braces on their own line after classes, functions, namespaces, structs and enums.
- [`.clang-tidy`](.clang-tidy) runs in the Jazzy CI job. Its findings are advisory, but please fix those in the code you touch.
- `vectorfield_core` stays ROS-free (Eigen and the vendored muParser only), allocates nothing in steady state and keeps `Tracker::step` and the other hot paths `noexcept`.
- `ament_lint_auto` excludes uncrustify, cpplint, flake8, pep257 and cppcheck; clang-format, clang-tidy and ruff replace them.

### Python

- Lint and format with [ruff](https://docs.astral.sh/ruff/) using [`ruff.toml`](ruff.toml): 99 columns, single quotes, isort-ordered imports, pep257 docstrings.
- Code must run on Python 3.10 (Humble).

### Markdown and spelling

markdownlint ([`.markdownlint.yaml`](.markdownlint.yaml)) and codespell ([`.codespellrc`](.codespellrc)) run in pre-commit. Prose is not hard-wrapped.

## Tests

Add or update tests with every behaviour change. The test layers are:

1. **Core** (`vectorfield_core`, gtest): analytic oracles, scenario tests, upstream-regression tests and a steady-state no-allocation test. Also run under the `asan` and `tsan` presets. Core line coverage target: at least 90%.
2. **Controller plugin, in process** (`vectorfield_nav2_controller`): a server harness that reproduces each distro's `controller_server` call order with the real goal checkers (and, on Lyrical and Rolling, the real path handler), plus parameter, visualization and parity tests.
3. **Curve follower, in process** (`vectorfield_curve_follower`): one case per failure mode, cancel, preemption and the Nav2 interlock.
4. **Launch tests** (`vectorfield_system_tests`, per PR): each distro and motion model on the loopback simulator (Jazzy and later) and on `vectorfield_sim`, obstacle and corridor scenarios, and validation of every shipped config.
5. **Nightly** (non-gating): Gazebo and noise, latency and slip injection. Registered only with `-DVF_NIGHTLY_TESTS=ON`.

Rules for ROS tests:

- Use `ament_add_ros_isolated_gtest` / `ament_add_ros_isolated_pytest`, and `add_launch_test(... RUNNER ${ament_cmake_ros_DIR}/run_test_isolated.py TIMEOUT 300)`.
- Never set `ROS_DOMAIN_ID`. Use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Jazzy and later) or `ROS_LOCALHOST_ONLY=1` (Humble).
- System tests run serially with no retries; a flaky test is a bug.

Run one package's tests with:

```bash
colcon test --packages-select <package> --event-handlers console_direct+
colcon test-result --verbose
```

## Pull requests

- Branch from `main` and keep each pull request focused on one change.
- Fill in the [pull request template](.github/PULL_REQUEST_TEMPLATE.md).
- Required checks: pre-commit, the Humble, Jazzy (amd64 and arm64) and Lyrical builds, the standalone core jobs (sanitizers, macOS) and the docs build. Rolling runs nightly and does not block merges; its failures become issues.
- Pull requests are **squash-merged**. The pull request title becomes the commit message and, through `catkin_generate_changelog`, the changelog entry, so write it for users. Conventional Commit style titles (`feat(controller): ...`, `fix(core): ...`) are welcome but optional.
- Say in the description which distros you built and tested, and whether the change touches a stable surface (see the support policy below).
- Contributions are accepted under the Apache License 2.0 (section 5 of the [LICENSE](LICENSE)).

## Changelog

Every released package has a `CHANGELOG.rst`. You do not need to edit it in a pull request: at release time a maintainer runs `catkin_generate_changelog`, which collects the commit titles since the last tag, and then edits the result.

## Releasing (maintainers)

1. `catkin_generate_changelog`, then edit each `CHANGELOG.rst`.
2. Set `version` in `CITATION.cff` to the new version (CI checks it on tags), and commit it together with the changelogs.
3. `catkin_prepare_release`, which bumps every `package.xml`, commits and tags. `package.xml` is the single source of version truth: the Python package and the core's generated `version.hpp` read it.
4. `bloom-release` into `ros2-gbp/vectorfield_nav2-release` for Humble, Jazzy and Lyrical. `vectorfield_system_tests` and `vectorfield_mujoco` are never released: list both, one per line, in the release repository's `humble.ignored`, `jazzy.ignored` and `lyrical.ignored` files (bloom's `<track>.ignored`, on its master branch). `vectorfield_mujoco` gets MuJoCo through the pip-only rosdep key `python3-mujoco-pip`, which bloom cannot turn into a Debian or RPM dependency. The build farm's `source:` (devel) and `doc:` jobs resolve every package's dependencies in the same way, so add those entries only once `vectorfield_mujoco` no longer stops them (for example with its pip dependency made conditional, REP 149), and release without them until then.

Humble and Jazzy are released for Ubuntu only. On Lyrical every package except `vectorfield_bringup` is released as RPM. There is no Rolling release until Nav2 is released into Rolling.

## Support policy

This project follows [REP 2004](https://ros.org/reps/rep-2004.html).

- **Distributions:** Humble, Jazzy and Lyrical are supported and gate every pull request. Rolling is tested nightly and is best effort. Kilted is unsupported: it shares Jazzy's API and may build, but it has no CI, config or release.
- **Stable surfaces:** the plugin type, parameter names and semantics, topic and action names, and message and action definitions. Once released into a distribution, they are frozen there.
- **Breaking changes** merge to `main` only after a maintenance branch (`humble`, `jazzy`, `lyrical`) has been cut from that distribution's last release tag. `main` then targets Rolling and future distributions, and fixes are cherry-picked back.
- **Messages** evolve by adding new types; existing definitions are not edited on a released distribution.
- A renamed parameter keeps a deprecated alias for one minor release.
- The C++ API of `vectorfield_core` and the `vectorfield` Python API are experimental during 0.x.
- A distribution is dropped in the first minor release after its end of life.
