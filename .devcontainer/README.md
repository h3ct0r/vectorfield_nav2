# Development containers

One container per supported ROS 2 distribution, built natively for Apple Silicon
(arm64) and usable on any Linux host.

| Folder | Base image | Use it for |
|---|---|---|
| `humble/` | `ros:humble-ros-base-jammy` + Nav2 (apt) | build, tests, `kinematic_sim_demo` |
| `jazzy/` | `ros:jazzy-ros-base-noble` + Nav2, loopback sim, TB3 sim | build, tests, loopback and kinematic-sim demos |
| `lyrical/` | `ros:lyrical-ros-base-resolute` + Nav2, loopback sim, TB3 sim | build, tests, loopback and kinematic-sim demos |
| `rolling/` | `ghcr.io/ros-navigation/nav2_docker:rolling-nightly-standard-arm64` | compile and unit tests only (see below) |

Humble, Jazzy and Lyrical share `Dockerfile` (build arguments `BASE`,
`ROS_DISTRO`, `EXTRA_APT`); Rolling has its own `rolling/Dockerfile`. Each image
also has foxglove_bridge, RViz (not Rolling), generate_parameter_library,
ccache, gdb, clang/clang-tidy, lcov, GoogleTest and Google Benchmark.

## Open a container

VS Code with the Dev Containers extension: open the repository folder, run
**Dev Containers: Reopen in Container** and pick a distribution.

Command line ([devcontainer CLI](https://github.com/devcontainers/cli)):

```bash
devcontainer up   --workspace-folder . --config .devcontainer/jazzy/devcontainer.json
devcontainer exec --workspace-folder . --config .devcontainer/jazzy/devcontainer.json bash
```

The CLI does not forward ports; use VS Code (or the noVNC desktop below) for GUIs.

Layout inside the container:

- the repository is bind-mounted at `/ws/src/vectorfield_nav2`;
- `/ws/build`, `/ws/install`, `/ws/log` and `/ccache` are named volumes per
  distribution (`vf-<distro>-build`, `vf-<distro>-install`, `vf-<distro>-log`,
  `vf-<distro>-ccache`), so build output stays off the slow macOS bind mount
  and survives rebuilds of the container;
- `postCreateCommand` runs `rosdep install` for every package in the repository.

```bash
cd /ws
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

To start from scratch, delete the volumes, for example
`docker volume rm vf-jazzy-build vf-jazzy-install vf-jazzy-log`.

Builds use ccache through `CMAKE_<LANG>_COMPILER_LAUNCHER`, and every package
writes `compile_commands.json`. clangd reads `/ws/build/compile_commands.json`;
merge the per-package files after a build with:

```bash
python3 -c "import glob, json; json.dump(sum((json.load(open(f)) for f in glob.glob('/ws/build/*/compile_commands.json')), []), open('/ws/build/compile_commands.json', 'w'))"
```

If a build runs out of memory, raise Docker Desktop's memory limit or build
with `colcon build --parallel-workers 2` and `MAKEFLAGS=-j2`.

### ROS networking

`ROS_DOMAIN_ID` is deliberately unset: the isolated test runner
(`run_test_isolated.py`) gives each test its own domain only when it is unset.
Discovery is limited to the container (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
on Jazzy and later, `ROS_LOCALHOST_ONLY=1` on Humble). DDS discovery between
macOS and a container does not work under Docker Desktop, so GUIs connect
through Foxglove's WebSocket or run inside the container (noVNC).

## Seeing what the robot is doing

The controller and the curve follower publish their state as RViz markers
(`<ns>/markers`: phase and status text, footprint, curve, closest point,
control point, field components, command, heading target, collision
prediction, goal and braking zone, contour episode) and the field around the robot (`<ns>/field_grid`); see
the visualization guide in the docs, and the pre-configured `rviz/vectorfield.rviz` and
`foxglove/vectorfield.json` in `vectorfield_bringup`.

### Foxglove (recommended on macOS)

1. In the container:
   `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`.
2. VS Code forwards port 8765. In the Foxglove desktop app, open a connection
   to `ws://localhost:8765`.
3. Add a 3D panel and enable the `markers` and `field_grid` topics.

[Lichtblick](https://github.com/lichtblick-suite/lichtblick), an open-source
fork, connects the same way.

### RViz in the browser (noVNC)

`compose.novnc.yml` starts a small X server with noVNC. The humble, jazzy and
lyrical containers mount its X11 socket (volume `vf-x11`), so RViz runs in the
dev container, on the same ROS graph, and is shown in the browser:

```bash
# on macOS, from the repository root
docker compose -f .devcontainer/compose.novnc.yml up -d --build
open 'http://localhost:6080/vnc.html?autoconnect=1&resize=remote'

# in the dev container
export DISPLAY=:1
rviz2
```

Rendering is software OpenGL (there is no GPU in Docker Desktop), so expect a
modest frame rate; if RViz reports GLX errors, also set `LIBGL_ALWAYS_SOFTWARE=1`.
The page has no password and is published on `127.0.0.1` only. Stop it with
`docker compose -f .devcontainer/compose.novnc.yml down`.

## Rolling limitations

- Nav2 is not released into Rolling. The container uses the Nav2 nightly arm64
  image (Nav2 `main`), which has no `nav2_bringup`, simulators or RViz. Its
  Dockerfile installs Nav2's own dependencies with rosdep, and the shell sources
  `/root/nav2_ws/install`.
- Only compile and unit tests are supported:

  ```bash
  cd /ws
  colcon build --packages-up-to vectorfield_nav2
  colcon test --packages-up-to vectorfield_nav2 --ctest-args -LE launch_test
  colcon test-result --verbose
  ```

  `vectorfield_bringup` and `vectorfield_system_tests` need `nav2_bringup` and
  the simulators; run demos and system tests in the Lyrical container, which
  has the same Nav2 API.
- To pick up a newer Nav2 nightly, rebuild without cache
  (**Dev Containers: Rebuild Without Cache**).
- Never build Nav2 from source under Docker Desktop.

## Without a container

The ROS-free core and the Python package build natively on macOS with Homebrew
(`brew install eigen googletest google-benchmark pybind11 ninja`); see the
top-level README.
