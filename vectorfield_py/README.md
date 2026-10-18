# vectorfield_py

Python bindings for [vectorfield_nav2](https://github.com/h3ct0r/vectorfield_nav2)'s ROS-free
core. The package provides two Python packages:

- `vectorfield`: a pybind11 extension with NumPy I/O over `vectorfield_core`;
- `vectorfield_tools`: command-line tools for plotting, simulating and tuning fields.

The bindings call the same C++ code as the Nav2 controller plugin and the curve follower.

**Status: M4.** `vectorfield` binds the core API listed below; `vectorfield_tools` provides the
`vf-curve`, `vf-plot`, `vf-sim` and `vf-tune` command-line tools.

## Python API

Names are snake_case. Points are float64 arrays of shape (2,) or (N, 2), poses (N, 3)
`[x, y, yaw]`, grids (height, width) indexed `[iy, ix]`; every returned array is a copy.
Configuration and sample structs are mutable attribute objects with keyword constructors
(`vf.FieldConfig(desired_speed=0.3)`); a nested group is a live reference
(`config.motion.max_linear_vel = 0.8` edits `config`), and `copy()` is deep. Parameter enums
also accept their parameter strings (`config.field.gain_function = 'rational'`).

- Curves: `CurveSpec` (`polyline()`, `expression()`, `builtin()`), `make_curve(spec, options)`,
  `Curve.closest_point(x, context, hint)` and batch `closest_points()`, `PolylineCurve` with
  `SearchPolicy`, `builtin_shapes()`, `resample()`, `kappa_path()`.
- Fields and laws: `GainFunction`, `DistanceField` (`field_vectors()` for plots),
  `TerminalProfile`, `FieldSample`, `streamline()`, `Kinematics`, `ControlLaw` and the law
  helpers, `CommandLimiter`.
- Obstacles: `OccupancyGrid` (from a mask), `EuclideanDistanceTransform`, `ObstacleMap`,
  `Footprint`, `FootprintChecker`, `CollisionPredictor`.
- Configuration: `CoreConfig` and its groups, `validate_config(config, costmap_info=None)`
  (codes in `vectorfield.issue`).
- Controller: `Tracker(config)`, `Tracker.step(input, state) -> (output, next_state)`,
  `probe_points()` and `streamline()` for field plots, `TrackerInput`, `Phase`, `Status`.
- `vectorfield.sim`: `integrate()` and `ClosedLoopSimulator`, whose `run()` returns NumPy
  columns (t, pose, cmd, D, phase, ...).

Invalid configurations raise `vectorfield.ConfigError` (a `ValueError`); rejected curves raise
its subclass `CurveError`, whose `code` is `'INVALID_CURVE'` or `'UNSUPPORTED'`.

```python
import math
import vectorfield as vf

curve = vf.make_curve(vf.CurveSpec.builtin('ellipse', a=2.0, b=1.0))
config = vf.CoreConfig()
config.motion.motion_model = 'omnidirectional'
simulator = vf.sim.ClosedLoopSimulator(vf.Tracker(config))
simulator.reset((2.5, 0.0, math.pi / 2))
result = simulator.run(vf.TrackerInput(curve), 30.0)
print(abs(result.D[-100:]).max())
```

`ClosedLoopSimulator.run()` and the batch queries release the GIL; `Tracker.step()` does not.

## Command-line tools

They need the `tools` extra (matplotlib, Pillow and PyYAML, imported only when used) and print an
install hint without it. Every tool has `--help`; the exit status is 0 on success, 1 when an
optional dependency is missing and 2 for invalid input, curves or configurations.

- `vf-curve`: describe a curve (length, closure, regularity, `kappa_path`), plot it, and export
  it as an `x,y` CSV or as a `vectorfield_msgs/msg/Curve` YAML that `--curve` reads back.
- `vf-plot`: the field around a curve as arrows and streamlines; with obstacles (`--circle`,
  `--box`) or an open curve, the field the controller commands from `--pose`, contour included;
  `--decompose X,Y` draws and prints $-\eta G n$, $\eta H T$, $\Phi_T$ and $F$ at a point.
- `vf-sim`: a closed-loop simulation for `--model differential|skid_steer|omnidirectional|ackermann`
  with a summary (final and post-transient $|D|$, settle time, phases), a plot (`-o`), an animated
  GIF (`--gif`, at most 10 s at 12 fps by default) and the recorded columns (`--csv`).
- `vf-tune`: `validate_config()` on a parameter file (a Nav2 file with
  `controller_server.ros__parameters.FollowPath`, a node file or flat keys), with a suggested
  change per finding, the per-plan advice for `--curve`, and `--sweep NAME=START:STOP:STEP`
  simulations that report max $|D|$ and the settle time per value.

A curve is a built-in with optional parameters (`ellipse:a=2,b=1`), an expression pair in `s`
and `t` (`'2*cos(s); sin(s)'` with `--s-range`), or a CSV or YAML polyline file.

```bash
vf-sim --model differential --curve ellipse --gif ellipse.gif
vf-plot --curve lemniscate_bernoulli:a=2 -o field.svg
vf-tune nav2_params.yaml --curve path.csv --sweep convergence_gain=1:4:0.5
vf-curve --curve '2*cos(s); sin(s)' --export ellipse.yaml
```

## Install with pip (no ROS)

Build from a git checkout. The build compiles the sibling `vectorfield_core` directory, so only
in-tree builds are supported (no sdist or PyPI in 0.x). It needs a C++17 compiler, CMake 3.22 or
newer and Eigen 3.4 or newer (macOS: `brew install cmake eigen`). pip fetches scikit-build-core
and pybind11 for the build.

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install './vectorfield_py[tools]'  # without [tools]: the bindings only
pip install pytest
pytest vectorfield_py/test
python -c "import vectorfield; print(vectorfield.__version__)"
```

The wheel links a static, position-independent `vectorfield_core` into the extension and
contains only the `vectorfield/` and `vectorfield_tools/` packages, plus the `vf-*` entry
points. Optional extras are `tools` (matplotlib, Pillow and PyYAML) and `test` (pytest).

## Build with colcon

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to vectorfield_py
colcon test --packages-select vectorfield_py
colcon test-result --verbose
```

Under ament, the package links the installed shared `vectorfield_core` and installs both Python
packages and the extension into the Python install directory of the workspace. The CLIs run as
`ros2 run vectorfield_py vf-sim` (rosdep keys python3-matplotlib, python3-pil and python3-yaml).

## Versions

`package.xml` is the only source of the version. scikit-build-core reads it with its regex
metadata provider, and `ament_python_install_package` uses it for the package metadata.
`vectorfield.__version__` reports the compiled core's version, which is released in lockstep.

The bindings use only the pybind11 2.9 API, so they build against pybind11 2.9.1 (Humble/jammy)
up to 3.x (Homebrew, resolute). They need Python 3.10 or newer.

## License

Apache-2.0. See the repository `LICENSE` and `NOTICE` files.
