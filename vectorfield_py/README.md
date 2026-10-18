# vectorfield_py

Python bindings for [vectorfield_nav2](https://github.com/h3ct0r/vectorfield_nav2)'s ROS-free
core. The package provides two Python packages:

- `vectorfield`: a pybind11 extension with NumPy I/O over `vectorfield_core`;
- `vectorfield_tools`: command-line tools for plotting, simulating and tuning fields.

The bindings call the same C++ code as the Nav2 controller plugin and the curve follower.

**Status: M4.** `vectorfield` binds the core API listed below; the `vf-plot`, `vf-sim`,
`vf-tune` and `vf-curve` CLIs of `vectorfield_tools` are next.

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

## Install with pip (no ROS)

Build from a git checkout. The build compiles the sibling `vectorfield_core` directory, so only
in-tree builds are supported (no sdist or PyPI in 0.x). It needs a C++17 compiler, CMake 3.22 or
newer and Eigen 3.4 or newer (macOS: `brew install cmake eigen`). pip fetches scikit-build-core
and pybind11 for the build.

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install ./vectorfield_py            # add [tools] for matplotlib and Pillow (M4 CLIs)
pip install pytest
pytest vectorfield_py/test
python -c "import vectorfield; print(vectorfield.__version__)"
```

The wheel links a static, position-independent `vectorfield_core` into the extension and
contains only the `vectorfield/` and `vectorfield_tools/` packages. Optional extras are `tools`
(matplotlib and Pillow) and `test` (pytest).

## Build with colcon

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to vectorfield_py
colcon test --packages-select vectorfield_py
colcon test-result --verbose
```

Under ament, the package links the installed shared `vectorfield_core` and installs both Python
packages and the extension into the Python install directory of the workspace. From M4 the CLIs
also run as `ros2 run vectorfield_py vf-sim`.

## Versions

`package.xml` is the only source of the version. scikit-build-core reads it with its regex
metadata provider, and `ament_python_install_package` uses it for the package metadata.
`vectorfield.__version__` reports the compiled core's version, which is released in lockstep.

The bindings use only the pybind11 2.9 API, so they build against pybind11 2.9.1 (Humble/jammy)
up to 3.x (Homebrew, resolute). They need Python 3.10 or newer.

## License

Apache-2.0. See the repository `LICENSE` and `NOTICE` files.
