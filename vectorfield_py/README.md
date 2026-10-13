# vectorfield_py

Python bindings for [vectorfield_nav2](https://github.com/h3ct0r/vectorfield_nav2)'s ROS-free
core. The package provides two Python packages:

- `vectorfield`: a pybind11 extension with NumPy I/O over `vectorfield_core`;
- `vectorfield_tools`: command-line tools for plotting, simulating and tuning fields.

The bindings call the same C++ code as the Nav2 controller plugin and the curve follower.

**Status: M0 skeleton.** `vectorfield` exposes only `__version__`, and `vectorfield_tools` is
empty. Both build paths and the tests already work. M4 adds:

- bindings for curves, gain functions, fields, kinematics, tracking laws, the `Tracker`, the
  command limiter, grids and the EDT, the collision predictor, the simulator and
  `validate_config`;
- the `vf-plot`, `vf-sim`, `vf-tune` and `vf-curve` CLIs.

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
