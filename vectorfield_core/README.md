# vectorfield_core

The ROS-free C++17 core of [vectorfield_nav2](https://github.com/h3ct0r/vectorfield_nav2). It
implements the constructive time-varying artificial vector fields of Rezende, Gonçalves and
Pimenta (IEEE T-RO 38(2), 2022). The Nav2 controller plugin (`vectorfield_nav2_controller`), the
curve follower (`vectorfield_curve_follower`) and the Python bindings (`vectorfield_py`) all
call the same core code.

**Status:** complete (M1); used by every other package. Version 0.x; the C++ API is
experimental until 1.0.

## Features

- **Curves:** polylines with exact segment projection, parametric and muParser expression curves
  $r(s, t)$, and built-in shapes (ellipse, lemniscates, superellipse, Lissajous, rounded
  rectangle, regular polygon), with regularity and closure checks.
- **Fields:** the `atan` and `rational` gain functions, the static and time-varying distance
  field with the $\eta$ normalisation, and the terminal field that stops at the end of an open
  curve.
- **Tracking laws:** feedback linearisation, heading tracking (also for Ackermann) and holonomic,
  with the rotate-to-field, final-approach and rotate-to-heading phases.
- **Limits:** a kinematic box that scales the field, and the `CommandLimiter` (acceleration,
  Ackermann curvature, non-finite guard).
- **Obstacles:** an occupancy grid with a Euclidean distance and feature transform, footprint
  checks, collision prediction by rollout with a braking cap, and gated obstacle contouring.
- **`Tracker`:** one control cycle, `const noexcept`, with the cross-cycle state passed in and
  out explicitly and caller-owned scratch buffers, so a steady-state cycle does not allocate.
- **Validation:** `validateConfig()` and the per-plan advisor, with stable issue codes.
- **Simulation:** exact-arc integration of the four motion models, used by `vectorfield_sim` and
  the Python `ClosedLoopSimulator`.

The equations, guards and tolerances are specified in
[`docs/design/math_spec.md`](../docs/design/math_spec.md); the API reference is in the
[documentation](https://h3ct0r.github.io/vectorfield_nav2/api.html).

## ROS-free by design

- The only dependency is Eigen 3.4 or newer. Tests use GoogleTest; benchmarks use
  google-benchmark.
- muParser is vendored under `third_party/` and never exposed in public headers.
- No ROS header, type or library is used. `ament_cmake` is optional and only adds install and
  export rules.
- Everything is 2D (`Eigen::Vector2d` based). The core and Python APIs are experimental in 0.x.

## Build standalone (CMake presets)

Requires CMake 3.22 or newer, Ninja, Eigen and GoogleTest. On macOS:
`brew install cmake ninja eigen googletest` (add `google-benchmark` for the `bench` preset).

```bash
cd vectorfield_core
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
```

| Preset | Purpose |
|---|---|
| `dev` | Debug build with tests |
| `asan` | AddressSanitizer and UndefinedBehaviorSanitizer, with `clang++` from `PATH` |
| `tsan` | ThreadSanitizer, with `clang++` from `PATH` |
| `coverage` | `--coverage` instrumentation (GCC or clang) |
| `bench` | Release build with `VF_BUILD_BENCHMARKS=ON` |

Every preset has matching build and test presets. Build trees go to `build/<preset>/`, which is
gitignored. Each one gets `AMENT_IGNORE` and `COLCON_IGNORE` markers, so it is safe to build the
same checkout with colcon later. The standalone build defines no install rules.

| CMake option | Default | Effect |
|---|---|---|
| `BUILD_SHARED_LIBS` | `ON` | Shared core. The Python wheel build forces a static core. |
| `BUILD_TESTING` | `ON` | GoogleTest suite, registered with `gtest_discover_tests`. JUnit files go to `<build>/test_results/vectorfield_core/`. |
| `VF_WERROR` | `OFF` | Adds `-Werror` to vectorfield_core targets only, never to dependencies. |
| `VF_BUILD_BENCHMARKS` | `OFF` | google-benchmark targets `bench_field`, `bench_edt` and `bench_tracker` (non-gating). |
| `VF_BUILD_FUZZERS` | `OFF` | libFuzzer targets (LLVM clang only). Combine with the `asan` preset. |

## Build with colcon

```bash
colcon build --packages-up-to vectorfield_core
colcon test --packages-select vectorfield_core
colcon test-result --verbose
```

With `ament_cmake` available, the package installs its headers under
`include/vectorfield_core/` and exports the CMake target `vectorfield_core::vectorfield_core`,
with Eigen as an exported dependency. `colcon test` runs the GoogleTest suite and the ament
linters (copyright, lint_cmake, xmllint). The other linters are replaced by clang-format, ruff
and clang-tidy.

## Use from CMake

```cmake
find_package(vectorfield_core REQUIRED)
target_link_libraries(my_target PRIVATE vectorfield_core::vectorfield_core)
```

```cpp
#include "vectorfield_core/vectorfield_core.hpp"

namespace vf = vectorfield_core;

vf::CoreConfig config;                       // defaults; validated by the constructor
config.motion.motion_model = vf::MotionModel::kDifferential;
const vf::Tracker tracker(config);           // throws vf::ConfigError if invalid

vf::TrackerState state = vf::Tracker::initialState();
vf::StepScratch scratch;                     // one per thread, reused every cycle
vf::TrackerInput input;                      // pose, measured twist, curve, goal, obstacles, dt
// ... fill input each cycle ...
const vf::TrackerOutput output = tracker.step(input, state, &state, scratch);
// output.cmd is the body twist; output.phase and output.status explain it.
```

`vectorfield_core::version()` returns the version string.

`package.xml` is the only source of the version. CMake reads it before `project()` and writes
`vectorfield_core/version.hpp` (`VF_CORE_VERSION_MAJOR`, `_MINOR`, `_PATCH`, `_STR`) into the
build tree. The ament build installs that header.

## License

Apache-2.0. See the repository `LICENSE` and `NOTICE` files.
