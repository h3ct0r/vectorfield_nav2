# vectorfield_core

The ROS-free C++17 core of [vectorfield_nav2](https://github.com/h3ct0r/vectorfield_nav2). It
implements the constructive time-varying artificial vector fields of Rezende, Gonçalves and
Pimenta (IEEE T-RO 38(2), 2022). The Nav2 controller plugin (`vectorfield_nav2_controller`), the
curve follower (`vectorfield_curve_follower`) and the Python bindings (`vectorfield_py`) all
call the same core code.

**Status: M0 skeleton.** The library exposes only its version. The build, test, sanitizer,
benchmark and fuzzer mechanics are complete, so M1 adds sources only. M1 adds:

- curves: polylines, parametric and expression curves, built-in shapes;
- gain functions and the distance and terminal fields;
- obstacle contouring on an occupancy grid with a Euclidean distance transform;
- footprint checks and collision prediction;
- tracking laws, the command limiter, the `Tracker` and the kinematic simulator.

The design is in [`docs/design/math_spec.md`](../docs/design/math_spec.md).

## ROS-free by design

- The only dependency is Eigen 3.4 or newer. Tests use GoogleTest; benchmarks use
  google-benchmark.
- muParser is vendored under `third_party/` (from M1) and never exposed in public headers.
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
| `VF_BUILD_BENCHMARKS` | `OFF` | google-benchmark targets (non-gating; added in M1). |
| `VF_BUILD_FUZZERS` | `OFF` | libFuzzer targets (LLVM clang only; added in M1). Combine with the `asan` preset. |

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

static_assert(vectorfield_core::versionAtLeast(0, 0, 1));
const char * v = vectorfield_core::version();  // "MAJOR.MINOR.PATCH" from package.xml
```

`package.xml` is the only source of the version. CMake reads it before `project()` and writes
`vectorfield_core/version.hpp` (`VF_CORE_VERSION_MAJOR`, `_MINOR`, `_PATCH`, `_STR`) into the
build tree. The ament build installs that header.

## License

Apache-2.0. See the repository `LICENSE` and `NOTICE` files.
