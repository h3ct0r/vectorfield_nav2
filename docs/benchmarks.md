# Benchmarks

:::{note}
Benchmarks are informative and do not gate CI. The numbers below are reference values from one
machine.
:::

## Running them

```bash
cd vectorfield_core
cmake --preset bench
cmake --build --preset bench
build/bench/bench_field && build/bench/bench_edt && build/bench/bench_tracker
```

The `bench` preset is a Release build with `-DVF_BUILD_BENCHMARKS=ON` and needs Google Benchmark
(`brew install google-benchmark`, or `libbenchmark-dev` on Ubuntu).

The MarkerBuilder benchmark lives in `vectorfield_ros/bench/` and needs a ROS workspace. With
`libbenchmark-dev` installed:

```bash
colcon build --packages-up-to vectorfield_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DVF_BUILD_BENCHMARKS=ON
source install/setup.bash
build/vectorfield_ros/bench_marker_builder
```

## Core (`vectorfield_core`)

Apple M3 Pro, macOS, Apple clang 21, Release; median of 3 repetitions.

| Benchmark | What it measures | Target | Median |
|---|---|---|---|
| `BM_FieldEval_Polyline2k_Hinted` | closest point + field, 2000-vertex polyline, hint carried (follower) | < 5 µs | 0.15 µs |
| `BM_FieldEval_Polyline2k_GlobalStage` | same, invalid hint: scan of all pieces (first call of a goal) | — | 4.4 µs |
| `BM_FieldEval_Window100_WindowScan` | 100-vertex window with WindowScan, re-assigned every 100 calls (plugin) | — | 0.09 µs |
| `BM_FieldEval_BuiltinEllipse` | built-in ellipse, windowed golden section | — | 1.8 µs |
| `BM_Edt_Build/60` | distance + feature transform, 60 × 60 grid (stock local costmap) | < 100 µs | 17 µs |
| `BM_Edt_Build/200` | same, 200 × 200 grid | — | 248 µs |
| `BM_Edt_UpdateUnchanged` | version check that skips the rebuild | — | 1.6 ns |
| `BM_Edt_Query` | bilinear distance and normal at a point | — | 15 ns |
| `BM_TrackerStep_FreePath` | `Tracker::step`, 100-vertex window, 1.5 s rollout, contour on, gate closed | < 1 ms | 16 µs |
| `BM_TrackerStep_ActiveEpisode` | one cycle inside a contour episode (K flood fill, EDT_K, rollout) | < 1 ms | 41 µs |
| `BM_TrackerStep_EpisodeStart` | the cycle that starts an episode (adds the two side-probe rollouts) | reported | 134 µs |

At the default 20 Hz control rate, the worst cycle (an episode start) uses under 0.3% of the
50 ms period.

## Visualization (`vectorfield_ros`)

The same Apple M3 Pro, in a `ros:jazzy` container (Ubuntu 24.04, aarch64, GCC 13.3), Release;
median of 3 repetitions. The fixture is a 7 m straight window with a 0.3 m disc on it, stepped
until a contour episode is active, so every namespace of `markers`, `contour` included, is
populated.

| Benchmark | What it measures | Target | Median |
|---|---|---|---|
| `BM_Markers` | `MarkerBuilder::markers`: the full `<ns>/markers` array of one cycle | < 0.2 ms | 9.6 µs |
| `BM_FieldGrid/streamlines:0` | `field_grid` with the default 3.0 m × 0.25 m grid as arrows (12 × 12 nodes) | reported | 141 µs |
| `BM_FieldGrid/streamlines:1` | the same grid as streamlines | reported | 3.7 ms |

The marker array costs about 0.02% of a 50 ms cycle. The field grid is the expensive topic, which
is why it has its own topic, is off in the shipped RViz configuration, and is rate-limited by
`field_grid_rate` (2 Hz by default): with streamlines it adds about 7 ms per second of control
thread time while a viewer subscribes.
