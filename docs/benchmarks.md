# Benchmarks

:::{note}
Benchmarks are informative and do not gate CI. The numbers below are reference values from one
machine; MarkerBuilder (M2) is added when `vectorfield_ros` lands.
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

## Reference numbers (M1)

Apple M3 Pro, Apple clang 21, Release; median of 3 repetitions.

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
| MarkerBuilder, full `markers` set | M2 | < 0.2 ms | — |

At the default 20 Hz control rate, the worst cycle (an episode start) uses under 0.3% of the
50 ms period.
