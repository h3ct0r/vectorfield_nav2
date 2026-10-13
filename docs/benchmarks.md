# Benchmarks

:::{note}
Planned for M1, when the core benchmarks are built (`-DVF_BUILD_BENCHMARKS=ON`); reference
numbers measured on a Mac are published here in M6. Benchmarks are informative and do not gate
CI.
:::

The targets the benchmarks will be checked against:

| Benchmark | Target |
|---|---|
| Field evaluation on a 2000-point polyline | < 5 µs |
| Euclidean distance transform on a 60 × 60 grid | < 100 µs |
| `Tracker::step` (100-vertex window, 1.5 s horizon, contour on) | < 1 ms |
| Marker builder, full `markers` set (12 × 12 field grid reported separately) | < 0.2 ms |
| Obstacle-episode start (side probe) | reported separately |
