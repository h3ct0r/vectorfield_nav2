# maps

- `open_10x10`: an open 10 m × 10 m area (origin at its centre) for the per-model route tests;
- `corridor_door`: a 2 m × 12 m corridor with a 0.8 m door at x = 5 m. The door is wide enough
  for the TB3 (0.22 m radius) but narrower than twice the auto contour distance (0.47 m), so it
  checks that obstacle contouring does not trigger in narrow but free passages.

The robot starts at (0, 0) on both. map_server loads them under `lifecycle_manager_map_server`
with autostart.
