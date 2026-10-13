# maps

Maps arrive in M3:

- `open_10x10`: an open 10 m × 10 m area for the per-model route tests;
- `corridor_door`: a corridor with a door, used to check that obstacle contouring does not
  trigger in narrow but free passages.

map_server loads them under `lifecycle_manager_map_server` with autostart.
