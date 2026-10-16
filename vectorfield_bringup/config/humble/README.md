# config/humble

`collision_monitor.yaml` is the only distro-specific configuration file: on Humble the demos run
collision_monitor from `cmd_vel` to `cmd_vel_safe` with a footprint-approach polygon and the
laser scan as its source, because Humble's nav2_bringup does not include it in the command
chain.
