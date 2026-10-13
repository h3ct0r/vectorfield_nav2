# Obstacle avoidance

:::{note}
Planned for M2, with the scenario tests and demo configuration in M3. This page will explain
collision prediction and the braking cap, the blocked status and how Nav2 recovers from it, and
the optional obstacle contouring (`obstacle_mode: contour`): the path-blocked gate, the choice of
side, the contour distances $\lambda$, $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$, and the
watchdog. The underlying field is summarised in {doc}`theory`, and the RViz `contour` markers in
{doc}`visualization`.
:::
