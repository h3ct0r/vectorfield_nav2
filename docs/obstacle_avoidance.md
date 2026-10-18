# Obstacle avoidance

:::{note}
Planned for M2, with the scenario tests and demo configuration in M3. This page will explain
collision prediction and the braking cap, the blocked status and how Nav2 recovers from it, and
the optional obstacle contouring (`obstacle_mode: contour`): the path-blocked gate, the choice of
side, the contour distances $\lambda$, $D_{\mathrm{in}}$ and $D'_{\mathrm{in}}$, and the
watchdog. The underlying field is summarised in {doc}`theory`, and the RViz `contour` markers in
{doc}`visualization`.
:::

:::{figure} _static/figures/contour_disc.svg
:alt: A robot on a straight path meets a disc, circulates below it at distance lambda, then returns to the path and stops at the goal.
:width: 100%

Contouring a disc that blocks a straight path (differential drive, `obstacle_mode: contour`,
defaults). Dashed: the contour distance $\lambda$; dotted: $D_{\mathrm{in}}$ and
$D'_{\mathrm{in}}$, between which the commanded field blends from the path field to the contour
field. Orange arrows show the contour field from the pose where the episode starts (black
outline); the circulation side is latched for the episode. Reproduce it with
`vf-sim --curve line.csv --pose -3,0,0 --circle 0,0.05,0.3 -o contour.svg`, where `line.csv` holds the two
vertices `-3,0` and `3,0`.
:::
