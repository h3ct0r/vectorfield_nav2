^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_nav2_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-20)
------------------
* Plans with reversing segments are followed when allow_reversing is true: the plan is split at its
  cusps, the window ends at the next cusp, and the robot stops there before driving the next
  segment.
* Lyrical: plans with cusps or in-place rotations use the plugin's own window, so
  enforce_path_inversion and enforce_path_rotation never stall the robot.

0.1.1 (2026-10-20)
------------------
* The stop-shift-rotate test bounds the travel during a rotation by the braking distance.

0.1.0 (2026-10-19)
------------------
* First release of the nav2_core::Controller plugin VectorFieldController for Humble, Jazzy and
  Lyrical (setPlan on Humble-Kilted, the path handler on Lyrical and later).
* LegacyPathHandler: a port of Nav2's FeasiblePathHandler for Humble-Kilted, parity-tested against
  Nav2 on Lyrical.
* Plan intake (goal identity, end tangent, cusp rejection, tail conditioning), cancel and reset,
  speed limits, dynamic parameters, footprint re-check and visualization.
* Hold the last command when no time has passed since the previous cycle.
