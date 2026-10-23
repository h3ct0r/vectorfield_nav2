^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectorfield_nav2_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-10-23)
------------------
* obstacle_mode cbf for Ackermann (core 0.3.0): layer-2 case of a box beside an Ackermann plan,
  passed with the margin.
* Obstacles on a reverse leg are contoured (the core no longer turns the contour off in reverse
  segments), and max_reverse_vel bounds the reverse speed. Layer-2 tests: a box on the reverse
  leg of a cusp plan (differential, Ackermann) and the reverse speed bound.
* A forward plan with a corner sharper than 90° (a NavigateThroughPoses waypoint the next leg
  leaves at 150°, or a sharp 90° corner the grid makes slightly sharper) is no longer rejected as
  reversing (INVALID_PATH) or, with allow_reversing, split and driven backward: intakePlan() uses
  vectorfield_core::findCusps() and takes a cusp work buffer. The window drops backtracks shorter
  than l_c/2 (vectorfield_core::despike()): at a sharp NavigateThroughPoses corner, NavFn's step
  back to the next leg's start cell held ROTATE_TO_FIELD there. A car still cannot take such a
  corner (HEADING_INFEASIBLE).

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
