# Motion models and phases

:::{note}
Planned for M2, with the controller plugin; the per-model configuration overlays follow in M3.
This page will cover the differential, skid-steer, omnidirectional and Ackermann models, which
tracking laws each allows (feedback linearisation, heading tracking, holonomic), and the
phases: rotate to field, follow, final approach, rotate to heading and stopped. It will also
cover the Ackermann caveats, including Nav2 recovery behaviour trees that use Spin, which
car-like robots cannot perform. The theory behind feedback linearisation is in {doc}`theory`.
:::
