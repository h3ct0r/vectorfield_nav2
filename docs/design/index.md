# Design documents

The design documents record what the code implements and why. The code and its tests cite
them.

- **Mathematical specification**: every equation, symbol, guard and tolerance the core
  implements, with thesis equation numbers and the derivations this project adds.
- **Nav2 API matrix**: the controller-plugin interface on each supported distro and how the
  plugin adapts to it.
- **Upstream audit**: the defects found in the ROS 1 `vectorfield_stack` and how this
  implementation avoids each one.
- **Feature notes**: the 3D core and its node, the control-barrier-function mode, obstacle
  contouring for Ackermann bases, reversing, and the FollowCurve behaviour-tree node.

```{toctree}
:glob:
:maxdepth: 1

*
```
