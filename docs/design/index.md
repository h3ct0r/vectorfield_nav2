# Design documents

These documents distil the research and review rounds that preceded the implementation. Later
work cites them, not the raw research.

- **Mathematical specification**: every equation, symbol, guard and tolerance the core
  implements, with thesis equation numbers and the derivations this project adds.
- **Nav2 API matrix**: the controller-plugin interface on each supported distro and how the
  plugin adapts to it.
- **Upstream audit**: the defects found in the ROS 1 `vectorfield_stack` and how this
  implementation avoids each one.

```{toctree}
:glob:
:maxdepth: 1

*
```
