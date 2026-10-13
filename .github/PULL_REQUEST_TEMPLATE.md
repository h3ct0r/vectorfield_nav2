## Summary

<!-- What does this change do, and why? Link the issue it closes (e.g. "Closes #12"). -->

## Distro impact

<!-- Which distros did you build and test on? Does this touch compat.hpp or other distro-specific code? -->

- [ ] Humble
- [ ] Jazzy
- [ ] Lyrical
- [ ] Rolling (optional, nightly only)

## Checklist

- [ ] Tests added or updated, and `colcon test` / `colcon test-result --verbose` pass (or `ctest --preset dev` for core-only changes).
- [ ] Documentation updated (README, `docs/`, parameter descriptions) where behaviour or parameters changed.
- [ ] The pull request title is a user-facing changelog line (it becomes the squash commit and the `CHANGELOG.rst` entry).
- [ ] `pre-commit run --all-files` passes.
- [ ] No stable surface changed (plugin type, parameter names or semantics, topics, actions, messages), or the change follows the support policy in CONTRIBUTING.md.
- [ ] New source files carry the Apache-2.0 license header.
