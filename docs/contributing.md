# Contributing

Contributions are welcome. The contribution guide,
[CONTRIBUTING.md](https://github.com/h3ct0r/vectorfield_nav2/blob/main/CONTRIBUTING.md), covers
the workflow, coding style, tests and review. Please also read the
[code of conduct](https://github.com/h3ct0r/vectorfield_nav2/blob/main/CODE_OF_CONDUCT.md), and
report vulnerabilities privately as described in
[SECURITY.md](https://github.com/h3ct0r/vectorfield_nav2/blob/main/SECURITY.md).
Bugs and feature requests go to the
[issue tracker](https://github.com/h3ct0r/vectorfield_nav2/issues).

Contributions are accepted under the Apache-2.0 license. New source files carry the license
header used throughout the repository, and the pre-commit hooks check formatting and linting
before each commit.

## Building this documentation

The documentation needs Python 3.12 or later, Doxygen and Graphviz, but no ROS installation.
From the repository root:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r docs/requirements.txt
(cd docs && doxygen Doxyfile)
sphinx-build -W --keep-going -b html docs docs/_build/html
```

`docs/gen_params.py` runs automatically at the start of the build and regenerates the
{doc}`parameters` pages from the parameter definitions.

The figures in `docs/_static/figures/` are committed, because the docs job does not build the
C++ bindings: after changing the field, the tracker or the plotting helpers, regenerate them with
`pip install './vectorfield_py[tools]' && python docs/gen_figures.py` and commit the result.

The RViz screenshots and the demo GIF in `docs/_static/screenshots/` are captures of
`kinematic_sim_demo` with the shipped `rviz/vectorfield.rviz`, taken on Jazzy under a virtual X
server (Xvfb) with software rendering. Retake them when the markers, their colours or the RViz
configuration change; {doc}`visualization` lists the scenario of each. Keep each PNG under
300 KB and the GIF under 3 MB. The pre-commit hooks skip both image directories.
