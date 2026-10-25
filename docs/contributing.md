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

The RViz phase clips of {doc}`visualization` (`docs/_static/media/viz_*`, with their PNG
posters in `docs/_static/screenshots/`) are recordings of `kinematic_sim_demo` with the shipped
`rviz/vectorfield.rviz`, made on Jazzy under a virtual X server (Xvfb) with software rendering.
Record them again with `docs/tools/record_rviz_clips.sh` in the Jazzy devcontainer (with
`apt install xvfb`) when the markers, their colours or the RViz configuration change; the
script runs the scenario of each phase and writes the MP4, GIF and poster. Keep each MP4 under
3 MiB, each GIF under 2 MiB and each PNG under 300 KB, and the demo GIF under 3 MB. The
pre-commit hooks skip the screenshots directory.

The gallery clips in `docs/_static/media/` (the README's GIFs and the {doc}`examples/index`
videos) are recordings of the MuJoCo examples. Record them again with
`vectorfield_mujoco/scripts/record_gallery.sh` in the Jazzy devcontainer, with the workspace
built and sourced, after a change to an example, its scenario or the markers; it records each
example headless to the end of its scenario. Each MP4 stays under 3 MiB, each GIF under 2 MiB
and each poster JPEG under 150 KiB: the script fails over budget, and the pre-commit hooks
reject files over 3 MB. The `clip` directive (`docs/vf_clips.py`) embeds them; a clip whose
files are missing fails the docs build.
