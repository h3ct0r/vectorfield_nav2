# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sphinx configuration for the vectorfield_nav2 documentation site.

Build (from the repository root):

    (cd docs && doxygen Doxyfile)
    sphinx-build -W --keep-going -b html docs docs/_build/html
"""

import re
import sys
from pathlib import Path

from sphinx.util import logging

DOCS_DIR = Path(__file__).resolve().parent
REPO_ROOT = DOCS_DIR.parent
PACKAGE_XML = REPO_ROOT / 'vectorfield_nav2' / 'package.xml'

sys.path.insert(0, str(DOCS_DIR))
import gen_params  # noqa: E402

logger = logging.getLogger(__name__)


def _read_version():
    """Return the metapackage version; package.xml is the single source of truth."""
    try:
        text = PACKAGE_XML.read_text(encoding='utf-8')
    except OSError as exc:
        logger.warning('cannot read the docs version from %s: %s', PACKAGE_XML, exc)
        return '0.0.0+unknown'
    match = re.search(r'<version>\s*([^<\s]+)\s*</version>', text)
    if match is None:
        logger.warning('no <version> element in %s', PACKAGE_XML)
        return '0.0.0+unknown'
    return match.group(1)


# -- Project ------------------------------------------------------------------

project = 'vectorfield_nav2'
author = 'Héctor Azpúrua'
copyright = '2026, Héctor Azpúrua'
release = _read_version()
version = '.'.join(release.split('.')[:2])
language = 'en'

# -- General ------------------------------------------------------------------

extensions = [
    'myst_parser',
    'sphinx.ext.mathjax',
    'sphinx.ext.intersphinx',
    'sphinx.ext.graphviz',
    'sphinx.ext.githubpages',
    'breathe',
    'sphinxcontrib.bibtex',
    'sphinx_design',
    'sphinx_copybutton',
    # docs/vf_clips.py: the `clip` directive (gallery videos of docs/_static/media).
    'vf_clips',
]

root_doc = 'index'
source_suffix = {
    '.md': 'markdown',
    '.rst': 'restructuredtext',
}
# _doxygen holds Doxygen's XML output (read by Breathe), never Sphinx sources.
exclude_patterns = ['_build', '_doxygen', '**/.DS_Store']

# -- MyST ---------------------------------------------------------------------

myst_enable_extensions = [
    'dollarmath',
    'amsmath',
    'colon_fence',
    'deflist',
    'attrs_inline',
]
myst_heading_anchors = 3

# -- Intersphinx --------------------------------------------------------------

# docs.ros.org answers Sphinx's inventory request with a bot-check page, so the ROS 2
# documentation is linked directly instead of through an inventory.
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
}
intersphinx_timeout = 30

# -- Graphviz -----------------------------------------------------------------

graphviz_output_format = 'svg'

# -- Breathe (Doxygen XML from `cd docs && doxygen Doxyfile`) ------------------

breathe_projects = {'vectorfield_core': str(DOCS_DIR / '_doxygen' / 'xml')}
breathe_default_project = 'vectorfield_core'
breathe_default_members = ('members',)

# -- Bibliography -------------------------------------------------------------

bibtex_bibfiles = ['references.bib']
bibtex_default_style = 'plain'
bibtex_reference_style = 'author_year'

# -- HTML ---------------------------------------------------------------------

REPOSITORY_URL = 'https://github.com/h3ct0r/vectorfield_nav2'

# GitHub's mark (Octicons mark-github, MIT) for the footer.
GITHUB_ICON = (
    '<svg stroke="currentColor" fill="currentColor" stroke-width="0" viewBox="0 0 16 16">'
    '<path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17'
    '.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15'
    '-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78'
    '-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64'
    '-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51'
    '.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2'
    ' 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"></path></svg>'
)

html_theme = 'furo'
html_title = f'vectorfield_nav2 {release}'
html_baseurl = 'https://h3ct0r.github.io/vectorfield_nav2/'
# _static/media holds the gallery clips (vf_clips); custom.css styles the landing page and the
# gallery, clips.js plays the clips only while they are on screen.
html_static_path = ['_static']
html_css_files = ['custom.css']
html_js_files = [('clips.js', {'defer': 'defer'})]
html_theme_options = {
    # "Edit this page" and "View source" link to the Markdown sources on main.
    'source_repository': f'{REPOSITORY_URL}/',
    'source_branch': 'main',
    'source_directory': 'docs/',
    'navigation_with_keys': True,
    # The teal of the curve markers; lighter on the dark background.
    'light_css_variables': {
        'color-brand-primary': '#0b6e8a',
        'color-brand-content': '#0b6e8a',
        'color-brand-visited': '#0b6e8a',
    },
    'dark_css_variables': {
        'color-brand-primary': '#4cc3d9',
        'color-brand-content': '#4cc3d9',
        'color-brand-visited': '#4cc3d9',
    },
    'footer_icons': [
        {'name': 'GitHub', 'url': REPOSITORY_URL, 'html': GITHUB_ICON, 'class': ''},
    ],
}


# -- Build hooks --------------------------------------------------------------


def _generate_parameter_pages(app):
    """Write parameters/{controller,follower}.md before Sphinx collects sources."""
    for message in gen_params.generate(REPO_ROOT, DOCS_DIR / 'parameters'):
        logger.warning('gen_params: %s', message)


def _hide_generated_source_links(app, pagename, templatename, context, doctree):
# Renamed or removed pages and where their old URLs lead (GitHub Pages has no server redirects).
REDIRECTS = {
    'roadmap': 'overview',
    'design/m7_3d': 'design/core_3d',
    'design/m7_3d_ros': 'design/follower_3d',
    'design/m7_ackermann_contour': 'design/ackermann_contour',
    'design/m7_cbf': 'design/cbf',
    'design/m7_follow_curve_bt': 'design/follow_curve_bt',
    'design/m7_reversing': 'design/reversing',
}


def _write_redirects(app, exception):
    """Write a page at each old URL of REDIRECTS that forwards to its new one."""
    if exception is not None or app.builder.format != 'html':
        return
    for old, new in REDIRECTS.items():
        target = '../' * old.count('/') + f'{new}.html'
        page = Path(app.outdir) / f'{old}.html'
        page.parent.mkdir(parents=True, exist_ok=True)
        page.write_text(
            '<!DOCTYPE html>\n<html lang="en"><head><meta charset="utf-8">'
            f'<meta http-equiv="refresh" content="0; url={target}">'
            f'<link rel="canonical" href="{html_baseurl}{new}.html"><title>Moved</title></head>'
            f'<body><p>This page moved to <a href="{target}">{target}</a>.</p></body></html>\n',
            encoding='utf-8',
        )


    """Drop the edit and view buttons of the generated parameter pages: git has no source."""
    if pagename.startswith('parameters/'):
        context['page_source_suffix'] = ''


def setup(app):
    """Register the build hooks."""
    app.connect('builder-inited', _generate_parameter_pages)
    app.connect('html-page-context', _hide_generated_source_links)
    return {'parallel_read_safe': True, 'parallel_write_safe': True}
    app.connect('build-finished', _write_redirects)
