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
    'sphinx_rtd_theme',
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

html_theme = 'sphinx_rtd_theme'
html_title = f'vectorfield_nav2 {release}'
html_baseurl = 'https://h3ct0r.github.io/vectorfield_nav2/'
html_theme_options = {
    'navigation_depth': 3,
    'collapse_navigation': False,
}
html_context = {
    'display_github': True,
    'github_user': 'h3ct0r',
    'github_repo': 'vectorfield_nav2',
    'github_version': 'main',
    'conf_py_path': '/docs/',
}


# -- Build hooks --------------------------------------------------------------


def _generate_parameter_pages(app):
    """Write parameters/{controller,follower}.md before Sphinx collects sources."""
    for message in gen_params.generate(REPO_ROOT, DOCS_DIR / 'parameters'):
        logger.warning('gen_params: %s', message)


# sphinx_rtd_theme keeps every table cell on one line and scrolls wide tables sideways, which
# makes the long legend and roadmap tables unreadable. Let the cells wrap instead.
TABLE_WRAP_CSS = (
    '<style>.rst-content .wy-table-responsive table.docutils td, '
    '.rst-content .wy-table-responsive table.docutils th { white-space: normal; }</style>'
)


def _add_table_wrap_css(app, pagename, templatename, context, doctree):
    """Append the table-wrapping rule to every HTML page's head."""
    context['metatags'] = context.get('metatags', '') + TABLE_WRAP_CSS


def setup(app):
    """Register the build hooks."""
    app.connect('builder-inited', _generate_parameter_pages)
    app.connect('html-page-context', _add_table_wrap_css)
    return {'parallel_read_safe': True, 'parallel_write_safe': True}
