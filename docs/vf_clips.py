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

"""Sphinx extension: the ``clip`` directive, a gallery clip as a looping video.

The clips in ``docs/_static/media`` are recordings of the vectorfield_mujoco examples
(``vectorfield_mujoco/scripts/record_gallery.sh``): ``<name>.mp4``, ``<name>.gif`` and the
poster ``<name>.jpg``. In MyST::

    ```{clip} differential
    :alt: PAL TIAGo contours around an unmapped box
    :controls:
    ```

renders ``<video autoplay loop muted playsinline>`` of the MP4 with the JPEG as its poster and
the GIF as the fallback of browsers without video (HTML builders only). A missing file is a
warning, so ``sphinx-build -W`` fails on a clip that was not recorded.
"""

import html
from pathlib import Path

from docutils import nodes
from docutils.parsers.rst import directives
from sphinx.util import logging
from sphinx.util.docutils import SphinxDirective

MEDIA_DIR = Path(__file__).resolve().parent / '_static' / 'media'
SUFFIXES = ('mp4', 'gif', 'jpg')

logger = logging.getLogger(__name__)


class ClipDirective(SphinxDirective):
    """``{clip} <name>``: options ``alt``, ``class`` (extra classes) and ``controls``."""

    required_arguments = 1
    option_spec = {
        'alt': directives.unchanged,
        'class': directives.class_option,
        'controls': directives.flag,
    }

    def run(self):
        """Return the video as a raw HTML node."""
        name = self.arguments[0]
        missing = [f'{name}.{s}' for s in SUFFIXES if not (MEDIA_DIR / f'{name}.{s}').is_file()]
        if missing:
            logger.warning(
                'clip %s: %s missing from docs/_static/media',
                name,
                ', '.join(missing),
                location=(self.env.docname, self.lineno),
            )
            return []
        # Pages are <docname>.html (the html builder): climb to the root for _static.
        src = '../' * self.env.docname.count('/') + f'_static/media/{name}'
        alt = html.escape(self.options.get('alt', name.replace('_', ' ')), quote=True)
        classes = ' '.join(['vf-clip', *self.options.get('class', [])])
        controls = ' controls' if 'controls' in self.options else ''
        markup = (
            f'<video class="{classes}" autoplay loop muted playsinline{controls} '
            f'preload="metadata" poster="{src}.jpg" aria-label="{alt}">'
            f'<source src="{src}.mp4" type="video/mp4">'
            f'<img src="{src}.gif" alt="{alt}" loading="lazy">'
            '</video>'
        )
        return [nodes.raw('', markup, format='html')]


def setup(app):
    """Register the directive."""
    app.add_directive('clip', ClipDirective)
    return {'parallel_read_safe': True, 'parallel_write_safe': True}
