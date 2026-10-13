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

"""Import smoke tests for the installed vectorfield and vectorfield_tools packages.

They run against the installed packages: the pip wheel, or the ament install space under colcon.
"""

import re
from importlib import metadata
from pathlib import Path

import vectorfield
import vectorfield_tools

PACKAGE_XML = Path(__file__).resolve().parents[1] / 'package.xml'


def _package_xml_version():
    text = PACKAGE_XML.read_text(encoding='utf-8')
    match = re.search(r'<version>\s*([^<\s]+)\s*</version>', text)
    assert match is not None, f'no <version> tag in {PACKAGE_XML}'
    return match.group(1)


def test_version_matches_package_xml():
    """The compiled core reports the version in package.xml."""
    assert vectorfield.__version__ == _package_xml_version()


def test_distribution_metadata_matches_package_xml():
    """The installed distribution metadata (wheel or ament egg-info) carries the same version."""
    assert metadata.version('vectorfield') == _package_xml_version()


def test_extension_is_inside_the_package():
    """The compiled module is installed next to vectorfield/__init__.py.

    Paths are compared as installed, without resolving symlinks: under
    ``colcon build --symlink-install`` both files are symlinks in the same
    install directory, pointing into the build and source trees respectively.
    """
    extension = Path(vectorfield._vectorfield.__file__).absolute()
    assert extension.parent == Path(vectorfield.__file__).absolute().parent


def test_tools_package_imports():
    """The vectorfield_tools package is installed alongside vectorfield."""
    assert vectorfield_tools.__doc__
