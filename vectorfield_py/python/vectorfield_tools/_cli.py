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

"""Shared plumbing of the vf-* CLIs: exit codes, lazy optional imports and argument parsing."""

import argparse
import importlib
import math
import re
import sys

import vectorfield as vf

#: Exit codes of every CLI.
EXIT_OK = 0
EXIT_MISSING_DEPENDENCY = 1
EXIT_BAD_INPUT = 2

#: Optional modules: import name -> (pip distribution, rosdep key, what needs it).
_OPTIONAL = {
    'matplotlib': ('matplotlib', 'python3-matplotlib', 'plots and animations'),
    'PIL': ('Pillow', 'python3-pil', 'GIF animations'),
    'yaml': ('PyYAML', 'python3-yaml', 'YAML files'),
}


class InputError(ValueError):
    """Bad command-line input or input file; the CLI exits with EXIT_BAD_INPUT."""


class MissingDependencyError(ImportError):
    """An optional dependency of the CLIs is not installed."""


def require(module):
    """Import an optional module, or raise MissingDependencyError with an install hint."""
    try:
        return importlib.import_module(module)
    except ImportError as error:
        distribution, rosdep, purpose = _OPTIONAL[module]
        raise MissingDependencyError(
            f'{distribution} is required for {purpose} but is not installed. Install the tools '
            f"extra: pip install './vectorfield_py[tools]' (under ROS: the {rosdep} rosdep key)."
        ) from error


def pyplot(interactive=False):
    """Return matplotlib.pyplot; without ``interactive`` the Agg backend is selected."""
    matplotlib = require('matplotlib')
    if not interactive:
        matplotlib.use('Agg')
    return importlib.import_module('matplotlib.pyplot')


def run(prog, function, argv):
    """Call ``function(argv)`` and map the CLI exceptions to exit codes with a message."""
    try:
        return function(argv)
    except MissingDependencyError as error:
        print(f'{prog}: {error}', file=sys.stderr)
        return EXIT_MISSING_DEPENDENCY
    except vf.CurveError as error:
        print(f'{prog}: invalid curve ({error.code}): {error}', file=sys.stderr)
        return EXIT_BAD_INPUT
    except (InputError, vf.ConfigError, OSError) as error:
        print(f'{prog}: error: {error}', file=sys.stderr)
        return EXIT_BAD_INPUT


#: Options whose value may start with a minus sign (comma-separated numbers, expressions).
DASH_VALUE_OPTIONS = frozenset(
    {'--pose', '--circle', '--box', '--extent', '--decompose', '--slip', '--costmap'}
    | {'--s-range', '--curve'}
)


def parse_args(argp, argv):
    """Parse ``argv`` (default sys.argv[1:]), also accepting ``--pose -1,0,0``.

    argparse takes a value such as ``-1,0,0`` for an option, so it is joined to its option
    (``--pose=-1,0,0``) before parsing.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    joined, k = [], 0
    while k < len(argv):
        if argv[k] in DASH_VALUE_OPTIONS and k + 1 < len(argv) and argv[k + 1].startswith('-'):
            joined.append(f'{argv[k]}={argv[k + 1]}')
            k += 2
        else:
            joined.append(argv[k])
            k += 1
    return argp.parse_args(joined)


def parser(prog, description, epilog=None):
    """Return an ArgumentParser with the shared formatting."""
    return argparse.ArgumentParser(
        prog=prog,
        description=description,
        epilog=epilog,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )


_PI_TERM = re.compile(r'^([+-]?)(\d*\.?\d*)\s*\*?\s*pi\s*(?:/\s*(\d+\.?\d*))?$')


def number(text):
    """Parse a float, also written with pi: ``1.5``, ``pi``, ``-pi/2``, ``3*pi/4``, ``2pi``."""
    text = str(text).strip()
    try:
        return float(text)
    except ValueError:
        pass
    match = _PI_TERM.match(text.lower())
    if match is None:
        raise InputError(f'not a number: {text!r}')
    sign, factor, divisor = match.groups()
    value = (float(factor) if factor not in ('', '.') else 1.0) * math.pi
    if divisor:
        value /= float(divisor)
    return -value if sign == '-' else value


def numbers(text, count=None, name='value'):
    """Comma-separated numbers, optionally exactly ``count`` (a tuple of allowed counts)."""
    values = [number(part) for part in str(text).split(',') if part.strip()]
    if count is not None:
        allowed = count if isinstance(count, tuple) else (count,)
        if len(values) not in allowed:
            expected = ' or '.join(str(n) for n in allowed)
            raise InputError(f'{name} needs {expected} comma-separated numbers, got {text!r}')
    return values


def scalar(text):
    """Parse a command-line parameter value: bool, int, float or string."""
    lowered = text.strip().lower()
    if lowered in ('true', 'false'):
        return lowered == 'true'
    for convert in (int, number):
        try:
            return convert(text)
        except ValueError:
            pass
    return text


def assignment(text, convert=scalar):
    """Split ``name=value`` into (name, converted value)."""
    name, sep, value = str(text).partition('=')
    if not sep or not name.strip():
        raise InputError(f'expected NAME=VALUE, got {text!r}')
    return name.strip(), convert(value.strip())


def add_output_arguments(argp, what='the figure'):
    """``-o/--output`` and ``--show``."""
    argp.add_argument(
        '-o',
        '--output',
        metavar='FILE',
        help=f'save {what} (format from the extension: .png, .svg, .pdf)',
    )
    argp.add_argument('--show', action='store_true', help=f'show {what} in a window')
    argp.add_argument('--dpi', type=float, default=120.0, help='raster resolution (default 120)')


def finish_figure(plt, fig, args):
    """Save and/or show a figure as the output arguments ask, then close it."""
    if args.output:
        fig.savefig(args.output, dpi=args.dpi, metadata=svg_metadata(args.output))
        print(f'wrote {args.output}')
    if args.show:
        plt.show()
    plt.close(fig)


def svg_metadata(path):
    """Metadata that keeps SVG output reproducible (no date); None for other formats."""
    return {'Date': None} if str(path).lower().endswith('.svg') else None
