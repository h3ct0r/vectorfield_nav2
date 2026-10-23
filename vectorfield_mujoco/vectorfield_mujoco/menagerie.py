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

"""The MuJoCo Menagerie robots, fetched on demand at a pinned commit.

Menagerie assets are not vendored. :func:`ensure_menagerie` sparse-clones only the robot
directories the examples use (:data:`ROBOT_DIRS`) at :data:`MENAGERIE_COMMIT` into
``${VF_MENAGERIE_DIR:-~/.cache/vectorfield_mujoco/menagerie/<sha>}``. It is idempotent: a
complete checkout is reused without touching the network. ``VF_MENAGERIE_DIR`` may point at an
existing Menagerie checkout (any commit) for offline use; it is then used as is.

``ros2 run vectorfield_mujoco fetch_menagerie`` runs :func:`main`.
"""

import argparse
import os
import pathlib
import shutil
import subprocess
import sys

MENAGERIE_URL = 'https://github.com/google-deepmind/mujoco_menagerie.git'

#: google-deepmind/mujoco_menagerie main on 2026-10-23.
MENAGERIE_COMMIT = 'f3b6013629c7b94f8ee93fd977b161fa8350f9bc'

#: The only directories fetched; each keeps its own LICENSE.
ROBOT_DIRS = ('pal_tiago', 'stanford_tidybot', 'skydio_x2')

#: Overrides the checkout directory.
ENV_VAR = 'VF_MENAGERIE_DIR'

# Written last into a checkout made by ensure_menagerie(), so an interrupted fetch is redone.
_MARKER = '.vectorfield_mujoco_complete'


class MenagerieError(RuntimeError):
    """The Menagerie checkout is missing and could not be fetched."""


def default_dir(commit=MENAGERIE_COMMIT):
    """Return the cache directory of a commit (``$XDG_CACHE_HOME`` or ``~/.cache``)."""
    cache = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    return pathlib.Path(cache) / 'vectorfield_mujoco' / 'menagerie' / commit


def menagerie_dir():
    """Return the checkout directory: ``$VF_MENAGERIE_DIR`` if set, else :func:`default_dir`."""
    override = os.environ.get(ENV_VAR)
    return pathlib.Path(override).expanduser() if override else default_dir()


def has_robots(path, robot_dirs=ROBOT_DIRS):
    """Return whether every robot directory exists under ``path`` and holds an MJCF file."""
    path = pathlib.Path(path)
    return all(any((path / d).glob('*.xml')) for d in robot_dirs)


def is_available():
    """Return whether the robots are available without fetching (used by tests to skip)."""
    path = menagerie_dir()
    if os.environ.get(ENV_VAR):
        return has_robots(path)
    return (path / _MARKER).is_file() and has_robots(path)


def _git(args, cwd=None):
    try:
        result = subprocess.run(
            ['git', *args], cwd=cwd, check=False, capture_output=True, text=True, timeout=600
        )
    except FileNotFoundError as e:
        raise MenagerieError('git is required to fetch the MuJoCo Menagerie robots') from e
    except subprocess.TimeoutExpired as e:
        raise MenagerieError(f'git {args[0]} timed out') from e
    if result.returncode != 0:
        raise MenagerieError(f'git {" ".join(args)} failed: {result.stderr.strip()}')
    return result.stdout


def _clone(target, commit, robot_dirs):
    """Sparse, partial, shallow clone of ``robot_dirs`` at ``commit`` into ``target``."""
    _git(['init', '-q', str(target)])
    _git(['remote', 'add', 'origin', MENAGERIE_URL], cwd=target)
    _git(['sparse-checkout', 'set', *robot_dirs], cwd=target)
    # GitHub serves any reachable commit by its full SHA; blob:none downloads only the blobs
    # that the sparse checkout needs (about 20 MB for the three robots).
    _git(['fetch', '-q', '--depth', '1', '--filter=blob:none', 'origin', commit], cwd=target)
    _git(['-c', 'advice.detachedHead=false', 'checkout', '-q', 'FETCH_HEAD'], cwd=target)


def ensure_menagerie(path=None, commit=MENAGERIE_COMMIT, robot_dirs=ROBOT_DIRS, log=None):
    """Return a directory holding the Menagerie robots, fetching them first if needed.

    Args:
        path: The checkout directory (default :func:`menagerie_dir`).
        commit: The full commit SHA to fetch.
        robot_dirs: The robot directories to check out.
        log: Optional callable taking a progress message.

    Raises:
        MenagerieError: The directory holds something else, or the fetch failed (offline, no
            git); the message says how to provide a checkout instead.
    """
    explicit = path is not None or bool(os.environ.get(ENV_VAR))
    path = pathlib.Path(path) if path is not None else menagerie_dir()
    if (path / _MARKER).is_file() and has_robots(path, robot_dirs):
        return path
    if explicit and has_robots(path, robot_dirs):
        # A checkout made by someone else (VF_MENAGERIE_DIR): used as is, never modified.
        return path
    if path.exists() and any(path.iterdir()):
        raise MenagerieError(
            f'{path} exists but does not hold the Menagerie robots {", ".join(robot_dirs)}; '
            'remove it or point VF_MENAGERIE_DIR at a mujoco_menagerie checkout'
        )
    if log:
        log(f'Fetching MuJoCo Menagerie {commit[:12]} ({", ".join(robot_dirs)}) into {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f'{path.name}.tmp-{os.getpid()}')
    shutil.rmtree(tmp, ignore_errors=True)
    try:
        _clone(tmp, commit, robot_dirs)
        if not has_robots(tmp, robot_dirs):
            raise MenagerieError(f'commit {commit} does not contain {", ".join(robot_dirs)}')
        (tmp / _MARKER).write_text(commit + '\n')
        if path.exists():
            path.rmdir()  # empty, checked above
        try:
            tmp.rename(path)
        except OSError:
            # Another process finished the same fetch first.
            if not has_robots(path, robot_dirs):
                raise
    except MenagerieError as e:
        raise MenagerieError(
            f'{e}\nCould not fetch the MuJoCo Menagerie robots (offline?). Run '
            f'`ros2 run vectorfield_mujoco fetch_menagerie` with network access, or set '
            f'{ENV_VAR} to a mujoco_menagerie checkout.'
        ) from e
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return path


def main(argv=None):
    """Command-line entry point: fetch the robots and print the checkout directory."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument(
        '--dir',
        type=pathlib.Path,
        default=None,
        help=f'checkout directory (default: {ENV_VAR} '
        'or ~/.cache/vectorfield_mujoco/menagerie/<sha>)',
    )
    args = parser.parse_args(argv)
    try:
        path = ensure_menagerie(args.dir, log=lambda m: print(m, file=sys.stderr))
    except MenagerieError as e:
        print(f'error: {e}', file=sys.stderr)
        return 1
    print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
