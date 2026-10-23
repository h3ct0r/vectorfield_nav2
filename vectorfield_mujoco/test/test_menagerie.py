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

"""Unit tests of vectorfield_mujoco.menagerie without the network."""

import shutil

import pytest

from vectorfield_mujoco import menagerie


def _fake_checkout(path, dirs=menagerie.ROBOT_DIRS):
    for d in dirs:
        (path / d).mkdir(parents=True)
        (path / d / 'robot.xml').write_text('<mujoco/>')
    return path


def test_pinned_commit():
    """The pin is a full 40-hex-digit commit SHA."""
    assert len(menagerie.MENAGERIE_COMMIT) == 40
    int(menagerie.MENAGERIE_COMMIT, 16)


def test_default_dir(monkeypatch, tmp_path):
    """Without VF_MENAGERIE_DIR the checkout lives in the cache, keyed by the commit."""
    monkeypatch.delenv(menagerie.ENV_VAR, raising=False)
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    assert menagerie.menagerie_dir() == (
        tmp_path / 'vectorfield_mujoco' / 'menagerie' / menagerie.MENAGERIE_COMMIT
    )
    assert not menagerie.is_available()


def test_env_checkout_is_used_as_is(monkeypatch, tmp_path):
    """A VF_MENAGERIE_DIR checkout with the robots is used without fetching."""
    checkout = _fake_checkout(tmp_path / 'menagerie')
    monkeypatch.setenv(menagerie.ENV_VAR, str(checkout))

    def no_git(*args, **kwargs):
        raise AssertionError('must not fetch')

    monkeypatch.setattr(menagerie, '_git', no_git)
    assert menagerie.is_available()
    assert menagerie.ensure_menagerie() == checkout


def test_incomplete_directory_is_an_error(monkeypatch, tmp_path):
    """A non-empty directory without the robots is refused, not overwritten."""
    _fake_checkout(tmp_path, dirs=menagerie.ROBOT_DIRS[:1])
    monkeypatch.setattr(menagerie, '_git', lambda *a, **k: pytest.fail('must not fetch'))
    with pytest.raises(menagerie.MenagerieError, match='does not hold'):
        menagerie.ensure_menagerie(tmp_path)


def test_fetch_is_idempotent(monkeypatch, tmp_path):
    """The first call fetches; the second finds the complete checkout and does not."""
    calls = []

    def fake_clone(target, commit, robot_dirs):
        calls.append(target)
        _fake_checkout(target, robot_dirs)

    monkeypatch.setattr(menagerie, '_clone', fake_clone)
    target = tmp_path / 'cache' / 'sha'
    assert menagerie.ensure_menagerie(target) == target
    assert menagerie.ensure_menagerie(target) == target
    assert len(calls) == 1
    assert not list(target.parent.glob('*.tmp-*'))


def test_offline_error(monkeypatch, tmp_path):
    """A failed fetch raises with the offline hint and leaves nothing behind."""
    if shutil.which('git') is None:
        pytest.skip('git not installed')
    monkeypatch.setattr(menagerie, 'MENAGERIE_URL', str(tmp_path / 'no-such-repo'))
    with pytest.raises(menagerie.MenagerieError, match='VF_MENAGERIE_DIR'):
        menagerie.ensure_menagerie(tmp_path / 'checkout')
    # Nothing half-made is left behind.
    assert not (tmp_path / 'checkout').exists()
    assert menagerie.main(['--dir', str(tmp_path / 'checkout')]) == 1
