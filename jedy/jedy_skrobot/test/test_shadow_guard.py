"""The venv's start-up guard against a student's file named like a package.

Only the standard library is needed::

    python3 -m pytest test/test_shadow_guard.py
"""

import importlib
import os
import subprocess
import sys

import pytest

GUARD_DIR = os.path.join(os.path.dirname(__file__), '..', 'venv_site')
sys.path.insert(0, GUARD_DIR)

import _jedy_skrobot_shadow_guard as guard  # noqa: E402


@pytest.fixture
def on_path(tmp_path, monkeypatch):
    """A directory first on sys.path, as the current directory is for ipython."""
    monkeypatch.syspath_prepend(str(tmp_path))
    importlib.invalidate_caches()
    return tmp_path


def test_single_file_shadowing_a_package_is_refused(on_path):
    own = on_path / 'jedy_skrobot.py'
    own.write_text('x = 1\n')
    with pytest.raises(ImportError) as raised:
        guard.find_spec('jedy_skrobot')
    assert str(own) in str(raised.value)
    assert 'mv ' in str(raised.value)


def test_package_directory_is_left_alone(on_path):
    package = on_path / 'jedy_skrobot'
    package.mkdir()
    (package / '__init__.py').write_text('')
    assert guard.find_spec('jedy_skrobot') is None


def test_other_names_and_submodules_are_not_looked_at(on_path):
    (on_path / 'my_program.py').write_text('')
    (on_path / 'skrobot.py').write_text('')
    assert guard.find_spec('my_program') is None
    assert guard.find_spec('skrobot', path=[str(on_path)]) is None


def test_interpreter_with_the_guard_names_the_file(tmp_path):
    """End to end: the .pth's import line, then a shadowed import."""
    (tmp_path / 'skrobot.py').write_text('')
    code = ('import sys; sys.path.insert(0, {!r}); '
            'import _jedy_skrobot_shadow_guard; import skrobot').format(
                os.path.abspath(GUARD_DIR))
    result = subprocess.run([sys.executable, '-c', code], cwd=str(tmp_path),
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert str(tmp_path / 'skrobot.py') in result.stderr
