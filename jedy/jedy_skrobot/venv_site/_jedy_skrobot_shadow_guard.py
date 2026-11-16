"""Name the student's own file when it shadows a package this venv provides.

Loaded at interpreter start-up by ``_jedy_skrobot_shadow_guard.pth``, which
CMakeLists.txt installs into the venv's site-packages, so it runs for the venv's
``python``, ``ipython`` and every ``ros2 run jedy_skrobot ...`` wrapper.

The failure it replaces: a student saves a program as ``~/ros2_ws/jedy_skrobot.py``,
starts ``venv/bin/ipython`` in that directory, and ``from jedy_skrobot import
connect`` imports *their* file (the current directory comes first on sys.path)::

    ImportError: cannot import name 'connect' from 'jedy_skrobot'
    (/home/mech-user/ros2_ws/jedy_skrobot.py)

which reads like a broken install. Here the import fails instead with a message
that says which file it is and to rename it.

Only names that are *packages* are guarded, so a plain ``<name>.py`` found for
one of them is never the real thing. Nothing is loaded or changed for any other
import, and for the guarded names the normal import machinery still does the
actual import.
"""

import importlib.machinery
import os
import sys

#: Packages a student's program imports, whose name they might give a file.
#: Every one of them is a package (a directory), never a single ``.py`` file.
GUARDED = ("jedy_skrobot", "skrobot", "rclpy", "numpy")


def message(name, origin):
    """The error text for a file that shadows a guarded package.

    Parameters
    ----------
    name : str
        The package being imported, e.g. ``"jedy_skrobot"``.
    origin : str
        The file that would have been imported instead.

    Returns
    -------
    str
        A Japanese explanation with the rename to do.
    """
    directory = os.path.dirname(origin) or "."
    renamed = os.path.join(directory, "my_program.py")
    return (
        "\n"
        "  {origin} が，パッケージ {name} の代わりに読み込まれています．\n"
        "  自分で作ったファイルの名前がパッケージと同じ ({name}.py) だと，\n"
        "  そのファイルが先に見つかり，本物の {name} が使えなくなります．\n"
        "  ファイルの名前を変えてください．例:\n"
        "    mv {origin} {renamed}\n"
        "  (jedy_skrobot.py, skrobot.py, rclpy.py, numpy.py などの名前は使わない)"
    ).format(origin=origin, name=name, renamed=renamed)


def find_spec(name, path=None, target=None):
    """Meta path hook: refuse a single-file module that shadows a package.

    Parameters
    ----------
    name : str
        Fully qualified module name.
    path : list of str or None
        ``None`` for a top-level import, which is the only kind checked.
    target : module or None
        Unused; part of the finder protocol.

    Returns
    -------
    None
        Always, so the regular finders do the import.

    Raises
    ------
    ImportError
        If ``name`` is guarded and what sys.path would find for it is a
        single file rather than the package.
    """
    if path is not None or name not in GUARDED:
        return None
    spec = importlib.machinery.PathFinder.find_spec(name)
    if spec is None or spec.submodule_search_locations is not None:
        return None
    if not spec.origin or not os.path.isfile(spec.origin):
        return None
    raise ImportError(message(name, spec.origin), name=name,
                      path=spec.origin)


def install():
    """Put this module first on ``sys.meta_path`` (once)."""
    module = sys.modules[__name__]
    if module not in sys.meta_path:
        sys.meta_path.insert(0, module)


install()
