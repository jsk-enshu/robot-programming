"""Refuse to start a Jedy robot on top of one that is already running.

Only one of Gazebo / the real robot / Isaac Sim may run at a time (the course
docs say so): two of them publish the same topics, and a Gazebo server left
over from an earlier launch keeps the old world, so a "fresh" launch starts
with the robot wherever it was. Both used to fail late and confusingly
("Controller already loaded", the robot not at its start pose). This check
fails at once and names the processes and how to stop them.

Used from the top-level launch files::

    from jedy_preflight import preflight_action
    LaunchDescription([..., preflight_action(), ...])

Set ``JEDY_SKIP_PREFLIGHT=1`` to bypass (staff only).
"""

import os

from launch.actions import OpaqueFunction

#: Top-level bringups that own the robot's topics.
ROBOT_LAUNCH_FILES = (
    "jedy_gazebo.launch.py",
    "jedy_fullbody_all.launch.py",
    "jedy_isaac.launch.py",
    "jedy_four_dof_all.launch.py",
)


def _read_proc(pid):
    """Return (argv, env) of one of our own processes, or None."""
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            argv = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
        with open("/proc/%d/environ" % pid, "rb") as f:
            env = {}
            for item in f.read().split(b"\0"):
                if b"=" in item:
                    key, value = item.split(b"=", 1)
                    env[key.decode(errors="replace")] = value.decode(errors="replace")
    except OSError:
        return None
    return argv, env


def _age_minutes(pid):
    """Minutes since the process started."""
    try:
        with open("/proc/%d/stat" % pid) as f:
            fields = f.read().rsplit(")", 1)[1].split()
        start_ticks = int(fields[19])
        with open("/proc/uptime") as f:
            uptime = float(f.read().split()[0])
        return int((uptime - start_ticks / os.sysconf("SC_CLK_TCK")) / 60)
    except (OSError, ValueError, IndexError):
        return -1


def _ancestors(pid=None):
    """PIDs of a process (default: this one) and its parents."""
    pids = set()
    pid = os.getpid() if pid is None else pid
    while pid > 1 and pid not in pids:
        pids.add(pid)
        try:
            with open("/proc/%d/stat" % pid) as f:
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def find_conflicts():
    """Running Jedy bringups / Gazebo servers that would clash with this launch.

    Returns
    -------
    list of tuple
        ``(pid, age_minutes, kind, command)`` for each conflicting process.
    """
    my_domain = os.environ.get("ROS_DOMAIN_ID", "0") or "0"
    my_partition = os.environ.get("GZ_PARTITION", "")
    skip = _ancestors()
    uid = os.getuid()
    found = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in skip:
            continue
        try:
            if os.stat("/proc/%d" % pid).st_uid != uid:
                continue
        except OSError:
            continue
        info = _read_proc(pid)
        if info is None or not info[0]:
            continue
        argv, env = info
        if len(argv) == 1:
            # gz is a Ruby wrapper that rewrites its process title into one
            # space-joined string ("gz sim -r -s ...").
            argv = argv[0].split()
        command = " ".join(argv)
        if os.path.basename(argv[0]) == "gz" and argv[1:2] == ["sim"]:
            if env.get("GZ_PARTITION", "") == my_partition:
                found.append((pid, _age_minutes(pid), "gazebo", command))
        elif "launch" in argv and any(
                os.path.basename(a) in ROBOT_LAUNCH_FILES for a in argv):
            if (env.get("ROS_DOMAIN_ID", "0") or "0") == my_domain:
                found.append((pid, _age_minutes(pid), "launch", command))
    # A Gazebo server none of whose parents is a launch outlived its launch.
    launch_pids = {p for p, _, k, _ in found if k == "launch"}
    marked = []
    for pid, age, kind, command in found:
        if kind == "gazebo" and not (_ancestors(pid) & launch_pids):
            kind = "orphan"
        marked.append((pid, age, kind, command))
    return marked


def format_conflicts(found):
    """Human-readable error for students (Japanese) with the fix."""
    lines = [
        "Jedy (Gazebo / 実機 / Isaac Sim のどれか) がすでに起動しています。"
        "2 つ同時には起動できません (topic がぶつかり，古い Gazebo の状態が残ります)。",
        "動いているもの:",
    ]
    for pid, age, kind, command in found:
        note = "  <- launch が終わった後も残っている Gazebo" if kind == "orphan" else ""
        lines.append("  PID %d (%s): %s%s" % (
            pid, "%d 分前から" % age if age >= 0 else "?", command[:120], note))
    pids = " ".join(str(p) for p, _, _, _ in found)
    lines += [
        "止め方: それを起動したターミナルで Ctrl-C。見つからなければ:",
        "  kill -INT %s" % pids,
        "  (10 秒たっても残っていれば kill -9 %s)" % pids,
        "その後もう一度 launch してください。",
    ]
    return "\n".join(lines)


def _check(context, *args, **kwargs):
    if os.environ.get("JEDY_SKIP_PREFLIGHT") == "1":
        return []
    found = find_conflicts()
    if found:
        raise RuntimeError(format_conflicts(found))
    return []


def preflight_action():
    """Launch action that aborts the launch when a robot is already running."""
    return OpaqueFunction(function=_check)
