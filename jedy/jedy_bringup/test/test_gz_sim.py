"""jedy_gz_sim leaves nothing of Gazebo behind.

A fake ``gz`` on PATH stands in for the Ruby ``gz sim``: it forks a "server"
into a process group of its own (as gz sim does) and both ignore SIGINT, as a
hung server does. Neither runs Gazebo.
"""

import os
import signal
import subprocess
import sys
import textwrap
import time

SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts", "jedy_gz_sim")

FAKE_GZ = textwrap.dedent("""\
    #!{python}
    import os, signal, sys, time
    signal.signal(signal.SIGINT, signal.SIG_IGN if {hang} else signal.SIG_DFL)
    pid = os.fork()
    if pid == 0:
        os.setpgid(0, 0)
        with open(sys.argv[-1], "w") as f:
            f.write(str(os.getpid()))
        while True:
            time.sleep(1)
    while True:
        time.sleep(1)
""")


def alive(pid):
    try:
        with open("/proc/%d/stat" % pid) as handle:
            return handle.read().rsplit(")", 1)[1].split()[0] != "Z"
    except OSError:
        return False


def run(tmp_path, hang):
    fake = tmp_path / "gz"
    fake.write_text(FAKE_GZ.format(python=sys.executable, hang=hang))
    fake.chmod(0o755)
    pid_file = tmp_path / "server.pid"
    env = dict(os.environ, PATH="%s:%s" % (tmp_path, os.environ["PATH"]))
    wrapper = subprocess.Popen(
        [sys.executable, SCRIPT, "--grace", "1", "--", "sim", str(pid_file)],
        env=env)
    deadline = time.monotonic() + 10
    while not pid_file.exists() or not pid_file.read_text():
        assert time.monotonic() < deadline, "fake gz never started"
        time.sleep(0.05)
    server = int(pid_file.read_text())
    started = time.monotonic()
    wrapper.send_signal(signal.SIGINT)
    status = wrapper.wait(timeout=10)
    return status, server, time.monotonic() - started


def test_server_that_ignores_sigint_is_killed(tmp_path):
    status, server, took = run(tmp_path, hang=True)
    assert status == 0
    assert not alive(server)
    assert took < 5


def test_server_in_its_own_group_is_stopped(tmp_path):
    status, server, took = run(tmp_path, hang=False)
    assert status == 0
    assert not alive(server)
