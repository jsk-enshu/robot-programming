"""jedy_wait_port when the launch that reads its output goes away.

The launch reads the waiter's stderr through a pipe. Once the launch is gone
(Ctrl-C, or it died) the next "still waiting" line hits a closed pipe; that
used to end in a BrokenPipeError traceback, which students and the AI TA took
for the real error.
"""

import importlib.machinery
import importlib.util
import os
import subprocess
import sys

SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts",
                      "jedy_wait_port")
MISSING = "/dev/jedy_wait_port_test_no_such_device"


def load_script():
    """Import the extension-less script as a module."""
    loader = importlib.machinery.SourceFileLoader("jedy_wait_port", SCRIPT)
    spec = importlib.util.spec_from_loader("jedy_wait_port", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_missing_port_times_out():
    wait_port = load_script()
    clock = iter(range(100))
    lines = []

    class Out:
        def write(self, text):
            lines.append(text)

        def flush(self):
            pass

    status = wait_port.wait(MISSING, timeout=3, report_interval=100,
                            now=lambda: next(clock), sleep=lambda _: None,
                            out=Out())
    assert status == 1
    assert any("gave up waiting" in line for line in lines)


def test_closed_pipe_exits_quietly():
    """Read one line, close the pipe as a departing launch does."""
    process = subprocess.Popen(
        [sys.executable, SCRIPT, "--channel", MISSING,
         "--report-interval", "0.2"],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    first = process.stderr.readline().decode()
    process.stderr.close()
    status = process.wait(timeout=10)
    assert "waiting for" in first
    # 1: the wait did not finish. An uncaught BrokenPipeError exits 1 as
    # well, but then Python's flush at exit fails too and the status is 120.
    assert status == 1

