#!/usr/bin/env python3
"""Check the mechatrobot's hardware and say what to do next.

Talks to the board directly (``firmware/uno_r4_serial``), so stop
``serial_bridge.py`` first. Checks, in order:

1. ROS 2 stays on this PC (``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST``)
2. the board's serial port: present, readable, not held by another program
3. the board sends lines (firmware running, USB bridge not stalled)
4. the HC-SR04 wiring, with the firmware's ``diag`` self-test
   (no echo, Trig/Echo swapped, Echo stuck high)
5. with ``--servo``: moves the servo -45 -> 45 deg and back. The servo does
   not report its angle, so a person has to look at it.

Usage::

    ros2 run mechatrobot_ros2 check_hardware.py
    ros2 run mechatrobot_ros2 check_hardware.py --servo

Exits with 0 when nothing is wrong, 1 otherwise.
"""
import argparse
import glob
import os
import sys
import time

import serial

BAUD = 115200
# Where the course material puts the firmware (robot-programming-3-2026.md).
FIRMWARE_DIR = "~/ros2_ws/src/robot-programming/mechatrobot_ros2/firmware"
ARDUINO_USB_VENDOR = "2341"
SOUND_SPEED_CM_PER_US = 0.0343
LISTEN_SEC = 2.0
DIAG_TIMEOUT_SEC = 2.0
DIAG_TRIES = 3
SERVO_TEST_DEG = 45
SERVO_SETTLE_SEC = 1.0


class Report:
    """Collect results and print them as they come."""

    def __init__(self):
        self.problems = []

    def ok(self, text):
        """Print a passed check.

        Parameters
        ----------
        text : str
            What was found.
        """
        print(f"[OK] {text}")

    def info(self, text):
        """Print a note that is neither a pass nor a failure.

        Parameters
        ----------
        text : str
            The note.
        """
        print(f"[--] {text}")

    def ng(self, text, action):
        """Print a failed check and what to do about it.

        Parameters
        ----------
        text : str
            What is wrong.
        action : str
            What the student should do next.
        """
        print(f"[NG] {text}")
        print(f"     -> {action}")
        self.problems.append((text, action))


def check_ros_env(report):
    """Check that ROS 2 talks only within this PC.

    Parameters
    ----------
    report : Report
        Where results go.
    """
    value = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    if value == "LOCALHOST":
        report.ok("ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST (ROS 2 は自分の PC の中だけ)")
    else:
        report.ng(
            f"ROS_AUTOMATIC_DISCOVERY_RANGE が LOCALHOST でない ({value or '未設定'})．"
            "同じネットワークの他の人の PC とトピックが混ざる",
            "echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc して "
            "source ~/.bashrc し，ブリッジなどを起動し直す")


def arduino_on_usb():
    """Tell whether an Arduino (USB vendor 2341) is on the USB bus.

    Returns
    -------
    bool or None
        None when it cannot be told (no /sys, e.g. not Linux).
    """
    paths = glob.glob("/sys/bus/usb/devices/*/idVendor")
    if not paths:
        return None
    for path in paths:
        with open(path) as f:
            if f.read().strip() == ARDUINO_USB_VENDOR:
                return True
    return False


def processes_using(port):
    """List the processes that have ``port`` open.

    Parameters
    ----------
    port : str
        Device path.

    Returns
    -------
    list of str
        "pid: command line" for each process; empty when none or unknown.
    """
    target = os.path.realpath(port)
    users = []
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_dir.split("/")[2]
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) != target:
                    continue
            except OSError:
                continue
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            except OSError:
                cmd = "?"
            users.append(f"{pid}: {cmd}")
            break
    return users


def find_port(report, port):
    """Find the board's serial port and check that it can be opened.

    Parameters
    ----------
    report : Report
        Where results go.
    port : str or None
        Port given on the command line, or None to look for /dev/ttyACM*.

    Returns
    -------
    str or None
        The port to use, or None when it cannot be used.
    """
    if port is None:
        ports = sorted(glob.glob("/dev/ttyACM*"))
        if not ports:
            on_usb = arduino_on_usb()
            if on_usb:
                report.ng("Arduino は USB で見えているが /dev/ttyACM* が無い",
                          "USB ケーブルを挿し直す．それでも無ければ TA を呼ぶ (ta call)")
            else:
                report.ng("/dev/ttyACM* が無い (Arduino が USB で見えていない)",
                          "USB ケーブルを挿し直す．充電専用のケーブルでないか確かめる")
            return None
        port = ports[0]
        if len(ports) > 1:
            report.info(f"シリアルポートが複数ある ({', '.join(ports)})．{port} を調べる．"
                        "違うときは --port で指定する")
    if not os.path.exists(port):
        report.ng(f"{port} が無い", "ls /dev/ttyACM* でポートの名前を確かめる")
        return None
    if not os.access(port, os.R_OK | os.W_OK):
        report.ng(f"{port} を読み書きする権限が無い",
                  "資料 tips/arduino-device-detection の権限 (dialout グループ) を見る")
        return None
    users = processes_using(port)
    if users:
        report.ng(f"{port} を他のプログラムが開いている: " + " / ".join(users),
                  "serial_bridge.py ならそのウィンドウで Ctrl-C，"
                  "Arduino IDE のシリアルモニタなら閉じてから，もう一度実行する")
        return None
    report.ok(f"{port} を使える")
    return port


def read_lines(ser, sec):
    """Read the lines the board sends during ``sec`` seconds.

    Parameters
    ----------
    ser : serial.Serial
        Open port.
    sec : float
        How long to listen.

    Returns
    -------
    list of str
        Lines without the line end; undecodable bytes are replaced.
    """
    lines = []
    end = time.monotonic() + sec
    while time.monotonic() < end:
        raw = ser.readline()
        if raw:
            lines.append(raw.decode("ascii", errors="replace").strip())
    return lines


def check_stream(report, lines):
    """Check that the serial firmware is running and sending data.

    Parameters
    ----------
    report : Report
        Where results go.
    lines : list of str
        What the board sent while listening.

    Returns
    -------
    bool
        True when the serial firmware's lines arrive.
    """
    flash = (f"cd {FIRMWARE_DIR}/uno_r4_serial && uvx --from platformio pio run -t upload "
             "(書き込む前に，何を書き込むかを学生に説明して了承を得る)")
    if not lines:
        report.ng("ボードから何も届かない (UNO R4 の USB とシリアルの変換役 ESP32-S3 が止まっていることが多い)",
                  "Arduino の USB ケーブルを抜いて挿し直してからもう一度 "
                  f"({FIRMWARE_DIR}/scripts/recover_usb.sh でも同じ．権限が無いと失敗する)．"
                  "それでも届かなければファームウェアを書き込む: " + flash)
        return False
    known = [ln for ln in lines if ln.split(" ")[0] in ("range", "pos", "led")]
    if not known:
        report.ng(f"ボードから届くのがシリアル版の行でない (例: {lines[0][:40]!r})．"
                  "micro-ROS 版や別のスケッチが入っている",
                  "シリアル版のファームウェアを書き込む: " + flash)
        return False
    count = len([ln for ln in lines if ln.startswith("range ")])
    report.ok(f"ファームウェアが動いている (range の行が {LISTEN_SEC:.0f} 秒で {count} 行)")
    return True


def run_diag(ser):
    """Ask the firmware for its wiring self-test.

    Parameters
    ----------
    ser : serial.Serial
        Open port.

    Returns
    -------
    dict or None
        ``diag`` values by key, or None when the firmware does not know
        ``diag`` (older than the self-test) or did not answer.
    """
    ser.reset_input_buffer()
    ser.write(b"diag\n")
    values = {}
    end = time.monotonic() + DIAG_TIMEOUT_SEC
    while time.monotonic() < end:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line == "# ignored: diag":
            return None
        if not line.startswith("diag "):
            continue
        _, key, *rest = line.split(" ")
        if key == "end":
            return values
        if rest:
            values[key] = int(rest[0])
    return None


def check_sensor(report, ser):
    """Check the HC-SR04 wiring with the firmware's self-test.

    Parameters
    ----------
    report : Report
        Where results go.
    ser : serial.Serial
        Open port.

    Returns
    -------
    dict or None
        The last ``diag`` values, or None when the self-test is unavailable.
    """
    unplug = "USB ケーブルを抜いてから"
    diag = None
    for _ in range(DIAG_TRIES):
        diag = run_diag(ser)
        if diag is None:
            report.ng("ファームウェアが diag (配線の自己診断) に答えない．古いファームウェア",
                      f"ファームウェアを書き込み直す: cd {FIRMWARE_DIR}/uno_r4_serial && "
                      "uvx --from platformio pio run -t upload (学生の了承を得てから)")
            return None
        if diag.get("echo_us", 0) > 0:
            break
        time.sleep(0.3)
    if diag.get("echo_us", 0) > 0:
        cm = diag["echo_us"] * SOUND_SPEED_CM_PER_US / 2
        report.ok(f"超音波センサ: Trig D2 / Echo D3 で反射が返る (前方 {cm:.1f} cm)")
    elif diag.get("swapped_us", -1) > 0:
        report.ng("超音波センサの Trig と Echo が逆 (Trig を D3，Echo を D2 にすると反射が返る)",
                  f"{unplug}，Trig を D2，Echo を D3 につなぎ直す")
    elif diag.get("echo_idle", 0) == 1:
        report.ng("D3 (Echo) が何もしていないのに HIGH のまま",
                  f"{unplug}，D3 に 5V などがつながっていないか，Echo の線を確かめる")
    else:
        report.ng("超音波センサから反射が返らない (Trig/Echo を入れ替えても返らない)",
                  "センサの前 10〜30 cm に手をかざしてもう一度実行する．それでも返らなければ，"
                  f"{unplug}，VCC が 3.3V，GND が GND，Trig が D2，Echo が D3 か，線が抜けていないかを確かめる．"
                  "配線が正しいのに返らなければ 5V でしか動かない古いセンサかもしれないので，"
                  "VCC を 5V につなぎ替える (サーボと 5V を分けるのにブレッドボードを使う)")
    servo_pin = diag.get("servo_pin")
    if servo_pin is not None:
        report.info(f"サーボの信号線はファームウェアでは D{servo_pin}．"
                    "サーボは角度を返さないので，配線は動かして目で確かめる (--servo)")
    return diag


def latest_pos(lines):
    """Return the last ``pos`` value in ``lines``.

    Parameters
    ----------
    lines : list of str
        Lines from the board.

    Returns
    -------
    int or None
        The angle [deg], or None when there is no ``pos`` line.
    """
    for line in reversed(lines):
        if line.startswith("pos "):
            try:
                return int(line[4:])
            except ValueError:
                continue
    return None


def check_servo(report, ser, ask):
    """Move the servo so that a person can see whether it turns.

    Parameters
    ----------
    report : Report
        Where results go.
    ser : serial.Serial
        Open port.
    ask : bool
        Ask on this terminal whether it moved; otherwise leave the question
        to whoever runs this (e.g. the AI TA asks the student).
    """
    start = latest_pos(read_lines(ser, 0.3))
    if start is None:
        start = 0
    print(f"サーボを {-SERVO_TEST_DEG}° → {SERVO_TEST_DEG}° → {start}° (元の角度) に動かす．サーボを見ていること")
    reached = []
    for deg in (-SERVO_TEST_DEG, SERVO_TEST_DEG, start):
        ser.write(f"cmd {deg}\n".encode("ascii"))
        reached.append((deg, latest_pos(read_lines(ser, SERVO_SETTLE_SEC))))
    if any(pos != deg for deg, pos in reached):
        report.ng(f"ファームウェアの pos が指令に追いつかない ({reached})",
                  "ファームウェアを書き込み直す．それでも同じなら TA を呼ぶ (ta call)")
        return
    report.ok("ファームウェアはサーボに角度を書いた (pos が指令どおりに変わった)")
    not_moving = ("USB ケーブルを抜いてから，サーボの信号線 (黄かオレンジ) が D8，"
                  "赤が 5V，茶が GND かを確かめる．スマホで配線の写真を撮って AI TA に見せてもよい")
    if not ask:
        report.info("サーボが左右に動いたかは，見ていた人に確かめる．動かなかったときは: " + not_moving)
        return
    answer = input("サーボは左右に動きましたか? [y/n] ").strip().lower()
    if answer.startswith("y"):
        report.ok("サーボが動いた (目で確認)")
    else:
        report.ng("サーボが動かない (ファームウェアは角度を書いている)", not_moving)


def main():
    """Run the checks and print a summary."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: the first /dev/ttyACM*)")
    parser.add_argument("--servo", action="store_true",
                        help="also move the servo -45 -> 45 deg and back")
    args = parser.parse_args()

    report = Report()
    check_ros_env(report)
    port = find_port(report, args.port)
    if port is not None:
        try:
            ser = serial.Serial(port, BAUD, timeout=0.1, exclusive=True)
        except serial.SerialException as e:
            report.ng(f"{port} を開けない: {e}",
                      "serial_bridge.py やシリアルモニタを止めてからもう一度実行する")
            ser = None
        if ser is not None:
            try:
                with ser:
                    ser.reset_input_buffer()
                    if check_stream(report, read_lines(ser, LISTEN_SEC)):
                        diag = check_sensor(report, ser)
                        if args.servo and diag is not None:
                            check_servo(report, ser, sys.stdin.isatty())
            except serial.SerialException as e:
                report.ng(f"調べている途中で {port} が使えなくなった: {e}",
                          "USB ケーブルが抜けかけていないか確かめ，挿し直してからもう一度実行する")
    print()
    if report.problems:
        print(f"問題 {len(report.problems)} 件．上から順に直す (1 つ直したらもう一度実行する):")
        for i, (text, action) in enumerate(report.problems, 1):
            print(f"  {i}. {text}\n     -> {action}")
        sys.exit(1)
    if not args.servo:
        print("見つかった問題はない．サーボも確かめるときは --servo を付けて実行する")
    else:
        print("見つかった問題はない")


if __name__ == "__main__":
    main()
