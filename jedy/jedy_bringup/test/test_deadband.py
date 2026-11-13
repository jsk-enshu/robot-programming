"""Mapping a stick onto what the base can actually do.

The numbers: this base's wheels do not turn below ~2.83 rad/s and top out near
6.85, which is 0.071 to 0.173 m/s (25.25 mm wheels). Everything here is about that gap.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from jedy_teleop.deadband import scale_to_usable, wheel_speeds  # noqa: E402

R, K = 0.03, 0.11975
MIN, MAX = 2.83, 6.85


def scaled(vx, vy=0.0, wz=0.0):
    return scale_to_usable(vx, vy, wz, R, K, MIN, MAX)


def test_a_stopped_stick_stays_stopped():
    """Nothing is scaled up out of nothing -- a released stick must not produce
    the minimum speed."""
    vx, vy, wz, note = scaled(0.0)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert note is None


def test_a_tiny_deflection_is_lifted_to_the_slowest_the_base_can_go():
    """Rather than asking for a speed the wheels ignore, which is a dead stick
    followed by a lurch."""
    vx, _, _, note = scaled(0.001)
    assert vx == pytest.approx(MIN * R, rel=0.01)
    assert "deadband" in note


def test_direction_survives_the_scaling():
    """The whole point. On a mecanum base the ratio between wheels IS the
    heading, so scaling the twist as a whole is the only safe way to change its
    speed."""
    before = wheel_speeds(0.02, 0.01, 0.3, R, K)
    vx, vy, wz, _ = scaled(0.02, 0.01, 0.3)
    after = wheel_speeds(vx, vy, wz, R, K)
    ratios = [a / b for a, b in zip(after, before) if abs(b) > 1e-9]
    assert max(ratios) == pytest.approx(min(ratios), rel=1e-6)


def test_a_full_stick_is_capped_at_the_top_and_not_beyond():
    vx, _, _, note = scaled(10.0)
    assert vx == pytest.approx(MAX * R, rel=0.01)
    assert "scaled down" in note


def test_a_twist_already_inside_the_range_is_left_alone():
    vx, vy, wz, note = scaled(0.15)
    assert (vx, vy, wz) == (0.15, 0.0, 0.0)
    assert note is None


def test_a_mixed_twist_can_put_ONE_wheel_in_the_deadband():
    """Which is the failure separate per-axis scaling would miss: every
    component looks fine and one wheel still stops."""
    # 0.10 m/s forward while turning at 0.5 rad/s: two wheels at 5.33 rad/s,
    # the other two at 1.34 -- comfortably inside and comfortably below.
    speeds = [abs(s) for s in wheel_speeds(0.10, 0.0, 0.5, R, K)]
    assert min(speeds) < MIN < max(speeds), "this twist should straddle it"
    vx, vy, wz, note = scaled(0.10, 0.0, 0.5)
    assert note is not None


def test_when_both_bounds_cannot_be_met_the_FAST_one_wins():
    """Exceeding the top does not go faster, it saturates one wheel while the
    others keep their speed -- which changes the direction. Being slower than
    asked does not."""
    vx, vy, wz, note = scaled(0.10, 0.0, 0.5)
    speeds = [abs(s) for s in wheel_speeds(vx, vy, wz, R, K)]
    assert max(speeds) <= MAX * 1.001
    assert "cannot satisfy both" in note or "scaled down" in note


def test_the_note_says_what_happened_rather_than_going_quiet():
    """A driver who asked for one speed and got another should be able to find
    out why without reading the source."""
    for arguments in ((0.001,), (10.0,), (0.10, 0.0, 0.5)):
        assert scaled(*arguments)[3] is not None
