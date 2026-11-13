"""What the estimator must not claim from a wrapping potentiometer.

One fact underwrites all of it -- a wrap is one wheel revolution, established by
measuring wrap periods against commanded speed across ten offsets and four
wheels. Everything else is derived from that, and the tests below are mostly
about refusing to derive more than it supports.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from jedy_odometry.wheel_angle import WheelAngle  # noqa: E402

#: What the hardware actually does, measured. NOT 10667, which is what the ICS
#: pulse scale predicts and what an earlier version of this file assumed.
CYCLE = 7154


def revolution(wheel, cycle=CYCLE, step=115, start_time=0.0, dt=0.0025,
               rail_samples=0):
    """One revolution as the hardware reports it: a ramp, then a jump back."""
    t = start_time
    pulse = 0.0
    while pulse < cycle:
        wheel.update(3000 + min(pulse, cycle), t)
        pulse += step
        t += dt
    for _ in range(rail_samples):        # the stretch that does not advance
        wheel.update(3000 + cycle, t)
        t += dt
    wheel.update(3000.0, t)              # the wrap
    return t + dt


def test_the_scale_is_LEARNED_not_assumed():
    """The wire's units say 360 x 29.63 = 10667 pulses per revolution. The
    wheel does 7154. Taking the first number would have put a 33 % error into
    every distance this layer reports."""
    wheel = WheelAngle()
    assert wheel.pulses_per_rev is None      # nothing learned yet
    t = 0.0
    for _ in range(4):
        t = revolution(wheel, start_time=t)
    assert wheel.pulses_per_rev == pytest.approx(CYCLE, rel=0.03)


def test_revolutions_are_exact_before_any_scale_is_known():
    """Counting wraps needs no scale at all, so it works from the first one --
    which is what keeps the total distance right while the fraction is not."""
    wheel = WheelAngle()
    revolution(wheel)
    assert wheel.revolutions == 1
    assert wheel.pulses_per_rev is None
    assert wheel.angle == pytest.approx(2 * math.pi)


def test_going_back_the_way_it_came_counts_DOWN():
    wheel = WheelAngle()
    t = revolution(wheel)
    wheel.update(3000 + CYCLE, t)            # the wrap, the other way
    assert wheel.revolutions == 0


def test_speed_comes_from_the_wrap_period_and_is_signed():
    wheel = WheelAngle()
    t = 0.0
    for _ in range(3):
        t = revolution(wheel, start_time=t, dt=0.0025)
    # one revolution is 7154/115 = 62 samples of 2.5 ms = about 0.156 s
    assert wheel.speed_rev_s == pytest.approx(1.0 / 0.156, rel=0.05)
    assert wheel.speed_rev_s > 0


def test_no_speed_is_reported_before_two_wraps():
    """One wrap is a revolution; it takes two to make a period."""
    wheel = WheelAngle()
    revolution(wheel)
    assert wheel.speed_rev_s is None


def test_a_stalled_reading_HOLDS_the_angle_rather_than_extrapolating():
    """About 13 % of each revolution the reading does not advance. A held angle
    is visibly wrong to a consumer; a guessed one is not."""
    wheel = WheelAngle()
    t = 0.0
    for _ in range(3):
        t = revolution(wheel, start_time=t)
    for _ in range(20):
        t += 0.0025
        before = wheel.angle
        wheel.update(wheel._last_pulse, t)
        assert wheel.angle == before
    assert wheel.stalled is True


def test_advancing_fraction_is_about_the_PIPELINE_not_the_sensor():
    """It reads 0.44 on a real 400 Hz trace, because the board updates more
    slowly than that -- not because half the revolution is invisible. Pinned so
    nobody reads it as a sensor-quality number again."""
    wheel = WheelAngle()
    t = 0.0
    for _ in range(4):
        t = revolution(wheel, start_time=t, rail_samples=9)
    assert wheel.advancing_fraction == pytest.approx(62 / 71.0, abs=0.05)


def test_the_quality_number_is_the_spread_of_the_LEARNED_scale():
    """The revolution count cannot drift, so every uncertainty in a distance
    lives in the scale. Its spread is exactly that uncertainty."""
    wheel = WheelAngle()
    assert wheel.scale_spread is None            # nothing learned yet
    t = 0.0
    for _ in range(5):
        t = revolution(wheel, start_time=t)
    assert wheel.scale_spread < 0.05


def test_a_wheel_that_never_moved_reports_nothing_rather_than_confidence():
    wheel = WheelAngle()
    assert math.isnan(wheel.advancing_fraction)
    assert wheel.scale_spread is None


def test_direction_flips_the_angle_but_not_the_revolution_count():
    """The count is what the wheel did; `direction` is how this robot's URDF
    labels it. Mixing them would double-apply the sign somewhere downstream."""
    forward, reverse = WheelAngle(direction=1), WheelAngle(direction=-1)
    for wheel in (forward, reverse):
        revolution(wheel)
    assert forward.revolutions == reverse.revolutions == 1
    assert forward.angle == pytest.approx(-reverse.angle)


def test_a_stopped_wheel_would_report_its_last_speed_forever():
    """The estimate is a wrap PERIOD, so a stopped wheel produces nothing to
    contradict it. Only whoever commanded the stop knows, so it is their call
    -- and this test exists so nobody removes reset_speed() as redundant."""
    wheel = WheelAngle()
    t = 0.0
    for _ in range(3):
        t = revolution(wheel, start_time=t)
    assert wheel.speed_rev_s is not None
    for _ in range(500):                 # the wheel has stopped; no more wraps
        t += 0.0025
        wheel.update(wheel._last_pulse, t)
    assert wheel.speed_rev_s is not None, "nothing here can notice a stop"
    wheel.reset_speed()
    assert wheel.speed_rev_s is None


def ramp(wheel, start, stop, t, step=115, dt=0.0025):
    """Feed pulses from `start` towards `stop` (either way), no wrap."""
    sign = 1 if stop >= start else -1
    pulse = start
    while (stop - pulse) * sign > 0:
        wheel.update(pulse, t)
        pulse += sign * step
        t += dt
    wheel.update(stop, t)
    return t + dt


def test_a_short_move_advances_the_angle_with_the_nominal_scale():
    """The course drives in half revolutions. Before any wrap the nominal scale
    gives the fraction; without it the angle would not move at all."""
    wheel = WheelAngle(nominal_pulses_per_rev=CYCLE)
    ramp(wheel, 3000 + 1000, 3000 + 1000 + CYCLE / 2, 0.0)
    assert wheel.angle == pytest.approx(math.pi, rel=0.02)


def test_the_first_wrap_does_not_jump_the_angle():
    """Started mid-revolution: the pulses before the first wrap are not a whole
    revolution, so counting one there would add the part never travelled."""
    wheel = WheelAngle(nominal_pulses_per_rev=CYCLE)
    t = ramp(wheel, 3000 + CYCLE * 0.6, 3000 + CYCLE, 0.0)
    before = wheel.angle
    wheel.update(3000.0, t)                       # the wrap
    assert wheel.angle == pytest.approx(before, abs=0.05)
    assert wheel.angle == pytest.approx(0.4 * 2 * math.pi, rel=0.05)


def test_a_backward_wrap_keeps_the_angle_continuous():
    wheel = WheelAngle(nominal_pulses_per_rev=CYCLE)
    t = 0.0
    for _ in range(2):
        t = revolution(wheel, start_time=t)
    before = wheel.angle
    t = ramp(wheel, 3000.0, 3000.0 - 0.0, t)      # at the bottom
    wheel.update(3000 + CYCLE, t)                 # back through the wrap
    assert wheel.angle == pytest.approx(before, abs=0.05)
    t = ramp(wheel, 3000 + CYCLE, 3000 + CYCLE / 2, t + 0.0025)
    assert wheel.angle == pytest.approx(before - math.pi, rel=0.02)


def test_turning_back_between_wraps_does_not_poison_the_scale():
    """Measured 2026-11-13: forward and back moves of 15 cm drove the learned
    spread to 133 %. A cycle between a forward and a backward wrap is not a
    revolution and must not be learned, nor give a speed."""
    wheel = WheelAngle(nominal_pulses_per_rev=CYCLE)
    t = 0.0
    for _ in range(3):
        t = revolution(wheel, start_time=t)
    learned = wheel.pulses_per_rev
    for _ in range(4):                            # back and forth over the wrap
        wheel.update(3000 + CYCLE, t)             # backward wrap
        t = ramp(wheel, 3000 + CYCLE, 3000 + CYCLE - 500, t + 0.0025)
        t = ramp(wheel, 3000 + CYCLE - 500, 3000 + CYCLE, t)
        wheel.update(3000.0, t)                   # forward wrap
        t += 0.0025
        assert wheel.speed_rev_s is None
    assert wheel.pulses_per_rev == pytest.approx(learned)
    assert wheel.scale_spread < 0.05


def test_two_wraps_per_revolution_halve_the_angle_and_speed():
    """The RCB-4mini's reading wraps twice per wheel revolution."""
    one, two = WheelAngle(), WheelAngle(wraps_per_rev=2)
    for wheel in (one, two):
        t = 0.0
        for _ in range(3):
            t = revolution(wheel, start_time=t)
    assert two.revolutions == one.revolutions == 3
    assert two.angle == pytest.approx(one.angle / 2)
    assert two.speed_rev_s == pytest.approx(one.speed_rev_s / 2)
