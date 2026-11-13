"""The simulators' servo rule (jedy_sim.sim_servo.SimServoSwitch)."""

import math

from jedy_sim.sim_servo import SimServoSwitch
from jedy_sim.sim_servo import wheel_speeds


def make():
    return SimServoSwitch(['a', 'b'], ['w'])


def test_starts_driven_and_off_is_always_allowed():
    s = make()
    assert s.released == []
    ok, reason = s.disable(['b'])
    assert ok and s.released == ['b'] and 'still holding: a, w' in reason
    ok, _ = s.disable()
    assert ok and s.released == ['a', 'b', 'w']


def test_unknown_joint_is_refused():
    s = make()
    ok, reason = s.disable(['nope'])
    assert not ok and 'unknown' in reason and s.released == []


def test_on_refused_when_the_controller_commands_elsewhere():
    s = make()
    s.disable()
    ok, reason = s.enable({'a': 0.0, 'b': 1.0}, {'a': 0.0, 'b': 0.5})
    assert not ok and 'b is at 1.0000' in reason
    assert s.released == ['a', 'b', 'w']   # nothing changed on a refusal
    ok, _ = s.enable({'a': 0.0, 'b': 1.0}, {'a': 0.0, 'b': 0.99})
    assert ok and s.released == []


def test_on_only_judges_the_requested_joints():
    s = make()
    s.disable()
    ok, _ = s.enable({'a': 0.0, 'b': 1.0}, {'a': 0.0, 'b': 0.0}, ['a'])
    assert ok and s.released == ['b', 'w']


def test_unknown_measurement_is_not_safe():
    s = make()
    s.disable(['a'])
    ok, _ = s.enable({'a': math.nan}, {'a': 0.0})
    assert not ok
    ok, _ = s.enable({}, {'a': 0.0})
    assert not ok
    ok, reason = s.enable({}, {'a': 0.0}, force=True)
    assert ok and 'FORCED' in reason


def test_wheel_refused_while_moving():
    s = make()
    s.disable(['w'])
    ok, reason = s.enable({}, {'w': 3.0}, ['w'])
    assert not ok and 'asking for motion' in reason
    ok, _ = s.enable({}, {}, ['w'])   # no command: no motion
    assert ok


def test_wheel_speeds_invert_the_forward_kinematics():
    r, k = 0.03, 0.2
    fl, fr, rl, rr = wheel_speeds(0.1, 0.05, 0.3, r, k)
    assert math.isclose(r * (fl + fr + rl + rr) / 4, 0.1)
    assert math.isclose(r * (-fl + fr + rl - rr) / 4, 0.05)
    assert math.isclose(r * (-fl + fr - rl + rr) / (4 * k), 0.3)


def test_missing_servo_is_limp_and_refuses_on_even_forced():
    sw = SimServoSwitch(['a', 'b'], ['w'], missing=['b'])
    assert sw.released == ['b']
    assert not sw.is_present('b') and sw.is_present('a')
    measured = {'a': 0.0, 'b': 0.0}
    commanded = {'a': 0.0, 'b': 0.0}
    ok, reason = sw.enable(measured, commanded, force=True)
    assert not ok and 'b not answering' in reason
    ok, _ = sw.enable(measured, commanded, ['a'])
    assert ok


def test_missing_servo_must_be_a_joint():
    import pytest
    with pytest.raises(ValueError):
        SimServoSwitch(['a'], missing=['nope'])
