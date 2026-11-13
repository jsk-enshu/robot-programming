"""Turning a stick deflection into a twist this base can actually honour.

The problem, measured
---------------------
Jedy's wheels do not turn at all below about 9 degrees of command offset, and
they top out around 20. In base terms that is roughly **0.071 to 0.173 m/s**: the
bottom 42 % of a proportional stick asks for speeds no wheel will produce, and
the actuator correctly commands a stop and logs a refusal. A driver feels that
as a stick with a large dead zone and then a lurch.

Scaling each axis separately does not fix it either, and on a mecanum base it
makes something worse. The wheel speeds are a MIX of vx, vy and wz, so a twist
whose components are all comfortably above the minimum can still put one wheel
below it -- and that wheel stops while the other three turn. The ratio between
the wheels is the heading, so the result is not a slow robot, it is one going
somewhere else.

What this does instead
----------------------
Work out the four wheel speeds the twist implies, find the slowest one that is
not zero, and scale the WHOLE twist until it clears the minimum -- or down until
the fastest is within the maximum. Scaling the whole twist changes the speed and
leaves every ratio alone, so the direction the driver asked for is exactly the
direction they get.

If both bounds cannot be met at once -- a twist whose wheels differ by more than
the usable speed range, which a hard turn while creeping forward will do --
the fast bound wins and :func:`scale_to_usable` says so. Better a robot that
does not quite crawl than one that quietly drives off at an angle.
"""

from __future__ import annotations

from typing import Optional, Tuple


def wheel_speeds(vx: float, vy: float, wz: float, radius: float,
                 centre_projection: float) -> Tuple[float, float, float, float]:
    """The four wheel speeds (rad/s) a body twist implies, FL, FR, RR, RL.

    The same inverse kinematics mecanum_drive_controller applies, repeated here
    because it does not expose it and because this has to know the answer
    BEFORE the controller acts on it.
    """
    k = centre_projection
    return (
        (vx - vy - k * wz) / radius,
        (vx + vy + k * wz) / radius,
        (vx - vy + k * wz) / radius,
        (vx + vy - k * wz) / radius,
    )


def scale_to_usable(vx: float, vy: float, wz: float, radius: float,
                    centre_projection: float, min_wheel_rad_s: float,
                    max_wheel_rad_s: float
                    ) -> Tuple[float, float, float, Optional[str]]:
    """Scale a twist so every moving wheel is inside what the servos can do.

    Returns ``(vx, vy, wz, note)``. ``note`` is ``None`` when the twist was
    usable as given, and otherwise says what had to change -- because a driver
    who asked for one speed and got another should be able to find out why.
    """
    speeds = [abs(s) for s in wheel_speeds(vx, vy, wz, radius,
                                           centre_projection)]
    moving = [s for s in speeds if s > 1e-9]
    if not moving:
        return 0.0, 0.0, 0.0, None

    slowest, fastest = min(moving), max(moving)
    note = None
    scale = 1.0
    if slowest < min_wheel_rad_s:
        scale = min_wheel_rad_s / slowest
        note = "scaled up x{:.2f}: the slowest wheel was below the deadband".format(
            scale)
    if fastest * scale > max_wheel_rad_s:
        # The fast bound is the hard one: exceeding it does not go faster, it
        # SATURATES that wheel while the others keep their speed, which changes
        # the direction. Being slower than asked does not.
        capped = max_wheel_rad_s / fastest
        note = ("scaled down x{:.2f} to keep the fastest wheel inside its "
                "range".format(capped)
                if scale <= 1.0 else
                "cannot satisfy both ends: this twist spreads the wheels wider "
                "than the servos' usable range, so it is capped at the top and "
                "the slowest wheel will not turn")
        scale = capped
    return vx * scale, vy * scale, wz * scale, note
