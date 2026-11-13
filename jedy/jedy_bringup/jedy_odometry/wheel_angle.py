"""Wheel angle and speed from a wrapping potentiometer that measures itself.

Why this is not in actuo
------------------------
It is an estimator. Estimators live in a sibling layer, visibly separate from
the thing they estimate, so the driver goes on publishing the raw pulse and no
velocity at all and nothing downstream can mistake one for the other. The
numbers are this robot's too, and per-robot numbers stay out of the driver.

The one fact everything here rests on
-------------------------------------
**One wrap is one wheel revolution.** Established, not assumed: the calibration
sweep measured the time between wraps against commanded speed across ten
offsets and four wheels, and the revolution periods came out identical to
within 0 %.

Everything else is derived from that and needs no theory about what the pot is
geared to -- which matters, because the obvious theory was wrong. The reading
looked like it should advance 360 x 29.63 = 10667 pulses per revolution, being
the ordinary ICS scale. Measured, the cycle is **7154 pulses (spread 1.1 %)**.
A 33 % error, and it would have gone straight into every distance this layer
reports. So :class:`WheelAngle` does not take a scale: it **learns its own**
from the pulses accumulated between two wraps, per wheel, at runtime.

What it can and cannot do, measured on a real 8 s trace at 400 Hz
-----------------------------------------------------------------
* **Speed, from the wrap period: 0.17 - 0.32 % spread across the four wheels.**
  That is the good number, and it needs no scale at all -- a wrap is a
  revolution and the clock does the rest.
* **The cost is latency: one revolution**, about 1 s at full speed and 2 s near
  the deadband. Which is tolerable precisely because these servos hold whatever
  speed they were last commanded: a 0.2 % speed arriving a revolution late
  still integrates to the right distance while the base is running steadily.
  It is transients it cannot see.
* Part of each revolution the reading sits at an extreme and does not advance.
  For SPEED that is missing data, not wrong data, and it biases nothing: the
  wrap period still measures a whole revolution. For the within-revolution ANGLE it
  is an unknown, so the fractional part simply stops advancing there and
  :attr:`stalled` says so rather than interpolating.
* Replayed against the trace, the revolution count matches the wraps exactly
  (9, 8, 8, 8) and the speed matches an independent 1/period to 0.2 %:
  1.086 / 0.957 / 0.961 / 0.965 against 1.086 / 0.958 / 0.963 / 0.965.
  The learned scale came out 7178 / 7063 / 6983 / 6813 pulses per revolution --
  PER WHEEL, differing by 5 %, which is one more reason not to share a
  constant between them.

Short moves and reversals, measured on the robot 2026-11-13
------------------------------------------------------------
The course drives in short moves (0.1 m/s for a second or two, about half a
revolution) and turns back often. Two things went wrong there when the scale
was only learned at runtime: until two wraps had been seen the angle advanced
in whole revolutions (16 cm of travel per step), and a "cycle" between a
forward wrap and a backward one is not a revolution at all, so the learned
scale spread reached 133 %. Hence:

* ``nominal_pulses_per_rev`` (the 7154 above) gives the fraction from the
  first sample; the learned per-wheel scale replaces it once there is one.
* A cycle is learned, and a speed taken, only between two wraps in the SAME
  direction, and a learned cycle more than 20 % off the nominal is dropped.
* A backward wrap leaves the reading at the top of its range, so the
  accumulated pulses restart at one revolution there, not at zero.
* The first wrap does not jump the angle: where in the revolution the wheel
  started is unknown, so the angle is kept continuous across it.
* On the RCB-4mini the reading spans 2590-12477 and wraps every ~9850 pulses,
  and at a commanded 0.63 rev/s the wraps came every 0.77 s: TWO wraps per
  wheel revolution, if the base runs at its commanded speed (it looked like
  it did; not measured against the floor). ``wraps_per_rev`` says so; a
  "revolution" in the attributes below is then one wrap, and ``angle`` and
  ``speed_rev_s`` are converted to the wheel's.

A sub-revolution speed estimator -- fitting a slope through the clean stretches
-- would cut the latency and is deliberately absent: the first attempt at it
produced negative speeds and 3000 % spreads, and a broken estimator that
happens to be fast is worse than an honest one that is slow.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

#: Fraction of a learned cycle a step must exceed to count as a wrap. Before a
#: cycle is known, :data:`BOOTSTRAP_WRAP` is used instead.
WRAP_FRACTION = 0.5

#: Pulse step that counts as a wrap before the cycle has been learned. Well
#: above the ~130 pulses a step reaches at full speed and well below the ~7150
#: of a real wrap, so the bootstrap cannot pick either by accident.
BOOTSTRAP_WRAP = 2000


@dataclass
class WheelAngle:
    """Continuous wheel angle and speed from one wheel's raw pulse stream.

    Parameters
    ----------
    direction : int
        +1 or -1, matching the robot config's `direction` for this wheel, so
        the angle advances the way the URDF joint does.
    cycle_samples : int
        How many wrap-to-wrap cycles to keep when learning the scale. The
        median of these is :attr:`pulses_per_rev`; a median rather than a mean
        because a wrap detected one sample early costs a whole step.
    """

    direction: int = 1
    cycle_samples: int = 8
    #: Pulses per revolution to use until this wheel has learned its own
    #: (``None``: count whole revolutions only until then).
    nominal_pulses_per_rev: Optional[float] = None
    #: Wraps per wheel revolution (1 on the ARM-H7 trace above, 2 on the
    #: RCB-4mini). ``revolutions`` and ``pulses_per_rev`` count wraps.
    wraps_per_rev: int = 1
    #: A learned cycle further than this fraction from the nominal is a
    #: misread wrap, not a revolution, and is not learned.
    cycle_tolerance: float = 0.2

    #: Whole revolutions completed. EXACT -- it counts wraps, and a wrap is a
    #: revolution. Available from the first wrap, before any scale is known.
    revolutions: int = 0
    #: rev/s from the last wrap-to-wrap period, or ``None`` until two wraps have
    #: been seen. Signed.
    speed_rev_s: Optional[float] = None
    #: True while the reading is not advancing (an extreme, or a stationary
    #: wheel). The fractional angle is held, never extrapolated.
    stalled: bool = False

    _cycles: List[float] = field(default_factory=list, repr=False)
    _since_wrap: float = field(default=0.0, repr=False)
    _last_pulse: Optional[float] = field(default=None, repr=False)
    _last_wrap_time: Optional[float] = field(default=None, repr=False)
    _samples: int = field(default=0, repr=False)
    #: Direction of the last wrap (True = forward), ``None`` before the first.
    _last_wrap_forward: Optional[bool] = field(default=None, repr=False)
    #: ``_since_wrap`` right after the last wrap: 0 after a forward one, a
    #: revolution after a backward one.
    _cycle_start: float = field(default=0.0, repr=False)
    #: Radians added so that the first wrap does not jump the angle.
    _offset: float = field(default=0.0, repr=False)
    _stalled_samples: int = field(default=0, repr=False)

    @property
    def pulses_per_rev(self) -> Optional[float]:
        """Pulses the reading advances in one revolution, LEARNED from this
        wheel's own wraps. ``None`` until two wraps have been seen.

        Not a constant and not derived from the wire's units: those said 10667
        and the wheel does 7154.
        """
        if not self._cycles:
            return self.nominal_pulses_per_rev
        ordered = sorted(abs(c) for c in self._cycles)
        return ordered[len(ordered) // 2]

    @property
    def angle(self) -> float:
        """Radians turned since the first sample, direction applied.

        Whole revolutions are exact. The fraction of the current revolution is
        added once a scale is known (learned, or the nominal), and it stops
        advancing while the reading does -- so this is never ahead of the
        evidence.
        """
        return self.direction * (self._raw_angle() + self._offset) / self.wraps_per_rev

    def _raw_angle(self) -> float:
        whole = self.revolutions * 2.0 * math.pi
        per_rev = self.pulses_per_rev
        if per_rev:
            whole += self._since_wrap / per_rev * 2.0 * math.pi
        return whole

    @property
    def advancing_fraction(self) -> float:
        """Fraction of samples in which the reading changed at all.

        A diagnostic of the PIPELINE, not of the sensor, and the distinction is
        worth keeping straight: it reads 0.44 replaying a 400 Hz trace, because
        the board updates the angle more slowly than that and over half the
        samples repeat the last value. It is not saying the wheel spent half its
        revolution invisible. An earlier version of this docstring claimed it
        settled near 0.87, which was true only at the 100 Hz it was measured at.

        Useful for spotting a wheel that has stopped, or a sample rate wasted on
        a board that cannot keep up. For "how good is this estimate", see
        :attr:`scale_spread`.
        """
        if not self._samples:
            return float("nan")
        return 1.0 - self._stalled_samples / self._samples

    @property
    def scale_spread(self) -> Optional[float]:
        """How much this wheel's learned revolutions disagree, as a fraction.

        THIS is the quality number, because it is the one thing the estimate
        rests on that could be wrong. The revolution count cannot drift -- it
        counts wraps -- so all the uncertainty in a distance lives in the
        learned scale, and its spread is exactly that uncertainty. Measured on
        the robot: about 1 %.

        ``None`` until two cycles have been learned.
        """
        if len(self._cycles) < 2:
            return None
        values = [abs(c) for c in self._cycles]
        mean = sum(values) / len(values)
        if mean == 0:
            return None
        variance = sum((v - mean) ** 2 for v in values) / len(values)
        return math.sqrt(variance) / mean

    def update(self, pulse: float, stamp: float) -> float:
        """Feed one reading and its timestamp; return the angle in radians."""
        pulse = float(pulse)
        self._samples += 1
        if self._last_pulse is None:
            self._last_pulse = pulse
            return self.angle

        delta = pulse - self._last_pulse
        self._last_pulse = pulse

        per_rev = self.pulses_per_rev
        threshold = per_rev * WRAP_FRACTION if per_rev else BOOTSTRAP_WRAP
        if abs(delta) >= threshold:
            # A wrap. Its SIGN says which way the wheel went: the reading
            # climbs through a revolution and drops back, so a negative jump
            # completes a forward one.
            self.stalled = False
            forward = delta < 0
            before = self._raw_angle() + self._offset
            first = self._last_wrap_forward is None
            same_way = self._last_wrap_forward == forward
            if same_way and self._last_wrap_time is not None:
                period = stamp - self._last_wrap_time
                if period > 0:
                    self.speed_rev_s = ((1.0 if forward else -1.0) / period
                                        / self.wraps_per_rev)
                    # The pulses that accumulated over exactly one revolution
                    # ARE this wheel's scale. Learned here and nowhere else.
                    self._learn(abs(self._since_wrap - self._cycle_start))
            elif not same_way:
                # A turn-back between these two wraps: neither the period nor
                # the pulses are a revolution.
                self.speed_rev_s = None
            self.revolutions += 1 if forward else -1
            self._last_wrap_time = stamp
            self._last_wrap_forward = forward
            # Forward, the reading drops to the bottom of its range; backward,
            # it jumps to the top, a revolution's pulses on from the bottom.
            per_rev = self.pulses_per_rev
            self._cycle_start = 0.0 if forward or not per_rev else per_rev
            self._since_wrap = self._cycle_start
            if first and per_rev:
                # Where in the revolution the wheel started is unknown, so
                # the pulses before this wrap were not a whole revolution.
                self._offset = before - self._raw_angle()
            return self.angle

        self.stalled = delta == 0.0
        if self.stalled:
            self._stalled_samples += 1
        self._since_wrap += delta
        return self.angle

    def _learn(self, cycle: float) -> None:
        nominal = self.nominal_pulses_per_rev
        if nominal and abs(cycle - nominal) > self.cycle_tolerance * nominal:
            return
        self._cycles.append(cycle)
        del self._cycles[:-self.cycle_samples]

    def reset_speed(self) -> None:
        """Forget the last speed, for when the wheel has been commanded to stop.

        Needed because the estimate is a wrap period: a stopped wheel produces
        no wraps, so nothing would ever contradict the last one and this class
        would go on reporting the speed it had while rolling. Whoever commands
        the stop is the only one who knows, so it is their call to make.
        """
        self.speed_rev_s = None
        self._last_wrap_time = None
