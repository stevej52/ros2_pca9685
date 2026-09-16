# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Electronic speed controller (ESC) behaviour for continuous channels.

Hobby ESCs are not plain servos: they need an arming signal after power-up,
they ignore throttle close to neutral, and many treat the first reverse
command after driving forward as a brake. All of that is described by an
``EscConfig`` and carried out by an ``EscSequencer``, which is plain Python
with the time passed in, so it can be tested without ROS or hardware.
"""

from __future__ import annotations

from dataclasses import dataclass

# A sequence step: the throttle to send (-1.0 to 1.0) and for how many seconds.
Step = tuple[float, float]

FORWARD = 'forward'
REVERSE = 'reverse'


@dataclass(frozen=True)
class EscConfig:
    """How an ESC wants to be talked to."""

    arming: tuple[Step, ...] = ()  # steps sent at start-up before commands are accepted
    to_reverse: tuple[Step, ...] = ()  # steps sent before the first reverse after forward
    to_forward: tuple[Step, ...] = ()  # steps sent before the first forward after reverse
    deadband: float = 0.0  # commands within this distance of zero are sent as neutral
    forward_start: float = 0.0  # output at which the motor actually starts moving forward
    reverse_start: float = 0.0  # same for reverse, as a positive number

    @property
    def configured(self) -> bool:
        """Return whether anything differs from a plain continuous channel."""
        return self != EscConfig()

    def shape(self, throttle: float, min_limit: float, max_limit: float) -> float:
        """
        Map a command onto the band the ESC responds to.

        Commands inside the dead-band become neutral. Anything above it is
        spread linearly between ``forward_start`` and ``max_limit`` (or
        ``reverse_start`` and ``min_limit``), so the smallest command that is
        not neutral just makes the motor move and the limits stay the most the
        ESC is ever sent.
        """
        if abs(throttle) <= self.deadband:
            return 0.0
        if throttle > 0.0:
            top = max_limit
            if top <= self.deadband:
                return 0.0
            fraction = (min(throttle, top) - self.deadband) / (top - self.deadband)
            return self.forward_start + fraction * (top - self.forward_start)
        bottom = -min_limit
        if bottom <= self.deadband:
            return 0.0
        fraction = (min(-throttle, bottom) - self.deadband) / (bottom - self.deadband)
        return -(self.reverse_start + fraction * (bottom - self.reverse_start))


def sequence_duration(steps: tuple[Step, ...]) -> float:
    """Return the total length of a sequence in seconds."""
    return sum(seconds for _, seconds in steps)


class EscSequencer:
    """
    Decide what an ESC channel outputs at any moment.

    The node stores the throttle it wants with ``set_wanted`` and asks
    ``output`` for the value to send. While a sequence runs (arming, or the
    steps that precede a change of direction) the sequence wins; afterwards
    the wanted throttle, shaped for the dead-band, is sent.
    """

    def __init__(self, config: EscConfig, min_limit: float, max_limit: float) -> None:
        self._config = config
        self._min_limit = min_limit
        self._max_limit = max_limit
        self._wanted = 0.0
        self._mode = FORWARD  # the direction the ESC believes it is set up for
        self._steps: list[tuple[float, float]] = []  # (throttle, time the step ends)
        self._sequence = ''
        self._pending_mode: str | None = None

    @property
    def config(self) -> EscConfig:
        """Return the current configuration."""
        return self._config

    @property
    def busy(self) -> bool:
        """Return whether a sequence is running."""
        return bool(self._steps)

    @property
    def sequence(self) -> str:
        """Return the name of the running sequence, or an empty string."""
        return self._sequence

    @property
    def mode(self) -> str:
        """Return the direction the ESC is believed to be in."""
        return self._mode

    @property
    def wanted(self) -> float:
        """Return the last throttle requested."""
        return self._wanted

    def reconfigure(self, config: EscConfig, min_limit: float, max_limit: float) -> None:
        """Adopt new settings without forgetting the state of the ESC."""
        self._config = config
        self._min_limit = min_limit
        self._max_limit = max_limit

    def arm(self, now: float) -> bool:
        """Start the arming sequence; return False when none is configured."""
        if not self._config.arming:
            return False
        self._start('arming', self._config.arming, now)
        self._pending_mode = FORWARD
        return True

    def cancel(self) -> None:
        """Abandon any running sequence."""
        self._steps = []
        self._sequence = ''
        self._pending_mode = None

    def set_wanted(self, throttle: float) -> None:
        """Record the throttle the user wants; it is sent once no sequence runs."""
        self._wanted = throttle

    def output(self, now: float) -> float:
        """Return the throttle to send at time ``now``."""
        while self._steps and now >= self._steps[0][1]:
            self._steps.pop(0)
        if not self._steps and self._pending_mode is not None:
            self._mode = self._pending_mode
            self._pending_mode = None
            self._sequence = ''
        if self._steps:
            return self._steps[0][0]
        needed = self._direction(self._wanted)
        if needed is not None and needed != self._mode:
            steps = self._config.to_reverse if needed == REVERSE else self._config.to_forward
            if steps:
                self._start('to_' + needed, steps, now)
                self._pending_mode = needed
                return self._steps[0][0]
            self._mode = needed
        return self._config.shape(self._wanted, self._min_limit, self._max_limit)

    def _direction(self, throttle: float) -> str | None:
        if throttle > self._config.deadband:
            return FORWARD
        if throttle < -self._config.deadband:
            return REVERSE
        return None

    def _start(self, name: str, steps: tuple[Step, ...], now: float) -> None:
        ends_at = now
        self._steps = []
        for throttle, seconds in steps:
            ends_at += seconds
            self._steps.append((throttle, ends_at))
        self._sequence = name
