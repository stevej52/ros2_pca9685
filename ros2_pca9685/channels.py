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
Per-channel configuration and the maths that turns commands into pulses.

Everything in here is plain Python so it can be unit tested without ROS.
The ROS node declares one parameter per entry of ``CHANNEL_PARAMS`` for every
configured channel and hands the values to ``parse_channel``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Mapping

from ros2_pca9685.esc import EscConfig, sequence_duration, Step
from ros2_pca9685.pca9685 import NUM_CHANNELS

SERVO = 'servo'
CONTINUOUS = 'continuous'
PWM = 'pwm'
CHANNEL_TYPES = (SERVO, CONTINUOUS, PWM)

SHUTDOWN_OFF = 'off'
SHUTDOWN_HOME = 'home'
SHUTDOWN_HOLD = 'hold'
SHUTDOWN_MODES = (SHUTDOWN_OFF, SHUTDOWN_HOME, SHUTDOWN_HOLD)

TWIST_AXES = ('linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z')
ESC_SEQUENCES = ('arming', 'to_reverse', 'to_forward')

NAME_PATTERN = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')

# Per-channel parameters and their defaults.  ``None`` marks a value that is
# either required (``channel``) or derived from the other values while parsing.
CHANNEL_PARAMS: dict[str, object] = {
    'channel': None,
    'type': SERVO,
    'min_pulse_us': 750.0,
    'max_pulse_us': 2250.0,
    'neutral_pulse_us': None,
    'min_angle': 0.0,
    'max_angle': 180.0,
    'min_limit': None,
    'max_limit': None,
    'invert': False,
    'home': None,
    'home_on_start': False,
    'timeout': 0.0,
    'on_shutdown': SHUTDOWN_OFF,
    'joint': '',
    **{f'twist.{axis}': 0.0 for axis in TWIST_AXES},
    **{f'esc.{name}.values': [] for name in ESC_SEQUENCES},
    **{f'esc.{name}.durations': [] for name in ESC_SEQUENCES},
    'esc.deadband': 0.0,
    'esc.forward_start': 0.0,
    'esc.reverse_start': 0.0,
}

_UNITS = {SERVO: 'deg', CONTINUOUS: 'throttle', PWM: 'duty'}
_COMMAND_TOPICS = {SERVO: 'angle', CONTINUOUS: 'throttle', PWM: 'duty_cycle'}


class ConfigError(ValueError):
    """Raised for an invalid configuration value."""


@dataclass(frozen=True)
class ChannelConfig:
    """Validated configuration of one PCA9685 output."""

    name: str
    channel: int
    kind: str
    min_pulse_us: float
    max_pulse_us: float
    neutral_pulse_us: float
    min_angle: float
    max_angle: float
    min_limit: float
    max_limit: float
    invert: bool
    home: float
    home_on_start: bool
    timeout: float
    on_shutdown: str
    joint: str
    twist_gains: tuple[tuple[str, float], ...]
    esc: EscConfig

    @property
    def units(self) -> str:
        """Return the unit of the channel's command values."""
        return _UNITS[self.kind]

    @property
    def command_topic(self) -> str:
        """Return the name of the channel's main command topic."""
        return _COMMAND_TOPICS[self.kind]

    @property
    def twist_driven(self) -> bool:
        """Return whether the channel follows the Twist (cmd_vel) topic."""
        return bool(self.twist_gains)

    @property
    def is_esc(self) -> bool:
        """Return whether the channel is a continuous channel with ESC behaviour."""
        return self.kind == CONTINUOUS and self.esc.configured

    def clamp(self, value: float) -> float:
        """Limit a command to the configured travel."""
        return min(max(value, self.min_limit), self.max_limit)

    def pulse_width_us(self, value: float) -> float:
        """Return the pulse width for a command (servo and continuous channels)."""
        value = self.clamp(value)
        if self.kind == SERVO:
            fraction = (value - self.min_angle) / (self.max_angle - self.min_angle)
            if self.invert:
                fraction = 1.0 - fraction
            return self.min_pulse_us + fraction * (self.max_pulse_us - self.min_pulse_us)
        if self.kind == CONTINUOUS:
            if self.invert:
                value = -value
            if value >= 0.0:
                return self.neutral_pulse_us + value * (self.max_pulse_us - self.neutral_pulse_us)
            return self.neutral_pulse_us + value * (self.neutral_pulse_us - self.min_pulse_us)
        raise ValueError(f"'{self.name}' is a pwm channel; use duty_cycle()")

    def duty_cycle(self, value: float) -> float:
        """Return the duty cycle for a command (pwm channels)."""
        if self.kind != PWM:
            raise ValueError(f"'{self.name}' is not a pwm channel; use pulse_width_us()")
        value = self.clamp(value)
        return 1.0 - value if self.invert else value

    def twist_value(self, axes: Mapping[str, float]) -> float:
        """Return the command for a Twist message, given as a mapping of axis name to value."""
        value = self.home
        for axis, gain in self.twist_gains:
            value += gain * axes.get(axis, 0.0)
        return self.clamp(value)

    def as_params(self) -> dict[str, object]:
        """Return the configuration as parameter values (the inverse of ``parse_channel``)."""
        params: dict[str, object] = {
            'channel': self.channel,
            'type': self.kind,
            'min_pulse_us': self.min_pulse_us,
            'max_pulse_us': self.max_pulse_us,
            'neutral_pulse_us': self.neutral_pulse_us,
            'min_angle': self.min_angle,
            'max_angle': self.max_angle,
            'min_limit': self.min_limit,
            'max_limit': self.max_limit,
            'invert': self.invert,
            'home': self.home,
            'home_on_start': self.home_on_start,
            'timeout': self.timeout,
            'on_shutdown': self.on_shutdown,
            'joint': self.joint,
        }
        gains = dict(self.twist_gains)
        for axis in TWIST_AXES:
            params[f'twist.{axis}'] = gains.get(axis, 0.0)
        for name in ESC_SEQUENCES:
            steps = getattr(self.esc, name)
            params[f'esc.{name}.values'] = [throttle for throttle, _ in steps]
            params[f'esc.{name}.durations'] = [seconds for _, seconds in steps]
        params['esc.deadband'] = self.esc.deadband
        params['esc.forward_start'] = self.esc.forward_start
        params['esc.reverse_start'] = self.esc.reverse_start
        return params

    def describe(self) -> str:
        """Return a one-line human readable summary."""
        parts = [f'output {self.channel}', self.kind]
        if self.kind == SERVO:
            parts.append(f'{self.min_pulse_us:g}-{self.max_pulse_us:g} us over '
                         f'{self.min_angle:g}-{self.max_angle:g} deg')
        elif self.kind == CONTINUOUS:
            parts.append(
                f'{self.min_pulse_us:g}/{self.neutral_pulse_us:g}/{self.max_pulse_us:g} us')
        parts.append(f'limits {self.min_limit:g}..{self.max_limit:g} {self.units}')
        if self.invert:
            parts.append('inverted')
        parts.append(f'home {self.home:g}' + (' (on start)' if self.home_on_start else ''))
        if self.timeout > 0.0:
            parts.append(f'timeout {self.timeout:g} s')
        if self.twist_gains:
            gains = ' '.join(f'{axis}*{gain:g}' for axis, gain in self.twist_gains)
            parts.append(f'twist {gains}')
        if self.joint != self.name:
            parts.append(f'joint {self.joint}')
        if self.is_esc:
            parts.append(self._describe_esc())
        parts.append(f'on shutdown {self.on_shutdown}')
        return ', '.join(parts)

    def _describe_esc(self) -> str:
        details = []
        for name in ESC_SEQUENCES:
            steps = getattr(self.esc, name)
            if steps:
                details.append(f'{name} {len(steps)} step(s) {sequence_duration(steps):g} s')
        if self.esc.deadband > 0.0:
            details.append(f'deadband {self.esc.deadband:g}')
        if self.esc.forward_start > 0.0 or self.esc.reverse_start > 0.0:
            details.append(f'starts {self.esc.forward_start:g}/{self.esc.reverse_start:g}')
        return 'ESC ' + ' '.join(details)


def validate_channel_name(name: object) -> str:
    """Check that a channel name can be used in parameter and topic names."""
    if not isinstance(name, str) or not NAME_PATTERN.match(name):
        raise ConfigError(
            f'Channel name {name!r} is invalid: use letters, digits and underscores, '
            'starting with a letter')
    return name


def parse_channel(name: str, values: Mapping[str, object]) -> ChannelConfig:
    """
    Build a ``ChannelConfig`` from a mapping of parameter names to values.

    Keys are the per-channel parameter names without the channel prefix, for
    example ``'channel'`` or ``'twist.linear_x'``.  Missing keys and ``None``
    values fall back to the defaults in ``CHANNEL_PARAMS``.
    """
    validate_channel_name(name)

    def raw(key: str) -> object:
        value = values.get(key)
        return CHANNEL_PARAMS[key] if value is None else value

    def number(key: str, required: bool = False) -> float | None:
        value = raw(key)
        if value is None:
            if required:
                raise ConfigError(f"'{name}.{key}' is required")
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"'{name}.{key}' must be a number, got {value!r}")
        if not math.isfinite(value):
            raise ConfigError(f"'{name}.{key}' must be a finite number")
        return float(value)

    def numbers(key: str) -> list[float]:
        value = raw(key)
        if isinstance(value, (str, bytes)) and len(value) > 0:
            raise ConfigError(f"'{name}.{key}' must be a list of numbers, got {value!r}")
        try:
            items = list(value)
        except TypeError:
            raise ConfigError(f"'{name}.{key}' must be a list of numbers, got {value!r}") from None
        result = []
        for item in items:
            if isinstance(item, bool) or not isinstance(item, (int, float)) \
                    or not math.isfinite(item):
                raise ConfigError(f"'{name}.{key}' must be a list of numbers, got {value!r}")
            result.append(float(item))
        return result

    def boolean(key: str) -> bool:
        value = raw(key)
        if not isinstance(value, bool):
            raise ConfigError(f"'{name}.{key}' must be true or false, got {value!r}")
        return value

    def text(key: str) -> str:
        value = raw(key)
        if not isinstance(value, str):
            raise ConfigError(f"'{name}.{key}' must be a string, got {value!r}")
        return value

    channel = number('channel', required=True)
    if not channel.is_integer() or not 0 <= channel < NUM_CHANNELS:
        raise ConfigError(f"'{name}.channel' must be a whole number from 0 to {NUM_CHANNELS - 1}")

    kind = text('type')
    if kind not in CHANNEL_TYPES:
        raise ConfigError(
            f"'{name}.type' must be one of {', '.join(CHANNEL_TYPES)}, got '{kind}'")

    min_pulse_us = number('min_pulse_us')
    max_pulse_us = number('max_pulse_us')
    if not 0.0 < min_pulse_us < max_pulse_us:
        raise ConfigError(
            f"'{name}.min_pulse_us' ({min_pulse_us:g}) must be positive and less "
            f"than '{name}.max_pulse_us' ({max_pulse_us:g})")
    neutral_pulse_us = number('neutral_pulse_us')
    if neutral_pulse_us is None:
        neutral_pulse_us = (min_pulse_us + max_pulse_us) / 2.0
    if not min_pulse_us <= neutral_pulse_us <= max_pulse_us:
        raise ConfigError(
            f"'{name}.neutral_pulse_us' ({neutral_pulse_us:g}) must lie between "
            f'{min_pulse_us:g} and {max_pulse_us:g}')

    min_angle = number('min_angle')
    max_angle = number('max_angle')
    if not min_angle < max_angle:
        raise ConfigError(
            f"'{name}.min_angle' must be less than '{name}.max_angle' "
            "(set 'invert: true' to reverse the direction of travel)")

    if kind == SERVO:
        full_range = (min_angle, max_angle)
    elif kind == CONTINUOUS:
        full_range = (-1.0, 1.0)
    else:
        full_range = (0.0, 1.0)
    min_limit = number('min_limit')
    max_limit = number('max_limit')
    if min_limit is None:
        min_limit = full_range[0]
    if max_limit is None:
        max_limit = full_range[1]
    if not full_range[0] <= min_limit <= max_limit <= full_range[1]:
        raise ConfigError(
            f"'{name}' limits {min_limit:g}..{max_limit:g} must lie within "
            f'{full_range[0]:g}..{full_range[1]:g} {_UNITS[kind]} with min_limit <= max_limit')

    home = number('home')
    if home is None:
        home = (min_limit + max_limit) / 2.0 if kind == SERVO else 0.0
        home = min(max(home, min_limit), max_limit)
    elif not min_limit <= home <= max_limit:
        raise ConfigError(
            f"'{name}.home' ({home:g}) must lie within the limits {min_limit:g}..{max_limit:g}")

    timeout = number('timeout')
    if timeout < 0.0:
        raise ConfigError(f"'{name}.timeout' must not be negative")

    # YAML reads a bare ``off`` as the boolean false; accept it as the mode 'off'.
    on_shutdown = SHUTDOWN_OFF if raw('on_shutdown') is False else text('on_shutdown')
    if on_shutdown not in SHUTDOWN_MODES:
        raise ConfigError(
            f"'{name}.on_shutdown' must be one of {', '.join(SHUTDOWN_MODES)}, "
            f"got '{on_shutdown}'")

    joint = text('joint') or name

    twist_gains = []
    for axis in TWIST_AXES:
        gain = number(f'twist.{axis}')
        if gain != 0.0:
            twist_gains.append((axis, gain))

    sequences: dict[str, tuple[Step, ...]] = {}
    for sequence in ESC_SEQUENCES:
        throttles = numbers(f'esc.{sequence}.values')
        durations = numbers(f'esc.{sequence}.durations')
        if len(throttles) != len(durations):
            raise ConfigError(
                f"'{name}.esc.{sequence}.values' and '{name}.esc.{sequence}.durations' "
                'must have the same number of entries')
        for throttle, seconds in zip(throttles, durations):
            if not -1.0 <= throttle <= 1.0:
                raise ConfigError(
                    f"'{name}.esc.{sequence}.values' must lie between -1 and 1, got {throttle:g}")
            if seconds <= 0.0:
                raise ConfigError(
                    f"'{name}.esc.{sequence}.durations' must be positive, got {seconds:g}")
        sequences[sequence] = tuple(zip(throttles, durations))
    esc = EscConfig(
        arming=sequences['arming'],
        to_reverse=sequences['to_reverse'],
        to_forward=sequences['to_forward'],
        deadband=number('esc.deadband'),
        forward_start=number('esc.forward_start'),
        reverse_start=number('esc.reverse_start'),
    )
    if esc.configured:
        if kind != CONTINUOUS:
            raise ConfigError(
                f"'{name}.esc' settings only apply to continuous channels, not to {kind}")
        if not 0.0 <= esc.deadband < 1.0:
            raise ConfigError(f"'{name}.esc.deadband' must lie between 0 and 1")
        for key, start, extent in (('forward_start', esc.forward_start, max_limit),
                                   ('reverse_start', esc.reverse_start, -min_limit)):
            if not 0.0 <= start <= 1.0:
                raise ConfigError(f"'{name}.esc.{key}' must lie between 0 and 1")
            if extent > 0.0 and start >= extent:
                raise ConfigError(
                    f"'{name}.esc.{key}' ({start:g}) must be below the limit in that "
                    f'direction ({extent:g})')
            if extent > 0.0 and esc.deadband >= extent:
                raise ConfigError(
                    f"'{name}.esc.deadband' ({esc.deadband:g}) must be below the limit in "
                    f'that direction ({extent:g})')

    return ChannelConfig(
        name=name,
        channel=int(channel),
        kind=kind,
        min_pulse_us=min_pulse_us,
        max_pulse_us=max_pulse_us,
        neutral_pulse_us=neutral_pulse_us,
        min_angle=min_angle,
        max_angle=max_angle,
        min_limit=min_limit,
        max_limit=max_limit,
        invert=boolean('invert'),
        home=home,
        home_on_start=boolean('home_on_start'),
        timeout=timeout,
        on_shutdown=on_shutdown,
        joint=joint,
        twist_gains=tuple(twist_gains),
        esc=esc,
    )


def parse_channels(
    values_by_name: Mapping[str, Mapping[str, object]],
) -> dict[str, ChannelConfig]:
    """
    Parse every channel and check that they do not conflict with each other.

    ``values_by_name`` maps each channel name to its parameter values.
    """
    if not values_by_name:
        raise ConfigError("No channels configured: list at least one name in 'channels'")
    configs: dict[str, ChannelConfig] = {}
    outputs: dict[int, str] = {}
    joints: dict[str, str] = {}
    for name, values in values_by_name.items():
        config = parse_channel(name, values)
        if config.channel in outputs:
            raise ConfigError(
                f"'{name}' and '{outputs[config.channel]}' both use output {config.channel}")
        if config.joint in joints:
            raise ConfigError(
                f"'{name}' and '{joints[config.joint]}' both use joint name '{config.joint}'")
        outputs[config.channel] = name
        joints[config.joint] = name
        configs[name] = config
    return configs
