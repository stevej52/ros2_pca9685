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
Minimal driver for the NXP PCA9685 16-channel PWM controller.

The chip is driven through the Linux I2C device interface (``/dev/i2c-N``)
using only the Python standard library, so no extra Python packages are
needed on a Raspberry Pi, Jetson or any other Linux single-board computer.
A fake bus is included so the ROS node can run without hardware and so the
driver can be unit tested.

This module has no ROS dependencies.
"""

from __future__ import annotations

import errno
import fcntl
import math
import os
import time

# Register map (PCA9685 datasheet, section 7.3).
MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
ALL_LED_ON_L = 0xFA
PRESCALE = 0xFE

# MODE1 bits.
MODE1_RESTART = 0x80
MODE1_EXTCLK = 0x40
MODE1_AI = 0x20
MODE1_SLEEP = 0x10
MODE1_ALLCALL = 0x01

# Bit 4 of LEDn_ON_H / LEDn_OFF_H forces the output permanently on / off.
FULL_BIT = 0x10

NUM_CHANNELS = 16
RESOLUTION = 4096  # 12-bit counter per PWM period
DEFAULT_OSCILLATOR_HZ = 25_000_000.0
MIN_PRESCALE = 3
MAX_PRESCALE = 255

I2C_SLAVE = 0x0703  # ioctl request from <linux/i2c-dev.h>


class I2CError(OSError):
    """An I2C bus or device problem, with a hint about how to fix it."""


class LinuxI2CBus:
    """Access to one I2C device through ``/dev/i2c-<bus>``."""

    def __init__(self, bus: int, address: int) -> None:
        self.bus = int(bus)
        self.address = int(address)
        self.path = f'/dev/i2c-{self.bus}'
        self._fd: int | None = None
        try:
            self._fd = os.open(self.path, os.O_RDWR)
        except FileNotFoundError as exc:
            raise I2CError(
                f'{self.path} does not exist. Enable the I2C interface (raspi-config on a '
                f'Raspberry Pi) or set "i2c_bus" to one of: {self._available_buses()}'
            ) from exc
        except PermissionError as exc:
            raise I2CError(
                f'Permission denied opening {self.path}. Add your user to the i2c group '
                '("sudo usermod -aG i2c $USER", then log out and back in).'
            ) from exc
        try:
            fcntl.ioctl(self._fd, I2C_SLAVE, self.address)
        except OSError as exc:
            os.close(self._fd)
            self._fd = None
            if exc.errno == errno.EBUSY:
                raise I2CError(
                    f'Address 0x{self.address:02x} on {self.path} is already claimed by a kernel '
                    'driver (a device tree overlay for the pca9685-pwm driver, perhaps).'
                ) from exc
            raise I2CError(
                f'Cannot select address 0x{self.address:02x} on {self.path}: {exc}') from exc

    @staticmethod
    def _available_buses() -> str:
        buses = sorted(name for name in os.listdir('/dev') if name.startswith('i2c-'))
        if not buses:
            return '(no /dev/i2c-* devices found)'
        return ', '.join(name[len('i2c-'):] for name in buses)

    def write(self, register: int, data: bytes) -> None:
        """Write ``data`` to consecutive registers starting at ``register``."""
        buffer = bytes([register]) + bytes(data)
        written = os.write(self._fd, buffer)
        if written != len(buffer):
            raise I2CError(f'Short write to {self.path}: {written} of {len(buffer)} bytes')

    def read(self, register: int, length: int = 1) -> bytes:
        """Read ``length`` bytes starting at ``register``."""
        os.write(self._fd, bytes([register]))
        return os.read(self._fd, length)

    def close(self) -> None:
        """Close the device file."""
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


class FakeI2CBus:
    """In-memory stand-in for an I2C bus with a simulated PCA9685 attached."""

    def __init__(self) -> None:
        self.registers = bytearray(256)
        # Power-on defaults from the datasheet.
        self.registers[MODE1] = MODE1_SLEEP | MODE1_ALLCALL
        self.registers[MODE2] = 0x04
        self.registers[PRESCALE] = 0x1E
        self.writes: list[tuple[int, bytes]] = []
        self.closed = False

    def write(self, register: int, data: bytes) -> None:
        """Store ``data`` in the simulated registers (auto-increment is assumed)."""
        data = bytes(data)
        for offset, value in enumerate(data):
            target = (register + offset) & 0xFF
            self.registers[target] = value
            if ALL_LED_ON_L <= target < ALL_LED_ON_L + 4:
                # Writing the ALL_LED registers loads every LEDn register.
                for channel in range(NUM_CHANNELS):
                    self.registers[LED0_ON_L + 4 * channel + target - ALL_LED_ON_L] = value
        self.writes.append((register, data))

    def read(self, register: int, length: int = 1) -> bytes:
        """Return ``length`` bytes from the simulated registers."""
        return bytes(self.registers[register:register + length])

    def close(self) -> None:
        """Mark the bus as closed."""
        self.closed = True

    def channel_registers(self, channel: int) -> tuple[int, int]:
        """
        Return the raw (ON, OFF) 13-bit values of a channel.

        Bit 12 of either value is the "full on" / "full off" flag.
        """
        base = LED0_ON_L + 4 * channel
        on = self.registers[base] | (self.registers[base + 1] << 8)
        off = self.registers[base + 2] | (self.registers[base + 3] << 8)
        return on & 0x1FFF, off & 0x1FFF

    def channel_state(self, channel: int) -> str | int:
        """Return ``'off'``, ``'on'`` or the number of high ticks per period."""
        on, off = self.channel_registers(channel)
        if off & (FULL_BIT << 8):
            return 'off'
        if on & (FULL_BIT << 8):
            return 'on'
        return (off - on) % RESOLUTION


class Pca9685:
    """Driver for one PCA9685 chip on an I2C bus."""

    def __init__(self, bus, oscillator_hz: float = DEFAULT_OSCILLATOR_HZ) -> None:
        self._bus = bus
        self._oscillator_hz = float(oscillator_hz)
        self._prescale: int | None = None

    @staticmethod
    def prescale_for(frequency_hz: float, oscillator_hz: float = DEFAULT_OSCILLATOR_HZ) -> int:
        """Return the PRE_SCALE register value for a PWM frequency (datasheet equation 1)."""
        if frequency_hz <= 0.0 or oscillator_hz <= 0.0:
            raise ValueError('PWM and oscillator frequencies must be positive')
        prescale = math.floor(oscillator_hz / (RESOLUTION * frequency_hz) + 0.5) - 1
        if not MIN_PRESCALE <= prescale <= MAX_PRESCALE:
            lowest = oscillator_hz / (RESOLUTION * (MAX_PRESCALE + 1))
            highest = oscillator_hz / (RESOLUTION * (MIN_PRESCALE + 1))
            raise ValueError(
                f'PWM frequency {frequency_hz:g} Hz is outside the supported range '
                f'{lowest:.1f}-{highest:.1f} Hz')
        return prescale

    @staticmethod
    def frequency_for(prescale: int, oscillator_hz: float = DEFAULT_OSCILLATOR_HZ) -> float:
        """Return the PWM frequency produced by a PRE_SCALE register value."""
        return oscillator_hz / (RESOLUTION * (prescale + 1))

    @property
    def oscillator_hz(self) -> float:
        """Return the assumed oscillator frequency."""
        return self._oscillator_hz

    @oscillator_hz.setter
    def oscillator_hz(self, value: float) -> None:
        """Change the assumed oscillator frequency; call ``set_frequency`` afterwards."""
        if value <= 0.0:
            raise ValueError('The oscillator frequency must be positive')
        self._oscillator_hz = float(value)

    @property
    def frequency(self) -> float:
        """Return the actual PWM frequency in Hz (after prescaler rounding)."""
        if self._prescale is None:
            raise RuntimeError('PCA9685 has not been initialized')
        return self.frequency_for(self._prescale, self._oscillator_hz)

    @property
    def period_us(self) -> float:
        """Return the PWM period in microseconds."""
        return 1e6 / self.frequency

    def initialize(self, frequency_hz: float) -> float:
        """
        Reset the chip, program the PWM frequency and switch every output off.

        Returns the actual PWM frequency.
        """
        prescale = self.prescale_for(frequency_hz, self._oscillator_hz)
        try:
            # PRE_SCALE can only be written while the oscillator sleeps.  Auto
            # increment lets us write the four registers of a channel at once.
            self._bus.write(MODE1, bytes([MODE1_AI | MODE1_SLEEP]))
            self._bus.write(ALL_LED_ON_L, bytes([0x00, 0x00, 0x00, FULL_BIT]))
            self._bus.write(PRESCALE, bytes([prescale]))
            # Wake up.  ALLCALL is left off so the chip only answers on its own address.
            self._bus.write(MODE1, bytes([MODE1_AI]))
        except I2CError:
            raise
        except OSError as exc:
            raise I2CError(self._describe_no_device(exc)) from exc
        time.sleep(0.001)  # the oscillator needs up to 500 us to stabilise
        self._prescale = prescale
        return self.frequency

    def _describe_no_device(self, exc: OSError) -> str:
        where = ''
        if isinstance(self._bus, LinuxI2CBus):
            where = (f' at address 0x{self._bus.address:02x} on {self._bus.path}'
                     f' ("i2cdetect -y {self._bus.bus}" should list {self._bus.address:02x})')
        return (f'No response from the PCA9685{where}: {exc}. Check the wiring (SDA, SCL, GND '
                'and VCC), the address jumpers and that the board is powered.')

    def set_frequency(self, frequency_hz: float) -> float:
        """
        Change the PWM frequency.

        Every output is switched off in the process and has to be written again.
        """
        return self.initialize(frequency_hz)

    @staticmethod
    def _check_channel(channel: int) -> None:
        if not 0 <= channel < NUM_CHANNELS:
            raise ValueError(f'Channel {channel} is not in 0..{NUM_CHANNELS - 1}')

    def set_pwm(self, channel: int, on_tick: int, off_tick: int) -> None:
        """Set the raw counter values at which the output goes high and low."""
        self._check_channel(channel)
        if not (0 <= on_tick < RESOLUTION and 0 <= off_tick < RESOLUTION):
            raise ValueError('Tick values must be in 0..4095')
        self._bus.write(LED0_ON_L + 4 * channel, bytes([
            on_tick & 0xFF, on_tick >> 8, off_tick & 0xFF, off_tick >> 8]))

    def set_off(self, channel: int) -> None:
        """Hold the output low (no pulses at all)."""
        self._check_channel(channel)
        self._bus.write(LED0_ON_L + 4 * channel, bytes([0x00, 0x00, 0x00, FULL_BIT]))

    def set_full_on(self, channel: int) -> None:
        """Hold the output high."""
        self._check_channel(channel)
        self._bus.write(LED0_ON_L + 4 * channel, bytes([0x00, FULL_BIT, 0x00, 0x00]))

    def set_ticks(self, channel: int, ticks: int) -> int:
        """
        Make the output high for ``ticks`` of the 4096 counts in each period.

        Values at or below zero switch the output off, values at or above 4096
        switch it fully on.  Returns the clamped tick count.
        """
        ticks = int(round(ticks))
        if ticks <= 0:
            self.set_off(channel)
            return 0
        if ticks >= RESOLUTION:
            self.set_full_on(channel)
            return RESOLUTION
        self.set_pwm(channel, 0, ticks)
        return ticks

    def set_duty_cycle(self, channel: int, duty_cycle: float) -> int:
        """Set the fraction of each period (0.0 to 1.0) the output is high."""
        return self.set_ticks(channel, round(duty_cycle * RESOLUTION))

    def set_pulse_width_us(self, channel: int, pulse_us: float) -> int:
        """Output a pulse of ``pulse_us`` microseconds every period."""
        return self.set_ticks(channel, round(pulse_us / self.period_us * RESOLUTION))

    def all_off(self) -> None:
        """Switch every output off."""
        self._bus.write(ALL_LED_ON_L, bytes([0x00, 0x00, 0x00, FULL_BIT]))

    def is_configured(self) -> bool:
        """Return whether the chip still holds the configuration written by ``initialize``."""
        if self._prescale is None:
            return False
        mode1 = self._bus.read(MODE1, 1)[0]
        prescale = self._bus.read(PRESCALE, 1)[0]
        return (mode1 & (MODE1_SLEEP | MODE1_AI)) == MODE1_AI and prescale == self._prescale

    def close(self) -> None:
        """Release the bus."""
        self._bus.close()
