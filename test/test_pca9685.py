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

"""Tests for the PCA9685 driver, run against the fake bus."""

import pytest
from ros2_pca9685.pca9685 import ALL_LED_ON_L, FakeI2CBus, MODE1, MODE1_AI, MODE1_ALLCALL, \
    MODE1_SLEEP, Pca9685, PRESCALE


def make_chip(frequency=50.0):
    bus = FakeI2CBus()
    chip = Pca9685(bus)
    actual = chip.initialize(frequency)
    return bus, chip, actual


@pytest.mark.parametrize('frequency, prescale', [
    (50.0, 121),  # 25 MHz / (4096 * 50) = 122.07 -> 122 - 1
    (100.0, 60),
    (200.0, 30),  # 30.5 rounds up
    (24.0, 253),
    (1526.0, 3),
])
def test_prescale_matches_datasheet(frequency, prescale):
    assert Pca9685.prescale_for(frequency) == prescale
    assert Pca9685.frequency_for(prescale) == pytest.approx(25e6 / (4096 * (prescale + 1)))


@pytest.mark.parametrize('frequency', [0.0, -50.0, 10.0, 2000.0])
def test_unsupported_frequencies_are_rejected(frequency):
    with pytest.raises(ValueError):
        Pca9685.prescale_for(frequency)


def test_initialize_sleeps_programs_prescale_and_wakes():
    bus, chip, actual = make_chip(50.0)
    assert bus.writes == [
        (MODE1, bytes([MODE1_AI | MODE1_SLEEP])),
        (ALL_LED_ON_L, bytes([0x00, 0x00, 0x00, 0x10])),
        (PRESCALE, bytes([121])),
        (MODE1, bytes([MODE1_AI])),
    ]
    assert actual == pytest.approx(50.03, abs=0.01)  # 25 MHz / (4096 * 122)
    assert chip.frequency == actual
    assert chip.period_us == pytest.approx(1e6 / actual)
    assert all(bus.channel_state(channel) == 'off' for channel in range(16))
    assert bus.registers[MODE1] & MODE1_ALLCALL == 0


def test_custom_oscillator_frequency():
    chip = Pca9685(FakeI2CBus(), oscillator_hz=26_000_000.0)
    assert chip.initialize(50.0) == pytest.approx(26e6 / (4096 * 127))


def test_pulse_width_becomes_ticks():
    bus, chip, actual = make_chip(50.0)
    ticks = chip.set_pulse_width_us(3, 1500.0)
    expected = round(1500.0 / (1e6 / actual) * 4096)
    assert ticks == expected == 307
    assert bus.channel_registers(3) == (0, 307)
    assert bus.channel_state(3) == 307
    assert bus.channel_state(2) == 'off'


def test_duty_cycle_extremes():
    bus, chip, _ = make_chip()
    chip.set_duty_cycle(0, 0.0)
    chip.set_duty_cycle(1, 1.0)
    chip.set_duty_cycle(2, 0.5)
    assert bus.channel_state(0) == 'off'
    assert bus.channel_state(1) == 'on'
    assert bus.channel_state(2) == 2048


def test_set_ticks_clamps():
    bus, chip, _ = make_chip()
    assert chip.set_ticks(4, -5) == 0
    assert bus.channel_state(4) == 'off'
    assert chip.set_ticks(4, 5000) == 4096
    assert bus.channel_state(4) == 'on'
    assert chip.set_ticks(4, 4095) == 4095
    assert bus.channel_state(4) == 4095


def test_full_on_then_pwm_clears_the_full_bits():
    bus, chip, _ = make_chip()
    chip.set_full_on(7)
    chip.set_pwm(7, 100, 200)
    assert bus.channel_registers(7) == (100, 200)
    assert bus.channel_state(7) == 100


@pytest.mark.parametrize('channel', [-1, 16, 100])
def test_channel_numbers_are_checked(channel):
    _, chip, _ = make_chip()
    with pytest.raises(ValueError):
        chip.set_off(channel)
    with pytest.raises(ValueError):
        chip.set_pwm(channel, 0, 1)


def test_tick_values_are_checked():
    _, chip, _ = make_chip()
    with pytest.raises(ValueError):
        chip.set_pwm(0, 0, 4096)


def test_frequency_requires_initialize():
    chip = Pca9685(FakeI2CBus())
    with pytest.raises(RuntimeError):
        chip.frequency
    assert chip.is_configured() is False


def test_is_configured_detects_a_reset():
    bus, chip, _ = make_chip()
    assert chip.is_configured() is True
    bus.registers[MODE1] = MODE1_SLEEP | MODE1_ALLCALL  # power-on state after a brown-out
    assert chip.is_configured() is False
    chip.initialize(50.0)
    assert chip.is_configured() is True
    bus.registers[PRESCALE] = 0x1E
    assert chip.is_configured() is False


def test_set_frequency_switches_outputs_off():
    bus, chip, _ = make_chip(50.0)
    chip.set_pulse_width_us(5, 1500.0)
    actual = chip.set_frequency(100.0)
    assert actual == pytest.approx(100.06, abs=0.01)
    assert bus.channel_state(5) == 'off'
    assert chip.set_pulse_width_us(5, 1500.0) == 615


def test_all_off_and_close():
    bus, chip, _ = make_chip()
    chip.set_duty_cycle(9, 0.3)
    chip.all_off()
    assert bus.writes[-1] == (ALL_LED_ON_L, bytes([0x00, 0x00, 0x00, 0x10]))
    chip.close()
    assert bus.closed
