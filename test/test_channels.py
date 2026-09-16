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

"""Tests for channel configuration parsing and the command maths."""

import pytest
from ros2_pca9685.channels import CHANNEL_PARAMS, ConfigError, CONTINUOUS, parse_channel, \
    parse_channels, PWM, SERVO, TWIST_AXES
from ros2_pca9685.esc import EscConfig


def test_defaults():
    config = parse_channel('pan', {'channel': 0})
    assert config.channel == 0
    assert config.kind == SERVO
    assert (config.min_pulse_us, config.max_pulse_us) == (750.0, 2250.0)
    assert config.neutral_pulse_us == 1500.0
    assert (config.min_angle, config.max_angle) == (0.0, 180.0)
    assert (config.min_limit, config.max_limit) == (0.0, 180.0)
    assert config.home == 90.0
    assert config.invert is False
    assert config.home_on_start is False
    assert config.timeout == 0.0
    assert config.on_shutdown == 'off'
    assert config.joint == 'pan'
    assert config.twist_gains == ()
    assert config.twist_driven is False
    assert config.units == 'deg'
    assert config.command_topic == 'angle'


def test_integers_and_none_are_accepted():
    config = parse_channel('pan', {'channel': 3, 'home': 45, 'min_limit': None, 'timeout': 1})
    assert config.channel == 3
    assert config.home == 45.0
    assert config.min_limit == 0.0
    assert config.timeout == 1.0


def test_yaml_style_bare_off_means_shutdown_off():
    # YAML 1.1 turns an unquoted "off" into the boolean false.
    assert parse_channel('pan', {'channel': 3, 'on_shutdown': False}).on_shutdown == 'off'
    with pytest.raises(ConfigError, match='on_shutdown'):
        parse_channel('pan', {'channel': 3, 'on_shutdown': True})


def test_servo_pulse_widths():
    config = parse_channel('s', {'channel': 0})
    assert config.pulse_width_us(0.0) == 750.0
    assert config.pulse_width_us(90.0) == 1500.0
    assert config.pulse_width_us(180.0) == 2250.0
    assert config.pulse_width_us(45.0) == pytest.approx(1125.0)


def test_servo_calibration_range_and_invert():
    values = {'channel': 0, 'min_pulse_us': 500.0, 'max_pulse_us': 2500.0,
              'min_angle': -90.0, 'max_angle': 90.0}
    config = parse_channel('s', values)
    assert config.pulse_width_us(-90.0) == 500.0
    assert config.pulse_width_us(0.0) == 1500.0
    assert config.pulse_width_us(90.0) == 2500.0
    inverted = parse_channel('s', {**values, 'invert': True})
    assert inverted.pulse_width_us(-90.0) == 2500.0
    assert inverted.pulse_width_us(90.0) == 500.0
    assert inverted.pulse_width_us(0.0) == 1500.0


def test_limits_clamp_commands():
    config = parse_channel('steering', {'channel': 1, 'min_limit': 30.0, 'max_limit': 135.0})
    assert config.clamp(200.0) == 135.0
    assert config.clamp(-5.0) == 30.0
    assert config.pulse_width_us(200.0) == config.pulse_width_us(135.0)
    assert config.home == pytest.approx(82.5)


def test_continuous_pulse_widths():
    values = {'channel': 0, 'type': CONTINUOUS, 'min_pulse_us': 1000.0,
              'max_pulse_us': 2000.0}
    config = parse_channel('esc', values)
    assert config.units == 'throttle'
    assert config.command_topic == 'throttle'
    assert (config.min_limit, config.max_limit) == (-1.0, 1.0)
    assert config.home == 0.0
    assert config.pulse_width_us(0.0) == 1500.0
    assert config.pulse_width_us(1.0) == 2000.0
    assert config.pulse_width_us(-1.0) == 1000.0
    assert config.pulse_width_us(0.5) == 1750.0
    assert config.pulse_width_us(-0.5) == 1250.0
    assert config.pulse_width_us(3.0) == 2000.0
    asymmetric = parse_channel('esc', {**values, 'neutral_pulse_us': 1400.0})
    assert asymmetric.pulse_width_us(0.5) == 1700.0
    assert asymmetric.pulse_width_us(-0.5) == 1200.0
    inverted = parse_channel('esc', {**values, 'invert': True})
    assert inverted.pulse_width_us(1.0) == 1000.0
    assert inverted.pulse_width_us(-0.25) == 1625.0


def test_continuous_limits_and_home():
    config = parse_channel(
        'esc', {'channel': 0, 'type': CONTINUOUS, 'min_limit': 0.2, 'max_limit': 0.8})
    assert config.home == 0.2
    assert config.clamp(-1.0) == 0.2


def test_pwm_duty_cycle():
    config = parse_channel('led', {'channel': 15, 'type': PWM})
    assert config.units == 'duty'
    assert config.command_topic == 'duty_cycle'
    assert (config.min_limit, config.max_limit) == (0.0, 1.0)
    assert config.home == 0.0
    assert config.duty_cycle(0.25) == 0.25
    assert config.duty_cycle(4.0) == 1.0
    inverted = parse_channel('led', {'channel': 15, 'type': PWM, 'invert': True})
    assert inverted.duty_cycle(0.25) == 0.75
    with pytest.raises(ValueError):
        config.pulse_width_us(0.5)
    servo = parse_channel('s', {'channel': 0})
    with pytest.raises(ValueError):
        servo.duty_cycle(0.5)


def test_twist_value():
    config = parse_channel('steering', {
        'channel': 1, 'home': 85.0, 'min_limit': 30.0, 'max_limit': 135.0,
        'twist.angular_z': -18.33})
    assert config.twist_driven is True
    assert config.twist_gains == (('angular_z', -18.33),)
    assert config.twist_value({'angular_z': 1.0}) == pytest.approx(85.0 - 18.33)
    assert config.twist_value({'angular_z': -1.0}) == pytest.approx(85.0 + 18.33)
    assert config.twist_value({'angular_z': 10.0}) == 30.0
    assert config.twist_value({}) == 85.0
    mixed = parse_channel('left', {
        'channel': 2, 'type': CONTINUOUS, 'twist.linear_x': 1.0, 'twist.angular_z': -0.25})
    assert mixed.twist_value({'linear_x': 0.5, 'angular_z': 1.0}) == pytest.approx(0.25)


def test_round_trip_through_parameters():
    values = {
        'channel': 5, 'type': CONTINUOUS, 'min_pulse_us': 1100.0, 'max_pulse_us': 1900.0,
        'neutral_pulse_us': 1480.0, 'min_limit': -0.7, 'max_limit': 0.9, 'invert': True,
        'home': 0.1, 'home_on_start': True, 'timeout': 0.25, 'on_shutdown': 'home',
        'joint': 'wheel', 'twist.linear_x': 2.0, 'twist.angular_y': -1.0}
    config = parse_channel('drive', values)
    params = config.as_params()
    assert set(params) == set(CHANNEL_PARAMS)
    assert parse_channel('drive', params) == config
    for axis in TWIST_AXES:
        assert f'twist.{axis}' in params


def test_describe_mentions_the_essentials():
    config = parse_channel('steering', {
        'channel': 1, 'home': 85.0, 'home_on_start': True, 'timeout': 0.5, 'invert': True,
        'twist.angular_z': -18.0, 'joint': 'front_steer'})
    text = config.describe()
    for expected in ('output 1', 'servo', '750-2250 us', 'limits 0..180 deg', 'inverted',
                     'home 85 (on start)', 'timeout 0.5 s', 'angular_z*-18', 'joint front_steer'):
        assert expected in text


@pytest.mark.parametrize('values, message', [
    ({}, 'is required'),
    ({'channel': 16}, 'from 0 to 15'),
    ({'channel': -1}, 'from 0 to 15'),
    ({'channel': 1.5}, 'from 0 to 15'),
    ({'channel': '3'}, 'must be a number'),
    ({'channel': True}, 'must be a number'),
    ({'channel': 0, 'home': float('nan')}, 'finite'),
    ({'channel': 0, 'type': 'motor'}, "'s.type' must be one of"),
    ({'channel': 0, 'min_pulse_us': 2000.0, 'max_pulse_us': 1000.0}, 'min_pulse_us'),
    ({'channel': 0, 'min_pulse_us': -1.0}, 'min_pulse_us'),
    ({'channel': 0, 'neutral_pulse_us': 100.0}, 'neutral_pulse_us'),
    ({'channel': 0, 'min_angle': 90.0, 'max_angle': 0.0}, 'invert'),
    ({'channel': 0, 'min_limit': -10.0}, 'limits'),
    ({'channel': 0, 'max_limit': 200.0}, 'limits'),
    ({'channel': 0, 'min_limit': 100.0, 'max_limit': 50.0}, 'limits'),
    ({'channel': 0, 'type': CONTINUOUS, 'max_limit': 2.0}, 'limits'),
    ({'channel': 0, 'type': PWM, 'min_limit': -0.5}, 'limits'),
    ({'channel': 0, 'home': 181.0}, 'home'),
    ({'channel': 0, 'timeout': -1.0}, 'timeout'),
    ({'channel': 0, 'on_shutdown': 'explode'}, 'on_shutdown'),
    ({'channel': 0, 'invert': 'yes'}, 'true or false'),
    ({'channel': 0, 'home_on_start': 1}, 'true or false'),
    ({'channel': 0, 'joint': 5}, 'must be a string'),
    ({'channel': 0, 'twist.linear_x': 'fast'}, 'must be a number'),
])
def test_invalid_values_are_reported(values, message):
    with pytest.raises(ConfigError, match=message):
        parse_channel('s', values)


@pytest.mark.parametrize('name', ['9lives', 'a-b', 'with space', '', 'dotted.name', 42])
def test_invalid_names_are_rejected(name):
    with pytest.raises(ConfigError, match='invalid'):
        parse_channel(name, {'channel': 0})


def test_parse_channels_detects_conflicts():
    with pytest.raises(ConfigError, match='No channels'):
        parse_channels({})
    with pytest.raises(ConfigError, match='both use output 0'):
        parse_channels({'a': {'channel': 0}, 'b': {'channel': 0}})
    with pytest.raises(ConfigError, match="both use joint name 'j'"):
        parse_channels({'a': {'channel': 0, 'joint': 'j'}, 'b': {'channel': 1, 'joint': 'j'}})
    configs = parse_channels({'a': {'channel': 0}, 'b': {'channel': 1}})
    assert list(configs) == ['a', 'b']
    assert configs['b'].channel == 1


def test_esc_settings_are_parsed_for_continuous_channels():
    values = {
        'channel': 0, 'type': CONTINUOUS, 'max_limit': 0.6,
        'esc.arming.values': [0.0, 0.05, 0.0], 'esc.arming.durations': [2.5, 0.5, 0.5],
        'esc.to_reverse.values': [-0.3, 0.0], 'esc.to_reverse.durations': [0.2, 0.2],
        'esc.deadband': 0.03, 'esc.forward_start': 0.27, 'esc.reverse_start': 0.05}
    config = parse_channel('drive', values)
    assert config.is_esc is True
    assert config.esc == EscConfig(
        arming=((0.0, 2.5), (0.05, 0.5), (0.0, 0.5)), to_reverse=((-0.3, 0.2), (0.0, 0.2)),
        deadband=0.03, forward_start=0.27, reverse_start=0.05)
    assert 'ESC arming 3 step(s) 3.5 s to_reverse 2 step(s) 0.4 s deadband 0.03 starts 0.27/0.05' \
        in config.describe()
    params = config.as_params()
    assert params['esc.arming.values'] == [0.0, 0.05, 0.0]
    assert params['esc.to_forward.durations'] == []
    assert parse_channel('drive', params) == config
    plain = parse_channel('drive', {'channel': 0, 'type': CONTINUOUS})
    assert plain.is_esc is False
    assert plain.esc == EscConfig()
    assert 'ESC' not in plain.describe()


def test_esc_arrays_accept_integers_and_array_types():
    import array
    values = {'channel': 0, 'type': CONTINUOUS,
              'esc.arming.values': array.array('d', [0.0]), 'esc.arming.durations': [2]}
    assert parse_channel('drive', values).esc.arming == ((0.0, 2.0),)


@pytest.mark.parametrize('values, message', [
    ({'channel': 0, 'esc.deadband': 0.1}, 'only apply to continuous'),
    ({'channel': 0, 'type': PWM, 'esc.arming.values': [0.0], 'esc.arming.durations': [1.0]},
     'only apply to continuous'),
    ({'channel': 0, 'type': CONTINUOUS, 'esc.arming.values': [0.0, 0.1],
      'esc.arming.durations': [1.0]}, 'same number of entries'),
    ({'channel': 0, 'type': CONTINUOUS, 'esc.arming.values': [1.5],
      'esc.arming.durations': [1.0]}, 'between -1 and 1'),
    ({'channel': 0, 'type': CONTINUOUS, 'esc.to_forward.values': [0.0],
      'esc.to_forward.durations': [0.0]}, 'must be positive'),
    ({'channel': 0, 'type': CONTINUOUS, 'esc.arming.values': 'neutral',
      'esc.arming.durations': [1.0]}, 'list of numbers'),
    ({'channel': 0, 'type': CONTINUOUS, 'esc.arming.values': [True],
      'esc.arming.durations': [1.0]}, 'list of numbers'),
    ({'channel': 0, 'type': CONTINUOUS, 'esc.deadband': 1.0}, 'deadband'),
    ({'channel': 0, 'type': CONTINUOUS, 'esc.forward_start': 1.5}, 'forward_start'),
    ({'channel': 0, 'type': CONTINUOUS, 'max_limit': 0.3, 'esc.forward_start': 0.3},
     'below the limit'),
    ({'channel': 0, 'type': CONTINUOUS, 'min_limit': -0.2, 'esc.reverse_start': 0.25},
     'below the limit'),
    ({'channel': 0, 'type': CONTINUOUS, 'max_limit': 0.1, 'esc.deadband': 0.1},
     'deadband'),
])
def test_invalid_esc_settings_are_reported(values, message):
    with pytest.raises(ConfigError, match=message):
        parse_channel('drive', values)
