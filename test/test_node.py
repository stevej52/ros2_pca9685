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

"""Tests for the ROS node, run in simulation mode without hardware."""

import itertools
import math
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros2_pca9685.channels import ConfigError
from ros2_pca9685.pca9685_node import Pca9685Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

BASE_PARAMS = {
    'simulate': True,
    'channels': ['steering', 'esc', 'led'],
    'steering.channel': 1,
    'steering.home': 85.0,
    'steering.min_limit': 30.0,
    'steering.max_limit': 135.0,
    'steering.home_on_start': True,
    'steering.timeout': 0.3,
    'steering.twist.angular_z': -18.33,
    'esc.channel': 0,
    'esc.type': 'continuous',
    'esc.min_pulse_us': 1000.0,
    'esc.max_pulse_us': 2000.0,
    'esc.twist.linear_x': 0.5,
    'led.channel': 15,
    'led.type': 'pwm',
}

_counter = itertools.count()


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    """A node under test plus a helper node to talk to it."""

    def __init__(self, params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        self.name = f'pca9685_test_{next(_counter)}'
        self.node = Pca9685Node(self.name, parameter_overrides=overrides)
        self.helper = Node(f'{self.name}_helper')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.bus = self.node.bus

    def close(self):
        self.node.shutdown_outputs()
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.helper)
        self.node.destroy_node()
        self.helper.destroy_node()

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def wait_for(self, condition, publish=None, timeout=5.0):
        """Spin, optionally re-publishing a message, until ``condition`` holds."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish is not None:
                publish()
            self.executor.spin_once(timeout_sec=0.02)
            if condition():
                return True
        return False

    def publish(self, topic, msg_type, msg, condition):
        publisher = self.helper.create_publisher(msg_type, topic, 10)
        try:
            assert self.wait_for(condition, publish=lambda: publisher.publish(msg)), \
                f'{topic} did not have the expected effect'
        finally:
            self.helper.destroy_publisher(publisher)

    def topic(self, channel, suffix):
        return f'/{self.name}/{channel}/{suffix}'

    def ticks_for_pulse(self, pulse_us):
        return round(pulse_us / self.node.pca.period_us * 4096)

    def ticks_for_angle(self, angle, channel='steering'):
        return self.ticks_for_pulse(self.node._configs[channel].pulse_width_us(angle))

    def state(self, output):
        return self.bus.channel_state(output)


@pytest.fixture
def harness(ros_context):
    harness = Harness(BASE_PARAMS)
    yield harness
    harness.close()


def test_startup_state_and_topics(harness):
    assert harness.node.pca.frequency == pytest.approx(50.03, abs=0.01)
    assert harness.state(1) == harness.ticks_for_angle(85.0)  # home_on_start
    assert harness.state(0) == 'off'
    assert harness.state(15) == 'off'
    topics = dict(harness.node.get_topic_names_and_types())
    for expected in ('steering/angle', 'steering/pulse_width', 'esc/throttle',
                     'esc/pulse_width', 'led/duty_cycle', 'led/pulse_width'):
        assert topics[f'/{harness.name}/{expected}'] == ['std_msgs/msg/Float64']
    assert topics['/cmd_vel'] == ['geometry_msgs/msg/Twist']
    assert '/joint_states' not in topics


def test_resolved_defaults_are_visible_as_parameters(harness):
    node = harness.node
    assert node.get_parameter('steering.min_pulse_us').value == 750.0
    assert node.get_parameter('steering.joint').value == 'steering'
    assert node.get_parameter('esc.neutral_pulse_us').value == 1500.0
    assert node.get_parameter('esc.min_limit').value == -1.0
    assert node.get_parameter('led.home').value == 0.0
    assert node.get_parameter('steering.twist.linear_x').value == 0.0


def test_angle_topic_moves_the_servo_and_clamps(harness):
    harness.publish(harness.topic('steering', 'angle'), Float64, Float64(data=120.0),
                    lambda: harness.state(1) == harness.ticks_for_angle(120.0))
    harness.publish(harness.topic('steering', 'angle'), Float64, Float64(data=500.0),
                    lambda: harness.state(1) == harness.ticks_for_angle(135.0))
    harness.publish(harness.topic('steering', 'angle'), Float64, Float64(data=-500.0),
                    lambda: harness.state(1) == harness.ticks_for_angle(30.0))


def test_throttle_and_duty_cycle_topics(harness):
    harness.publish(harness.topic('esc', 'throttle'), Float64, Float64(data=0.5),
                    lambda: harness.state(0) == harness.ticks_for_pulse(1750.0))
    harness.publish(harness.topic('esc', 'throttle'), Float64, Float64(data=-2.0),
                    lambda: harness.state(0) == harness.ticks_for_pulse(1000.0))
    harness.publish(harness.topic('led', 'duty_cycle'), Float64, Float64(data=0.5),
                    lambda: harness.state(15) == 2048)
    harness.publish(harness.topic('led', 'duty_cycle'), Float64, Float64(data=1.0),
                    lambda: harness.state(15) == 'on')


def test_pulse_width_topic_bypasses_calibration(harness):
    harness.publish(harness.topic('steering', 'pulse_width'), Float64, Float64(data=1500.0),
                    lambda: harness.state(1) == harness.ticks_for_pulse(1500.0))
    harness.publish(harness.topic('steering', 'pulse_width'), Float64, Float64(data=0.0),
                    lambda: harness.state(1) == 'off')


def test_non_finite_values_are_ignored(harness):
    before = harness.state(1)
    harness.publish(harness.topic('steering', 'angle'), Float64, Float64(data=math.nan),
                    lambda: True)
    harness.spin(0.2)
    assert harness.state(1) == before


def test_twist_drives_channels_and_timeout_returns_home(harness):
    twist = Twist()
    twist.linear.x = 1.0
    twist.angular.z = 1.0
    harness.publish('/cmd_vel', Twist, twist,
                    lambda: harness.state(0) == harness.ticks_for_pulse(1750.0)
                    and harness.state(1) == harness.ticks_for_angle(85.0 - 18.33))
    # Only the steering channel has a timeout; the ESC keeps its last command.
    assert harness.wait_for(lambda: harness.state(1) == harness.ticks_for_angle(85.0), timeout=2.0)
    assert harness.state(0) == harness.ticks_for_pulse(1750.0)


def test_parameter_updates_are_validated_and_applied(harness):
    node = harness.node
    harness.publish(harness.topic('steering', 'angle'), Float64, Float64(data=120.0),
                    lambda: harness.state(1) == harness.ticks_for_angle(120.0))

    result = node.set_parameters([Parameter('steering.max_limit', value=100.0)])[0]
    assert result.successful
    assert harness.wait_for(lambda: harness.state(1) == harness.ticks_for_angle(100.0))
    assert node._configs['steering'].max_limit == 100.0

    result = node.set_parameters([Parameter('steering.home', value=500.0)])[0]
    assert not result.successful
    assert 'home' in result.reason
    assert node.get_parameter('steering.home').value == 85.0

    result = node.set_parameters([Parameter('steering.channel', value=3)])[0]
    assert not result.successful
    assert 'restart' in result.reason

    result = node.set_parameters([Parameter('pwm_frequency', value=10.0)])[0]
    assert not result.successful
    assert 'pwm_frequency' in result.reason

    result = node.set_parameters([Parameter('pwm_frequency', value=100)])[0]
    assert result.successful
    assert harness.wait_for(lambda: node.pca.frequency == pytest.approx(100.06, abs=0.01))
    assert harness.wait_for(lambda: harness.state(1) == harness.ticks_for_angle(100.0))
    assert harness.state(1) != harness.ticks_for_pulse(750.0)


def test_joint_state_input_and_output(ros_context):
    params = {**BASE_PARAMS, 'joint_state_topic': 'joint_commands',
              'joint_state_publish_rate': 20.0, 'steering.joint': 'steer_joint'}
    harness = Harness(params)
    try:
        command = JointState()
        command.name = ['other_joint', 'steer_joint']
        command.position = [1.0, math.radians(60.0)]
        harness.publish('/joint_commands', JointState, command,
                        lambda: harness.state(1) == harness.ticks_for_angle(60.0))
        received = []
        harness.helper.create_subscription(JointState, '/joint_states', received.append, 10)
        assert harness.wait_for(lambda: any(msg.name == ['steer_joint'] for msg in received))
        latest = received[-1]
        assert latest.position[0] == pytest.approx(math.radians(60.0))
    finally:
        harness.close()


def test_shutdown_modes(ros_context):
    params = {**BASE_PARAMS, 'steering.on_shutdown': 'home', 'esc.on_shutdown': 'hold',
              'led.on_shutdown': 'off'}
    harness = Harness(params)
    try:
        harness.publish(harness.topic('steering', 'angle'), Float64, Float64(data=120.0),
                        lambda: harness.state(1) == harness.ticks_for_angle(120.0))
        harness.publish(harness.topic('esc', 'throttle'), Float64, Float64(data=0.5),
                        lambda: harness.state(0) == harness.ticks_for_pulse(1750.0))
        harness.publish(harness.topic('led', 'duty_cycle'), Float64, Float64(data=0.5),
                        lambda: harness.state(15) == 2048)
        home_ticks = harness.ticks_for_angle(85.0)
        throttle_ticks = harness.ticks_for_pulse(1750.0)
        harness.node.shutdown_outputs()
        assert harness.state(1) == home_ticks
        assert harness.state(0) == throttle_ticks
        assert harness.state(15) == 'off'
        assert harness.bus.closed
        assert harness.node.pca is None
    finally:
        harness.close()


def test_recovers_when_the_chip_resets(harness):
    # The ESC channel has no timeout, so its command must survive the recovery.
    harness.publish(harness.topic('esc', 'throttle'), Float64, Float64(data=0.5),
                    lambda: harness.state(0) == harness.ticks_for_pulse(1750.0))
    harness.bus.registers[0x00] = 0x11  # power-on MODE1: sleeping, all-call on
    harness.bus.registers[0xFE] = 0x1E
    harness.bus.registers[0x06:0x46] = bytes(0x40)  # the outputs are gone too
    assert harness.state(0) == 0
    assert harness.wait_for(
        lambda: (harness.node.pca.is_configured()
                 and harness.state(0) == harness.ticks_for_pulse(1750.0)),
        timeout=4.0)
    assert harness.state(1) == 'off' or harness.state(1) == harness.ticks_for_angle(85.0)


def test_integer_values_and_unknown_parameters_are_tolerated(ros_context):
    params = {**BASE_PARAMS, 'steering.home': 85, 'esc.timeout': 1, 'steerng.home': 90.0,
              'steering.hom': 1.0, 'unrelated': 'x'}
    harness = Harness(params)
    try:
        assert harness.node._configs['steering'].home == 85.0
        assert harness.node._configs['esc'].timeout == 1.0
    finally:
        harness.close()


@pytest.mark.parametrize('params, message', [
    ({'simulate': True}, "'channels'"),
    ({'simulate': True, 'channels': []}, "'channels'"),
    ({'simulate': True, 'channels': ['a', 'a'], 'a.channel': 0}, 'twice'),
    ({'simulate': True, 'channels': ['a', 'b'], 'a.channel': 0, 'b.channel': 0},
     'both use output'),
    ({'simulate': True, 'channels': ['a']}, "'a.channel' is required"),
    ({'simulate': True, 'channels': ['a'], 'a.channel': 0, 'a.type': 'x'}, "'a.type'"),
    ({'simulate': True, 'channels': ['a'], 'a.channel': 0, 'pwm_frequency': 5.0}, 'pwm_frequency'),
    ({'simulate': True, 'channels': ['a'], 'a.channel': 0, 'i2c_address': 1}, 'i2c_address'),
    ({'simulate': True, 'channels': ['a'], 'a.channel': 0, 'a.twist.linear_x': 1.0,
      'twist_topic': ''}, 'twist_topic'),
    ({'simulate': 'yes', 'channels': ['a'], 'a.channel': 0}, "'simulate'"),
])
def test_configuration_errors_are_reported(ros_context, params, message):
    overrides = [Parameter(name, value=value) for name, value in params.items()]
    with pytest.raises(ConfigError, match=message):
        Pca9685Node(f'pca9685_test_{next(_counter)}', parameter_overrides=overrides)


ESC_PARAMS = {
    'simulate': True,
    'channels': ['drive', 'plain'],
    'drive.channel': 0,
    'drive.type': 'continuous',
    'drive.min_pulse_us': 1000.0,
    'drive.max_pulse_us': 2000.0,
    'drive.esc.arming.values': [0.0, 0.1],
    'drive.esc.arming.durations': [0.4, 0.2],
    'drive.esc.to_reverse.values': [-0.3, 0.0],
    'drive.esc.to_reverse.durations': [0.2, 0.2],
    'drive.esc.deadband': 0.05,
    'drive.esc.forward_start': 0.2,
    'plain.channel': 1,
    'plain.type': 'continuous',
}

# 0.5 throttle shaped for a 0.05 dead-band and a 0.2 forward start.
FORWARD_HALF = 0.2 + (0.5 - 0.05) / 0.95 * 0.8
# -0.4 throttle shaped for the dead-band alone.
REVERSE_04 = -(0.4 - 0.05) / 0.95


@pytest.fixture
def esc_harness(ros_context):
    harness = Harness(ESC_PARAMS)
    yield harness
    harness.close()


def wait_for_arming(harness):
    """Command half throttle and wait until the arming steps have played out."""
    harness.publish(harness.topic('drive', 'throttle'), Float64, Float64(data=0.5),
                    lambda: harness.state(0) == harness.ticks_for_pulse(1550.0))
    harness.publish(harness.topic('drive', 'throttle'), Float64, Float64(data=0.5),
                    lambda: harness.state(0) == forward_half_ticks(harness))


def forward_half_ticks(harness):
    return harness.ticks_for_pulse(1500.0 + 500.0 * FORWARD_HALF)


def reverse_04_ticks(harness):
    return harness.ticks_for_pulse(1500.0 + 500.0 * REVERSE_04)


def test_esc_arms_at_startup_then_applies_the_command(esc_harness):
    harness = esc_harness
    assert harness.state(0) == harness.ticks_for_pulse(1500.0)  # first arming step: neutral
    assert harness.state(1) == 'off'  # a plain continuous channel waits for a command
    wait_for_arming(harness)
    assert harness.node._states['drive'].sequencer.busy is False
    assert harness.node._states['drive'].sequencer.mode == 'forward'
    topics = dict(harness.node.get_topic_names_and_types())
    assert topics[f'/{harness.name}/drive/throttle'] == ['std_msgs/msg/Float64']
    services = dict(harness.node.get_service_names_and_types())
    assert services[f'/{harness.name}/drive/arm'] == ['std_srvs/srv/Trigger']


def test_esc_reverse_entry_sequence(esc_harness):
    harness = esc_harness
    wait_for_arming(harness)
    harness.publish(harness.topic('drive', 'throttle'), Float64, Float64(data=-0.4),
                    lambda: harness.state(0) == harness.ticks_for_pulse(1350.0))  # brake tap
    harness.publish(harness.topic('drive', 'throttle'), Float64, Float64(data=-0.4),
                    lambda: harness.state(0) == harness.ticks_for_pulse(1500.0))  # neutral
    harness.publish(harness.topic('drive', 'throttle'), Float64, Float64(data=-0.4),
                    lambda: harness.state(0) == reverse_04_ticks(harness))
    assert harness.node._states['drive'].sequencer.mode == 'reverse'
    # Forward again is direct: no to_forward sequence is configured.
    harness.publish(harness.topic('drive', 'throttle'), Float64, Float64(data=0.5),
                    lambda: harness.state(0) == forward_half_ticks(harness))
    assert harness.node._states['drive'].sequencer.mode == 'forward'


def call_arm(harness, channel):
    client = harness.helper.create_client(Trigger, harness.topic(channel, 'arm'))
    assert harness.wait_for(lambda: client.service_is_ready())
    future = client.call_async(Trigger.Request())
    assert harness.wait_for(future.done)
    harness.helper.destroy_client(client)
    return future.result()


def test_arm_service_restarts_the_arming_sequence(esc_harness):
    harness = esc_harness
    wait_for_arming(harness)
    result = call_arm(harness, 'drive')
    assert result.success is True
    assert 'arming drive: 2 step(s), 0.6 s' == result.message
    assert harness.state(0) == harness.ticks_for_pulse(1500.0)
    assert harness.node._states['drive'].sequencer.sequence == 'arming'
    assert harness.wait_for(lambda: harness.state(0) == harness.ticks_for_pulse(1550.0))
    assert harness.wait_for(lambda: harness.state(0) == forward_half_ticks(harness))
    result = call_arm(harness, 'plain')
    assert result.success is False
    assert 'no esc.arming sequence' in result.message


def test_esc_rearms_after_a_chip_reset(esc_harness):
    harness = esc_harness
    wait_for_arming(harness)
    harness.bus.registers[0x00] = 0x11  # power-on MODE1: sleeping, all-call on
    harness.bus.registers[0xFE] = 0x1E
    harness.bus.registers[0x06:0x46] = bytes(0x40)  # the outputs are gone too
    assert harness.wait_for(
        lambda: harness.state(0) == harness.ticks_for_pulse(1550.0), timeout=4.0)
    assert harness.wait_for(lambda: harness.state(0) == forward_half_ticks(harness))
    assert harness.node.pca.is_configured()


def test_raw_pulse_cancels_esc_sequences(esc_harness):
    harness = esc_harness
    harness.publish(harness.topic('drive', 'pulse_width'), Float64, Float64(data=1700.0),
                    lambda: harness.state(0) == harness.ticks_for_pulse(1700.0))
    assert harness.node._states['drive'].sequencer.busy is False
    harness.spin(0.7)
    assert harness.state(0) == harness.ticks_for_pulse(1700.0)
