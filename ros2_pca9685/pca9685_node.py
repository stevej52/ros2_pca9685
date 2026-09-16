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
ROS 2 node that exposes every output of a PCA9685 as a configurable channel.

Channels are described in the node's parameters (normally loaded from a YAML
file, see ``config/example.yaml``).  Each channel gets its own command topics,
optionally follows a ``geometry_msgs/Twist`` topic, and can be driven from
``sensor_msgs/JointState`` messages.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import signal
import sys
from typing import Mapping

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_pca9685 import channels, pca9685
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

WATCHDOG_PERIOD = 0.05  # seconds between timeout checks
HEALTH_CHECK_PERIOD = 2.0  # seconds between checks that the chip kept its configuration

# Node-wide parameters and their defaults.  ``channels`` has no default: it
# must be supplied.
GLOBAL_PARAMS: dict[str, object] = {
    'i2c_bus': 1,
    'i2c_address': 0x40,
    'pwm_frequency': 50.0,
    'oscillator_frequency': pca9685.DEFAULT_OSCILLATOR_HZ,
    'simulate': False,
    'channels': None,
    'twist_topic': 'cmd_vel',
    'joint_state_topic': '',
    'joint_state_publish_rate': 0.0,
}

# Parameters that only take effect when the node starts.
RESTART_ONLY = (
    'i2c_bus', 'i2c_address', 'simulate', 'channels',
    'twist_topic', 'joint_state_topic', 'joint_state_publish_rate')
RESTART_ONLY_CHANNEL = ('channel', 'type', 'joint')

MIN_I2C_ADDRESS = 0x40
MAX_I2C_ADDRESS = 0x7F


@dataclass
class _ChannelState:
    """What the node last wrote to one channel."""

    value: float | None = None  # last command in the channel's units; None = output off
    pulse_us: float | None = None  # set instead of ``value`` after a raw pulse width command
    stamp: Time | None = None  # when the last command arrived, for the timeout

    @property
    def active(self) -> bool:
        """Return whether the output is currently producing pulses."""
        return self.value is not None or self.pulse_us is not None


def _twist_axes(msg: Twist) -> dict[str, float]:
    return {
        'linear_x': msg.linear.x, 'linear_y': msg.linear.y, 'linear_z': msg.linear.z,
        'angular_x': msg.angular.x, 'angular_y': msg.angular.y, 'angular_z': msg.angular.z,
    }


class Pca9685Node(Node):
    """Drive the outputs of a PCA9685 from ROS topics."""

    def __init__(self, node_name: str = 'pca9685', **kwargs) -> None:
        super().__init__(
            node_name, automatically_declare_parameters_from_overrides=True, **kwargs)
        self.bus = None
        self.pca: pca9685.Pca9685 | None = None
        self._config_dirty = False
        self._command_subscriptions = []

        self._names = self._declare_parameters()
        self._settings = self._read_settings()
        self._configs = self._read_channel_configs()
        self._states = {name: _ChannelState() for name in self._configs}
        self._write_back_resolved_defaults()
        self._joint_to_channel = {
            config.joint: name for name, config in self._configs.items()
            if config.kind == channels.SERVO}

        self.bus, self.pca = self._open_board()
        self._last_health_check = self.get_clock().now()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, config in self._configs.items():
            self._command_subscriptions.append(self.create_subscription(
                Float64, f'~/{name}/{config.command_topic}', self._value_callback(name), qos))
            self._command_subscriptions.append(self.create_subscription(
                Float64, f'~/{name}/pulse_width', self._pulse_callback(name), qos))
        if any(config.twist_driven for config in self._configs.values()):
            self._command_subscriptions.append(self.create_subscription(
                Twist, self._settings['twist_topic'], self._on_twist, qos))
        if self._settings['joint_state_topic']:
            self._command_subscriptions.append(self.create_subscription(
                JointState, self._settings['joint_state_topic'], self._on_joint_state, qos))
        self._joint_state_publisher = None
        rate = self._settings['joint_state_publish_rate']
        if rate > 0.0:
            self._joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
            self.create_timer(1.0 / rate, self._publish_joint_states)
        self.create_timer(WATCHDOG_PERIOD, self._watchdog)

        for name, config in self._configs.items():
            self.get_logger().info(f'{name}: {config.describe()}')
            if config.home_on_start:
                self._command(name, config.home, 'home_on_start', stamp=False)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.get_logger().info(
            f'{len(self._configs)} channel(s) ready; command topics are under '
            f'{self.get_fully_qualified_name()}/<channel>/')

    # ----------------------------------------------------------------- parameters --

    def _declare(self, name: str, default: object) -> None:
        # Parameters found in the YAML file were auto-declared with a fixed type.
        # Re-declare them with dynamic typing so that "90" and "90.0" are both fine.
        if self.has_parameter(name):
            self.undeclare_parameter(name)
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))

    def _declare_parameters(self) -> list[str]:
        for name, default in GLOBAL_PARAMS.items():
            self._declare(name, default)
        names = self._channel_names()
        for name in names:
            for key, default in channels.CHANNEL_PARAMS.items():
                self._declare(f'{name}.{key}', default)
        self._warn_about_unknown_parameters(names)
        return names

    def _channel_names(self) -> list[str]:
        value = self.get_parameter('channels').value
        if isinstance(value, str):
            value = [value]
        if not isinstance(value, (list, tuple)) or not value:
            raise channels.ConfigError(
                "'channels' must list at least one channel name, "
                'for example: channels: [steering, throttle]')
        names = [channels.validate_channel_name(name) for name in value]
        for name in names:
            if names.count(name) > 1:
                raise channels.ConfigError(f"Channel '{name}' is listed twice in 'channels'")
            if name in GLOBAL_PARAMS:
                raise channels.ConfigError(f"'{name}' cannot be used as a channel name")
        return names

    def _warn_about_unknown_parameters(self, names: list[str]) -> None:
        known = set(GLOBAL_PARAMS) | {'use_sim_time', 'start_type_description_service'}
        known.update(f'{name}.{key}' for name in names for key in channels.CHANNEL_PARAMS)
        for full_name in sorted(self.get_parameters_by_prefix('')):
            if full_name in known or full_name.startswith('qos_overrides.'):
                continue
            prefix, _, rest = full_name.partition('.')
            if prefix in names:
                hint = f"'{prefix}' has no setting called '{rest}'"
            elif rest:
                hint = f"'{prefix}' is not listed in 'channels'"
            else:
                hint = 'not a setting of this node'
            self.get_logger().warning(f"Ignoring unknown parameter '{full_name}': {hint}")

    def _param_value(self, name: str, overrides: Mapping[str, object] | None = None) -> object:
        if overrides is not None and name in overrides:
            return overrides[name]
        return self.get_parameter(name).value

    def _read_settings(self, overrides: Mapping[str, object] | None = None) -> dict[str, object]:
        """Return the validated node-wide settings, optionally with proposed new values."""
        values = {name: self._param_value(name, overrides) for name in GLOBAL_PARAMS}
        for name in GLOBAL_PARAMS:
            if values[name] is None and name != 'channels':
                values[name] = GLOBAL_PARAMS[name]

        def integer(name: str, lowest: int, highest: int) -> int:
            value = values[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or float(value) != int(value) or not lowest <= int(value) <= highest:
                raise channels.ConfigError(
                    f"'{name}' must be a whole number from {lowest} to {highest}, got {value!r}")
            return int(value)

        def number(name: str) -> float:
            value = values[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value):
                raise channels.ConfigError(f"'{name}' must be a number, got {value!r}")
            return float(value)

        def text(name: str) -> str:
            value = values[name]
            if not isinstance(value, str):
                raise channels.ConfigError(f"'{name}' must be a string, got {value!r}")
            return value

        settings: dict[str, object] = {
            'i2c_bus': integer('i2c_bus', 0, 999),
            'i2c_address': integer('i2c_address', MIN_I2C_ADDRESS, MAX_I2C_ADDRESS),
            'pwm_frequency': number('pwm_frequency'),
            'oscillator_frequency': number('oscillator_frequency'),
            'twist_topic': text('twist_topic'),
            'joint_state_topic': text('joint_state_topic'),
            'joint_state_publish_rate': number('joint_state_publish_rate'),
        }
        if not isinstance(values['simulate'], bool):
            raise channels.ConfigError(
                f"'simulate' must be true or false, got {values['simulate']!r}")
        settings['simulate'] = values['simulate']
        if settings['oscillator_frequency'] <= 0.0:
            raise channels.ConfigError("'oscillator_frequency' must be positive")
        try:
            pca9685.Pca9685.prescale_for(
                settings['pwm_frequency'], settings['oscillator_frequency'])
        except ValueError as exc:
            raise channels.ConfigError(f"'pwm_frequency': {exc}") from exc
        if settings['joint_state_publish_rate'] < 0.0:
            raise channels.ConfigError("'joint_state_publish_rate' must not be negative")
        return settings

    def _read_channel_configs(
        self, overrides: Mapping[str, object] | None = None,
    ) -> dict[str, channels.ChannelConfig]:
        """Return the validated channel configurations, optionally with proposed new values."""
        values_by_name = {}
        for name in self._names:
            values_by_name[name] = {
                key: self._param_value(f'{name}.{key}', overrides)
                for key in channels.CHANNEL_PARAMS}
        configs = channels.parse_channels(values_by_name)
        twist_driven = [name for name, config in configs.items() if config.twist_driven]
        if twist_driven and not self._param_value('twist_topic', overrides):
            raise channels.ConfigError(
                f"'twist_topic' must be set because {', '.join(twist_driven)} use "
                'twist gains')
        return configs

    def _write_back_resolved_defaults(self) -> None:
        """Fill in derived defaults so ``ros2 param dump`` shows the complete configuration."""
        updates = []
        for name, config in self._configs.items():
            for key, value in config.as_params().items():
                full_name = f'{name}.{key}'
                if self.get_parameter(full_name).value != value:
                    updates.append(Parameter(full_name, value=value))
        if updates:
            self.set_parameters(updates)

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        proposed = {param.name: param.value for param in params}
        for full_name in proposed:
            prefix, _, key = full_name.partition('.')
            restart_only = prefix in self._names and key in RESTART_ONLY_CHANNEL
            if full_name in RESTART_ONLY or restart_only:
                return SetParametersResult(
                    successful=False,
                    reason=f"'{full_name}' can only be changed by restarting the node")
        try:
            self._read_settings(proposed)
            self._read_channel_configs(proposed)
        except channels.ConfigError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        # The new values are applied by the watchdog once rclpy has stored them.
        self._config_dirty = True
        return SetParametersResult(successful=True)

    def _reload_configuration(self) -> None:
        try:
            settings = self._read_settings()
            configs = self._read_channel_configs()
        except channels.ConfigError as exc:
            self.get_logger().error(f'Configuration not applied: {exc}')
            return
        frequency_changed = (
            settings['pwm_frequency'] != self._settings['pwm_frequency']
            or settings['oscillator_frequency'] != self._settings['oscillator_frequency'])
        self._settings = settings
        changed = []
        for name, config in configs.items():
            if config != self._configs[name]:
                self._configs[name] = config
                changed.append(name)
                self.get_logger().info(f'{name}: {config.describe()}')
            state = self._states[name]
            if state.value is not None:
                state.value = config.clamp(state.value)
        if frequency_changed:
            self.pca.oscillator_hz = settings['oscillator_frequency']
            try:
                actual = self.pca.set_frequency(settings['pwm_frequency'])
            except OSError as exc:
                self.get_logger().error(f'Could not change the PWM frequency: {exc}')
                return
            self.get_logger().info(
                f'PWM frequency is now {actual:.2f} Hz '
                f"(requested {settings['pwm_frequency']:g} Hz)")
            changed = list(self._configs)
        for name in changed:
            if self._states[name].active:
                self._apply(name, 'parameter change')

    # ------------------------------------------------------------------ hardware --

    def _open_board(self):
        settings = self._settings
        if settings['simulate']:
            bus = pca9685.FakeI2CBus()
            self.get_logger().warning('Simulation mode: not talking to any hardware')
            where = 'simulated PCA9685'
        else:
            bus = pca9685.LinuxI2CBus(settings['i2c_bus'], settings['i2c_address'])
            where = f"PCA9685 at 0x{settings['i2c_address']:02x} on {bus.path}"
        chip = pca9685.Pca9685(bus, settings['oscillator_frequency'])
        try:
            actual = chip.initialize(settings['pwm_frequency'])
        except OSError:
            bus.close()
            raise
        self.get_logger().info(
            f"{where}: PWM {actual:.2f} Hz (requested {settings['pwm_frequency']:g} Hz)")
        return bus, chip

    def _write_output(self, config: channels.ChannelConfig, value: float) -> str:
        """Write a command in channel units to the chip and describe what was written."""
        if config.kind == channels.PWM:
            duty = config.duty_cycle(value)
            self.pca.set_duty_cycle(config.channel, duty)
            return f'{value:g} {config.units}'
        pulse_us = config.pulse_width_us(value)
        self.pca.set_pulse_width_us(config.channel, pulse_us)
        return f'{value:g} {config.units} = {pulse_us:.0f} us'

    def _apply(self, name: str, source: str) -> None:
        """Write the stored state of a channel to the chip."""
        config = self._configs[name]
        state = self._states[name]
        try:
            if state.pulse_us is not None:
                self.pca.set_pulse_width_us(config.channel, state.pulse_us)
                detail = f'{state.pulse_us:g} us'
            elif state.value is None:
                self.pca.set_off(config.channel)
                detail = 'off'
            else:
                detail = self._write_output(config, state.value)
        except OSError as exc:
            self.get_logger().error(f'{name}: I2C write failed: {exc}', throttle_duration_sec=5.0)
            return
        log = self.get_logger().info if self._settings['simulate'] else self.get_logger().debug
        log(f'{name}: {detail} ({source})')

    def _reapply_all(self, source: str) -> None:
        for name, state in self._states.items():
            if state.active:
                self._apply(name, source)

    def _check_board(self) -> None:
        try:
            if self.pca.is_configured():
                return
        except OSError as exc:
            self.get_logger().error(f'I2C read failed: {exc}', throttle_duration_sec=5.0)
            return
        self.get_logger().warning(
            'The PCA9685 lost its configuration (power glitch or reset?); re-initialising')
        try:
            self.pca.initialize(self._settings['pwm_frequency'])
        except OSError as exc:
            self.get_logger().error(f'Re-initialisation failed: {exc}', throttle_duration_sec=5.0)
            return
        self._reapply_all('recovery')

    # ------------------------------------------------------------------ commands --

    def _command(self, name: str, value: float, source: str, stamp: bool = True) -> None:
        """Store and write a command given in the channel's units."""
        config = self._configs[name]
        state = self._states[name]
        if not math.isfinite(value):
            self.get_logger().warning(
                f'{name}: ignoring non-finite value from {source}', throttle_duration_sec=5.0)
            return
        clamped = config.clamp(value)
        if clamped != value:
            self.get_logger().debug(f'{name}: {value:g} clamped to {clamped:g} {config.units}')
        state.value = clamped
        state.pulse_us = None
        state.stamp = self.get_clock().now() if stamp else None
        self._apply(name, source)

    def _command_pulse(self, name: str, pulse_us: float, source: str) -> None:
        """Store and write a raw pulse width; zero or less switches the output off."""
        state = self._states[name]
        if not math.isfinite(pulse_us):
            self.get_logger().warning(
                f'{name}: ignoring non-finite pulse width from {source}',
                throttle_duration_sec=5.0)
            return
        state.value = None
        state.pulse_us = pulse_us if pulse_us > 0.0 else None
        state.stamp = self.get_clock().now() if state.pulse_us is not None else None
        self._apply(name, source)

    def _value_callback(self, name: str):
        def callback(msg: Float64) -> None:
            self._command(name, msg.data, 'topic')
        return callback

    def _pulse_callback(self, name: str):
        def callback(msg: Float64) -> None:
            self._command_pulse(name, msg.data, 'pulse_width topic')
        return callback

    def _on_twist(self, msg: Twist) -> None:
        axes = _twist_axes(msg)
        for name, config in self._configs.items():
            if config.twist_driven:
                self._command(name, config.twist_value(axes), 'twist')

    def _on_joint_state(self, msg: JointState) -> None:
        for index, joint in enumerate(msg.name):
            name = self._joint_to_channel.get(joint)
            if name is None or index >= len(msg.position):
                continue
            self._command(name, math.degrees(msg.position[index]), 'joint_state')

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, config in self._configs.items():
            state = self._states[name]
            if config.kind == channels.SERVO and state.value is not None:
                msg.name.append(config.joint)
                msg.position.append(math.radians(state.value))
        self._joint_state_publisher.publish(msg)

    def _watchdog(self) -> None:
        now = self.get_clock().now()
        for name, config in self._configs.items():
            state = self._states[name]
            if config.timeout <= 0.0 or state.stamp is None:
                continue
            if (now - state.stamp).nanoseconds >= config.timeout * 1e9:
                self.get_logger().warning(
                    f'{name}: no command for {config.timeout:g} s, returning to home '
                    f'({config.home:g} {config.units})')
                self._command(name, config.home, 'timeout', stamp=False)
        if self._config_dirty:
            self._config_dirty = False
            self._reload_configuration()
        if (now - self._last_health_check).nanoseconds >= HEALTH_CHECK_PERIOD * 1e9:
            self._last_health_check = now
            self._check_board()

    # ------------------------------------------------------------------ shutdown --

    def shutdown_outputs(self) -> None:
        """Apply each channel's ``on_shutdown`` setting and release the bus."""
        if self.pca is None:
            return
        for name, config in self._configs.items():
            try:
                if config.on_shutdown == channels.SHUTDOWN_OFF:
                    self.pca.set_off(config.channel)
                elif config.on_shutdown == channels.SHUTDOWN_HOME:
                    self._write_output(config, config.home)
            except OSError as exc:
                self._log_late('error', f'{name}: I2C write failed during shutdown: {exc}')
        self.pca.close()
        self.pca = None
        self._log_late('info', 'Outputs set as configured by on_shutdown; bus closed')

    def _log_late(self, level: str, message: str) -> None:
        """Log during shutdown, when the ROS context may already be gone."""
        if self.context.ok():
            getattr(self.get_logger(), level)(message)
        else:
            print(f'[{level.upper()}] [{self.get_name()}]: {message}', file=sys.stderr)


def main(args=None) -> int:
    """Run the node until it is interrupted."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = Pca9685Node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (channels.ConfigError, pca9685.I2CError) as exc:
        get_logger('pca9685').fatal(str(exc))
        exit_code = 1
    finally:
        # ros2 launch and a terminal Ctrl-C can both deliver a SIGINT; a second
        # one must not cut the clean-up short and leave outputs running.
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(signum, signal.SIG_IGN)
            except (ValueError, OSError):
                pass  # not the main thread
        if node is not None:
            node.shutdown_outputs()
            node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
