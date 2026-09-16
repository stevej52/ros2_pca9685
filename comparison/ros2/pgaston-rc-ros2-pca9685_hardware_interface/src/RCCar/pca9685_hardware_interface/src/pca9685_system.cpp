#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pca9685_hardware_interface
{
namespace
{
using Params = std::unordered_map<std::string, std::string>;

rclcpp::Logger logger() { return rclcpp::get_logger("Pca9685SystemHardware"); }

// Read an optional numeric param, keeping the default when absent, and record
// the key as consumed so unknown keys can be reported (no decorative params).
double read_param(const Params & params, const std::string & key, double fallback,
                  std::set<std::string> & consumed)
{
  consumed.insert(key);
  const auto it = params.find(key);
  return it == params.end() ? fallback : std::stod(it->second);
}

void warn_unread(const std::string & owner, const Params & params, const std::set<std::string> & consumed)
{
  for (const auto & kv : params) {
    if (!consumed.count(kv.first)) {
      RCLCPP_WARN(logger(), "%s: param '%s' is not read by this plugin; remove it or fix the name",
                  owner.c_str(), kv.first.c_str());
    }
  }
}

ServoConfig parse_servo(const Params & p, std::set<std::string> & consumed)
{
  ServoConfig c;
  c.min_angle = read_param(p, "min_angle", c.min_angle, consumed);
  c.max_angle = read_param(p, "max_angle", c.max_angle, consumed);
  c.offset = read_param(p, "offset", c.offset, consumed);
  c.min_pulse_us = read_param(p, "min_pulse_us", c.min_pulse_us, consumed);
  c.neutral_pulse_us = read_param(p, "neutral_pulse_us", c.neutral_pulse_us, consumed);
  c.max_pulse_us = read_param(p, "max_pulse_us", c.max_pulse_us, consumed);
  return c;
}

PwmMotorController::Config parse_motor(const Params & p, std::set<std::string> & consumed)
{
  PwmMotorController::Config c;
  c.min_pulse_ms = read_param(p, "min_pulse_us", c.min_pulse_ms * 1000.0, consumed) / 1000.0;
  c.neutral_pulse_ms = read_param(p, "neutral_pulse_us", c.neutral_pulse_ms * 1000.0, consumed) / 1000.0;
  c.max_pulse_ms = read_param(p, "max_pulse_us", c.max_pulse_ms * 1000.0, consumed) / 1000.0;
  c.max_wheel_speed_rad_s = read_param(p, "max_wheel_speed_rad_s", c.max_wheel_speed_rad_s, consumed);
  c.input_deadband_rad_s = read_param(p, "input_deadband_rad_s", c.input_deadband_rad_s, consumed);
  c.forward_offset = read_param(p, "forward_offset", c.forward_offset, consumed);
  c.reverse_offset = read_param(p, "reverse_offset", c.reverse_offset, consumed);
  c.max_output = read_param(p, "max_output", c.max_output, consumed);
  c.watchdog_timeout_s = read_param(p, "watchdog_timeout_s", c.watchdog_timeout_s, consumed);
  c.arming_neutral_s = read_param(p, "arming_neutral_s", c.arming_neutral_s, consumed);
  c.arming_pulse_s = read_param(p, "arming_pulse_s", c.arming_pulse_s, consumed);
  c.arming_pulse_output = read_param(p, "arming_pulse_output", c.arming_pulse_output, consumed);
  c.arming_settle_s = read_param(p, "arming_settle_s", c.arming_settle_s, consumed);
  c.reverse_brake_s = read_param(p, "reverse_brake_s", c.reverse_brake_s, consumed);
  c.reverse_tap_output = read_param(p, "reverse_tap_output", c.reverse_tap_output, consumed);
  c.reverse_release_s = read_param(p, "reverse_release_s", c.reverse_release_s, consumed);
  c.reverse_settle_s = read_param(p, "reverse_settle_s", c.reverse_settle_s, consumed);
  c.forward_settle_s = read_param(p, "forward_settle_s", c.forward_settle_s, consumed);
  return c;
}
}  // namespace

hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Hardware params: device, address, frequency.
  std::set<std::string> consumed{"device", "address"};
  std::string device = "/dev/i2c-7";
  int address = 0x40;
  if (auto it = info_.hardware_parameters.find("device"); it != info_.hardware_parameters.end()) {
    device = it->second;
  }
  if (auto it = info_.hardware_parameters.find("address"); it != info_.hardware_parameters.end()) {
    address = std::stoi(it->second);
  }
  const double frequency_hz = read_param(info_.hardware_parameters, "frequency", 50.0, consumed);
  warn_unread("hardware", info_.hardware_parameters, consumed);

  RCLCPP_INFO(logger(), "Opening PCA9685 on %s, address 0x%02X, %.0f Hz", device.c_str(), address, frequency_hz);
  try {
    pca = std::make_unique<PiPCA9685::PCA9685>(device, address);
    pca->set_pwm_freq(frequency_hz);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger(), "Failed to initialize PCA9685: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  joint_configs_.resize(info_.joints.size());
  motor_controllers_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    JointConfig & cfg = joint_configs_[i];

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const std::string & interface_name = joint.command_interfaces[0].name;
    if (interface_name != hardware_interface::HW_IF_VELOCITY &&
        interface_name != hardware_interface::HW_IF_POSITION &&
        interface_name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(logger(), "Joint '%s' has '%s' command interface. Only position, velocity or effort expected.",
                   joint.name.c_str(), interface_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    cfg.interface_type = interface_name;

    std::set<std::string> joint_consumed{"channel"};
    const auto channel_param = joint.parameters.find("channel");
    if (channel_param == joint.parameters.end()) {
      RCLCPP_FATAL(logger(), "Joint '%s' missing required 'channel' parameter.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    cfg.channel = std::stoi(channel_param->second);

    if (interface_name == hardware_interface::HW_IF_POSITION) {
      cfg.servo = parse_servo(joint.parameters, joint_consumed);
      RCLCPP_INFO(logger(), "Servo '%s': channel %d, angle [%.3f, %.3f] rad -> pulse [%.0f, %.0f] us, neutral %.0f us",
                  joint.name.c_str(), cfg.channel, cfg.servo.min_angle, cfg.servo.max_angle,
                  cfg.servo.min_pulse_us, cfg.servo.max_pulse_us, cfg.servo.neutral_pulse_us);
    } else if (interface_name == hardware_interface::HW_IF_VELOCITY) {
      cfg.motor = parse_motor(joint.parameters, joint_consumed);
      RCLCPP_INFO(logger(), "ESC '%s': channel %d, %.1f rad/s -> max_output %.2f, offsets %.2f / %.2f, watchdog %.2f s",
                  joint.name.c_str(), cfg.channel, cfg.motor.max_wheel_speed_rad_s, cfg.motor.max_output,
                  cfg.motor.forward_offset, cfg.motor.reverse_offset, cfg.motor.watchdog_timeout_s);
    } else {
      RCLCPP_INFO(logger(), "LED '%s': channel %d", joint.name.c_str(), cfg.channel);
    }
    warn_unread("joint '" + joint.name + "'", joint.parameters, joint_consumed);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    // Position and velocity for every joint: the steering controller claims both.
    // Neither is measured; see read().
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    if (joint_configs_[i].interface_type == hardware_interface::HW_IF_EFFORT) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, joint_configs_[i].interface_type, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    if (std::isnan(hw_commands_[i])) {
      hw_commands_[i] = 0;
    }
    const JointConfig & cfg = joint_configs_[i];
    if (cfg.interface_type == hardware_interface::HW_IF_VELOCITY) {
      // configure() resets the state machine, so every activation re-arms the ESC.
      motor_controllers_[i].configure(cfg.motor);
      RCLCPP_INFO(logger(), "ESC on channel %d reset. Starting arming sequence.", cfg.channel);
    }
  }
  RCLCPP_INFO(logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// There is no feedback from the servo or the ESC. The state interfaces echo
// the command; the steering controller is configured open_loop: true so its
// odometry never mistakes this for a measurement.
hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    if (std::isnan(hw_commands_[i])) { continue; }
    const JointConfig & cfg = joint_configs_[i];
    if (cfg.interface_type == hardware_interface::HW_IF_POSITION) {
      hw_positions_[i] = std::clamp(hw_commands_[i], cfg.servo.min_angle, cfg.servo.max_angle);
      hw_velocities_[i] = 0.0;
    } else if (cfg.interface_type == hardware_interface::HW_IF_VELOCITY) {
      const double v = motor_controllers_[i].get_velocity();
      hw_positions_[i] += v * period.seconds();
      hw_velocities_[i] = v;
    } else {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
    }
  }
  return hardware_interface::return_type::OK;
}

double Pca9685SystemHardware::command_to_duty_cycle_effort(double command)
{
  // LED brightness 0..1 -> 0..20 ms at 50 Hz (full period)
  return std::clamp(command, 0.0, 1.0) * 20.0;
}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    if (std::isnan(hw_commands_[i])) { continue; }
    const JointConfig & cfg = joint_configs_[i];
    double duty_cycle_ms;
    if (cfg.interface_type == hardware_interface::HW_IF_POSITION) {
      duty_cycle_ms = servo_pulse_us(hw_commands_[i], cfg.servo) / 1000.0;
    } else if (cfg.interface_type == hardware_interface::HW_IF_EFFORT) {
      duty_cycle_ms = command_to_duty_cycle_effort(hw_commands_[i]);
    } else {
      motor_controllers_[i].set_command(hw_commands_[i]);
      motor_controllers_[i].update(period.seconds());
      duty_cycle_ms = motor_controllers_[i].get_duty_cycle();
    }
    pca->set_pwm_ms(cfg.channel, duty_cycle_ms);
    static rclcpp::Clock throttle_clock;
    RCLCPP_DEBUG_THROTTLE(logger(), throttle_clock, 1000,
      "Joint %u (%s): command=%.3f, pulse=%.3f ms, channel=%d",
      i, cfg.interface_type.c_str(), hw_commands_[i], duty_cycle_ms, cfg.channel);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
