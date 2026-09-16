#include "pca9685_hardware_interface/pwm_motor_controller.hpp"

#include <algorithm>
#include <cmath>

namespace pca9685_hardware_interface
{

void PwmMotorController::configure(const Config & config)
{
  config_ = config;
  if (config_.max_wheel_speed_rad_s <= 0.0) {
    config_.max_wheel_speed_rad_s = Config{}.max_wheel_speed_rad_s;
  }
  config_.max_output = std::clamp(std::abs(config_.max_output), 0.0, 1.0);
  config_.forward_offset = std::clamp(config_.forward_offset, 0.0, config_.max_output);
  config_.reverse_offset = std::clamp(config_.reverse_offset, -config_.max_output, 0.0);
  config_.reverse_tap_output = std::clamp(config_.reverse_tap_output, 0.0, 1.0);
  state_ = MotorState::INITIALIZING;
  time_in_state_s_ = 0.0;
  time_since_command_s_ = 0.0;
  target_command_ = 0.0;
  current_duty_cycle_ = config_.neutral_pulse_ms;
}

void PwmMotorController::set_command(double wheel_speed_rad_s)
{
  target_command_ = wheel_speed_rad_s;
  time_since_command_s_ = 0.0;
}

bool PwmMotorController::is_command_forward() const
{
  return target_command_ >= config_.input_deadband_rad_s;
}

bool PwmMotorController::is_command_reverse() const
{
  return target_command_ <= -config_.input_deadband_rad_s;
}

void PwmMotorController::enter(MotorState next)
{
  state_ = next;
  time_in_state_s_ = 0.0;
}

double PwmMotorController::output_to_duty_cycle(double output) const
{
  if (output > 0.0) {
    return config_.neutral_pulse_ms + output * (config_.max_pulse_ms - config_.neutral_pulse_ms);
  }
  if (output < 0.0) {
    return config_.neutral_pulse_ms + output * (config_.neutral_pulse_ms - config_.min_pulse_ms);
  }
  return config_.neutral_pulse_ms;
}

double PwmMotorController::compute_duty_cycle(double command) const
{
  if (std::abs(command) < config_.input_deadband_rad_s) {
    return config_.neutral_pulse_ms;
  }
  // rad/s -> fraction of full speed, then onto the usable throttle band
  // [offset, max_output] so the smallest moving command just clears the
  // ESC dead-band and max_wheel_speed_rad_s lands exactly on max_output.
  const double fraction = std::clamp(command / config_.max_wheel_speed_rad_s, -1.0, 1.0);
  double output;
  if (fraction > 0.0) {
    output = config_.forward_offset + fraction * (config_.max_output - config_.forward_offset);
  } else {
    output = config_.reverse_offset + fraction * (config_.max_output + config_.reverse_offset);
  }
  output = std::clamp(output, -config_.max_output, config_.max_output);
  return output_to_duty_cycle(output);
}

void PwmMotorController::update(double dt)
{
  if (dt < 0.0) { dt = 0.0; }
  time_since_command_s_ += dt;
  time_in_state_s_ += dt;

  if (config_.watchdog_timeout_s > 0.0 && time_since_command_s_ > config_.watchdog_timeout_s) {
    target_command_ = 0.0;
  }

  switch (state_) {
    case MotorState::INITIALIZING:
      current_duty_cycle_ = config_.neutral_pulse_ms;
      enter(MotorState::ARMING_NEUTRAL_1);
      break;

    case MotorState::ARMING_NEUTRAL_1:
      current_duty_cycle_ = config_.neutral_pulse_ms;
      if (time_in_state_s_ >= config_.arming_neutral_s) { enter(MotorState::ARMING_PULSE); }
      break;

    case MotorState::ARMING_PULSE:
      current_duty_cycle_ = output_to_duty_cycle(config_.arming_pulse_output);
      if (time_in_state_s_ >= config_.arming_pulse_s) { enter(MotorState::ARMING_NEUTRAL_2); }
      break;

    case MotorState::ARMING_NEUTRAL_2:
      current_duty_cycle_ = config_.neutral_pulse_ms;
      if (time_in_state_s_ >= config_.arming_settle_s) { enter(MotorState::FORWARD); }
      break;

    case MotorState::FORWARD:
      if (is_command_reverse()) {
        enter(MotorState::TO_REVERSE_NEUTRAL_1);
        current_duty_cycle_ = config_.neutral_pulse_ms;
      } else {
        current_duty_cycle_ = compute_duty_cycle(target_command_);
      }
      break;

    case MotorState::TO_REVERSE_NEUTRAL_1:
      current_duty_cycle_ = config_.neutral_pulse_ms;
      if (time_in_state_s_ >= config_.reverse_brake_s) { enter(MotorState::TO_REVERSE_PULSE); }
      break;

    case MotorState::TO_REVERSE_PULSE:
      // The tap the ESC needs before it will reverse. Sending neutral here
      // (as the first rewrite did) leaves the ESC in forward mode and every
      // reverse command does nothing.
      current_duty_cycle_ = output_to_duty_cycle(config_.reverse_tap_output);
      if (time_in_state_s_ >= config_.reverse_release_s) { enter(MotorState::TO_REVERSE_NEUTRAL_2); }
      break;

    case MotorState::TO_REVERSE_NEUTRAL_2:
      current_duty_cycle_ = config_.neutral_pulse_ms;
      if (time_in_state_s_ >= config_.reverse_settle_s) { enter(MotorState::REVERSE); }
      break;

    case MotorState::REVERSE:
      if (is_command_forward()) {
        enter(MotorState::TO_FORWARD_NEUTRAL);
        current_duty_cycle_ = config_.neutral_pulse_ms;
      } else {
        current_duty_cycle_ = compute_duty_cycle(target_command_);
      }
      break;

    case MotorState::TO_FORWARD_NEUTRAL:
      current_duty_cycle_ = config_.neutral_pulse_ms;
      if (time_in_state_s_ >= config_.forward_settle_s) { enter(MotorState::FORWARD); }
      break;
  }
}

double PwmMotorController::get_velocity() const
{
  if (std::abs(current_duty_cycle_ - config_.neutral_pulse_ms) < 0.001) {
    return 0.0;
  }
  if (state_ != MotorState::FORWARD && state_ != MotorState::REVERSE) {
    return 0.0;
  }
  return target_command_;
}

}  // namespace pca9685_hardware_interface
