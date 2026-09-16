#ifndef PCA9685_HARDWARE_INTERFACE__SERVO_MAPPING_HPP_
#define PCA9685_HARDWARE_INTERFACE__SERVO_MAPPING_HPP_

#include <algorithm>

namespace pca9685_hardware_interface
{

// Calibration of one position-controlled servo channel. Every field is read
// from the joint's <param> entries in the URDF <ros2_control> block.
struct ServoConfig
{
  double min_angle = -0.5;       // rad, reached at min_pulse_us
  double max_angle = 0.5;        // rad, reached at max_pulse_us
  double offset = 0.0;           // rad, added to the command before clamping (trim)
  double min_pulse_us = 1000.0;
  double neutral_pulse_us = 1500.0;  // pulse at angle 0
  double max_pulse_us = 2000.0;
};

// Commanded angle (rad) to servo pulse width (us). Piecewise linear through
// (min_angle, min_pulse), (0, neutral_pulse), (max_angle, max_pulse), so an
// asymmetric lock or an off-centre servo horn is expressed by the config,
// not by the caller. The result is clamped to the pulse range.
inline double servo_pulse_us(double angle_command, const ServoConfig & c)
{
  const double angle = std::clamp(angle_command + c.offset, c.min_angle, c.max_angle);
  double pulse;
  if (angle >= 0.0) {
    const double span = c.max_angle > 0.0 ? c.max_angle : 1.0;
    pulse = c.neutral_pulse_us + (angle / span) * (c.max_pulse_us - c.neutral_pulse_us);
  } else {
    const double span = c.min_angle < 0.0 ? -c.min_angle : 1.0;
    pulse = c.neutral_pulse_us + (angle / span) * (c.neutral_pulse_us - c.min_pulse_us);
  }
  return std::clamp(pulse, c.min_pulse_us, c.max_pulse_us);
}

}  // namespace pca9685_hardware_interface

#endif  // PCA9685_HARDWARE_INTERFACE__SERVO_MAPPING_HPP_
