#ifndef PCA9685_HARDWARE_INTERFACE__PWM_MOTOR_CONTROLLER_HPP_
#define PCA9685_HARDWARE_INTERFACE__PWM_MOTOR_CONTROLLER_HPP_

// Pure module: no ROS, no I2C, no clock. The hardware interface feeds it the
// commanded wheel speed and the loop period; it answers with a pulse width.

namespace pca9685_hardware_interface
{

class PwmMotorController
{
public:
  enum class MotorState {
    INITIALIZING,
    ARMING_NEUTRAL_1,
    ARMING_PULSE,
    ARMING_NEUTRAL_2,
    FORWARD,
    REVERSE,
    TO_REVERSE_NEUTRAL_1,
    TO_REVERSE_PULSE,
    TO_REVERSE_NEUTRAL_2,
    TO_FORWARD_NEUTRAL
  };

  // ESC calibration. Every field is read from the traction joint's <param>
  // entries in the URDF <ros2_control> block; the defaults here are the
  // fallbacks when a param is absent, and match the car as last driven.
  struct Config {
    // Unit contract at the seam: the command is rear-wheel angular velocity
    // in rad/s, as emitted by the bicycle steering controller (v / wheel_radius).
    double max_wheel_speed_rad_s = 10.0;   // command that reaches max_output. PLACEHOLDER until measured.
    double input_deadband_rad_s = 0.01;    // |command| below this is neutral

    // Throttle fractions in [-1, 1] of the pulse range either side of neutral.
    double forward_offset = 0.18;   // first fraction that makes the ESC move forward
    double reverse_offset = -0.18;  // first fraction that makes the ESC move backward
    double max_output = 0.40;       // absolute cap; reached at max_wheel_speed_rad_s

    double min_pulse_ms = 1.0;
    double neutral_pulse_ms = 1.5;
    double max_pulse_ms = 2.0;

    double watchdog_timeout_s = 0.0;  // <= 0 disables; command decays to 0 after this silence

    // ESC arming sequence on activation.
    double arming_neutral_s = 2.5;
    double arming_pulse_s = 0.5;
    double arming_pulse_output = 0.05;  // fraction sent during arming_pulse_s
    double arming_settle_s = 0.5;

    // Direction change into reverse, found experimentally on the bench
    // (TestPCA9685ESC.py): neutral, a short forward tap, neutral again, and
    // only then does the ESC accept reverse commands as reverse.
    double reverse_brake_s = 0.20;       // neutral before the tap
    double reverse_tap_output = 0.10;    // fraction sent during the tap
    double reverse_release_s = 0.20;     // tap length
    double reverse_settle_s = 0.20;      // neutral after the tap
    double forward_settle_s = 0.20;
  };

  PwmMotorController() = default;

  // Resets the state machine to INITIALIZING; the next update() starts arming.
  void configure(const Config & config);
  void set_command(double wheel_speed_rad_s);
  // Advance by dt seconds. Call once per control cycle, after set_command.
  void update(double dt);

  double get_duty_cycle() const { return current_duty_cycle_; }  // pulse width, ms
  // Wheel speed the module believes it is producing: the command, or 0 while
  // neutral, arming, or changing direction. There is no encoder; this is an
  // echo, and the steering controller is configured open_loop accordingly.
  double get_velocity() const;
  MotorState get_state() const { return state_; }
  const Config & config() const { return config_; }

private:
  Config config_;
  MotorState state_ = MotorState::INITIALIZING;
  double time_in_state_s_ = 0.0;
  double time_since_command_s_ = 0.0;
  double target_command_ = 0.0;
  double current_duty_cycle_ = 1.5;

  bool is_command_forward() const;
  bool is_command_reverse() const;
  void enter(MotorState next);
  double output_to_duty_cycle(double output) const;
  double compute_duty_cycle(double command) const;
};

}  // namespace pca9685_hardware_interface

#endif  // PCA9685_HARDWARE_INTERFACE__PWM_MOTOR_CONTROLLER_HPP_
