#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_
#define PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pca9685_hardware_interface/visibility_control.h"
#include <pca9685_hardware_interface/pca9685_comm.h>
#include "pca9685_hardware_interface/pwm_motor_controller.hpp"
#include "pca9685_hardware_interface/servo_mapping.hpp"

namespace pca9685_hardware_interface
{
class Pca9685SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // One entry per joint. Everything here is parsed from the joint's
  // <param> entries in the URDF <ros2_control> block; see on_init.
  struct JointConfig {
    int channel = 0;
    std::string interface_type;   // "position" (servo), "velocity" (ESC) or "effort" (LED)
    ServoConfig servo;            // position joints
    PwmMotorController::Config motor;  // velocity joints
  };

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<JointConfig> joint_configs_;
  std::unique_ptr<PiPCA9685::PCA9685> pca;
  std::vector<PwmMotorController> motor_controllers_;

  static double command_to_duty_cycle_effort(double command);
};

}  // namespace pca9685_hardware_interface

#endif  // PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_
