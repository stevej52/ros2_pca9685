/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2016 cocasema
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include "pca9685/driver.h"

#include "pwm_msgs/SetSinglePinDutyCycle.h"
#include "pwm_msgs/SetMultiPinDutyCycle.h"

#include <boost/lexical_cast.hpp>
#include <boost/scope_exit.hpp>
#include <ros/ros.h>

namespace pca9685 {

class Service
{
public:
  Service(std::string const& frame_id,
          uint8_t i2c_bus, uint8_t i2c_address,
          uint16_t frequency)
    : frame_id_(frame_id)
    , driver_(i2c_bus, i2c_address)
  {
    if (frequency == 0) {
      ROS_INFO("Service: skipping setting driver frequency");
    }
    else if (!driver_t::frequency_t::value_in_range(frequency)) {
      ROS_ERROR("Service: can't set driver frequency to %u (OOB [%u %u])",
                frequency, driver_t::frequency_t::MIN, driver_t::frequency_t::MAX);
    }
    else {
      ROS_INFO("Service: setting driver frequency to %u", frequency);
      driver_.set_frequency(frequency);
    }
  }

  bool SetSinglePinDutyCycle(ros::ServiceEvent<
                               pwm_msgs::SetSinglePinDutyCycle::Request,
                               pwm_msgs::SetSinglePinDutyCycle::Response
                             >& event)
  {
    auto const& req = event.getRequest();
    auto& rsp = event.getResponse();

    BOOST_SCOPE_EXIT(&rsp) {
      if (!rsp.success) {
        ROS_ERROR("Service: SetSinglePinDutyCycle failed: %s", rsp.error.c_str());
      }
    } BOOST_SCOPE_EXIT_END

    if (!(rsp.success = validatePin(req.requested.pin, rsp.error))
     || !(rsp.success = validateDutyCycle(req.requested.dc, rsp.error))) {
      return false;
    }

    if (!(rsp.success = setPinDutyCycle(req.requested.pin, req.requested.dc, rsp.error))) {
      return false;
    }

    rsp.success = true;
    return true;
  }

  bool SetMultiPinDutyCycle(ros::ServiceEvent<
                              pwm_msgs::SetMultiPinDutyCycle::Request,
                              pwm_msgs::SetMultiPinDutyCycle::Response
                            >& event)
  {
    auto const& req = event.getRequest();
    auto& rsp = event.getResponse();

    BOOST_SCOPE_EXIT(&rsp) {
      if (!rsp.success) {
        ROS_ERROR("Service: SetMultiPinDutyCycle failed: %s", rsp.error.c_str());
      }
    } BOOST_SCOPE_EXIT_END

    for (auto const& pin : req.requested.pins) {
      if (!(rsp.success = validatePin(pin.pin, rsp.error))
       || !(rsp.success = validateDutyCycle(pin.dc, rsp.error))) {
        return false;
      }
    }

    for (auto const& pin : req.requested.pins) {
      if (!(rsp.success = setPinDutyCycle(pin.pin, pin.dc, rsp.error))) {
        return false;
      }
    }

    rsp.success = true;
    return true;
  }

private:
  std::string const frame_id_;

  typedef PCA9685Driver driver_t;
  driver_t driver_;

  static bool validatePin(uint16_t pin, std::string& error)
  {
    if (!driver_t::pin_t::value_in_range(pin)) {
      error = ros::console::formatToString(
          "pin %u is out of valid range [%u %u]",
          pin, driver_t::pin_t::MIN, driver_t::pin_t::MAX);
      return false;
    }
    return true;
  }

  static bool validateDutyCycle(pwm_msgs::DutyCycle const& dc, std::string& error)
  {
    if (dc.use_value) {
      if (!driver_t::value_t::value_in_range(dc.value.offset)) {
        error = ros::console::formatToString(
            "duty_cycle offset %u is out of valid range [%u %u]",
            dc.value.offset, driver_t::value_t::MIN, driver_t::value_t::MAX);
        return false;
      }
      if (!driver_t::value_t::value_in_range(dc.value.value)) {
        error = ros::console::formatToString(
            "duty_cycle value %u is out of valid range [%u %u]",
            dc.value.value, driver_t::value_t::MIN, driver_t::value_t::MAX);
        return false;
      }
    }
    else {
      if (dc.percent.offset < 0.f || 100.f < dc.percent.offset) {
        error = ros::console::formatToString(
            "duty_cycle offset %f is out of valid range [0 100]", dc.percent.offset);
        return false;
      }
      if (dc.percent.value < 0.f || 100.f < dc.percent.value) {
        error = ros::console::formatToString(
            "duty_cycle value %f is out of valid range [0 100]", dc.percent.value);
        return false;
      }
    }
    return true;
  }

  bool setPinDutyCycle(uint16_t pin, pwm_msgs::DutyCycle const& dc, std::string& error)
  {
    if (dc.use_value) {
      if (dc.value.offset != 0) {
        ROS_WARN("Service: offset is not supported yet");
      }
      if (!driver_.set_duty_cycle(pin, driver_t::value_t(dc.value.value))) {
        error = ros::console::formatToString(
            "set_duty_cycle(%u, %u)", pin, dc.value.value);
        return false;
      }
      ROS_DEBUG("Service: set_duty_cycle(%u, %u)", pin, dc.value.value);
    }
    else {
      if (dc.percent.offset != 0.f) {
        ROS_WARN("Service: offset is not supported yet");
      }
      if (!driver_.set_duty_cycle(pin, dc.percent.value)) {
        error = ros::console::formatToString(
              "set_duty_cycle(%u, %f)", pin, dc.percent.value);
        return false;
      }
      ROS_DEBUG("Service: set_duty_cycle(%u, %f)", pin, dc.percent.value);
    }

    return true;
  }
};

} // namespace pca9685

int
main(int argc, char **argv)
{
  using pca9685::PCA9685Driver;
  using pca9685::Service;

  ros::init(argc, argv, "pca9685");

  std::string frame_id = "pca9685";
  int32_t i2c_bus = PCA9685Driver::DEFAULT_I2C_BUS;
  uint8_t i2c_address = PCA9685Driver::DEFAULT_I2C_ADDR;
  int32_t frequency = PCA9685Driver::DEFAULT_FREQUENCY;

  ros::NodeHandle nh;
  try {
    ros::NodeHandle nh_("~");
    nh_.param("frame_id", frame_id, frame_id);
    nh_.param("i2c_bus", i2c_bus, i2c_bus);

    std::string i2c_address_str = std::to_string(i2c_address);
    nh_.param("i2c_address", i2c_address_str, i2c_address_str);
    i2c_address = (uint8_t)std::stoul(i2c_address_str, nullptr, 0);

    nh_.param("frequency", frequency, frequency);

    ROS_INFO("Read params: {frame_id: %s, i2c_bus: %i, i2c_address: 0x%0x, frequency: %i}",
             frame_id.c_str(), i2c_bus, i2c_address, frequency);
  }
  catch (boost::bad_lexical_cast const& ex) {
    ROS_ERROR("Failed to read params: %s", ex.what());
    return 1;
  }

  if (ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
  }
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
  }

  Service service(frame_id, i2c_bus, i2c_address, frequency);
  auto server_sp = nh.advertiseService<Service>("pwm_single_pin", &Service::SetSinglePinDutyCycle, &service);
  auto server_mp = nh.advertiseService<Service>("pwm_multi_pin",  &Service::SetMultiPinDutyCycle,  &service);

  ros::spin();

  return 0;
}
