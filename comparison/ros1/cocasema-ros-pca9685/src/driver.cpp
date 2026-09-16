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

#include <simple_io/i2c.h>

#include <ros/ros.h>

#include <algorithm>

namespace pca9685 {

enum Registers : uint8_t {
  MODE1         = 0x00, // R/W  Mode register 1
  MODE2         = 0x01, // R/W  Mode register 2

  SUBADR1       = 0x02, // R/W  I2C-bus subaddress 1
  SUBADR2       = 0x03, // R/W  I2C-bus subaddress 2
  SUBADR3       = 0x04, // R/W  I2C-bus subaddress 3

  ALLCALLADR    = 0x05, // R/W  LED All Call I2C-bus address

  LED0_ON_L     = 0x06, // R/W  LED0 output and brightness control byte 0
  LED0_ON_H     = 0x07, // R/W  LED0 output and brightness control byte 1
  LED0_OFF_L    = 0x08, // R/W  LED0 output and brightness control byte 2
  LED0_OFF_H    = 0x09, // R/W  LED0 output and brightness control byte 3

  LED15_ON_L    = 0x42, // R/W  LED15 output and brightness control byte 0
  LED15_ON_H    = 0x43, // R/W  LED15 output and brightness control byte 1
  LED15_OFF_L   = 0x44, // R/W  LED15 output and brightness control byte 2
  LED15_OFF_H   = 0x45, // R/W  LED15 output and brightness control byte 3

  ALL_LED_ON_L  = 0xFA, // write/read zero  load all the LEDn_ON registers, byte 0
  ALL_LED_ON_H  = 0xFB, // write/read zero  load all the LEDn_ON registers, byte 1
  ALL_LED_OFF_L = 0xFC, // write/read zero  load all the LEDn_OFF registers, byte 0
  ALL_LED_OFF_H = 0xFD, // write/read zero  load all the LEDn_OFF registers, byte 1

  // Writes to PRE_SCALE register are blocked when SLEEP bit is logic 0 (MODE 1)
  PRE_SCALE     = 0xFE, // R/W  prescaler for PWM output frequency

  // Reserved. Writes to this register may cause unpredictable results.
  TEST_MODE     = 0xFF, // R/W  defines the test mode to be entered
};

enum Mode1 : uint8_t {
  /* R Shows state of RESTART logic. See Section 7.3.1.1 for detail.
     W User writes logic 1 to this bit to clear it to logic 0. A user write of logic 0 will have no
       effect. See Section 7.3.1.1 for detail.
      *0 Restart disabled.
       1 Restart enabled.
  */
  RESTART = 1 << 7,

  /* R/W To use the EXTCLK pin, this bit must be set by the following sequence:
       1. Set the SLEEP bit in MODE1. This turns off the internal oscillator.
       2. Write logic 1s to both the SLEEP and EXTCLK bits in MODE1. The switch is
          now made. The external clock can be active during the switch because the
          SLEEP bit is set.
          This bit is a ‘sticky bit’, that is, it cannot be cleared by writing a logic 0 to it. The
          EXTCLK bit can only be cleared by a power cycle or software reset.
          EXTCLK range is DC to 50 MHz.
       *0 Use internal clock.
        1 Use EXTCLK pin clock.
  */
  EXTCLK = 1 << 6,

  /* R/W 0* Register Auto-Increment disabled.
         1  Register Auto-Increment enabled.
     When the Auto Increment flag is set, AI = 1, the Control register is automatically
     incremented after a read or write. This allows the user to program the registers sequentially.
  */
  AI = 1 << 5,

  /* R/W 0 Normal mode.
           It takes 500 us max. for the oscillator to be up and running
           once SLEEP bit has been set to logic 0. Timings on LEDn outputs are not
           guaranteed if PWM control registers are accessed within the 500 us window.
           There is no start-up delay required when using the EXTCLK pin as the PWM clock.
        *1 Low power mode. Oscillator off.
           No PWM control is possible when the oscillator is off.
           When the oscillator is off (Sleep mode) the LEDn outputs cannot be turned on, off or dimmed/blinked.
  */
  SLEEP = 1 << 4,

  /* R/W *0 PCA9685 does not respond to I2C-bus subaddress 1.
          1 PCA9685 responds to I2C-bus subaddress 1.
  */
  SUB1 = 1 << 3,

  /* R/W *0 PCA9685 does not respond to I2C-bus subaddress 2.
          1 PCA9685 responds to I2C-bus subaddress2.
  */
  SUB2 = 1 << 2,

  /* R/W *0 PCA9685 does not respond to I2C-bus subaddress 3.
          1 PCA9685 responds to I2C-bus subaddress 3.
  */
  SUB3 = 1 << 1,

  /* R/W  0 PCA9685 does not respond to LED All Call I2C-bus address.
         *1 PCA9685 responds to LED All Call I2C-bus address.
  */
  ALLCALL = 1 << 0,

  /* Default state */
  DEFAULT = AI | ALLCALL
};

/*
 * PCA9685Driver
 */
PCA9685Driver::PCA9685Driver(uint8_t i2c_bus, uint8_t i2c_address)
  : i2c_(new simple_io::I2C(i2c_bus, i2c_address))
{
  ROS_INFO("Driver: Created");

  if (!i2c_->init()) {
    ROS_ERROR("Driver: Failed to init i2c");
    return;
  }

  if (!i2c_->write(Registers::MODE1, (uint8_t)0x00/*Mode1::AI*/)) {
    ROS_ERROR("Driver: Failed to reset MODE1");
    return;
  }
}

PCA9685Driver::~PCA9685Driver()
{}

bool
PCA9685Driver::set_frequency(frequency_t value)
{
  auto const OSC_CLOCK_HZ = 25'000'000.;

  // The maximum PWM frequency is 1526 Hz if the PRE_SCALE register is set "0x03".
  // The minimum PWM frequency is 24 Hz if the PRE_SCALE register is set "0xFF".
  auto prescale = (uint8_t)(std::round(OSC_CLOCK_HZ / (4096. * value)) - 1);

  // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
  uint8_t mode1 = 0;
  if (!i2c_->read (Registers::MODE1,     &mode1)
   || !i2c_->write(Registers::MODE1,     (uint8_t)((mode1 & ~Mode1::RESTART) | Mode1::SLEEP))
   || !i2c_->write(Registers::PRE_SCALE, prescale)
   || !i2c_->write(Registers::MODE1,     mode1)) {
    ROS_ERROR("Driver: Failed to read/update MODE1/PRE_SCALE");
    return false;
  }

  ROS_DEBUG("Driver: Set prescale to %u (for %u Hz)", prescale, *value);

  // It takes 500 us max. for the oscillator to be up and running once SLEEP bit has been set to logic 0.
  // Timings on LEDn outputs are not guaranteed if PWM control registers are accessed within the 500 us window.
  usleep(500);

  if (!i2c_->write(Registers::MODE1, (uint8_t)(mode1 | Mode1::AI))){
    ROS_ERROR("Driver: Failed to update MODE1 with AI");
    return false;
  }

  return true;
}

bool
PCA9685Driver::set_duty_cycle(pin_t pin, float/*0..100*/ value)
{
  value = clamp(value, 0.f, 100.f) / 100.f;
  return set_duty_cycle(pin, value_t((uint16_t)(value * 4095.f)));
}

bool
PCA9685Driver::set_duty_cycle(pin_t pin, value_t value)
{
  /*
  The turn-on time of each LED driver output and the duty cycle of PWM can be controlled
  independently using the LEDn_ON and LEDn_OFF registers.
  There will be two 12-bit registers per LED output. These registers will be programmed by
  the user. Both registers will hold a value from 0 to 4095. One 12-bit register will hold a
  value for the ON time and the other 12-bit register will hold the value for the OFF time. The
  ON and OFF times are compared with the value of a 12-bit counter that will be running
  continuously from 0000h to 0FFFh (0 to 4095 decimal).
  Update on ACK requires all 4 PWM channel registers to be loaded before outputs will
  change on the last ACK.
  The ON time, which is programmable, will be the time the LED output will be asserted and
  the OFF time, which is also programmable, will be the time when the LED output will be
  negated. In this way, the phase shift becomes completely programmable. The resolution
  for the phase shift is 1/4096 of the target frequency. Table 7 lists these registers.
  */

  uint16_t const LED_FULL_ON  = 1 << 12;
  uint16_t const LED_FULL_OFF = 1 << 12;

  uint16_t on  = 0;
  uint16_t off = 0;

  if (value == 4095) {
    on = LED_FULL_ON;
  }
  else if (value == 0) {
    off = LED_FULL_OFF;
  }
  else {
    off = value;
  }

  uint8_t state[4] = {
      (uint8_t)(on),
      (uint8_t)(on >> 8),
      (uint8_t)(off),
      (uint8_t)(off >> 8)
  };

  ROS_DEBUG("Driver: Setting pin %u on=%u off=%u", *pin, on, off);

  return i2c_->write(Registers::LED0_ON_L + 4 * pin, state);
}

} // namespace pca9685
