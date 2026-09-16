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

#include "params.h"

#include <cstdint>
#include <memory>

namespace simple_io {
class I2C;
}

namespace pca9685 {

/*
 * PCA9685Driver
 */
class PCA9685Driver
{
public:
  enum : uint8_t  { DEFAULT_I2C_BUS   = 1 };
  enum : uint8_t  { DEFAULT_I2C_ADDR  = 0x40 };
  enum : uint16_t { DEFAULT_FREQUENCY = 200 };

public:
  PCA9685Driver(uint8_t i2c_bus     = DEFAULT_I2C_BUS,
                uint8_t i2c_address = DEFAULT_I2C_ADDR);
  ~PCA9685Driver();

  PCA9685Driver (PCA9685Driver const&) = delete;
  void operator=(PCA9685Driver const&) = delete;

  typedef uint16_r<24, 1526> frequency_t;
  typedef uint8_r<0, 15> pin_t;
  typedef uint16_r<0, 4095> value_t;

  bool set_frequency(frequency_t);

  bool set_duty_cycle(pin_t pin, float/*0..100*/ value);
  bool set_duty_cycle(pin_t pin, value_t value);

private:
  std::unique_ptr<simple_io::I2C> i2c_;
};

} // namespace pca9685
