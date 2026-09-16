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

// TODO: extract into separate library

#pragma once

#include <limits>
#include <cstdint>

template <typename T>
constexpr T clamp(T value, T low, T high)
{
  return value < low ? low : (high < value ? high : value);
}

template <typename T, T RMIN = std::numeric_limits<T>::min(), T RMAX = std::numeric_limits<T>::max()>
struct limited_range
{
  enum : T { MIN = RMIN, MAX = RMAX };
  static_assert(MIN <= MAX, "Must be true: MIN <= MAX");

  limited_range(T value) : value(clamp(value, RMIN, RMAX)) {}
  limited_range(limited_range const& lr) : value(lr.value) {}

  limited_range& operator=(T value)
  {
    this->value = clamp(value, RMIN, RMAX);
    return *this;
  }

  limited_range& operator=(limited_range lr)
  {
    value = lr.value;
    return *this;
  }

  T operator*() const { return value; }
  operator T() const  { return value; }

  template <typename TV>
  static constexpr bool value_in_range(TV const& value)
  {
    return RMIN <= value && value <= RMAX;
  }

  T value;
};

template <typename TV, typename TR, TR RMIN, TR RMAX>
constexpr bool in_range(TV const& value, limited_range<TR, RMIN, RMAX> const&)
{
  return RMIN <= value && value <= RMAX;
}

template <typename T, T SMIN, T SMAX, T DMIN, T DMAX>
limited_range<T, DMIN, DMAX> range_cast(limited_range<T, SMIN, SMAX> src)
{
  return limited_range<T, DMIN, DMAX>(src.value);
}

template <int8_t MIN, int8_t MAX>
using int8_r = limited_range<int8_t, MIN, MAX>;

template <int16_t MIN, int16_t MAX>
using int16_r = limited_range<int16_t, MIN, MAX>;

template <int32_t MIN, int32_t MAX>
using int32_r = limited_range<int32_t, MIN, MAX>;

template <int64_t MIN, int64_t MAX>
using int64_r = limited_range<int64_t, MIN, MAX>;


template <uint8_t MIN, uint8_t MAX>
using uint8_r = limited_range<uint8_t, MIN, MAX>;

template <uint16_t MIN, uint16_t MAX>
using uint16_r = limited_range<uint16_t, MIN, MAX>;

template <uint32_t MIN, uint32_t MAX>
using uint32_r = limited_range<uint32_t, MIN, MAX>;

template <uint64_t MIN, uint64_t MAX>
using uint64_r = limited_range<uint64_t, MIN, MAX>;
