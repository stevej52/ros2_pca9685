// MIT License

// Copyright (c) 2024 Takumi Asada
// Copyright (c) 2019 Pushkal Katara

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// https://github.com/jetsonhacks/JHPWMDriver
// https://github.com/DiamondSheep/Servo_driver

/*------------------------------------------------------------------------------*/
/* Define MACRO */
/*------------------------------------------------------------------------------*/
#ifndef PCA9685_COMPONENT_HPP_
#define PCA9685_COMPONENT_HPP_

/*------------------------------------------------------------------------------*/
/* Include */
/*------------------------------------------------------------------------------*/
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pca9685_ros2/JHPWMPCA9685.h>

/*------------------------------------------------------------------------------*/
/* Class */
/*------------------------------------------------------------------------------*/
class Controller : public rclcpp::Node {
public:
	Controller();
	~Controller();

    // Calibrated for SG-90 servo (top of the pan-tilt) and MG-90S servo (bottom of the pan-tilt)
    int servoMin = 110; // 150 for the SG-90 on the top of the pan-tilt
    int servoMax = 490; // 336 for the SG-90 on the top of the pan-tilt
    int servoMiddle = 300;
    int bus = 1; // bus 1 (pin 3 and pin 5)
    int frequency = 50; // default 50 Hz
    PCA9685 *controller;
	PCA9685 *controller_2;
	float error = 5;

    void init(PCA9685 *controller);
    void angle(PCA9685 *controller, int channel, float angle, float angle_error);
    void dcdriver(PCA9685 *controller, int in_1, int in_2, int speed, bool direction);

private:
    // map degrees to the servo value
    int map(int degree);
};

#endif /* PCA9685_COMPONENTR_HPP_ */
