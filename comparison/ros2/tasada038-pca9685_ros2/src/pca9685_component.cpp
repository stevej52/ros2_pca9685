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

/*------------------------------------------------------------------------------*/
/* Include */
/*------------------------------------------------------------------------------*/
#include "pca9685_ros2/pca9685_component.hpp"
#include <time.h>

/*------------------------------------------------------------------------------*/
/* Constructor */
/*------------------------------------------------------------------------------*/
Controller::Controller()
: Node("controller_node")
{
	RCLCPP_INFO(this->get_logger(), "Initialize");

	controller = new PCA9685(0x40, bus);
	controller_2 = new PCA9685(0x41, bus);
	controller->openPCA9685();
	controller_2->openPCA9685();
  printf("PCA9685 Device Address: 0x%02X\n",controller->kI2CAddress);
  this->init(controller);
	this->init(controller_2);
  controller->setPWMFrequency(frequency);
	controller_2->setPWMFrequency(frequency);
	sleep(1);

  /* DC Driver Sample */
	// this->dcdriver(controller, 4, 5, 4095, true);
  //   sleep(10);

	// this->dcdriver(controller, 4, 5, 0, true);
  //   sleep(2);

	// this->dcdriver(controller, 4, 5, 4095, false);
  //   sleep(10);

	// this->dcdriver(controller, 4, 5, 0, false);
  //   sleep(2);

  /* Servo Motor PWM Sample*/
	this->angle(controller, 0, 30, error);
	this->angle(controller_2, 0, 45, error);
	sleep(1);
	this->angle(controller, 0, 0, error);
	this->angle(controller_2, 0, 0, error);
	sleep(1);
	this->angle(controller, 0, -45, error);
	this->angle(controller_2, 0, -45, error);
	sleep(1);
	this->angle(controller, 0, 0, error);
	this->angle(controller_2, 0, 0, error);
	sleep(1);
}

/*------------------------------------------------------------------------------*/
/* Destructor */
/*------------------------------------------------------------------------------*/
Controller::~Controller() {
	RCLCPP_INFO(this->get_logger(), "closePWM");
	this->angle(controller, 0, 0, error);
	controller->closePCA9685();
}

void Controller::init(PCA9685 *controller){
    controller->setAllPWM(0, 0);
    controller->reset();
	RCLCPP_INFO(this->get_logger(), "setAllPWM and reset DONE");
}

void Controller::angle(PCA9685 *controller, int channel, float angle, float angle_error){
    int value = this->map((angle + angle_error));
    controller->setPWM(channel, 0, value);
}

/*------------------------------------------------------------------------------*/
/**
 * @fn dcdriver
 * @brief DC Driver using PWM
 * @param in_1: Input Pin 1
 * @param in_2: Input Pin 2
 * @param speed: DC Driver Speed from 0 to 4095
 * @param direction: true is CW, false is CCW.
 */
/*------------------------------------------------------------------------------*/
void Controller::dcdriver(PCA9685 *controller, int in_1, int in_2, int speed, bool direction){
	if(direction) { //CW
		RCLCPP_INFO(this->get_logger(), "CW");
		controller->setPWM(in_1, 0, speed);
		controller->setPWM(in_2, 0, 0);
	}
	else { //CCW
		RCLCPP_INFO(this->get_logger(), "CCW");
		controller->setPWM(in_1, 0, 0);
		controller->setPWM(in_2, 0, speed);
	}
}

// private
int Controller::map (int degree) {
    if (degree >= -90 && degree <= 90)
        return ((degree+90) * (servoMax-servoMin)/180 + servoMin);
    else {
	printf("Out of angle range.\n");
	return servoMiddle;
    }
}
