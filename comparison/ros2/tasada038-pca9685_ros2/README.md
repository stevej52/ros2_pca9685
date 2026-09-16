# pca9685_ros2

A ROS 2 Node for the Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface - PCA9685.
http://www.adafruit.com/product/815

## Supported ROS 2 distributions

[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

## Environment
Built for the NVIDIA Jetson Nano Development Kit.

Members of the Jetson family have different I2C Bus layouts. Check the pinout diagrams to determine which Bus is being used for your particular model. Pinouts are available here: https://www.jetsonhacks.com/pinouts/

## Usage

The default address for the PCA9685 are 0x40 and 0x41. On some Jetson models, this address is being used by other hardware. You may have to switch to another I2C bus.

In order to be able inspect the PCA9685, you may find it useful to install the i2c tools:

```sh
sudo apt-get install libi2c-dev i2c-tools
```

As an example, after installation, in a Terminal execute:

```sh
sudo i2cdetect -y -r 1
```

Replace the bus number 1 with the appropriate bus number.

You should see an entry of 0x40 and 0x41, which is the default address of the PCA9685. If you have soldered the address pins on the PCA9685 (as is the case if you are using multiple boards chained together) you should see the appropriate address.

You must have root permission to use the I2C bus, ie you must use sudo. To run the example:

```sh
colcon build --packages-select pca9685_ros2
. install/setup.bash
ros2 run pca9685_ros2 controller
```

## License
This repository is licensed under the MIT license, see LICENSE.

[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html

[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/

