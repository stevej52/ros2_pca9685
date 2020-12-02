# ros2_pca9685
ROS2 Package to convert /cmd-vel to PWM signals using a PCA9685


cd dev_ws

colcon build --packages-select ros2_pca9685

source ./install/setup.bash

ros2 run ros2_pca9685 listener
