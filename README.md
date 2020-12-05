# ros2_pca9685
ROS2 Package to convert /cmd_vel topic to PWM signals using a PCA9685

With only this package and the ROS2 teleop_twist_joy package you can control ESC's and servos with a joystick or gamepad using ROS2.

Nav2 movement is also controlled through /cmd_vel so Nav2 works as well.

Tested on ROS2 Dashing and ROS2 Foxy

All code was adapted from these two tutorials.

https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/

https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/

Need to install adafruit_pca9685 and adafruit_servokit libraries.

sudo pip3 install adafruit-pca9685

sudo pip3 install adafruit-circuitpython-servokit


-----Changes needed to subscriber_member_function.py------------------

pca.frequency = 100 # I keep reading that 50Hz is good for servos even though I thought it was 75Hz, but after lots of testing I'm using 100Hz mainly for proper ESC zero

maxr=135 # Max Right servo travel

minl=30 # Max Left servo travel

maxthr=125 # Max Throttle PWM

minthr= 65 # Min Throttle PWM

thrinit = 90 # Throttle Zero

strinit = 85 # Steering Zero

print("Initializing Propulsion System") #

kit.servo[0].angle = thrinit # this Throttle Servo number will need to be set

time.sleep(1) # wait 1 second for the ESC to initialize zero
 
print("Initializing Steering System")

kit.servo[1].angle = strinit #this Steering Servo number will need to be set

bs1=180-thrinit # reverse the steering number for 4-Wheel steering

kit.servo[2].angle = bs1 #this is the back servo for 4-Wheel Steering Servo- Runs reverse direction - number will need to be set


Also change servo numbers in move_robot() function

Build after all changes made:


cd dev_ws

colcon build --packages-select ros2_pca9685

source ./install/setup.bash

ros2 run ros2_pca9685 listener


