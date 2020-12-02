# ros2_pca9685
ROS2 Package to convert /cmd-vel to PWM signals using a PCA9685

cd dev_ws

colcon build --packages-select ros2_pca9685

source ./install/setup.bash

ros2 run ros2_pca9685 listener


-----Changes needed to subscriber_member_function.py------------------

pca.frequency = 100 # I keep reading that 50Hz is good for servos even thought I thought it was 75Hz, but after lots of testing I'm using 100Hz mainly for proper ESC zero

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


also change servo numbers in move_robot() function

