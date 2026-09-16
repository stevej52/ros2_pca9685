'''
To fix in c++ code:
- normalize input to -1 to 1 range
   - deadband
   - reduce top speed
- add arming sequence for ESC 
- add watchdog timer to stop motor if no commands received for a certain time


pip install adafruit-blinka adafruit-circuitpython-pca9685
pip install --upgrade Jetson.GPIO adafruit-blinka

Hook up:
nano / pca9685
 1 - (VCC comes from ESC via BEC)
 2 - V+ (for 9685 board)
 3 - SDA
 4 - open
 5 - SCL
 6 - GND

        # Deadbands - wheels just barely move when off the ground
        # forward: 0.271
        # backward: -0.0405

Control an ESC drive motor using PCA9685 on Jetson Orin Nano
- ESC is what came with the AMORIL 1/10 RTR Brushless Fast RC Car
    Arming: t.b.d., but likely set to neutral for 1 second, then forward for 1 second, then neutral again
    Switch to reverse: do arming sequence
    Switch to forward: do arming sequence
- A previous ESC is a Hyric brushless motor controller - Brush-X60-RTR
    Arming: set to neutral for 2 seconds
    Switch to reverse: set to neutral, then min pulse, then neutral again
    Switch to forward: no change needed, just set to neutral
- pwm 0 is for steering servo
- pwm 1 is for drive motor (ESC)

Control range is from -1 to 0 to 1
- Steering - -1 is full left, 0 is center, 1 is full right
- Drive - -1 is full reverse, 0 is stop, 1 is full forward

Uses adafruit_pca9685 library for PWM control

Could get fancy and use a class for each channel, but this is simpler for now.
'''

import board
import busio
import adafruit_pca9685
import time



class DriveMotorController: # PCA9685 based controller for ESC drive moto
    def __init__(self):
        self.ESCinitialized = False   # channel 1

        i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C 1 ok!")

        self.pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
        self.pca.frequency = 50

        # Capture servo and drive motor channels
        # self.servo_channel = self.pca.channels[0]
        self.drive_channel = self.pca.channels[1]

        # PWM values (adjust these based on your ESC)
        # Same for the servo as well
        self.min_pulse = 3277    # ~1ms (full reverse or minimum)
        self.neutral_pulse = 4915 # ~1.5ms (neutral/stop)
        self.max_pulse = 6553    # ~2ms (full forward or maximum)

        # Initialize ESC
        print("Initializing ESC...")
        # Deadbands - wheels just barely move when off the ground
        # forward: 0.271
        # backward: -0.0405

        self.deadbandForward = 0.271  # Minimum power to move
        self.deadbandReverse = -0.0405 # Minimum power to move in reverse
        self.max_speed = 0.4  # Limit top speed to 40% - above deadband
        self.current_direction = 0 # 1 for Fwd, -1 for Rev, 0 for Neutral

        self.driveInitialize()

        # self.set_DriveDirection(1)   # go forward first
        print("ESC initialized!")

        

    # raw - set duty cycle directly - range from -1.0 to 0 to 1.0
    def set_MotorSpeed(self, channel, speed):
        speed = max(-1.0, min(1.0, speed))      # clamp
        if abs(speed) < 0.01:
            print("Within deadband - stopping motor")
            duty_cycle = self.neutral_pulse     # Neutral position
        elif speed > 0:
            # Forward direction
            duty_cycle = int(self.neutral_pulse + speed * (self.max_pulse - self.neutral_pulse))
        else:
            # Reverse direction (if ESC supports it - ours does)
            duty_cycle = int(self.neutral_pulse + speed * (self.neutral_pulse - self.min_pulse))
        
        channel.duty_cycle = duty_cycle         # send to actual motor
        print(f"Motor speed: {speed*100:.1f}% (duty cycle: {duty_cycle})")

    def done(self):
        # Turn off PWM signal to ESC
        self.drive_channel.duty_cycle = 0
        self.pca.deinit()
        print("Steering and Drive Motor Controller complete!")
 
    def driveInitialize(self):
        # per Gemini
        initTime = 0.5
        self.set_MotorSpeed(self.drive_channel, 0.0)
        time.sleep(initTime)
        self.set_MotorSpeed(self.drive_channel, 0.1)
        time.sleep(initTime)
        self.set_MotorSpeed(self.drive_channel, 0.0)
        time.sleep(initTime)
        self.current_direction = 1
        print("Initialize drive direction complete")

    def set_DriveDirection(self, desired_direction):
        if desired_direction == self.current_direction:
            print("Already in desired direction, no change needed" + str(desired_direction))
            return  # No change needed

        print("Switching drive direction to " + str(desired_direction))
        kQkSleep = 0.2
        # figured this out experimentally...
        self.set_MotorSpeed(self.drive_channel, 0.0)
        time.sleep(kQkSleep)
        if desired_direction == -1:
            print("Reverse requires extra tap")
            self.set_MotorSpeed(self.drive_channel, 0.1)
            time.sleep(kQkSleep)
            self.set_MotorSpeed(self.drive_channel, 0.0)
            time.sleep(kQkSleep)
        self.current_direction = desired_direction
        print("Reset drive direction complete")


    def set_Drivespeed(self, input_speed):
        if abs(input_speed) < 0.01:
            self.set_MotorSpeed(self.drive_channel, 0.0)
            return

        desired_direction = 1 if input_speed >= 0 else -1
        self.set_DriveDirection(desired_direction)

        # 2. Scale and Apply Deadband
        # Formula: output = deadband + (input * (max_limit))
        if input_speed > 0:
            final_speed = self.deadbandForward + (input_speed * self.max_speed)
        else:
            # input_speed is negative here
            final_speed = self.deadbandReverse + (input_speed * self.max_speed)


        # speed = max(-1.0, min(1.0, speed))      # clamp
        # speed = speed * self.max_speed        # Scale speed to max limit
        # if speed > 0:
        #     speed = speed + self.deadbandForward  # Add deadband to forward speeds
        # else:
        #     speed = speed + self.deadbandReverse  # Add deadband to reverse speeds
        print("Fixed speed after scaling/deadband: " + str(final_speed))

        self.set_MotorSpeed(self.drive_channel, final_speed)  # Ensure ESC is stopped before changing steering

    def reset(self):
        self.set_MotorSpeed(self.drive_channel, 0)
        self.current_direction = 0



def motor_test():
    """Test motor movement with ESC"""
    print("Testing ESC motor control...")
    
    motors = DriveMotorController()

    
    try:
        # Deadbands - wheels just barely move when off the ground
        # forward: 0.271
        # backward: -0.0405
        for speed in [-0.1, 0.06, -0.1, 0.06]:        # -0.1, 
            print(f"Testing movement...   {speed}")
            # motors.set_MotorSpeed(motors.drive_channel, speed)
            motors.set_Drivespeed(speed)
            time.sleep(3)
            # motors.set_MotorSpeed(motors.drive_channel, 0.0)
            motors.set_Drivespeed(0.0)
            time.sleep(0.5)

        print("Final stop...")
        motors.reset()
        
        print("ESC motor test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        motors.reset()
    except Exception as e:
        print(f"Motor test failed: {e}")
        motors.reset()
    finally:
        motors.done()


if __name__ == "__main__":
    motor_test()
