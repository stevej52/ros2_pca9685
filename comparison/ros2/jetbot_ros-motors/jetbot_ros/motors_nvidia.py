import rclpy

from jetbot_ros.motors import MotorController
from Adafruit_MotorHAT import Adafruit_MotorHAT


class MotorControllerNV(MotorController):
    """
    Motor controller node that supports the original NVIDIA JetBot.
    @see motors.py for the base class to implement different controllers.
    """
    MOTOR_LEFT = 1      # left motor ID
    MOTOR_RIGHT = 2     # right motor ID
    
    def __init__(self):
        super().__init__()
        
        # open Adafruit MotorHAT driver
        self.driver = Adafruit_MotorHAT(i2c_bus=1)
        
        # get motor objects from driver
        self.motors = {
            self.MOTOR_LEFT : self.driver.getMotor(self.MOTOR_LEFT),
            self.MOTOR_RIGHT : self.driver.getMotor(self.MOTOR_RIGHT)
        }
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        self._set_pwm(self.MOTOR_LEFT, left, self.left_trim)
        self._set_pwm(self.MOTOR_RIGHT, right, self.right_trim)
      
    def _set_pwm(self, motor, value, trim):
        # apply trim and convert [-1,1] to PWM value
        pwm = int(min(max((abs(value) + trim) * self.max_pwm, 0), self.max_pwm))
        self.motors[motor].setSpeed(pwm)
        
        # set the motor direction
        cmd = Adafruit_MotorHAT.RELEASE
        
        if value > 0:
            cmd = Adafruit_MotorHAT.FORWARD
        elif value < 0:
            cmd = Adafruit_MotorHAT.BACKWARD
            
        self.motors[motor].run(cmd)
 

def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControllerNV()
    node.get_logger().info("listening for velocity messages...")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
	

