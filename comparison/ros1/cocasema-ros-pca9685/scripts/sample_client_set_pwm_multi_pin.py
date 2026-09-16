#!/usr/bin/env python

import sys
import rospy
from pwm_msgs.msg import *
from pwm_msgs.srv import *
from time import sleep


def pwm_multi_pin():
    service = rospy.ServiceProxy('pwm_multi_pin', SetMultiPinDutyCycle)
    for j in xrange(100, 500, 10):
        req = SetMultiPinDutyCycleRequest()
        for i in xrange(0, 16):
            pin = SinglePinDutyCycle()
            pin.pin = i
            pin.dc.use_value = True
            pin.dc.value.value = j
            req.requested.pins.append(pin)
        sleep(0.005)
        rsp = service(req)


def main():
    pwm_multi_pin()

if __name__ == "__main__":
    main()
