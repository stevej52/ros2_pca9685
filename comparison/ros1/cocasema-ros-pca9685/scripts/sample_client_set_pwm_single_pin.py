#!/usr/bin/env python

import sys
import rospy
from pwm_msgs.msg import *
from pwm_msgs.srv import *
from time import sleep


def pwm_single_pin():
    service = rospy.ServiceProxy('pwm_single_pin', SetSinglePinDutyCycle)
    for i in xrange(0, 16):
        for j in xrange(0, 100):
            req = SetSinglePinDutyCycleRequest()
            req.requested.pin = i
            req.requested.dc.percent.value = 5 * i
            rsp = service(req)
        req = SetSinglePinDutyCycleRequest()
        req.requested.pin = i
        req.requested.dc.percent.value = 15
        rsp = service(req)


def main():
    pwm_single_pin()

if __name__ == "__main__":
    main()
