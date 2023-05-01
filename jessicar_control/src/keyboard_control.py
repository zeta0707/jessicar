#!/usr/bin/env python

"""
Node for control PCA9685 using teleop_twist_keyboard msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

import time
import rospy
from threading import Thread
from geometry_msgs.msg import Twist
from myutil import clamp, PCA9685

global speed_pulse
global steering_pulse

global SPEED_CENTER
global SPEED_CENTER
global STEER_CENTER
global STEER_LIMIT
global STEER_DIR

class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    def __init__(self, controller=None,
                       max_pulse=4095,
                       min_pulse=-4095,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

    def run(self, throttle):
        pulse = int(throttle)
        if throttle > 0:
            self.controller.pwm.set_pwm(self.controller.channel,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+1,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+2,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+3,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+4,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+7,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+6,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+5,0,4095)
        else:
            self.controller.pwm.set_pwm(self.controller.channel,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+2,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+1,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+3,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+4,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+7,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+5,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+6,0,4095)

    def shutdown(self):
        self.run(0) #stop vehicle

class Vehicle(object):
    def __init__(self, name="Jessicar"):
        
        self._steering_servo = PCA9685(channel=0, address=0x40, busnum=1)
        rospy.loginfo("Steering Controller Awaked!!")

        throttle_controller = PCA9685(channel=0, address=0x60, busnum=1)
        self._throttle = PWMThrottle(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
        rospy.loginfo("Throttle Controller Awaked!!") 
        
        self._name = name
        self._teleop_sub = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.keyboard_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.loginfo("Keyboard Subscriber Awaked!! Waiting for keyboard...")

    def keyboard_callback(self, msg):

        #rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Components: [%0.2f, %0.2f]"%(msg.linear.x, msg.angular.z))

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        speed_pulse = SPEED_CENTER + msg.linear.x*SPEED_LIMIT 
        speed_pulse = clamp(speed_pulse, -SPEED_LIMIT, SPEED_LIMIT)

        steering_pulse = STEER_CENTER + msg.angular.z*STEER_LIMIT*STEER_DIR
        steering_pulse = clamp(steering_pulse, STEER_CENTER - STEER_LIMIT, STEER_CENTER + STEER_LIMIT)

        print(
            "speed_pulse : "
            + str(speed_pulse)
            + " / "
            + "steering_pulse : "
            + str(steering_pulse)
        )

        self._throttle.run(speed_pulse)
        self._steering_servo.run(steering_pulse)


if __name__ == "__main__":

    rospy.init_node("jessicar_control", anonymous=True)

    STEER_CENTER = rospy.get_param("/steer_center") 
    STEER_LIMIT = rospy.get_param("/steer_limit")
    STEER_DIR = rospy.get_param("/steer_dir")
    SPEED_CENTER = rospy.get_param("/speed_center") 
    SPEED_LIMIT = rospy.get_param("/speed_limit")   

    myCar = Vehicle("Jessicar")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()