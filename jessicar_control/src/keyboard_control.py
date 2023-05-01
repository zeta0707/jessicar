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
from myutil import clamp, PCA9685, PWMThrottle

global speed_pulse
global steering_pulse

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