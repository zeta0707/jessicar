#!/usr/bin/env python

"""
Node for control PCA9685 using AckermannDriveStamped msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""
import time
import rospy
from threading import Thread
from ackermann_msgs.msg import AckermannDriveStamped
from myutil import clamp, PCA9685, PWMThrottle, PWMThrottle2Wheel, PWMSteering

class Vehicle(object):
    def __init__(self, name="Jessicar"):

        if hasSteer == 1:
            if isDCSteer == 1:
                steer_controller = PCA9685(channel=0, address=i2caddr0, busnum=1)
                self._steering = PWMSteering(controller=steering_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
                rospy.loginfo("Steering Controller Awaked!!")
            else:
                self._steering = PCA9685(channel=0, address=i2caddr0, busnum=1)
                rospy.loginfo("Steering Controller Awaked!!") 

            throttle_controller = PCA9685(channel=0, address=i2caddr1, busnum=1)
            self._throttle = PWMThrottle(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            rospy.loginfo("Throttle Controller Awaked!!") 
        else:
            throttle_controller = PCA9685(channel=0, address=i2caddr0, busnum=1)
            self._throttle = PWMThrottle2Wheel(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)

        self._name = name
        self._teleop_sub = rospy.Subscriber(
            "/jessicar_teleop",
            AckermannDriveStamped,
            self.joy_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.loginfo("Teleop Subscriber Awaked!! Waiting for joystick...")

    def joy_callback(self, msg):
        speed_pulse = msg.drive.speed
        steering_pulse = msg.drive.steering_angle

        if hasSteer == 1:
            print(
                "speed_pulse : "
                + str(speed_pulse)
                + " / "
                + "steering_pulse : "
                + str(steering_pulse)
            )
        else:
             print(
                "speed_pulse : "
                + str(speed_pulse)
                + " / "
                + "steer % : "
                + str(steering_pulse*100/STEER_LIMIT)
            )

        if hasSteer == 1:
            self._throttle.run(speed_pulse)
            self._steering.run(steering_pulse)
        else:
            self._throttle.run(speed_pulse,steering_pulse)


if __name__ == "__main__":

    rospy.init_node("jessicar_control")

    STEER_CENTER = rospy.get_param("/steer_center") 
    STEER_LIMIT = rospy.get_param("/steer_limit")
    STEER_DIR = rospy.get_param("/steer_dir")
    SPEED_CENTER = rospy.get_param("/speed_center") 
    SPEED_LIMIT = rospy.get_param("/speed_limit")   
    hasSteer = rospy.get_param("/has_steer")  
    isDCSteer = rospy.get_param("/isDCSteer") 

    i2caddr0 = rospy.get_param("/i2caddr0") 
    i2caddr1 = rospy.get_param("/i2caddr1") 

    myCar = Vehicle("Jessicar")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()