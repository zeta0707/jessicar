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
from myutil import clamp, PCA9685, PWMThrottleRacer, PWMThrottle2Wheel, PWMThrottleHat, PWMSteering

class Vehicle(object):
    def __init__(self, name="Jessicar"):

        #RCcar which has steering
        if hasSteer == 1:
            #Steer with DC motor driver 
            if isDCSteer == 1:
                steer_controller = PCA9685(channel=0, address=i2cSteer, busnum=1)
                self._steering = PWMSteering(controller=steer_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            #Steer with servo motor
            else:
                self._steering = PCA9685(channel=0, address=i2cSteer, busnum=1)
            rospy.loginfo("Steering Controller Awaked!!") 
             
            throttle_controller = PCA9685(channel=0, address=i2cThrottle, busnum=1)
            if i2cThrottle == 0x60:
                #Throttle with Jetracer
                self._throttle = PWMThrottleRacer(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095) 
            else:
                #Throttle with Motorhat B
                self._throttle = PWMThrottleHat(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            rospy.loginfo("Throttle Controller Awaked!!") 
            
        #2wheel RCcar, i2cSteer=i2cThrottle
        else:
            throttle_controller = PCA9685(channel=0, address=i2cThrottle, busnum=1)
            self._throttle = PWMThrottle2Wheel(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            rospy.loginfo("2wheel Throttle Controller Awaked!!") 

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

    i2cSteer = rospy.get_param("/i2cSteer") 
    i2cThrottle = rospy.get_param("/i2cThrottle") 

    myCar = Vehicle("Jessicar")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
