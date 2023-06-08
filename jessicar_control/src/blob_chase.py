#!/usr/bin/env python

"""
referenced from those projects

DkLowLevelCtrl, ServoConvert part from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

PCA9685 part from donkeycar
url: https://github.com/autorope/donkeycar/blob/99c853b1737f12019ae598d3c7f00699d2166472/donkeycar/parts/actuator.py#L12

Listens to /dkcar/control/cmd_vel for corrective actions to the /cmd_vel coming from keyboard or joystick

"""
import rospy
from geometry_msgs.msg import Twist
import time
from myutil import clamp, PCA9685, PWMThrottle, PWMThrottle2Wheel, PWMThrottleHat, PWMSteering

class ServoConvert:
    def __init__(self, id=1, center_value=0, range=8192, direction=1):
        self.value = 0.0
        self.value_out = center_value
        self._center = center_value
        self._range = range
        self._half_range = 0.5 * range # 45
        self._dir = direction # 1 or -1 
        self.id = id

        # --- Convert its range in [-1, 1]
        self._sf = 1.0 / self._half_range # 1 / 45

    def get_value_out(self, value_in):
        # --- twist type value is in  [-1, 1]
        self.value = value_in
        self.value_out = int(self._dir * value_in * self._half_range + self._center)
        return self.value_out


def saturate(value, min, max):
    if value <= min:
        return min
    elif value >= max:
        return max
    else:
        return value


class DkLowLevelCtrl:
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        # --- Initialize the node
        rospy.init_node("blob_chase_node")

        self.hasSteer = rospy.get_param("/has_steer") 
        STEER_CENTER = rospy.get_param("/steer_center") 
        STEER_LIMIT = rospy.get_param("/steer_limit")
        STEER_DIR = rospy.get_param("/steer_dir")
        SPEED_CENTER = rospy.get_param("/speed_center") 
        SPEED_LIMIT = rospy.get_param("/speed_limit")   
        i2caddr0 = rospy.get_param("/i2caddr0") 
        i2caddr1 = rospy.get_param("/i2caddr1") 
        self.isDCSteer = rospy.get_param("/isDCSteer") 

        #RCcar which has steering
        if self.hasSteer == 1:
            #Steer with DC motor driver 
            if self.isDCSteer == 1:
                steer_controller = PCA9685(channel=0, address=i2caddr0, busnum=1)
                self._steering = PWMSteering(controller=steer_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            #Steer with servo motor
            else:
                self._steering = PCA9685(channel=0, address=i2caddr0, busnum=1)
            rospy.loginfo("Steering Controller Awaked!!") 
           
            throttle_controller = PCA9685(channel=0, address=i2caddr1, busnum=1)
            if self.isDCSteer == 1:  
                 #Throttle with Motorhat             
                self._throttle = PWMThrottleHat(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)            
            else:
                #Throttle with Jetracer
                self._throttle = PWMThrottle(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            rospy.loginfo("Throttle Controller Awaked!!") 
            
        #2wheel RCcar
        else:
            throttle_controller = PCA9685(channel=0, address=i2caddr0, busnum=1)
            self._throttle = PWMThrottle2Wheel(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            rospy.loginfo("2wheel Throttle Controller Awaked!!")           

        self.actuators = {}
        self.actuators["throttle"] = ServoConvert(id=1, center_value=SPEED_CENTER, range=SPEED_LIMIT*2, direction=1)
        self.actuators["steering"] = ServoConvert(id=2, center_value=STEER_CENTER, range=STEER_LIMIT*2, direction=1)  # -- positive left
        rospy.loginfo("> Actuators corrrectly initialized")

        # --- Create a debug publisher for resulting cmd_vel
        #self.ros_pub_debug_command = rospy.Publisher(
        #    "/dkcar/debug/cmd_vel", Twist, queue_size=1
        #)
        #rospy.loginfo("> Publisher corrrectly initialized")

        # --- Create the Subscriber to Twist commands
        #self.ros_sub_twist = rospy.Subscriber(
        #    "/cmd_vel", Twist, self.update_message_from_command
        #)
        #rospy.loginfo("> Subscriber corrrectly initialized")

        # --- Create the Subscriber to obstacle_avoidance commands
        self.ros_sub_twist = rospy.Subscriber(
            "/dkcar/control/cmd_vel", Twist, self.update_message_from_chase
        )
        rospy.loginfo("> Subscriber corrrectly initialized")

        self.throttle_cmd = 0.0
        self.throttle_chase = 0.0
        self.steer_cmd = 0.0
        self.steer_chase = 0.0

        self._debud_command_msg = Twist()

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._last_time_chase_rcv = time.time()
        self._timeout_ctrl = 100
        self._timeout_blob = 1

        rospy.loginfo("Initialization complete")

    def update_message_from_command(self, message):
        self._last_time_cmd_rcv = time.time()
        self.throttle_cmd = message.linear.x
        self.steer_cmd = message.angular.z

    def update_message_from_chase(self, message):
        self._last_time_chase_rcv = time.time()
        self.throttle_chase = message.linear.x
        self.steer_chase = message.angular.z
        print(self.throttle_chase, self.steer_chase)

    def compose_command_velocity(self):
        self.throttle = saturate(self.throttle_cmd + self.throttle_chase, -1, 1)
        # -- Add steering
        self.steer = saturate(self.steer_cmd + self.steer_chase, -1, 1)

        #self._debud_command_msg.linear.x = self.throttle
        #self._debud_command_msg.angular.z = self.steer
        #self.ros_pub_debug_command.publish(self._debud_command_msg)

        self.set_actuators_from_cmdvel(self.throttle, self.steer)

    def set_actuators_from_cmdvel(self, throttle, steering):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        # -- Convert vel into servo values
        self.actuators["throttle"].get_value_out(throttle)
        self.actuators["steering"].get_value_out(steering)
        # rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(throttle, steering))

        self.set_pwm_pulse(self.actuators["throttle"].value_out, self.actuators["steering"].value_out)
        print( "throttle: " + str(self.actuators["throttle"].value_out) +  ", steering: " + str(self.actuators["steering"].value_out))


    def set_pwm_pulse(self, speed_pulse, steering_pulse):
        if self.hasSteer == 1:
            self._throttle.run(speed_pulse)
            self._steering.run(steering_pulse)
        else:
            self._throttle.run(speed_pulse, steering_pulse)

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.throttle_cmd = 0.0
        self.steer_cmd = 0.0

    def reset_avoid(self):
        self.throttle_chase = 0.0
        self.steer_avoid = 0.0
        
    @property
    def is_controller_connected(self):
        # print time.time() - self._last_time_cmd_rcv
        return time.time() - self._last_time_cmd_rcv < self._timeout_ctrl

    @property
    def is_chase_connected(self):
        return time.time() - self._last_time_chase_rcv < self._timeout_blob

    def run(self):

        # --- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.compose_command_velocity()

            if not self.is_controller_connected:
                self.set_actuators_idle()

            if not self.is_chase_connected:
                self.reset_avoid()

            rate.sleep()


if __name__ == "__main__":
    dk_llc = DkLowLevelCtrl()
    dk_llc.run()
