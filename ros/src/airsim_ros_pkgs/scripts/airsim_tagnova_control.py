#!/usr/bin/env python

import datetime
import threading
import time
import rospy
import airsim_ros_pkgs as air
import airsim_ros_pkgs.msg
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class TagnovaControl:
    def __init__(self):
        # Get parameters from ROS parameter server
        update_rate_hz = rospy.get_param('~update_rate_hz', 20.0)
        max_curvature = rospy.get_param('~max_curvature', 0.75)
        steer_sign = rospy.get_param('~steer_sign', -1)
        throttle_brake_sign = rospy.get_param('~throttle_brake_sign', 1)
        car_control_topic = rospy.get_param('~car_control_topic', '/airsim_node/drone_1/car_cmd')

        # Initialize class attributes
        # Create a lock object to synchronize access to shared resources
        self.lock = threading.Lock()
        # Set the last message time to the current time
        self.last_msg_time = datetime.datetime.now()
        self.magnetig_time = datetime.datetime.now()
        self.magnetig_state = False
        self.angular = 0

        self.joy_msg = None  # Initialize a variable to store Autoware commands received

        # Subscribe to a ROS topics to receive Autoware commands
        self.joy_sub = rospy.Subscriber('out_twist_cmd', TwistStamped, self.handle_joy)
        self.twist_raw_sub = rospy.Subscriber('twist_raw', TwistStamped, self.take_time)       

        # Create a ROS publisher to send car control commands
        self.command_pub = rospy.Publisher(car_control_topic, air.msg.CarControls, queue_size=0)        

        # Create a ROS timer to periodically execute a callback function
        self.update_time = rospy.Timer(rospy.Duration(1.0/update_rate_hz), self.handle_update_timer)

        print("AIRSIM CONTROLL START")
    

    def take_time(self, msg):
        self.angular = msg.twist.angular.z
        self.last_msg_time = datetime.datetime.now()

    def handle_joy(self, msg):
        with self.lock:
            self.joy_msg = msg

    def handle_update_timer(self, ignored):
        joy = None
        with self.lock:
            joy = self.joy_msg

        if joy is None:
            return

        controls = airsim_ros_pkgs.msg.CarControls()
        # if self.angular >= 1:
        #     controls.steering = -2 * self.angular
        # else:
        controls.steering = -1.5 * self.angular    
        u = joy.twist.linear.x
        if u > 0.0 and u < 0.27:  # 1km/h
            controls.throttle = 0.111
            controls.brake = 0
        elif u >= 0.277 and u < 0.55:  # 2km/h
            controls.throttle = 0.13
            controls.brake = 0
        elif u >= 0.555 and u < 0.83:  # 3km/h
            controls.throttle = 0.163
            controls.brake = 0
        elif u >= 0.833 and u < 1.38:  # 5km/h
            controls.throttle = 0.21
            controls.brake = 0
        elif u >= 1.388 and u < 1.94:  # 7km/h
            controls.throttle = 0.265
            controls.brake = 0
        elif u >= 1.944:  # 10km/h
            controls.throttle = 0.34
            controls.brake = 0
        else:
            controls.throttle = 0.0
            controls.brake = 1
            pass

        time_index = datetime.datetime.now() - self.last_msg_time
        if time_index > datetime.timedelta(seconds=0.2):
            self.joy_msg = None
            controls.throttle = 0.0
            controls.brake = 1

        controls.manual = False
        controls.handbrake = False
        now = rospy.Time.now()
        controls.header.stamp = now
        controls.gear_immediate = True
        self.command_pub.publish(controls)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('airsim_tagnova_control')
    node = TagnovaControl()
    node.run()
