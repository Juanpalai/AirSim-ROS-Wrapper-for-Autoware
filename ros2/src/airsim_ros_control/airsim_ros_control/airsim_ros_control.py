import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import airsim_interfaces as air
import airsim_interfaces.msg
import autoware_auto_control_msgs
import autoware_auto_control_msgs.msg
import threading
import time
import datetime


class TagnovaControl(Node):
    def __init__(self):
        super().__init__('airsim_ros_control')

        # Get parameters from ROS parameter server
        self.update_rate_hz = self.declare_parameter('~update_rate_hz', 20.0).value
        self.max_curvature = self.declare_parameter('~max_curvature', 0.75).value
        self.steer_sign = self.declare_parameter('~steer_sign', -1).value
        self.throttle_brake_sign = self.declare_parameter('~throttle_brake_sign', 1).value
        self.car_control_topic = self.declare_parameter('~car_control_topic', '/airsim_node/drone_1/car_cmd').value

        # Initialize class attributes
        # Create a lock object to synchronize access to shared resources
        self.lock = threading.Lock()
        # Set the last message time to the current time
        self.last_msg_time = datetime.datetime.now()
        self.magnetig_time = datetime.datetime.now()
        self.magnetig_state = False
        self.angular = 0

        self.joy_msg = None  # Initialize a variable to store Autoware commands received

        # Subscribe to ROS topics to receive Autoware commands
        #         
        self.twist_raw_sub = self.create_subscription(autoware_auto_control_msgs.msg.AckermannControlCommand, '/control/command/control_cmd', self.take_time, 10)

        # Create ROS publishers to send car control commands
        self.command_pub = self.create_publisher(air.msg.CarControls, self.car_control_topic, 10)
        

        # Create a ROS timer to periodically execute a callback function
        self.update_time = self.create_timer(1.0/self.update_rate_hz, self.handle_update_timer)

        self.get_logger().info("AIRSIM CONTROL START")

    def magnetig_callback(self, msg):
        if msg.data == "start":
            msg.data = "stop"
            time.sleep(6)
            self.manet_drive_state_pub.publish(msg)
            time.sleep(6)
            self.manet_drive_state_pub.publish(msg)
            time.sleep(6)
            self.manet_drive_state_pub.publish(msg)
            time.sleep(6)
            self.manet_drive_state_pub.publish(msg)

    def take_time(self, msg):
        # self.angular = msg.twist.angular.z
        self.joy_msg = msg
        self.last_msg_time = datetime.datetime.now()

    def handle_joy(self, msg):
        with self.lock:
            self.joy_msg = msg

    def handle_update_timer(self):
        joy = None
        with self.lock:
            joy = self.joy_msg

        if joy is None:
            return

        controls = airsim_interfaces.msg.CarControls()
        controls.steering = -1.5 * joy.lateral.steering_tire_angle   # self.angular

        u = joy.longitudinal.speed

        if 0.0 < u < 0.27:  # 1km/h
            controls.throttle = 0.13
            controls.brake = 0.0
        elif 0.277 <= u < 0.55:  # 2km/h
            controls.throttle = 0.16
            controls.brake = 0.0
        elif 0.555 <= u < 0.83:  # 3km/h
            controls.throttle = 0.18
            controls.brake = 0.0
        elif 0.833 <= u < 1.38:  # 5km/h
            controls.throttle = 0.22
            controls.brake = 0.0
        elif 1.388 <= u < 1.94:  # 7km/h
            controls.throttle = 0.27
            controls.brake = 0.0
        elif u >= 1.944:  # 10km/h
            controls.throttle = 0.335
            controls.brake = 0.0
        else:
            controls.throttle = 0.0
            controls.brake = 1.0       

        time_index = datetime.datetime.now() - self.last_msg_time
        if time_index > datetime.timedelta(seconds=0.2):
            self.joy_msg = None
            controls.throttle = 0.0
            controls.brake = 1.0

        controls.manual = False
        controls.handbrake = False
        now = self.get_clock().now()
        controls.header.stamp = now.to_msg()
        controls.gear_immediate = True
        self.command_pub.publish(controls)

    def run(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    node = TagnovaControl()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    

