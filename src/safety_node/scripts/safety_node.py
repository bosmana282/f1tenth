#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.speed = 0.0
        # Create ROS subscribers and publishers.
        self.scan_subscription = self.create_subscription( # Subscribe to the scan topic
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom', # this is more reliable than /pf/pose/odom
            self.odom_callback,
            10
        )
 
        self.ackcmd_subscription = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd', # this is more reliable than /pf/pose/odom
            self.ack_callback,
            10
        )

        # Update the speed of the car
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd2', 1000)
        #self.teleop_publisher_ = self.create_publisher(AckermannDriveStamped, 'ackermann_to_vesc_node', 1000)
        #self.bool_publisher_ = self.create_publisher(Bool, '/emergency_breaking', 1000)
        self.emergency_flag = False

    def odom_callback(self, odom_msg):
        # Update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calculate TTC
        # emergency_breaking = False
        brake_range=0
        brake_angle=0
        for idx, r in enumerate(scan_msg.ranges):
            if (np.isnan(r)or r > scan_msg.range_max or r < scan_msg.range_min): continue
            threshold = 0.08 # To be tuned in real vehicle
            if r / max(self.speed * np.cos(idx * scan_msg.angle_increment), 0.001) < threshold:
                brake_range = r 
                brake_angle = 180 * idx * scan_msg.angle_increment / np.pi
                emergency_breaking = True
                self.emergency_flag = True
                break

        #self.emergency_flag = emergency_breaking
        # Publish command to brake
        if self.emergency_flag:
            self.get_logger().info("emergency brake engaged at speed {}".format(self.speed)) # Output to Log
            self.get_logger().info("obstacle detected at {} m".format(brake_range)) # Output to Log
            self.get_logger().info("obstacle detected at {} degrees".format(brake_angle)) # Output to Log
            # self.publisher_.publish(drive_msg) # for autonomous control
            # self.teleop_publisher_.publish(drive_msg) # for manual control
        
    def ack_callback(self, ack_msg):
        new_message = AckermannDriveStamped()
        if self.emergency_flag:
            new_message.drive = ack_msg.drive
            new_message.drive.speed = 0.0
            self.publisher_.publish(new_message)
        else:
            new_message.drive = ack_msg.drive
            self.publisher_.publish(new_message)
    
def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()