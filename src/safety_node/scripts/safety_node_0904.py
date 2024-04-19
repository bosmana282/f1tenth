#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import time
# TODO: include needed ROS msg type headers and libraries
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
        self.emergency_flag_pb1 = False
        self.emergency_flag_pb2 = False
        self.emergency_flag_fb = False

    def odom_callback(self, odom_msg):
        # Update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
    # Calculate TTC
       # emergency_breaking = False
        brake_range=0
        brake_angle=0
        for idx, r in enumerate(scan_msg.ranges):
            if (np.isnan(r)or r > scan_msg.range_max or r < scan_msg.range_min):
                # To be tuned in real vehicle 
                threshold_pb1 = 20.0 # Treshold for partial braking: slow down easily   
                threshold_pb2 = 20.0 # Treshold for partial braking: slow down          
                threshold_fb = 4.0 # Treshold for full braking
                # if r / max(self.speed * np.cos(idx * scan_msg.angle_increment), 0.001) > threshold_pb1:
                #     self.emergency_flag_pb1 = False
                #     self.emergency_flag_pb2 = False
                #     self.emergency_flag_fb = False
                # elif threshold_pb2 < r / max(self.speed * np.cos(idx * scan_msg.angle_increment), 0.001) < threshold_pb1:
                #     brake_range = r 
                #     brake_angle = 180 * idx * scan_msg.angle_increment / np.pi
                #     self.emergency_flag_pb1 = True
                #     break
                # elif threshold_fb < r / max(self.speed * np.cos(idx * scan_msg.angle_increment), 0.001) < threshold_pb2:
                #     brake_range = r 
                #     brake_angle = 180 * idx * scan_msg.angle_increment / np.pi
                #     self.emergency_flag_pb2 = True
                #     break
                if r / max(self.speed * np.cos(idx * scan_msg.angle_increment), 0.001) < threshold_fb:
                    brake_range = r 
                    brake_angle = 180 * idx * scan_msg.angle_increment / np.pi
                    self.emergency_flag_fb = True
                    break

        #self.emergency_flag = emergency_breaking
        # Publish command to brake
        if self.emergency_flag_fb:
            self.get_logger().info("emergency brake engaged at speed {}".format(self.speed)) # Output to Log
            self.get_logger().info("obstacle detected at {} m".format(brake_range)) # Output to Log
            self.get_logger().info("obstacle detected at {} degrees".format(brake_angle)) # Output to Log
        elif self.emergency_flag_pb2:
            self.get_logger().info("partially braking 2 engaged at speed {}".format(self.speed)) # Output to Log
            self.get_logger().info("obstacle detected at {} m".format(brake_range)) # Output to Log
            self.get_logger().info("obstacle detected at {} degrees".format(brake_angle)) # Output to Log
        elif self.emergency_flag_pb1:
            self.get_logger().info("partially braking 1 engaged at speed {}".format(self.speed)) # Output to Log
            self.get_logger().info("obstacle detected at {} m".format(brake_range)) # Output to Log
            self.get_logger().info("obstacle detected at {} degrees".format(brake_angle)) # Output to Log
            # self.publisher_.publish(drive_msg) # for autonomous control
            # self.teleop_publisher_.publish(drive_msg) # for manual control
        
    def ack_callback(self, ack_msg):
        new_message = AckermannDriveStamped()
        if self.emergency_flag_fb:
            for t in range(2,30,1): # t in steps of 0.2 from 0.2 till 1 to decrease the speed; 1 should be replaced by self.speed in order to bring speed to 0
                new_message.drive = ack_msg.drive
                new_message.drive.speed = self.speed - 0.1*t
                if new_message.drive.speed > 0.0:
                    self.publisher_.publish(new_message)
                else:
                    new_message.drive.speed = 0.0
                    self.publisher_.publish(new_message)
                    break
                    
        # elif self.emergency_flag_pb2:
        #     new_message.drive = ack_msg.drive
        #     new_message.drive.speed = 0.5
        #     self.publisher_.publish(new_message)
        # elif self.emergency_flag_pb1:
        #     new_message.drive = ack_msg.drive
        #     new_message.drive.speed = 1.0
        #     self.publisher_.publish(new_message)
            #time.sleep(3)
            #self.emergency_flag_pb1 = False
        else:
            print("no flags")
            self.get_logger().info("driving at speed {}".format(self.speed))
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