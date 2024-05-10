import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import euler_from_quaternion
from .motion_control_interface import MotionControlInterface
from .motion_feedback import FeedbackControlOA
from .motion_ppursuit import PurePursuitOA

class WaypointNavigation(Node):
    def __init__(self, motion_controller: MotionControlInterface):
        """
        Handles the navigation of the robot from beginning to start.
        """
        super().__init__('waypoint_navigation')
        self.motion_controller = motion_controller

        # Initialize waypoint navigation parameters
        # self.waypoints = [(0, 0), (2, 2), (0, 4), (-2, 2), (0, 0)]
        self.waypoints = [(0, 0), (2, 2), (0, 5)]
        self.current_waypoint_index = 0
        self.wheelbase = 0.3
        self.stop_distance = 0.5 # distance from which vehicle will slow down in front of final goal
        self.create_timer(0.1, self.timer_callback)

        self.v = 0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.pose = (0,0,0) # [m,m,rad]
        self.WP_index = 0
        self.WP_flag = False
        self.arrival_flag = False

        # Create ROS subscribers and publishers
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

        # Update the speed and steering angle of the car
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd2', 1000)


    def odom_callback(self, odom_msg):
        # Update current speed, steering angle and pose
        self.speed = odom_msg.twist.twist.linear.x
        self.steering_angle = odom_msg.twist.twist.angular.z

        pos_x = odom_msg.pose.pose.position.x
        pos_y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        theta = euler_from_quaternion(quaternion)[2] 
        self.pose = pos_x,pos_y,theta

    def timer_callback(self):
        # Find the target point
        target_point = self.find_target_point(self.waypoints)

        self.distance_WP = np.linalg.norm(np.array(target_point) - np.array([self.pose[0], self.pose[1]]))
        self.get_logger().info("WP check: {}".format(self.distance_WP))
        self.get_logger().info("Target Point: {}".format(target_point)) 
        self.get_logger().info("Current position: {}".format(self.pose))    

        if self.WP_index == len(self.waypoints)-1:
            self.get_logger().info("Heading towards the last WP: {}".format(target_point))
            if np.linalg.norm(np.array(self.waypoints[-1]) - np.array([self.pose[0], self.pose[1]])) < self.stop_distance:
                self.arrival_flag = True
        elif self.distance_WP < 0.1:
            self.WP_flag = True
        if np.linalg.norm(np.array(self.waypoints[-1]) - np.array([self.pose[0], self.pose[1]])) < 0.05:
            self.arrival_flag = True
            self.get_logger().info("Arrived at goal: {}".format(target_point)) # evt add stop node


        # Calculate steering angle with selected motion controller
        self.v, self.omega = self.motion_controller.calculate_steering(target_point, self.pose)

    def find_target_point(self, waypoints): # If necessary, change PP "find_target_point" to this one
        """
        Makes the car follow the list of WPs. Once close enough to the current one, the target point will change to the next WP.
        """

        if self.WP_flag == True:
            self.WP_index = self.WP_index + 1
            self.WP_flag = False

        return waypoints[self.WP_index]

    def ack_callback(self, ack_msg):
        # Publish control commands
        new_message = AckermannDriveStamped()
        new_message.drive = ack_msg.drive
        if self.arrival_flag == True: # make car stop at exact right spot
            new_message.drive.speed = 0.0
            new_message.drive.steering_angle = 0.0
            self.publisher_.publish(new_message)
        else:
            new_message.drive.speed = max(self.v, 1.0) # tune based on chosen terrain/track; also add min(self.v, 2.5)?
            new_message.drive.steering_angle = self.omega #steering_angle_velocity
            self.publisher_.publish(new_message)

    # Other functions for waypoint navigation

def main(args=None):
    rclpy.init(args=args)

    # Allow the user to select the motion control method
    motion_control_method = input("Select motion control method: pure_pursuit (1) or feedback_control (2): ")

    if motion_control_method == "1":
        motion_controller = PurePursuitOA()
    elif motion_control_method == "2":
        motion_controller = FeedbackControlOA()
    else:
        print("Invalid motion control method. Please select either '1' or '2'.")
        return

    waypoint_navigation = WaypointNavigation(motion_controller)
    rclpy.spin(waypoint_navigation)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
