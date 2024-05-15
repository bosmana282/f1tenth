import rclpy
from rclpy.node import Node
import math
import numpy as np

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from .motion_control_interface import MotionControlInterface
from .motion_feedback import FeedbackControlOA
from .motion_ppursuit import PurePursuitOA
from .obstacle_avoidance_interface import ObstacleAvoidanceInterface
from .avoidance_bug2 import Bug2OA
from .avoidance_tangent import TangentBugOA

class ObstacleAvoidance(Node):
    def __init__(self, motion_controller: MotionControlInterface, obstacle_avoider: ObstacleAvoidanceInterface):
        """
        Handles the navigation of the robot from beginning to start.
        """
        super().__init__('obstacle_avoidance')
        self.motion_controller = motion_controller
        self.obstacle_avoider = obstacle_avoider

        # Initialize waypoint navigation parameters
        self.relative_polar_hit_point = (0.0,0.0)
        self.scan_angles = []
        self.scan_ranges = []
        self.obstacle_hit_threshold = 1.2 # also in avoidance_bug2!
        self.obstacle_hit = False

        # self.waypoints = [(0, 0), (2, 2), (0, 4), (-2, 2), (0, 0)]
        self.waypoints = [(0, 0), (5, 0)]
        self.current_waypoint_index = 0
        self.wheelbase = 0.3
        self.stop_distance = 0.5 # distance from which vehicle will slow down in front of final goal
        self.create_timer(0.1, self.timer_callback)

        self.v = 0.0
        self.omega = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.pose = (0,0,0) # [m,m,rad]
        self.WP_index = 0
        self.WP_flag = False
        self.arrival_flag = False

        # Create ROS subscribers and publishers
        self.scan_subscription = self.create_subscription( 
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

    def scan_callback(self, scan_msg):
        scan_ranges = scan_msg.ranges
        min_range = np.min(scan_ranges)
        
        if min_range < self.obstacle_hit_threshold:
            self.obstacle_hit = True
            index = np.argmin(scan_ranges)
            min_angle = 180 * index * scan_msg.angle_increment / np.pi
            relative_polar_hit_point = (min_range,min_angle)
        else:
            self.obstacle_hit = False
            relative_polar_hit_point = (None,None)
        self.scan_angles = [180*i*scan_msg.angle_increment/np.pi for i in range(len(scan_ranges))]
        
        self.scan_ranges = scan_ranges
        self.relative_polar_hit_point = relative_polar_hit_point

    def timer_callback(self):
        # Find the target point
        target_point, gen_direct = self.find_target_point(self.waypoints)

        self.distance_WP = np.linalg.norm(np.array(target_point) - np.array([self.pose[0], self.pose[1]]))
        # self.get_logger().info("WP check: {}".format(self.distance_WP))
        self.get_logger().info("Target Point: {}".format(target_point)) 
        self.get_logger().info("Current position: {}".format(self.pose)) 
        self.get_logger().info("General direction: {}".format(gen_direct))   

        if self.WP_index == len(self.waypoints)-1:
            #self.get_logger().info("Heading towards the last WP: {}".format(target_point))
            if np.linalg.norm(np.array(self.waypoints[-1]) - np.array([self.pose[0], self.pose[1]])) < self.stop_distance:
                self.arrival_flag = True
        elif self.distance_WP < 0.1:
            self.WP_flag = True
        if np.linalg.norm(np.array(self.waypoints[-1]) - np.array([self.pose[0], self.pose[1]])) < 0.05:
            self.arrival_flag = True
            #self.get_logger().info("Arrived at goal")
            #self.get_logger().info("Arrived at goal: {}".format(target_point)) # evt add stop node

        # Find point where car hits the obstacle (if there is one...)
        min_range,min_angle = self.relative_polar_hit_point
        self.get_logger().info("Hit angle: {}".format(min_angle))
        self.get_logger().info("Min range: {}".format(min_range))

        # Calculate steering angle with selected motion controller
        self.v, self.omega, leave_flag, average, goal = self.obstacle_avoider.calculate_reaction(target_point, self.pose, self.relative_polar_hit_point, self.scan_angles, self.scan_ranges, self.arrival_flag, self.obstacle_hit, gen_direct) 
        self.get_logger().info("v, omega: {}".format([self.v, self.omega])) 
        self.get_logger().info("Leave: {}".format(leave_flag))
        self.get_logger().info("Average wall dist: {}".format(average))
        self.get_logger().info("Goal direction: {}".format(goal))

    def find_target_point(self, waypoints): 
        """
        Makes the car follow the list of WPs. Once close enough to the current one, the target point will change to the next WP.
        """
        if self.WP_flag == True:
            self.WP_index = self.WP_index + 1
            self.WP_flag = False
        general_direction = self.calculate_general_direction(waypoints)
        return waypoints[self.WP_index], general_direction

    def calculate_general_direction(self, waypoints):
        general_direction = 0.0
        if self.WP_index > 0:
            x1, y1 = waypoints[self.WP_index-1]
            x2, y2 = waypoints[self.WP_index]
            general_direction = np.degrees(math.atan2(y2-y1,x2-x1))
        return general_direction

    def ack_callback(self, ack_msg):
        # Publish control commands
        new_message = AckermannDriveStamped()
        new_message.drive = ack_msg.drive
        if self.arrival_flag == True: # make car stop at exact right spot
            new_message.drive.speed = 0.0
            new_message.drive.steering_angle = 0.0
            self.publisher_.publish(new_message)
        else:
            new_message.drive.speed = max(self.v, 0.8) # tune based on chosen terrain/track; also add min(self.v, 2.5)?
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

    obstacle_avoidance_method = input("Select obstacle avoidance method: Bug2 (1), Tangent Bug (2), or VFH (3): ")

    if obstacle_avoidance_method == "1":
        obstacle_avoider = Bug2OA(motion_controller)
    elif obstacle_avoidance_method == "2":
         obstacle_avoider = TangentBugOA(motion_controller)
    # elif obstacle_avoidance_method == "3":
    #     obstacle_avoider = VFHOA()
    else:
        print("Invalid obstacle avoidance method. Please select either '1', '2' or '3'.")
        return

    obstacle_avoidance = ObstacleAvoidance(motion_controller, obstacle_avoider)
    rclpy.spin(obstacle_avoidance)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
