import rclpy
from rclpy.node import Node

import numpy as np
import math
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import euler_from_quaternion

class FeedbackController(Node):
    """
    The class that handles motion control by feedback control.
    """
    def __init__(self):
        super().__init__('feedback_control')
        """
        Publishing to the /drive topic with a AckermannDriveStamped drive message.
        Subscription to /ego_racecar/odom topic to get the current linear and angularspeed of the vehicle.
        
        x component of the linear velocity in odom is the speed
        z component of the angular velocity in odom is the turning speed
        """
        
        self.k_alpha = 2
        self.k_beta = -1
        self.k_rho = 0.15 # possibly add minimal value for speed since too close to target --> too slow --> stall
        self.wheelbase = 0.3
        self.stop_distance = 0.5 # distance from which vehicle will slow down in front of final goal
        self.create_timer(0.1, self.timer_callback)

        # Initialization
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
        theta = euler_from_quaternion(quaternion)[2] #+np.pi/2
        self.pose = pos_x,pos_y,theta

    def timer_callback(self):
        # Example waypoints
        # waypoints = [(5, 10), (20, 10), (30, 10)] # TO BE ADAPTED to a function that generates standard waypoints based on input values start and ending point
        waypoints = [(3, 0), (3, 4)]
        # waypoints = [(6, 0)]
        # Find the target point
        target_point = self.find_target_point(waypoints)

        self.distance_WP = np.linalg.norm(np.array(target_point) - np.array([self.pose[0], self.pose[1]]))
        self.get_logger().info("WP check: {}".format(self.distance_WP))    

        if target_point == waypoints[-1]:
            self.get_logger().info("Heading towards the last WP: {}".format(target_point))
            # if np.linalg.norm(np.array(waypoints[-1]) - np.array([self.pose[0], self.pose[1]])) < self.stop_distance:
            #      self.arrival_flag = True
        elif self.distance_WP < 0.1:
            self.WP_flag = True
        if np.linalg.norm(np.array(waypoints[-1]) - np.array([self.pose[0], self.pose[1]])) < 0.08:
            self.arrival_flag = True
            self.get_logger().info("Arrived at goal")
            self.get_logger().info("Arrived at goal: {}".format(target_point)) # evt add stop node


        # Calculate the steering angle
        self.v, self.omega = self.calculate_steering(target_point)
        
        # Print results
        self.get_logger().info("Target Point: {}".format(target_point))
        self.get_logger().info("Current position: {}".format(self.pose))

    def find_target_point(self, waypoints): # If necessary, change PP "find_target_point" to this one
        """
        Makes the car follow the list of WPs. Once close enough to the current one, the target point will change to the next WP.
        """
        if self.WP_flag == True:
            self.WP_index = self.WP_index + 1
            self.WP_flag = False

        return waypoints[self.WP_index]

    def calculate_steering(self, target_point):
        """
        Calculates the angular and linear velocity required to follow the target point.
        """
        x, y, theta = self.pose
        tx, ty = target_point
        rho =  math.sqrt(pow(ty-y,2) + pow(tx-x,2)) # distance from robot to target
        alpha = - theta + math.atan2(ty-y, tx-x)
        beta = - theta - alpha       
        v = self.k_rho*rho
        omega1 = self.k_alpha*alpha + self.k_beta*beta
        
        if omega1 > 0:
             omega = min(omega1, 0.36) # 0.36 rad = max turning radius
        else:
             omega = max(omega1, -0.36) # 0.36 rad = max turning radius
        self.get_logger().info("Actual [rho, alpha, beta]: {}".format([rho, alpha, beta]))
        self.get_logger().info("Actual [v, omega1, omega]: {}".format([v, omega1, omega]))
        return v, omega        

    def ack_callback(self, ack_msg):
        new_message = AckermannDriveStamped()
        new_message.drive = ack_msg.drive
        if self.arrival_flag == True: # make car stop at exact right spot
            new_message.drive.speed = 0.0
            new_message.drive.steering_angle = 0.0
            self.publisher_.publish(new_message)
        else:
            new_message.drive.speed = max(self.v, 1.0) # tune based on chosen terrain/track; also add min(self.v, 2.5)?
            new_message.drive.steering_angle = self.omega
            self.publisher_.publish(new_message)

    # def ack_callback(self, ack_msg):
    #     new_message = AckermannDriveStamped()
    #     new_message.drive = ack_msg.drive
        
    #     new_message.drive.speed = 1.0
    #     new_message.drive.steering_angle = 0.36
    #     self.publisher_.publish(new_message)


def main(args=None):
    rclpy.init(args=args)
    feedback_control = FeedbackController()
    rclpy.spin(feedback_control)

    #pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
