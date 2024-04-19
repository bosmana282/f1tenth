import rclpy
from rclpy.node import Node

import numpy as np
import math
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import euler_from_quaternion

class PurePursuitController(Node):
    """
    The class that handles motion control by pure pursuit.
    """
    def __init__(self):
        super().__init__('pure_pursuit')
        """
        Publishing to the /drive topic with a AckermannDriveStamped drive message.
        Subscription to /ego_racecar/odom topic to get the current linear and angularspeed of the vehicle.
        
        x component of the linear velocity in odom is the speed
        z component of the angular velocity in odom is the turning speed
        """
        
        self.lookahead_distance = 0.5
        self.wheelbase = 0.3
        self.create_timer(0.1, self.timer_callback)

        # Initialization
        self.speed = 0.0
        self.steering_angle = 0.0
        self.pose = (0,0,0) # [m,m,rad]
        self.arrival_flag = False

        # Create ROS subscribers and publishers.
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
        waypoints = [(0, 0), (10, 10), (20, 10), (30, 10)] # TO BE ADAPTED to a function that generates standard waypoints based on input values start and ending point

        # Find the target point
        target_point = self.find_target_point(waypoints)

        if target_point == waypoints[-1]:
            self.get_logger().info("Heading towards the last WP: {}".format(target_point))
            if np.linalg.norm(np.array(waypoints[-1]) - np.array([self.pose[0], self.pose[1]])) < 1:
                self.arrival_flag = True
                self.get_logger().info("Arrived at goal: {}".format(target_point))

        # Calculate the steering angle
        delta = self.calculate_steering(target_point)
        self.delta = delta
        # Print results
        self.get_logger().info("Target Point: {}".format(target_point))
        self.get_logger().info("Current position: {}".format(self.pose))

    def find_target_point(self, waypoints):
        """
        Finds the target point on the path that is closest to the vehicle.
        """
        closest_dist = float('inf')
        closest_index = 0
        
        for i, waypoint in enumerate(waypoints):
            dist = np.linalg.norm(np.array(waypoint) - np.array([self.pose[0], self.pose[1]]))
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i
                
        lookahead_index = closest_index
        total_distance = 0.0

        while total_distance < self.lookahead_distance and lookahead_index < len(waypoints) - 1:
            total_distance += np.linalg.norm(np.array(waypoints[lookahead_index]) - np.array(waypoints[lookahead_index + 1]))
            lookahead_index += 1

        

        return waypoints[lookahead_index]
    
    # def check_target_point(self):
    #     wp = 

    def calculate_steering(self, target_point):
        """
        Calculates the steering angle required to follow the target point.
        """
        x, y, theta = self.pose
        tx, ty = target_point
        l = self.wheelbase
        l_d = self.lookahead_distance
        alpha = math.atan2(ty - y, tx - x) - theta
        L = np.linalg.norm(np.array(target_point) - np.array([x, y])) # lateral error

        delta = math.atan2(2*l*math.sin(alpha),l_d)
        
        if delta > 0:
             delta = min(delta, 0.3) # 0.36 rad = max turning radius
        else:
             delta = max(delta, -0.3) # 0.36 rad = max turning radius
        angles = np.degrees(theta), np.degrees(alpha), np.degrees(delta)
        self.get_logger().info("Actual steering angle: {}".format(self.steering_angle))
        self.get_logger().info("Steering angle test: {}".format(angles))
        return delta
        

    def ack_callback(self, ack_msg):
        new_message = AckermannDriveStamped()
        new_message.drive = ack_msg.drive
        if self.arrival_flag == True:            
            for time in range(2,150,1):
                new_speed = max(1.0 - time*0.01, 0.0)
                new_message.drive.speed = new_speed
                self.get_logger().info("Speed: {}".format(new_speed))
                new_message.drive.steering_angle = 0.0
                self.publisher_.publish(new_message)
                if new_speed == 0.0:
                    break
        else:
            new_message.drive.speed = 1.0
            new_message.drive.steering_angle = self.delta
            self.publisher_.publish(new_message)


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuitController()
    rclpy.spin(pure_pursuit)

    #pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
