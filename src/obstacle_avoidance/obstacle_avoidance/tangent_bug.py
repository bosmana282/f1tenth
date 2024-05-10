import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TangentBug(Node):
    def __init__(self):
        super().__init__('tangent_bug')
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.goal_distance = 1.0  # Adjust as needed
        self.min_distance = 0.2  # Minimum distance to obstacle
        self.following_wall = False

    def scan_callback(self, msg):
        # Check if there are obstacles in front of the robot
        if min(msg.ranges) < self.min_distance:
            # If following wall, check if obstacle allows resuming towards goal
            if self.following_wall:
                min_angle_index = msg.ranges.index(min(msg.ranges))
                angle_range = 20  # Angle range to check for clear path
                if all(d > self.min_distance for d in msg.ranges[min_angle_index - angle_range: min_angle_index + angle_range]):
                    self.following_wall = False

            if not self.following_wall:
                # Robot too close to obstacle, stop and rotate to find clear path
                self.rotate_to_clear_path()
        else:
            # No obstacle in front of the robot, move towards goal
            self.move_to_goal()

    def move_to_goal(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Adjust speed as needed
        twist_msg.angular.z = 0.0
        self.cmd_publisher.publish(twist_msg)

    def rotate_to_clear_path(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.5  # Adjust rotation speed as needed
        self.cmd_publisher.publish(twist_msg)
        self.following_wall = True

def main(args=None):
    rclpy.init(args=args)
    tangent_bug = TangentBug()
    rclpy.spin(tangent_bug)
    tangent_bug.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
