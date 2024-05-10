import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class VFHPlus(Node):
    def __init__(self):
        super().__init__('vfh_plus')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()

        self.angle_increment = None
        self.min_angle = None
        self.max_angle = None
        self.num_ranges = None
        self.scan_ranges = None
        self.obstacle_range = 0.5  # Define obstacle avoidance threshold

    def scan_callback(self, msg):
        self.angle_increment = msg.angle_increment
        self.min_angle = msg.angle_min
        self.max_angle = msg.angle_max
        self.num_ranges = len(msg.ranges)
        self.scan_ranges = msg.ranges

        # Perform VFH+ obstacle avoidance
        best_direction = self.compute_best_direction()
        
        # Publish the velocity command
        self.publish_velocity_command(best_direction)

    def compute_best_direction(self):
        # Compute obstacle density in each sector
        sector_size = 10  # Define sector size in degrees
        num_sectors = int(np.ceil((self.max_angle - self.min_angle) / sector_size))
        sector_densities = []

        for sector in range(num_sectors):
            start_index = int(sector * sector_size / self.angle_increment)
            end_index = int((sector + 1) * sector_size / self.angle_increment)
            sector_ranges = self.scan_ranges[start_index:end_index]
            obstacle_count = np.count_nonzero(np.array(sector_ranges) < self.obstacle_range)
            sector_density = obstacle_count / len(sector_ranges)
            sector_densities.append(sector_density)

        # Find the sector with the lowest obstacle density
        best_sector = np.argmin(sector_densities)

        # Calculate the angle corresponding to the center of the chosen sector
        best_direction = self.min_angle + (best_sector * sector_size + sector_size / 2) * self.angle_increment
        return best_direction

    def publish_velocity_command(self, best_direction):
        # Set linear velocity
        self.cmd_vel.linear.x = 0.5  # Move forward

        # Set angular velocity based on the best direction
        angular_speed = 0.5
        self.cmd_vel.angular.z = np.clip(angular_speed * (best_direction - 0), -1, 1)  # Adjust to zero-centered direction

        # Publish the velocity command
        self.cmd_pub.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    vfh_plus = VFHPlus()
    rclpy.spin(vfh_plus)
    vfh_plus.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
