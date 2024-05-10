import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
import math

class Bug2Navigation(Node):
    def __init__(self):
        super().__init__('bug2_navigation')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_reached = False
        self.obstacle_hit = False
        self.obstacle_hit_point = None
        self.resume_point = None
        self.goal_distance_threshold = 0.5  # Threshold distance to consider goal reached
        self.obstacle_hit_distance_threshold = 0.2  # Threshold distance to consider obstacle hit
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def scan_callback(self, msg):
        if not self.goal_reached:
            if not self.obstacle_hit:
                # Move straight toward the goal
                self.move_forward()
            else:
                # Follow obstacle boundary
                self.follow_obstacle_boundary(msg.ranges)
        else:
            self.stop_robot()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.publisher.publish(twist)

    def follow_obstacle_boundary(self, ranges):
        if not self.resume_point:
            # Find the obstacle hit point
            self.obstacle_hit_point = self.find_obstacle_hit_point(ranges)
            # Find a point to resume movement towards the goal
            self.resume_point = self.find_resume_point(ranges)
        else:
            # Calculate angle to resume point
            angle_to_resume = math.atan2(self.resume_point.y, self.resume_point.x)
            # Calculate difference between current angle and angle to resume point
            angle_diff = angle_to_resume - self.current_angle
            # Determine the direction of turn
            if abs(angle_diff) > math.pi:
                if angle_diff > 0:
                    angle_diff -= 2 * math.pi
                else:
                    angle_diff += 2 * math.pi
            if angle_diff > 0:
                self.turn_right()
            else:
                self.turn_left()

    def turn_left(self):
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -self.angular_speed
        self.publisher.publish(twist)

    def find_obstacle_hit_point(self, ranges):
        min_range = min(ranges)
        if min_range < self.obstacle_hit_distance_threshold:
            index = ranges.index(min_range)
            angle = index * math.pi * 2 / len(ranges) - math.pi
            return Point(min_range * math.cos(angle), min_range * math.sin(angle))
        return None

    def find_resume_point(self, ranges):
        # Find the point where the obstacle ends
        obstacle_end_point = None
        for i in range(len(ranges)):
            if ranges[i] > self.obstacle_hit_distance_threshold:
                obstacle_end_point = Point(ranges[i] * math.cos(i * math.pi * 2 / len(ranges) - math.pi),
                                           ranges[i] * math.sin(i * math.pi * 2 / len(ranges) - math.pi))
                break
        if obstacle_end_point:
            # Find the closest point to the goal among obstacle_end_point and obstacle_hit_point
            distance_to_goal = math.sqrt(self.goal_position.x ** 2 + self.goal_position.y ** 2)
            distance_to_obstacle_end = math.sqrt(obstacle_end_point.x ** 2 + obstacle_end_point.y ** 2)
            distance_to_obstacle_hit = math.sqrt(self.obstacle_hit_point.x ** 2 + self.obstacle_hit_point.y ** 2)
            if distance_to_obstacle_end < distance_to_obstacle_hit and distance_to_obstacle_end < distance_to_goal:
                return obstacle_end_point
            elif distance_to_obstacle_hit < distance_to_obstacle_end and distance_to_obstacle_hit < distance_to_goal:
                return self.obstacle_hit_point
        # If no point found or if goal is closer than obstacle_end_point, return goal
        return self.goal_position

    def stop_robot(self):
        twist = Twist()
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    bug2_nav = Bug2Navigation()
    rclpy.spin(bug2_nav)
    bug2_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
