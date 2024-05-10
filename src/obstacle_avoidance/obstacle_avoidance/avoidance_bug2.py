import rclpy
from rclpy.node import Node
import math
import numpy as np

from .motion_control_interface import MotionControlInterface
from .motion_feedback import FeedbackControlOA
from .motion_ppursuit import PurePursuitOA
from .obstacle_avoidance_interface import ObstacleAvoidanceInterface

class Bug2OA(ObstacleAvoidanceInterface):
    def __init__(self, motion_controller: MotionControlInterface):
        #super().__init__(self)
        self.motion_controller = motion_controller

        # Initialize obstacle avoidance parameters
        self.obstacle_hit_threshold = 0.6 # is not in meters!
        self.relative_polar_hit_point = (0.0,0.0)
        self.obstacle_hit = False
        self.obstacle_leave = False
        self.angles = None
        self.complete_turn = False

        self.v = 0.0
        self.omega = 0.0

    def calculate_reaction(self, target_point, current_pose, relative_polar_hit_point, scan_angles, scan_ranges, arrival_flag):
        v = self.v
        omega = self.omega
        self.arrival_flag = arrival_flag
        min_range,min_angle = relative_polar_hit_point
        self.relative_polar_hit_point = relative_polar_hit_point
        if not self.arrival_flag:
            if not self.obstacle_hit:
                #self.get_logger().info("Default motion control")
                v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            else:
                #self.get_logger().info("Obstacle motion control")
                if 0.0 <= min_angle < 180:
                    # Turn to right until obstacle at 90 deg, then follow the obstacle
                    if not self.complete_turn:
                        v, omega = self.turn_right()
                        #self.get_logger().info("Turning right")
                    else:
                        v, omega = self.follow_obstacle_boundary_left(scan_angles, scan_ranges)
                        #self.get_logger().info("Following boundary at left")
                elif 180 <= min_angle < 360:
                    # Turn to left until obstacle at 270 deg, then follow the obstacle
                    if not self.complete_turn:
                        v, omega = self.turn_left()
                        #self.get_logger().info("Turning left")
                    else:
                        v, omega = self.follow_obstacle_boundary_right(scan_angles, scan_ranges)
                        #self.get_logger().info("Following boundary at right")
                
                # Go back to business as usual once leave point is reached
                self.find_leave_point(target_point, current_pose, min_angle)
            #self.get_logger().info("v, omega: {}".format([self.v, self.omega]))
        return v, omega

    def follow_obstacle_boundary_left(self, scan_angles, scan_ranges):
        v = 0.0
        omega = 0.0
        min_angle = 75
        max_angle = 105

        right_side_ranges = [scan_ranges[i] for i in range(len(scan_ranges)) if min_angle <= scan_angles[i] <= max_angle]
        right_side_ranges = [scan_ranges[i] for i in range(len(right_side_ranges)) if np.isnan(right_side_ranges)]
        #self.get_logger().info("Right side ranges: {}".format(right_side_ranges))
        if np.sum(right_side_ranges)/len(right_side_ranges) > 0.9:
            #self.get_logger().info("Sum ranges: {}".format(np.sum(right_side_ranges)/len(right_side_ranges)))
            omega = 0.1
        elif np.sum(right_side_ranges)/len(right_side_ranges) < 0.5:
            #self.get_logger().info("Sum ranges: {}".format(np.sum(right_side_ranges)/len(right_side_ranges)))
            omega = -0.1
        else:
            omega = 0.0
        return v, omega
    
    def follow_obstacle_boundary_right(self, scan_angles, scan_ranges):
        v = 1.3
        omega = 0.0
        min_angle = 255
        max_angle = 285

        right_side_ranges = [scan_ranges[i] for i in range(len(scan_ranges)) if min_angle <= scan_angles[i] <= max_angle]
        right_side_ranges = [scan_ranges[i] for i in range(len(right_side_ranges)) if np.isnan(right_side_ranges)]
        #self.get_logger().info("Right side ranges: {}".format(right_side_ranges))
        if np.sum(right_side_ranges)/len(right_side_ranges) > 0.9:
            #self.get_logger().info("Sum ranges: {}".format(np.sum(right_side_ranges)/len(right_side_ranges)))
            omega = 0.1
        elif np.sum(right_side_ranges)/len(right_side_ranges) < 0.5:
            #self.get_logger().info("Sum ranges: {}".format(np.sum(right_side_ranges)/len(right_side_ranges)))
            omega = -0.1
        else:
            omega = 0.0
        return v, omega

    def turn_left(self):
        v = 0.0
        omega = 0.0
        min_range, min_angle = self.relative_polar_hit_point
        if min_range > 2*self.obstacle_hit_threshold/3:
            # Obstacle still 'far' away --> go faster
            v = 1.5
        else:
            # Obstacle close --> go slower
            v = 1.2
        if min_angle > 345 or min_angle < 15:
            # Obstacle straight ahead --> sharp turn
            omega = 0.4
        elif 270 <= min_angle <= 345:
            self.complete_turn = True
            omega = 0.2
        return v,omega
    
    def turn_right(self):
        v = 0.0
        omega = 0.0
        min_range, min_angle = self.relative_polar_hit_point
        if min_range > 2*self.obstacle_hit_threshold/3:
            # Obstacle still 'far' away --> go faster
            v = 1.5
        else:
            # Obstacle close --> go slower
            v = 1.2
        if min_angle > 345 or min_angle < 15:
            # Obstacle straight ahead --> sharp turn
            omega = -0.4
        elif 15 <= min_angle <= 90:
            self.complete_turn = True
            omega = -0.2
        return v,omega


    def find_leave_point(self, target_point, current_pose, hit_angle):
        # Leave obstacle once goal is at same angle as before obstacle
        x, y, theta = current_pose  # CHECK current pose in right format?
        tx, ty = target_point
        alpha = - theta + math.atan2(ty-y, tx-x)
        if abs(360 - hit_angle + 90 - np.degrees(alpha)) < 5: # precision of 5 deg
            self.obstacle_hit = False
            #self.get_logger().info("Leave point reached")  