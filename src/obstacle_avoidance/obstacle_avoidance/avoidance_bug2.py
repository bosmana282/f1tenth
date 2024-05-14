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
        self.obstacle_hit_threshold = 1.2 
        self.relative_polar_hit_point = (0.0,0.0)
        self.obstacle_hit = False
        self.obstacle_leave = False
        self.angles = None
        self.complete_turn = False
        self.leave_flag = False

        self.v = 0.0
        self.omega = 0.0

    def calculate_reaction(self, target_point, current_pose, relative_polar_hit_point, scan_angles, scan_ranges, arrival_flag, obstacle_hit, gen_direction):
        #leave_flag = False
        goal_direction = None
        average = None
        v = self.v
        omega = self.omega
        min_range,min_angle = relative_polar_hit_point

        self.obstacle_hit = obstacle_hit
        self.arrival_flag = arrival_flag
        self.relative_polar_hit_point = relative_polar_hit_point

        if obstacle_hit and not self.obstacle_hit:
            self.leave_flag = False

        if not self.arrival_flag:
            if not self.obstacle_hit:
                #self.get_logger().info("Default motion control")
                v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            elif self.leave_flag:
                v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            else:
                #self.get_logger().info("Obstacle motion control")
                if 0.0 <= min_angle < 180:
                    # Turn to right until obstacle at 90 deg, then follow the obstacle
                    if not self.complete_turn:
                        v, omega = self.turn_right()
                        #self.get_logger().info("Turning right")
                    else:
                        v, omega, average = self.follow_obstacle_boundary_left(scan_angles, scan_ranges)
                        if not self.leave_flag:
                            self.leave_flag, goal_direction = self.find_leave_point2(target_point, current_pose, min_angle, gen_direction)
                        else:
                            if min_range > self.obstacle_hit_threshold:
                                self.leave_flag = False
                                self.complete_turn = False  
                        #self.get_logger().info("Following boundary at left")
                elif 180 <= min_angle < 360:
                    # Turn to left until obstacle at 270 deg, then follow the obstacle
                    if not self.complete_turn:
                        v, omega = self.turn_left()
                        #self.get_logger().info("Turning left")
                    else:
                        v, omega, average = self.follow_obstacle_boundary_right(scan_angles, scan_ranges)
                        if not self.leave_flag:
                            self.leave_flag, goal_direction = self.find_leave_point2(target_point, current_pose, min_angle, gen_direction)
                        else:
                            if min_range > self.obstacle_hit_threshold:
                                self.leave_flag = False
                                self.complete_turn = False
                        #self.get_logger().info("Following boundary at right")
                else:
                    v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
                # Go back to business as usual once leave point is reached
                
            #self.get_logger().info("v, omega: {}".format([self.v, self.omega]))
        return v, omega, self.leave_flag, average, goal_direction

    def follow_obstacle_boundary_left(self, scan_angles, scan_ranges):
        v = 0.7
        omega = 0.0        
        min_angle = 75
        max_angle = 115

        left_side_ranges = [scan_ranges[i] for i in range(len(scan_ranges)) if min_angle < scan_angles[i] < max_angle]
        left_side_ranges = [x for x in left_side_ranges if np.isfinite(x)]
    
        if len(left_side_ranges) > 0:
            average_left = np.sum(left_side_ranges) / len(left_side_ranges)
            if average_left > 0.7:
                omega = average_left*0.18
            elif average_left < 0.5:
                omega = -(1/average_left)*0.05
            else:
                omega = 0.0
        else:
            average_left = 0.6
    
        return v, omega, average_left
    
    def follow_obstacle_boundary_right(self, scan_angles, scan_ranges):
        v = 0.7
        omega = 0.0        
        min_angle = 255
        max_angle = 285

        right_side_ranges = [scan_ranges[i] for i in range(len(scan_ranges)) if min_angle < scan_angles[i] < max_angle]
        right_side_ranges = [x for x in right_side_ranges if np.isfinite(x)]
    
        if len(right_side_ranges) > 0:
            average_right = np.sum(right_side_ranges) / len(right_side_ranges)
            if average_right > 0.7:
                omega = -average_right*0.18
            elif average_right < 0.5:
                omega = (1/average_right)*0.05
            else:
                omega = 0.0
        else:
            average_right = 0.6
    
        return v, omega, average_right

    def turn_left(self):
        # v = 0.0
        # omega = 0.0
        min_range, min_angle = self.relative_polar_hit_point
        if min_range > 2*self.obstacle_hit_threshold/3:
            # Obstacle still 'far' away --> go faster
            v = 0.7
        else:
            # Obstacle close --> go slower
            v = 0.6
        if min_angle > 325 or min_angle < 25:
            # Obstacle straight ahead --> sharp turn
            omega = 0.4
        elif 260 <= min_angle <= 325:
            omega = 0.4
            if 260 <= min_angle <= 280:
                self.complete_turn = True
        return v,omega
    
    def turn_right(self):
        # v = 0.0
        # omega = 0.0
        min_range, min_angle = self.relative_polar_hit_point
        if min_range > 2*self.obstacle_hit_threshold/3:
            # Obstacle still 'far' away --> go faster
            v = 0.7
        else:
            # Obstacle close --> go slower
            v = 0.6
        if min_angle > 325 or min_angle < 25:
            # Obstacle straight ahead --> sharp turn
            omega = -0.4
        elif 25 <= min_angle <= 100:
            omega = -0.4 # compensate steering in opposite direction
            if 80 <= min_angle <= 100:
                self.complete_turn = True
        return v,omega


    # def find_leave_point(self, target_point, current_pose, hit_angle):
    #     # Leave obstacle once goal is at same angle as before obstacle
    #     x, y, theta = current_pose  # CHECK current pose in right format?
    #     tx, ty = target_point
    #     alpha = - theta + math.atan2(ty-y, tx-x)
    #     leave_flag = False
    #     goal_direction = hit_angle - 180 - np.degrees(alpha)
    #     if abs(hit_angle - 180 - np.degrees(alpha)) < 5: # precision of 5 deg
    #         self.obstacle_hit = False
    #         #self.get_logger().info("Leave point reached")
    #         leave_flag = True
        return leave_flag, goal_direction
    
    def find_leave_point2(self, target_point, current_pose, hit_angle, gen_direction):
        # Leave obstacle once goal is at same angle as before obstacle
        x, y, theta = current_pose  # CHECK current pose in right format?
        tx, ty = target_point
        alpha = math.atan2(ty-y, tx-x)
        
        leave_flag = self.leave_flag
        if abs(gen_direction - np.degrees(alpha)) < 5: # precision of 5 deg
            self.obstacle_hit = False
            #self.get_logger().info("Leave point reached")
            leave_flag = True
        return leave_flag, np.degrees(alpha)
        