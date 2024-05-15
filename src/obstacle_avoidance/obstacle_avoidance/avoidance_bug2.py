from rclpy.node import Node
import math
import numpy as np

from .motion_control_interface import MotionControlInterface
from .obstacle_avoidance_interface import ObstacleAvoidanceInterface

class Bug2OA(ObstacleAvoidanceInterface):
    def __init__(self, motion_controller: MotionControlInterface):
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
                # Default motion control
                v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            elif self.leave_flag:
                # Go back to business as usual once leave point is reached
                v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            else:
                # Appropriate reaction to avoid the obstacle
                if 0.0 <= min_angle < 180:
                    # Turn to right until obstacle at 90 deg, then follow the obstacle
                    if not self.complete_turn:
                        v, omega = self.turn_right()
                    else:
                        v, omega, average = self.follow_obstacle_boundary_left(scan_angles, scan_ranges, min_angle)
                        if not self.leave_flag:
                            self.leave_flag, goal_direction = self.find_leave_point(target_point, current_pose, gen_direction)
                        else:
                            if min_range > self.obstacle_hit_threshold:
                                self.leave_flag = False
                                self.complete_turn = False  
                elif 180 <= min_angle < 360:
                    # Turn to left until obstacle at 270 deg, then follow the obstacle
                    if not self.complete_turn:
                        v, omega = self.turn_left()
                    else:
                        v, omega, average = self.follow_obstacle_boundary_right(scan_angles, scan_ranges, min_angle)
                        if not self.leave_flag:
                            self.leave_flag, goal_direction = self.find_leave_point(target_point, current_pose, gen_direction)
                        else:
                            if min_range > self.obstacle_hit_threshold:
                                self.leave_flag = False
                                self.complete_turn = False
                else:
                    v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
                
        return v, omega, self.leave_flag, average, goal_direction

    def follow_obstacle_boundary_left(self, scan_angles, scan_ranges, hit_angle):
        """
        If needed, new 'if' to react earlier when min_angle > 100, even tough average is below 0.7
        """
        v = 0.7
        omega = 0.0        
        min_angle = 75
        max_angle = 115

        left_side_ranges = [scan_ranges[i] for i in range(len(scan_ranges)) if min_angle < scan_angles[i] < max_angle]
        left_side_ranges = [x for x in left_side_ranges if np.isfinite(x)]
    
        obstacle_dir = 2-abs(math.sin(np.radians(hit_angle)))

        if len(left_side_ranges) > 0:
            average_left = np.sum(left_side_ranges) / len(left_side_ranges)
            if average_left > 0.7:
                omega = average_left*0.18*pow(obstacle_dir,2)
            elif average_left < 0.5:
                omega = -(1/average_left)*0.05*pow(obstacle_dir,2)
            else:
                omega = 0.0
        else:
            average_left = 0.6
    
        return v, omega, average_left
    
    def follow_obstacle_boundary_right(self, scan_angles, scan_ranges, hit_angle):
        """
        If needed, new 'if' to react earlier when min_angle > 100, even tough average is below 0.7
        """
        v = 0.7
        omega = 0.0        
        min_angle = 255
        max_angle = 285

        right_side_ranges = [scan_ranges[i] for i in range(len(scan_ranges)) if min_angle < scan_angles[i] < max_angle]
        right_side_ranges = [x for x in right_side_ranges if np.isfinite(x)]

        obstacle_dir = 2-abs(math.sin(np.radians(hit_angle)))

        if len(right_side_ranges) > 0:
            average_right = np.sum(right_side_ranges) / len(right_side_ranges)
            if average_right > 0.7:
                omega = -average_right*0.18*pow(obstacle_dir,2)
            elif average_right < 0.5:
                omega = (1/average_right)*0.05*pow(obstacle_dir,2)
            else:
                omega = 0.0
        else:
            average_right = 0.6
    
        return v, omega, average_right

    def turn_left(self):
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
            omega = -0.4 
            if 80 <= min_angle <= 100:
                self.complete_turn = True
        return v,omega
    
    def find_leave_point(self, target_point, current_pose, gen_direction):
        # Leave obstacle once goal is at same angle as before obstacle
        leave_flag = self.leave_flag
        x, y, theta = current_pose  
        tx, ty = target_point
        alpha = math.atan2(ty-y, tx-x)
        
        if abs(gen_direction - np.degrees(alpha)) < 5: # margin of 5 deg
            self.obstacle_hit = False
            leave_flag = True
        return leave_flag, np.degrees(alpha)
        