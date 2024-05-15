from rclpy.node import Node
import math
import numpy as np

from .motion_control_interface import MotionControlInterface
from .obstacle_avoidance_interface import ObstacleAvoidanceInterface

class TangentBugOA(ObstacleAvoidanceInterface):
    def __init__(self, motion_controller: MotionControlInterface):

        self.motion_controller = motion_controller

        # Initialize obstacle avoidance parameters
        self.obstacle_hit_threshold = 1.2
        self.tangent_hit_threshold = 2.0 
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
        obst_angles = None
        average = None
        v = self.v
        omega = self.omega
        min_range,min_angle = relative_polar_hit_point
        hit_point = None
        hit_point_rel = None

        self.obstacle_hit = obstacle_hit # Use for reset
        self.arrival_flag = arrival_flag
        self.relative_polar_hit_point = relative_polar_hit_point

        if obstacle_hit and not self.obstacle_hit:
            self.leave_flag = False

        if not self.arrival_flag:
            if self.leave_flag:
                # Go back to business as usual once leave point is reached
                v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            elif not min_range == None and min_range < self.tangent_hit_threshold:
                obst_angles, obst_ranges = self.find_hit_point(scan_ranges, scan_angles, relative_polar_hit_point)
                hit_point, hit_point_rel = self.select_hit_point(obst_angles, obst_ranges, current_pose, target_point)
                v, omega = self.motion_controller.calculate_steering(hit_point, current_pose)
                if min_range < self.obstacle_hit_threshold and 0.0 <= min_angle < 180:
                    v, omega, average = self.follow_obstacle_boundary_right(scan_angles, scan_ranges, min_angle)
                    if not self.leave_flag:
                            self.leave_flag, goal_direction = self.find_leave_point(target_point, current_pose, scan_ranges, scan_angles)
                    else:
                        if min_range > self.obstacle_hit_threshold:
                            self.leave_flag = False
                            self.complete_turn = False
                elif min_range < self.obstacle_hit_threshold and 180 <= min_angle < 360:
                    v, omega, average = self.follow_obstacle_boundary_left(scan_angles, scan_ranges, min_angle)
                    if not self.leave_flag:
                        self.leave_flag, goal_direction = self.find_leave_point(target_point, current_pose, scan_ranges, scan_angles)
                    else:
                        if min_range > self.obstacle_hit_threshold:
                            self.leave_flag = False
                            self.complete_turn = False
                else:
                    v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            elif not self.obstacle_hit:
                # Default motion control
                v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
        return v, omega, self.leave_flag, average, obst_angles, hit_point, hit_point_rel

    def find_hit_point2(self, scan_ranges, scan_angles, rel_pol_hit):
        min_range, min_angle = rel_pol_hit
        obst_angles = None
        obst_ranges = None
        if min_range < self.tangent_hit_threshold and len(scan_ranges) > 0:
            index = np.argmin(scan_ranges)
            obst_angles = [scan_angles[index]] # [min_angle]
            obst_ranges = [scan_ranges[index]] # [min_angle]
            # obst_angles = [scan_angles[i] for i in range(len(scan_angles)) if abs(scan_ranges[i] - scan_ranges[i-1]) < 0.01]

            i = index
            while i + 1 < len(scan_ranges) and abs(scan_ranges[i] - scan_ranges[i+1]) < 0.01:
                i += 1 
                obst_angles.append(scan_angles[i])
                obst_ranges.append(scan_ranges[i])

            i = index 
            while i - 1 > -len(scan_ranges) and abs(scan_ranges[i] - scan_ranges[i-1]) < 0.01:
                i -= 1 
                obst_angles.insert(0,scan_angles[i])
                obst_ranges.append(scan_ranges[i])

        return obst_angles, obst_ranges

    def find_hit_point(self, scan_ranges, scan_angles, rel_pol_hit):
        min_range, min_angle = rel_pol_hit
        obst_angles = None
        obst_ranges = None
    
        if min_range < self.tangent_hit_threshold and len(scan_ranges) > 0:
            # Find the index of the minimum range value
            min_index = np.argmin(scan_ranges)
        
            # Initialize lists to store angles and ranges for the obstacle edge
            obst_angles = []
            obst_ranges = []
        
            # Start from the minimum index and move backward to find the start of the obstacle segment
            i = min_index
            while scan_ranges[i] < self.tangent_hit_threshold:
                obst_angles.insert(0, scan_angles[i])
                obst_ranges.insert(0, scan_ranges[i])
                i = (i - 1) % len(scan_ranges)
        
            # Move forward to find the end of the obstacle segment
            i = (min_index + 1) % len(scan_ranges)
            while scan_ranges[i] < self.tangent_hit_threshold:
                obst_angles.append(scan_angles[i])
                obst_ranges.append(scan_ranges[i])
                i = (i + 1) % len(scan_ranges)
    
        return obst_angles, obst_ranges


    def select_hit_point(self, obst_angles, obst_ranges, pose, target):
        tx, ty = target
        x_p, y_p, theta = pose
        hit_point = 0.0,0.0
        hit_point_rel = 0.0,0.0
        
        if not obst_angles == None:
            hit_left_polar = obst_angles[0], obst_ranges[0]
            hit_right_polar = obst_angles[-1], obst_ranges[-1]

            x_o_l = x_p + hit_left_polar[1]*math.cos(hit_left_polar[0]+theta)
            y_o_l = y_p + hit_left_polar[1]*math.sin(hit_left_polar[0]+theta)
            x_o_r = x_p + hit_right_polar[1]*math.cos(hit_right_polar[0]+theta)
            y_o_r = y_p + hit_right_polar[1]*math.sin(hit_right_polar[0]+theta)

            dist_left = math.dist([x_o_l, y_o_l],[tx, ty])
            dist_right = math.dist([x_o_r, y_o_r],[tx, ty])

            min_dist = min(dist_left, dist_right)
            if min_dist == dist_left:
                hit_point = x_o_l, y_o_l
                hit_point_rel = hit_left_polar
            elif min_dist == dist_right:
                hit_point = x_o_r, y_o_r
                hit_point_rel = hit_right_polar

        return hit_point, hit_point_rel

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

    def find_leave_point(self, target_point, current_pose, scan_ranges, scan_angles):
        """ 
        Main difference with Bug2 algorithm: robot leaves the obstacle once the direction toward
        the goal is free of any obstacles, while Bug2 only left the obstacle when the initial heading
        direction could be continued. 
        """
        # Leave obstacle once goal dircetion is free
        leave_flag = self.leave_flag
        x, y, theta = current_pose  
        tx, ty = target_point
        alpha = math.atan2(ty-y, tx-x)

        scan_angles = np.array(scan_angles)
        # Compute the absolute differences between alpha and each angle in scan_angles
        diff = [np.abs(scan_angles[i] - alpha) for i in range(len(scan_angles))]
        # Find the index of the minimum difference
        if len(diff) > 0:
            index = np.argmin(diff)
        
            if scan_ranges[index] > self.obstacle_hit_threshold: # margin of 5 deg
                self.obstacle_hit = False
                leave_flag = True
        return leave_flag, np.degrees(alpha)
