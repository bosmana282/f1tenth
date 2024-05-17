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
        self.hit_flag = False
        self.steering_point = 10.0, 0.0
        self.steering_index = 0

        self.v = 0.0
        self.omega = 0.0

    def calculate_reaction(self, target_point, current_pose, relative_polar_hit_point, scan_angles, scan_ranges, arrival_flag, obstacle_hit2, gen_direction):
        goal_direction = None
        leave_cond = None
        goal_range = None
        obst_angles = None
        average = None
        difference = None
        v = self.v
        omega = self.omega
        min_range,min_angle = relative_polar_hit_point
        hit_point = None
        hit_point_rel = None
        motion_control = "None"
        #steering_point = None

        self.relative_polar_hit_point = relative_polar_hit_point

        if not min_range == None and min_range < self.tangent_hit_threshold and (0.0 <= min_angle < 120 or 240 <= min_angle < 360):
            obst_angles, obst_ranges = self.find_hit_point(scan_ranges, scan_angles, relative_polar_hit_point)
            hit_point, hit_point_rel = self.select_hit_point(obst_angles, obst_ranges, current_pose, target_point)
            v, omega = self.motion_controller.calculate_steering(self.steering_point, current_pose)
            motion_control = "Toward hit point"
            if np.linalg.norm(np.array([current_pose[0], current_pose[1]]) - np.array([self.steering_point[0],self.steering_point[1]])) < 0.15 and 0.0 <= min_angle < 100:
                v, omega, average = self.follow_obstacle_boundary_left(scan_angles, scan_ranges, min_angle)
                motion_control = "Follow boundary left"
                if not self.leave_flag:
                    self.leave_flag, goal_direction, goal_range, difference, leave_cond = self.find_leave_point(target_point, current_pose, scan_ranges, scan_angles)
                    if min_range > self.obstacle_hit_threshold:
                        self.leave_flag = False
                        self.complete_turn = False
            elif np.linalg.norm(np.array([current_pose[0], current_pose[1]]) - np.array([self.steering_point[0],self.steering_point[1]])) < 0.15 and 260 <= min_angle < 360:
                v, omega, average = self.follow_obstacle_boundary_right(scan_angles, scan_ranges, min_angle)
                motion_control = "Follow boundary right"
                if not self.leave_flag:
                    self.leave_flag, goal_direction, goal_range, difference, leave_cond = self.find_leave_point(target_point, current_pose, scan_ranges, scan_angles)
                    if min_range > self.obstacle_hit_threshold:
                        self.leave_flag = False
                        self.complete_turn = False
        else:
            v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            motion_control = "Default: nothing detected yet"
            self.leave_flag = False
            self.hit_flag = False
            self.steering_index = 0

        if self.leave_flag:
            v, omega = self.motion_controller.calculate_steering(target_point, current_pose)
            motion_control = "Default: leave"  
            if min_range > self.obstacle_hit_threshold:
                self.leave_flag = False
                self.hit_flag = False

        return v, omega, self.leave_flag, self.steering_index, obst_angles, hit_point, hit_point_rel, motion_control, self.steering_point, goal_direction, goal_range, difference, leave_cond

    def find_hit_point(self, scan_ranges, scan_angles, rel_pol_hit):
        min_range, min_angle = rel_pol_hit
        obst_angles = None
        obst_ranges = None
    
        if min_range < self.tangent_hit_threshold and len(scan_ranges) > 0:
            min_index = np.argmin(scan_ranges)

            obst_angles = []
            obst_ranges = []
        
            # Start from the minimum index and move backward to find the start of the obstacle segment
            i = min_index
            while scan_ranges[i] < self.tangent_hit_threshold:
                obst_angles.insert(0, scan_angles[i])
                obst_ranges.insert(0, scan_ranges[i])
                i = (i - 1) % len(scan_ranges)
                if i == min_index:
                    break

            # Move forward to find the end of the obstacle segment
            i = (min_index + 1) % len(scan_ranges)
            while scan_ranges[i] < self.tangent_hit_threshold:
                obst_angles.append(scan_angles[i])
                obst_ranges.append(scan_ranges[i])
                i = (i + 1) % len(scan_ranges)
                if i == min_index:
                    break
    
        return obst_angles, obst_ranges


    def select_hit_point(self, obst_angles, obst_ranges, pose, target):
        tx, ty = target
        x_p, y_p, theta = pose
        hit_point = 0.0,0.0
        hit_point_rel = 0.0,0.0
        hit_point_margin = 0.0,0.0
        #steering_point = 0.0,0.0
        
        if not obst_angles == None and not self.hit_flag:
            hit_left_polar = obst_angles[-1], obst_ranges[-1] # Most left value --> last value
            hit_right_polar = obst_angles[0], obst_ranges[0] # Most right value --> first value

            hit_left_polar_margin = obst_angles[-1]+5, obst_ranges[-1]
            hit_right_polar_margin = obst_angles[0]-5, obst_ranges[0]

            x_o_l = x_p + hit_left_polar[1]*math.cos(np.radians(hit_left_polar[0])+theta)
            y_o_l = y_p + hit_left_polar[1]*math.sin(np.radians(hit_left_polar[0])+theta)
            x_o_r = x_p + hit_right_polar[1]*math.cos(np.radians(hit_right_polar[0])+theta)
            y_o_r = y_p + hit_right_polar[1]*math.sin(np.radians(hit_right_polar[0])+theta)

            # dist_left = math.dist([x_o_l, y_o_l],[tx, ty])
            # dist_right = math.dist([x_o_r, y_o_r],[tx, ty])

            # min_dist = min(dist_left, dist_right)
            min_dist = min(obst_ranges[0], obst_ranges[-1])
            if min_dist == obst_ranges[-1]:
                hit_point = x_o_l, y_o_l
                hit_point_rel = hit_left_polar
                hit_point_margin = hit_left_polar_margin
            elif min_dist == obst_ranges[0]:
                hit_point = x_o_r, y_o_r
                hit_point_rel = hit_right_polar
                hit_point_margin = hit_right_polar_margin

            steering_point_x = x_p + hit_point_margin[1]*math.cos(np.radians(hit_point_margin[0])+theta)
            steering_point_y = y_p + hit_point_margin[1]*math.sin(np.radians(hit_point_margin[0])+theta)

            # Inconsistency lidar data
            if self.steering_index == 0:
                self.steering_point = steering_point_x, steering_point_y 
            elif np.linalg.norm(np.array([steering_point_x, steering_point_y]) - np.array([pose[0],pose[1]])) > np.linalg.norm(np.array([self.steering_point[0], self.steering_point[1]]) - np.array([pose[0],pose[1]])):
                self.steering_point = steering_point_x, steering_point_y
            if self.steering_index < 4:
                self.steering_index = self.steering_index + 1
            else:
                self.hit_flag = True

        return hit_point, hit_point_rel

    def follow_obstacle_boundary_left(self, scan_angles, scan_ranges, hit_angle):
        """
        If needed, new 'if' to react earlier when min_angle > 100, even tough average is below 0.7
        """
        v = 0.7
        omega = 0.0        
        min_angle = 75
        max_angle = 105

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

        obstacle_dir = 2 - abs(math.sin(np.radians(hit_angle)))

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
        goal_range = 0.0
        difference = 174
        x, y, theta = current_pose  
        tx, ty = target_point
        alpha = - theta + math.atan2(ty-y, tx-x) % 2*np.pi
        leave_condition = "nothing"
        goal_range_L = 0.0
        goal_range_R = 0.0

        scan_angles = np.array(scan_angles)
        # Compute the absolute differences between alpha and each angle in scan_angles
        diff = [np.abs(scan_angles[i] - np.degrees(alpha)) for i in range(len(scan_angles))]
        # Find the index of the minimum difference
        if len(diff) > 0:
            index = np.argmin(diff)
            difference = min(diff)
            goal_range = scan_ranges[index]
            goal_range_L = scan_ranges[index+7]
            goal_range_R = scan_ranges[index-7]
            leave_condition = "1"

            if 0 <=  alpha < 180:
                leave_condition = "2"
                if goal_range > self.obstacle_hit_threshold and goal_range_L > self.obstacle_hit_threshold: 
                    self.obstacle_hit = False
                    leave_flag = True
                    leave_condition = "3"

            if 180 <=  alpha < 360:
                leave_condition = "4"
                if goal_range > self.obstacle_hit_threshold and goal_range_R > self.obstacle_hit_threshold: 
                    self.obstacle_hit = False
                    leave_flag = True
                    leave_condition = "5"

        return leave_flag, np.degrees(alpha), goal_range, difference, leave_condition
