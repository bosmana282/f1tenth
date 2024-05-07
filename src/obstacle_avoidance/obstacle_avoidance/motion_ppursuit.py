from .motion_control_interface import MotionControlInterface
import numpy as np
import math
from tf_transformations import euler_from_quaternion

class PurePursuitOA(MotionControlInterface):
    def __init__(self):
        self.lookahead_distance = 0.3
        self.wheelbase = 0.3
        self.speed = 1.5 

    def calculate_steering(self, target_point, current_pose):
        # Implement pure pursuit steering calculation
        v = self.speed
        x, y, theta = current_pose
        tx, ty = target_point
        l = self.wheelbase
        l_d = self.lookahead_distance
        alpha = math.atan2(ty - y, tx - x) - theta
        L = np.linalg.norm(np.array(target_point) - np.array([x, y])) # lateral error

        delta = math.atan2(2*l*math.sin(alpha),l_d)
        
        if delta > 0:
             omega = min(delta, 0.36) # 0.36 rad = max turning radius
        else:
             omega = max(delta, -0.36) # 0.36 rad = max turning radius
        angles = np.degrees(theta), np.degrees(alpha), np.degrees(delta)
        # self.get_logger().info("Steering angle test: {}".format(angles))
        
        return v, omega    
