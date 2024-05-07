from .motion_control_interface import MotionControlInterface
import math
from tf_transformations import euler_from_quaternion

class FeedbackControlOA(MotionControlInterface):
    def __init__(self):
        self.wheelbase = 0.3

        # Initialize parameters for feedback control
        self.k_alpha = 2
        self.k_beta = -1
        self.k_rho = 0.15

    def calculate_steering(self, target_point, current_pose):
        # Implement feedback control steering calculation
        """
        Calculates the angular and linear velocity required to follow the target point.
        """
        x, y, theta = current_pose  # CHECK current pose in right format?
        tx, ty = target_point
        rho =  math.sqrt(pow(ty-y,2) + pow(tx-x,2)) # distance from robot to target
        alpha = - theta + math.atan2(ty-y, tx-x)
        beta = - theta - alpha       
        v = self.k_rho*rho
        omega1 = self.k_alpha*alpha + self.k_beta*beta
        
        if omega1 > 0:
             omega = min(omega1, 0.36) # 0.36 rad = max turning radius
        else:
             omega = max(omega1, -0.36) # 0.36 rad = max turning radius
        # self.get_logger().info("Actual [rho, alpha, beta]: {}".format([rho, alpha, beta]))
        # self.get_logger().info("Actual [v, omega1, omega]: {}".format([v, omega1, omega]))
        return v, omega     
