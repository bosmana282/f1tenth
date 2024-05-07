import rclpy
from rclpy.node import Node

from motion_control_interface import MotionControlInterface
from obstacle_avoidance_interface import ObstacleAvoidanceInterface

class Bug2OA(Node):
    def __init__(self, motion_controller: MotionControlInterface):
        super().__init__('obstacle_avoidance')
        self.motion_controller = motion_controller

        # Initialize any necessary parameters or subscribers/publishers for obstacle avoidance

    def run_obstacle_avoidance(self):
        # Implement your obstacle avoidance algorithm here
        # This method should continuously check for obstacles and adjust the motion controller accordingly
        v = 1
        omega = 1
        return v, omega 