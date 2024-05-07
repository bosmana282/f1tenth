from abc import ABC, abstractmethod

class MotionControlInterface(ABC):
    @abstractmethod
    def calculate_steering(self, target_point, current_pose):
        pass
