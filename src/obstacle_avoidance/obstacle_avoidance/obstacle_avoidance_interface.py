from abc import ABC, abstractmethod

class ObstacleAvoidanceInterface(ABC):
    @abstractmethod
    def calculate_reaction(self, target_point, current_pose):
        pass
