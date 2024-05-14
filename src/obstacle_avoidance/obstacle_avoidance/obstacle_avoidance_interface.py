from abc import ABC, abstractmethod

class ObstacleAvoidanceInterface(ABC):
    @abstractmethod
    def calculate_reaction(self, target_point, current_pose, relative_polar_hit_point, scan_angles, scan_ranges, arrival_flag, obstacle_hit, gen_direct):
        pass
