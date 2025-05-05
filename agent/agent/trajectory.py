from geometry_msgs.msg import Pose2D

from dataclasses import dataclass

@dataclass
class Trajectory:
    poses: list[Pose2D]

@dataclass
class TrajectoryEvaluation:
    progress: float
    cost: float

    def __init__(self, progress: float = 0.0, cost: float = 0.0):
        self.progress = progress
        self.cost = cost