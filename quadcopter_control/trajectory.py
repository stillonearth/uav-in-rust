import numpy as np
import quaternion

from dataclasses import dataclass
from collections import deque

MAX_TRAJECTORY_POINTS = 5000


@dataclass
class TrajectoryPoint:
    time: float
    position: np.ndarray      # [x, y, z]
    velocity: np.ndarray      # [vx, vy, vz]
    omega: np.ndarray         # [ωx, ωy, ωz]
    acceleration: np.ndarray  # [ax, ay, az]
    attitude: np.quaternion


class Trajectory:
    def __init__(self):
        self.points = deque(maxlen=MAX_TRAJECTORY_POINTS)

    def add_point(self, traj_pt: TrajectoryPoint):
        self.points.append(traj_pt)

    def clear(self):
        self.points.clear()
