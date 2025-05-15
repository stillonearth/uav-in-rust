import numpy as np
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from collections import deque

MAX_TRAJECTORY_POINTS = 1000


@dataclass
class TrajectoryPoint:
    time: float
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    omega: np.ndarray     # [ωx, ωy, ωz]
    acceleration: np.ndarray     # [ax, ay, az]
    attitude: R


class Trajectory:
    def __init__(self, filename=None, config=None):
        self.traj = deque(maxlen=MAX_TRAJECTORY_POINTS)
        self._cur_traj_point = 0

    def clear(self):
        self.traj.clear()

    def set_log_file(self, filename):
        self._log_filename = filename
        if self._log_file:
            self._log_file.close()
            self._log_file = None
        if self._log_filename:
            self._log_file = open(self._log_filename, 'w')

    def add_point(self, traj_pt):
        self.traj.append(traj_pt)

    def next_trajectory_point(self, time) -> TrajectoryPoint:
        if not self.traj:
            return TrajectoryPoint(0.0, np.zeros(3), np.zeros(3), R.from_quat([0, 0, 0, 1]), np.zeros(3))

        for i in reversed(range(len(self.traj))):
            if self.traj[i].time < time:
                self._cur_traj_point = i
                return self.traj[i]
        self._cur_traj_point = len(self.traj) - 1
        return self.traj[-1]
