import numpy as np
import quaternion

from dataclasses import dataclass
from collections import deque

MAX_TRAJECTORY_POINTS = 5000


@dataclass
class TrajectoryPoint:
    time: float
    position: np.ndarray     # [x, y, z]
    velocity: np.ndarray     # [vx, vy, vz]
    omega: np.ndarray        # [ωx, ωy, ωz]
    acceleration: np.ndarray  # [ax, ay, az]
    attitude: np.quaternion


class Trajectory:
    def __init__(self, filename=None, config=None):
        self.traj = deque(maxlen=MAX_TRAJECTORY_POINTS)
        self._log_file = None
        self._log_filename = None
        self._cur_traj_point = 0
        self.config = config or {}

        if filename:
            self.read_file(filename)

    def read_file(self, filename):
        self.traj.clear()
        self._cur_traj_point = 0

        try:
            with open(filename, 'r') as f:
                for line in f:
                    self._parse_line(line.strip())
        except IOError:
            return False

        if not self.traj:
            initial_pos = self.config.get("Quad.InitialPos", np.zeros(3))
            initial_vel = self.config.get("Quad.InitialVel", np.zeros(3))
            initial_omega = self.config.get("Quad.InitialOmega", np.zeros(3))
            initial_ypr = self.config.get("Quad.InitialYPR", np.zeros(3))

            attitude = quaternion.from_euler_angles(*initial_ypr)
            traj_pt = TrajectoryPoint(
                time=0.0,
                position=initial_pos,
                velocity=initial_vel,
                attitude=attitude,
                omega=initial_omega
            )
            self.traj.append(traj_pt)
        return True

    def _parse_line(self, line):
        if not line or line.startswith(('#', '//')):
            return

        parts = line.split(',')
        if len(parts) != 13:
            return

        try:
            values = list(map(float, parts))
        except ValueError:
            return

        traj_pt = TrajectoryPoint(
            time=values[0],
            position=np.array(values[1:4]),
            velocity=np.array(values[4:7]),
            attitude=quaternion.from_euler_angles(*values[7:10]),
            omega=np.array(values[10:13])
        )
        self.traj.append(traj_pt)

    def add_point(self, traj_pt: TrajectoryPoint):
        self.traj.append(traj_pt)

    def clear(self):
        self.traj.clear()
        self._cur_traj_point = 0
        if self._log_file:
            self._log_file.close()
            self._log_file = None
        if self._log_filename:
            self._log_file = open(self._log_filename, 'w')

    def set_log_file(self, filename):
        self._log_filename = filename
        if self._log_file:
            self._log_file.close()
            self._log_file = None
        if self._log_filename:
            self._log_file = open(self._log_filename, 'w')

    def add_trajectory_point(self, traj_pt):
        self.traj.append(traj_pt)
        if self._log_file:
            self._write_trajectory_point(traj_pt)

    def next_trajectory_point(self, time) -> TrajectoryPoint:
        if not self.traj:
            return TrajectoryPoint(0.0, np.zeros(3), np.zeros(3), np.quaternion(1, 0, 0, 0), np.zeros(3))

        for i in reversed(range(len(self.traj))):
            if self.traj[i].time < time:
                self._cur_traj_point = i
                return self.traj[i]
        self._cur_traj_point = len(self.traj) - 1
        return self.traj[-1]

    def _write_trajectory_point(self, traj_pt):
        if not self._log_file:
            return

        ypr = traj_pt.attitude.as_euler_angles()
        line = (f"{traj_pt.time:.3f},{traj_pt.position[0]:.3f},{traj_pt.position[1]:.3f},{traj_pt.position[2]:.3f},"
                f"{traj_pt.velocity[0]:.3f},{traj_pt.velocity[1]:.3f},{traj_pt.velocity[2]:.3f},"
                f"{ypr[0]:.3f},{ypr[1]:.3f},{ypr[2]:.3f},"
                f"{traj_pt.omega[0]:.3f},{traj_pt.omega[1]:.3f},{traj_pt.omega[2]:.3f}\n")
        self._log_file.write(line)
        self._log_file.flush()

    def __del__(self):
        if self._log_file:
            self._log_file.close()
