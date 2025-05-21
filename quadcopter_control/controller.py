import numpy as np

from scipy.spatial.transform import Rotation as R


class QuadcopterController:
    def __init__(self, dt, mass, Ixx, Iyy, Izz, max_tilt_angle, max_accel_xy, max_vel_xy, max_ascent_rate, l, min_motor_thrust, max_motor_thrust):

        self.dt = dt

        # drone physical parameters
        self.max_tilt_angle = max_tilt_angle
        self.max_ascent_rate = max_ascent_rate

        self.mass = mass
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz

        self.max_motor_thrust = max_motor_thrust  #
        self.min_motor_thrust = min_motor_thrust  # 0.26477955 / 4 * 0.9
        self.max_accel_xy = max_accel_xy
        self.max_vel_xy = max_vel_xy
        self.l = l

        # controller errors
        self.integrated_altitude_error = 0.0
        self.set_gains()

    def set_gains(self, kp_pqr=0.0, kp_bank=0.0, kp_pos_z=0.0, kp_vel_z=0.0,
                  ki_pos_z=0.0, kp_pos_xy=0.0, kp_yaw=0.0, kp_vel_xy=0.0,
                  kappa=1.0):
        self.kp_pqr = kp_pqr
        self.kp_bank = kp_bank
        self.kp_pos_z = kp_pos_z
        self.kp_vel_z = kp_vel_z
        self.ki_pos_z = ki_pos_z
        self.kp_pos_xy = kp_pos_xy
        self.kp_yaw = kp_yaw
        self.kp_vel_xy = kp_vel_xy
        self.kappa = kappa

    def body_rate_control(self, pqr_cmd, pqr):
        """
        Calculate a desired 3-axis moment given a desired and current body rate

        Args:
          pqr_cmd: desired body rates [rad/s]
          pqr: current or estimated body rates [rad/s]

        Returns:
          A 3x1 numpy array containing the desired moments for each of the 3 axes.
        """

        I = np.diag([self.Ixx, self.Iyy, self.Izz])
        pqr_err = (pqr_cmd - pqr)
        moment_cmd = self.kp_pqr * (I @ pqr_err)

        return moment_cmd

    def altitude_control(self, pos_z_cmd, vel_z_cmd, pos_z, vel_z, attitude, accel_z_cmd, dt):
        """
        Calculate desired quad thrust based on altitude setpoint, actual altitude,
          vertical velocity setpoint, actual vertical velocity, and a vertical
          acceleration feed-forward command

        Args:
          pos_z_cmd, vel_z_cmd: desired vertical position and velocity in NED [m]
          pos_z, vel_z: current vertical position and velocity in NED [m]
          accel_z_cmd: feed-forward vertical acceleration in NED [m/s2]
          dt: the time step of the measurements [seconds]

        Returns:
          A collective thrust command in [N]
        """

        R = attitude.as_matrix()

        pos_z_err = pos_z_cmd - pos_z
        vel_z_err = vel_z_cmd - vel_z
        self.integrated_altitude_error += pos_z_err * dt

        b_z = R[2, 2]

        p_term = self.kp_pos_z * pos_z_err
        d_term = self.kp_vel_z * vel_z_err
        i_term = self.ki_pos_z * self.integrated_altitude_error

        u1_bar = p_term + i_term + d_term + accel_z_cmd

        acc = (u1_bar + 9.81) / b_z

        acc = np.clip(
            acc,
            -self.max_ascent_rate / dt,
            +self.max_ascent_rate / dt
        )

        return self.mass * acc

    def roll_pitch_control(self, accel_cmd, attitude, thrust):
        pqr_cmd = np.zeros(3)
        R = attitude.as_matrix()
        if thrust == 0:
            return pqr_cmd

        coll_accel = -thrust / self.mass

        bx_cmd = np.clip(
            accel_cmd[0] / coll_accel,
            -self.max_tilt_angle,
            +self.max_tilt_angle
        )
        by_cmd = np.clip(
            accel_cmd[1] / coll_accel,
            -self.max_tilt_angle,
            +self.max_tilt_angle
        )

        bx_err = (bx_cmd + R[0, 2])
        by_err = (by_cmd + R[1, 2])

        bx_p_term = self.kp_bank * bx_err
        by_p_term = self.kp_bank * by_err

        r11 = +R[1, 0] / R[2, 2]
        r12 = -R[0, 0] / R[2, 2]
        r21 = +R[1, 1] / R[2, 2]
        r22 = -R[0, 1] / R[2, 2]

        pqr_cmd[0] = r11 * bx_p_term + r12 * by_p_term
        pqr_cmd[1] = r21 * bx_p_term + r22 * by_p_term

        return pqr_cmd

    def lateral_position_control(self, pos_cmd, vel_cmd, pos, vel, accel_cmd_ff):
        """
        Calculate a desired horizontal acceleration based on
          desired lateral position/velocity/acceleration and current pose

        Args:
          pos_cmd: desired position, in NED [m]
          vel_cmd: desired velocity, in NED [m/s]
          pos: current position, NED [m]
          vel: current velocity, NED [m/s]
          accel_cmd_ff: feed-forward acceleration, NED [m/s2]

        Returns:
          A 3x1 numpy array with desired horizontal accelerations.
            the Z component should be 0
        """

        accel_cmd_ff[2] = 0
        vel_cmd[2] = 0
        pos_cmd[2] = pos[2]

        accel_cmd = accel_cmd_ff

        pos_err = pos_cmd - pos
        vel_err = vel_cmd - vel

        accel = self.kp_pos_xy * pos_err + self.kp_vel_xy * vel_err + accel_cmd_ff
        accel_cmd[0] = accel[0]
        accel_cmd[1] = accel[1]

        return accel_cmd

    def yaw_control(self, yaw_cmd, yaw):
        """
        Calculate a desired yaw rate to control yaw to yawCmd

        Args:
          yaw_cmd: commanded yaw [rad]
          yaw: current yaw [rad]

        Returns:
          a desired yaw rate [rad/s]
        """

        yaw_rate_cmd = self.kp_yaw * (yaw_cmd - yaw)
        return yaw_rate_cmd

    def generate_motor_commands(self, coll_thrust_cmd, moment_cmd):
        """
        Convert a desired 3-axis moment and collective thrust command to
          individual motor thrust commands

        Args:
          coll_thrust_cmd: desired collective thrust [N]
          moment_cmd: desired rotation moment about each axis [N m]

        Returns:
          a 4-vec with motor commands
        """

        l = self.l / np.sqrt(2.0)

        t1 = moment_cmd[0] / l
        t2 = moment_cmd[1] / l
        t3 = -moment_cmd[2] / self.kappa
        t4 = coll_thrust_cmd

        thrust = np.array([
            (+t1 + t2 + t3 + t4) / 4.0,  # front left  - f1
            (-t1 + t2 - t3 + t4) / 4.0,  # front right - f2
            (+t1 - t2 - t3 + t4) / 4.0,  # rear left   - f4
            (-t1 - t2 + t3 + t4) / 4.0,  # rear right  - f3
        ])

        return thrust

    def run_control(self, t_pos, t_vel, t_acc, t_att, est_pos, est_vel, est_omega, est_att):

        thrust = self.altitude_control(
            t_pos[2],
            t_vel[2],
            est_pos[2],
            est_vel[2],
            est_att,
            t_acc[2],
            self.dt
        )

        thrust_margin = 0.1 * (self.max_motor_thrust - self.min_motor_thrust)
        thrust = np.clip(
            thrust,
            (self.min_motor_thrust + thrust_margin) * 4.0,
            (self.max_motor_thrust - thrust_margin) * 4.0
        )

        des_acc = self.lateral_position_control(
            np.copy(t_pos),
            np.copy(t_vel),
            np.copy(est_pos),
            np.copy(est_vel),
            np.copy(t_acc)
        )

        des_acc = np.clip(
            des_acc,
            -self.max_accel_xy,
            +self.max_accel_xy,
        )

        des_omega = self.roll_pitch_control(des_acc, est_att, thrust)

        traj_euler_angles = t_att.as_euler('zyx', degrees=False)
        traj_yaw = traj_euler_angles[0]
        est_yaw = est_att.as_euler('zyx', degrees=False)[0]
        des_omega[2] = self.yaw_control(traj_yaw, est_yaw)

        des_moment = self.body_rate_control(des_omega, est_omega)

        motors = self.generate_motor_commands(thrust, des_moment)

        return np.clip(motors, self.min_motor_thrust, self.max_motor_thrust)
