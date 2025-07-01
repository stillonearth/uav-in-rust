import numpy as np
import time
import asyncio

from intrepid_environment.simulator import Simulator
from quadcopter_control.controller import QuadcopterController
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

# Sim Constants

DT_S = 1. / 60

DRONE_MASS = 0.027
Ixx = 1.4e-5
Iyy = 1.4e-5
Izz = 2.17e-5

MIN_THRUST = DRONE_MASS * 9.81 / 4 * 0.8
MAX_THRUST = DRONE_MASS * 9.81 / 4 * 1.5

NEWTON_TO_RPM = 1


async def main():

    sim = Simulator()
    await sim.connect()

    async def reset():
        await sim.rpc("session.restart")
        entity = await sim.rpc("map.spawn_urdf", {
            "robot_id": 0,
            "urdf_path": "crazyflie_urdf/cf2x.urdf",
            "mesh_dir": "assets/crazyflie_urdf",
            "create_colliders_from_visual_shapes": False,
            "create_colliders_from_collision_shapes": True,
            "robot_type": "drone",
            "position": {
                "x": 0,
                "y": 0,
                "z": 0,
            },
        })
        time.sleep(1.0)
        await sim.rpc("session.run")
        return entity

    print("spawning drone")
    entity = await reset()

    async def sim_step(motors):

        motors = np.array(motors)
        motors = np.array([motors[0], motors[2], motors[3], motors[1]])

        (sim_time, _, state) = await asyncio.gather(
            sim.step(),
            sim.rpc(f"object_{entity}.thrust_control", list(motors)),
            sim.rpc("script.eval", {
                    "code": """{
                        uav_state = sim.object.urdf_uav_state(ARGS),
                    }""",
                    "args": entity,
                    }),
        )

        position = np.array([state['uav_state']['position_x'],
                            state['uav_state']['position_y'],
                            state['uav_state']['position_z']], dtype=np.float64)

        velocity = np.array([state['uav_state']['velocity_x'],
                            state['uav_state']['velocity_y'],
                            state['uav_state']['velocity_z']], dtype=np.float64)

        attitude = R.from_euler('xyz', [
            state['uav_state']['roll'],
            state['uav_state']['pitch'],
            state['uav_state']['yaw'],
        ])

        omega = np.array([
            -state['uav_state']['roll_rate'],
            -state['uav_state']['pitch_rate'],
            state['uav_state']['yaw_rate'],
        ])

        return {
            'time': sim_time,
            'position': position,
            'velocity': velocity,
            'attitude': attitude,
            'omega': omega
        }

    quadcopter_controller = QuadcopterController(
        DT_S,
        DRONE_MASS * 0.99,
        Ixx, Iyy, Izz,
        0.7,
        12, 5, 5,
        0.03,
        MIN_THRUST, MAX_THRUST,
    )

    async def tune_controller(
            target={
                "position": np.array([0, 0, 5]),
                "velocity": np.zeros(3),
                "acceleration": np.zeros(3),
                "attitude": R.from_quat([0, 0, 0, 1]),
            },
            kp_pqr=np.array([95.0, 95.0, 6.0]),
            kp_bank=0.0,
            kp_pos_z=0.0,
            kp_vel_z=0.0,
            ki_pos_z=0.0,
            kp_pos_xy=0.0,
            kp_yaw=0.0,
            kp_vel_xy=0.0,
            kappa=1.0,
            n_episodes=500,
    ):

        quadcopter_controller.integrated_altitude_error = 0.0
        quadcopter_controller.set_gains(
            kp_pqr=kp_pqr,
            kp_bank=kp_bank,
            kp_pos_z=kp_pos_z,
            kp_vel_z=kp_vel_z,
            ki_pos_z=ki_pos_z,
            kp_pos_xy=kp_pos_xy,
            kp_yaw=kp_yaw,
            kp_vel_xy=kp_vel_xy,
            kappa=kappa
        )

        times = []
        positions = []
        velocities = []
        attitudes = []
        motors_ = []
        omegas = []

        print("getting initial state")
        state = await sim_step([0.0, 0.0, 0.0, 0.0])

        t = 0
        print("flying")
        for i in tqdm(range(n_episodes)):

            motors = quadcopter_controller.run_control(
                target['position'],
                target['velocity'],
                target['acceleration'],
                target['attitude'],
                state['position'],
                state['velocity'],
                state['omega'],
                state['attitude'],
            )
            motors_.append(motors)

            # accelerations.append(state['acceleration'])
            attitudes.append(state['attitude'].as_quat())
            positions.append(state['position'])
            omegas.append(state['omega'])
            velocities.append(state['velocity'])
            times.append(t)
            state = await sim_step(motors)

            t += DT_S

        return (
            np.array(times),
            np.array(positions),
            np.array(velocities),
            np.array(attitudes),
            np.array(omegas),
            np.array(motors_)
        )

    target = {
        "position": np.array([-5, 3, 10]),
        "velocity": np.zeros(3),
        "acceleration": np.zeros(3),
        "attitude": R.from_quat([0, 0, 0, 1]),
    }

    print("flying drone to -5, 3, 10")
    (times, positions, attitudes) = await tune_controller(
        kp_pqr=np.array([45, 45, 10]),
        ki_pos_z=2,
        kp_pos_z=6,
        kp_vel_z=12,
        kp_pos_xy=8.0,
        kp_vel_xy=8.0,
        kp_yaw=8.0,
        kp_bank=4.0,
        n_episodes=2500,
        target=target
    )

    print("done")

asyncio.run(main())
