import numpy as np
import time
import asyncio
import asyncio

from intrepid_environment.simulator import Simulator
from scipy.spatial.transform import Rotation as R


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

    entity = await reset()

    async def sim_step(motors):

        motors = np.array(motors) * NEWTON_TO_RPM
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

    for i in range(0, 10):
        await sim_step([0.1, 0.1, 0.1, 0.1])

asyncio.run(main())
