import asyncio
import time

from intrepid_environment.simulator import Simulator


async def main():

    sim = Simulator()
    await sim.connect()

    await sim.rpc("session.restart")
    entity = await sim.rpc("map.spawn_uav", {
        "robot_id": 0,
        "position": {
            "x": 0,
            "y": 0,
            "z": 0,
        },
    })
    await sim.rpc("session.run")
    # time.sleep(0.1)
    print(entity)

    await asyncio.gather(
        sim.step(),
        sim.rpc(f"object_{entity}.actuator_control", [0, 0, 0, 0]),
        sim.rpc("script.eval", {
                "code": """{
                    position = sim.object.position(ARGS),
                    attitude = sim.object.rotation_quat(ARGS),
                    velocity = sim.object.linear_velocity(ARGS),
                    omega = sim.object.angular_velocity(ARGS),
                    acceleration = sim.object.acceleration(ARGS),
                }""",
                "args": entity,
                }),
    )

    await sim.rpc("session.restart")
    entity = await sim.rpc("map.spawn_uav", {
        "robot_id": 0,
        "position": {
            "x": 0,
            "y": 0,
            "z": 0,
        },
    })
    await sim.rpc("session.run")
    # time.sleep(0.1)
    print(entity)

    await asyncio.gather(
        sim.step(),
        sim.rpc(f"object_{entity}.actuator_control", [0, 0, 0, 0]),
        sim.rpc("script.eval", {
            "code": """{
                position = sim.object.position(ARGS),
                attitude = sim.object.rotation_quat(ARGS),
                velocity = sim.object.linear_velocity(ARGS),
                omega = sim.object.angular_velocity(ARGS),
                acceleration = sim.object.acceleration(ARGS),
            }""",
            "args": entity,
        }),
    )


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
