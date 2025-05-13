import asyncio
import numpy as np

from centrifuge import Client


async def main():
    client = Client("ws://localhost:9120/connection/websocket")
    await client.connect()
    await client.rpc("session.restart", None)
    entity = await client.rpc("map.spawn_urdf", {
        "robot_id": 1,
        "position": {"x": 0, "y": 0},
        "rotation": {"yz": 0, "zx": 0, "xy": 0},
        "urdf_path": "flamingo_edu/urdf/Edu_v4.urdf",
        "mesh_dir": "assets/flamingo_edu/urdf",
    },)
    print(await client.rpc("object_{object}.position".format(object=entity.data), None))
    await client.rpc("object_{object}.despawn".format(object=entity.data), None)
    entity = await client.rpc("map.spawn_urdf", {
        "robot_id": 2,
        "position": {"x": 0, "y": 0},
        "rotation": {"yz": 0, "zx": 0, "xy": 0},
        "urdf_path": "flamingo_edu/urdf/Edu_v4.urdf",
        "mesh_dir": "assets/flamingo_edu/urdf",
    },)
    print(await client.rpc("object_{object}.urdf_state".format(object=entity.data), None))
    print(await client.rpc("object_{object}.position".format(object=entity.data), None))

    control = np.random.uniform(-5, 5, 8)
    await client.rpc("object_{object}.actuator_control".format(object=entity.data), control.tolist())
    await client.rpc("session.step".format(object=entity.data), None)
    print(await client.rpc("object_{object}.urdf_state".format(object=entity.data), None))
    print(await client.rpc("object_{object}.position".format(object=entity.data), None))


loop = asyncio.get_event_loop()
loop.run_until_complete(main())
