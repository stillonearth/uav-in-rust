import asyncio
import math
import numpy as np

from centrifuge import Client, SubscriptionEventHandler, PublicationContext

TIMESTEP_MS = 1000
ROBOT_COUNT = 1
SPAWN_RADIUS = 0.9


class WorldControllerBase:
    def __init__(self):
        client = Client("ws://localhost:9120/connection/websocket")

        class EventHandler(SubscriptionEventHandler):
            async def on_publication(_, ctx: PublicationContext) -> None:
                print(f"tick {ctx.pub.data}")
                self._last_tick_received = ctx.pub.data
                self._process_tick(ctx.pub.data)

        sync = client.new_subscription("sync", EventHandler())
        asyncio.ensure_future(sync.subscribe())

        self.client = client
        self._dt_ms = TIMESTEP_MS
        self._last_tick_received = -1
        self._user_task = None

        fut = asyncio.ensure_future(client.connect())
        fut.add_done_callback(lambda _: asyncio.ensure_future(self.on_start()))

    async def on_start(self):
        # implemented by the user
        pass

    async def on_tick(self, _tick):
        # implemented by the user
        pass

    async def rpc(self, method, args):
        result = await self.client.rpc(method, args)
        return result.data

    def _process_tick(self, tick):
        if self._user_task and self._last_tick_received > 0:
            return  # busy

        # send sync
        next_tick = tick + self._dt_ms * 1_000
        sync = self.client.get_subscription("sync")
        asyncio.ensure_future(sync.publish(next_tick))
        # print(f"send sync {next_tick}")

        def on_task_done(_):
            self._user_task = None
            if self._last_tick_received >= next_tick:
                self._process_tick(next_tick)

        self._user_task = asyncio.ensure_future(self.on_tick(tick))
        self._user_task.add_done_callback(on_task_done)


class WorldController(WorldControllerBase):
    async def on_start(self):
        await self.rpc("session.restart", None)
        self.robot = VehicleController(1)
        commands = [
            self.robot.spawn(self),
            self.rpc("session.step", None),
        ]

        await asyncio.gather(*commands)

    async def on_tick(self, _tick):
        commands = [
            self.rpc("session.step", None),
            self.robot.control(self, self.control_arguments),
        ]
        await asyncio.gather(*commands)

    async def step(self, control_arguments):
        commands = [
            self.robot.control(self, control_arguments),
            self.rpc("session.step", None),
        ]
        state = await self.robot.state(self)
        await asyncio.gather(*commands)

        return state


class VehicleController:

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.entity = None

    async def spawn(self, world):
        self.entity = await world.rpc(
            "map.spawn_urdf",
            {
                "robot_id": 1,
                "position": {"x": 0, "y": 0},
                "rotation": {"yz": 0, "zx": 0, "xy": 0},
                "urdf_path": "flamingo_edu/urdf/Edu_v4.urdf",
                "mesh_dir": "assets/flamingo_edu/urdf",
            },
        )

    async def despawn(self, world):
        await world.rpc("object_{object}.despawn".format(object=self.entity), None)
        self.entity = None

    async def state(self, world):
        state = await world.rpc(
            "object_{object}.state_urdf".format(object=self.entity),
            None,
        )
        print(state)
        state_vec = np.array(state["state"])
        return state_vec

    async def control(self, world, control_arguments):
        pass
