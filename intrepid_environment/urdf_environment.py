import asyncio
import numpy as np
import gymnasium as gym

from .base import WorldControllerBase, sync_wrap
from scipy.spatial import distance


ROBOT_URDF = "flamingo_edu/urdf/Edu_v4.urdf"
ROBOT_ASSETS = "assets/flamingo_edu/urdf"
OBSERVATION_SIZE = 69
ACTION_SIZE = 8


class URDFRobotController:

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.entity = None

    async def spawn(self, world):
        self.entity = await world.rpc(
            "map.spawn_urdf",
            {
                "robot_id": self.robot_id,
                "position": {"x": 0, "y": 0},
                "rotation": {"yz": 0, "zx": 0, "xy": 0},
                "urdf_path": "flamingo_edu/urdf/Edu_v4.urdf",
                "mesh_dir": "assets/flamingo_edu/urdf",
            },
        )
        # sleep 200ms for si to re-render eveything
        # await asyncio.sleep(500.0 / 1000)

    async def despawn(self, world):
        cmd = "object_{object}.despawn".format(object=self.entity)
        await world.rpc(cmd, None)
        self.entity = None

    async def urdf_state(self, world):
        state = await world.rpc(
            "object_{object}.urdf_state".format(object=self.entity),
            None,
        )
        return np.array(state["state"])

    async def position(self, world):

        state = await world.rpc(
            "object_{object}.position".format(object=self.entity),
            None,
        )
        return np.array([state["x"], state["y"], state["z"]])

    async def actuator_control(self, world, args):
        cmd = "object_{object}.actuator_control".format(object=self.entity)
        await world.rpc(cmd, args)


class WorldController(WorldControllerBase):

    def __init__(self):

        super(WorldController, self).__init__()

        self.robot_id = 1
        self.robot_entity = None

    async def on_start(self):
        await self.session_restart()
        self.robot = URDFRobotController(self.robot_id)

        commands = [
            self.robot.spawn(self),
            self.session_step(),
        ]

        await asyncio.gather(*commands)

    async def step(self, args):
        commands = [
            self.robot.actuator_control(self, args),
            self.session_step(),
        ]
        position = await self.robot.position(self)
        state = await self.robot.urdf_state(self)

        await asyncio.gather(*commands)

        return (state, position)

    async def restart(self):

        commands = [
            self.robot.despawn(self),
            self.robot.spawn(self),
        ]
        await asyncio.gather(*commands)

    # session rpc calls

    async def session_step(self):
        return await self.rpc("session.step", None)

    async def session_restart(self):
        return await self.rpc("session.restart", None)


class URDFGymEnvironment(gym.Env):

    def __init__(self):
        self.world = WorldController()
        self.step_count = 0

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(OBSERVATION_SIZE,), dtype=float
        )
        self.action_space = gym.spaces.Box(
            low=-10.0, high=10.0, shape=(ACTION_SIZE,), dtype=float
        )
        self.prev_distance = 0

    @sync_wrap
    async def step(self, action: np.ndarray):
        action = action.tolist()
        (state, position) = await self.world.step(action)
        dst = distance.euclidean(position, np.zeros(3))
        reward = dst - self.prev_distance
        self.prev_distance = dst

        self.step_count += 1

        is_terminated = (self.step_count % 500) == 0

        return (
            state,
            reward,
            is_terminated,
            is_terminated,
            {},
        )

    @sync_wrap
    async def reset(self, seed=1337):
        await self.world.restart()
        return np.zeros(OBSERVATION_SIZE), {}

    def render():
        None
