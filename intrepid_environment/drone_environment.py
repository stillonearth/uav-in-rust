import numpy as np

from .base import WorldControllerBase, sync_wrap


class DroneComputer:

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.entity = None

    async def spawn(self, world):
        self.entity = await world.rpc('map.spawn_uav', {
            'robot_id': self.robot_id,
            'position': {
                'x': 0,
                'y': 0,
                'z': 0,
            },
        })

    async def despawn(self, world):
        cmd = "object_{object}.despawn".format(object=self.entity)
        await world.rpc(cmd, None)
        self.entity = None

    async def position(self, world):
        state = await world.rpc(
            "object_{object}.position".format(object=self.entity),
            None,
        )
        return np.array([state["x"], state["y"], state["z"]])

    async def rotation_angles(self, world):
        rotation_angles = await world.rpc(
            "object_{object}.rotation_angles".format(object=self.entity),
            None,
        )
        return np.array([rotation_angles["yz"], rotation_angles["zx"], rotation_angles["xy"]])

    async def linear_velocity(self, world):
        linear_velocity = await world.rpc(
            "object_{object}.linear_velocity".format(object=self.entity),
            None,
        )
        return np.array([linear_velocity["x"], linear_velocity["y"], linear_velocity["z"]])

    async def acceleration(self, world):
        acceleration = await world.rpc(
            "object_{object}.acceleration".format(object=self.entity),
            None,
        )
        return np.array([acceleration["x"], acceleration["y"], acceleration["z"]])

    async def angular_velocity(self, world):
        angular_velocity = await world.rpc(
            "object_{object}.angular_velocity".format(object=self.entity),
            None,
        )
        return np.array([angular_velocity["yz"], angular_velocity["zx"], angular_velocity["xy"]])

    async def actuator_control(self, world, args):
        cmd = "object_{object}.actuator_control".format(object=self.entity)
        await world.rpc(cmd, args)


class DroneController(WorldControllerBase):

    def __init__(self, dt_ms):

        super(DroneController, self).__init__(dt_ms=dt_ms)

        self.drone_id = 1
        self.drone_entity = None

    async def on_start(self):

        await self.simulation_restart()
        self.robot = DroneComputer(self.drone_id)
        await self.robot.spawn(self)

    # Drone controller methods

    @sync_wrap
    async def rotation_angles(self):
        result = await self.robot.rotation_angles(self)
        return result

    @sync_wrap
    async def acceleration(self):
        result = await self.robot.acceleration(self)
        return result

    @sync_wrap
    async def linear_velocity(self):
        result = await self.robot.linear_velocity(self)
        return result

    @sync_wrap
    async def angular_velocity(self):
        result = await self.robot.angular_velocity(self)
        return result

    @sync_wrap
    async def control_motors(self, m1, m2, m3, m4):
        result = await self.robot.actuator_control(self, [m1, m2, m3, m4])
        return result

    # Convenience methods

    @sync_wrap
    async def restart(self):
        await self.robot.despawn(self)
        await self.robot.spawn(self)
