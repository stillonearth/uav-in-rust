import asyncio

from centrifuge import Client, SubscriptionEventHandler, PublicationContext


class Simulator:
    def __init__(self):
        self.client = Client("ws://localhost:9120/connection/websocket")
        self.last_tick = 0
        self.next_step = None

    async def connect(self):
        class EventHandler(SubscriptionEventHandler):
            async def on_publication(_, ctx: PublicationContext) -> None:
                if self.next_step:
                    self.next_step.set_result(ctx.pub.data)
                    self.next_step = None
                self.last_tick = ctx.pub.data

        next_step = self.next_step = asyncio.Future()
        sync = self.client.new_subscription("sync", EventHandler())
        await sync.subscribe()
        await self.client.connect()
        return await next_step

    async def rpc(self, method, args=None):
        result = await self.client.rpc(method, args)
        return result.data

    async def step(self):
        if self.next_step:
            self.next_step.set_exception(Exception("cancelled"))
            self.next_step = None

        next_step = self.next_step = asyncio.Future()
        sync = self.client.get_subscription("sync")
        await sync.publish(self.last_tick + 1)
        return await next_step
