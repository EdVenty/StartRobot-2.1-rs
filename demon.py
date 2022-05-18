from loguru import logger
from threading import Thread
from websockets import serve
import asyncio

LOCAL_SERVICE_KEYWORD = 'service'


class Connection:
    def __init__(self, ws, local = False) -> None:
        self.ws = ws
        self.local = local

    async def send(self, msg):
        await self.ws.send(msg)

class Channel:
    def __init__(self, id: str, port: int) -> None:
        self.id = id
        self.port = port
        self.ip = "0.0.0.0"
        self.connections: list[Connection] = []
        self._thread = None

    @logger.catch()
    async def run(self):
        @logger.catch()
        async def handler(ws):
            logger.debug(f"New connection received from {ws.remote_address} in channel {self.id}.")
            c = Connection(ws)
            self.connections.append(c)
            try:
                async for message in ws:
                    if message == LOCAL_SERVICE_KEYWORD:
                        logger.debug(f"Connection {ws.remote_address} identified as local service in channel {self.id}.")
                        c.local = True
                    else:
                        comparator = lambda x: not x.local if c.local else lambda x: x.local
                        for client in filter(comparator, self.connections.copy()):
                            try:
                                await client.send(message)
                            except Exception as err:
                                logger.warning(err)
                                if client in self.connections:
                                    self.connections.remove(client)
                                    logger.debug(f"Connection {ws.remote_address} removed in channel {self.id}.")
            except Exception as err:
                logger.error(err)

        while True:
            try:
                async with serve(handler, self.ip, self.port):
                    logger.debug(f"Channel {self.id} started.")
                    await asyncio.Future()
            except Exception as err:
                logger.error(err)
            logger.warning(f"An exception occurred in ws server ({self.id}). Restarting.")

    @logger.catch()
    def start(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        asyncio.get_event_loop().run_until_complete(self.run())

    @logger.catch()
    def start_in_thread(self):
        logger.debug(f"Starting {self.id} channel thread.")
        self._thread = Thread(target=self.start, name=self.id)
        self._thread.start()

class Demon:
    def __init__(self) -> None:
        self._control_thread = None
        self._file_thread = None
        self.control_channel = Channel('control', 8020)
        self.file_channel = Channel('file', 8030)
        self.terminal_channel = Channel('terminal', 8040)
        self.video_channel = Channel('video', 8010)
        self.info_channel = Channel('info', 8050)

    def start(self):
        self.control_channel.start_in_thread()
        self.file_channel.start_in_thread()
        self.terminal_channel.start_in_thread()
        self.video_channel.start_in_thread()
        self.info_channel.start_in_thread()
        logger.info("Demon started.")

if __name__ == "__main__":
    demon = Demon()
    demon.start()