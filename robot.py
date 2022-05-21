import abc
import time
from typing import Dict, Generic, List, TypeVar, Union, Type, Callable
from dataclasses import dataclass
import json
from queue import Empty, Full, Queue
from enum import Enum
import cv2
from loguru import logger
from threading import Thread
import websockets
import asyncio
import sys
from turbojpeg import TurboJPEG

LOCAL_SERVICE_KEYWORD = 'service'
logger.remove()
logger.add(sys.stdout, level='INFO')

class EventState:
    KeyDown = 0
    KeyUp = 1

@dataclass
class Vibration:
    left_trigger: float = 0
    right_trigger: float = 0
    left_motor: float = 0
    right_motor: float = 0

@dataclass
class ControlEvent:
    key: str = None
    state: int = None
    gamepad: Union[Dict[str, Union[bool, float]], None] = None

    vibration: Union[Vibration, None] = None

@dataclass
class RobotSettings:
    video_compress_turbo_jpeg_enabled = True
    turbo_jpeg_lib_path = None

class AbstractTimeWaiter(abc.ABC):
    @abc.abstractmethod
    def tick(self):
        """Abstract tick. Refreshes some states, calls methods, etc. Calls every loop iteration.
        """
    
    @abc.abstractmethod
    def start(self, time_: int):
        """Start waiter with specified time.

        Args:
            time_ (int): Specified time.
        """

    @abc.abstractmethod
    def stop(self):
        """Stop waiter. Resets time.
        """

class Timer(AbstractTimeWaiter):
    def __init__(self) -> None:
        self._time_set = 0
        self._exceed_in = 0
        self._callbacks = []
        self.working = False

    def callback(self, func):
        """Decorator to register a new callback which will be called when time will exceed.

        Args:
            func (Function): Callback function.

        Returns:
            __wrapper__ (this is decorator, so it must to return wrapper. Don't interact with it in your code.)
        """
        if func not in self._callbacks:
            self._callbacks.append(func)
        def __wrapper__(*args, **kwargs):
            func(args, kwargs)
        return __wrapper__

    def start(self, time_: int):
        """Start timer for a specified time.

        Args:
            time_ (int): Time for waiting.
        """
        self._time_set = time_
        self._exceed_in = time.time() + time_
        self.working = True

    def stop(self):
        """Stop timer. Resets time waiting.
        """
        self._time_set = 0
        self._exceed_in = 0
        self.working = False

    def tick(self):
        """Check for the time exceeding and call callback if needed. Must be called in all iterations of loop. Perfer to put timer to a robot with Robot.add_waiter().
        """
        if self.working and time.time() > self._exceed_in:
            self.working = False
            for c in self._callbacks:
                c()

class Interval(AbstractTimeWaiter):
    def __init__(self) -> None:
        self._time_set = 0
        self._exceed_in = 0
        self._callbacks = []
        self.working = False

    def callback(self, func):
        """Decorator to register a new callback which will be called when time will exceed.

        Args:
            func (Function): Callback function.

        Returns:
            __wrapper__ (this is decorator, so it must to return wrapper. Don't interact with it in your code.)
        """
        if func not in self._callbacks:
            self._callbacks.append(func)
        def __wrapper__(*args, **kwargs):
            func(args, kwargs)
        return __wrapper__

    def start(self, time_: int):
        """Start timeout for a specified time.

        Args:
            time_ (int): Time for waiting.
        """
        self._time_set = time_
        self._exceed_in = time.time() + time_
        self.working = True

    def stop(self):
        """Stop timeout. Resets time waiting.
        """
        self._time_set = 0
        self._exceed_in = 0
        self.working = False

    def tick(self):
        """Check for the time exceeding and call callback if needed, then refreshes time. Must be called in all iterations of loop. Perfer to put timer to a robot with Robot.add_waiter().
        """
        if self.working and time.time() > self._exceed_in:
            start_call_time = time.time()
            for c in self._callbacks:
                c()
            self._exceed_in = self._time_set - (time.time() - start_call_time)

# TEnum = TypeVar('TEnum', Type[Enum])

# class Stater(Generic[TEnum]):
#     def __init__(self) -> None:
#         self.state: Union[TEnum, None] = None
#         self._callbacks: Dict[TEnum, List[Callable]] = {}

#     def set(self, state: Type[Enum]):
#         self.state = state

#     def clear(self):
#         self.state = None

#     def callback(self, state: Type[Enum], priority: int = 1):
#         def __func_wrapper__(f):
#             new_list = self._callbacks.get(state, [])
#             new_list.insert(priority, f)
#             self._callbacks[state] = new_list
#             def __wrapper__(*args, **kwargs):
#                 f(*args, **kwargs)
#             return __wrapper__
#         return __func_wrapper__

#     def tick(self, *args, **kwargs):
#         callbacks = self._callbacks.get(self.state)
#         if callbacks is not None:
#             for callback in callbacks:
#                 callback(*args, **kwargs)

class Robot:
    """Robot class
    """
    def __init__(self, settings: RobotSettings = RobotSettings()) -> None:
        """Created a new robot object.

        Args:
            settings (RobotSettings): Settings of the robot. Defaults to RobotSettings()
        """
        # "C:/libjpeg-turbo-gcc64/bin/libturbojpeg.dll"
        self.settings = settings
        self.control_events_queue = Queue()
        self.control_transfer_queue = Queue()
        self._control_channel_thread = None
        self._video_channel_thread = None
        self.last_unread_control_event = None
        self.last_frame = None
        self.prev_frame = None
        self.control_ws = None
        self.waiters: List[Type[AbstractTimeWaiter]] = []
        if self.settings.video_compress_turbo_jpeg_enabled:
            self.initialize_turbo_jpeg()

    def initialize_turbo_jpeg(self):
        """Initialize Turbo JPEG library. Must be called before using turbo jpeg dependent methods (set_frame, if you're using turbo jpeg compress algorithm).
        """
        self.jpeg = TurboJPEG(self.settings.turbo_jpeg_lib_path)

    def get_control_event(self) -> ControlEvent:
        """Get last control channel event.

        Returns:
            ControlEvent: Event of user's input from the remove computer.
        """
        return self.last_unread_control_event

    def add_waiter(self, waiter: Type[AbstractTimeWaiter]):
        """Add waiter.

        Args:
            waiter (Type[AbstractTimeWaiter]): Any object of type implements AbstractTimeWaiter interface.
        """
        self.waiters.append(waiter)

    def remove_waiter(self, waiter: Type[AbstractTimeWaiter]):
        """Remove waiter.

        Args:
            waiter (Type[AbstractTimeWaiter]): Any object of type implements AbstractTimeWaiter interface.
        """
        self.waiters.remove(waiter)

    def set_frame(self, frame, turbo_jpeg_quality: int = 30):
        """Set frame that will be sent to a remove computer.

        Args:
            frame (numpy.ndarray): CV2 BGR frame.
            turbo_jpeg_quality (int, optional): Turbo JPEG saving quality. Defaults to 30.
        """
        if self.settings.video_compress_turbo_jpeg_enabled:
            assert self.jpeg, "Turbo JPEG was not initialized. Call initialize_turbo_jpeg() method before."
            self.last_frame = self.jpeg.encode(frame, turbo_jpeg_quality)
        else:
            r, encoded = cv2.imencode('.jpg', frame)
            self.last_frame = bytearray(encoded)

    def vibrate(self, left_motor: int = 0, right_motor: int = 0, left_trigger: int = 0, right_trigger: int = 0):
        """Vibrate gamepad that connected to a remove computer.

        Args:
            left_motor (int, optional): Left motor strength from 0 to 100. Defaults to 0.
            right_motor (int, optional): Right motor strength from 0 to 100. Defaults to 0.
            left_trigger (int, optional): Left trigger motor strength from 0 to 100. Defaults to 0.
            right_trigger (int, optional): Right trigger motor strength from 0 to 100. Defaults to 0.
        """
        try:
            self.control_transfer_queue.put_nowait(ControlEvent(vibration=Vibration(
                left_trigger=left_trigger / 100,
                left_motor=left_motor / 100,
                right_motor=right_motor / 100,
                right_trigger=right_trigger / 100
            )))
        except Full:
            pass

    def tick(self):
        """Call every component tick method and refreshes control event. Must be called every iteration of loop.
        """
        try:
            self.last_unread_control_event = self.control_events_queue.get_nowait()
        except Empty:
            self.last_unread_control_event = None
        for timer in self.waiters:
            timer.tick()

    def on_control_event(self, event: ControlEvent):
        """On control event received.

        Args:
            event (ControlEvent): Event of user's input from the remove computer.
        """
        logger.debug(f"New control event got. (key: {event.key}, state: {event.state}")
        try:
            self.control_events_queue.put_nowait(event)
        except Full:
            logger.warning("Events queue overflowed.")

    async def _control_transfer(self):
        while True:
            try:
                event: ControlEvent = self.control_transfer_queue.get_nowait()
                await self.control_ws.send(json.dumps({
                    "Vibration": {
                        "LeftMotor": event.vibration.left_motor,
                        "RightMotor": event.vibration.right_motor,
                        "LeftTrigger": event.vibration.left_trigger,
                        "RightTrigger": event.vibration.right_trigger
                    }
                }))
            except Empty:
                await asyncio.sleep(0.1)
            except Exception as err:
                logger.error(err)

    async def _control_channel(self):
        while True:
            try:
                async with websockets.connect("ws://localhost:8020") as websocket:
                    self.control_ws = websocket
                    logger.debug("Control channel started.")
                    await websocket.send(LOCAL_SERVICE_KEYWORD)
                    logger.debug("Control channel identified as local service.")
                    asyncio.create_task(self._control_transfer())
                    while True:
                        data = await websocket.recv()
                        try:
                            decoded = json.loads(data)
                        except json.JSONDecodeError:
                            logger.warning(f"Received control event message with broken JSON data: {data}.")
                            continue
                        self.on_control_event(ControlEvent(decoded['Key'], decoded['KeyState'], decoded['Gamepad']))
            except Exception as err:
                logger.error(err)

    async def _video_channel(self):
        while True:
            try:
                async with websockets.connect("ws://localhost:8010") as websocket:
                    logger.debug("Video channel started.")
                    await websocket.send(LOCAL_SERVICE_KEYWORD)
                    logger.debug("Video channel identified as local service.")
                    while True:
                        if self.last_frame != self.prev_frame:
                            self.prev_frame = self.last_frame
                            await websocket.send(self.last_frame)
                        else:
                            await asyncio.sleep(0.1)
            except Exception as err:
                logger.error(err)

    def _run_control_thread(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        asyncio.get_event_loop().run_until_complete(self._control_channel())

    def _start_control_thread(self):
        logger.debug("Starting control channel thread.")
        self._control_channel_thread = Thread(target=self._run_control_thread, name="Control channel thread")
        self._control_channel_thread.start()

    def _run_video_thread(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        asyncio.get_event_loop().run_until_complete(self._video_channel())

    def _start_video_thread(self):
        logger.debug("Starting video channel thread.")
        self._video_channel_thread = Thread(target=self._run_video_thread, name="Video channel thread")
        self._video_channel_thread.start()

    def start(self):
        """Start robot. Must be called before any interaction.
        """
        logger.info("Starting robot.")
        self._start_control_thread()
        self._start_video_thread()

last_ltr_state = None
if __name__ == '__main__':
    robot = Robot()
    robot.start()
    cap = cv2.VideoCapture(1)
    while True:
        event = robot.get_control_event()
        if event is not None:
            if event.gamepad is not None:
                ltX = event.gamepad.get('LeftThumbstickX', 0)
                ltr = event.gamepad.get('LeftTrigger', 0)
                rtr = event.gamepad.get('RightTrigger', 0)
                if (ltr, rtr) != last_ltr_state:
                    robot.vibrate(left_trigger=ltr * 100, right_trigger=rtr * 100)
                    last_ltr_state = (ltr, rtr)
        r, frame = cap.read()
        robot.set_frame(frame)
        robot.tick()
            