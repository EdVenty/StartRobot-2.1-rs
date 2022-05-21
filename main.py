import time
import cv2
from newRapi import GPIORobotDuoMotorsDuoServosWithLaser,  arduino_map, constrain
from robot import Interval, Timer, EventState
from storage import Storage

class Config(Storage):
    flip_frame: bool = False
    frame_quality: int = 20
    frame_boost_quality: int = 60

config = Config('config.json')
if not config.exists:
    config.save()

config.load()

robot = GPIORobotDuoMotorsDuoServosWithLaser(
    motor_pwm_pin=13,
    motor_cw_pin=6,
    motor_ccw_pin=5,
    motor2_pwm_pin=12,
    motor2_cw_pin=24,
    motor2_ccw_pin=23,
    servo_pin=22,
    servo2_pin=27,
    acceleration_enabled=False
)

robot.start()
robot.move(0, 0)

cap = cv2.VideoCapture(0)

last_event_timestamp = time.time()
last_ltr_state = None
last_vibration_update = 0
quality_boosted = False
quality_boost_vibration_timer = Timer()
servo1_angle = 0
servo2_angle = 0

speed_reducer = False

@quality_boost_vibration_timer.callback
def on_exceed():
    robot.vibrate()

while 1:
    e = robot.get_control_event()
    if e is not None and e.gamepad is not None:
        last_event_timestamp = time.time()
        leftThumbstickX = e.gamepad.get('LeftThumbstickX', 0)
        leftThumbstickY = e.gamepad.get('LeftThumbstickY', 0)
        rightThumbstickX = e.gamepad.get('RightThumbstickX', 0)
        rightThumbstickY = e.gamepad.get('RightThumbstickY', 0)
        leftTriggerValue = e.gamepad.get('LeftTrigger', 0)
        rightShoulderPressed = e.gamepad.get('ButtonRightShoulder', False)

        thrustX = arduino_map(leftThumbstickX, -1, 1, -255 // 2, 255 // 2) if not(-0.1 < leftThumbstickX < 0.1) else 0
        thrustY = arduino_map(leftThumbstickY, -1, 1, -255, 255) if not(-0.1 < leftThumbstickY < 0.1) else 0

        robot.move(thrustY - thrustX - speed_reducer * 100, thrustY + thrustX - speed_reducer * 100)

        servo1_angle = arduino_map(-rightThumbstickX, -1, 1, -90, 90)
        servo2_angle = arduino_map(-rightThumbstickY, -1, 1, -30, 30)
        robot.serv(servo1_angle)
        robot.serv2(servo2_angle)

        if leftTriggerValue != last_ltr_state:
            last_ltr_state = leftTriggerValue
            if leftTriggerValue > 0:
                robot.vibrate(left_trigger=arduino_map(leftTriggerValue, 0, 1, 0, 100))
            else:
                robot.vibrate()
        if quality_boosted != rightShoulderPressed and rightShoulderPressed:
            robot.vibrate(right_motor=50)
            quality_boost_vibration_timer.start(0.3)
        quality_boosted = rightShoulderPressed
    elif e is not None and e.state == 0:
        last_event_timestamp = time.time()
        if e.key == 'W':
            robot.move(200, 200)
        elif e.key == 'A':
            robot.move(150, -150)
        elif e.key == 'S':
            robot.move(-200, -200)
        elif e.key == 'D':
            robot.move(-150, 150)
        if e.key == "Up":
            servo2_angle += 1
        elif e.key == "Down":
            servo2_angle -= 1
        elif e.key == 'Left':
            servo1_angle += 1
        elif e.key == 'Right':
            servo1_angle -= 1
        elif e.key == 'Space':
            robot.laser(255)
            print('laser')
        servo1_angle = constrain(servo1_angle, -80, 80)
        servo2_angle = constrain(servo2_angle, -30, 30)
        robot.serv(servo1_angle)
        robot.serv2(servo2_angle)
    elif e is not None and e.state == 0:
        robot.move(0)
        robot.laser(0)

    if time.time() - last_event_timestamp > 0.2:
        robot.move(0, 0, 1)
        robot.laser(0)

    r, frame = cap.read()
    if config.flip_frame:
        frame = cv2.flip(frame, -1)
    frame = cv2.resize(frame, (640, 480))
    robot.set_frame(frame, quality = config.frame_boost_quality if quality_boosted else config.frame_quality)
    robot.tick()
    quality_boost_vibration_timer.tick()