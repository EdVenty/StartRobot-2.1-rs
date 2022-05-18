import time
import cv2
from newRapi import GPIORobotDuoMotorsDuoServosWithMPUandLaser, arduino_map
from robot import Interval, Timer, EventState

robot = GPIORobotDuoMotorsDuoServosWithMPUandLaser(
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
mpu_interval = Interval()
punch_timer = Timer()
robot.add_waiter(mpu_interval)
robot.add_waiter(punch_timer)

speed_reducer = False

servo1_angle = 0
servo2_angle = 0

@quality_boost_vibration_timer.callback
def on_exceed():
    robot.vibrate()

last_accel = robot.mpu6050().accelerometer_y

@mpu_interval.callback
def on_mpu():
    global last_accel, speed_reducer
    y = robot.mpu6050().accelerometer_y
    delta = last_accel - y
    
    if delta > 1:
        print(y, last_accel)
        punch_timer.start(2)
        speed_reducer = True
        robot.vibrate(80, 80)
    last_accel = y

@punch_timer.callback
def on_punch_end():
    global speed_reducer
    robot.vibrate()
    speed_reducer = False

mpu_interval.start(0.5)

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

        robot.move((thrustY - thrustX) / (speed_reducer + 1), (thrustY + thrustX) / (speed_reducer + 1))

        servo1_angle = arduino_map(-rightThumbstickX, -1, 1, -90, 90)
        servo2_angle = arduino_map(-rightThumbstickY, -1, 1, -30, 30)
        robot.serv(servo1_angle)
        robot.serv2(servo2_angle)

        if leftTriggerValue != last_ltr_state:
            last_ltr_state = leftTriggerValue
            robot.laser(arduino_map(leftTriggerValue, 0, 1, 0, 255))
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
            robot.move(255, 255)
        elif e.key == 'A':
            robot.move(255, -255)
        elif e.key == 'S':
            robot.move(-255, -255)
        elif e.key == 'D':
            robot.move(-255, 255)
        elif e.key == "Up":
            servo2_angle -= 1
        elif e.key == "Down":
            servo2_angle += 1
        elif e.key == 'Left':
            servo1_angle += 1
        elif e.key == 'Right':
            servo1_angle -= 1
        robot.serv(servo1_angle)
        robot.serv2(servo2_angle)
    elif e is not None and e.state == 0:
        robot.move(0, 0)

    if time.time() - last_event_timestamp > 0.2:
        robot.move(0, 0, 1)

    r, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    robot.set_frame(frame, quality = 60 if quality_boosted else 5)
    robot.tick()
    quality_boost_vibration_timer.tick()