import abc
from dataclasses import dataclass
from enum import Enum
import subprocess
from typing import List, Set, Type

import cv2
# import RPi.GPIO as GPIO
import pigpio
import time
import robot

GPIO_PWM_SERVO = 13
GPIO_PWM_MOTOR = 12
GPIO_PIN_CW = 17
GPIO_PIN_CCW = 27

WORK_TIME = 10
DUTY_CYCLE = 50
FREQUENCY = 100

CLOCKWISE = True
COUNTERCLOCKWISE = False

pi = pigpio.pi()


def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def constrain(x, out_min, out_max):
    return out_min if x < out_min else out_max if x > out_max else x

class ComponentNotFoundException(Exception): pass

class Inverter:
    def __init__(self, *inverted) -> None:
        self._inverted = set(inverted)
        self.pi = None

    def set_pi(self, pi):
        self.pi = pi

    def invert(self, pin):
        self._inverted.add(pin)

    def normalize(self, pin):
        self._inverted.remove(pin)

    def write(self, pin, value):
        self.pi.write(pin, 1 - value if pin in self._inverted else value)

    def set_PWM_dutycycle(self, pin, value):
        p = constrain(value, 0, 255)
        self.pi.set_PWM_dutycycle(pin, 255 - p if pin in self._inverted else p)


class Tag(Enum):
    OUTPUT = 0
    INPUT = 1
    LED = 2
    MOTOR = 3
    LEFT = 4
    RIGHT = 5
    REMOTE = 6
    GATE = 7
    BOARD = 8
    MOTORCONTROL = 9
    BUTTON = 10
    BUZZER = 11
    SERVO = 12
    ACCELEROMETER = 13
    GYROSCOPE = 14
    MAGNETOMETER = 15
    SENSOR = 16

class AbstractComponent(abc.ABC):
    __tags__: Set[Tag] = set()
    @abc.abstractmethod
    def __init__(self):
        self.started = False

    @abc.abstractmethod
    def start(self):
        ...

    @abc.abstractmethod
    def stop(self):
        ...

    def set_inverter(self, inverter: Inverter):
        self.inverter = inverter

    def cleanup(self):
        ...


class AbstractOutputComponent(AbstractComponent):
    __tags__ = {Tag.OUTPUT}
    @abc.abstractmethod
    def write(self):
        ...


class AbstractInputComponent(AbstractComponent):
    __tags__ = {Tag.INPUT}
    @abc.abstractmethod
    def read(self):
        ...


class AbstractInputOutputComponent(AbstractInputComponent, AbstractOutputComponent):
    __tags__ = {Tag.INPUT, Tag.OUTPUT}

class AbstractRemoteComponent(AbstractComponent):
    __tags__ = {Tag.REMOTE}

class AbstractGateComponent(AbstractComponent):
    __tags__ = {Tag.GATE}

class BoardComponent(AbstractComponent):
    def __init__(self):
        self.__tags__.add(Tag.BOARD)
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def _read_voltage(self, device: str):
        return float(subprocess.run(["vcgencmd", "measure_volts", device]).stdout.decode('ascii').removeprefix('volt=').removesuffix('V'))

    def read_throttle(self):
        return int(subprocess.run(["vcgencmd", "get_throttled"]).stdout.decode('ascii').removeprefix('throttled='), 16)
    
    def read_voltage_core(self):
        return self._read_voltage('core')

    def read_voltage_sdram_i(self):
        return self._read_voltage('sdram_i')

    def read_voltage_sdram_p(self):
        return self._read_voltage('sdram_p')

    def read_voltage_sdram_c(self):
        return self._read_voltage('sdram_c')

class Motor(AbstractOutputComponent):
    def __init__(self, pwm_pin: int, pin_CW: int, pin_CCW: int, frequency: int = 100, enable_timer=False, delay: int = 0.1) -> None:
        self.__tags__.add(Tag.MOTOR)
        self.pin_pwm = pwm_pin
        self.freq = frequency
        self.pwm = None
        self.pin_CW = pin_CW
        self.pin_CCW = pin_CCW
        self.started = False
        self.timer_enabled = enable_timer
        self.timer = 0
        self.delay = delay

    def start(self):
        pi.set_mode(self.pin_CW, pigpio.OUTPUT)
        pi.set_mode(self.pin_CCW, pigpio.OUTPUT)
        pi.set_mode(self.pin_pwm, pigpio.OUTPUT)
        self.inverter.set_PWM_dutycycle(self.pin_pwm, 0)
        self.started = True

    def write(self, force: int, clockwise=True):
        assert self.started, "Start servo before using this method."
        if not self.timer_enabled or time.time() > self.timer:
            if clockwise:
                self.inverter.write(self.pin_CCW, 0)
                self.inverter.write(self.pin_CW, 1)
            else:
                self.inverter.write(self.pin_CW, 0)
                self.inverter.write(self.pin_CCW, 1)
            # arduino_map(force, 0, 100, 0, 255)
            self.inverter.set_PWM_dutycycle(self.pin_pwm, constrain(force, 0, 255))
            self.timer = time.time() + self.delay

    def stop(self):
        assert self.started, "Start servo before using this method."
        # self.pwm.stop()
        self.write(0)
        self.started = False

    def cleanup(self):
        # GPIO.cleanup()
        ...


class Button(AbstractInputComponent):
    def __init__(self, pin: int, invert: bool = False) -> None:
        self.__tags__.add(Tag.BUTTON)
        self.pin = pin
        self.invert = invert
        self.started = False

    def start(self):
        pi.set_mode(self.pin, pigpio.INPUT)
        pi.set_pull_up_down(self.pin, pigpio.PUD_UP)
        self.started = True

    def read(self):
        return (0 if self.invert else 1) if pi.read(self.pin) else (1 if self.invert else 0)

    def stop(self):
        self.started = False

    def cleanup(self):
        pass


class Buzzer(AbstractOutputComponent):
    def __init__(self, pin: int):
        self.__tags__.add(Tag.BUZZER)
        self.pin = pin
        self.started = False

    def start(self):
        pi.set_mode(self.pin, pigpio.OUTPUT)
        self.started = True

    def write(self, duty: int):
        self.inverter.set_PWM_dutycycle(self.pin, constrain(duty, 0, 255))

    def set_frequency(self, freq: int):
        pi.set_PWM_frequency(self.pin, freq)

    def stop(self):
        self.write(0)
        self.started = False

    def cleanup(self):
        ...


class Led(AbstractOutputComponent):
    def __init__(self, pin: int, invert: bool = False):
        self.__tags__.add(Tag.LED)
        self.pin = pin
        self.invert = invert
        self.started = False

    def start(self):
        pi.set_PWM_dutycycle(self.pin, abs(255 * self.invert))
        pi.set_mode(self.pin, pigpio.OUTPUT)
        self.started = True

    def write(self, duty: int):
        pi.set_PWM_dutycycle(self.pin, abs(
            255 * constrain(self.invert - duty, 0, 255)))

    def stop(self):
        self.write(0)
        self.started = False

    def cleanup(self):
        ...


class RGBLed(AbstractOutputComponent):
    def __init__(self, pin_red: int, pin_green: int, pin_blue: int, invert: bool = False):
        self.__tags__.add(Tag.LED)
        self.red = Led(pin_red, invert)
        self.green = Led(pin_green, invert)
        self.blue = Led(pin_blue, invert)
        self.started = False

    def start(self):
        self.red.start()
        self.green.start()
        self.blue.start()
        self.started = True

    def write(self, duty_red: int, duty_green: int, duty_blue: int):
        self.red.write(duty_red)
        self.green.write(duty_green)
        self.blue.write(duty_blue)

    def stop(self):
        self.write(0, 0, 0)
        self.started = False

    def cleanup(self):
        self.red.cleanup()
        self.green.cleanup()
        self.blue.cleanup()


class Servo(AbstractOutputComponent):
    def __init__(self, pwm_pin: int, frequency: int = 100) -> None:
        self.__tags__.add(Tag.SERVO)
        self.pin = pwm_pin
        self.freq = frequency
        self.angle = 0
        self.pwm = None
        self.started = False

    def start(self):
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.pin, GPIO.OUT)
        # self.pwm = GPIO.PWM(self.pin, self.freq)
        # self.pwm =
        # self.pwm.start(0)
        self.started = True

    def write(self, angle: int):
        assert self.started, "Start servo before using this method."
        self.angle = angle + 90
        duty = constrain(arduino_map(self.angle, 0, 180, 500, 2500), 0, 2500)
        # duty = float(self.angle) / 10.0 + 2.5
        # self.pwm.ChangeDutyCycle(duty)
        pi.set_servo_pulsewidth(self.pin, duty)

    def stop(self):
        assert self.started, "Start servo before using this method."
        self.write(0)
        # self.pwm.stop()
        self.started = False

    def cleanup(self):
        # GPIO.cleanup()
        ...

@dataclass
class MPUData:
    gyroscope_x: float
    gyroscope_y: float
    gyroscope_z: float
    accelerometer_x: float
    accelerometer_y: float
    accelerometer_z: float

class MPU6050(AbstractInputComponent):
    def __init__(self, addr_pwr_mgmt_1: int = 0x6B, addr_smplrt_div: int = 0x19, addr_config: int = 0x1A, addr_gyro_config: int = 0x1B, addr_int_enable: int = 0x38, addr_accel_xout_h: int = 0x3B, addr_accel_yout_h: int = 0x3D, addr_accel_zout_h: int = 0x3F, addr_gyro_xout_h: int = 0x43, addr_gyro_yout_h: int = 0x45, addr_gyro_zout_h: int = 0x47, device_address: int = 0x68, bus: int = 1):
        self.__tags__.add(Tag.ACCELEROMETER)
        self.__tags__.add(Tag.GYROSCOPE)
        self.__tags__.add(Tag.SENSOR)
        self.addr_pwr_mgmt_1 = addr_pwr_mgmt_1
        self.addr_smplrt_div = addr_smplrt_div
        self.addr_config = addr_config
        self.addr_gyro_config = addr_gyro_config
        self.addr_int_enable = addr_int_enable
        self.addr_accel_xout_h = addr_accel_xout_h
        self.addr_accel_yout_h = addr_accel_yout_h
        self.addr_accel_zout_h = addr_accel_zout_h
        self.addr_gyro_xout_h = addr_gyro_xout_h
        self.addr_gyro_yout_h = addr_gyro_yout_h
        self.addr_gyro_zout_h = addr_gyro_zout_h
        self.bus = bus
        self.device_id = None
        self.device_address = device_address
        self.started = False

    def initialize_mpu(self):
        self.device_id = pi.i2c_open(self.bus, self.device_address)
        pi.i2c_write_byte_data(self.device_id, self.addr_smplrt_div, 7)
        pi.i2c_write_byte_data(self.device_id, self.addr_pwr_mgmt_1, 1)
        pi.i2c_write_byte_data(self.device_id, self.addr_config, 0)
        pi.i2c_write_byte_data(self.device_id, self.addr_gyro_config, 24)
        pi.i2c_write_byte_data(self.device_id, self.addr_int_enable, 1)

    def close_mpu(self):
        assert self.started, "MPU6050 was not started yet."
        pi.i2c_close(self.device_id)

    def start(self):
        self.initialize_mpu()
        self.started = True

    def read_raw_data(self, addr: int) -> float:
        high = pi.i2c_read_byte_data(self.device_id, addr)
        low = pi.i2c_read_byte_data(self.device_id, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

    def read(self) -> MPUData:
        return MPUData(
            accelerometer_x=self.read_raw_data(self.addr_accel_xout_h) / 16384.0,
            accelerometer_y=self.read_raw_data(self.addr_accel_yout_h) / 16384.0,
            accelerometer_z=self.read_raw_data(self.addr_accel_zout_h) / 16384.0,
            gyroscope_x=self.read_raw_data(self.addr_gyro_xout_h) / 131.0,
            gyroscope_y=self.read_raw_data(self.addr_gyro_yout_h) / 131.0,
            gyroscope_z=self.read_raw_data(self.addr_gyro_zout_h) / 131.0
        )

    def stop(self):
        assert self.started, "MPU6050 was not started yet."
        self.close_mpu()
        self.started = False

    def cleanup(self):
        ...

class RobotMotor:
    def __init__(self, motor_pwm_pin=12, motor_cw_pin=27, motor_ccw_pin=17) -> None:
        self._motor = Motor(motor_pwm_pin, motor_cw_pin, motor_ccw_pin)
        self._thrust = 0
        self._clockwise = True
        self._start_thrust = 0
        self._required_thrust = 0
        self._acceleration_timer = 0
        self._acceleration_duration = 0
        self._acceleration_enabled = True

    def set_inverter(self, inverter: Inverter):
        self.inverter = inverter
        self._motor.set_inverter(self.inverter)

    def disable_acceleration(self):
        self._acceleration_enabled = False

    def enable_acceleration(self):
        self._acceleration_enabled = True

    def start(self):
        self._motor.start()

    def stop(self):
        self._motor.stop()

    def cleanup(self):
        self._motor.cleanup()

    def move(self, thrust: int, clockwise=True, acceleration_duration=3):
        if self._acceleration_enabled:
            self._clockwise = clockwise
            self._acceleration_duration = acceleration_duration
            self._start_thrust = self._thrust
            self._required_thrust = thrust
            self._acceleration_timer = time.time()
        else:
            self._motor_write(thrust, clockwise)

    def _motor_write(self, thrust: int, clockwise=True):
        self._motor.write(thrust, clockwise)

    def tick(self):
        if self._thrust != self._required_thrust:
            acceleration_elapsed = constrain(
                time.time() - self._acceleration_timer, 0, self._acceleration_duration)
            self._thrust = arduino_map(
                acceleration_elapsed, 0, self._acceleration_duration, self._thrust, self._required_thrust)
            self._motor_write(abs(self._thrust), self._clockwise)

class ConstructableRobot(robot.Robot):
    def __init__(self, components: List[Type[AbstractComponent]] = [], settings: robot.RobotSettings = robot.RobotSettings(), inverter: Inverter = Inverter()) -> None:
        super().__init__(settings)
        self.invert = inverter
        self.invert.set_pi(pi)
        self.components = components
        for component in self.components:
            component.set_inverter(self.invert)
            component.start()

    def get_component_by_tags(self, *tags: Set[Tag]):
        for component in self.components:
            if component.__tags__ & tags == tags:
                return component
        raise ComponentNotFoundException(f"Component with specified tags ({', '.join(tags)}) was not found.")


class GPIORobotApi(robot.Robot):
    def __init__(self, motor_pwm_pin=12, motor_cw_pin=27, motor_ccw_pin=17, servo_pin=13, button_pin=22, buzzer_pin=18, led_pin_red=2, led_pin_green=3, led_pin_blue=4, invert_led=True, invert_button=True, inverter = Inverter()):
        super().__init__()
        self.invert = inverter
        self.invert.set_pi(pi)
        self._servo = Servo(servo_pin)
        self._motor = RobotMotor(motor_pwm_pin, motor_cw_pin, motor_ccw_pin)
        # self._button = Button(button_pin, invert_button)
        # self._buzzer = Buzzer(buzzer_pin)
        # self._led = RGBLed(led_pin_red, led_pin_green,
                        #    led_pin_blue, invert=invert_led)
        self._board = BoardComponent()

        self._servo.set_inverter(self.invert)
        self._motor.set_inverter(self.invert)
        # self._button.set_inverter(self.invert)
        # self._buzzer.set_inverter(self.invert)
        # self._led.set_inverter(self.invert)
        self._board.set_inverter(self.invert)
        
        self._servo.start()
        self._motor.start()
        # self._button.start()
        # self._buzzer.start()
        # self._led.start()
        self._board.start()

        self._fps_counter = 0
        self._fps_timer = time.time() + 1
        self.fps = -1

    def __del__(self):
        self.stop()

    def stop(self):
        self._servo.stop()
        self._motor.stop()
        # self._button.stop()
        # self._buzzer.stop()
        # self._led.stop()
        self._board.stop()

    def cleanup(self):
        self._servo.cleanup()
        self._motor.cleanup()
        # self._button.cleanup()
        # self._buzzer.cleanup()
        # self._led.cleanup()
        self._board.cleanup()

    def show_fps(self, frame, x=20, y=20, pattern="FPS: {fps}", font_color=(255, 255, 255), font_size=2):
        self.text_to_frame(frame, pattern.format(
            fps=self.fps), x, y, font_color, font_size)

    def move(self, thrust: int, clockwise=True, acceleration_duration=3):
        self._motor.move(thrust, clockwise, acceleration_duration)

    def tick(self):
        self._fps_counter += 1
        if time.time() > self._fps_timer:
            self.fps = self._fps_counter
            self._fps_counter = 0
            self._fps_timer = time.time()
        self._motor.tick()
        return super().tick()

    def serv(self, angle: int):
        self._servo.write(angle)

    # def button(self):
    #     return self._button.read()

    # def beep(self):
    #     self._buzzer.write(255)
    #     time.sleep(0.1)
    #     self._buzzer.write(0)

    # def light(self, red, green, blue):
    #     self._led.write(red, green, blue)

    # def buzzer(self, value, freq=30):
    #     self._buzzer.set_frequency(freq)
    #     self._buzzer.write(value)

    def set_frame(self, frame, flip_vertical=False, flip_horizontal=False, quality=30):
        if flip_vertical:
            frame = cv2.flip(frame, 0)
        if flip_horizontal:
            frame = cv2.flip(frame, 1)
        return super().set_frame(frame, quality)

    def read_throttle(self):
        return self._board.read_throttle()
    
    def read_voltage_core(self):
        return self._board.read_voltage_core()

    def read_voltage_sdram_i(self):
        return self._board.read_voltage_sdram_i()

    def read_voltage_sdram_p(self):
        return self._board.read_voltage_sdram_p()

    def read_voltage_sdram_c(self):
        return self._board.read_voltage_sdram_c()


class GPIORobotDuoMotors(GPIORobotApi):
    def __init__(self, motor_pwm_pin=12, motor_cw_pin=27, motor_ccw_pin=17, motor2_pwm_pin=12, motor2_cw_pin=27, motor2_ccw_pin=17, servo_pin=13, button_pin=22, buzzer_pin=18, led_pin_red=2, led_pin_green=3, led_pin_blue=4, invert_led=True, invert_button=True, acceleration_enabled = True, inverter = Inverter()):
        super().__init__(motor_pwm_pin, motor_cw_pin, motor_ccw_pin, servo_pin, button_pin, buzzer_pin, led_pin_red, led_pin_green,
                         led_pin_blue, invert_led, invert_button, inverter)
        self._motor2 = RobotMotor(
            motor2_pwm_pin, motor2_cw_pin, motor2_ccw_pin)
        self._motor2.set_inverter(inverter)
        self._motor2.start()
        if not acceleration_enabled:
            self._motor.disable_acceleration()
            self._motor2.disable_acceleration()

    def move(self, thrust1: int, thrust2: int, acceleration_duration=3):
        super().move(abs(thrust1), thrust1 >= 0, acceleration_duration)
        self._motor2.move(abs(thrust2), thrust2 >= 0,
                          acceleration_duration=acceleration_duration)

    def tick(self):
        self._motor2.tick()
        return super().tick()

    def stop(self):
        super().stop()
        self._motor2.stop()

    def cleanup(self):
        super().cleanup()
        self._motor2.cleanup()


class GPIORobotDuoMotorsDuoServos(GPIORobotDuoMotors):
    def __init__(self, motor_pwm_pin=12, motor_cw_pin=27, motor_ccw_pin=17, motor2_pwm_pin=12, motor2_cw_pin=27, motor2_ccw_pin=17, servo_pin=13, servo2_pin=4, button_pin=22, buzzer_pin=18, led_pin_red=2, led_pin_green=3, led_pin_blue=4, invert_led=True, invert_button=True, acceleration_enabled=True, inverter=Inverter()):
        super().__init__(motor_pwm_pin, motor_cw_pin, motor_ccw_pin, motor2_pwm_pin, motor2_cw_pin, motor2_ccw_pin, servo_pin, button_pin, buzzer_pin, led_pin_red, led_pin_green, led_pin_blue, invert_led, invert_button, acceleration_enabled, inverter)
        self._servo2 = Servo(servo2_pin)
        self._servo2.set_inverter(inverter)
        self._servo2.start()

    def stop(self):
        super().stop()
        self._servo2.stop()

    def serv2(self, angle: int):
        self._servo2.write(angle)

    def cleanup(self):
        super().cleanup()
        self._servo2.cleanup()

class GPIORobotDuoMotorsDuoServosWithMPUandLaser(GPIORobotDuoMotorsDuoServos):
    def __init__(self, motor_pwm_pin=12, motor_cw_pin=27, motor_ccw_pin=17, motor2_pwm_pin=12, motor2_cw_pin=27, motor2_ccw_pin=17, servo_pin=13, servo2_pin=4, button_pin=22, buzzer_pin=18, led_pin_red=2, led_pin_green=3, led_pin_blue=4, invert_led=True, invert_button=True, acceleration_enabled=True, inverter=Inverter()):
        super().__init__(motor_pwm_pin, motor_cw_pin, motor_ccw_pin, motor2_pwm_pin, motor2_cw_pin, motor2_ccw_pin, servo_pin, servo2_pin, button_pin, buzzer_pin, led_pin_red, led_pin_green, led_pin_blue, invert_led, invert_button, acceleration_enabled, inverter)
        self._mpu6050 = MPU6050()
        self._mpu6050.set_inverter(self.invert)
        self._mpu6050.start()
        self._laser = Led(26)
        self._laser.set_inverter(self.invert)
        self._laser.start()

    def laser(self, duty: int = 0):
        self._laser.write(duty)

    def mpu6050(self) -> MPUData:
        return self._mpu6050.read()

    def stop(self):
        super().stop()
        self._mpu6050.stop()

    def cleanup(self):
        super().cleanup()
        self._mpu6050.cleanup()

class GPIORobotDuoMotorsDuoServosWithLaser(GPIORobotDuoMotorsDuoServos):
    def __init__(self, motor_pwm_pin=12, motor_cw_pin=27, motor_ccw_pin=17, motor2_pwm_pin=12, motor2_cw_pin=27, motor2_ccw_pin=17, servo_pin=13, servo2_pin=4, button_pin=22, buzzer_pin=18, led_pin_red=2, led_pin_green=3, led_pin_blue=4, invert_led=True, invert_button=True, acceleration_enabled=True, inverter=Inverter()):
        super().__init__(motor_pwm_pin, motor_cw_pin, motor_ccw_pin, motor2_pwm_pin, motor2_cw_pin, motor2_ccw_pin, servo_pin, servo2_pin, button_pin, buzzer_pin, led_pin_red, led_pin_green, led_pin_blue, invert_led, invert_button, acceleration_enabled, inverter)
        self._laser = Led(26)
        self._laser.set_inverter(self.invert)
        self._laser.start()

    def laser(self, duty: int = 0):
        self._laser.write(duty)

    def stop(self):
        super().stop()

    def cleanup(self):
        super().cleanup()

if __name__ == "__main__":
    # m = MPU6050()
    # m.start()
    # print(m.read())
    # m.stop()
    # m.cleanup()
    l = Led(26)
    l.write(255)
    time.sleep(1)
    l.write(0)
    time.sleep(1)
    l.stop()
    l.cleanup()