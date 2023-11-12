import time
import math
from motor import Motor, motor2040

motor_a = Motor(motor2040.MOTOR_A)
motor_b = Motor(motor2040.MOTOR_B)

speed = 0.5

all_motor = [motor_a, motor_b]


def go_forward():
    motor_a.speed(speed)
    motor_b.speed(-speed)


def go_backward():
    motor_a.speed(-speed)
    motor_b.speed(speed)


def stop():
    motor_a.coast()
    motor_b.coast()


def rotate_right():
    motor_a.speed(-speed)
    motor_b.speed(-(speed))


def rotate_left():
    motor_a.speed((speed))
    motor_b.speed(speed)


def motor_initalize():
    for motor in all_motor:
        motor.enable()
    time.sleep(1)


def motor_deinitialize():
    for motor in all_motor:
        motor.disable()


def stop_full_speed():
    global speed
    speed = 0.5


def full_speed():
    global speed
    speed = 1000
