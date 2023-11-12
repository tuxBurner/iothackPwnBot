import time
import math
from motor import Motor, motor2040

motor_a = Motor(motor2040.MOTOR_A)
motor_b = Motor(motor2040.MOTOR_B)

normal_speed = 0.5
docking_speed = 0.15
full_speed = 10000
speed = normal_speed

all_motor = [motor_a, motor_b]


def go_forward():
    motor_a.speed(speed)
    motor_b.speed(-speed)
    time.sleep(0.2)
    stop()


def go_backward():
    motor_a.speed(-speed)
    motor_b.speed(speed)
    stop()


def stop():
    motor_a.coast()
    motor_b.coast()


def rotate_right():
    motor_a.speed(-speed)
    motor_b.speed(-(speed))
    stop()


def rotate_left():
    motor_a.speed((speed))
    motor_b.speed(speed)
    stop()


def motor_initalize():
    for motor in all_motor:
        motor.enable()
    time.sleep(1)


def motor_deinitialize():
    for motor in all_motor:
        motor.disable()


def stop_full_speed():
    global speed
    speed = normal_speed


def full_speed():
    global speed
    speed = full_speed

def docking_mode():
    global speed
    speed = docking_speed

def stop_docking_mode():
    global speed
    speed = normal_speed
