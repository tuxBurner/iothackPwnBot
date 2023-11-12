import time
import math
from motor import Motor, motor2040
from arm import EasyArm


motor_a = Motor(motor2040.MOTOR_A)
motor_b = Motor(motor2040.MOTOR_B)

ARM_PIN_1 = 38
ARM_PIN_2 = 40
arm = EasyArm(ARM_PIN_1, ARM_PIN_2)

speed = 0.5
arm_steps = 1

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

def move_arm_up():
    arm.setHeight(arm.getHeight() + arm_steps)

def move_arm_down():
    arm.setHeight(arm.getHeight() - arm_steps)