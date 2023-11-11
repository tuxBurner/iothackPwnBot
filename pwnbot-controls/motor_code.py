import time
import math
from motor import Motor, motor2040

MOTOR_PINS = [motor2040.MOTOR_A, motor2040.MOTOR_B]
motors = [Motor(pins) for pins in MOTOR_PINS]

def motor_initalize():
    for m in motors:
        m.enable()
    time.sleep(2)

def motor_start():
    # Drive at full positive
    for m in motors:
        m.full_positive()
    time.sleep(2)

def motor_stop():
    for m in motors:
        m.disable()