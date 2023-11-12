"""Servo interface"""

import RPi.GPIO as GPIO
import pigpio
import numpy


class Servo:
    """Controls a single servo."""

    def __init__(self, pin: int) -> None:
        """Setup servo. Assumes already initialized GPIO."""

        GPIO.setup(pin, GPIO.OUT)
        self.pwm = pigpio.pi()
        self.pwm.set_PWM_frequency(pin, 50)
        self.angle = 0
        self.pin = pin

    def __del__(self) -> None:
        """Close servo connection."""

        self.close()

    def _mapAngle(self, angle: float) -> float:
        """Maps angle to PWM value."""
        return numpy.interp(angle, [0, 180], [500, 2500])

    def setAngle(self, angle: float = None) -> None:
        """Set the current angle of the servo."""

        angle = numpy.min((numpy.max((angle, 0)), 180))
        self.pwm.set_servo_pulsewidth(self.pin, self._mapAngle(angle))
        self.angle = angle

    def getAngle(self) -> float:
        """Return internal angle state."""

        return self.angle

    def close(self) -> None:
        """Manually close servos."""
        self.pwm.set_PWM_dutycycle(self.pin, 0)
        self.pwm.set_PWM_frequency(self.pin, 0)
