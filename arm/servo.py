"""Servo interface"""

import RPi.GPIO as GPIO
import numpy


class Servo:
    """Controls a single servo."""

    def __init__(self, pin: int) -> None:
        """Setup servo. Assumes already initialized GPIO."""

        GPIO.setup(pin, GPIO.OUT)
        
        self._servo = GPIO.PWM(pin, 50)
        self._servo.start(0)
        self.angle = 0

    def __del__(self) -> None:
        """Close servo connection."""

        self.close()

    def _mapAngle(self, angle: float) -> float:
        """Maps angle to PWM value."""

        return numpy.interp(angle, [0, 180], [2, 12])
    
    def setAngle(self, angle: float = None) -> None:
        """Set the current angle of the servo."""

        angle = numpy.min((numpy.max((angle, 0)), 180))     # clip to [0, 180]
        self._servo.ChangeDutyCycle(self._mapAngle(angle))
        self.angle = angle

    def getAngle(self) -> float:
        """Return internal angle state."""

        return self.angle
    
    def close(self) -> None:
        """Manually close servos."""

        self._servo.stop()