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

        try:
            self.close()

        except:
            ...

    def _mapAngle(self, angle: float) -> float:
        """Maps angle to PWM value."""

        angle = numpy.min((numpy.max((angle, 0)), 180))     # clip to [0, 180]
        return numpy.interp(angle, [0, 180], [2, 12])
    
    def setAngle(self, angle: float = None) -> None:
        """Set the current angle of the servo."""

        angle = self._mapAngle(angle)
        self._servo.ChangeDutyCycle(angle)
        self.angle = angle

    def getAngle(self) -> float:
        """Return internal angle state."""

        return self.angle
    
    def close(self) -> None:
        """Manually close servos."""

        self._servo.close()
