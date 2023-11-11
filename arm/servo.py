"""Servo interface"""

import RPi.GPIO as GPIO
import numpy
import time


class Servo:
    """Controls a single servo."""

    STEP_SIZE = 5      # Step size in ° for each step.
    STEP_TIME = 0.2    # Step time in s for each step.

    def __init__(self, pin: int) -> None:
        """Setup servo. Assumes already initialized GPIO."""

        GPIO.setup(pin, GPIO.OUT)
        
        self._servo = GPIO.PWM(pin, 50)
        self._servo.start(0)
        self.angle = None

    def __del__(self) -> None:
        """Close servo connection."""

        self.close()

    def _mapAngle(self, angle: float) -> float:
        """Maps angle to PWM value."""

        angle = numpy.min(numpy.max(angle, 0), 180)     # clip to [0, 180]
        return numpy.interp(angle, [0, 180], [2, 12])
    
    def setAngle(self, angle: float = None) -> None:
        """Set the current angle of the servo."""

        if angle is not None:
            angles = numpy.arrange(self.angle, angle, numpy.sign(angle - self.angle) * self.STEP_SIZE)
            for th in angles:
                self._servo.ChangeDutyCycle(self._mapAngle(th))
                time.sleep(self.STEP_TIME)

        self.angle = angle

    def getAngle(self) -> float:
        """Return internal angle state."""

        return self.angle
    
    def close(self) -> None:
        """Manually close servos."""

        self._servo.close()
