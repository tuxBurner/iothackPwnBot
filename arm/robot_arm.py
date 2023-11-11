"""Interface for the robot arm."""

import RPi.GPIO as GPIO
import time
import numpy

## Notes
#   - two servo angles th1, th2
#   - three geometric constants L1, L2 determining the length of the arms
#   - two controllable cartesian coordinates x(th1, th2), y(th1, th2) of tip
#   - two phases of the servo's ph1, ph2 depending on the build geometry
#   - x(th1, th2, L1, L2) = [vec(L1) + vec(L2)]_x = L1 * cos(th1 + ph1) + L2 * cos(th2 + ph2)
#   - y(th1, th2, L1, L2) = [vec(L1) + vec(L2)]_y = L1 * sin(th1 + ph1) + L2 * sin(th2 + ph2)
#   - Goal: search for th1(x, y), th2(x, y) for x = const. and y = const.


class RobotArm:
    """Controls the tip movement of the arm."""

    L1 = 80 # Shoulder to elbow length
    L2 = 80 # Elbow to wrist length

    ph1 = 0 # servo1 phase offset
    ph2 = 0 # servo2 phase offset

    def __init__(self, pin_s1: int, pin_s2: int) -> None:
        """Setup hardware."""

        GPIO.setmode(GPIO.board)

        # servo 1
        GPIO.setup(pin_s1, GPIO.out)
        self._servo1 = GPIO.PWM(pin_s1, 50)
        self._servo1.start(0)

        # servo 1
        GPIO.setup(pin_s2, GPIO.out)
        self._servo2 = GPIO.PWM(pin_s2, 50)
        self._servo2.start(0)

    def __del__(self) -> None:
        """Close connection."""

        self._servo1.stop()
        self._servo2.stop()
        GPIO.cleanup()

    def moveX(self, x: int) -> None:
        """Move the tip horizontally."""

        # solve for x(th1, th2)

    def moveY(self, y: int) -> None:
        """Move the tip vertically."""

        # solve for y(th1, th2)

    def move1(self, angle: float) -> None:
        """Controls the angle of the first servo."""

        self._servo1.ChangeDutyCycle(self._mapAngle(angle))

    def move2(self, angle: float) -> None:
        """Controls the angle of the second servo."""

        self._servo2.ChangeDutyCycle(self._mapAngle(angle))

    def _mapAngle(self, angle: float) -> float:
        """Maps angle to PWM value."""

        return numpy.interp(angle, [0, 180], [2, 12])
    