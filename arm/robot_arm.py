"""Interface for the robot arm."""

import RPi.GPIO as GPIO
from . import servo

## Notes
#   - two servo angles th1, th2
#   - three geometric constants L1, L2 determining the length of the arms
#   - two controllable cartesian coordinates x(th1, th2), y(th1, th2) of tip
#   - two phases of the servo's ph1, ph2 depending on the build geometry
#   - x(th1, th2, L1, L2) = [vec(L1) + vec(L2)]_x = L1 * cos(th1 + ph1) + L2 * cos(th2 + ph2)
#   - y(th1, th2, L1, L2) = [vec(L1) + vec(L2)]_y = L1 * sin(th1 + ph1) + L2 * sin(th2 + ph2)
#   - Goal: search for th1(x, y), th2(x, y) for x = const. and y = const.e


class RobotArm:
    """Controls the tip movement of the arm."""

    L1 = 80 # Shoulder to elbow length
    L2 = 80 # Elbow to wrist length

    ph1 = 0 # servo1 phase offset
    ph2 = 0 # servo2 phase offset

    def __init__(self, pin_s1: int, pin_s2: int) -> None:
        """Setup hardware."""

        GPIO.setmode(GPIO.board)

        # servos
        self._servo1 = servo.Servo(pin_s1)
        self._servo2 = servo.Servo(pin_s2)

    def __del__(self) -> None:
        """Close connection."""

        self._servo1.close()
        self._servo2.close()

        GPIO.cleanup()

    def moveX(self, x: int) -> None:
        """Move the tip horizontally."""

        # solve for x(th1, th2)

    def moveY(self, y: int) -> None:
        """Move the tip vertically."""

        # solve for y(th1, th2)

    def setAngles(self, th1: float = None, th2: float = None) -> None:
        """Sets the servo angles."""

        self._servo1.setAngle(th1)
        self._servo2.setAngle(th2)

    def getAngles(self) -> tuple[float, float]:
        """Return current angles."""

        return self._servo1.getAngle(), self._servo2.getAngle()
    
    def configureEasing(self, step_size: float = 10, step_time: float = 0.5) -> None:
        """Configure the easing behavior of both servos."""

        self._servo1.STEP_SIZE = step_size
        self._servo1.STEP_TIME = step_time

        self._servo2.STEP_SIZE = step_size
        self._servo2.STEP_TIME = step_time
    