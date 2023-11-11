"""Interface for the robot arm."""

from math import acos, atan, sin, asin
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
    origin = L1, L2

    def __init__(self, pin_s1: int, pin_s2: int) -> None:
        """Setup hardware."""

        GPIO.setmode(GPIO.board)

        # servos
        self._servo1 = servo.Servo(pin_s1)
        self._servo2 = servo.Servo(pin_s2)

        # internal state
        self._x = None
        self._y = None

        # goto origin
        self.setPosition(0, 0)

    def __del__(self) -> None:
        """Close connection."""

        self._servo1.close()
        self._servo2.close()

        GPIO.cleanup()

    def setPosition(self, x: float = None, y: float = None):
        """Sets the tip position in cartesian coordinates."""

        if x is None:
            x = self._x

        if y is None:
            y = self._y

        _x = x - self.origin[0]
        _y = y - self.origin[1]

        cos_th2 = (_x**2 + _y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        th2 = acos(cos_th2)
        th1 = atan(_y/_x) - atan((self.L2 *sin(th2)) / (self.L1 + self.L2 * cos_th2))

        self.setAngles(th1 - self.ph1, th2 - self.ph2)

        self._x = x
        self._y = y

    def getPosition(self) -> tuple[float, float]:
        """Returns current position."""

        return self.x, self.y

    def setAngles(self, th1: float = None, th2: float = None) -> None:
        """Sets the servo angles."""

        self._servo1.setAngle(th1)
        self._servo2.setAngle(th2)

    def getAngles(self) -> tuple[float, float]:
        """Return current angles."""

        return self._servo1.getAngle(), self._servo2.getAngle()
    
    def configureEasing(self, step_size: float = 5, step_time: float = 0.2) -> None:
        """Configure the easing behavior of both servos."""

        self._servo1.STEP_SIZE = step_size
        self._servo1.STEP_TIME = step_time

        self._servo2.STEP_SIZE = step_size
        self._servo2.STEP_TIME = step_time

    @classmethod
    def configureGeometry(self, l1: float = None, l2: float = None,
                          ph1: float = None, ph2: float = None, 
                          origin: float = None) -> None:

        if l1 is not None:
            self.L1 = l1
        
        if l2 is not None:
            self.L2 = l2

        if ph1 is not None:
            self.ph1 = ph1

        if ph2 is not None:
            self.ph2 = ph2

        if origin is None:
            origin = self.L2, self.L1

        else:
            origin = origin
    

class EasyArm:
    """Easy two servo construct."""

    L1 = 40 # Shoulder to elbow length

    ph1 = 0 # servo1 phase offset
    ph2 = 0 # servo2 phase offset

    def __init__(self, pin_s1: int, pin_s2: int) -> None:
        """Setup hardware."""

        GPIO.setmode(GPIO.board)

        # servos
        self._servo1 = servo.Servo(pin_s1)
        self._servo2 = servo.Servo(pin_s2)

        # internal state
        self._h = None

        # goto origin
        self.setHeight(0)

    def __del__(self) -> None:
        """Close connection."""

        self._servo1.close()
        self._servo2.close()

        GPIO.cleanup()

    def setHeight(self, h: float):
        """Sets the tip height in cartesian coordinates."""

        th = asin(h / self.L1)
        self.setAngles(th - self.ph1, -th - self.ph2)

        self._h = h

    def getHeight(self) -> tuple[float, float]:
        """Returns current position."""

        return self._h

    def setAngles(self, th1: float = None, th2: float = None) -> None:
        """Sets the servo angles."""

        self._servo1.setAngle(th1)
        self._servo2.setAngle(th2)

    def getAngles(self) -> tuple[float, float]:
        """Return current angles."""

        return self._servo1.getAngle(), self._servo2.getAngle()
    
    def configureEasing(self, step_size: float = 5, step_time: float = 0.2) -> None:
        """Configure the easing behavior of both servos."""

        self._servo1.STEP_SIZE = step_size
        self._servo1.STEP_TIME = step_time

        self._servo2.STEP_SIZE = step_size
        self._servo2.STEP_TIME = step_time

    @classmethod
    def configureGeometry(self, l1: float = None,
                          ph1: float = None, ph2: float = None) -> None:

        if l1 is not None:
            self.L1 = l1

        if ph1 is not None:
            self.ph1 = ph1

        if ph2 is not None:
            self.ph2 = ph2
    
