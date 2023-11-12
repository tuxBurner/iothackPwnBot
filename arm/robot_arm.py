"""Interface for the robot arm."""

from math import acos, atan, sin, asin, pi
import RPi.GPIO as GPIO
from . import servo
import time
import numpy

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

    STEP_SIZE = 1
    STEP_TIME = 0.02

    def __init__(self, pin_s1: int, pin_s2: int) -> None:
        """Setup hardware."""

        GPIO.setmode(GPIO.BOARD)

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

    def _ease_single(self, servo, angle):

        if angle != servo.getAngles():
            angles = numpy.arange(servo.getAngle(), angle, self.STEP_SIZE)
            for th in angles:
                servo.setAngle(th)
                time.sleep(self.STEP_TIME)

    def setAngles(self, th1: float = None, th2: float = None) -> None:
        """Sets the servo angles."""

        diff1 = th1 - self._servo1.getAngle()
        diff2 = th2 - self._servo2.getAngle()

        if (th1 is None) or (diff1 == 0):
            return self._ease_single(self._servo2, th2)
            
        if (th2 is None) or (diff2 == 0):
            return self._ease_single(self._servo1, th1)

        if numpy.abs(diff1) > numpy.abs(diff2):
            th1_angles = numpy.arange(self._servo1.getAngle(), th1 + numpy.sign(diff1), numpy.sign(diff1) * self.STEP_SIZE)
            th2_angles = numpy.linspace(self._servo2.getAngle(), th2, len(th1_angles))
        
        else:
            th2_angles = numpy.arange(self._servo2.getAngle(), th2 + numpy.sign(diff2), numpy.sign(diff2) * self.STEP_SIZE)
            th1_angles = numpy.linspace(self._servo1.getAngle(), th1, len(th2_angles))

        for t1, t2 in zip(th1_angles, th2_angles):
            self._servo1.setAngle(t1)
            self._servo2.setAngle(t2)
            time.sleep(self.STEP_TIME)

    def getAngles(self) -> tuple[float, float]:
        """Return current angles."""

        return self._servo1.getAngle(), self._servo2.getAngle()
    
    def configureEasing(self, step_size: float = 1, step_time: float = 0.02) -> None:
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
    BOTTOM_LIMIT = 20

    ph1 = 90 # servo1 phase offset
    ph2 = 90 # servo2 phase offset

    STEP_SIZE = 1
    STEP_TIME = 0.02

    def __init__(self, pin_s1: int, pin_s2: int) -> None:
        """Setup hardware."""

        GPIO.setmode(GPIO.BOARD)

        # servos
        self._servo1 = servo.Servo(pin_s1)
        self._servo2 = servo.Servo(pin_s2)

        # internal state
        self._h = 0

        # goto origin
        self.setHeight(0)

    def __del__(self) -> None:
        """Close connection."""

        self._servo1.close()
        self._servo2.close()

        GPIO.cleanup()

    def setHeight(self, h: float):
        """Sets the tip height in cartesian coordinates."""

        h = max(min(h, self.L1), -self.L1 + self.BOTTOM_LIMIT)

        th = - asin(h / self.L1)
        th = th * 360 / (2 * pi)
        self.setAngles(th + self.ph1, th + self.ph2)

        self._h = h

    def getHeight(self):
        """Returns current position."""

        return self._h
    
    def _ease_single(self, servo, angle):

        if angle != servo.getAngles():
            angles = numpy.arange(servo.getAngle(), angle, self.STEP_SIZE)
            for th in angles:
                servo.setAngle(th)
                time.sleep(self.STEP_TIME)

    def setAngles(self, th1: float = None, th2: float = None) -> None:
        """Sets the servo angles."""

        diff1 = th1 - self._servo1.getAngle()
        diff2 = th2 - self._servo2.getAngle()

        if (th1 is None) or (diff1 == 0):
            return self._ease_single(self._servo2, th2)
            
        if (th2 is None) or (diff2 == 0):
            return self._ease_single(self._servo1, th1)

        if numpy.abs(diff1) > numpy.abs(diff2):
            th1_angles = numpy.arange(self._servo1.getAngle(), th1 + numpy.sign(diff1), numpy.sign(diff1) * self.STEP_SIZE)
            th2_angles = numpy.linspace(self._servo2.getAngle(), th2, len(th1_angles))
        
        else:
            th2_angles = numpy.arange(self._servo2.getAngle(), th2 + numpy.sign(diff2), numpy.sign(diff2) * self.STEP_SIZE)
            th1_angles = numpy.linspace(self._servo1.getAngle(), th1, len(th2_angles))

        for t1, t2 in zip(th1_angles, th2_angles):
            self._servo1.setAngle(t1)
            self._servo2.setAngle(t2)
            time.sleep(self.STEP_TIME)

    def getAngles(self) -> tuple[float, float]:
        """Return current angles."""

        return self._servo1.getAngle(), self._servo2.getAngle()
    
    @classmethod
    def configureEasing(self, step_size: float = 1, step_time: float = 0.02) -> None:
        """Configure the easing behavior of both servos."""

        self.STEP_SIZE = step_size
        self.STEP_TIME = step_time

    @classmethod
    def configureGeometry(self, l1: float = None, b_limit: float = None,
                          ph1: float = None, ph2: float = None) -> None:

        if l1 is not None:
            self.L1 = l1

        if b_limit is not None:
            self.BOTTOM_LIMIT = b_limit

        if ph1 is not None:
            self.ph1 = ph1

        if ph2 is not None:
            self.ph2 = ph2
    
