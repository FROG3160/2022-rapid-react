from collections import deque
import math
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d


class Buffer(deque):
    def __init__(self, size: int, validLength: int = 1):
        """Constructor for Buffer

        Args:
            size (int): Maximum size of the buffer.  The largest number of values
                the buffer will keep.
            validLength (int, optional): The number of values in the buffer needed
                to treat the amount of data as valid. average() returns None if
                there aren't enough values.  Defaults to 1.
        """
        self.validLength = validLength
        super().__init__(maxlen=size)

    def _filterList(self):
        # our calculations can't accept None values
        return [x for x in self if x is not None]

    def _getBufferLength(self):
        return len(self._filterList())

    def _isValidData(self):
        return self._getBufferLength() >= self.validLength

    def average(self) -> float:
        """Get the average of all values in the buffer.

        Returns:
            float: The average of all values in the buffer if the number of values
                is >= the validLength parameter.
            None:  Returned if there aren't enough values to be >= the validLength
                parameter.
        """
        if self._isValidData():
            filteredList = self._filterList()
            return sum(filteredList) / len(filteredList)
        else:
            return None


def remap(val, OldMin, OldMax, NewMin, NewMax):
    """take a value in the old range and return a value in the new range"""
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


class Rescale:
    def __init__(
        self,
        original_scale: tuple[float, float],
        new_scale: tuple[float, float],
        deadband: float = 0.0,
    ) -> None:
        """Class for transferring a value from one scale to another

        Args:
            original_scale (tuple[float, float]): the original scale the the given value will fall in.
                Expressed as a tuple: (minimum: float, maximum: float)
            new_scale (tuple[float, float]): the new scale the given value wil fall in.
                Expressed as a tuple: (minimum: float, maximum: float)
            deadband (0.0): A deadband value, if desired, defaults to 0.0.  If a deadband is specified,
                the deadband is subtracted from the original value and then matched to the new scale.  If
                the value is within the deadband, 0 is returned.
        """
        self.orig_min = original_scale[0] + deadband
        self.orig_max = original_scale[1] - deadband
        self.new_min, self.new_max = new_scale
        self.deadband = deadband

    def __call__(self, value):
        value = (
            math.copysign(abs(value) - self.deadband, value)
            if abs(value) > self.deadband
            else 0
        )
        return (
            ((value - self.orig_min) * (self.new_max - self.new_min))
            / (self.orig_max - self.orig_min)
        ) + self.new_min

    def setNewMax(self, value: float):
        self.new_max = value


class TalonPID:
    """Class that holds contants for PID controls"""

    # TODO: Add methods to apply to motor?

    def __init__(
        self,
        slot: int = 0,
        p: float = 0,
        i: float = 0,
        d: float = 0,
        f: float = 0,
        iZone: float = 0,
    ):
        self.slot = slot
        self.p = p
        self.i = i
        self.d = d
        self.f = f
        self.iZone = iZone

    def configTalon(self, motor_control):
        motor_control.config_kP(self.slot, self.p, 0)
        motor_control.config_kI(self.slot, self.i, 0)
        motor_control.config_kD(self.slot, self.d, 0)
        motor_control.config_kF(self.slot, self.f, 0)
        motor_control.config_IntegralZone(self.slot, self.iZone, 0)


class PowerCurve:
    def __init__(self, power):
        self.setPower(power)

    def setPower(self, power):
        self.power = power

    def __call__(self, value):
        return math.pow(value, self.power)


class DriveUnit:
    def __init__(
        self, gear_stages: list, motor_rpm: int, diameter: float, cpr: int
    ):
        """Constructs a DriveUnit object that stores data about the drive, gear stages, and wheel.
              The gear_stages is a list of tuples where each tuple defines one stage .e.g. (14, 28)

        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            motor_rpm (int): Maximum rpm of the attached motor
            diameter (float): Diameter of the attached wheel in meters
            cpr (int): Number of encoder counts per revolution
        """
        self.gearing = math.prod(gear_stages)
        self.motor_rpm = motor_rpm
        self.cpr = cpr
        self.circumference = math.pi * diameter

    def speedToVelocity(self, speed: float) -> float:
        """Converts linear speed to Falcon velocity

        Args:
            speed (float): desired linear speed in meters per second

        Returns:
            float: velocity in encoder counts per 100ms
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = wheel_rotations_sec / self.gearing
        ticks_per_sec = motor_rotations_sec * self.cpr
        return ticks_per_sec / 10

    def velocityToSpeed(self, velocity: float) -> float:
        """Converts Falcon velocity to linear speed

        Args:
            velocity (float): velocity in encoder counts per 100ms

        Returns:
            float: linear speed in meters per second
        """
        ticks_per_sec = velocity * 10
        motor_rotations_sec = ticks_per_sec / self.cpr
        wheel_rotations_sec = motor_rotations_sec * self.gearing
        return wheel_rotations_sec * self.circumference


if __name__ == "__main__":
    wheel = DriveUnit(
        [(14.0 / 50.0), (27.0 / 17.0), (15.0 / 45.0)], 6380, 0.10033, 2048
    )
    scaled = Rescale((-1, 1), (-180, 180), 0.2)
    pass
