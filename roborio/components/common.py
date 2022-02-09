from collections import deque


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
