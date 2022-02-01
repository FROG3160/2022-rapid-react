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