# components that are part of the shooter
# Issue No. 6
from ctre import (
    WPI_TalonFX, 
    FeedbackDevice, 
    ControlMode, 
    NeutralMode, 
    TalonFXInvertType)
# from .common import TalonPID
# TODO Refine and 

# TODO Find out the Min/Max of the velocity and the tolerence for the Flywheel
FLYWHEEL_MODE = ControlMode.Velocity
FLYWHEEL_PID = self.TalonPID(0, p=0.4, f=0.0515)
FLYWHEEL_VELOCITY = 7300
FLYWHEEL_MAX_VEL = 25000
FLYWHEEL_MAX_ACCEL = (FLYWHEEL_MAX_VEL / 50)
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 500

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

class Flywheel:
    flywheel_motor_top: WPI_TalonFX
    flywheel_motor_bottom: WPI_TalonFX

    def __init__(self):
        self.enabled = False
        self._controlMode = FLYWHEEL_MODE

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def setup(self):
        

    def execute(self):
        if self.enabled:
            self.flywheel_motor_top.set(self._controlMode, self._velocity)
            self.flywheel_motor_bottom.set(self._controlMode, self._velocity)
        
        else:
            self.flywheel_motor_top.set(0)
            self.flywheel_motor_bottom.set(0)