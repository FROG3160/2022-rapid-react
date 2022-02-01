# components that are part of the shooter
# Issue No. 6
from ctre import (
    WPI_TalonFX, 
    FeedbackDevice, 
    ControlMode, 
    NeutralMode, 
    TalonFXInvertType)

FLYWHEEL_MODE = ControlMode.Velocity
# TODO Find out the 

class Flywheel:
    flywheel_motor_top: WPI_TalonFX
    flwwheel_motor_bottom: WPI_TalonFX

    def __init__(self):
        self.enabled = False
        self._controlMode = FLYWHEEL_MODE



    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def setup(self):
        pass

    def execute(self):
        if self.enabled:
            self.flywheel_motor_top.set(self._controlMode, self._velocity)
            self.flywheel_motor_bottom.set(self._controlMode, self._velocity)
        
        else:
            self.flywheel_motor_top.set(0)
            self.flywheel_motor_bottom.set(0)