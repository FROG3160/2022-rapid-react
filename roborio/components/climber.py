from ctre import TalonFX, LimitSwitchSource, LimitSwitchNormal
from wpilib import Solenoid

limitSwitchConfig = (LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen)
class FROGLift:
    lift1: TalonFX
    lift2: TalonFX
    angle: TalonFX

    cylinder1: Solenoid
    cylinder2: Solenoid

    def setup(self):
        # these settings are different for each motor, so we
        # set them here
        self.lift1.setSensorPhase(True)
        self.lift2.setSensorPhase(True)
        self.angle.setSensorPhase(True)

        self.lift1.setInverted(True)
        self.lift2.setInverted(False)
        self.angle.setInverted(False)

        self.lift1.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.lift2.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.lift1.configReverseLimitSwitchSource(*limitSwitchConfig)
        self.lift2.configReverseLimitSwitchSource(*limitSwitchConfig)


        self.set_manual()
        self.enable()

    def execute(self):

    