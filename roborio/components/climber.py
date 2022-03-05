from ctre import TalonFX, LimitSwitchSource, LimitSwitchNormal
from wpilib import Solenoid

limitSwitchConfig = (LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen)
class FROGLift:
    stage1extend: TalonFX
    stage1tilt: TalonFX
    stage2extend: TalonFX

    stage1claw: Solenoid  # grabber - hook
    stage2tilt: Solenoid  # arm 2 tilt
    stage3tilt: Solenoid  # arm 3 tilt
    stage3release: Solenoid  # pin release

    def setup(self):
        # these settings are different for each motor, so we
        # set them here
        self.stage1extend.setSensorPhase(True)
        self.stage1tilt.setSensorPhase(True)
        self.stage2extend.setSensorPhase(True)

        self.stage1extend.setInverted(True)
        self.stage1tilt.setInverted(False)
        self.stage2extend.setInverted(False)

        self.stage1extend.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.stage2extend.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.stage1extend.configReverseLimitSwitchSource(*limitSwitchConfig)
        self.stage2extend.configReverseLimitSwitchSource(*limitSwitchConfig)


        self.set_manual()
        self.enable()

    def execute(self):
        pass
    