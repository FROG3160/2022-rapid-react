from ctre import TalonFX, LimitSwitchSource, LimitSwitchNormal
from wpilib import Solenoid
from magicbot import tunable
from sensors import FROGGyro

STAGE1_THRESHOLD = 475757.0
STAGE1_TILT_THRESHOLD = 19000
STAGE2_THRESHOLD = 164638.0

limitSwitchConfig = (
    LimitSwitchSource.FeedbackConnector,
    LimitSwitchNormal.NormallyOpen,
)


class FROGLift:
    stage1extend: TalonFX
    stage1tilt: TalonFX
    stage2extend: TalonFX

    stage1claw: Solenoid  # grabber - hook
    stage2tilt: Solenoid  # arm 2 tilt
    stage3tilt: Solenoid  # arm 3 tilt
    stage3release: Solenoid  # pin release
    
    gyro: FROGGyro

    stage1speed = tunable(0.3)

    def setup(self):
        # these settings are different for each motor, so we
        # set them here
        self.stage1extend.setInverted(False)
        self.stage1extend.setSensorPhase(False)
        self.stage1extend.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.stage1extend.configReverseLimitSwitchSource(*limitSwitchConfig)
        self.stage1extend.configForwardSoftLimitEnable(True)
        self.stage1extend.configForwardSoftLimitThreshold(STAGE1_THRESHOLD)
        self.stage1extend.configReverseSoftLimitEnable(True)
        self.stage1extend.configReverseSoftLimitThreshold(0)
        
        self.stage2extend.setInverted(True)
        self.stage2extend.setSensorPhase(True)
        self.stage2extend.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.stage2extend.configReverseLimitSwitchSource(*limitSwitchConfig)
        self.stage2extend.configForwardSoftLimitEnable(True)
        self.stage2extend.configForwardSoftLimitThreshold(STAGE2_THRESHOLD)
        self.stage2extend.configReverseSoftLimitEnable(True)
        self.stage2extend.configReverseSoftLimitThreshold(0)

        self.stage1tilt.setSensorPhase(True)
        self.stage1tilt.setInverted(False)
        self.stage1tilt.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.stage1tilt.configReverseLimitSwitchSource(*limitSwitchConfig)

    def extendStage1(self):
        self.stage1extend.set(self.stage1speed)

    def retractStage1(self):
        self.stage1extend.set(-self.stage1speed)

    

    


    def execute(self):
        pass
