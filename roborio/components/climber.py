from ctre import (
    TalonFX,
    LimitSwitchSource,
    LimitSwitchNormal,
    StatusFrameEnhanced,
    SensorInitializationStrategy,
    NeutralMode,
)
from wpilib import Solenoid
from magicbot import tunable, feedback
from components.sensors import FROGGyro

STAGE1_THRESHOLD = 534000.0
STAGE1_TILT_THRESHOLD = 24000
STAGE2_THRESHOLD = 164638.0

limitSwitchConfig = (
    LimitSwitchSource.FeedbackConnector,
    LimitSwitchNormal.NormallyOpen,
)


class FROGLift:
    stage1extend: TalonFX
    # stage1tilt: TalonFX
    # stage2extend: TalonFX

    # stage1claw: Solenoid  # grabber - hook
    # stage2tilt: Solenoid  # arm 2 tilt
    # stage3tilt: Solenoid  # arm 3 tilt
    # stage3release: Solenoid  # pin release
    # gyro: FROGGyro

    stage1ExtendSpeed = tunable(0.6)
    stage1RetractSpeed = tunable(0.4)
    # stage2speed = tunable(0.4)
    # stage1RotateForward = tunable(0.08)
    # stage1RotateBack = tunable(0.15)

    def setup(self):
        # these settings are different for each motor, so we
        # set them here
        self.stage1extend.setInverted(False)
        self.stage1extend.setSensorPhase(False)
        self.stage1extend.configForwardLimitSwitchSource(*limitSwitchConfig)
        self.stage1extend.configReverseLimitSwitchSource(*limitSwitchConfig)
        self.stage1extend.configForwardSoftLimitEnable(True)
        self.stage1extend.configForwardSoftLimitThreshold(STAGE1_THRESHOLD)
        self.stage1extend.configReverseSoftLimitEnable(False)
        self.stage1extend.setStatusFramePeriod(
            StatusFrameEnhanced.Status_1_General, 250
        )
        self.stage1extend.setNeutralMode(NeutralMode.Brake)

        # self.stage2extend.setInverted(True)
        # self.stage2extend.setSensorPhase(True)
        # self.stage2extend.configForwardLimitSwitchSource(*limitSwitchConfig)
        # self.stage2extend.configReverseLimitSwitchSource(*limitSwitchConfig)
        # self.stage2extend.configForwardSoftLimitEnable(True)
        # self.stage2extend.configForwardSoftLimitThreshold(STAGE2_THRESHOLD)
        # self.stage2extend.configReverseSoftLimitEnable(True)
        # self.stage2extend.configReverseSoftLimitThreshold(0)
        # self.stage2extend.setStatusFramePeriod(
        #     StatusFrameEnhanced.Status_1_General, 250
        # )

        # self.stage1tilt.setSensorPhase(True)
        # self.stage1tilt.setInverted(True)
        # self.stage1tilt.configForwardLimitSwitchSource(*limitSwitchConfig)
        # self.stage1tilt.configReverseLimitSwitchSource(*limitSwitchConfig)
        # self.stage1tilt.configForwardSoftLimitEnable(True)
        # self.stage1tilt.configForwardSoftLimitThreshold(STAGE1_TILT_THRESHOLD)
        # self.stage1tilt.configReverseSoftLimitEnable(True)
        # self.stage1tilt.configReverseSoftLimitThreshold(0)
        # self.stage1tilt.setStatusFramePeriod(
        #     StatusFrameEnhanced.Status_1_General, 250
        # )

    def extendStage1(self):
        if not self.stage1extend.isFwdLimitSwitchClosed():
            self.stage1extend.set(self.stage1ExtendSpeed)
        else:
            self.stage1extend.set(0)

    # def extendStage2(self):
    #     if not self.stage2extend.isFwdLimitSwitchClosed():
    #         self.stage2extend.set(self.stage2speed)
    #     else:
    #         self.stage2extend.set(0)

    # def retractStage2(self):
    #     if not self.stage2extend.isRevLimitSwitchClosed():
    #         self.stage2extend.set(-self.stage2speed)
    #     else:
    #         self.stage2extend.set(0)

    def retractStage1(self):
        if not self.stage1extend.isRevLimitSwitchClosed():
            self.stage1extend.set(-self.stage1RetractSpeed)
        else:
            self.stage1extend.set(0)

    # def tiltStage1Forward(self):
    #     if not self.stage1tilt.isFwdLimitSwitchClosed():
    #         self.stage1tilt.set(self.stage1RotateForward)
    #     else:
    #         self.stage1tilt.set(0)

    # def tiltStage1Back(self):
    #     if not self.stage1tilt.isRevLimitSwitchClosed():
    #         self.stage1tilt.set(-self.stage1RotateBack)
    #     else:
    #         self.stage1tilt.set(0)

    # def activateClaw(self):
    #     self.stage1claw.set(True)

    # def deactivateClaw(self):
    #     self.stage1claw.set(False)

    @feedback
    def getStage1ExtendPosition(self):
        return self.stage1extend.getSelectedSensorPosition()

    # @feedback
    # def getStage1TiltPosition(self):
    #     return self.stage1tilt.getSelectedSensorPosition()

    # @feedback
    # def getStage2ExtendPosition(self):
    #     return self.stage2extend.getSelectedSensorPosition()

    # @feedback
    # def getGyroPitch(self):
    #     return self.gyro.getPitch()

    # @feedback
    # def getGyroRoll(self):
    #     return self.gyro.getRoll()

    def execute(self):
        pass
