from wpilib import DriverStation, Solenoid
from ctre import (
    WPI_TalonFX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from magicbot import feedback
from components.common import TalonPID
from components.sensors import FROGdar


# TODO Find out the Min/Max of the velocity and the tolerence for the Flywheel
FLYWHEEL_MODE = ControlMode.Velocity
FLYWHEEL_PID = TalonPID(0, p=0, f=0.042)
FLYWHEEL_VELOCITY = 0
FLYWHEEL_MAX_VEL = 22000  # Falcon ()
FLYWHEEL_MAX_ACCEL = FLYWHEEL_MAX_VEL / 50
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 100
FLYWHEEL_VEL_TOLERANCE = 100
FLYWHEEL_LOOP_RAMP = 0.25


class Flywheel:
    motor: WPI_TalonFX

    def __init__(self):
        self.enabled = False
        self._controlMode = FLYWHEEL_MODE
        self._velocity = FLYWHEEL_VELOCITY

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback(key="isReady")
    def isReady(self):
        return (
            abs(self.getVelocity() - self.getCommandedVelocity())
            < FLYWHEEL_VEL_TOLERANCE
        )

    # read current encoder velocity
    @feedback(key="velocity")
    def getVelocity(self):
        # sensor values are reversed.  we command a positive value and the
        # sensor shows a negative one, so we negate the output
        return -self.motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor
        )

    @feedback(key="commanded")
    def getCommandedVelocity(self):
        return self._velocity

    def setup(self):
        # Falcon500 motors use the integrated sensor
        self.motor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )
        # self.motor.setSensorPhase(False)
        # = setInverted(True)
        # self.motor.setInverted(TalonFXInvertType.CounterClockwise)
        self.motor.setNeutralMode(NeutralMode.Coast)
        FLYWHEEL_PID.configTalon(self.motor)
        # use closed loop ramp to accelerate smoothly
        self.motor.configClosedloopRamp(FLYWHEEL_LOOP_RAMP)

    def setVelocity(self, velocity):
        # self._controlMode = ControlMode.Velocity
        if velocity > FLYWHEEL_MAX_VEL:
            velocity = FLYWHEEL_MAX_VEL
        elif velocity < 0:
            velocity = 0
        self._velocity = velocity

    def incrementSpeed(self):
        velocity = self._velocity + FLYWHEEL_INCREMENT
        self.setVelocity(velocity)

    def decrementSpeed(self):
        velocity = self._velocity - FLYWHEEL_INCREMENT
        self.setVelocity(velocity)

    def execute(self):
        if self.enabled:
            self.motor.set(self._controlMode, self._velocity)
        else:
            self.motor.set(0)


class Intake:
    retrieve: Solenoid
    hold: Solenoid
    launch: Solenoid

    def __init__(self):
        pass

    def activateRetrieve(self):
        self.retrieve.set(True)

    def deactivateRetrieve(self):
        self.retrieve.set(False)

    def activateHold(self):
        self.hold.set(True)

    def deactivateHold(self):
        self.hold.set(False)

    def activateLaunch(self):
        self.launch.set(True)

    def deactivateLaunch(self):
        self.launch.set(False)

    def execute(self):
        pass


class FROGShooter:
    lidar: FROGdar
    lowerFlywheel: Flywheel
    upperFlywheel: Flywheel

    def __init__(self):
        self._enable = False
        self._automatic = False
        self.ratio_lower = 5
        self.ratio_upper = 5
        self._flywheel_speeds = 0

    def enable(self):
        self._enabled = True
        self.lowerFlywheel.setVelocity(0)
        self.upperFlywheel.setVelocity(0)
        self.lowerFlywheel.enable()
        self.upperFlywheel.enable()

    def set_automatic(self):
        self._automatic = True

    def set_manual(self):
        self._automatic = False
        self.lowerFlywheel.setVelocity(0)
        self.upperFlywheel.setVelocity(0)

    def setup(self):
        # these settings are different for each motor, so we
        # set them here
        self.lowerFlywheel.motor.setInverted(TalonFXInvertType.Clockwise)
        self.upperFlywheel.motor.setInverted(TalonFXInvertType.CounterClockwise)
        self.lowerFlywheel.motor.setSensorPhase(True)
        self.upperFlywheel.motor.setSensorPhase(True)
        self.set_manual()
        self.enable()

    def disable(self):
        self._enabled = False
        self.lowerFlywheel.disable()
        self.upperFlywheel.disable()

    def setFlywheelSpeeds(self, speed: int):
        if speed > FLYWHEEL_MAX_VEL:
            speed = FLYWHEEL_MAX_VEL
        elif speed < 0:
            speed = 0
        self._flywheel_speeds = speed

    def incrementFlywheelSpeeds(self):
        self.setFlywheelSpeeds(self._flywheel_speeds + FLYWHEEL_INCREMENT)

    def decrementFlywheelSpeeds(self):
        self.setFlywheelSpeeds(self._flywheel_speeds - FLYWHEEL_INCREMENT)

    def setLowerRatio(self, val: int):
        self.ratio_lower = val

    def setUpperRatio(self, val: int):
        self.ratio_upper = val

    @feedback()
    def getLowerRatio(self):
        return self.ratio_lower

    @feedback()
    def getUpperRatio(self):
        return self.ratio_upper

    @feedback()
    def getdistance(self):
        # get the value/distance from the lidar in inches
        return self.lidar.getDistance()

    def execute(self):
        if self._enabled:
            if self._automatic:
                # get value from self.getdistance() and adjust
                # the speeds of the motors
                pass
            else:
                self.lowerFlywheel.setVelocity(self._flywheel_speeds)
                self.upperFlywheel.setVelocity(self._flywheel_speeds*(1/(self.ratio_lower/self.ratio_upper)))

                # run the motors at the speeds they already have
                pass
