from wpilib import DriverStation, Solenoid
from ctre import (
    WPI_TalonFX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from magicbot import feedback, state, timed_state
from magicbot.state_machine import StateMachine
from components.common import TalonPID
from components.sensors import FROGdar, FROGsonic, FROGColor
from components.vision import FROGVision
from magicbot import tunable


# TODO Find out the Min/Max of the velocity and the tolerence for the Flywheel
FLYWHEEL_MODE = ControlMode.Velocity
FLYWHEEL_PID = TalonPID(0, p=0, f=0.0475)
FLYWHEEL_VELOCITY = 0
FLYWHEEL_MAX_VEL = 22000  # Falcon ()
FLYWHEEL_MAX_ACCEL = FLYWHEEL_MAX_VEL / 50
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 100
FLYWHEEL_VEL_TOLERANCE = 100
FLYWHEEL_LOOP_RAMP = 0.25

ULTRASONIC_DISTANCE_INCHES = 9.5  # 8.65
PROXIMITY_THRESHOLD = 1375
TARGET_TOLERANCE = 0.1

RED = DriverStation.Alliance.kRed  # 0
BLUE = DriverStation.Alliance.kBlue  # 1


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
        # TODO: remove below comment if it works without negating the value.
        # sensor values are reversed for one of the motors.  we never command
        # them to a negative view, so we'll just get the absolute value
        return abs(
            self.motor.getSelectedSensorVelocity(
                FeedbackDevice.IntegratedSensor
            )
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

    def __init__(self):
        pass

    def activateRetrieve(self):
        self.retrieve.set(True)

    def extendGrabber(self):
        self.retrieve.set(True)

    def retractGrabber(self):
        self.retrieve.set(False)

    def deactivateRetrieve(self):
        self.retrieve.set(False)

    def activateHold(self):
        self.hold.set(True)

    def deactivateHold(self):
        self.hold.set(False)

    def execute(self):
        pass


class FROGShooter:
    lowerFlywheel: Flywheel
    upperFlywheel: Flywheel
    launch: Solenoid

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
        self.lowerFlywheel.motor.setInverted(True)
        self.upperFlywheel.motor.setInverted(False)
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

    def raiseLaunch(self):
        self.launch.set(False)

    def dropLaunch(self):
        self.launch.set(True)

    @feedback()
    def isReady(self):
        return (
            self.lowerFlywheel.isReady()
            and self.upperFlywheel.isReady()
            and not self._flywheel_speeds == 0
        )

    @feedback()
    def getLowerRatio(self):
        return self.ratio_lower

    @feedback()
    def getUpperRatio(self):
        return self.ratio_upper

    def execute(self):
        if self._enabled:
            if self._automatic:
                # get value from self.getdistance() and adjust
                # the speeds of the motors
                pass
            else:
                self.lowerFlywheel.setVelocity(self._flywheel_speeds)
                self.upperFlywheel.setVelocity(
                    self._flywheel_speeds
                    * (1 / (self.ratio_lower / self.ratio_upper))
                )

                # run the motors at the speeds they already have
                pass


class ShooterControl(StateMachine):
    intake: Intake
    shooter: FROGShooter
    sonic: FROGsonic
    vision: FROGVision
    color: FROGColor
    #

    flywheel_speed = tunable(10000)

    def __init__(self):
        self.autoIntake = False
        self.autoFire = False
        self.ballColor = None

    @state(first=True)
    def waitForBall(self, initial_call):
        if initial_call:
            self.reset_pneumatics()

        if self.isInRange():
            self.next_state("grab")

    @timed_state(duration=1, next_state="retrieve")
    def grab(self, initial_call):
        if initial_call:
            # extend arms and grab ball
            self.intake.extendGrabber()

    @timed_state(duration=2, next_state="checkBallColor")
    def retrieve(self, initial_call):
        # pull ball in while dropping launch
        if initial_call:
            self.intake.retractGrabber()
            self.shooter.dropLaunch()

    @state()
    def checkBallColor(self):
        self.ballColor = self.getBallInPosition()
        if self.ballColor:
            self.next_state("holdBall")
        else:
            self.next_state("release")

    @timed_state(duration=1)
    def holdBall(self, initial_call):
        if initial_call:
            # clamp onto ball
            self.intake.activateHold()
        if self.autoFire:
            self.next_state("waitForGoal")

    @state()
    def waitForGoal(self):
        if self.vision.hasGoalTargets:
            self.next_state("waitForFlywheel")

    @state()
    def waitForFlywheel(self):
        self.shooter.setFlywheelSpeeds(self.calculateFlywheelSpeed())
        if (
            self.shooter.isReady()
            and abs(self.vision.getFilteredGoalYaw()) < TARGET_TOLERANCE
        ):
            self.next_state("fire")

    # @timed_state(duration=1, must_finish=True, next_state="fire")
    # def waitToFire(self):
    #     pass

    @timed_state(duration=1, must_finish=True, next_state="release")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.shooter.raiseLaunch()

    @state(must_finish=True)
    def release(self):
        self.reset_pneumatics()
        if self.autoIntake:
            self.next_state("waitForBall")

    @feedback()
    def isInRange(self):
        return self.sonic.getInches() <= ULTRASONIC_DISTANCE_INCHES

    @feedback()
    def getBallColor(self):
        # only return a color if we
        if self.getBallInPosition():
            return self.color.getRed() < self.color.getBlue()

    @feedback()
    def getBallInPosition(self):
        return self.color.getProximity() > PROXIMITY_THRESHOLD

    def calculateFlywheelSpeed(self):
        return 8000 + self.vision.getRangeInches()

    def reset_pneumatics(self):
        self.intake.deactivateHold()
        self.shooter.raiseLaunch()
        self.intake.deactivateRetrieve()
        self.shooter.setFlywheelSpeeds(0)
