from wpilib import DriverStation, Solenoid, Timer
from ctre import (
    WPI_TalonFX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from magicbot import feedback, state, timed_state, tunable
from magicbot.state_machine import StateMachine
from components.common import TalonPID, Vector2, toleranceFromRange
from components.sensors import FROGsonic, FROGColor, FROGGyro
from components.vision import FROGVision
from components.led import FROGLED
from components.drivetrain import SwerveChassis
from logging import Logger

# TODO Find out the Min/Max of the velocity and the tolerence for the Flywheel
FLYWHEEL_MODE = ControlMode.Velocity
LOWER_FLYWHEEL_PID = TalonPID(0, p=0, i=0, d=0, f=0.04755)
UPPER_FLYWHEEL_PID = TalonPID(0, p=0, i=0, d=0, f=0.04700)
FLYWHEEL_VELOCITY = 0
FLYWHEEL_MAX_VEL = 22000  # Falcon ()
FLYWHEEL_MAX_ACCEL = FLYWHEEL_MAX_VEL / 50
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 100
FLYWHEEL_VEL_TOLERANCE = 0.03
FLYWHEEL_LOOP_RAMP = 0.1
FLYWHEEL_REJECT_SPEED = 5000

ULTRASONIC_DISTANCE_INCHES = 9.5  # 8.65
PROXIMITY_THRESHOLD = 1000

RED = DriverStation.Alliance.kRed  # 0
BLUE = DriverStation.Alliance.kBlue  # 1

TARGET_CARGO = 0
TARGET_GOAL = 1
DEFAULT_TARGET_TOLERANCE = 2.0


class Flywheel:
    motor: WPI_TalonFX

    def __init__(self):
        self.enabled = False
        self._controlMode = FLYWHEEL_MODE
        self._velocity = FLYWHEEL_VELOCITY
        self.tolerance = 0

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback(key="isReady")
    def isReady(self):
        return (
            abs(self.getVelocity() - self.getCommandedVelocity())
            <= self.tolerance * self.getCommandedVelocity()
            and self.getCommandedVelocity() > 0
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
        # FLYWHEEL_PID.configTalon(self.motor)
        # use closed loop ramp to accelerate smoothly
        self.motor.configClosedloopRamp(FLYWHEEL_LOOP_RAMP)
        self.motor.configVoltageCompSaturation(12)
        self.motor.enableVoltageCompensation(True)

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
    rollerDeploy: Solenoid
    rollerMotor: WPI_TalonFX

    def __init__(self):
        pass

    def setup(self):
        self.rollerMotor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )
        self.rollerMotor.setNeutralMode(NeutralMode.Coast)
        self.rollerMotor.setInverted(True)

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

    def extendRoller(self):
        self.rollerDeploy.set(True)

    def retractRoller(self):
        self.rollerDeploy.set(False)

    def runRoller(self):
        self.rollerMotor.set(ControlMode.PercentOutput, 0.3)

    def stopRoller(self):
        self.rollerMotor.set(0)

    def reverseRoller(self):
        self.rollerMotor.set(ControlMode.PercentOutput, -0.3)

    def intakeBall(self):
        self.extendRoller()
        self.runRoller()

    def rejectBall(self):
        self.deactivateHold()
        self.extendRoller()
        self.reverseRoller()

    def raiseIntake(self):
        self.retractRoller()
        self.stopRoller()

    def execute(self):
        pass


class FROGShooter:
    lowerFlywheel: Flywheel
    upperFlywheel: Flywheel
    launch: Solenoid
    flywheel_tolerance = tunable(0.075)

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
        LOWER_FLYWHEEL_PID.configTalon(self.lowerFlywheel.motor)
        UPPER_FLYWHEEL_PID.configTalon(self.upperFlywheel.motor)
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
        self.launch.set(True)

    def dropLaunch(self):
        self.launch.set(False)

    def getLaunchRaised(self):
        self.launch.get()

    @feedback()
    def isReady(self):
        self.lowerFlywheel.tolerance = self.flywheel_tolerance
        self.upperFlywheel.tolerance = self.flywheel_tolerance
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
    led: FROGLED
    gyro: FROGGyro
    swerveChassis: SwerveChassis

    flywheel_speed = tunable(0)
    flywheel_trim = tunable(1.06)
    trimIncrement = tunable(0.01)
    target_tolerance = tunable(DEFAULT_TARGET_TOLERANCE)
    logger: Logger
    tofDivisor = tunable(100)

    def __init__(self):
        self.autoIntake = True
        self.autoFire = True
        self.ballColor = None
        self.targetAzimuth = None
        self.shotLowerVelocity = None
        self.shotUpperVelocity = None
        self.shotRange = None
        self.shotTimer = Timer()
        self.objectTargeted = None
        self.dynamicTolerance = True

    @state(first=True)
    def waitForBall(self, initial_call):
        if initial_call:
            self.reset_pneumatics()

        if self.vision.hasCargoTargets and self.objectTargeted == TARGET_CARGO:
            if self.vision.allianceColor == BLUE:
                self.led.foundBlue()
            else:
                self.led.foundRed()

        if self.getBallInPosition():
            self.shooter.dropLaunch()
            self.next_state("holdBall")
        elif self.isInRange():
            self.next_state("grab")

    @timed_state(duration=0.25, next_state="retrieve")
    def grab(self, initial_call):
        if initial_call:
            if (
                self.shooter.getLaunchRaised is True
                and self.getBallColor is not None
            ):
                self.shooter.dropLaunch()
                self.next_state("holdBall")
            elif self.shooter.getLaunchRaised() is False:
                self.shooter.raiseLaunch()
                # extend arms and grab ball
            self.intake.extendGrabber()

    @timed_state(duration=0.5, next_state="checkBallColor")
    def retrieve(self, initial_call):
        # pull ball in while dropping launch
        if initial_call:
            self.intake.retractGrabber()
            self.shooter.dropLaunch()

    @state()
    def checkBallColor(self):
        # returns True if Blue, False if Red
        self.ballColor = self.getBallColor()
        if self.ballColor is not None:
            if self.ballColor:
                self.led.ColorChangeBlue()
            else:
                self.led.ColorChangeRed()
            self.next_state("holdBall")
        else:
            self.next_state("release")

    @state()
    def holdBall(self, initial_call):
        if initial_call:
            # clamp onto ball
            self.shooter.dropLaunch()
            self.intake.activateHold()
        if self.autoFire:
            self.next_state("waitForGoal")
        # else:
        #     if not self.getBallInPosition():
        #         self.next_state("grab")
        #     else:
        #         self.next_state("reject_ball")

    @state()
    def waitForGoal(self):
        if self.vision.hasGoalTargets and self.objectTargeted == TARGET_GOAL:
            self.led.foundGoal()
            self.next_state("waitForFlywheel")

    @state()
    def waitForFlywheel(self):
        if self.flywheel_speed > 0:
            flyspeed = self.flywheel_speed
        else:
            flyspeed = self.calculateFlywheelSpeed()
        self.shooter.setFlywheelSpeeds(flyspeed)

        if self.objectTargeted == TARGET_GOAL:
            if self.isOnTarget():
                self.led.ColorChangeGreen()
            else:
                if self.vision.hasGoalTargets:
                    self.led.foundGoal()
                else:
                    self.led.targetingGoal()

        if not flyspeed == 0:
            if self.shooter.isReady():
                if self.fireCommanded:
                    self.next_state_now("fire")
                elif (
                    self.isOnTarget()
                    and self.isRotatingSlow()
                    and self.isMoveXSlow()
                    and self.isMoveYSlow()
                ):
                    self.next_state("fire")

    @timed_state(duration=0.25, must_finish=True, next_state="fire")
    def waitToFire(self):
        pass

    @timed_state(duration=0.5, must_finish=True, next_state="release")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.shooter.raiseLaunch()
            self.shotTimer.reset()
            self.shotTimer.start()
            self.shotLowerVelocity = self.shooter.lowerFlywheel.getVelocity()
            self.shotUpperVelocity = self.shooter.upperFlywheel.getVelocity()
            self.shotRange = self.vision.getRangeInches()
            self.logger.info(
                "Shot fired -- lower: %s, upper: %s, range: %s, yaw_tolerance: %s, yaw: %s",
                self.shooter.lowerFlywheel.getVelocity(),
                self.shooter.upperFlywheel.getVelocity(),
                self.vision.getRangeInches(),
                self.target_tolerance,
                self.vision.getFilteredGoalYaw(),
            )

    @feedback()
    def getShotLowerVelocity(self):
        return self.shotLowerVelocity

    @feedback()
    def getShotUpperVelocity(self):
        return self.shotUpperVelocity

    @feedback()
    def getShotRange(self):
        return self.shotRange

    @state(must_finish=True)
    def release(self):
        self.reset_pneumatics()
        if self.autoIntake:
            self.next_state("waitForBall")

    @feedback()
    def isRotatingSlow(self):
        return abs(self.gyro.getRotationDPS()) < 0.1

    @feedback()
    def isMoveXSlow(self):
        return abs(self.swerveChassis.getChassisX_FPS()) < 0.2

    @feedback()
    def isMoveYSlow(self):
        return abs(self.swerveChassis.getChassisY_FPS()) < 0.2

    # @timed_state(duration=0.25, next_state='fire_reject')
    # def reject_ball(self, initial_call):
    #     self.shooter.setFlywheelSpeeds(FLYWHEEL_REJECT_SPEED)
    
    # @state()
    # def fire_reject(self):
    #     self.shooter.raiseLaunch()
    #     self.next_state("release")

    @feedback()
    def isInRange(self):
        return self.sonic.getInches() <= ULTRASONIC_DISTANCE_INCHES

    @feedback()
    def isOnTarget(self):
        if self.dynamicTolerance:
            if new_tolerance := toleranceFromRange(self.vision.getRangeInches()):
                self.target_tolerance = new_tolerance
            else:
                self.target_tolerance = DEFAULT_TARGET_TOLERANCE
        else:
            self.target_tolerance = DEFAULT_TARGET_TOLERANCE
        if yaw := self.vision.getFilteredGoalYaw():
            return abs(yaw) < self.target_tolerance

    @feedback()
    def isAtAzimuth(self):
        if self.targetAzimuth:
            return (
                abs(self.gyro.getOffsetYaw() - self.targetAzimuth)
                < self.target_tolerance
            )

    @feedback()
    def getBallColor(self):
        # only return a color if a ball is in position
        if self.getBallInPosition():
            # returns true if blue, false if red
            return self.color.getRed() < self.color.getBlue()

    @feedback()
    def getBallInPosition(self):
        return self.color.getProximity() > PROXIMITY_THRESHOLD

    def calculateFlywheelSpeed(self, range=None):
        calc_speed = 0
        if not range:
            if range := self.vision.getRangeInches():
                calc_speed = self.flywheelSpeedFromRange(range)
        else:
            calc_speed = self.flywheelSpeedFromRange(range)
        if calc_speed:
            return calc_speed * self.flywheel_trim
        else:
            return 0

    def flywheelSpeedFromRange(self, range):
        return (16.705 * range) + 8055.2
        # return (17.597 * range + 8481.9)

    def reset_pneumatics(self):
        self.intake.deactivateHold()
        self.shooter.raiseLaunch()
        self.intake.deactivateRetrieve()
        self.shooter.setFlywheelSpeeds(0)
        self.led.Default()

    def raiseFlywheelTrim(self):
        self.flywheel_trim += self.trimIncrement

    def lowerFlywheelTrim(self):
        self.flywheel_trim -= self.trimIncrement

    def getFlywheelTrim(self):
        return self.flywheel_trim

    def commandToFire(self, mode):
        self.fireCommanded = mode

    def logShottimer(self):
        self.logger.info("ToF: %s", self.shotTimer.get())

    @feedback()
    def getDynamicTolerance(self):
        return self.dynamicTolerance

    def toggleDynamicTolerance(self):
        self.dynamicTolerance = [True, False][self.dynamicTolerance]

    def calcMovingShot(self, vX, vY, target_range, target_azimuth):
        # TODO: calculate time of flight from range
        # and flywheel speed?
        self.logger.info(
            "calcMovingShot inputs: %s, %s, %s, %s",
            vX,
            vY,
            target_range,
            target_azimuth,
        )
        tof = (
            target_range / self.tofDivisor
        )  # assuming 200 inches in one second

        vC = Vector2(vX * tof, vY * tof)
        vT = Vector2.from_polar(target_range, target_azimuth)

        vS = vT - vC
        # print(sV.as_polar())
        # return sV.as_polar()
        self.logger.info("Calculated shot vector: %s", vS)
        return vS.to_polar()


if __name__ == "__main__":

    s = ShooterControl()

    print("----------------CALCULATIONS")
    print("\n=====Quadrant 1")
    s.calcMovingShot(-36, 0, 200, -135)
    s.calcMovingShot(36, 0, 200, -135)
    s.calcMovingShot(0, -36, 200, -135)
    s.calcMovingShot(0, 36, 200, -135)
    s.calcMovingShot(36, -36, 200, -135)
    s.calcMovingShot(-36, 36, 200, -135)
    s.calcMovingShot(-36, -36, 200, -135)
    s.calcMovingShot(36, 36, 200, -135)
    print("\n=====Quadrant 2")
    s.calcMovingShot(-36, 0, 200, -45)
    s.calcMovingShot(36, 0, 200, -45)
    s.calcMovingShot(0, -36, 200, -45)
    s.calcMovingShot(0, 36, 200, -45)
    s.calcMovingShot(36, -36, 200, -45)
    s.calcMovingShot(-36, 36, 200, -45)
    s.calcMovingShot(-36, -36, 200, -45)
    s.calcMovingShot(36, 36, 200, -45)
    print("\n=====Quadrant 3")
    s.calcMovingShot(-36, 0, 200, 45)
    s.calcMovingShot(36, 0, 200, 45)
    s.calcMovingShot(0, -36, 200, 45)
    s.calcMovingShot(0, 36, 200, 45)
    s.calcMovingShot(-36, -36, 200, 45)
    s.calcMovingShot(36, 36, 200, 45)
    s.calcMovingShot(-36, 36, 200, 45)
    s.calcMovingShot(36, -36, 200, 45)
    print("\n=====Quadrant 4")
    s.calcMovingShot(-36, 0, 200, 135)
    s.calcMovingShot(36, 0, 200, 135)
    s.calcMovingShot(0, -36, 200, 135)
    s.calcMovingShot(0, 36, 200, 135)
    s.calcMovingShot(36, -36, 200, 135)
    s.calcMovingShot(-36, 36, 200, 135)
    s.calcMovingShot(-36, -36, 200, 135)
    s.calcMovingShot(36, 36, 200, 135)
