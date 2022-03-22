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
from components.common import TalonPID, Vector2
from components.sensors import FROGsonic, FROGColor
from components.vision import FROGVision
from magicbot import tunable
from logging import Logger

# TODO Find out the Min/Max of the velocity and the tolerence for the Flywheel
FLYWHEEL_MODE = ControlMode.Velocity
LOWER_FLYWHEEL_PID = TalonPID(0, p=0, i=0, d=0, f=0.04752)
UPPER_FLYWHEEL_PID = TalonPID(0, p=0, i=0, d=0, f=0.047)
FLYWHEEL_VELOCITY = 0
FLYWHEEL_MAX_VEL = 22000  # Falcon ()
FLYWHEEL_MAX_ACCEL = FLYWHEEL_MAX_VEL / 50
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 100
FLYWHEEL_VEL_TOLERANCE = 0.03
FLYWHEEL_LOOP_RAMP = 0.25

ULTRASONIC_DISTANCE_INCHES = 9.5  # 8.65
PROXIMITY_THRESHOLD = 1000

RED = DriverStation.Alliance.kRed  # 0
BLUE = DriverStation.Alliance.kBlue  # 1


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

    flywheel_speed = tunable(0)
    flywheel_trim = tunable(0.9)
    trimIncrement = tunable(0.01)
    target_tolerance = tunable(5.0)
    logger: Logger

    def __init__(self):
        self.autoIntake = True
        self.autoFire = True
        self.ballColor = None
        self.driverstation = DriverStation
        self.targetAzimuth = None

    @state(first=True)
    def waitForBall(self, initial_call):
        if initial_call:
            self.reset_pneumatics()

        if self.isInRange():
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
        self.ballColor = self.getBallColor()
        if self.ballColor is not None:
            self.next_state("holdBall")
            # else:
            #     self.next_state("release")
        else:
            self.next_state("release")

    @state()
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
        if self.flywheel_speed > 0:
            flyspeed = self.flywheel_speed
        else:
            flyspeed = self.calculateFlywheelSpeed()
        self.shooter.setFlywheelSpeeds(flyspeed)

        if not flyspeed == 0:
            if self.shooter.isReady() and self.fireCommanded:
                self.next_state_now("waitToFire")
            elif self.shooter.isReady() and self.isOnTarget():
                self.next_state("waitToFire")

    @timed_state(duration=0.25, must_finish=True, next_state="fire")
    def waitToFire(self):
        pass

    @timed_state(duration=1, must_finish=True, next_state="release")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.shooter.raiseLaunch()
            self.logger.info(
                "Shot fired -- flywheel speed: %s, range: %s",
                self.shooter._flywheel_speeds,
                self.vision.getRangeInches()
            )

    @state(must_finish=True)
    def release(self):
        self.reset_pneumatics()
        if self.autoIntake:
            self.next_state("waitForBall")

    @feedback()
    def isInRange(self):
        return self.sonic.getInches() <= ULTRASONIC_DISTANCE_INCHES

    @feedback()
    def isOnTarget(self):
        if yaw := self.vision.getFilteredGoalYaw():
            return abs(yaw) < self.target_tolerance

    @feedback()
    def isAtAzimuth(self):
        if self.targetAzimuth:
            return abs(self.gyro.getAngle() - self.targetAzimuth) < self.target_tolerance 

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
                calc_speed = (16.908 * range + 9282)
        else:
            calc_speed = (16.908 * range + 9282)
        if calc_speed:
            return calc_speed * self.flywheel_trim
        else:
            return 0

    def reset_pneumatics(self):
        self.intake.deactivateHold()
        self.shooter.raiseLaunch()
        self.intake.deactivateRetrieve()
        self.shooter.setFlywheelSpeeds(0)

    def raiseFlywheelTrim(self):
        self.flywheel_trim += self.trimIncrement

    def lowerFlywheelTrim(self):
        self.flywheel_trim -= self.trimIncrement

    def getFlywheelTrim(self):
        return self.flywheel_trim

    def commandToFire(self, mode):
        self.fireCommanded = mode

    def calcMovingShot(self, vX, vY, target_range, target_azimuth):
        print("--PARAMS--", vX, vY, target_azimuth)
        # TODO: calculate time of flight from range
        # and flywheel speed?
        tof = range / 200  # assuming 200 inches in one second

        vC = Vector2(vX * tof, vY * tof)
        vT = Vector2.from_polar((target_range, target_azimuth))

        vS = vT - vC
        # print(sV.as_polar())
        # return sV.as_polar()

        return vS.to_polar()

        # if vX == 0:
        #     # can't divide by 0
        #     chassisMove = math.copysign(math.pi/2, vY)
        # else:
        #     chassisMove = math.atan(vY/vX)
        # if vX < 0 and vY < 0:
        #     chassisMove = math.radians(180) - chassisMove
        # elif vX < 0 and vY > 0:
        #     chassisMove = math.radians(180) + chassisMove
        # if chassisMove == 0:
        #     if vX < 0:
        #         chassisMove = math.pi
        #     else:
        #         chassisMove = 0
        # print("Chassis Move Direction: ", math.degrees(chassisMove))

        # #angleB = math.copysign(chassisMove - math.radians(heading), -heading
        # angleB = math.pi - chassisMove - math.radians(heading)
        # # if heading > -90 and heading < 90:
        # #     angleB = -angleB

        # print("Angle B: ", math.degrees(angleB))
        # sideA = math.sqrt(vX**2 + vY**2)
        # sideC = range
        # sideB = math.sqrt(sideA**2 + sideC**2 - (2 * sideA * sideC * math.cos(angleB)))
        # #sin(angleA)/sideA = (sin(angleB) / sideB)
        # angleA = math.degrees(math.asin((math.sin(angleB) / sideB) * sideA))
        # print('Angle offset: {}, calculated range: {}'.format(angleA, sideB))


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
