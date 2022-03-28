#!/usr/bin/env python3
from ctre import WPI_CANCoder, WPI_TalonFX, CANifier
import magicbot
from magicbot import feedback, tunable
import wpilib
import math
from wpilib import (
    PneumaticsControlModule,
    Solenoid,
    PneumaticsModuleType,
    DriverStation,
)
from components.drivetrain import SwerveModule, SwerveChassis
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import (
    PIDController,
    HolonomicDriveController,
    ProfiledPIDControllerRadians,
)
from components.driverstation import FROGStick, FROGBoxGunner
from components.sensors import FROGGyro, FROGdar, FROGsonic, FROGColor
from components.shooter import FROGShooter, Flywheel, Intake
from components.vision import FROGVision, LC_Y_div
from components.common import Rescale
from components.shooter import ShooterControl
from components.climber import FROGLift
from components.led import FROGLED


# robot characteristics
# we are specifying inches and dividing by 12 to get feet,
# giving us values that can be used with the fromFeet method
# to get a correct Translation2d object
trackwidth = 27.75 / 12  # feet between wheels side to side
wheelbase = 21.75 / 12  # feet between wheels front to back
kDeadzone = 0.2
joystickAxisDeadband = Rescale((-1, 1), (-1, 1), 0.15)
joystickTwistDeadband = Rescale((-1, 1), (-1, 1), 0.2)
visionTwistDeadband = Rescale((-1, 1), (-0.35, 0.35))


# visionDeadband = Rescale((-1,1), (.25, .25))
CTRE_PCM = PneumaticsModuleType.CTREPCM
TARGET_CARGO = 0
TARGET_GOAL = 1

FIELD_ORIENTED = 0  # drive orientation
ROBOT_ORIENTED = 1
AUTO_DRIVE = 0  # drive mode
MANUAL_DRIVE = 1


class FROGbot(magicbot.MagicRobot):
    """
    Initialize components here.
    """

    firecontrol: ShooterControl
    lift: FROGLift

    gyro: FROGGyro
    # lidar: FROGdar
    color: FROGColor
    vision: FROGVision
    swerveChassis: SwerveChassis
    shooter: FROGShooter
    intake: Intake
    sonic: FROGsonic

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveBackLeft: SwerveModule
    swerveBackRight: SwerveModule

    lowerFlywheel: Flywheel
    upperFlywheel: Flywheel

    # driverstation: DriverStation

    rotationFactor = tunable(0.15)
    speedFactor = tunable(0.075)

    testTwist = tunable(0.0)
    testX = tunable(0.0)
    testY = tunable(0.0)

    def allianceColor(self):

        self.driverstation.getAlliance()

    def createObjects(self):
        """Create motors and inputs"""
        # Swerve drive motors
        self.swerveFrontLeft_drive = WPI_TalonFX(11)
        self.swerveFrontRight_drive = WPI_TalonFX(12)
        self.swerveBackLeft_drive = WPI_TalonFX(13)
        self.swerveBackRight_drive = WPI_TalonFX(14)
        # Swerve steer motors
        self.swerveFrontLeft_steer = WPI_TalonFX(21)
        self.swerveFrontRight_steer = WPI_TalonFX(22)
        self.swerveBackLeft_steer = WPI_TalonFX(23)
        self.swerveBackRight_steer = WPI_TalonFX(24)
        # Swerve steer encoders (canifier)
        self.swerveFrontLeft_encoder = WPI_CANCoder(31)
        self.swerveFrontRight_encoder = WPI_CANCoder(32)
        self.swerveBackLeft_encoder = WPI_CANCoder(33)
        self.swerveBackRight_encoder = WPI_CANCoder(34)
        # Swerve module locations
        # TODO: move to swerveChassis?
        self.swerveFrontLeft_location = Translation2d.fromFeet(
            wheelbase / 2,
            trackwidth / 2,
        )
        self.swerveFrontRight_location = Translation2d.fromFeet(
            wheelbase / 2,
            -trackwidth / 2,
        )
        self.swerveBackLeft_location = Translation2d.fromFeet(
            -wheelbase / 2,
            trackwidth / 2,
        )
        self.swerveBackRight_location = Translation2d.fromFeet(
            -wheelbase / 2,
            -trackwidth / 2,
        )

        self.swerveFrontLeft_steerOffset = (
            133.330078  # 136.143  # 26.807  # 21.796875  # 18.5449219 #13.008
        )
        self.swerveFrontRight_steerOffset = (
            -150.380859  # -146.602
        )  # 178.0664  # 177.1875  # 174.023438 #171.914
        self.swerveBackLeft_steerOffset = (
            3.95507813  # -0.527  # 31.9921875  # 23.0273438  # 22.764
        )
        self.swerveBackRight_steerOffset = (
            -150.117188  # -140.361
        )  # -43.33008  # -43.41797  # -43.242

        # flywheel motors
        self.lowerFlywheel_motor = WPI_TalonFX(41)
        self.upperFlywheel_motor = WPI_TalonFX(42)

        self.lift_stage1extend = WPI_TalonFX(51)
        # self.lift_stage1tilt = WPI_TalonFX(52)
        # self.lift_stage2extend = WPI_TalonFX(53)

        # TODO:  Add in CANdle on channel 35
        self.led = FROGLED(35)

        # CANifier for LIDAR
        # self.lidar_canifier = CANifier(36)
        self.sonic_cargoUltrasonic = wpilib.AnalogInput(0)
        self.sonic_mm = 25.4
        self.sonic_mv = 9.8

        # PCM
        # self.pcm = PneumaticsControlModule()
        # Solenoids for shooter
        self.intake_retrieve = Solenoid(CTRE_PCM, 2)
        self.intake_hold = Solenoid(CTRE_PCM, 1)
        self.shooter_launch = Solenoid(CTRE_PCM, 0)

        # self.lift_stage1claw = Solenoid(CTRE_PCM, 3)  # grabber - hook
        # self.lift_stage2tilt = Solenoid(CTRE_PCM, 4)  # arm 2 tilt
        # self.lift_stage3tilt = Solenoid(CTRE_PCM, 5)  # arm 3 tilt
        # self.lift_stage3release = Solenoid(CTRE_PCM, 6)  # pin release

        # self.vision_driverstation = DriverStation

        # config for saitek joystick
        # self.driveStick = FROGStick(0, 0, 1, 3, 2)
        # config for Logitech Extreme 3D
        self.driveStick = FROGStick(0, 0, 1, 2, 3)
        self.gunnerControl = FROGBoxGunner(1)

        self.field = wpilib.Field2d()
        # simulation already places field data in SmartDashboard
        # so we need to keep this from overwriting that data
        # during simulation
        if not self.isSimulation():
            wpilib.SmartDashboard.putData(self.field)

        # self.driverstation = DriverStation

        # keep robot rotated to a target
        self.targetLock = True
        # autoDrive is for moving the robot toward a target,
        # particularly the balls/cargo
        self.autoDrive = False
        self.objectTargeted = TARGET_GOAL

        self.xOrig = self.yOrig = self.tOrig = 0

        self.vX = 0
        self.vY = 0
        self.vT = 0

    @feedback(key="Auto Drive")
    def getAutoDrive(self):
        return self.autoDrive

    @feedback(key="Target Lock")
    def getTargetLock(self):
        return self.targetLock

    @feedback(key="Auto Intake")
    def getAutoIntake(self):
        return self.firecontrol.autoIntake

    @feedback(key="Auto Fire")
    def getAutoFire(self):
        return self.firecontrol.autoFire

    @feedback(key="Object Targeted")
    def getObjectTargeted(self):
        return ["CARGO", "GOAL"][self.objectTargeted]

    @feedback(key="TargetX")
    def getSelectedTargetX(self):
        return [
            self.vision.getFilteredCargoX(),
            self.vision.getFilteredGoalX(),
        ][self.objectTargeted]

    @feedback(key="TargetYaw")
    def getSelectedTargetYaw(self):
        return [
            self.vision.getFilteredCargoYaw(),
            self.vision.getFilteredGoalYaw(),
        ][self.objectTargeted]

    @feedback(key="TargetY")
    def getSelectedTargetY(self):
        return [
            self.vision.getFilteredCargoY(),
            self.vision.getFilteredGoalY(),
        ][self.objectTargeted]

    @feedback()
    def getControllerAxes(self):
        return self.xOrig, self.yOrig, self.tOrig

    @feedback()
    def getVX(self):
        return self.vX

    @feedback()
    def getVY(self):
        return self.vY

    @feedback()
    def getVT(self):
        return self.vT

    def autonomousInit(self):
        self.swerveChassis.enable()
        pass

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.swerveChassis.enable()
        self.autoDrive = False
        self.vision.setup()
        if self.firecontrol.getBallColor() is not None:
            self.firecontrol.next_state("holdBall")
            self.objectTargeted = TARGET_GOAL
        else:
            self.objectTargeted = TARGET_CARGO

        # self.firecontrol.engage()

    # def getRotationPID(self, target):
    #     self.rotationController.setP(self.rotationP)
    #     self.rotationController.setI(self.rotationI)
    #     self.rotationController.setD(self.rotationD)
    #     return self.rotationController.calculate(target)

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        # if not self.firecontrol.is_executing:
        #     self.firecontrol.engage()

        # Get gunner controls
        if self.gunnerControl.getYButtonReleased():
            self.shooter.incrementFlywheelSpeeds()
        if self.gunnerControl.getAButtonReleased():
            self.shooter.decrementFlywheelSpeeds()
        # if self.gunnerControl.getBButtonReleased():
        #     self.shooter.lowerFlywheel.incrementSpeed()
        # if self.gunnerControl.getXButtonReleased():
        #     self.shooter.lowerFlywheel.decrementSpeed()

        if self.gunnerControl.getLeftBumperPressed():
            # toggle autoIntake between true and false
            # if False, autoIntake is set to True, etc.
            self.firecontrol.autoIntake = [True, False][
                self.firecontrol.autoIntake
            ]

        if self.gunnerControl.getRightBumperPressed():
            self.firecontrol.autoFire = [True, False][self.firecontrol.autoFire]

        if (
            self.gunnerControl.getLeftTriggerAxis() > 0.5
            and not self.firecontrol.autoIntake
        ):
            if not self.firecontrol.is_executing:
                self.firecontrol.next_state_now("grab")
            self.firecontrol.engage()

        if self.gunnerControl.getRightTriggerAxis() > 0.5:
            self.firecontrol.flywheel_speed = 10500
            self.firecontrol.fireCommanded = True
            self.firecontrol.next_state_now("waitForFlywheel")
            self.firecontrol.engage()
        else:
            self.firecontrol.fireCommanded = False
            self.firecontrol.flywheel_speed = 0

        if self.firecontrol.autoIntake or self.firecontrol.autoFire:
            self.firecontrol.engage()

        # if self.gunnerControl.getRightBumperReleased():
        #     self.firecontrol.next_state("retrieve")

        # if self.gunnerControl.getLeftBumperPressed():
        #     self.firecontrol.next_state("hold")

        # if self.gunnerControl.getLeftBumperReleased():
        #     self.firecontrol.next_state("release")
        #

        # toggles self.objectTargeted
        if self.gunnerControl.getBButtonReleased():
            self.objectTargeted = [TARGET_GOAL, TARGET_CARGO][
                self.objectTargeted
            ]

        # toggles targeting mode
        if self.gunnerControl.getXButtonReleased():
            self.targetLock = [True, False][self.targetLock]
            # self.autoDrive = [True, False][self.autoDrive]
            # if self.autoDrive:
            #     self.vision.deactivateDriverMode()
            # else:
            #     self.vision.activateDriverMode()

        if self.gunnerControl.getLeftY() < -0.5:
            self.lift.extendStage1()
        elif self.gunnerControl.getLeftY() > 0.5:
            self.lift.retractStage1()
        else:
            self.lift.stage1extend.set(0)

        # if self.gunnerControl.getRightY() < -0.5:
        #     self.lift.tiltStage1Forward()
        # elif self.gunnerControl.getRightY() > 0.5:
        #     self.lift.tiltStage1Back()
        # else:
        #     self.lift.stage2extend.set(0)

        pov = self.gunnerControl.get_debounced_POV()
        if pov == 0:
            self.firecontrol.raiseFlywheelTrim()
        elif pov == 180:
            self.firecontrol.lowerFlywheelTrim()

        # allows driver to override targeting control of rotation
        if self.driveStick.getRawButton(2):
            self.overrideTargeting = True
        else:
            self.overrideTargeting = False

        self.xOrig = joystickAxisDeadband(self.driveStick.getFieldForward())
        self.yOrig = joystickAxisDeadband(self.driveStick.getFieldLeft())

        # targetX = self.getSelectedTargetX()
        targetY = self.getSelectedTargetY()
        targetYaw = self.getSelectedTargetYaw()

        # ! If we are in autodrive and we have a target, use it
        # ! otherwise use manual drive
        # ! autodrive will need to not use field-oriented
        # ! we also have auto rotate to target.... call target lock?
        # ! for targeting the goal, we will most likely want to
        # !   allow the joystick to control X and Y, get rotation from
        # !   targeting.

        self.vX = self.vY = self.vT = 0
        targetAngle = None

        if self.driveStick.getTrigger():
            self.autoDrive = True
        else:
            self.autoDrive = False

        # if self.driveStick.getRawButtonPressed(7):
        #     self.lift.activateClaw()

        # if self.driveStick.getRawButtonPressed(8):
        #     self.lift.deactivateClaw()

        # determine twist/rotation
        if self.targetLock and targetYaw and not self.overrideTargeting:
            # self.tOrig = self.getRotationPID(target)
            # * self.rotationFactor
            # targetX = -visionTwistDeadband(targetX)
            # self.vT = math.copysign(targetX**2, targetX)
            # #self.vT = -math.copysign(targetX ** self.rotationFactor, targetX)
            targetAngle = self.gyro.getYaw() - targetYaw
        elif self.driveStick.getPOV() > -1:
            targetAngle = -(self.driveStick.getPOV() - 180)
            # angleX = visionTwistDeadband((targetAngle - self.gyro.getYaw())/360)
            # self.vT = angleX
        else:
            new_twist = joystickTwistDeadband(
                self.driveStick.getFieldRotation()
            )
            if not new_twist == 0:
                self.vT = math.copysign(new_twist**3, new_twist)
            else:
                self.vT = 0

        # determine linear velocities
        if (
            self.autoDrive
            and targetY
            and not self.overrideTargeting
            and self.objectTargeted == TARGET_CARGO
        ):
            targetY = (targetY + 1) / 1.25
            self.vX = -math.copysign(
                abs(
                    (((targetY + 1) / 2) * self.speedFactor) + self.speedFactor
                ),
                targetY,
            )
            self.driveMode = ROBOT_ORIENTED
            # self.driveController = AUTO_DRIVE
            self.vY = 0
        else:
            self.vX, self.vY = (
                math.copysign(self.xOrig**2, self.xOrig),
                math.copysign(self.yOrig**2, self.yOrig),
            )
            self.driveMode = FIELD_ORIENTED

        if self.vX or self.vY or self.vT or targetAngle:
            if self.driveMode == FIELD_ORIENTED:
                self.swerveChassis.field_oriented_drive(
                    self.vX, self.vY, self.vT, targetAngle
                )
            else:
                self.swerveChassis.drive(self.vX, self.vY, self.vT, targetAngle)
        else:
            self.swerveChassis.field_oriented_drive(0, 0, 0)

        if self.driveStick.getRawButtonPressed(3):
            self.gyro.resetGyro()
            self.swerveChassis.resetRemoteEncoders()
            self.swerveChassis.field_oriented_drive(0, 0, 0)

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        self.led.ColorChangeBlue()
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
