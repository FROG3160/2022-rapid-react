#!/usr/bin/env python3
from ctre import WPI_CANCoder, WPI_TalonFX, CANifier
import magicbot
from magicbot import feedback
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
from components.sensors import FROGGyro, FROGdar, FROGsonic
from components.shooter import FROGShooter, Flywheel, Intake
from components.vision import FROGVision
from components.common import Rescale
from components.shooter import ShooterControl


# robot characteristics
# we are specifying inches and dividing by 12 to get feet,
# giving us values that can be used with the fromFeet method
# to get a correct Translation2d object
trackwidth = 27.75 / 12  # feet between wheels side to side
wheelbase = 21.75 / 12  # feet between wheels front to back
kDeadzone = 0.2
joystickAxisDeadband = Rescale((-1, 1), (-1, 1), 0.15)
joystickTwistDeadband = Rescale((-1, 1), (-1, 1), 0.2)
# visionDeadband = Rescale((-1,1), (.25, .25))
CTRE_PCM = PneumaticsModuleType.CTREPCM
TARGET_CARGO = 0
TARGET_GOAL = 1


class FROGbot(magicbot.MagicRobot):
    """
    Initialize components here.
    """

    gyro: FROGGyro
    lidar: FROGdar
    vision: FROGVision
    swerveChassis: SwerveChassis
    shooter: FROGShooter
    intake: Intake
    sonic: FROGsonic

    firecontrol: ShooterControl

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveBackLeft: SwerveModule
    swerveBackRight: SwerveModule

    lowerFlywheel: Flywheel
    upperFlywheel: Flywheel

    rotationP = magicbot.tunable(0.3)
    rotationI = magicbot.tunable(0.0)
    rotationD = magicbot.tunable(0.04)

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

        self.swerveFrontLeft_steerOffset = 18.5449219 #13.008
        self.swerveFrontRight_steerOffset = 174.023438 #171.914
        self.swerveBackLeft_steerOffset = 23.0273438 #22.764
        self.swerveBackRight_steerOffset = -43.41797 #-43.242

        # flywheel motors
        self.lowerFlywheel_motor = WPI_TalonFX(41)
        self.upperFlywheel_motor = WPI_TalonFX(42)

        # TODO:  Add in CANdle on channel 35

        # CANifier for LIDAR
        self.lidar_canifier = CANifier(36)
        self.sonic_cargoUltrasonic = wpilib.AnalogInput(0)
        self.sonic_mm = 25.4
        self.sonic_mv = 9.8

        # PCM
        #self.pcm = PneumaticsControlModule()
        # Solenoids for shooter
        self.intake_retrieve = Solenoid(CTRE_PCM, 2)
        self.intake_hold = Solenoid(CTRE_PCM, 1)
        self.shooter_launch = Solenoid(CTRE_PCM, 0)

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

        self.driverstation = DriverStation

        self.autoTargeting = False
        self.objectTargeted = TARGET_GOAL

        self.xOrig = self.yOrig = self.tOrig = 0

        self.rotationController = PIDController(0, 0, 0)

    @feedback(key="Auto Targeting")
    def getAutoTargeting(self):
        return self.autoTargeting

    @feedback(key="Object Targeted")
    def getObjectTargeted(self):
        return ["Cargo", "Goal"][self.objectTargeted]

    @feedback(key="TargetX")
    def getTarget(self):
        return [
            self.vision.getCargoXAverage(),
            self.vision.getGoalXAverage(),
        ][self.objectTargeted]

    @feedback()
    def getControllerAxes(self):
        return self.xOrig, self.yOrig, self.tOrig

    def autonomousInit(self):
        pass

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.swerveChassis.enable()
        self.autoTargeting = False
        #self.firecontrol.engage()

    def getRotationPID(self, target):
        self.rotationController.setP(self.rotationP)
        self.rotationController.setI(self.rotationI)
        self.rotationController.setD(self.rotationD)
        return self.rotationController.calculate(target)

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

        if self.gunnerControl.getRightTriggerAxis() > 0.5:
            self.firecontrol.next_state("waitToFire")
            self.firecontrol.engage()

        if self.gunnerControl.getRightBumperPressed():
            self.firecontrol.next_state("grab")
            self.firecontrol.engage()

        if self.gunnerControl.getLeftBumperPressed():
            if self.firecontrol.is_executing:
                self.firecontrol.done()
            else:
                self.firecontrol.engage()

        # if self.gunnerControl.getRightBumperReleased():
        #     self.firecontrol.next_state("retrieve")

        # if self.gunnerControl.getLeftBumperPressed():
        #     self.firecontrol.next_state("hold")

        # if self.gunnerControl.getLeftBumperReleased():
        #     self.firecontrol.next_state("release")

        # toggles self.objectTargeted
        if self.gunnerControl.getBButtonReleased():
            self.objectTargeted = [TARGET_GOAL, TARGET_CARGO][
                self.objectTargeted
            ]

        # toggles targeting mode
        if self.gunnerControl.getXButtonReleased():
            self.autoTargeting = [True, False][self.autoTargeting]
            if self.autoTargeting:
                self.vision.deactivateDriverMode()
            else:
                self.vision.activateDriverMode()

        # allows driver to override targeting control of rotation
        if self.driveStick.getRawButton(2):
            self.overrideTargeting = True
        else:
            self.overrideTargeting = False

        self.xOrig = joystickAxisDeadband(self.driveStick.getFieldForward())
        self.yOrig = joystickAxisDeadband(self.driveStick.getFieldLeft())

        target = self.getTarget()

        if self.autoTargeting and target and not self.overrideTargeting:
            # self.tOrig = -(math.copysign(abs(visionDeadband(target)), target)) * .5
            self.tOrig = self.getRotationPID(target)
            # -math.copysign(abs(target * .5) + .15, target)
        else:
            new_twist = joystickTwistDeadband(
                self.driveStick.getFieldRotation()
            )
            self.tOrig = math.copysign(new_twist**2, new_twist)

        # Get driver controls
        vX, vY, vT = (
            math.copysign(self.xOrig**2, self.xOrig),
            math.copysign(self.yOrig**2, self.yOrig),
            self.tOrig,
        )

        if vX or vY or vT:
            self.swerveChassis.field_oriented_drive(vX, vY, vT)

        if self.driveStick.getTrigger():
            self.gyro.resetGyro()
            self.swerveChassis.field_oriented_drive(0, 0, 0)

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
