#!/usr/bin/env python3

from ctre import WPI_CANCoder, WPI_TalonFX, CANifier
import magicbot
import wpilib

from wpilib import (
    PneumaticsControlModule,
    Solenoid,
    PneumaticsModuleType,
    DriverStation,
)
from components.drivetrain import SwerveModule, SwerveChassis
from wpimath.geometry import Translation2d
from components.driverstation import FROGStick, FROGBoxGunner
from components.sensors import FROGGyro, FROGdar
from components.shooter import FROGShooter, Flywheel, Intake
from components.vision import FROGVision
from components.common import Rescale


# robot characteristics
# we are specifying inches and dividing by 12 to get feet,
# giving us values that can be used with the fromFeet method
# to get a correct Translation2d object
trackwidth = 27.75 / 12  # feet between wheels side to side
wheelbase = 21.75 / 12  # feet between wheels front to back
kDeadzone = 0.2
joystickAxisDeadband = Rescale((-1, 1), (-1, 1), 0.15)
CTRE_PCM = PneumaticsModuleType.CTREPCM


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

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveBackLeft: SwerveModule
    swerveBackRight: SwerveModule

    lowerFlywheel: Flywheel
    upperFlywheel: Flywheel

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

        self.swerveFrontLeft_steerOffset = 13.008
        self.swerveFrontRight_steerOffset = -175.166
        self.swerveBackLeft_steerOffset = 22.764
        self.swerveBackRight_steerOffset = -43.242

        # flywheel motors
        self.lowerFlywheel_motor = WPI_TalonFX(41)
        self.upperFlywheel_motor = WPI_TalonFX(42)

        # TODO:  Add in CANdle on channel 35

        # CANifier for LIDAR
        self.lidar_canifier = CANifier(36)

        # PCM
        self.pcm = PneumaticsControlModule(1)
        # Solenoids for shooter
        self.intake_retrieve = Solenoid(CTRE_PCM, 0)
        self.intake_hold = Solenoid(CTRE_PCM, 1)
        self.intake_launch = Solenoid(CTRE_PCM, 2)

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

    def autonomousInit(self):
        self.vision.setAllianceColor(self.driverstation.getAlliance())

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.swerveChassis.enable()
        self.vision.setAllianceColor(self.driverstation.getAlliance())

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        # Get gunner controls
        if self.gunnerControl.getYButtonReleased():
            self.shooter.incrementFlywheelSpeeds()
        if self.gunnerControl.getAButtonReleased():
            self.shooter.decrementFlywheelSpeeds()
        # if self.gunnerControl.getBButtonReleased():
        #     self.shooter.lowerFlywheel.incrementSpeed()
        # if self.gunnerControl.getXButtonReleased():
        #     self.shooter.lowerFlywheel.decrementSpeed()

        if self.gunnerControl.getLeftBumper():
            self.intake.activateHold()
        else:
            self.intake.deactivateHold()

        if self.gunnerControl.getRightBumper():
            self.intake.activateLaunch()
        else:
            self.intake.deactivateLaunch()

        # Get driver controls
        vX, vY, vT = (
            joystickAxisDeadband(self.driveStick.getFieldForward()),
            joystickAxisDeadband(self.driveStick.getFieldLeft()),
            joystickAxisDeadband(self.driveStick.getFieldRotation()) ** 3
        )
        if vX or vY or vT:
            self.swerveChassis.field_oriented_drive(vX, vY, vT)

        if self.driveStick.getTrigger():
            self.gyro.resetGyro()

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
