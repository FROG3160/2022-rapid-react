#!/usr/bin/env python3

from ctre import WPI_CANCoder, WPI_TalonFX
import magicbot
import wpilib
from components.drivetrain import SwerveModule, SwerveChassis
from wpimath.geometry import Translation2d
from wpilib import Joystick

# robot characteristics
# we are specifying inches and dividing by 12 to get feet,
# giving us values that can be used with the fromFeet method
# to get a correct Translation2d object
trackwidth = 27.75 / 12  # feet between wheels side to side
wheelbase = 21.75 / 12  # feet between wheels front to back

kDeadzone = 0.05


class FROGbot(magicbot.MagicRobot):
    """
    Initialize components here.
    """

    swerveChassis: SwerveChassis

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveRearLeft: SwerveModule
    swerveRearRight: SwerveModule

    def createObjects(self):
        """Create motors and inputs"""
        # Swerve drive motors
        self.swerveFrontLeft_drive = WPI_TalonFX(11)
        self.swerveFrontRight_drive = WPI_TalonFX(12)
        self.swerveRearLeft_drive = WPI_TalonFX(13)
        self.swerveRearRight_drive = WPI_TalonFX(14)
        # Swerve steer motors
        self.swerveFrontLeft_steer = WPI_TalonFX(21)
        self.swerveFrontRight_steer = WPI_TalonFX(22)
        self.swerveRearLeft_steer = WPI_TalonFX(23)
        self.swerveRearRight_steer = WPI_TalonFX(24)
        # Swerve steer encoders (canifier)
        self.swerveFrontLeft_encoder = WPI_CANCoder(31)
        self.swerveFrontRight_encoder = WPI_CANCoder(32)
        self.swerveRearLeft_encoder = WPI_CANCoder(33)
        self.swerveRearRight_encoder = WPI_CANCoder(34)
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
        self.swerveRearLeft_location = Translation2d.fromFeet(
            -wheelbase / 2,
            trackwidth / 2,
        )
        self.swerveRearRight_location = Translation2d.fromFeet(
            -wheelbase / 2,
            -trackwidth / 2,
        )

        self.swerveFrontLeft_steerOffset = 0.0
        self.swerveFrontRight_steerOffset = 0.0
        self.swerveRearLeft_steerOffset = 0.0
        self.swerveRearRight_steerOffset = 0.0

        self.driveStick = Joystick(0)

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.swerveChassis.enable()
        pass

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""
        vX, vY, vT = (
            -(self.driveStick.getY(), 0)[
                abs(self.driveStick.getY()) < kDeadzone
            ],
            -(self.driveStick.getX(), 0)[
                abs(self.driveStick.getX()) < kDeadzone
            ],
            -(self.driveStick.getRawAxis(3), 0)[
                abs(self.driveStick.getRawAxis(3)) < kDeadzone
            ],
        )
        self.swerveChassis.drive(vX, vY, vT)

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
