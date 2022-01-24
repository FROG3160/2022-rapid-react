#!/usr/bin/env python3

from ctre import CANCoder, WPI_TalonFX
import magicbot
import wpilib
from components.drivetrain import SwerveModule, SwerveChassis
from wpimath.geometry import Translation2d

# robot characteristics
# TODO: check to see if we need to convert to feet or meters
#   for the kinematics to work correctly
trackwidth = 27.75 # inches between wheels side to side
wheelbase = 21.75 # inches between wheels front to back


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
        self.swerveFrontLeft_encoder = CANCoder(31)
        self.swerveFrontRight_encoder = CANCoder(32)
        self.swerveRearLeft_encoder = CANCoder(33)
        self.swerveRearRight_encoder = CANCoder(34)
        # Swerve module locations
        self.swerveFrontLeft_location = Translation2d(trackwidth / 2, wheelbase / 2)
        self.swerveFrontRight_location = Translation2d(trackwidth / 2, -wheelbase / 2)
        self.swerveRearLeft_location = Translation2d(-trackwidth / 2, wheelbase / 2)
        self.swerveRearRight_location = Translation2d(-trackwidth / 2, -wheelbase / 2)

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""
        pass

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
