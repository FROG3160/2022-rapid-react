#!/usr/bin/env python3

from ctre import CANCoder, WPI_TalonFX
import magicbot
import wpilib
from components.drivetrain import SwerveModule, SwerveChassis


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
        #Swerve drive motors
        self.swerveFrontLeft_drive = WPI_TalonFX(11)
        self.swerveFrontRight_drive = WPI_TalonFX(12)
        self.swerveRearLeft_drive = WPI_TalonFX(13)
        self.swerveRearRight_drive = WPI_TalonFX(14)
        #Swerve steer motors
        self.swerveFrontLeft_steer = WPI_TalonFX(21)
        self.swerveFrontRight_steer = WPI_TalonFX(22)
        self.swerveRearLeft_steer = WPI_TalonFX(23)
        self.swerveRearRight_steer = WPI_TalonFX(24)
        #Swerve steer encoders (canifier)
        self.swerveFrontLeft_encoder = CANCoder(31) 
        self.swerveFrontRight_encoder = CANCoder(32)
        self.swerveRearLeft_encoder = CANCoder(33)
        self.swerveRearRight_encoder = CANCoder(34)




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
