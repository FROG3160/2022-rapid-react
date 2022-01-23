#!/usr/bin/env python3

from ctre import CANCoder, WPI_TalonFX
import magicbot
import wpilib


class FROGbot(magicbot.MagicRobot):
    """
        Initialize components here.
    """


    def createObjects(self):
        """Create motors and inputs"""
        #Swerve drive motors
        self.driveLeftFront = WPI_TalonFX(11)
        self.driveRightFront = WPI_TalonFX(12)
        self.driveLeftRear = WPI_TalonFX(13)
        self.driveRightRear = WPI_TalonFX(14)
        #Swerve steer motors
        self.steerLeftFront = WPI_TalonFX(21)
        self.steerRightFront = WPI_TalonFX(22)
        self.steerLeftRear = WPI_TalonFX(23)
        self.steerRightRear = WPI_TalonFX(24)
        #Swerve steer encoders (canifier)
        self.encoderLeftFront = CANCoder(31) 
        self.encoderRightFront = CANCoder(32) 
        self.encoderLeftRear = CANCoder(33)
        self.encoderRightRear = CANCoder(34) 




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
