#!/usr/bin/env python3

import magicbot
import wpilib


class FROGbot(magicbot.MagicRobot):
    """
        Initialize components with createObjects.
    """
    def createObjects(self):
        """Create motors and inputs"""
        pass

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
