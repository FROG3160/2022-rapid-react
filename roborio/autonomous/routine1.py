from magicbot import AutonomousStateMachine, timed_state, state
from logging import Logger
import wpilib
import math
from components.drivetrain import SwerveChassis
from components.vision import FROGVision
from components.shooter import ShooterControl, Intake, FROGShooter
from components.sensors import FROGGyro
from components.common import Vector2, angleErrorToRotation
from components.led import FROGLED
from wpimath.trajectory import TrapezoidProfileRadians, TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.controller import (
    ProfiledPIDControllerRadians,
    PIDController,
    HolonomicDriveController,
)

ROTATION_FACTOR = 0.15

TARGET_CARGO = 0
TARGET_GOAL = 1


class HoldCargo(AutonomousStateMachine):
    MODE_NAME = "Hold Cargo"
    swerveChassis: SwerveChassis
    vision: FROGVision
    firecontrol: ShooterControl
    gyro: FROGGyro

    @state(first=True)
    def hold_ball(self):
        self.firecontrol.intake.activateHold()


class StraightBackMoveShoot(AutonomousStateMachine):
    MODE_NAME = "Straight Back Move Shoot"
    DEFAULT = True
    swerveChassis: SwerveChassis
    vision: FROGVision
    intake: Intake
    shooter: FROGShooter
    gyro: FROGGyro
    rotation = 0.1
    linear = 0.2
    flywheel_trim = 1.07
    target_tolerance = 3
    min_range = 112
    object_targeted = TARGET_GOAL

    @state(first=True)
    def holdBall(self, initial_call):
        if initial_call:
            # clamp onto ball
            self.shooter.dropLaunch()
            self.intake.activateHold()
        self.next_state("waitForGoal")

    @state()
    def waitForGoal(self):
        self.swerveChassis.drive(-0.25, 0, 0)
        if range := self.vision.getRangeInches():
            if range > self.min_range:
                self.swerveChassis.field_oriented_drive(0, 0, 0)
                self.next_state("rotateToGoal")

    @state()
    def rotateToGoal(self):

        self.flyspeed = self.calculateFlywheelSpeed()
        self.shooter.setFlywheelSpeeds(self.flyspeed)

        # get angle for vT
        targetOffset = self.vision.getFilteredGoalYaw()
        vT = self.calculateT(targetOffset)

        if self.isRotatingSlow() and self.isOnTarget():
            self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state("waitForFlywheel")
        else:
            self.swerveChassis.field_oriented_drive(0, 0, vT)

    @state()
    def rotateToBall(self):
        pass

    @timed_state(duration=5, must_finish=True, next_state="fire")
    def waitForFlywheel(self, initial_call):
        if initial_call:
            self.flyspeed = self.calculateFlywheelSpeed()
            self.shooter.setFlywheelSpeeds(self.flyspeed)

        if not self.flyspeed == 0:
            if (
                self.shooter.isReady()
                and self.isOnTarget()
            ):
                self.next_state("waitToFire")

    @timed_state(duration=0.25, must_finish=True, next_state="fire")
    def waitToFire(self):
        pass

    @timed_state(duration=1, must_finish=True, next_state="finish")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.shooter.raiseLaunch()

    # @state()
    # def rotateToBall(self):
        
    @state()
    def finish(self):
        self.flyspeed = 0
        self.shooter.setFlywheelSpeeds(self.flyspeed)
        self.swerveChassis.field_oriented_drive(0, 0, 0)

    def getSelectedTargetYaw(self):
        return [
            self.vision.getFilteredCargoYaw(),
            self.vision.getFilteredGoalYaw(),
        ][self.objectTargeted]

    def calculateT(self, targetOffset):
        vT = 0
        if targetOffset:
            if self.isOnTarget():
                self.vT = 0
            else:
                self.vT = angleErrorToRotation(-targetOffset)
        else:
            self.flyspeed = 0
            self.shooter.setFlywheelSpeeds(self.flyspeed)
            if self.gyro.getOffsetYaw() > 90:
                vT = -self.rotation
            elif self.gyro.getOffsetYaw() < -90:
                vT = self.rotation
        return vT

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

    def isRotatingSlow(self):
        return abs(self.gyro.getRotationDPS()) < 0.1

    def isMoveXSlow(self):
        return abs(self.swerveChassis.getChassisX_FPS()) < 0.2

    def isMoveYSlow(self):
        return abs(self.swerveChassis.getChassisY_FPS()) < 0.2

    def isOnTarget(self):
        if yaw := self.vision.getFilteredGoalYaw():
            return abs(yaw) < self.target_tolerance
