from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

from components.drivetrain import SwerveChassis
from components.vision import FROGVision
from components.shooter import ShooterControl
from components.sensors import FROGGyro
from components.common import Vector2

ROTATION_FACTOR = 0.15


class HoldCargo(AutonomousStateMachine):
    MODE_NAME = "Hold Cargo"
    DEFAULT = True
    swerveChassis: SwerveChassis
    vision: FROGVision
    firecontrol: ShooterControl
    gyro: FROGGyro

    @state(first=True)
    def hold_ball(self):
        self.firecontrol.intake.activateHold()

# class MoveThenShootBase(AutonomousStateMachine):
#     MODE_NAME = "Right Side Move Shoot"
#     swerveChassis: SwerveChassis
#     vision: FROGVision
#     firecontrol: ShooterControl
#     gyro: FROGGyro

#     @state(first=True)
#     def holdBall(self, initial_call):
#         if initial_call:
#             # clamp onto ball
#             self.firecontrol.intake.activateHold()
#         self.next_state("waitForGoal")

#     @timed_state(duration=2, next_state="finish")
#     def waitForGoal(self):
#         self.swerveChassis.field_oriented_drive(-0.25, -0.25, 0, 45)
#         if self.vision.hasGoalTargets:
#             # self.swerveChassis.field_oriented_drive(0, 0, 0)
#             self.next_state_now("rotateToTarget")

#     @state()
#     def rotateToTarget(self):
#         targetOffset = self.vision.getFilteredGoalYaw()

#         if targetOffset:
#             if not self.vision.getRangeInches() > 95:
#                 vX = -.25
#             else:
#                 vX = 0
#             targetAngle = self.gyro.getYaw() - targetOffset
#         else:
#             targetAngle = None
#             vX = 0
            
#         if self.firecontrol.isOnTarget() and vX == 0:
#             self.swerveChassis.field_oriented_drive(0, 0, 0)
#             self.next_state("waitForFlywheel")
#         else:
#             self.swerveChassis.drive(vX, 0, 0, targetAngle)

#     @timed_state(duration=5, must_finish=True, next_state="fire")
#     def waitForFlywheel(self, initial_call):
#         if initial_call:
#             self.flyspeed = self.firecontrol.calculateFlywheelSpeed()
#             self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)

#         if not self.flyspeed == 0:
#             if (
#                 self.firecontrol.shooter.isReady()
#                 and self.firecontrol.isOnTarget()
#             ):
#                 self.next_state("waitToFire")

#     @timed_state(duration=0.25, must_finish=True, next_state="fire")
#     def waitToFire(self):
#         pass

#     @timed_state(duration=1, must_finish=True, next_state="finish")
#     def fire(self, initial_call):
#         if initial_call:
#             # raise launch, self.intake.grab resets
#             self.firecontrol.shooter.raiseLaunch()

#     @state()
#     def finish(self):
#         self.flyspeed = 0
#         self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)
#         self.swerveChassis.field_oriented_drive(0, 0, 0)


class MoveAndShoot(AutonomousStateMachine):
    MODE_NAME = "Move And Shoot"
    swerveChassis: SwerveChassis
    vision: FROGVision
    firecontrol: ShooterControl
    gyro: FROGGyro

    @state(first=True)
    def holdBall(self, initial_call):
        if initial_call:
            # clamp onto ball
            self.firecontrol.intake.activateHold()
        self.next_state("waitForGoal")

    @timed_state(duration=2, next_state="finish")
    def waitForGoal(self):
        self.swerveChassis.field_oriented_drive(-0.10, 0.10, 0.25)
        if self.vision.hasGoalTargets:
            # self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state_now("rotateToTarget")

    @timed_state(duration=3, next_state="finish")
    def rotateToTarget(self):
        if targetYaw := self.vision.getFilteredGoalYaw():
            fovC = Vector2.from_chassis_speed(
                self.swerveChassis.current_speeds.vx,
                self.swerveChassis.current_speeds.vy,
                self.gyro.getYaw())
            newRange, newAzimuth = self.firecontrol.calcMovingShot(
                fovC.x,
                fovC.y,
                self.vision.getRangeInches(),
                self.gyro.getYaw() - targetYaw,

            )
            self.firecontrol.targetAzimuth = newAzimuth
            self.flyspeed = self.firecontrol.calculateFlywheelSpeed(newRange)
            self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)
            
            self.swerveChassis.field_oriented_drive(-0.1, 0.1, 0, newAzimuth)
        if (
            self.firecontrol.isAtAzimuth()
            and self.firecontrol.shooter.isReady()
        ):
            self.swerveChassis.field_oriented_drive(-0.1, 0.1, 0, newAzimuth)
            self.next_state("fire")

    @timed_state(duration=1, must_finish=True, next_state="finish")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.firecontrol.shooter.raiseLaunch()

    @state()
    def finish(self):
        self.flyspeed = 0
        self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)
        self.swerveChassis.field_oriented_drive(0, 0, 0)


class StraightBackMoveShoot(AutonomousStateMachine):
    MODE_NAME = "Straight Back Move Shoot"
    swerveChassis: SwerveChassis
    vision: FROGVision
    firecontrol: ShooterControl

    @state(first=True)
    def holdBall(self, initial_call):
        if initial_call:
            # clamp onto ball
            self.firecontrol.intake.activateHold()
        self.next_state("waitForGoal")

    @timed_state(duration=2, next_state="finish")
    def waitForGoal(self):
        self.swerveChassis.field_oriented_drive(-0.25, 0, 0)
        if self.vision.hasGoalTargets:
            self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state_now("rotateToTarget")

    @state()
    def rotateToTarget(self):
        targetOffset = self.vision.getFilteredGoalYaw()

        if targetOffset:
            if not self.vision.getRangeInches() > 95:
                vX = -.25
            else:
                vX = 0
            targetAngle = self.gyro.getYaw() - targetOffset
        else:
            targetAngle = None
            vX = 0
            
        if self.firecontrol.isOnTarget() and vX == 0:
            self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state("waitForFlywheel")
        else:
            self.swerveChassis.drive(vX, 0, 0, targetAngle)

    @timed_state(duration=5, must_finish=True, next_state="fire")
    def waitForFlywheel(self, initial_call):
        if initial_call:
            self.flyspeed = self.firecontrol.calculateFlywheelSpeed()
            self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)

        if not self.flyspeed == 0:
            if (
                self.firecontrol.shooter.isReady()
                and self.firecontrol.isOnTarget()
            ):
                self.next_state("waitToFire")

    @timed_state(duration=0.25, must_finish=True, next_state="fire")
    def waitToFire(self):
        pass

    @timed_state(duration=1, must_finish=True, next_state="finish")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.firecontrol.shooter.raiseLaunch()

    @state()
    def finish(self):
        self.flyspeed = 0
        self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)
        self.swerveChassis.field_oriented_drive(0, 0, 0)


class LeftSideMoveShoot(AutonomousStateMachine):
    MODE_NAME = "Left Side Move Shoot"
    swerveChassis: SwerveChassis
    vision: FROGVision
    firecontrol: ShooterControl
    gyro: FROGGyro

    @state(first=True)
    def holdBall(self, initial_call):
        if initial_call:
            # clamp onto ball
            self.firecontrol.intake.activateHold()
        self.next_state("waitForGoal")

    @timed_state(duration=2, next_state="finish")
    def waitForGoal(self):
        self.swerveChassis.field_oriented_drive(-0.25, 0.25, -0.25)
        if self.vision.hasGoalTargets:
            self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state_now("rotateToTarget")

    @state()
    def rotateToTarget(self):
        targetX = self.vision.getFilteredGoalX()
        if targetX:
            self.vT = -targetX * ROTATION_FACTOR
            self.swerveChassis.field_oriented_drive(0, 0, self.vT)
        if self.firecontrol.isOnTarget():
            self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state("waitForFlywheel")

    @timed_state(duration=5, must_finish=True, next_state="fire")
    def waitForFlywheel(self, initial_call):
        if initial_call:
            self.flyspeed = self.firecontrol.calculateFlywheelSpeed()
            self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)

        if not self.flyspeed == 0:
            if (
                self.firecontrol.shooter.isReady()
                and self.firecontrol.isOnTarget()
            ):
                self.next_state("waitToFire")

    @timed_state(duration=0.25, must_finish=True, next_state="fire")
    def waitToFire(self):
        pass

    @timed_state(duration=1, must_finish=True, next_state="finish")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.firecontrol.shooter.raiseLaunch()

    @state()
    def finish(self):
        self.flyspeed = 0
        self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)
        self.swerveChassis.field_oriented_drive(0, 0, 0)


class RightSideMoveShoot(AutonomousStateMachine):
    MODE_NAME = "Right Side Move Shoot"
    swerveChassis: SwerveChassis
    vision: FROGVision
    firecontrol: ShooterControl
    gyro: FROGGyro

    @state(first=True)
    def holdBall(self, initial_call):
        if initial_call:
            # clamp onto ball
            self.firecontrol.intake.activateHold()
        self.next_state("waitForGoal")

    @timed_state(duration=2, next_state="finish")
    def waitForGoal(self):
        self.swerveChassis.field_oriented_drive(-0.25, -0.25, 0, 45)
        if self.vision.hasGoalTargets:
            # self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state_now("rotateToTarget")

    @state()
    def rotateToTarget(self):
        targetOffset = self.vision.getFilteredGoalYaw()

        if targetOffset:
            if not self.vision.getRangeInches() > 95:
                vX = -.25
            else:
                vX = 0
            targetAngle = self.gyro.getYaw() - targetOffset
        else:
            targetAngle = None
            vX = 0
            
        if self.firecontrol.isOnTarget() and vX == 0:
            self.swerveChassis.field_oriented_drive(0, 0, 0)
            self.next_state("waitForFlywheel")
        else:
            self.swerveChassis.drive(vX, 0, 0, targetAngle)

    @timed_state(duration=5, must_finish=True, next_state="fire")
    def waitForFlywheel(self, initial_call):
        if initial_call:
            self.flyspeed = self.firecontrol.calculateFlywheelSpeed()
            self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)

        if not self.flyspeed == 0:
            if (
                self.firecontrol.shooter.isReady()
                and self.firecontrol.isOnTarget()
            ):
                self.next_state("waitToFire")

    @timed_state(duration=0.25, must_finish=True, next_state="fire")
    def waitToFire(self):
        pass

    @timed_state(duration=1, must_finish=True, next_state="finish")
    def fire(self, initial_call):
        if initial_call:
            # raise launch, self.intake.grab resets
            self.firecontrol.shooter.raiseLaunch()

    @state()
    def finish(self):
        self.flyspeed = 0
        self.firecontrol.shooter.setFlywheelSpeeds(self.flyspeed)
        self.swerveChassis.field_oriented_drive(0, 0, 0)
