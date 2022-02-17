#!/usr/bin/env python3

from asyncio.tasks import _T1
from ctre import WPI_CANCoder, WPI_TalonFX, CANifier
import magicbot
from pyparsing import trace_parse_action
import wpilib
from components import drivetrain
from wpilib import PneumaticsControlModule, Solenoid, PneumaticsModuleType 
from components.drivetrain import SwerveModule, SwerveChassis
import wpimath
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics,SwerveDrive4Odometry
from components.driverstation import FROGStick, FROGBoxGunner
from components.sensors import FROGGyro, FROGdar
from components.shooter import FROGShooter, Flywheel, Intake
from components import drivetrain
from wpimath import trajectory, controller, geometry
from wpimath.trajectory import constraint, TrapezoidProfileRadians, TrapezoidProfile
import math
from wpimath.controller import PIDController, ProfiledPIDController, HolonomicDriveController

# robot characteristics
# we are specifying inches and dividing by 12 to get feet,
# giving us values that can be used with the fromFeet method
# to get a correct Translation2d object
trackwidth = 27.75 / 12  # feet between wheels side to side
wheelbase = 21.75 / 12  # feet between wheels front to back

kDeadzone = 0.05

CTRE_PCM = PneumaticsModuleType.CTREPCM


class FROGbot(magicbot.MagicRobot):
    """
    Initialize components here.
    """

    gyro: FROGGyro
    lidar: FROGdar
    swerveChassis: SwerveChassis
    shooter: FROGShooter
    intake: Intake

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveBackLeft: SwerveModule
    swerveBackRight: SwerveModule

    lowerFlywheel: Flywheel
    upperFlywheel: Flywheel

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

        self.swerveFrontLeft_steerOffset = 0.0
        self.swerveFrontRight_steerOffset = 0.0
        self.swerveBackLeft_steerOffset = 0.0
        self.swerveBackRight_steerOffset = 0.0

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

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.swerveChassis.enable()
        pass

    def autonomousInit(self):
        self.vision.setAllianceColor(self,drivetrain.getAlliance)
        self.automodes.start()
        
        # Defining some variables
        self.pose = Pose2d(Translation2d(2,2), Rotation2d(0))
        maxVelocity = 4.96824
        maxAcceleration = 2.48412
        radianValueFor10dgrs = 0.17453293
        
        # TrajectoryConfig
        trajectoryConfig = trajectory.TrajectoryConfig(maxVelocity, maxAcceleration)
        MinMaxAcceleration = constraint.TrajectoryConstraint.minMaxAcceleration(Pose2d, radianValueFor10dgrs, 2)
        trajectoryConfig.setKinematics(SwerveDrive4Kinematics)
        trajectoryConfig.setStartVelocity(2)
        trajectoryConfig.setEndVelocity(0)
        trajectoryConfig.setReversed(False)
        trajectoryConfig.startVelocity()
        trajectoryConfig.endVelocity()
        trajectoryConfig.isReversed()
        
        #TrajectoryGenerate
        trajectoryGenerator =  trajectory.TrajectoryGenerator.generateTrajectory(
			geometry.Pose2d(0, 0, geometry.Rotation2d.fromDegrees(0)), # Starting position
			[geometry.Translation2d(2,1), geometry.Translation2d(3,2), geometry.Translation2d(2,3), geometry.Translation2d(1,2)], # Waypoints
			geometry.Pose2d(0, 0, geometry.Rotation2d.fromDegrees(0)), # Ending position
            trajectoryConfig)

     
        #HolonomicDriveController
        holonomicDC = controller.HolonomicDriveController(controller.PIDController(1, 0, 0),
        controller.PIDController(1, 0, 0), controller.ProfiledPIDController(1, 0, 0))
        holonomicDC.atReference()
        holonomicDC.calculate(Pose2d, maxAcceleration, Rotation2d)
        holonomicDC.setEnabled(True)
        holonomicDC.setTolerance(Pose2d)




             #Called on each iteration of the control loop

        # Get gunner controlsotDimensions])

        
    def teleopPeriodic(self):
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
            (self.driveStick.getFieldForward(), 0)[
                abs(self.driveStick.getFieldForward()) < kDeadzone
            ],
            (self.driveStick.getFieldLeft(), 0)[
                abs(self.driveStick.getFieldLeft()) < kDeadzone
            ],
            (self.driveStick.getFieldRotation(), 0)[
                abs(self.driveStick.getFieldRotation()) < kDeadzone
            ],
        )
        self.swerveChassis.field_oriented_drive(vX, vY, vT)

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
