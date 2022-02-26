import wpilib
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry 
from components.driverstation import FROGStick, FROGBoxGunner
from components.sensors import FROGGyro, FROGdar
from components.shooter import FROGShooter, Flywheel, Intake
from components.drivetrain import SwerveChassis
from components import drivetrain
from wpimath import trajectory, controller, geometry, kinematics
from wpimath.trajectory import constraint, TrapezoidProfileRadians, TrapezoidProfile
import math
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from components.drivetrain import SwerveModule, SwerveChassis


class trajectoryController:
    swerveChassis: SwerveChassis

    
    def __init__(self):
        
        self.pose = Pose2d(Translation2d(2,2), Rotation2d(0))
        self.maxVelocity = 4.96824
        self.maxAcceleration = 2.48412
        radianValueFor10dgrs = 0.17453293
        self.Timer = wpilib.Timer()
        self.Timer.start()
        
       
    def setup(self):

        # TrajectoryConfig
        self.trajectoryConfig = trajectory.TrajectoryConfig(self.maxVelocity, self.maxAcceleration)
        
        self.trajectoryConfig.setKinematics(self.swerveChassis.kinematics)
        
        # Holonomic (Swerve) Controller
        Constraint = trajectory.TrapezoidProfileRadians.Constraints(self.maxVelocity, self.maxAcceleration)
        xController = controller.PIDController(1, 0, 0, 0.02)
        yController = controller.PIDController(1, 0, 0, 0.02)
        angleController = controller.ProfiledPIDControllerRadians(1, 0, 0, Constraint, 0.02)
        angleController.atGoal()
        angleController.atSetpoint()
        angleController.calculate(0)
        angleController.enableContinuousInput(-1, 1)
        
        self.swerveController = controller.HolonomicDriveController(xController, yController, angleController)
 
        # TrajectoryGenerate
        self.trajectory1 =  trajectory.TrajectoryGenerator.generateTrajectory(
			geometry.Pose2d(Translation2d(0, 0), geometry.Rotation2d.fromDegrees(0)), # Starting position
			[geometry.Translation2d(2,1), geometry.Translation2d(3,2), geometry.Translation2d(2,3), geometry.Translation2d(1,2)], # Waypoints
			geometry.Pose2d(0, 0, geometry.Rotation2d.fromDegrees(0)), # Ending position
            self.trajectoryConfig)
  
    
    def updatedChassisSpeeds(self):

        sample = self.trajectory1.sample(self.Timer.get())
        adjustedSpeeds = self.swerveController.calculate(self.swerveChassis.odometry.getPose(), sample, geometry.Rotation2d.fromDegrees(0))
        self.swerveChassis.setChassisSpeeds(adjustedSpeeds)
        
    