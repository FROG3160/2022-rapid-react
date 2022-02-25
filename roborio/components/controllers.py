from components.drivetrain import SwerveModule, SwerveChassis, ChassisSpeeds
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



class trajectoryController:

    
    def __init__(self):
        
        self.pose = Pose2d(Translation2d(2,2), Rotation2d(0))
        maxVelocity = 4.96824
        maxAcceleration = 2.48412
        radianValueFor10dgrs = 0.17453293
        Translation2d
        
        # TrajectoryConfig
        self.trajectoryConfig = trajectory.TrajectoryConfig(maxVelocity, maxAcceleration)
        
        self.trajectoryConfig.setKinematics(self.swerveChassis.kinematics)

        Constraint = trajectory.TrapezoidProfileRadians.Constraints(maxVelocity, maxAcceleration)
        xController = controller.PIDController(1, 0, 0, 0.02)
        yController = controller.PIDController(1, 0, 0, 0.02)
        angleController = controller.ProfiledPIDControllerRadians(1, 0, 0, Constraint, 0.02)
        # According to the docs it says that we need all of the below methods, but the example code has only the enableContinuousInput method
        angleController.atGoal()
        angleController.atSetpoint()
        angleController.calculate(0)
        angleController.enableContinuousInput(-1, 1)
        

        
        self.swerveController = controller.HolonomicDriveController(xController, yController, angleController)

        self.Timer = wpilib.Timer()
        self.Timer.start()

    def setTrajectory(self):

        #TrajectoryGenerate
        self.trajectoryGenerator =  trajectory.TrajectoryGenerator.generateTrajectory(
			geometry.Pose2d(Translation2d(0, 0), geometry.Rotation2d.fromDegrees(0)), # Starting position
			[geometry.Translation2d(2,1), geometry.Translation2d(3,2), geometry.Translation2d(2,3), geometry.Translation2d(1,2)], # Waypoints
			geometry.Pose2d(0, 0, geometry.Rotation2d.fromDegrees(0)), # Ending position
            self.trajectoryConfig)    
    

    def setChassisSpeeds(self):

        sample = self.trajectoryGenerator.sample(self.Timer.get())
        adjustedSpeeds = self.swerveController.calculate(self.swerveChassis.odometry.getPose(), sample, geometry.Rotation2d.fromDegrees(0))
        # Adjusted ChassisSpeeds to the Swerve Chassis
        self.swerveChassis.setChassisSpeeds(adjustedSpeeds)
        