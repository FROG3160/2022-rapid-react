from ctre import (
    FeedbackDevice,
    WPI_TalonFX,
    CANCoder,
    TalonFXInvertType,
    ControlMode,
    SensorInitializationStrategy,
    AbsoluteSensorRange,
)
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
import math


# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.MotionMagic
# TODO: ensure we are using correct units
kMaxFeetPerSec = 16.3  # taken from SDS specs for MK4i-L2
# kMaxFeetPerSec = 18 # taken from SDS specks for MK4-L3
kMaxMetersPerSec = kMaxFeetPerSec * 0.3038
# how quickly to rotate the robot
kMaxRevolutionsPerSec = 2
kMaxRadiansPerSec = kMaxRevolutionsPerSec * 2 * math.pi


class SwerveModule:
    drive: WPI_TalonFX
    steer: WPI_TalonFX
    encoder: CANCoder
    location: Translation2d

    def __init__(self):
        # set initial states for the component
        self.velocity = 0
        # TODO: set angle to sensed angle from CANCoder?
        self.angle = 0
        self.enabled = False

    def disable(self):
        self.enabled = False
        self.steer.disable()
        self.drive.disable()

    def enable(self):
        self.enabled = True
        self.steer.enable()
        self.drive.enable()

    def getAngle(self):
        self.encoder.getAbsolutePosition()

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.
        # configure steer motor
        self.steer.setInverted(TalonFXInvertType.Clockwise)
        # define a remote CANCoder as Remote Feedback 0
        self.steer.configRemoteFeedbackFilter(self.encoder, 0)
        # configure Falcon to use Remote Feedback 0
        self.steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
        # configure drive motor
        self.drive.setInverted(TalonFXInvertType.Clockwise)
        # configure CANcoder
        # False = CCW rotation of magnet is positive
        self.encoder.configSensorDirection(False)
        # TODO: create variable and pass in from robot.py
        self.encoder.configMagnetOffset(
            0
        )  
        self.encoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        self.encoder.configAbsoluteSensorRange(
            AbsoluteSensorRange.Signed_PlusMinus180
        )

    def setState(self, state):
        # adjusts the speed and angle for minimal change
        # requires getting the current angle from the steer
        # motor and creating a Rotation2d object from it.
        self.state = state.optimize(
            state, Rotation2d.fromDegrees(self.getAngle())
        )

    def setAngle(self, angle):
        self.angle = angle

    def setVelocity(self, velocity):
        self.velocity = velocity

    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:
            self.steer.set(POSITION_MODE, self.angle)
            self.drive.set(VELOCITY_MODE, self.velocity)
        else:
            # TODO: decide whether we should
            # zero velocity on disable.
            self.steer.set(0)
            self.drive.set(0)


class SwerveChassis:

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveRearLeft: SwerveModule
    swerveRearRight: SwerveModule

    def __init__(self):
        self.enabled = False
        self.speeds = ChassisSpeeds(0, 0, 0)
        self.center = Translation2d(0, 0)

    def disable(self):
        self.enabled = False
        for module in [
            self.swerveFrontLeft,
            self.swerveFrontRight,
            self.swerveRearLeft,
            self.swerveRearRight,
        ]:
            module.disable()

    def drive(self, vX, vY, vT):
        # takes values from the joystick and translates it
        # into chassis movement
        self.speeds = ChassisSpeeds(
            vX * kMaxMetersPerSec, vY * kMaxMetersPerSec, vT * kMaxRadiansPerSec
        )

    def enable(self):
        self.enabled = True
        for module in self.modules:
            module.enable()

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.

        # creating a list of our SwerveModules so they can
        # always be used in the correct order.
        self.modules = [
            self.swerveFrontLeft,
            self.swerveFrontRight,
            self.swerveRearLeft,
            self.swerveRearRight,
        ]

        self.kinematics = SwerveDrive4Kinematics(
            # the splat operator (asterisk) below expands
            # the list into positional arguments for the
            # kinematics object.  We are taking the location
            # property of each swerveModule object and passing
            # it to SwerveDrive4Kinematics the order defined by
            # self.modules above.  Order is critical here.
            # We will receive back drive and steer values for
            # each SwerveModule in the same order we use here.
            *[m.location for m in self.modules]
        )

    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:
            # pass in the commanded speeds and center of rotation
            # and get back the speed and angle values for each module
            module_states = self.kinematics.toSwerveModuleStates(
                self.speeds, self.center
            )
            # normalizing wheel speeds so they don't exceed the
            # maximum defined speed
            module_states = self.kinematics.normalizeWheelSpeeds(
                module_states, kMaxMetersPerSec
            )
            for module in self.modules:
                module.setState(module_states.pop(0))

        else:
            pass
