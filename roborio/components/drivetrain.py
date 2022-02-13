from ctre import (
    FeedbackDevice,
    RemoteSensorSource,
    WPI_TalonFX,
    WPI_CANCoder,
    TalonFXInvertType,
    ControlMode,
    SensorInitializationStrategy,
    AbsoluteSensorRange,
    TalonFXConfiguration,
    CANCoderConfiguration,
    BaseTalonPIDSetConfiguration,
)
from wpilib import Field2d
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    ChassisSpeeds,
    SwerveModuleState,
)
import math
from magicbot import feedback
from .sensors import FROGGyro
from .common import DriveUnit


# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.Position
kMaxFeetPerSec = 8  # taken from SDS specs for MK4i-L2
# kMaxFeetPerSec = 18 # taken from SDS specks for MK4-L3
kMaxMetersPerSec = kMaxFeetPerSec * 0.3038
# how quickly to rotate the robot
kMaxRevolutionsPerSec = 1
kMaxRadiansPerSec = kMaxRevolutionsPerSec * 2 * math.pi

kWheelDiameter = 0.1016  # in meters  TODO: Confirm this value
kFalconTicksPerRotation = 2048
kFalconMaxRPM = 6380
kCANCoderTicksPerRotation = 4096
# the multiple of 10 in the divisor is to get to how many ticks per 100ms
# or 1/10th of a second.  Multiplying the speed of SwerveModuleState by
# this multiplier will give us the velocity to give the TalonFX in ticks/100ms
kVelocityMultiplier = kFalconTicksPerRotation / (10 * kWheelDiameter * math.pi)
kModuleDriveGearing = [(14.0 / 50.0), (27.0 / 17.0), (15.0 / 45.0)]
kModuleSteerGearing = []

# CANCoder base config
# Offset will be different for each module and will need to be
# set during SwerveModule.setup()
cfgSteerEncoder = CANCoderConfiguration()
cfgSteerEncoder.sensorDirection = False  # CCW spin of magnet is positive
cfgSteerEncoder.initializationStrategy = (
    SensorInitializationStrategy.BootToAbsolutePosition
)
cfgSteerEncoder.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180

# Steer Motor base config
# TODO: Tune and adjust PID
cfgSteerMotor = TalonFXConfiguration()
cfgSteerMotor.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder
cfgSteerMotor.primaryPID = BaseTalonPIDSetConfiguration(
    FeedbackDevice.RemoteSensor0
)
cfgSteerMotor.slot0.kP = 1.2
cfgSteerMotor.slot0.kI = 0.0001
cfgSteerMotor.slot0.kD = 0.0
cfgSteerMotor.slot0.kF = 0.0
cfgSteerMotor.slot0.allowableClosedloopError = 5

# Drive Motor base config
# TODO: Tune and adjust PID
cfgDriveMotor = TalonFXConfiguration()
cfgDriveMotor.initializationStrategy = SensorInitializationStrategy.BootToZero
cfgDriveMotor.primaryPID = BaseTalonPIDSetConfiguration(
    FeedbackDevice.IntegratedSensor
)
cfgDriveMotor.slot0.kP = 0.0
cfgDriveMotor.slot0.kI = 0.0
cfgDriveMotor.slot0.kD = 0.0
cfgDriveMotor.slot0.kF = 0.058


class SwerveModule:
    drive: WPI_TalonFX
    steer: WPI_TalonFX
    encoder: WPI_CANCoder
    location: Translation2d
    steerOffset: float

    def __init__(self):
        # set initial states for the component
        self.velocity = 0
        # TODO: set angle to sensed angle from CANCoder?
        self.angle = 0
        self.enabled = False
        self.state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        self.drive_unit = DriveUnit(
            kModuleDriveGearing,
            kFalconMaxRPM,
            kWheelDiameter,
            kFalconTicksPerRotation,
        )

    def disable(self):
        self.enabled = False

    @feedback
    def enable(self):
        self.enabled = True

    @feedback()
    def getEncoderPostion(self):
        self.encoder.getPosition()

    @feedback()
    def getEncoderAbsolutePosition(self):
        self.encoder.getAbsolutePosition()

    @feedback()
    def getStateFPS(self):
        return self.state.speed_fps

    @feedback()
    def getStateSpeed(self):
        return self.state.speed

    @feedback()
    def getCommandedTicks(self):
        return self.degreesToTicks(self.state.angle.degrees())

    @feedback()
    def getSteerPosition(self):
        return self.steer.getSelectedSensorPosition(0)

    def degreesToTicks(self, degrees: float) -> float:
        return degrees * (kCANCoderTicksPerRotation / 360)

    def ticksToDegrees(self, ticks: float) -> float:
        return ticks * (360 / kCANCoderTicksPerRotation)

    # TODO: Determine which way we want these
    # to read.  Right now they are inverted
    # to visually show positive angles to the
    # right (clockwise) to match the smartdashboard
    @feedback()
    def getCommandedDegrees(self):
        return -self.state.angle.degrees()

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.
        # configure steer motor

        # configure CANCoder
        # No worky: self.encoder.configAllSettings(cfgSteerEncoder)
        self.encoder.configAbsoluteSensorRange(
            AbsoluteSensorRange.Signed_PlusMinus180
        )
        self.encoder.configSensorDirection(False)
        self.encoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        # adjust 0 degree point with offset
        self.encoder.configMagnetOffset(self.steerOffset)

        self.steer.configAllSettings(cfgSteerMotor)
        # define the remote CANCoder as Remote Feedback 0
        self.steer.configRemoteFeedbackFilter(
            self.encoder.getDeviceNumber(), RemoteSensorSource.CANCoder, 0
        )
        self.steer.setInverted(TalonFXInvertType.Clockwise)
        # TODO: the following is now handled in the base config
        #   with the ..primaryPID property.
        #   - confirm this is the proper method
        # configure Falcon to use Remote Feedback 0
        # self.steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
        self.steer.configIntegratedSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        self.steer.configIntegratedSensorAbsoluteRange(
            AbsoluteSensorRange.Signed_PlusMinus180
        )
        self.steer.setSelectedSensorPosition(0)
        self.steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
        self.steer.setSensorPhase(True)
        # configure drive motor
        self.drive.configAllSettings(cfgDriveMotor)
        self.drive.setInverted(TalonFXInvertType.Clockwise)

    def setState(self, state):
        # adjusts the speed and angle for minimal change
        # requires getting the current angle from the steer
        # motor and creating a Rotation2d object from it.
        self.state = state.optimize(
           state, Rotation2d.fromDegrees(self.ticksToDegrees(self.steer.getSelectedSensorPosition()))
        )


    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:
            # TODO: confirm correct units for position mode,
            # and determine if the value needs to be inverted or not
            # since the angle coming from the SwerveModuleState is
            # positive to the left (counter-clockwise)
            self.steer.set(POSITION_MODE, self.getCommandedTicks())
            self.getSteerPosition()
            # set velocity for Falcon to ticks/100ms
            self.drive.set(
                VELOCITY_MODE, self.drive_unit.speedToVelocity(self.state.speed)
            )
        else:
            # TODO: decide whether we should
            # zero velocity on disable.
            self.steer.set(0)
            self.drive.set(0)


class SwerveChassis:
    # TODO: Add gyro to chassis, needed for field-oriented movement
    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveBackLeft: SwerveModule
    swerveBackRight: SwerveModule
    gyro: FROGGyro
    field: Field2d

    def __init__(self):
        self.enabled = False
        self.speeds = ChassisSpeeds(0, 0, 0)
        self.center = Translation2d(0, 0)

    def disable(self):
        self.enabled = False
        for module in [
            self.swerveFrontLeft,
            self.swerveFrontRight,
            self.swerveBackLeft,
            self.swerveBackRight,
        ]:
            module.disable()

    def drive(self, vX, vY, vT):
        # takes values from the joystick and translates it
        # into chassis movement
        self.speeds = ChassisSpeeds(
            vX * kMaxMetersPerSec, vY * kMaxMetersPerSec, vT * kMaxRadiansPerSec
        )

    def field_oriented_drive(self, vX, vY, vT):
        # takes values from the joystick and translates it
        # into chassis movement
        self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vX * kMaxMetersPerSec,
            vY * kMaxMetersPerSec,
            vT * kMaxRadiansPerSec,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
        )

    def enable(self):
        self.enabled = True
        for module in self.modules:
            module.enable()

    @feedback()
    def getChassisX_FPS(self):
        return self.speeds.vx_fps

    @feedback()
    def getChassisY_FPS(self):
        return self.speeds.vy_fps

    @feedback()
    def getChassisT_DPS(self):
        return self.speeds.omega_dps

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.

        # creating a list of our SwerveModules so they can
        self.modules = (
            self.swerveFrontLeft,
            self.swerveFrontRight,
            self.swerveBackLeft,
            self.swerveBackRight,
        )

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

        self.odometry = SwerveDrive4Odometry(
            self.kinematics, Rotation2d.fromDegrees(self.gyro.getAngle())
        )
        self.gyro.resetGyro()

    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:
            # pass in the commanded speeds and center of rotation
            # and get back the speed and angle values for each module
            # TODO: Change to field oriented
            states = self.kinematics.toSwerveModuleStates(
                self.speeds, self.center
            )
            # normalizing wheel speeds so they don't exceed the
            # maximum defined speed
            states = self.kinematics.desaturateWheelSpeeds(
                states, kMaxMetersPerSec
            )
            # pairing the module with the state it should get and
            # sending the state to the associated module

            # updating odometry to keep track of position and angle
            self.odometry.update(Rotation2d(self.gyro.getAngle()), *states)
            self.field.setRobotPose(self.odometry.getPose())

            for module, state in zip(self.modules, states):
                module.setState(state)

        else:
            pass
