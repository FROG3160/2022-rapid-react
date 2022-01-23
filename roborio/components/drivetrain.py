from ctre import WPI_TalonFX, CANCoder, TalonFXInvertType, ControlMode
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics


# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.MotionMagic
trackwidth = 24
wheelbase = 36


class SwerveModule:
    drive: WPI_TalonFX
    steer: WPI_TalonFX
    encoder: CANCoder

    def __init__(self):
        # set initial states for the component
        self.velocity = 0
        # TODO set angle to sensed angle from CANCoder?
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

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.
        # configure steer motor
        self.steer.setInverted(TalonFXInvertType.Clockwise)
        # configure drive motor
        self.steer.setInverted(TalonFXInvertType.Clockwise)
        # configure CANcoder

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

        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(trackwidth / 2, wheelbase / 2),
            Translation2d(trackwidth / 2, -wheelbase / 2),
            Translation2d(-trackwidth / 2, wheelbase / 2),
            Translation2d(-trackwidth / 2, -wheelbase / 2),
        )
        self.enabled = False

    def disable(self):
        self.enabled = False
        for module in [
            self.swerveFrontLeft,
            self.swerveFrontRight,
            self.swerveRearLeft,
            self.swerveRearRight,
        ]:
            module.disable()

    def enable(self):
        self.enabled = True
        for module in [
            self.swerveFrontLeft,
            self.swerveFrontRight,
            self.swerveRearLeft,
            self.swerveRearRight,
        ]:
            module.enable()

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.
        pass

    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:
            pass
        else:
            pass
