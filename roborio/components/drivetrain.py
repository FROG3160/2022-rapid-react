from ctre import WPI_TalonFX, CANCoder, TalonFXInvertType, ControlMode
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics


# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.MotionMagic
trackwidth = 24
wheelbase = 36


class DriveMotor:
    def __init__(self, motorID):
        self.drivemotor = WPI_TalonFX(motorID)
        self.enabled = False
        self.velocity = 0

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.
        self.drivemotor.setInverted(TalonFXInvertType.Clockwise)

    def setVelocity(self, value):
        self.velocity = value

    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:
            # drive at the speed
            self.drivemotor.set(VELOCITY_MODE, self.velocity)
        else:
            # stop
            self.drivemotor.set(0)


class SteerMotor:
    def __init__(self, motorID):
        self.steermotor = WPI_TalonFX(motorID)
        self.enabled = False
        self.position = 0

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def setup(self):
        # magicbot calls setup() when creating components
        # configure motors and other objects here.
        self.steermotor.setInverted(TalonFXInvertType.Clockwise)

    def setPosition(self, angle):
        self.position = angle

    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:
            # drive at the speed
            self.steermotor.set(POSITION_MODE, self.position)
        else:
            # stop - don't send voltage
            self.steermotor.set(0)


class SwerveModule:
    # TODO: add Cancoder object for steer

    def __init__(self, driveID, steerID):
        # set initial states for the component
        self.steer = SteerMotor(steerID)
        self.drive = DriveMotor(driveID)
        self.velocity = 0
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
        pass

    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        if self.enabled:

            self.steer.setPosition(self.angle)
            self.drive.setVelocity(self.velocity)
        else:
            # TODO: decide whether we should
            # zero velocity on disable.
            pass


class SwerveChassis:
    def __init__(self, lfModule, rfModule, lrModule, rrModule):
        self.lfSwerve = SwerveModule(**lfModule)
        self.rfSwerve = SwerveModule(**rfModule)
        self.lrSwerve = SwerveModule(**lrModule)
        self.rrSwerve = SwerveModule(**rrModule)
        # TODO: double check the signs on the Translation2d
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(trackwidth / 2, wheelbase / 2),
            Translation2d(trackwidth / 2, -wheelbase / 2),
            Translation2d(-trackwidth / 2, wheelbase / 2),
            Translation2d(-trackwidth / 2, -wheelbase / 2),
        )
        self.enabled = False

    def disable(self):
        self.enabled = False
        for module in [self.lfSwerve, self.rfSwerve, self.lrSwerve, self.rrSwerve]:
            module.disable()

    def enable(self):
        self.enabled = True
        for module in [self.lfSwerve, self.rfSwerve, self.lrSwerve, self.rrSwerve]:
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
