#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.units import units
from wpilib.simulation import SimDeviceSim
from pyfrc.physics.units import units
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from components.drivetrain import kVelocityMultiplier

# import typing

# if typing.TYPE_CHECKING:
from robot import FROGbot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "FROGbot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot objet
        """

        self.physics_controller = physics_controller
        self.robot = robot



        """
        # Change these parameters to fit your robot!

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(1)
        self.r_motor = wpilib.simulation.PWMSim(2)

        bumper_width = 3.25 * units.inch

        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio \omega In/\omega Out
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
        """
        # # Drive motors
        # self.simFL_drive = self.simBot.swerveFrontLeft_drive.getSimCollection()
        # self.simFR_drive = self.simBot.swerveFrontRight_drive.getSimCollection()
        # self.simBL_drive = self.simBot.swerveRearLeft_drive.getSimCollection()
        # self.simBR_drive = self.simBot.swerveRearRight_drive.getSimCollection()
        

        # # Steer motors
        # self.simFL_steer = self.simBot.swerveFrontLeft_steer.getSimCollection()
        # self.simFR_steer = self.simBot.swerveFrontRight_steer.getSimCollection()
        # self.simBL_steer = self.simBot.swerveRearLeft_steer.getSimCollection()
        # self.simBR_steer = self.simBot.swerveRearRight_steer.getSimCollection()

        # self.simFL_encoder = (
        #     self.simBot.swerveFrontLeft_encoder.getSimCollection()
        # )
        # self.simFR_encoder = (
        #     self.simBot.swerveFrontRight_encoder.getSimCollection()
        # )
        # self.simBL_encoder = (
        #     self.simBot.swerveRearLeft_encoder.getSimCollection()
        # )
        # self.simBR_encoder = (
        #     self.simBot.swerveRearRight_encoder.getSimCollection()
        # )

        # # initializing the CANCoder. All wheels are forward to start
        # self.simFL_encoder.setRawPosition(0)
        # self.simFR_encoder.setRawPosition(0)
        # self.simBL_encoder.setRawPosition(0)
        # self.simBR_encoder.setRawPosition(0)

    def update_sim(self, now, tm_diff):
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        """
        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)
        """

        
        '''
        (function) four_motor_swerve_drivetrain: (lr_motor: float, rr_motor: float, lf_motor: float, rf_motor: float, lr_angle: float, rr_angle: float, lf_angle: float, rf_angle: float, x_wheelbase=2, y_wheelbase=2, speed=5, deadzone=None) -> ChassisSpeeds
        Four motors that can be rotated in any direction

        If any motors are inverted, then you will need to multiply that motor's value by -1.

        :param lr_motor: Left rear motor value (-1 to 1); 1 is forward
        :param rr_motor: Right rear motor value (-1 to 1); 1 is forward
        :param lf_motor: Left front motor value (-1 to 1); 1 is forward
        :param rf_motor: Right front motor value (-1 to 1); 1 is forward

        :param lr_angle: Left rear motor angle in degrees (0 to 360 measured clockwise from forward position)
        :param rr_angle: Right rear motor angle in degrees (0 to 360 measured clockwise from forward position)
        :param lf_angle: Left front motor angle in degrees (0 to 360 measured clockwise from forward position)
        :param rf_angle: Right front motor angle in degrees (0 to 360 measured clockwise from forward position)

        :param x_wheelbase: The distance in feet between right and left wheels.
        :param y_wheelbase: The distance in feet between forward and rear wheels.
        :param speed: Speed of robot in feet per second (see above)
        :param deadzone: A function that adjusts the output of the motor (see linear_deadzone)

        :returns: ChassisSpeeds that can be passed to 'drive'
        '''
        # TODO: calculate all speeds and angles and pass them to four_motor_swerve_drivetrain()
        # chassis_speeds = four_motor_swerve_drivetrain()
        # self.physics_controller.drive(*chassis_speeds)

        # this works to update a motor in the sim GUI, but only changing the percentOutput and motorOutputLeadVoltage attributes.
        #self.simFL_drive.setIntegratedSensorVelocity(round(self.simBot.swerveFrontLeft_drive.getSelectedSensorVelocity()))
        self.physics_controller.drive(self.robot.swerveChassis.speeds, tm_diff)
        pass
