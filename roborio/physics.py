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
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
from wpilib.simulation import SimDeviceSim

# import typing

# if typing.TYPE_CHECKING:
from robot import FROGbot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot objet
        """

        self.physics_controller = physics_controller
        self.simBot = FROGbot()
        self.simBot.createObjects()

        print("TODO: modify simulation for my robot")

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
        # Object creation sing wpilib.simulation.SimDeviceSim
        # This was abandoned in favor of using CTRE's built in getSimCollection method
        # below.
        # values for Talon FX devices: (using .e.g. SimDeviceSim('CANMotor:Talon FX[11]').enumerateValues())
        # [('percentOutput', True), ('motorOutputLeadVoltage', True), ('supplyCurrent', False), ('motorCurrent', False), ('busVoltage', False)]

        # self.frontLeft_drive = SimDeviceSim('CANMotor:Talon FX[11]/Integrated Sensor').getDouble('velocity')
        # self.frontRight_drive = SimDeviceSim('CANMotor:Talon FX[12]/Integrated Sensor').getDouble('velocity')
        # self.backLeft_drive = SimDeviceSim('CANMotor:Talon FX[13]/Integrated Sensor').getDouble('velocity')
        # self.backRight_drive = SimDeviceSim('CANMotor:Talon FX[14]/Integrated Sensor').getDouble('velocity')

        # self.frontLeft_steer = SimDeviceSim('CANMotor:Talon FX[21]').getDouble('percentOutput')
        # self.frontRight_steer = SimDeviceSim('CANMotor:Talon FX[22]').getDouble('percentOutput')
        # self.backLeft_steer = SimDeviceSim('CANMotor:Talon FX[23]').getDouble('percentOutput')
        # self.backRight_steer = SimDeviceSim('CANMotor:Talon FX[24]').getDouble('percentOutput')

        # self.frontLeft_encoder = SimDeviceSim('CANMotor:Talon FX[31]').getDouble('absolutePosition')
        # self.frontRight_encoder = SimDeviceSim('CANMotor:Talon FX[32]').getDouble('absolutePosition')
        # self.backLeft_encoder = SimDeviceSim('CANMotor:Talon FX[33]').getDouble('absolutePosition')
        # self.backRight_encoder = SimDeviceSim('CANMotor:Talon FX[34]').getDouble('absolutePosition')

        # it seems that using getSimCollection allows us to use the CTRE objects in a similar
        # way to the main code, so we are trying this approach over using the SimDevicesSim
        # objects from wpilib.simulation
        # Drive motors
        self.simFL_drive = self.simBot.swerveFrontLeft_drive.getSimCollection()
        self.simFR_drive = self.simBot.swerveFrontRight_drive.getSimCollection()
        self.simBL_drive = self.simBot.swerveRearLeft_drive.getSimCollection()
        self.simBR_drive = self.simBot.swerveRearRight_drive.getSimCollection()

        # Steer motors
        self.simFL_steer = self.simBot.swerveFrontLeft_steer.getSimCollection()
        self.simFR_steer = self.simBot.swerveFrontRight_steer.getSimCollection()
        self.simBL_steer = self.simBot.swerveRearLeft_steer.getSimCollection()
        self.simBR_steer = self.simBot.swerveRearRight_steer.getSimCollection()

        self.simFL_encoder = (
            self.simBot.swerveFrontLeft_encoder.getSimCollection()
        )
        self.simFR_encoder = (
            self.simBot.swerveFrontRight_encoder.getSimCollection()
        )
        self.simBL_encoder = (
            self.simBot.swerveRearLeft_encoder.getSimCollection()
        )
        self.simBR_encoder = (
            self.simBot.swerveRearRight_encoder.getSimCollection()
        )

        # initializing the CANCoder. All wheels are forward to start
        self.simFL_encoder.setRawPosition(0)
        self.simFR_encoder.setRawPosition(0)
        self.simBL_encoder.setRawPosition(0)
        self.simBR_encoder.setRawPosition(0)

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
