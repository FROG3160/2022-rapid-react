from ctre import (
    WPI_TalonFX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from magicbot import feedback
from components.common import TalonPID
from components.sensors import FROGdar


# TODO Find out the Min/Max of the velocity and the tolerence for the Flywheel
FLYWHEEL_MODE = ControlMode.Velocity
FLYWHEEL_PID = TalonPID(0, p=0.4, f=0.0515)
FLYWHEEL_VELOCITY = 7300
FLYWHEEL_MAX_VEL = 25000
FLYWHEEL_MAX_ACCEL = FLYWHEEL_MAX_VEL / 50
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 500
FLYWHEEL_VEL_TOLERANCE = 300
FLYWHEEL_LOOP_RAMP = 0.25


class Flywheel:
    flywheel_motor: WPI_TalonFX

    def __init__(self):
        self.enabled = False
        self._controlMode = FLYWHEEL_MODE
        self._velocity = FLYWHEEL_VELOCITY

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback(key="isReady")
    def isReady(self):
        return (
            abs(self.getVelocity() - self.getCommandedVelocity())
            < FLYWHEEL_VEL_TOLERANCE
        )

    # read current encoder velocity
    @feedback(key="velocity")
    def getVelocity(self):
        #sensor values are reversed.  we command a positive value and the
        #sensor shows a negative one, so we negate the output
        return -self.flywheel_motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor)

    @feedback(key="commanded")
    def getCommandedVelocity(self):
        return self._velocity

    def setup(self):
        # Falcon500 motors use the integrated sensor
        self.flywheel_motor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )
        self.flywheel_motor.setSensorPhase(False)
        # = setInverted(True)
        self.flywheel_motor.setInverted(
            TalonFXInvertType.CounterClockwise)
        self.flywheel_motor.setNeutralMode(
            NeutralMode.Coast)
        FLYWHEEL_PID.configTalon(
            self.flywheel_motor)
        # use closed loop ramp to accelerate smoothly
        self.flywheel_motor.configClosedloopRamp(
            FLYWHEEL_LOOP_RAMP)

    def setVelocity(self, velocity):
        self._controlMode = ControlMode.Velocity
        self._velocity = velocity
        
    def execute(self):
        if self.enabled:
            self.flywheel_motor.set(self._controlMode, self._velocity)
        else:
            self.flywheel_motor.set(0)


class FROGShooter:
    lidar: FROGdar

    def __init__(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def set_automatic(self):
        pass

    def set_manual(self):
        pass

    def disable(self):
        pass

    def getdistance(self):
        # get the value/distance from the lidar in inches
        return self.lidar.getDistance()

    def incrementSpeed(self):
        self._velocity += FLYWHEEL_INCREMENT
        self.setVelocity(self._velocity)

    def decrementSpeed(self):
        self._velocity -= FLYWHEEL_INCREMENT
        self.setVelocity(self._velocity)

    def execute(self):
        pass


