import wpilib
from navx import AHRS
from magicbot import feedback, tunable
from ctre import CANifier
from components.common import Buffer
from rev import ColorSensorV3
import math
from wpimath.geometry import Rotation2d

BUFFERLEN = 50

SENSORUNITS_IN_INCHES = 0.0394
SENSORUNITS_IN_FEET = 0.00328
SENSORUNITS_IN_METERS = 0.001


class FROGGyro:

    starting_angle = tunable(0.0)

    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.offset = 0
        # self.field_heading = 360-242
        # self.gyro.reset()
        # self.gyro.setAngleAdjustment(-self.field_heading)

    @feedback()
    def getYaw(self):
        # returns gyro heading +180 to -180 degrees
        return -self.gyro.getYaw()

    def setOffset(self, offset):
        self.offset = offset

    @feedback()
    def getOffsetYaw(self):
        chassisYaw = self.gyro.getYaw()
        fieldYaw = Rotation2d(chassisYaw + self.offset)
        return math.degrees(math.atan2(fieldYaw.sin(), fieldYaw.cos()))

    def resetGyro(self):
        # sets yaw reading to 0
        self.gyro.reset()

    def execute(self):
        pass

    def setAngle(self, angle):
        self.gyro.setAngleAdjustment(angle)

    @feedback()
    def getRadiansCCW(self):
        return math.radians(self.gyro.getYaw())

    @feedback()
    def getCompassHeading(self):
        return self.gyro.getCompassHeading()

    @feedback()
    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()

    @feedback()
    def getPitch(self):
        return self.gyro.getPitch()

    @feedback()
    def getRoll(self):
        return self.gyro.getRoll()

    @feedback()
    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()
        
class FROGdar:
    canifier: CANifier

    def __init__(self):
        self.enabled = False
        self.targetRange = None
        self.rangeBuffer = Buffer(BUFFERLEN)

    def disable(self):
        self.enabled = False

    def enable(self):
        # clear range data to get fresh information
        self.rangeBuffer.clear()
        self.enabled = True

    def isValidData(self):
        return self.rangeBuffer._isValidData() and self.targetRange is not None

    @feedback(key="sensor_raw")
    def getSensorData(self):
        errorcode, (val1, val2) = self.canifier.getPWMInput(
            CANifier.PWMChannel.PWMChannel0
        )
        return val1

    @feedback(key="sensor_buffered")
    def getBufferedSensorData(self):
        return self.targetRange

    @feedback(key="range_inches")
    def getDistance(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_INCHES

    @feedback(key="range_feet")
    def getDistanceFeet(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_FEET

    @feedback(key="range_meters")
    def getDistanceMeters(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_METERS

    def execute(self):
        if self.enabled:
            # stream data into our counter
            self.rangeBuffer.append(self.getSensorData())
            if self.rangeBuffer._getBufferLength() > 0:
                self.targetRange = self.rangeBuffer.average()
            else:
                self.targetRange = None
        else:
            self.rangeBuffer.clear()
            self.targetRange = None


class FROGColor:
    def __init__(self):
        self.enabled = False
        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    @feedback(key="Red")
    def getRed(self):
        return self.colorSensor.getColor().red

    @feedback(key="Blue")
    def getBlue(self):
        return self.colorSensor.getColor().blue

    @feedback(key="Proximity")
    def getProximity(self):
        return self.colorSensor.getProximity()

    def execute(self):
        pass


class FROGsonic:
    cargoUltrasonic: wpilib.AnalogInput
    mm: float
    mv: float

    def __init__(self):
        pass

    def execute(self):
        pass

    def __call__(self):
        self.getInches()

    @feedback()
    def getInches(self):
        self.USVolt = self.cargoUltrasonic.getVoltage()
        return (self.USVolt * self.mm / (self.mv / 1000)) * 0.039
